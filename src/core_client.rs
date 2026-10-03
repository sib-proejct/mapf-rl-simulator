//! Authenticated Core REST/WebSocket transport.

use crate::controller::RuntimeProfile;
use crate::protocol::{
    MAX_WS_MESSAGE_BYTES, ReportBatchOutcome, ReportBatchRequest, ReportEnvelope, SimulatorSnapshot,
};
use futures_util::{SinkExt, StreamExt};
use http::HeaderValue;
use reqwest::StatusCode;
use std::fmt;
use tokio::net::TcpStream;
use tokio_tungstenite::tungstenite::client::IntoClientRequest;
use tokio_tungstenite::tungstenite::http::header::SEC_WEBSOCKET_PROTOCOL;
use tokio_tungstenite::tungstenite::protocol::Message;
use tokio_tungstenite::{MaybeTlsStream, WebSocketStream, connect_async};
use url::Url;
use uuid::Uuid;

const API_KEY_HEADER: &str = "X-API-Key";
const RESUME_AFTER_HEADER: &str = "X-Resume-After";
const WS_SUBPROTOCOL: &str = "mapf.v1";

#[derive(Clone)]
pub struct ApiKey(HeaderValue);

impl ApiKey {
    pub fn new(value: &str) -> Result<Self, CoreClientError> {
        if value.is_empty() {
            return Err(CoreClientError::InvalidApiKey);
        }
        let mut header =
            HeaderValue::from_str(value).map_err(|_| CoreClientError::InvalidApiKey)?;
        header.set_sensitive(true);
        Ok(Self(header))
    }
}

impl fmt::Debug for ApiKey {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str("ApiKey([REDACTED])")
    }
}

#[derive(Clone, Debug)]
pub struct CoreClientConfig {
    pub profile: RuntimeProfile,
    pub simulator_id: String,
    pub rest_base_url: Url,
    pub websocket_url: Url,
    pub api_key: ApiKey,
}

impl CoreClientConfig {
    pub fn new(
        profile: RuntimeProfile,
        simulator_id: String,
        rest_base_url: Url,
        websocket_url: Url,
        api_key: ApiKey,
    ) -> Result<Self, CoreClientError> {
        Self::new_with_local_compose(
            profile,
            simulator_id,
            rest_base_url,
            websocket_url,
            api_key,
            false,
        )
    }

    /// Explicit local-only Docker service transport; other profiles still require TLS.
    pub fn new_with_local_compose(
        profile: RuntimeProfile,
        simulator_id: String,
        rest_base_url: Url,
        websocket_url: Url,
        api_key: ApiKey,
        local_compose: bool,
    ) -> Result<Self, CoreClientError> {
        if simulator_id.is_empty()
            || simulator_id.len() > 128
            || simulator_id.chars().any(char::is_control)
        {
            return Err(CoreClientError::InvalidSimulatorId);
        }
        validate_transport(profile, &rest_base_url, &websocket_url, local_compose)?;
        Ok(Self {
            profile,
            simulator_id,
            rest_base_url,
            websocket_url,
            api_key,
        })
    }
}

#[derive(Clone, Debug)]
pub struct CoreClient {
    config: CoreClientConfig,
    http: reqwest::Client,
}

impl CoreClient {
    pub fn new(config: CoreClientConfig) -> Result<Self, CoreClientError> {
        let http = reqwest::Client::builder()
            .timeout(std::time::Duration::from_secs(3))
            .redirect(reqwest::redirect::Policy::none())
            .build()?;
        Ok(Self { config, http })
    }

    pub async fn fetch_snapshot(&self) -> Result<SimulatorSnapshot, CoreClientError> {
        let path = format!("api/v1/simulators/{}/snapshot", self.config.simulator_id);
        let url = self.config.rest_base_url.join(&path)?;
        let response = self
            .http
            .get(url)
            .header(API_KEY_HEADER, self.config.api_key.0.clone())
            .send()
            .await?;
        require_success(response.status())?;
        let snapshot = response.json::<SimulatorSnapshot>().await?;
        snapshot.validate()?;
        Ok(snapshot)
    }

    pub async fn submit_report_batch(
        &self,
        reports: &[ReportEnvelope],
    ) -> Result<ReportBatchOutcome, CoreClientError> {
        if reports.is_empty() || reports.len() > 100 {
            return Err(CoreClientError::InvalidBatchSize);
        }
        for report in reports {
            report.validate()?;
        }
        let path = format!(
            "api/v1/simulators/{}/report-batches",
            self.config.simulator_id
        );
        let url = self.config.rest_base_url.join(&path)?;
        let request_id = Uuid::new_v4();
        let response = self
            .http
            .post(url)
            .header(API_KEY_HEADER, self.config.api_key.0.clone())
            .json(&ReportBatchRequest {
                request_id,
                reports,
            })
            .send()
            .await?;
        require_success(response.status())?;
        let outcome = response.json::<ReportBatchOutcome>().await?;
        if outcome.request_id != request_id || outcome.outcomes.len() != reports.len() {
            return Err(CoreClientError::InvalidResponse);
        }
        for (item, report) in outcome.outcomes.iter().zip(reports) {
            if item.report_message_id != report.message_id
                || item.report_sequence != report.report_sequence
            {
                return Err(CoreClientError::InvalidResponse);
            }
        }
        Ok(outcome)
    }

    pub async fn connect_websocket(
        &self,
        resume_after: Option<u64>,
    ) -> Result<CoreWebSocket, CoreClientError> {
        let mut request = self.config.websocket_url.as_str().into_client_request()?;
        request
            .headers_mut()
            .insert(API_KEY_HEADER, self.config.api_key.0.clone());
        request.headers_mut().insert(
            SEC_WEBSOCKET_PROTOCOL,
            HeaderValue::from_static(WS_SUBPROTOCOL),
        );
        if let Some(sequence) = resume_after {
            request.headers_mut().insert(
                RESUME_AFTER_HEADER,
                HeaderValue::from_str(&sequence.to_string())
                    .map_err(|_| CoreClientError::InvalidResponse)?,
            );
        }
        let (stream, response) =
            tokio::time::timeout(std::time::Duration::from_secs(3), connect_async(request))
                .await
                .map_err(|_| CoreClientError::Timeout)??;
        let selected = response
            .headers()
            .get(SEC_WEBSOCKET_PROTOCOL)
            .and_then(|value| value.to_str().ok());
        if selected != Some(WS_SUBPROTOCOL) {
            return Err(CoreClientError::SubprotocolMismatch);
        }
        Ok(CoreWebSocket { stream })
    }
}

pub struct CoreWebSocket {
    stream: WebSocketStream<MaybeTlsStream<TcpStream>>,
}

impl CoreWebSocket {
    /// A successful return only means bytes reached the WebSocket. The caller must
    /// retain the report until the matching Core `report.ack` is processed.
    pub async fn write_report(&mut self, report: &ReportEnvelope) -> Result<(), CoreClientError> {
        report.validate()?;
        let encoded = serde_json::to_string(report)?;
        if encoded.len() > MAX_WS_MESSAGE_BYTES {
            return Err(CoreClientError::MessageTooLarge);
        }
        tokio::time::timeout(
            std::time::Duration::from_secs(3),
            self.stream.send(Message::Text(encoded.into())),
        )
        .await
        .map_err(|_| CoreClientError::Timeout)??;
        Ok(())
    }

    /// Only socket frame reads are cancellable. Ping/Pong writes must finish.
    pub async fn poll_json(&mut self) -> Result<Option<serde_json::Value>, CoreClientError> {
        let read =
            tokio::time::timeout(std::time::Duration::from_millis(1), self.stream.next()).await;
        let message = match read {
            Err(_) => return Ok(None),
            Ok(frame) => frame.ok_or(CoreClientError::Disconnected)??,
        };
        match message {
            Message::Text(text) => {
                if text.len() > MAX_WS_MESSAGE_BYTES {
                    return Err(CoreClientError::MessageTooLarge);
                }
                Ok(Some(serde_json::from_str(text.as_str())?))
            }
            Message::Binary(_) => Err(CoreClientError::UnexpectedBinaryMessage),
            Message::Ping(bytes) => {
                tokio::time::timeout(
                    std::time::Duration::from_secs(3),
                    self.stream.send(Message::Pong(bytes)),
                )
                .await
                .map_err(|_| CoreClientError::Timeout)??;
                Ok(None)
            }
            Message::Pong(_) | Message::Frame(_) => Ok(None),
            Message::Close(_) => Err(CoreClientError::Disconnected),
        }
    }

    pub async fn next_json(&mut self) -> Result<serde_json::Value, CoreClientError> {
        tokio::time::timeout(std::time::Duration::from_secs(3), self.next_json_inner())
            .await
            .map_err(|_| CoreClientError::Timeout)?
    }

    async fn next_json_inner(&mut self) -> Result<serde_json::Value, CoreClientError> {
        loop {
            let message = self
                .stream
                .next()
                .await
                .ok_or(CoreClientError::Disconnected)??;
            match message {
                Message::Text(text) => {
                    if text.len() > MAX_WS_MESSAGE_BYTES {
                        return Err(CoreClientError::MessageTooLarge);
                    }
                    return Ok(serde_json::from_str(text.as_str())?);
                }
                Message::Binary(bytes) => {
                    if bytes.len() > MAX_WS_MESSAGE_BYTES {
                        return Err(CoreClientError::MessageTooLarge);
                    }
                    return Err(CoreClientError::UnexpectedBinaryMessage);
                }
                Message::Ping(bytes) => self.stream.send(Message::Pong(bytes)).await?,
                Message::Pong(_) | Message::Frame(_) => {}
                Message::Close(_) => return Err(CoreClientError::Disconnected),
            }
        }
    }
}

#[derive(Debug)]
pub enum CoreClientError {
    Http(reqwest::Error),
    WebSocket(tokio_tungstenite::tungstenite::Error),
    Json(serde_json::Error),
    Url(url::ParseError),
    Protocol(crate::protocol::ProtocolError),
    HttpStatus(StatusCode),
    Disconnected,
    Timeout,
    InsecureTransport,
    InvalidApiKey,
    InvalidBatchSize,
    InvalidResponse,
    InvalidSimulatorId,
    MessageTooLarge,
    SubprotocolMismatch,
    UnexpectedBinaryMessage,
}

impl From<reqwest::Error> for CoreClientError {
    fn from(value: reqwest::Error) -> Self {
        Self::Http(value)
    }
}

impl From<tokio_tungstenite::tungstenite::Error> for CoreClientError {
    fn from(value: tokio_tungstenite::tungstenite::Error) -> Self {
        Self::WebSocket(value)
    }
}

impl From<serde_json::Error> for CoreClientError {
    fn from(value: serde_json::Error) -> Self {
        Self::Json(value)
    }
}

impl From<url::ParseError> for CoreClientError {
    fn from(value: url::ParseError) -> Self {
        Self::Url(value)
    }
}

impl From<crate::protocol::ProtocolError> for CoreClientError {
    fn from(value: crate::protocol::ProtocolError) -> Self {
        Self::Protocol(value)
    }
}

impl fmt::Display for CoreClientError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Http(error) => write!(formatter, "Core REST request failed: {error}"),
            Self::WebSocket(error) => write!(formatter, "Core WebSocket failed: {error}"),
            Self::Json(error) => write!(formatter, "Core JSON failed: {error}"),
            Self::Url(error) => write!(formatter, "Core URL failed: {error}"),
            Self::Protocol(error) => error.fmt(formatter),
            Self::HttpStatus(status) => write!(formatter, "Core returned HTTP {status}"),
            Self::Disconnected => formatter.write_str("Core WebSocket disconnected"),
            Self::Timeout => formatter.write_str("Core WebSocket operation timed out"),
            Self::InsecureTransport => {
                formatter.write_str("Core transport violates the selected profile")
            }
            Self::InvalidApiKey => formatter.write_str("Simulator API key is missing or invalid"),
            Self::InvalidBatchSize => {
                formatter.write_str("REST report batch must contain 1 to 100 reports")
            }
            Self::InvalidResponse => formatter.write_str("Core response identity is invalid"),
            Self::InvalidSimulatorId => formatter.write_str("simulatorId is invalid"),
            Self::MessageTooLarge => formatter.write_str("WebSocket message exceeds one MiB"),
            Self::SubprotocolMismatch => formatter.write_str("Core did not select mapf.v1"),
            Self::UnexpectedBinaryMessage => {
                formatter.write_str("Core sent an unsupported binary message")
            }
        }
    }
}

impl std::error::Error for CoreClientError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Http(error) => Some(error),
            Self::WebSocket(error) => Some(error),
            Self::Json(error) => Some(error),
            Self::Url(error) => Some(error),
            Self::Protocol(error) => Some(error),
            _ => None,
        }
    }
}

fn require_success(status: StatusCode) -> Result<(), CoreClientError> {
    if status.is_success() {
        Ok(())
    } else {
        Err(CoreClientError::HttpStatus(status))
    }
}

fn validate_transport(
    profile: RuntimeProfile,
    rest: &Url,
    websocket: &Url,
    local_compose: bool,
) -> Result<(), CoreClientError> {
    let secure = rest.scheme() == "https" && websocket.scheme() == "wss";
    if secure {
        return Ok(());
    }
    let compose_endpoint =
        |url: &Url| local_compose && url.host_str() == Some("core") && url.port() == Some(8000);
    if profile != RuntimeProfile::Local
        || rest.scheme() != "http"
        || websocket.scheme() != "ws"
        || !(is_loopback(rest) || compose_endpoint(rest))
        || !(is_loopback(websocket) || compose_endpoint(websocket))
    {
        return Err(CoreClientError::InsecureTransport);
    }
    Ok(())
}

fn is_loopback(url: &Url) -> bool {
    match url.host() {
        Some(url::Host::Domain("localhost")) => true,
        Some(url::Host::Ipv4(address)) => address.is_loopback(),
        Some(url::Host::Ipv6(address)) => address.is_loopback(),
        _ => false,
    }
}
