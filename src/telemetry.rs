//! Latest-only volatile telemetry transport, isolated from physical ticks.
use crate::core_client::CoreClient;
use serde_json::Value;
use std::sync::{
    Arc,
    atomic::{AtomicBool, AtomicU64, Ordering},
};
use std::time::{Duration, Instant};
use tokio::sync::watch;

pub struct TelemetryTransport {
    sender: watch::Sender<Option<Value>>,
    ready: Arc<AtomicBool>,
    last_ack: Arc<AtomicU64>,
    started: Instant,
    worker: tokio::task::JoinHandle<()>,
}

impl TelemetryTransport {
    pub fn start(client: CoreClient) -> Self {
        let (sender, mut receiver) = watch::channel::<Option<Value>>(None);
        let ready = Arc::new(AtomicBool::new(false));
        let last_ack = Arc::new(AtomicU64::new(0));
        let started = Instant::now();
        let worker_ready = ready.clone();
        let worker_ack = last_ack.clone();
        let worker = tokio::spawn(async move {
            loop {
                worker_ready.store(false, Ordering::Release);
                let result = client.connect_telemetry().await;
                if let Ok(mut socket) = result {
                    let mut sent: Option<Value> = None;
                    let mut confirmed = Instant::now();
                    let mut confirmed_sequence: Option<u64> = None;
                    loop {
                        if receiver.has_changed().is_err() {
                            return;
                        }
                        let latest = receiver.borrow_and_update().clone();
                        if let Some(frame) = latest.as_ref()
                            && sent.as_ref() != Some(frame)
                        {
                            if socket.send_json(frame).await.is_err() {
                                break;
                            }
                            sent = Some(frame.clone());
                        }
                        let message = tokio::select! {
                            biased;
                            _ = tokio::time::sleep_until(tokio::time::Instant::from_std(
                                confirmed + Duration::from_secs(3),
                            )) => break,
                            changed = receiver.changed() => {
                                if changed.is_err() { return; }
                                continue;
                            }
                            message = socket.read_frame() => message,
                        };
                        // Never cancel Ping/Pong writes when a publication arrives.
                        let message = match message {
                            Ok(frame) => socket.handle_frame(frame).await,
                            Err(error) => Err(error),
                        };
                        match message {
                            Ok(Some(ack)) => {
                                let Some(frame) = sent.as_ref() else {
                                    break;
                                };
                                if ack["telemetryVersion"] != "1.0.0"
                                    || ack["messageType"] != "telemetry.ack"
                                    || ack["sessionEpoch"] != frame["sessionEpoch"]
                                    || ack["simulatorBootId"] != frame["simulatorBootId"]
                                    || ack["simulatorId"] != frame["simulatorId"]
                                    || ack["robotId"] != frame["robotId"]
                                    || ack["durability"] != "PROJECTION"
                                    || !matches!(
                                        ack["disposition"].as_str(),
                                        Some("ACCEPTED" | "STALE")
                                    )
                                    || ack["telemetrySequence"].as_u64().is_none_or(|seq| {
                                        seq > frame["telemetrySequence"].as_u64().unwrap_or(0)
                                            || confirmed_sequence.is_some_and(|prior| seq <= prior)
                                    })
                                {
                                    break;
                                }
                                confirmed_sequence = ack["telemetrySequence"].as_u64();
                                confirmed = Instant::now();
                                worker_ack
                                    .store(started.elapsed().as_millis() as u64, Ordering::Release);
                                worker_ready.store(true, Ordering::Release);
                            }
                            Ok(None) => {}
                            Err(_) => break,
                        }
                        if confirmed.elapsed() >= Duration::from_secs(3) {
                            break;
                        }
                    }
                }
                worker_ready.store(false, Ordering::Release);
                tokio::time::sleep(Duration::from_secs(1)).await;
            }
        });
        Self {
            sender,
            ready,
            last_ack,
            started,
            worker,
        }
    }

    pub fn publish(&self, value: Value) {
        self.sender.send_replace(Some(value));
    }

    pub fn ready(&self) -> bool {
        self.ready.load(Ordering::Acquire)
            && (self.started.elapsed().as_millis() as u64)
                .saturating_sub(self.last_ack.load(Ordering::Acquire))
                < 3_000
    }
}

impl Drop for TelemetryTransport {
    fn drop(&mut self) {
        self.worker.abort();
    }
}

/// Only business/safety transitions require durable evidence; pose and progress do not.
pub fn durable_fingerprint(payload: &Value) -> Value {
    let battery = payload["stationState"]["batteryPercent"]
        .as_f64()
        .unwrap_or(100.0);
    serde_json::json!({
        "orderId": payload["orderId"], "orderUpdateId": payload["orderUpdateId"],
        "operationalState": payload["operationalState"], "safety": payload["safety"],
        "phase": payload["stationState"]["phase"], "action": payload["stationState"]["action"],
        "loaded": payload["stationState"]["loaded"],
        "batteryBand": if battery <= 0.0 { 0 } else if battery <= 20.0 { 1 } else if battery >= 100.0 { 3 } else { 2 },
    })
}
