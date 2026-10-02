//! Deterministic single-load station actions. Persist before publishing their result.
use crate::contracts::generated::StationAction;
use serde::{Deserialize, Serialize};

pub use crate::contracts::generated::StationPhase;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct StationState {
    pub loaded: bool,
    pub battery_percent: f64,
    pub phase: StationPhase,
    pub elapsed_ms: u64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub action: Option<StationAction>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub order_id: Option<String>,
}
impl Default for StationState {
    fn default() -> Self {
        Self {
            loaded: false,
            battery_percent: 100.0,
            phase: StationPhase::Idle,
            elapsed_ms: 0,
            action: None,
            order_id: None,
        }
    }
}
#[derive(Clone, Copy, Debug)]
pub struct StationConfig {
    pub pick_duration_ms: u64,
    pub place_duration_ms: u64,
    pub charge_percent_per_second: f64,
}
impl Default for StationConfig {
    fn default() -> Self {
        Self {
            pick_duration_ms: 2000,
            place_duration_ms: 2000,
            charge_percent_per_second: 1.0,
        }
    }
}
impl StationState {
    pub fn valid(&self) -> bool {
        self.battery_percent.is_finite()
            && (0.0..=100.0).contains(&self.battery_percent)
            && self.action.is_some() == self.order_id.is_some()
            && (self.phase == StationPhase::Idle || self.action.is_some())
    }
    /// Called once per control tick. Replanning preserves action identity by Order ID.
    pub fn advance(
        &mut self,
        order_id: &str,
        action: StationAction,
        safe_at_goal: bool,
        tick_ms: u64,
        config: StationConfig,
    ) {
        if self.order_id.as_deref() != Some(order_id) {
            if !safe_at_goal {
                return;
            }
            self.order_id = Some(order_id.to_owned());
            self.action = Some(action);
            self.elapsed_ms = 0;
            self.phase = if matches!(action, StationAction::Pick) && self.loaded
                || matches!(action, StationAction::Place) && !self.loaded
            {
                StationPhase::Failed
            } else {
                StationPhase::Running
            };
        }
        if self.action != Some(action) {
            self.phase = StationPhase::Failed;
            return;
        }
        if matches!(self.phase, StationPhase::Completed | StationPhase::Failed) {
            return;
        }
        if !safe_at_goal {
            self.phase = StationPhase::Paused;
            return;
        }
        self.phase = StationPhase::Running;
        self.elapsed_ms = self.elapsed_ms.saturating_add(tick_ms);
        match action {
            StationAction::Pick if self.elapsed_ms >= config.pick_duration_ms => {
                self.loaded = true;
                self.phase = StationPhase::Completed;
            }
            StationAction::Place if self.elapsed_ms >= config.place_duration_ms => {
                self.loaded = false;
                self.phase = StationPhase::Completed;
            }
            StationAction::Charge => {
                self.battery_percent = (self.battery_percent
                    + config.charge_percent_per_second * tick_ms as f64 / 1000.0)
                    .min(100.0);
                if self.battery_percent >= 100.0 {
                    self.phase = StationPhase::Completed;
                }
            }
            _ => {}
        }
    }
    pub fn completed(&self, order_id: &str, action: StationAction) -> bool {
        self.order_id.as_deref() == Some(order_id)
            && self.action == Some(action)
            && self.phase == StationPhase::Completed
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn pick_place_wait_for_safe_arrival_and_execute_once() {
        let mut state = StationState::default();
        let config = StationConfig::default();
        state.advance("pick", StationAction::Pick, false, 2000, config);
        assert!(!state.loaded);
        state.advance("pick", StationAction::Pick, true, 1000, config);
        assert!(!state.loaded);
        state.advance("pick", StationAction::Pick, false, 5000, config);
        assert_eq!(state.elapsed_ms, 1000);
        state.advance("pick", StationAction::Pick, true, 1000, config);
        assert!(state.loaded);
        let encoded = serde_json::to_vec(&state).unwrap();
        let mut restored: StationState = serde_json::from_slice(&encoded).unwrap();
        restored.advance("pick", StationAction::Pick, true, 2000, config);
        assert_eq!(state, restored);
        restored.advance("place", StationAction::Place, true, 2000, config);
        assert!(!restored.loaded);
        assert!(restored.completed("place", StationAction::Place));
    }
    #[test]
    fn invalid_load_actions_fail_and_charge_pauses() {
        let mut state = StationState {
            battery_percent: 99.0,
            ..StationState::default()
        };
        let config = StationConfig::default();
        state.advance("empty", StationAction::Place, true, 100, config);
        assert_eq!(state.phase, StationPhase::Failed);
        state.advance("charge", StationAction::Charge, true, 500, config);
        assert_eq!(state.battery_percent, 99.5);
        state.advance("charge", StationAction::Charge, false, 5000, config);
        assert_eq!(state.battery_percent, 99.5);
        state.advance("charge", StationAction::Charge, true, 500, config);
        assert!(state.completed("charge", StationAction::Charge));
        assert_eq!(state.battery_percent, 100.0);
    }
}
