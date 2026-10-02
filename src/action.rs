use crate::contracts::generated::ActionCandidate;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AdaptedAction {
    candidate: ActionCandidate,
    invalid_input: bool,
}

impl AdaptedAction {
    pub const fn candidate(self) -> ActionCandidate {
        self.candidate
    }

    pub const fn invalid_input(self) -> bool {
        self.invalid_input
    }
}

pub struct ActionAdapter;

impl ActionAdapter {
    pub const fn from_index(index: i32) -> AdaptedAction {
        let candidate = match index {
            0 => ActionCandidate::Wait,
            1 => ActionCandidate::North,
            2 => ActionCandidate::East,
            3 => ActionCandidate::South,
            4 => ActionCandidate::West,
            _ => {
                return AdaptedAction {
                    candidate: ActionCandidate::Wait,
                    invalid_input: true,
                };
            }
        };
        AdaptedAction {
            candidate,
            invalid_input: false,
        }
    }

    pub fn from_logits(logits: &[f32]) -> AdaptedAction {
        if logits.len() != 5 || logits.iter().any(|value| !value.is_finite()) {
            return Self::from_index(-1);
        }
        let mut best_index = 0;
        let mut best_value = logits[0];
        for (index, value) in logits.iter().copied().enumerate().skip(1) {
            if value > best_value {
                best_index = index;
                best_value = value;
            }
        }
        Self::from_index(best_index as i32)
    }
}

pub(crate) const fn action_vector(action: ActionCandidate) -> (f64, f64) {
    match action {
        ActionCandidate::Wait => (0.0, 0.0),
        ActionCandidate::North => (0.0, 1.0),
        ActionCandidate::East => (1.0, 0.0),
        ActionCandidate::South => (0.0, -1.0),
        ActionCandidate::West => (-1.0, 0.0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn invalid_and_non_finite_actions_fail_to_wait() {
        assert_eq!(
            ActionAdapter::from_index(99).candidate(),
            ActionCandidate::Wait
        );
        assert!(ActionAdapter::from_index(99).invalid_input());
        assert!(ActionAdapter::from_logits(&[0.0, f32::NAN]).invalid_input());
    }

    #[test]
    fn exact_logit_tie_uses_lowest_index() {
        let action = ActionAdapter::from_logits(&[1.0; 5]);
        assert_eq!(action.candidate(), ActionCandidate::Wait);
        assert!(!action.invalid_input());
    }
}
