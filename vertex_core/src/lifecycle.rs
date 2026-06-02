//! The ROS 2 managed-node lifecycle state machine (design §4.4).
//!
//! `rclrs` does not (as of v0.7.0) ship a `LifecycleNode`, so we run this state
//! machine ourselves and — in the `ros` layer — expose it through a
//! `/vertex/transition` service plus a latched `/vertex/lifecycle/state` topic
//! (design §8.4 fallback #2). The state transitions and their guards are pure
//! logic and live here so they are unit-testable without ROS.

use thiserror::Error;

/// The four ROS 2 *primary* lifecycle states. Transition (intermediate) states
/// such as `Configuring` are represented by [`Transition::intermediate_name`]
/// for logging/telemetry rather than as distinct primary states.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LifecycleState {
    Unconfigured,
    Inactive,
    Active,
    Finalized,
}

impl LifecycleState {
    /// The canonical lowercase label used on the `/vertex/lifecycle/state` topic
    /// and by the `ros2 lifecycle` CLI vocabulary.
    pub fn label(self) -> &'static str {
        match self {
            LifecycleState::Unconfigured => "unconfigured",
            LifecycleState::Inactive => "inactive",
            LifecycleState::Active => "active",
            LifecycleState::Finalized => "finalized",
        }
    }
}

/// The operator-invokable lifecycle transitions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Transition {
    Configure,
    Activate,
    Deactivate,
    Cleanup,
    Shutdown,
}

impl Transition {
    /// Parse the transition label accepted on the `/vertex/transition` service
    /// (the same verbs as the `ros2 lifecycle set` CLI).
    pub fn parse(s: &str) -> Option<Transition> {
        match s.trim().to_ascii_lowercase().as_str() {
            "configure" => Some(Transition::Configure),
            "activate" => Some(Transition::Activate),
            "deactivate" => Some(Transition::Deactivate),
            "cleanup" => Some(Transition::Cleanup),
            "shutdown" => Some(Transition::Shutdown),
            _ => None,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Transition::Configure => "configure",
            Transition::Activate => "activate",
            Transition::Deactivate => "deactivate",
            Transition::Cleanup => "cleanup",
            Transition::Shutdown => "shutdown",
        }
    }

    /// The ROS transition-state name reported while the callback runs.
    pub fn intermediate_name(self) -> &'static str {
        match self {
            Transition::Configure => "configuring",
            Transition::Activate => "activating",
            Transition::Deactivate => "deactivating",
            Transition::Cleanup => "cleaningup",
            Transition::Shutdown => "shuttingdown",
        }
    }
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum LifecycleError {
    #[error("transition {transition:?} is not valid from state {from:?}")]
    InvalidTransition {
        from: LifecycleState,
        transition: Transition,
    },
    #[error("node is finalized and cannot transition further")]
    Finalized,
}

impl LifecycleState {
    /// Validate a transition and return the resulting primary state, without
    /// running any side effects. The [`Controller`](crate::Controller) calls
    /// this first, runs the matching callback, and only commits the new state
    /// if the callback succeeds.
    pub fn target(self, transition: Transition) -> Result<LifecycleState, LifecycleError> {
        use LifecycleState::*;
        use Transition::*;

        if self == Finalized {
            return Err(LifecycleError::Finalized);
        }

        match (self, transition) {
            (Unconfigured, Configure) => Ok(Inactive),
            (Inactive, Activate) => Ok(Active),
            (Active, Deactivate) => Ok(Inactive),
            (Inactive, Cleanup) => Ok(Unconfigured),
            // Shutdown is valid from any non-finalized primary state.
            (Unconfigured | Inactive | Active, Shutdown) => Ok(Finalized),
            (from, transition) => Err(LifecycleError::InvalidTransition { from, transition }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::LifecycleState::*;
    use super::Transition::*;
    use super::*;

    #[test]
    fn canonical_happy_path() {
        let s = Unconfigured;
        let s = s.target(Configure).unwrap();
        assert_eq!(s, Inactive);
        let s = s.target(Activate).unwrap();
        assert_eq!(s, Active);
        let s = s.target(Deactivate).unwrap();
        assert_eq!(s, Inactive);
        let s = s.target(Cleanup).unwrap();
        assert_eq!(s, Unconfigured);
    }

    #[test]
    fn shutdown_from_any_state_finalizes() {
        for st in [Unconfigured, Inactive, Active] {
            assert_eq!(st.target(Shutdown).unwrap(), Finalized);
        }
    }

    #[test]
    fn cannot_activate_from_unconfigured() {
        assert_eq!(
            Unconfigured.target(Activate),
            Err(LifecycleError::InvalidTransition {
                from: Unconfigured,
                transition: Activate,
            })
        );
    }

    #[test]
    fn cannot_configure_twice() {
        assert!(Inactive.target(Configure).is_err());
    }

    #[test]
    fn finalized_is_terminal() {
        assert_eq!(Finalized.target(Configure), Err(LifecycleError::Finalized));
        assert_eq!(Finalized.target(Shutdown), Err(LifecycleError::Finalized));
    }

    #[test]
    fn transition_label_round_trip() {
        for t in [Configure, Activate, Deactivate, Cleanup, Shutdown] {
            assert_eq!(Transition::parse(t.label()), Some(t));
        }
        assert_eq!(Transition::parse("bogus"), None);
    }
}
