//! Bounded priority queue for report intents before durable spooling.

use crate::protocol::{EventSeverity, ReportEnvelope, ReportPayload};
use std::collections::VecDeque;
use std::fmt;

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum ReportClass {
    Critical = 1,
    Control = 2,
    Durable = 3,
    StateProjection = 4,
    Diagnostic = 5,
}

impl ReportClass {
    pub fn classify(report: &ReportEnvelope) -> Self {
        match &report.payload {
            ReportPayload::CommandAck(_) => Self::Control,
            ReportPayload::RobotEvent(event) if event.severity == EventSeverity::Critical => {
                Self::Critical
            }
            ReportPayload::RobotEvent(_) => Self::Durable,
            ReportPayload::State(_) => Self::StateProjection,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EnqueueOutcome {
    Queued,
    CoalescedProjection,
    DroppedProjection,
    DroppedDiagnostic,
}

#[derive(Debug)]
pub struct BoundedReportQueue {
    capacity: usize,
    queues: [VecDeque<ReportEnvelope>; 5],
    dropped_projections: u64,
    dropped_diagnostics: u64,
    critical_backpressure: bool,
}

impl BoundedReportQueue {
    pub fn new(capacity: usize) -> Result<Self, ReportQueueError> {
        if capacity == 0 || capacity > crate::spool::DEFAULT_REPORT_CAPACITY {
            return Err(ReportQueueError::InvalidCapacity);
        }
        Ok(Self {
            capacity,
            queues: std::array::from_fn(|_| VecDeque::new()),
            dropped_projections: 0,
            dropped_diagnostics: 0,
            critical_backpressure: false,
        })
    }

    pub fn enqueue(
        &mut self,
        report: ReportEnvelope,
        class: ReportClass,
    ) -> Result<EnqueueOutcome, ReportQueueError> {
        if ReportClass::classify(&report) != class && class != ReportClass::Diagnostic {
            return Err(ReportQueueError::ClassMismatch);
        }
        if self.len() < self.capacity {
            self.queue_mut(class).push_back(report);
            return Ok(EnqueueOutcome::Queued);
        }
        match class {
            ReportClass::StateProjection => {
                if let Some(existing) = self
                    .queue_mut(ReportClass::StateProjection)
                    .iter_mut()
                    .find(|existing| existing.robot_id == report.robot_id)
                {
                    *existing = report;
                    self.dropped_projections = self.dropped_projections.saturating_add(1);
                    Ok(EnqueueOutcome::CoalescedProjection)
                } else {
                    self.dropped_projections = self.dropped_projections.saturating_add(1);
                    Ok(EnqueueOutcome::DroppedProjection)
                }
            }
            ReportClass::Diagnostic => {
                self.dropped_diagnostics = self.dropped_diagnostics.saturating_add(1);
                Ok(EnqueueOutcome::DroppedDiagnostic)
            }
            ReportClass::Critical | ReportClass::Control | ReportClass::Durable => {
                if self.queue_mut(ReportClass::Diagnostic).pop_back().is_some() {
                    self.dropped_diagnostics = self.dropped_diagnostics.saturating_add(1);
                    self.queue_mut(class).push_back(report);
                    return Ok(EnqueueOutcome::Queued);
                }
                if self
                    .queue_mut(ReportClass::StateProjection)
                    .pop_back()
                    .is_some()
                {
                    self.dropped_projections = self.dropped_projections.saturating_add(1);
                    self.queue_mut(class).push_back(report);
                    return Ok(EnqueueOutcome::Queued);
                }
                self.critical_backpressure = true;
                Err(ReportQueueError::CriticalBackpressure)
            }
        }
    }

    pub fn pop(&mut self) -> Option<ReportEnvelope> {
        self.queues.iter_mut().find_map(VecDeque::pop_front)
    }

    pub fn len(&self) -> usize {
        self.queues.iter().map(VecDeque::len).sum()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub const fn capacity(&self) -> usize {
        self.capacity
    }

    pub const fn dropped_projections(&self) -> u64 {
        self.dropped_projections
    }

    pub const fn dropped_diagnostics(&self) -> u64 {
        self.dropped_diagnostics
    }

    pub const fn critical_backpressure(&self) -> bool {
        self.critical_backpressure
    }

    pub fn clear_backpressure_after_reconciliation(&mut self) {
        self.critical_backpressure = false;
    }

    fn queue_mut(&mut self, class: ReportClass) -> &mut VecDeque<ReportEnvelope> {
        &mut self.queues[class as usize - 1]
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReportQueueError {
    InvalidCapacity,
    ClassMismatch,
    CriticalBackpressure,
}

impl fmt::Display for ReportQueueError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidCapacity => formatter.write_str("report queue capacity is invalid"),
            Self::ClassMismatch => formatter.write_str("report class does not match its payload"),
            Self::CriticalBackpressure => {
                formatter.write_str("critical report queue is full; stop and reconcile")
            }
        }
    }
}

impl std::error::Error for ReportQueueError {}
