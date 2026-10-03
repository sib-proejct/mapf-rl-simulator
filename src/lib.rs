//! Deterministic virtual-robot domain engine for the MAPF-RL Simulator.
//!
//! The Phase 1 engine owns all authoritative state behind mutable access, advances
//! simulation time in fixed 100 ms ticks, and applies motion only after the local
//! deterministic safety kernel accepts it.

pub mod action;
pub mod checkpoint;
pub mod contracts;
pub mod controller;
pub mod core_client;
pub mod fault;
pub mod fleet;
pub mod motion;
pub mod operational;
pub mod plan;
pub mod protocol;
pub mod random;
pub mod report_queue;
pub mod route;
pub mod runtime;
pub mod safety;
pub mod scenario;
pub mod sensing;
pub mod session;
pub mod simulation;
pub mod spool;
pub mod station;
pub mod types;
pub mod world;

pub mod fleet_operational;
