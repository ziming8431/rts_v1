// ============================================================================
// RTS Manufacturing System - Real-Time Sensor-Actuator Control
// ============================================================================
// This system simulates a precision manufacturing line with:
// - Sensor data generation and processing (Component A)
// - Actuator control with PID algorithms (Component B)
// - Shared resource synchronization
// - Performance benchmarking
// - Advanced features: fault injection, CPU load simulation
// ============================================================================

pub mod sensor;
pub mod actuator;
pub mod shared_resource;
pub mod ipc;
pub mod pid_controller;
pub mod benchmark;
pub mod fault_injection;
pub mod types;
pub mod config;
