# RTS Manufacturing System

## Real-Time Sensor-Actuator Control System for Automated Manufacturing

A comprehensive real-time system simulation implemented in Rust, demonstrating sensor data processing, actuator control with PID algorithms, and various advanced features for a precision manufacturing line scenario.

---

## Table of Contents

1. [Overview](#overview)
2. [Features](#features)
3. [System Architecture](#system-architecture)
4. [Requirements](#requirements)
5. [Installation & Running](#installation--running)
6. [Project Structure](#project-structure)
7. [Component Details](#component-details)
8. [Advanced Features](#advanced-features)
9. [Benchmarking](#benchmarking)
10. [Configuration](#configuration)

---

## Overview

This system simulates a real-time control system for an automated manufacturing line where robotic arms perform high-speed assembly operations. The system consists of two main components:

- **Component A (Sensor Module)**: Generates and processes sensor data from force, position, and temperature sensors
- **Component B (Actuator Module)**: Controls multiple actuators (gripper, motor, stabilizer) using PID control

The system demonstrates real-time programming concepts including:
- Multi-threaded concurrent operation
- Shared resource synchronization (Mutex, RwLock, Atomics)
- Inter-process communication via channels
- Deadline monitoring and fail-safe mechanisms
- Performance benchmarking

---

## Features

### Core Features
- ✅ Multi-sensor data generation at 5ms intervals
- ✅ Moving average noise reduction filtering
- ✅ Anomaly detection using statistical analysis
- ✅ PID and predictive control algorithms
- ✅ Multiple actuator management (Gripper, Motor, Stabilizer)
- ✅ Closed feedback loop between actuator and sensor
- ✅ Shared resource synchronization with contention tracking
- ✅ Comprehensive performance benchmarking

### Advanced Features (Distinction Level)
- ✅ **Fault Injection**: Simulated sensor dropouts, delays, and data corruption
- ✅ **Fail-Safe Mode**: Automatic degradation when thresholds are violated
- ✅ **CPU Load Simulation**: Testing under varying system loads
- ✅ **Async vs Multi-Threaded Comparison**: Performance comparison between paradigms
- ✅ **Health Monitoring**: Real-time system health assessment

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     SHARED RESOURCES                            │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐  │
│  │ Diagnostic   │  │ Config       │  │ Status Memory        │  │
│  │ Log (Mutex)  │  │ Buffer       │  │ (Lock-Free Atomics)  │  │
│  └──────────────┘  │ (RwLock)     │  └──────────────────────┘  │
│                    └──────────────┘                             │
└─────────────────────────────────────────────────────────────────┘
         │                    │                    │
         ▼                    ▼                    ▼
┌─────────────────────┐              ┌─────────────────────────┐
│   SENSOR MODULE     │              │    ACTUATOR MODULE      │
│   (Component A)     │              │    (Component B)        │
│                     │              │                         │
│ ┌─────────────────┐ │              │ ┌─────────────────────┐ │
│ │ Sensor Simulators│ │              │ │ Virtual Actuators   │ │
│ │ - Force         │ │  Sensor Data │ │ - Gripper           │ │
│ │ - Position      │ │──────────────▶│ │ - Motor             │ │
│ │ - Temperature   │ │   (Channel)  │ │ - Stabilizer        │ │
│ └─────────────────┘ │              │ └─────────────────────┘ │
│                     │              │                         │
│ ┌─────────────────┐ │              │ ┌─────────────────────┐ │
│ │ Data Processor  │ │              │ │ PID Controllers     │ │
│ │ - Moving Avg    │ │              │ │ - Per Actuator      │ │
│ │ - Anomaly Det.  │ │◀─────────────│ │ - Predictive Mode   │ │
│ └─────────────────┘ │   Feedback   │ └─────────────────────┘ │
│                     │   (Channel)  │                         │
│                     │              │ ┌─────────────────────┐ │
│                     │              │ │ Fail-Safe Manager   │ │
│                     │              │ │ Health Monitor      │ │
│                     │              │ └─────────────────────┘ │
└─────────────────────┘              └─────────────────────────┘
```

---

## Requirements

- **Rust**: 1.70 or later (2021 edition)
- **Cargo**: Included with Rust installation
- **OS**: Linux, macOS, or Windows

### Dependencies (managed by Cargo)
- `crossbeam` / `crossbeam-channel`: High-performance channels
- `parking_lot`: Fast synchronization primitives
- `tokio`: Async runtime for comparison benchmarks
- `rand` / `rand_distr`: Random number generation
- `serde` / `serde_json`: Serialization
- `criterion`: Benchmarking framework

---

## Installation & Running

### Clone/Setup
```bash
# Navigate to project directory
cd rts_project

# Build the project
cargo build --release
```

### Run the Main Program
```bash
# Run with optimizations (recommended)
cargo run --release

# Run in debug mode (slower, more checks)
cargo run
```

### Run Benchmarks
```bash
# Run criterion benchmarks
cargo bench

# Run only specific benchmark
cargo bench -- sensor_generation
```

### Run Tests
```bash
# Run all tests
cargo test

# Run tests with output
cargo test -- --nocapture
```

---

## Project Structure

```
rts_project/
├── Cargo.toml              # Project configuration and dependencies
├── README.md               # This file
├── src/
│   ├── lib.rs              # Library root - module declarations
│   ├── main.rs             # Main entry point - system demonstration
│   ├── config.rs           # System configuration constants
│   ├── types.rs            # Data structures and types
│   ├── sensor.rs           # Component A - Sensor module
│   ├── actuator.rs         # Component B - Actuator module
│   ├── pid_controller.rs   # PID and predictive control algorithms
│   ├── shared_resource.rs  # Shared resources with synchronization
│   ├── ipc.rs              # Inter-process communication channels
│   ├── fault_injection.rs  # Fault injection for testing
│   ├── failsafe.rs         # Fail-safe and health monitoring
│   └── benchmark.rs        # Performance benchmarking utilities
└── benches/
    └── system_benchmark.rs # Criterion benchmark suite
```

---

## Component Details

### Component A: Sensor Module (`sensor.rs`)

**Responsibilities:**
1. Generate sensor readings at fixed intervals (5ms)
2. Apply noise reduction (moving average filter)
3. Detect anomalies using z-score analysis
4. Transmit processed data within 0.1ms deadline
5. Process feedback for dynamic recalibration

**Key Classes:**
- `SensorSimulator`: Generates realistic sensor data with noise
- `DataProcessor`: Filters data and detects anomalies
- `SensorModule`: Orchestrates the complete sensor pipeline

### Component B: Actuator Module (`actuator.rs`)

**Responsibilities:**
1. Receive sensor data efficiently
2. Apply PID control to compute actuator outputs
3. Manage multiple actuators concurrently
4. Send feedback within 0.5ms deadline
5. Handle fail-safe transitions

**Key Classes:**
- `VirtualActuator`: Simulates physical actuator dynamics
- `ActuatorModule`: Manages all actuators and control logic
- `PidControllerBank`: Collection of PID controllers

---

## Advanced Features

### 1. Fault Injection (`fault_injection.rs`)

Tests system robustness by injecting:
- **Dropouts**: Complete sensor reading loss
- **Delays**: Artificial packet delays
- **Corruption**: Data value corruption
- **Noise**: Excessive noise spikes

```rust
// Example: Configure fault probabilities
fault_injector.set_probabilities(
    0.05,  // 5% dropout rate
    0.03,  // 3% delay rate
    0.02,  // 2% corruption rate
    0.05,  // 5% noise rate
);
```

### 2. Fail-Safe Mode (`failsafe.rs`)

State machine with transitions:
```
Normal → Warning → Degraded → Critical → Recovery → Normal
```

Triggered by:
- Consecutive missed deadlines
- Consecutive sensor anomalies
- Manual trigger

### 3. CPU Load Simulation

Tests performance under load:
- 0% load (baseline)
- 30% load
- 60% load
- 80% load

### 4. Async vs Multi-Threaded Comparison

Compares:
- Average latency
- Maximum latency
- Throughput (ops/sec)

---

## Benchmarking

### Criterion Benchmarks

Run comprehensive benchmarks:
```bash
cargo bench
```

Results are saved to `target/criterion/` with HTML reports.

### Benchmark Categories

| Benchmark | Description | Typical Result |
|-----------|-------------|----------------|
| `sensor_generation` | Single sensor reading generation | < 1 µs |
| `data_processing` | Moving average + anomaly detection | < 5 µs |
| `pid_control_update` | PID controller calculation | < 1 µs |
| `channel_send` | Channel send/receive pair | < 1 µs |
| `mutex_lock_log` | Mutex lock for logging | < 1 µs |
| `rwlock_read` | RwLock read operation | < 0.5 µs |
| `atomic_increment` | Atomic counter increment | < 0.1 µs |

### Timing Constraints

| Operation | Deadline | Typical |
|-----------|----------|---------|
| Processing | 0.2 ms (200 µs) | ~5-20 µs |
| Transmission | 0.1 ms (100 µs) | ~1-5 µs |
| Actuator Response | 2 ms | ~50-200 µs |
| Feedback | 0.5 ms (500 µs) | ~10-50 µs |

---

## Configuration

Key configuration parameters in `src/config.rs`:

```rust
// Timing
pub const SENSOR_SAMPLE_INTERVAL: Duration = Duration::from_millis(5);
pub const PROCESSING_DEADLINE: Duration = Duration::from_micros(200);
pub const ACTUATOR_DEADLINE: Duration = Duration::from_millis(2);

// PID Parameters
pub const PID_KP: f64 = 0.5;  // Proportional gain
pub const PID_KI: f64 = 0.1;  // Integral gain
pub const PID_KD: f64 = 0.05; // Derivative gain

// Fail-Safe Thresholds
pub const FAILSAFE_MISSED_DEADLINE_THRESHOLD: usize = 3;
pub const FAILSAFE_ANOMALY_THRESHOLD: usize = 5;

// Fault Injection
pub const FAULT_DROPOUT_PROBABILITY: f64 = 0.05;
pub const FAULT_DELAY_PROBABILITY: f64 = 0.03;
```

---

## Sample Output

```
============================================================
  RTS Manufacturing System - Real-Time Control Simulation
============================================================

=== Part 1: Multi-Threaded Integration ===
Initializing multi-threaded system...

--- Multi-Threaded System Results ---
Total Runtime: 2.50 seconds
Total Sensor Cycles: 500
Total Actuator Cycles: 498

Sensor Performance:
  Avg Generation Time:   0.850 µs
  Avg Processing Time:   3.200 µs
  Avg Transmission Time: 1.100 µs
  Missed Deadlines:      0

Actuator Performance:
  Avg Reception Time:    12.500 µs
  Avg Control Time:      2.800 µs
  Avg Feedback Time:     1.500 µs
  Missed Deadlines:      0
  Fail-Safe State:       Normal

=== Shared Resource Synchronization Statistics ===
Diagnostic Log:
  Total Writes:     15
  Lock Contentions: 0
  Contention Rate:  0.00%
```

---

## License

This project is developed for educational purposes as part of the CT087-3-3-RTS Real-Time Systems course.

---

## Author

Student Assignment - Real-Time Systems Module
