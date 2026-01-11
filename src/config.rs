// ============================================================================
// Configuration Module
// ============================================================================
// Contains all system-wide constants and configuration parameters.
// These values define timing constraints, thresholds, and system behavior.
// ============================================================================

use std::time::Duration;

// ----------------------------------------------------------------------------
// Timing Constraints (Simulated Real-Time Deadlines)
// ----------------------------------------------------------------------------
// Note: These deadlines are intentionally tight for experimentation.
// Real-world OS limitations mean actual performance may vary.

/// Sensor sampling interval (5 ms as per requirements)
pub const SENSOR_SAMPLE_INTERVAL: Duration = Duration::from_millis(5);

/// Maximum allowed processing time per cycle (0.2 ms target)
pub const PROCESSING_DEADLINE: Duration = Duration::from_micros(200);

/// Maximum transmission latency after processing (0.1 ms target)
pub const TRANSMISSION_DEADLINE: Duration = Duration::from_micros(100);

/// Actuator response deadline (1-2 ms)
pub const ACTUATOR_DEADLINE: Duration = Duration::from_millis(2);

/// Feedback transmission deadline (0.5 ms)
pub const FEEDBACK_DEADLINE: Duration = Duration::from_micros(500);

// ----------------------------------------------------------------------------
// Sensor Configuration
// ----------------------------------------------------------------------------

/// Number of sensor types in the system
pub const NUM_SENSOR_TYPES: usize = 3;

/// Sensor type identifiers
pub const SENSOR_FORCE: usize = 0;
pub const SENSOR_POSITION: usize = 1;
pub const SENSOR_TEMPERATURE: usize = 2;

/// Sensor names for display
pub const SENSOR_NAMES: [&str; NUM_SENSOR_TYPES] = ["Force", "Position", "Temperature"];

/// Moving average window size for noise filtering
pub const MOVING_AVERAGE_WINDOW: usize = 5;

/// Anomaly detection threshold (standard deviations from mean)
pub const ANOMALY_THRESHOLD: f64 = 3.0;

/// Inject an outlier reading every N cycles (0 disables injection)
pub const ANOMALY_INJECTION_PERIOD: usize = 50;

// ----------------------------------------------------------------------------
// Actuator Configuration
// ----------------------------------------------------------------------------

/// Number of actuators in the system
pub const NUM_ACTUATORS: usize = 3;

/// Actuator type identifiers
pub const ACTUATOR_GRIPPER: usize = 0;
pub const ACTUATOR_MOTOR: usize = 1;
pub const ACTUATOR_STABILIZER: usize = 2;

/// Actuator names for display
pub const ACTUATOR_NAMES: [&str; NUM_ACTUATORS] = ["Gripper", "Motor", "Stabilizer"];

// ----------------------------------------------------------------------------
// PID Controller Default Parameters
// ----------------------------------------------------------------------------

/// Proportional gain
pub const PID_KP: f64 = 0.5;

/// Integral gain
pub const PID_KI: f64 = 0.1;

/// Derivative gain
pub const PID_KD: f64 = 0.05;

/// Output limits for PID controller
pub const PID_OUTPUT_MIN: f64 = -100.0;
pub const PID_OUTPUT_MAX: f64 = 100.0;

// ----------------------------------------------------------------------------
// System Limits
// ----------------------------------------------------------------------------

/// Maximum number of cycles to run in simulation
pub const MAX_SIMULATION_CYCLES: usize = 1000;

/// Demo cycles for multi-threaded integration run
pub const DEMO_INTEGRATION_CYCLES: usize = 5000;

/// Demo cycles for fault injection run
pub const DEMO_FAULT_INJECTION_CYCLES: usize = 300;

/// Demo cycles for CPU load comparison run
pub const DEMO_LOAD_CYCLES: usize = 200;

/// Channel buffer size for IPC
pub const CHANNEL_BUFFER_SIZE: usize = 100;

/// Shared resource access timeout
pub const RESOURCE_ACCESS_TIMEOUT: Duration = Duration::from_millis(1);

// ----------------------------------------------------------------------------
// Fault Injection Configuration (Advanced Feature)
// ----------------------------------------------------------------------------

/// Probability of sensor dropout (0.0 to 1.0)
pub const FAULT_DROPOUT_PROBABILITY: f64 = 0.05;

/// Probability of delayed packet (0.0 to 1.0)
pub const FAULT_DELAY_PROBABILITY: f64 = 0.03;

/// Maximum artificial delay for fault simulation
pub const FAULT_MAX_DELAY: Duration = Duration::from_millis(10);

// ----------------------------------------------------------------------------
// Benchmark Configuration
// ----------------------------------------------------------------------------

/// Number of warmup iterations before benchmarking
pub const BENCHMARK_WARMUP_ITERATIONS: usize = 100;

/// Number of benchmark iterations
pub const BENCHMARK_ITERATIONS: usize = 1000;

// ----------------------------------------------------------------------------
// Display Configuration
// ----------------------------------------------------------------------------

/// How often to print status updates (in cycles)
pub const STATUS_PRINT_INTERVAL: usize = 100;

/// Enable verbose logging
pub const VERBOSE_LOGGING: bool = false;
