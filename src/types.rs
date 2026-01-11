// ============================================================================
// Types Module
// ============================================================================
// Defines all data structures used for communication between components.
// These types represent sensor readings, actuator commands, and feedback.
// ============================================================================

use serde::{Deserialize, Serialize};
use std::time::{Duration, Instant};

// ----------------------------------------------------------------------------
// Sensor Data Types
// ----------------------------------------------------------------------------

/// Raw sensor reading from a single sensor
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorReading {
    /// Unique identifier for the sensor
    pub sensor_id: usize,
    /// Sensor type name
    pub sensor_type: String,
    /// Raw sensor value
    pub value: f64,
    /// Timestamp when reading was taken
    pub timestamp_ns: u128,
    /// Sequence number for ordering
    pub sequence: u64,
}

impl SensorReading {
    pub fn new(sensor_id: usize, sensor_type: String, value: f64, sequence: u64) -> Self {
        Self {
            sensor_id,
            sensor_type,
            value,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos(),
            sequence,
        }
    }
}

/// Processed sensor data after filtering and analysis
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProcessedSensorData {
    /// Original sensor ID
    pub sensor_id: usize,
    /// Sensor type name
    pub sensor_type: String,
    /// Filtered value (after noise reduction)
    pub filtered_value: f64,
    /// Raw value before filtering
    pub raw_value: f64,
    /// Is this reading anomalous?
    pub is_anomaly: bool,
    /// Confidence score (0.0 to 1.0)
    pub confidence: f64,
    /// Processing time in nanoseconds
    pub processing_time_ns: u64,
    /// Timestamp when processing completed
    pub timestamp_ns: u128,
    /// Sequence number
    pub sequence: u64,
}

impl ProcessedSensorData {
    pub fn new(
        sensor_id: usize,
        sensor_type: String,
        filtered_value: f64,
        raw_value: f64,
        is_anomaly: bool,
        confidence: f64,
        processing_time_ns: u64,
        sequence: u64,
    ) -> Self {
        Self {
            sensor_id,
            sensor_type,
            filtered_value,
            raw_value,
            is_anomaly,
            confidence,
            processing_time_ns,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos(),
            sequence,
        }
    }
}

// ----------------------------------------------------------------------------
// Actuator Types
// ----------------------------------------------------------------------------

/// Command sent to an actuator
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActuatorCommand {
    /// Target actuator ID
    pub actuator_id: usize,
    /// Actuator name
    pub actuator_name: String,
    /// Commanded output value
    pub output_value: f64,
    /// Priority level (higher = more urgent)
    pub priority: u8,
    /// Source sensor ID that triggered this command
    pub source_sensor_id: usize,
    /// Timestamp
    pub timestamp_ns: u128,
    /// Sequence number
    pub sequence: u64,
}

impl ActuatorCommand {
    pub fn new(
        actuator_id: usize,
        actuator_name: String,
        output_value: f64,
        priority: u8,
        source_sensor_id: usize,
        sequence: u64,
    ) -> Self {
        Self {
            actuator_id,
            actuator_name,
            output_value,
            priority,
            source_sensor_id,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos(),
            sequence,
        }
    }
}

/// Current state of an actuator
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActuatorState {
    /// Actuator ID
    pub actuator_id: usize,
    /// Current position/output value
    pub current_value: f64,
    /// Target value from latest command
    pub target_value: f64,
    /// Error (difference between target and current)
    pub error: f64,
    /// Timestamp
    pub timestamp_ns: u128,
}

impl ActuatorState {
    pub fn new(actuator_id: usize) -> Self {
        Self {
            actuator_id,
            current_value: 0.0,
            target_value: 0.0,
            error: 0.0,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos(),
        }
    }
}

// ----------------------------------------------------------------------------
// Feedback Types
// ----------------------------------------------------------------------------

/// Feedback sent from actuator back to sensor module
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActuatorFeedback {
    /// Source actuator ID
    pub actuator_id: usize,
    /// Acknowledgment of received command sequence
    pub ack_sequence: u64,
    /// Current actuator state
    pub state: ActuatorState,
    /// Any error messages
    pub error_message: Option<String>,
    /// Suggested calibration adjustment
    pub calibration_adjustment: Option<f64>,
    /// Response time in nanoseconds
    pub response_time_ns: u64,
    /// Timestamp
    pub timestamp_ns: u128,
}

impl ActuatorFeedback {
    pub fn new(actuator_id: usize, ack_sequence: u64, state: ActuatorState) -> Self {
        Self {
            actuator_id,
            ack_sequence,
            state,
            error_message: None,
            calibration_adjustment: None,
            response_time_ns: 0,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos(),
        }
    }
}

// ----------------------------------------------------------------------------
// Timing and Performance Types
// ----------------------------------------------------------------------------

/// Timing metrics for a single operation
#[derive(Debug, Clone, Default)]
pub struct TimingMetrics {
    /// Start time of the operation
    pub start: Option<Instant>,
    /// End time of the operation
    pub end: Option<Instant>,
    /// Duration in nanoseconds
    pub duration_ns: u64,
    /// Did the operation meet its deadline?
    pub met_deadline: bool,
    /// The deadline that was set
    pub deadline: Duration,
}

impl TimingMetrics {
    pub fn new(deadline: Duration) -> Self {
        Self {
            start: None,
            end: None,
            duration_ns: 0,
            met_deadline: true,
            deadline,
        }
    }

    /// Start timing
    pub fn start(&mut self) {
        self.start = Some(Instant::now());
    }

    /// Stop timing and check deadline
    pub fn stop(&mut self) {
        let end = Instant::now();
        self.end = Some(end);
        if let Some(start) = self.start {
            let duration = end.duration_since(start);
            self.duration_ns = duration.as_nanos() as u64;
            self.met_deadline = duration <= self.deadline;
        }
    }
}

/// Aggregated statistics for a series of operations
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PerformanceStats {
    /// Total number of operations
    pub count: usize,
    /// Minimum latency in nanoseconds
    pub min_latency_ns: u64,
    /// Maximum latency in nanoseconds
    pub max_latency_ns: u64,
    /// Average latency in nanoseconds
    pub avg_latency_ns: f64,
    /// Standard deviation of latency
    pub std_dev_ns: f64,
    /// Number of missed deadlines
    pub missed_deadlines: usize,
    /// Deadline hit rate (0.0 to 1.0)
    pub deadline_hit_rate: f64,
    /// Throughput (operations per second)
    pub throughput: f64,
}

impl PerformanceStats {
    /// Create new stats from a list of timing measurements
    pub fn from_measurements(latencies_ns: &[u64], missed: usize, total_time_s: f64) -> Self {
        if latencies_ns.is_empty() {
            return Self::default();
        }

        let count = latencies_ns.len();
        let min = *latencies_ns.iter().min().unwrap();
        let max = *latencies_ns.iter().max().unwrap();
        let sum: u64 = latencies_ns.iter().sum();
        let avg = sum as f64 / count as f64;

        // Calculate standard deviation
        let variance: f64 = latencies_ns
            .iter()
            .map(|&x| (x as f64 - avg).powi(2))
            .sum::<f64>()
            / count as f64;
        let std_dev = variance.sqrt();

        let deadline_hit_rate = if count > 0 {
            (count - missed) as f64 / count as f64
        } else {
            1.0
        };

        let throughput = if total_time_s > 0.0 {
            count as f64 / total_time_s
        } else {
            0.0
        };

        Self {
            count,
            min_latency_ns: min,
            max_latency_ns: max,
            avg_latency_ns: avg,
            std_dev_ns: std_dev,
            missed_deadlines: missed,
            deadline_hit_rate,
            throughput,
        }
    }

    /// Print a formatted summary of the statistics
    pub fn print_summary(&self, name: &str) {
        println!("\n=== {} Performance Statistics ===", name);
        println!("  Total Operations: {}", self.count);
        println!("  Min Latency:      {:.3} µs", self.min_latency_ns as f64 / 1000.0);
        println!("  Max Latency:      {:.3} µs", self.max_latency_ns as f64 / 1000.0);
        println!("  Avg Latency:      {:.3} µs", self.avg_latency_ns / 1000.0);
        println!("  Std Dev:          {:.3} µs", self.std_dev_ns / 1000.0);
        println!("  Jitter (Max-Min): {:.3} µs", (self.max_latency_ns - self.min_latency_ns) as f64 / 1000.0);
        println!("  Missed Deadlines: {}", self.missed_deadlines);
        println!("  Deadline Hit Rate: {:.2}%", self.deadline_hit_rate * 100.0);
        println!("  Throughput:       {:.2} ops/sec", self.throughput);
    }
}

// ----------------------------------------------------------------------------
// System State Types
// ----------------------------------------------------------------------------

/// Overall system operating mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SystemMode {
    /// Normal operation
    Normal,
    /// System shutdown
    Shutdown,
}

impl Default for SystemMode {
    fn default() -> Self {
        SystemMode::Normal
    }
}

/// Diagnostic log entry
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiagnosticEntry {
    /// Log level
    pub level: LogLevel,
    /// Source component
    pub source: String,
    /// Message
    pub message: String,
    /// Timestamp
    pub timestamp_ns: u128,
}

impl DiagnosticEntry {
    pub fn new(level: LogLevel, source: &str, message: &str) -> Self {
        Self {
            level,
            source: source.to_string(),
            message: message.to_string(),
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos(),
        }
    }
}

/// Log severity levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LogLevel {
    Debug,
    Info,
    Warning,
    Error,
    Critical,
}
