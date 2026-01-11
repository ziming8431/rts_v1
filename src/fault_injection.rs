// ============================================================================
// Fault Injection Module (Advanced Feature)
// ============================================================================
// Implements simulated faults to test system robustness:
// - Sensor dropouts (missing data)
// - Delayed packets
// - Corrupted data
// - Communication failures
// ============================================================================

use crate::config::*;
use crate::types::*;
use rand::Rng;
use std::time::Duration;

// ----------------------------------------------------------------------------
// Fault Types
// ----------------------------------------------------------------------------

/// Types of faults that can be injected
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultType {
    /// Sensor reading is completely dropped
    Dropout,
    /// Sensor reading is delayed
    Delay,
    /// Sensor reading is corrupted (wrong value)
    Corruption,
    /// Sensor reading has excessive noise
    Noise,
    /// Communication channel failure
    ChannelFailure,
    /// No fault
    None,
}

/// Record of an injected fault for logging
#[derive(Debug, Clone)]
pub struct FaultRecord {
    /// Type of fault that was injected
    pub fault_type: FaultType,
    /// When the fault was injected
    pub timestamp_ns: u128,
    /// Which sensor was affected
    pub sensor_id: usize,
    /// Original value before corruption (if applicable)
    pub original_value: Option<f64>,
    /// Corrupted value (if applicable)
    pub corrupted_value: Option<f64>,
    /// Delay amount (if applicable)
    pub delay: Option<Duration>,
}

// ----------------------------------------------------------------------------
// Fault Injector
// ----------------------------------------------------------------------------

/// Manages fault injection for testing system robustness
pub struct FaultInjector {
    /// Is fault injection enabled?
    enabled: bool,
    /// Probability of dropout (0.0 to 1.0)
    dropout_probability: f64,
    /// Probability of delay (0.0 to 1.0)
    delay_probability: f64,
    /// Probability of corruption (0.0 to 1.0)
    corruption_probability: f64,
    /// Probability of noise spike (0.0 to 1.0)
    noise_probability: f64,
    /// Maximum delay for delayed packets
    max_delay: Duration,
    /// Maximum corruption magnitude
    max_corruption: f64,
    /// History of injected faults
    fault_history: Vec<FaultRecord>,
    /// Total dropouts injected
    total_dropouts: usize,
    /// Total delays injected
    total_delays: usize,
    /// Total corruptions injected
    total_corruptions: usize,
    /// Random number generator
    rng: rand::rngs::ThreadRng,
}

impl FaultInjector {
    /// Create a new fault injector with default probabilities
    pub fn new() -> Self {
        Self {
            enabled: true,
            dropout_probability: FAULT_DROPOUT_PROBABILITY,
            delay_probability: FAULT_DELAY_PROBABILITY,
            corruption_probability: 0.02,
            noise_probability: 0.05,
            max_delay: FAULT_MAX_DELAY,
            max_corruption: 50.0,
            fault_history: Vec::new(),
            total_dropouts: 0,
            total_delays: 0,
            total_corruptions: 0,
            rng: rand::thread_rng(),
        }
    }

    /// Enable or disable fault injection
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    /// Check if fault injection is enabled
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Set fault probabilities
    pub fn set_probabilities(
        &mut self,
        dropout: f64,
        delay: f64,
        corruption: f64,
        noise: f64,
    ) {
        self.dropout_probability = dropout.clamp(0.0, 1.0);
        self.delay_probability = delay.clamp(0.0, 1.0);
        self.corruption_probability = corruption.clamp(0.0, 1.0);
        self.noise_probability = noise.clamp(0.0, 1.0);
    }

    /// Determine what type of fault (if any) to inject
    pub fn should_inject_fault(&mut self) -> FaultType {
        if !self.enabled {
            return FaultType::None;
        }

        let roll: f64 = self.rng.gen();

        // Check each fault type in order of severity
        if roll < self.dropout_probability {
            return FaultType::Dropout;
        }
        
        let roll: f64 = self.rng.gen();
        if roll < self.delay_probability {
            return FaultType::Delay;
        }
        
        let roll: f64 = self.rng.gen();
        if roll < self.corruption_probability {
            return FaultType::Corruption;
        }
        
        let roll: f64 = self.rng.gen();
        if roll < self.noise_probability {
            return FaultType::Noise;
        }

        FaultType::None
    }

    /// Apply a fault to sensor data
    /// Returns None if data should be dropped, Some(data) otherwise
    pub fn apply_fault(
        &mut self,
        data: ProcessedSensorData,
    ) -> Option<(ProcessedSensorData, FaultRecord)> {
        if !self.enabled {
            let record = FaultRecord {
                fault_type: FaultType::None,
                timestamp_ns: data.timestamp_ns,
                sensor_id: data.sensor_id,
                original_value: None,
                corrupted_value: None,
                delay: None,
            };
            return Some((data, record));
        }

        let fault_type = self.should_inject_fault();
        
        match fault_type {
            FaultType::Dropout => {
                self.total_dropouts += 1;
                let record = FaultRecord {
                    fault_type: FaultType::Dropout,
                    timestamp_ns: data.timestamp_ns,
                    sensor_id: data.sensor_id,
                    original_value: Some(data.filtered_value),
                    corrupted_value: None,
                    delay: None,
                };
                self.fault_history.push(record.clone());
                None // Data is dropped
            }
            FaultType::Delay => {
                self.total_delays += 1;
                let delay_ms = self.rng.gen_range(1..=self.max_delay.as_millis() as u64);
                let delay = Duration::from_millis(delay_ms);
                
                // Simulate delay (in real system, would actually delay)
                std::thread::sleep(delay);
                
                let record = FaultRecord {
                    fault_type: FaultType::Delay,
                    timestamp_ns: data.timestamp_ns,
                    sensor_id: data.sensor_id,
                    original_value: Some(data.filtered_value),
                    corrupted_value: None,
                    delay: Some(delay),
                };
                self.fault_history.push(record.clone());
                Some((data, record))
            }
            FaultType::Corruption => {
                self.total_corruptions += 1;
                let original = data.filtered_value;
                let corruption = self.rng.gen_range(-self.max_corruption..self.max_corruption);
                let mut corrupted_data = data;
                corrupted_data.filtered_value += corruption;
                corrupted_data.is_anomaly = true;
                
                let record = FaultRecord {
                    fault_type: FaultType::Corruption,
                    timestamp_ns: corrupted_data.timestamp_ns,
                    sensor_id: corrupted_data.sensor_id,
                    original_value: Some(original),
                    corrupted_value: Some(corrupted_data.filtered_value),
                    delay: None,
                };
                self.fault_history.push(record.clone());
                Some((corrupted_data, record))
            }
            FaultType::Noise => {
                let original = data.filtered_value;
                let noise = self.rng.gen_range(-10.0..10.0);
                let mut noisy_data = data;
                noisy_data.filtered_value += noise;
                noisy_data.confidence *= 0.8; // Reduce confidence
                
                let record = FaultRecord {
                    fault_type: FaultType::Noise,
                    timestamp_ns: noisy_data.timestamp_ns,
                    sensor_id: noisy_data.sensor_id,
                    original_value: Some(original),
                    corrupted_value: Some(noisy_data.filtered_value),
                    delay: None,
                };
                self.fault_history.push(record.clone());
                Some((noisy_data, record))
            }
            FaultType::None | FaultType::ChannelFailure => {
                let record = FaultRecord {
                    fault_type: FaultType::None,
                    timestamp_ns: data.timestamp_ns,
                    sensor_id: data.sensor_id,
                    original_value: None,
                    corrupted_value: None,
                    delay: None,
                };
                Some((data, record))
            }
        }
    }

    /// Get fault injection statistics
    pub fn get_stats(&self) -> FaultStats {
        FaultStats {
            total_faults: self.fault_history.len(),
            dropouts: self.total_dropouts,
            delays: self.total_delays,
            corruptions: self.total_corruptions,
            dropout_rate: if !self.fault_history.is_empty() {
                self.total_dropouts as f64 / self.fault_history.len() as f64
            } else {
                0.0
            },
        }
    }

    /// Get recent fault history
    pub fn get_recent_faults(&self, count: usize) -> Vec<FaultRecord> {
        self.fault_history
            .iter()
            .rev()
            .take(count)
            .cloned()
            .collect()
    }

    /// Clear fault history
    pub fn clear_history(&mut self) {
        self.fault_history.clear();
        self.total_dropouts = 0;
        self.total_delays = 0;
        self.total_corruptions = 0;
    }

    /// Print fault statistics
    pub fn print_stats(&self) {
        let stats = self.get_stats();
        println!("\n=== Fault Injection Statistics ===");
        println!("  Enabled:        {}", self.enabled);
        println!("  Total Faults:   {}", stats.total_faults);
        println!("  Dropouts:       {} ({:.2}%)", 
            stats.dropouts, 
            stats.dropout_rate * 100.0);
        println!("  Delays:         {}", stats.delays);
        println!("  Corruptions:    {}", stats.corruptions);
        println!("  Probabilities:");
        println!("    Dropout:      {:.1}%", self.dropout_probability * 100.0);
        println!("    Delay:        {:.1}%", self.delay_probability * 100.0);
        println!("    Corruption:   {:.1}%", self.corruption_probability * 100.0);
        println!("    Noise:        {:.1}%", self.noise_probability * 100.0);
    }
}

impl Default for FaultInjector {
    fn default() -> Self {
        Self::new()
    }
}

/// Statistics about injected faults
#[derive(Debug, Clone)]
pub struct FaultStats {
    pub total_faults: usize,
    pub dropouts: usize,
    pub delays: usize,
    pub corruptions: usize,
    pub dropout_rate: f64,
}

// ----------------------------------------------------------------------------
// Fault Detection
// ----------------------------------------------------------------------------

/// Detects and reports faults in incoming data
pub struct FaultDetector {
    /// Expected sequence numbers for each sensor
    expected_sequences: Vec<u64>,
    /// Maximum acceptable gap in sequence numbers
    max_sequence_gap: u64,
    /// Last received timestamps
    last_timestamps: Vec<u128>,
    /// Maximum acceptable timestamp gap (ns)
    max_timestamp_gap_ns: u128,
    /// Detected fault count
    detected_faults: usize,
}

impl FaultDetector {
    /// Create a new fault detector
    pub fn new(num_sensors: usize) -> Self {
        let max_timestamp_gap_ns = SENSOR_SAMPLE_INTERVAL.as_nanos() as u128 * 2;
        Self {
            expected_sequences: vec![0; num_sensors],
            max_sequence_gap: 0,
            last_timestamps: vec![0; num_sensors],
            max_timestamp_gap_ns,
            detected_faults: 0,
        }
    }

    /// Check sensor data for faults
    /// Returns a list of detected issues
    pub fn check_data(&mut self, data: &ProcessedSensorData) -> Vec<String> {
        let mut issues = Vec::new();
        let sensor_id = data.sensor_id;
        let mut fault_detected = false;

        if sensor_id >= self.expected_sequences.len() {
            issues.push(format!("Unknown sensor ID: {}", sensor_id));
            self.detected_faults += 1;
            return issues;
        }

        // Check sequence number
        let expected = self.expected_sequences[sensor_id];
        if data.sequence > expected + self.max_sequence_gap {
            issues.push(format!(
                "Sequence gap detected: expected {}, got {}",
                expected, data.sequence
            ));
            fault_detected = true;
        }
        self.expected_sequences[sensor_id] = data.sequence + 1;

        // Check timestamp gap
        let last_ts = self.last_timestamps[sensor_id];
        if last_ts > 0 && data.timestamp_ns > last_ts + self.max_timestamp_gap_ns {
            let gap_ms = (data.timestamp_ns - last_ts) / 1_000_000;
            issues.push(format!("Timestamp gap of {} ms detected", gap_ms));
            fault_detected = true;
        }
        self.last_timestamps[sensor_id] = data.timestamp_ns;

        // Check for anomalies
        if data.is_anomaly {
            issues.push(format!(
                "Anomaly flagged: value={:.2}, confidence={:.2}",
                data.filtered_value, data.confidence
            ));
            fault_detected = true;
        }

        // Check confidence
        if data.confidence < 0.5 {
            issues.push(format!("Low confidence reading: {:.2}", data.confidence));
            fault_detected = true;
        }

        if fault_detected {
            self.detected_faults += 1;
        }

        issues
    }

    /// Get total detected faults
    pub fn get_fault_count(&self) -> usize {
        self.detected_faults
    }

    /// Reset the detector
    pub fn reset(&mut self) {
        self.expected_sequences.iter_mut().for_each(|x| *x = 0);
        self.last_timestamps.iter_mut().for_each(|x| *x = 0);
        self.detected_faults = 0;
    }
}
