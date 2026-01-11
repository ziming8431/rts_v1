// ============================================================================
// Sensor Module (Component A)
// ============================================================================
// Implements the sensor data simulator with:
// - Multi-sensor data generation at fixed intervals
// - Noise reduction filtering (moving average)
// - Anomaly detection
// - Real-time data transmission
// ============================================================================

use crate::config::*;
use crate::fault_injection::{FaultInjector, FaultRecord, FaultType};
use crate::ipc::*;
use crate::shared_resource::*;
use crate::types::*;
use rand_distr::{Distribution, Normal};
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

// ----------------------------------------------------------------------------
// Sensor Simulator
// ----------------------------------------------------------------------------

/// Simulates a single sensor with realistic behavior
pub struct SensorSimulator {
    /// Sensor ID
    id: usize,
    /// Sensor type name
    sensor_type: String,
    /// Base value for simulation
    base_value: f64,
    /// Amplitude of variation
    amplitude: f64,
    /// Current phase for sinusoidal variation
    phase: f64,
    /// Phase increment per sample
    phase_increment: f64,
    /// Random number generator
    rng: rand::rngs::ThreadRng,
    /// Normal distribution for noise
    noise_dist: Normal<f64>,
    /// Sequence counter
    sequence: u64,
}

impl SensorSimulator {
    /// Create a new sensor simulator
    pub fn new(id: usize, sensor_type: &str) -> Self {
        // Configure based on sensor type
        let (base_value, amplitude, noise_level) = match id {
            SENSOR_FORCE => (50.0, 20.0, 2.0),        // Force sensor: 50 ± 20 N
            SENSOR_POSITION => (100.0, 30.0, 1.5),    // Position sensor: 100 ± 30 mm
            SENSOR_TEMPERATURE => (25.0, 5.0, 0.5),   // Temperature: 25 ± 5 °C
            _ => (50.0, 10.0, 1.0),
        };

        Self {
            id,
            sensor_type: sensor_type.to_string(),
            base_value,
            amplitude,
            phase: 0.0,
            phase_increment: 0.05, // Controls frequency of variation
            rng: rand::thread_rng(),
            noise_dist: Normal::new(0.0, noise_level).unwrap(),
            sequence: 0,
        }
    }

    /// Generate a new sensor reading
    pub fn generate_reading(&mut self) -> SensorReading {
        // Sinusoidal variation
        let variation = self.amplitude * self.phase.sin();
        
        // Add random noise
        let noise = self.noise_dist.sample(&mut self.rng);
        
        // Calculate final value
        let value = self.base_value + variation + noise;
        
        // Update phase for next sample
        self.phase += self.phase_increment;
        if self.phase > 2.0 * std::f64::consts::PI {
            self.phase -= 2.0 * std::f64::consts::PI;
        }
        
        self.sequence += 1;
        
        SensorReading::new(
            self.id,
            self.sensor_type.clone(),
            value,
            self.sequence,
        )
    }

    /// Inject an anomaly (for testing)
    pub fn generate_anomaly(&mut self) -> SensorReading {
        let anomaly_value = self.base_value + self.amplitude * 5.0; // Way outside normal range
        self.sequence += 1;
        
        SensorReading::new(
            self.id,
            self.sensor_type.clone(),
            anomaly_value,
            self.sequence,
        )
    }

    /// Get sensor ID
    pub fn get_id(&self) -> usize {
        self.id
    }
}

// ----------------------------------------------------------------------------
// Data Processor (Filtering and Anomaly Detection)
// ----------------------------------------------------------------------------

/// Processes raw sensor data with filtering and anomaly detection
#[derive(Clone)]
pub struct DataProcessor {
    /// Moving average buffers for each sensor
    moving_avg_buffers: Vec<VecDeque<f64>>,
    /// Window size for moving average
    window_size: usize,
    /// Running statistics for anomaly detection
    running_means: Vec<f64>,
    running_variances: Vec<f64>,
    sample_counts: Vec<usize>,
    /// Anomaly threshold (standard deviations)
    anomaly_threshold: f64,
    /// Calibration offsets
    calibration_offsets: Vec<f64>,
}

impl DataProcessor {
    /// Create a new data processor
    pub fn new(num_sensors: usize) -> Self {
        Self {
            moving_avg_buffers: (0..num_sensors)
                .map(|_| VecDeque::with_capacity(MOVING_AVERAGE_WINDOW))
                .collect(),
            window_size: MOVING_AVERAGE_WINDOW,
            running_means: vec![0.0; num_sensors],
            running_variances: vec![1.0; num_sensors],
            sample_counts: vec![0; num_sensors],
            anomaly_threshold: ANOMALY_THRESHOLD,
            calibration_offsets: vec![0.0; num_sensors],
        }
    }

    /// Process a raw sensor reading
    pub fn process(&mut self, reading: &SensorReading) -> ProcessedSensorData {
        let start = Instant::now();
        let sensor_id = reading.sensor_id;
        
        // Apply calibration offset
        let calibrated_value = reading.value + self.calibration_offsets[sensor_id];
        
        // Apply moving average filter
        let filtered_value = self.apply_moving_average(sensor_id, calibrated_value);
        
        // Update running statistics
        self.update_statistics(sensor_id, filtered_value);
        
        // Detect anomalies
        let (is_anomaly, confidence) = self.detect_anomaly(sensor_id, filtered_value);
        
        let processing_time = start.elapsed().as_nanos() as u64;
        
        ProcessedSensorData::new(
            sensor_id,
            reading.sensor_type.clone(),
            filtered_value,
            reading.value,
            is_anomaly,
            confidence,
            processing_time,
            reading.sequence,
        )
    }

    /// Apply moving average filter
    fn apply_moving_average(&mut self, sensor_id: usize, value: f64) -> f64 {
        let buffer = &mut self.moving_avg_buffers[sensor_id];
        
        // Add new value
        buffer.push_back(value);
        
        // Remove oldest if buffer is full
        if buffer.len() > self.window_size {
            buffer.pop_front();
        }
        
        // Calculate average
        let sum: f64 = buffer.iter().sum();
        sum / buffer.len() as f64
    }

    /// Update running statistics using Welford's algorithm
    fn update_statistics(&mut self, sensor_id: usize, value: f64) {
        self.sample_counts[sensor_id] += 1;
        let n = self.sample_counts[sensor_id] as f64;
        
        let delta = value - self.running_means[sensor_id];
        self.running_means[sensor_id] += delta / n;
        
        let delta2 = value - self.running_means[sensor_id];
        self.running_variances[sensor_id] += delta * delta2;
    }

    /// Detect if a value is anomalous
    fn detect_anomaly(&self, sensor_id: usize, value: f64) -> (bool, f64) {
        let n = self.sample_counts[sensor_id];
        
        // Need minimum samples for reliable detection
        if n < 10 {
            return (false, 1.0);
        }
        
        let mean = self.running_means[sensor_id];
        let variance = self.running_variances[sensor_id] / (n - 1) as f64;
        let std_dev = variance.sqrt().max(0.001); // Prevent division by zero
        
        let z_score = (value - mean).abs() / std_dev;
        let is_anomaly = z_score > self.anomaly_threshold;
        
        // Calculate confidence (inverse of z-score, clamped)
        let confidence = (1.0 - (z_score / (self.anomaly_threshold * 2.0))).clamp(0.0, 1.0);
        
        (is_anomaly, confidence)
    }

    /// Set calibration offset for a sensor
    pub fn set_calibration_offset(&mut self, sensor_id: usize, offset: f64) {
        if sensor_id < self.calibration_offsets.len() {
            self.calibration_offsets[sensor_id] = offset;
        }
    }

    /// Apply runtime configuration from shared buffer
    pub fn apply_config(&mut self, config: &SystemConfig) {
        self.anomaly_threshold = config.anomaly_threshold;
        for (dst, src) in self
            .calibration_offsets
            .iter_mut()
            .zip(config.sensor_offsets.iter())
        {
            *dst = *src;
        }
    }

    /// Reset processor state
    pub fn reset(&mut self) {
        for buffer in &mut self.moving_avg_buffers {
            buffer.clear();
        }
        self.running_means.iter_mut().for_each(|x| *x = 0.0);
        self.running_variances.iter_mut().for_each(|x| *x = 1.0);
        self.sample_counts.iter_mut().for_each(|x| *x = 0);
    }
}

// ----------------------------------------------------------------------------
// Sensor Module (Complete Component A)
// ----------------------------------------------------------------------------

/// Complete sensor module with generation, processing, and transmission
pub struct SensorModule {
    /// Individual sensor simulators
    sensors: Vec<SensorSimulator>,
    /// Data processor
    processor: DataProcessor,
    /// Data sender channel
    data_sender: SensorDataSender,
    /// Feedback receiver channel
    feedback_receiver: FeedbackReceiver,
    /// Shared resources
    shared: SharedResources,
    /// Running flag
    running: Arc<AtomicBool>,
    /// Performance metrics
    generation_times: Vec<u64>,
    processing_times: Vec<u64>,
    transmission_times: Vec<u64>,
    missed_deadlines: usize,
    total_cycles: u64,
    /// Last known-good data per sensor for recovery
    last_good_data: Vec<Option<ProcessedSensorData>>,
    /// Recovery actions applied during fault handling
    recovery_count: usize,
}

impl SensorModule {
    /// Create a new sensor module
    pub fn new(
        data_sender: SensorDataSender,
        feedback_receiver: FeedbackReceiver,
        shared: SharedResources,
        running: Arc<AtomicBool>,
    ) -> Self {
        // Create sensors for each type
        let sensors: Vec<SensorSimulator> = (0..NUM_SENSOR_TYPES)
            .map(|i| SensorSimulator::new(i, SENSOR_NAMES[i]))
            .collect();

        Self {
            sensors,
            processor: DataProcessor::new(NUM_SENSOR_TYPES),
            data_sender,
            feedback_receiver,
            shared,
            running,
            generation_times: Vec::new(),
            processing_times: Vec::new(),
            transmission_times: Vec::new(),
            missed_deadlines: 0,
            total_cycles: 0,
            last_good_data: vec![None; NUM_SENSOR_TYPES],
            recovery_count: 0,
        }
    }

    /// Run one cycle of sensor operations
    pub fn run_cycle(&mut self) -> Result<Vec<ProcessedSensorData>, String> {
        let _cycle_start = Instant::now();
        let mut processed_data = Vec::with_capacity(NUM_SENSOR_TYPES);

        // Pull latest configuration for calibration/anomaly tuning.
        let config = self.shared.config_buffer.read();
        self.processor.apply_config(&config);

        // Phase 1: Generate sensor data
        let gen_start = Instant::now();
        let inject_anomaly = ANOMALY_INJECTION_PERIOD > 0
            && self.total_cycles > 0
            && (self.total_cycles % ANOMALY_INJECTION_PERIOD as u64 == 0);
        let readings: Vec<SensorReading> = self.sensors
            .iter_mut()
            .map(|s| {
                if inject_anomaly && s.get_id() == SENSOR_FORCE {
                    s.generate_anomaly()
                } else {
                    s.generate_reading()
                }
            })
            .collect();
        let gen_time = gen_start.elapsed().as_nanos() as u64;
        self.generation_times.push(gen_time);

        // Phase 2: Process each reading
        for reading in &readings {
            let proc_start = Instant::now();
            let mut processed = self.processor.process(reading);
            if inject_anomaly && reading.sensor_id == SENSOR_FORCE {
                processed.is_anomaly = true;
                processed.confidence = processed.confidence.min(0.2);
            }
            let proc_time = proc_start.elapsed().as_nanos() as u64;
            self.processing_times.push(proc_time);

            // Check processing deadline
            if proc_time > PROCESSING_DEADLINE.as_nanos() as u64 {
                self.missed_deadlines += 1;
                self.shared.status_memory.increment_missed_deadlines();
            }

            // Track anomalies
            if processed.is_anomaly {
                self.shared.status_memory.increment_anomalies();
                self.shared.diagnostic_log.try_log(
                    LogLevel::Warning,
                    "Sensor",
                    &format!("Anomaly detected on sensor {}: {:.2}", 
                        processed.sensor_id, processed.filtered_value),
                );
            }

            if !processed.is_anomaly && processed.confidence >= 0.5 {
                self.last_good_data[processed.sensor_id] = Some(processed.clone());
            }

            processed_data.push(processed);
        }

        // Phase 3: Transmit data
        for data in &processed_data {
            let tx_start = Instant::now();
            self.data_sender.try_send(data.clone())
                .map_err(|e| format!("Transmission failed: {}", e))?;
            let tx_time = tx_start.elapsed().as_nanos() as u64;
            self.transmission_times.push(tx_time);

            // Check transmission deadline
            if tx_time > TRANSMISSION_DEADLINE.as_nanos() as u64 {
                self.missed_deadlines += 1;
            }
        }

        // Phase 4: Process any feedback
        self.process_feedback();

        // Update cycle counter
        self.total_cycles += 1;
        self.shared.status_memory.increment_cycles();

        // Log to shared diagnostic (non-blocking)
        if self.total_cycles % 100 == 0 {
            self.shared.diagnostic_log.try_log(
                LogLevel::Info,
                "Sensor",
                &format!("Completed {} cycles", self.total_cycles),
            );
        }

        Ok(processed_data)
    }

    /// Run one cycle with fault injection applied before transmission
    pub fn run_cycle_with_faults(
        &mut self,
        fault_injector: &mut FaultInjector,
    ) -> Result<Vec<(ProcessedSensorData, FaultRecord)>, String> {
        let _cycle_start = Instant::now();
        let mut processed_data = Vec::with_capacity(NUM_SENSOR_TYPES);

        // Pull latest configuration for calibration/anomaly tuning.
        let config = self.shared.config_buffer.read();
        self.processor.apply_config(&config);

        // Phase 1: Generate sensor data
        let gen_start = Instant::now();
        let inject_anomaly = ANOMALY_INJECTION_PERIOD > 0
            && self.total_cycles > 0
            && (self.total_cycles % ANOMALY_INJECTION_PERIOD as u64 == 0);
        let readings: Vec<SensorReading> = self
            .sensors
            .iter_mut()
            .map(|s| {
                if inject_anomaly && s.get_id() == SENSOR_FORCE {
                    s.generate_anomaly()
                } else {
                    s.generate_reading()
                }
            })
            .collect();
        let gen_time = gen_start.elapsed().as_nanos() as u64;
        self.generation_times.push(gen_time);

        // Phase 2: Process each reading
        for reading in &readings {
            let proc_start = Instant::now();
            let mut processed = self.processor.process(reading);
            if inject_anomaly && reading.sensor_id == SENSOR_FORCE {
                processed.is_anomaly = true;
                processed.confidence = processed.confidence.min(0.2);
            }
            let proc_time = proc_start.elapsed().as_nanos() as u64;
            self.processing_times.push(proc_time);

            // Check processing deadline
            if proc_time > PROCESSING_DEADLINE.as_nanos() as u64 {
                self.missed_deadlines += 1;
                self.shared.status_memory.increment_missed_deadlines();
            }

            // Track anomalies
            if processed.is_anomaly {
                self.shared.status_memory.increment_anomalies();
                self.shared.diagnostic_log.try_log(
                    LogLevel::Warning,
                    "Sensor",
                    &format!(
                        "Anomaly detected on sensor {}: {:.2}",
                        processed.sensor_id, processed.filtered_value
                    ),
                );
            }

            if !processed.is_anomaly && processed.confidence >= 0.5 {
                self.last_good_data[processed.sensor_id] = Some(processed.clone());
            }

            processed_data.push(processed);
        }

        // Phase 3: Transmit data with fault injection
        let mut sent_data = Vec::with_capacity(processed_data.len());
        for data in processed_data {
            let sensor_id = data.sensor_id;
            let sequence = data.sequence;
            let original_value = data.filtered_value;
            let tx_start = Instant::now();
            let maybe_faulted = fault_injector.apply_fault(data);
            let tx_time = tx_start.elapsed().as_nanos() as u64;
            self.transmission_times.push(tx_time);

            if tx_time > TRANSMISSION_DEADLINE.as_nanos() as u64 {
                self.missed_deadlines += 1;
            }

            if let Some((mut faulty_data, record)) = maybe_faulted {
                if matches!(record.fault_type, FaultType::Noise) {
                    faulty_data.is_anomaly = true;
                    faulty_data.confidence = faulty_data.confidence.min(0.3);
                }

                if matches!(record.fault_type, FaultType::Corruption | FaultType::Noise) {
                    if let Some(last_good) = self.last_good_data[sensor_id].clone() {
                        faulty_data = Self::build_recovery_data(last_good, sequence);
                        self.recovery_count += 1;
                        self.shared.diagnostic_log.try_log(
                            LogLevel::Warning,
                            "Sensor",
                            &format!(
                                "Recovered from {:?} on sensor {} using last good reading",
                                record.fault_type, sensor_id
                            ),
                        );
                    }
                }

                self.data_sender
                    .try_send(faulty_data.clone())
                    .map_err(|e| format!("Transmission failed: {}", e))?;

                if record.fault_type != FaultType::None {
                    self.shared.diagnostic_log.try_log(
                        LogLevel::Warning,
                        "Sensor",
                        &format!(
                            "Injected {:?} fault on sensor {}",
                            record.fault_type, record.sensor_id
                        ),
                    );
                }

                if matches!(record.fault_type, FaultType::None | FaultType::Delay) {
                    self.last_good_data[sensor_id] = Some(faulty_data.clone());
                }

                sent_data.push((faulty_data, record));
            } else if let Some(last_good) = self.last_good_data[sensor_id].clone() {
                let recovery = Self::build_recovery_data(last_good, sequence);
                let record = FaultRecord {
                    fault_type: FaultType::Dropout,
                    timestamp_ns: recovery.timestamp_ns,
                    sensor_id,
                    original_value: Some(original_value),
                    corrupted_value: None,
                    delay: None,
                };
                self.data_sender
                    .try_send(recovery.clone())
                    .map_err(|e| format!("Transmission failed: {}", e))?;
                self.recovery_count += 1;
                self.shared.diagnostic_log.try_log(
                    LogLevel::Warning,
                    "Sensor",
                    &format!(
                        "Dropout detected on sensor {}, using last good reading",
                        sensor_id
                    ),
                );
                sent_data.push((recovery, record));
            } else {
                self.shared.diagnostic_log.try_log(
                    LogLevel::Warning,
                    "Sensor",
                    &format!("Dropped packet from sensor {} (no recovery available)", sensor_id),
                );
            }
        }

        // Phase 4: Process any feedback
        self.process_feedback();

        // Update cycle counter
        self.total_cycles += 1;
        self.shared.status_memory.increment_cycles();

        // Log to shared diagnostic (non-blocking)
        if self.total_cycles % 100 == 0 {
            self.shared.diagnostic_log.try_log(
                LogLevel::Info,
                "Sensor",
                &format!("Completed {} cycles", self.total_cycles),
            );
        }

        Ok(sent_data)
    }

    /// Process feedback from actuator module
    fn process_feedback(&mut self) {
        while let Ok(feedback) = self.feedback_receiver.try_recv() {
            // Apply calibration adjustments if suggested
            if let Some(adjustment) = feedback.calibration_adjustment {
                let sensor_id = feedback.actuator_id % NUM_SENSOR_TYPES;
                self.processor.set_calibration_offset(sensor_id, adjustment);
                
                self.shared.diagnostic_log.try_log(
                    LogLevel::Info,
                    "Sensor",
                    &format!("Applied calibration adjustment {:.3} to sensor {}", 
                        adjustment, sensor_id),
                );
            }

            // Handle error messages
            if let Some(ref error) = feedback.error_message {
                self.shared.diagnostic_log.try_log(
                    LogLevel::Warning,
                    "Sensor",
                    &format!("Actuator error: {}", error),
                );
            }
        }
    }

    /// Run the sensor module continuously
    pub fn run(&mut self) {
        self.shared.diagnostic_log.log(
            LogLevel::Info,
            "Sensor",
            "Sensor module started",
        );

        let mut last_sample = Instant::now();
        
        while self.running.load(Ordering::Relaxed) {
            // Wait for next sampling interval
            let elapsed = last_sample.elapsed();
            if elapsed < SENSOR_SAMPLE_INTERVAL {
                std::thread::sleep(SENSOR_SAMPLE_INTERVAL - elapsed);
            }
            last_sample = Instant::now();

            // Run one cycle
            if let Err(e) = self.run_cycle() {
                self.shared.diagnostic_log.log(
                    LogLevel::Error,
                    "Sensor",
                    &format!("Cycle error: {}", e),
                );
            }

            // Check for shutdown
            if self.shared.status_memory.is_emergency_stop() {
                break;
            }
        }

        self.shared.diagnostic_log.log(
            LogLevel::Info,
            "Sensor",
            "Sensor module stopped",
        );
    }

    /// Get performance statistics
    pub fn get_stats(&self) -> SensorStats {
        SensorStats {
            generation: PerformanceStats::from_measurements(
                &self.generation_times,
                0,
                self.total_cycles as f64 * SENSOR_SAMPLE_INTERVAL.as_secs_f64(),
            ),
            processing: PerformanceStats::from_measurements(
                &self.processing_times,
                self.missed_deadlines,
                self.total_cycles as f64 * SENSOR_SAMPLE_INTERVAL.as_secs_f64(),
            ),
            transmission: PerformanceStats::from_measurements(
                &self.transmission_times,
                0,
                self.total_cycles as f64 * SENSOR_SAMPLE_INTERVAL.as_secs_f64(),
            ),
            total_cycles: self.total_cycles,
            missed_deadlines: self.missed_deadlines,
        }
    }

    /// Get number of recovery actions performed during fault injection
    pub fn get_recovery_count(&self) -> usize {
        self.recovery_count
    }

    /// Print performance summary
    pub fn print_stats(&self) {
        let stats = self.get_stats();
        stats.generation.print_summary("Sensor Generation");
        stats.processing.print_summary("Sensor Processing");
        stats.transmission.print_summary("Sensor Transmission");
        println!("\n  Total Sensor Cycles: {}", stats.total_cycles);
        println!("  Total Missed Deadlines: {}", stats.missed_deadlines);
    }
}

impl SensorModule {
    fn build_recovery_data(
        mut last_good: ProcessedSensorData,
        sequence: u64,
    ) -> ProcessedSensorData {
        last_good.sequence = sequence;
        last_good.timestamp_ns = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        last_good.is_anomaly = true;
        last_good.confidence = last_good.confidence.min(0.2);
        last_good
    }
}

/// Statistics from sensor module
#[derive(Debug, Clone)]
pub struct SensorStats {
    pub generation: PerformanceStats,
    pub processing: PerformanceStats,
    pub transmission: PerformanceStats,
    pub total_cycles: u64,
    pub missed_deadlines: usize,
}
