// ============================================================================
// Actuator Module (Component B)
// ============================================================================
// Implements the actuator commander with:
// - Efficient sensor data reception
// - PID-based predictive control
// - Multiple actuator management
// - Feedback loop to sensor module
// ============================================================================

use crate::config::*;
use crate::ipc::*;
use crate::pid_controller::*;
use crate::shared_resource::*;
use crate::types::*;
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

// ----------------------------------------------------------------------------
// Virtual Actuator
// ----------------------------------------------------------------------------

/// Simulates a physical actuator with realistic dynamics
pub struct VirtualActuator {
    /// Actuator ID
    id: usize,
    /// Actuator name
    name: String,
    /// Current position/value
    current_value: f64,
    /// Target value
    target_value: f64,
    /// Maximum rate of change per cycle
    max_rate: f64,
    /// Is actuator enabled?
    enabled: bool,
    /// Last command received
    last_command: Option<ActuatorCommand>,
    /// Execution count
    execution_count: u64,
}

impl VirtualActuator {
    /// Create a new virtual actuator
    pub fn new(id: usize, name: &str) -> Self {
        // Configure max rate based on actuator type
        let max_rate = match id {
            ACTUATOR_GRIPPER => 10.0,    // Fast gripper
            ACTUATOR_MOTOR => 5.0,       // Moderate motor speed
            ACTUATOR_STABILIZER => 15.0, // Fast stabilizer
            _ => 8.0,
        };

        Self {
            id,
            name: name.to_string(),
            current_value: 0.0,
            target_value: 0.0,
            max_rate,
            enabled: true,
            last_command: None,
            execution_count: 0,
        }
    }

    /// Apply a command to the actuator
    pub fn apply_command(&mut self, command: ActuatorCommand) -> ActuatorState {
        self.last_command = Some(command.clone());
        self.target_value = command.output_value;
        self.execution_count += 1;

        // Simulate actuator dynamics (rate-limited movement)
        let error = self.target_value - self.current_value;
        let change = error.clamp(-self.max_rate, self.max_rate);
        self.current_value += change;

        self.get_state()
    }

    /// Get current actuator state
    pub fn get_state(&self) -> ActuatorState {
        ActuatorState {
            actuator_id: self.id,
            current_value: self.current_value,
            target_value: self.target_value,
            error: self.target_value - self.current_value,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos(),
        }
    }

    /// Enable/disable actuator
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    /// Check if actuator is enabled
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Get actuator ID
    pub fn get_id(&self) -> usize {
        self.id
    }

    /// Get actuator name
    pub fn get_name(&self) -> &str {
        &self.name
    }
}

// ----------------------------------------------------------------------------
// Actuator Module (Complete Component B)
// ----------------------------------------------------------------------------

/// Complete actuator module with reception, control, and feedback
pub struct ActuatorModule {
    /// Virtual actuators
    actuators: Vec<Arc<Mutex<VirtualActuator>>>,
    /// PID controllers for each actuator
    controllers: PidControllerBank,
    /// Data receiver channel
    data_receiver: SensorDataReceiver,
    /// Feedback sender channel
    feedback_sender: FeedbackSender,
    /// Shared resources
    shared: SharedResources,
    /// Running flag
    running: Arc<AtomicBool>,
    /// Setpoints for each actuator (mapped from sensors)
    setpoints: Vec<f64>,
    /// Performance metrics
    reception_times: Vec<u64>,
    control_times: Vec<u64>,
    feedback_times: Vec<u64>,
    missed_deadlines: usize,
    total_cycles: u64,
    /// Last received sequence per sensor
    last_sequences: HashMap<usize, u64>,
}

impl ActuatorModule {
    /// Create a new actuator module
    pub fn new(
        data_receiver: SensorDataReceiver,
        feedback_sender: FeedbackSender,
        shared: SharedResources,
        running: Arc<AtomicBool>,
    ) -> Self {
        // Create actuators for each type
        let actuators: Vec<Arc<Mutex<VirtualActuator>>> = (0..NUM_ACTUATORS)
            .map(|i| Arc::new(Mutex::new(VirtualActuator::new(i, ACTUATOR_NAMES[i]))))
            .collect();

        // Initialize setpoints to reasonable defaults
        let setpoints = vec![50.0, 100.0, 25.0]; // Force, Position, Temperature targets

        Self {
            actuators,
            controllers: PidControllerBank::new(),
            data_receiver,
            feedback_sender,
            shared,
            running,
            setpoints,
            reception_times: Vec::new(),
            control_times: Vec::new(),
            feedback_times: Vec::new(),
            missed_deadlines: 0,
            total_cycles: 0,
            last_sequences: HashMap::new(),
        }
    }

    /// Run one cycle of actuator operations
    pub fn run_cycle(&mut self) -> Result<Vec<ActuatorFeedback>, String> {
        let _cycle_start = Instant::now();
        let mut feedbacks = Vec::new();

        // Phase 1: Receive sensor data
        let rx_start = Instant::now();
        let mut sensor_data = self.receive_data()?;
        let rx_time = rx_start.elapsed().as_nanos() as u64;
        self.reception_times.push(rx_time);

        // Check if we received any data
        if sensor_data.is_empty() {
            return Ok(feedbacks);
        }

        // Prioritize anomalies first, then by sequence for stable ordering.
        sensor_data.sort_by(|a, b| {
            let pa = if a.is_anomaly { 2 } else { 1 };
            let pb = if b.is_anomaly { 2 } else { 1 };
            pb.cmp(&pa).then_with(|| a.sequence.cmp(&b.sequence))
        });

        // Phase 2: Control each actuator concurrently based on sensor data
        let mut grouped: Vec<Vec<ProcessedSensorData>> = vec![Vec::new(); NUM_ACTUATORS];
        for data in sensor_data {
            let actuator_id = data.sensor_id % NUM_ACTUATORS;
            grouped[actuator_id].push(data);
        }

        let feedback_sender = self.feedback_sender.clone();
        let setpoints = self.setpoints.clone();
        let mut handles = Vec::new();

        for (actuator_id, data_group) in grouped.into_iter().enumerate() {
            if data_group.is_empty() {
                continue;
            }

            let actuator = Arc::clone(&self.actuators[actuator_id]);
            let controller = match self.controllers.get_controller(actuator_id) {
                Some(controller) => controller,
                None => continue,
            };
            let feedback_sender = feedback_sender.clone();
            let setpoint = setpoints[actuator_id];

            handles.push(thread::spawn(move || {
                let mut local_feedbacks = Vec::new();
                let mut control_times = Vec::new();
                let mut feedback_times = Vec::new();
                let mut missed_deadlines = 0;

                for data in data_group {
                    let ctrl_start = Instant::now();

                    let (output, error, _dt) = {
                        let mut controller = controller.lock().unwrap();
                        controller.set_setpoint(setpoint);
                        controller.update(data.filtered_value)
                    };

                    let priority = if data.is_anomaly { 2 } else { 1 };
                    let command = ActuatorCommand::new(
                        actuator_id,
                        ACTUATOR_NAMES[actuator_id].to_string(),
                        output,
                        priority,
                        data.sensor_id,
                        data.sequence,
                    );

                    let state = {
                        let mut actuator = actuator.lock().unwrap();
                        actuator.apply_command(command)
                    };

                    let ctrl_time = ctrl_start.elapsed().as_nanos() as u64;
                    control_times.push(ctrl_time);
                    if ctrl_time > ACTUATOR_DEADLINE.as_nanos() as u64 {
                        missed_deadlines += 1;
                    }

                    let fb_start = Instant::now();
                    let mut feedback = ActuatorFeedback::new(actuator_id, data.sequence, state);
                    feedback.response_time_ns = ctrl_time;
                    if error.abs() > 10.0 {
                        feedback.calibration_adjustment = Some(-error * 0.1);
                    }
                    if let Err(e) = feedback_sender.try_send(feedback.clone()) {
                        feedback.error_message = Some(format!("Feedback send failed: {}", e));
                    }

                    let fb_time = fb_start.elapsed().as_nanos() as u64;
                    feedback_times.push(fb_time);
                    if fb_time > FEEDBACK_DEADLINE.as_nanos() as u64 {
                        missed_deadlines += 1;
                    }

                    local_feedbacks.push(feedback);
                }

                (local_feedbacks, control_times, feedback_times, missed_deadlines)
            }));
        }

        for handle in handles {
            let (local_feedbacks, control_times, feedback_times, missed) = handle
                .join()
                .map_err(|_| "Actuator worker panicked".to_string())?;
            self.control_times.extend(control_times);
            self.feedback_times.extend(feedback_times);
            self.missed_deadlines += missed;
            feedbacks.extend(local_feedbacks);
        }

        // Update cycle counter
        self.total_cycles += 1;

        if self.total_cycles % 100 == 0 {
            let gain_value = if (self.total_cycles / 100) % 2 == 0 { 1.0 } else { 0.95 };
            self.shared.config_buffer.update(|config| {
                config.mode = SystemMode::Normal;
                config.anomaly_threshold = ANOMALY_THRESHOLD;
                for gain in &mut config.actuator_gains {
                    *gain = gain_value;
                }
            });
        }

        // Periodic logging
        if self.total_cycles % 100 == 0 {
            self.shared.diagnostic_log.try_log(
                LogLevel::Info,
                "Actuator",
                &format!("Completed {} cycles, mode: {:?}", self.total_cycles, SystemMode::Normal),
            );
        }

        Ok(feedbacks)
    }

    /// Receive sensor data from channel
    fn receive_data(&mut self) -> Result<Vec<ProcessedSensorData>, String> {
        let mut data = Vec::new();
        let timeout = Duration::from_millis(10);
        
        // Receive all available data (non-blocking after first)
        match self.data_receiver.recv_timeout(timeout) {
            Ok(first) => {
                // Check for dropped packets
                let last_seq = self.last_sequences.get(&first.sensor_id).copied().unwrap_or(0);
                if first.sequence > last_seq + 1 && last_seq > 0 {
                    self.shared.diagnostic_log.try_log(
                        LogLevel::Warning,
                        "Actuator",
                        &format!("Dropped {} packets from sensor {}", 
                            first.sequence - last_seq - 1, first.sensor_id),
                    );
                }
                self.last_sequences.insert(first.sensor_id, first.sequence);
                data.push(first);

                // Get any additional pending data
                while let Ok(more) = self.data_receiver.try_recv() {
                    self.last_sequences.insert(more.sensor_id, more.sequence);
                    data.push(more);
                }
            }
            Err(_) => {
                // No data available, not necessarily an error
            }
        }

        Ok(data)
    }

    /// Run the actuator module continuously
    pub fn run(&mut self) {
        self.shared.diagnostic_log.log(
            LogLevel::Info,
            "Actuator",
            "Actuator module started",
        );

        while self.running.load(Ordering::Relaxed) {
            // Run one cycle
            if let Err(e) = self.run_cycle() {
                self.shared.diagnostic_log.log(
                    LogLevel::Error,
                    "Actuator",
                    &format!("Cycle error: {}", e),
                );
            }

            // Check for emergency stop
            if self.shared.status_memory.is_emergency_stop() {
                for actuator in &self.actuators {
                    actuator.lock().unwrap().set_enabled(false);
                }
                break;
            }

            // Small sleep to prevent busy-waiting
            std::thread::sleep(Duration::from_micros(100));
        }

        self.shared.diagnostic_log.log(
            LogLevel::Info,
            "Actuator",
            "Actuator module stopped",
        );
    }

    /// Set setpoint for an actuator
    pub fn set_setpoint(&mut self, actuator_id: usize, setpoint: f64) {
        if actuator_id < self.setpoints.len() {
            self.setpoints[actuator_id] = setpoint;
            if let Some(controller) = self.controllers.get_controller(actuator_id) {
                controller.lock().unwrap().set_setpoint(setpoint);
            }
        }
    }

    /// Get all actuator states
    pub fn get_actuator_states(&self) -> Vec<ActuatorState> {
        self.actuators
            .iter()
            .map(|a| a.lock().unwrap().get_state())
            .collect()
    }

    /// Get performance statistics
    pub fn get_stats(&self) -> ActuatorStats {
        let total_time = self.total_cycles as f64 * SENSOR_SAMPLE_INTERVAL.as_secs_f64();
        
        ActuatorStats {
            reception: PerformanceStats::from_measurements(
                &self.reception_times,
                0,
                total_time,
            ),
            control: PerformanceStats::from_measurements(
                &self.control_times,
                self.missed_deadlines,
                total_time,
            ),
            feedback: PerformanceStats::from_measurements(
                &self.feedback_times,
                0,
                total_time,
            ),
            total_cycles: self.total_cycles,
            missed_deadlines: self.missed_deadlines,
        }
    }

    /// Print performance summary
    pub fn print_stats(&self) {
        let stats = self.get_stats();
        stats.reception.print_summary("Actuator Reception");
        stats.control.print_summary("Actuator Control");
        stats.feedback.print_summary("Actuator Feedback");
        println!("\n  Total Actuator Cycles: {}", stats.total_cycles);
        println!("  Total Missed Deadlines: {}", stats.missed_deadlines);
    }
}

/// Statistics from actuator module
#[derive(Debug, Clone)]
pub struct ActuatorStats {
    pub reception: PerformanceStats,
    pub control: PerformanceStats,
    pub feedback: PerformanceStats,
    pub total_cycles: u64,
    pub missed_deadlines: usize,
}
