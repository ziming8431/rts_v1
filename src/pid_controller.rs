// ============================================================================
// PID Controller Module
// ============================================================================
// Implements a Proportional-Integral-Derivative controller for precise
// actuator control. Used to dynamically adjust actuator outputs based on
// sensor feedback to maintain stable operation.
// ============================================================================

use crate::config::*;
use std::time::Instant;

// ----------------------------------------------------------------------------
// PID Controller
// ----------------------------------------------------------------------------

/// A PID (Proportional-Integral-Derivative) controller for real-time control
pub struct PidController {
    /// Proportional gain
    kp: f64,
    /// Integral gain
    ki: f64,
    /// Derivative gain
    kd: f64,
    /// Setpoint (target value)
    setpoint: f64,
    /// Accumulated integral term
    integral: f64,
    /// Previous error (for derivative calculation)
    previous_error: f64,
    /// Last update time
    last_update: Option<Instant>,
    /// Output limits
    output_min: f64,
    output_max: f64,
    /// Integral windup limit
    integral_limit: f64,
    /// Controller name for identification
    name: String,
}

impl PidController {
    /// Create a new PID controller with specified gains
    pub fn new(name: &str, kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            setpoint: 0.0,
            integral: 0.0,
            previous_error: 0.0,
            last_update: None,
            output_min: PID_OUTPUT_MIN,
            output_max: PID_OUTPUT_MAX,
            integral_limit: 50.0, // Prevent integral windup
            name: name.to_string(),
        }
    }

    /// Create a PID controller with default parameters
    pub fn with_defaults(name: &str) -> Self {
        Self::new(name, PID_KP, PID_KI, PID_KD)
    }

    /// Set the target setpoint
    pub fn set_setpoint(&mut self, setpoint: f64) {
        self.setpoint = setpoint;
    }

    /// Get the current setpoint
    pub fn get_setpoint(&self) -> f64 {
        self.setpoint
    }

    /// Update PID gains dynamically
    pub fn set_gains(&mut self, kp: f64, ki: f64, kd: f64) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    /// Set output limits
    pub fn set_output_limits(&mut self, min: f64, max: f64) {
        self.output_min = min;
        self.output_max = max;
    }

    /// Calculate the control output based on current measurement
    /// Returns (output, error, dt)
    pub fn update(&mut self, measurement: f64) -> (f64, f64, f64) {
        let now = Instant::now();
        
        // Calculate time delta
        let dt = match self.last_update {
            Some(last) => {
                let elapsed = now.duration_since(last);
                elapsed.as_secs_f64()
            }
            None => 0.001, // Default to 1ms for first update
        };
        self.last_update = Some(now);

        // Calculate error
        let error = self.setpoint - measurement;

        // Proportional term
        let p_term = self.kp * error;

        // Integral term (with anti-windup)
        self.integral += error * dt;
        // Clamp integral to prevent windup
        self.integral = self.integral.clamp(-self.integral_limit, self.integral_limit);
        let i_term = self.ki * self.integral;

        // Derivative term (based on error change)
        let derivative = if dt > 0.0 {
            (error - self.previous_error) / dt
        } else {
            0.0
        };
        let d_term = self.kd * derivative;
        self.previous_error = error;

        // Calculate total output
        let output = p_term + i_term + d_term;

        // Clamp output to limits
        let clamped_output = output.clamp(self.output_min, self.output_max);

        (clamped_output, error, dt)
    }

    /// Calculate output with explicit time delta (for testing/simulation)
    pub fn update_with_dt(&mut self, measurement: f64, dt: f64) -> (f64, f64) {
        // Calculate error
        let error = self.setpoint - measurement;

        // Proportional term
        let p_term = self.kp * error;

        // Integral term (with anti-windup)
        self.integral += error * dt;
        self.integral = self.integral.clamp(-self.integral_limit, self.integral_limit);
        let i_term = self.ki * self.integral;

        // Derivative term
        let derivative = if dt > 0.0 {
            (error - self.previous_error) / dt
        } else {
            0.0
        };
        let d_term = self.kd * derivative;
        self.previous_error = error;

        // Calculate and clamp output
        let output = p_term + i_term + d_term;
        let clamped_output = output.clamp(self.output_min, self.output_max);

        (clamped_output, error)
    }

    /// Reset the controller state
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.previous_error = 0.0;
        self.last_update = None;
    }

    /// Get controller name
    pub fn get_name(&self) -> &str {
        &self.name
    }

    /// Get current gains
    pub fn get_gains(&self) -> (f64, f64, f64) {
        (self.kp, self.ki, self.kd)
    }

    /// Get accumulated integral value
    pub fn get_integral(&self) -> f64 {
        self.integral
    }
}

// ----------------------------------------------------------------------------
// PID Controller Bank (for multiple actuators)
// ----------------------------------------------------------------------------

/// Manages multiple PID controllers for different actuators
pub struct PidControllerBank {
    controllers: Vec<PidController>,
}

impl PidControllerBank {
    /// Create a new bank with controllers for each actuator
    pub fn new() -> Self {
        let mut controllers = Vec::with_capacity(NUM_ACTUATORS);
        
        for i in 0..NUM_ACTUATORS {
            let name = ACTUATOR_NAMES[i];
            let mut controller = PidController::with_defaults(name);
            
            // Set different gains for different actuator types
            // (in a real system, these would be tuned per actuator)
            match i {
                ACTUATOR_GRIPPER => {
                    // Gripper needs fast response
                    controller.set_gains(0.8, 0.15, 0.08);
                    controller.set_output_limits(-50.0, 50.0);
                }
                ACTUATOR_MOTOR => {
                    // Motor needs smooth control
                    controller.set_gains(0.4, 0.1, 0.05);
                    controller.set_output_limits(-100.0, 100.0);
                }
                ACTUATOR_STABILIZER => {
                    // Stabilizer needs high precision
                    controller.set_gains(0.6, 0.12, 0.06);
                    controller.set_output_limits(-30.0, 30.0);
                }
                _ => {}
            }
            
            controllers.push(controller);
        }
        
        Self { controllers }
    }

    /// Get a mutable reference to a specific controller
    pub fn get_controller(&mut self, index: usize) -> Option<&mut PidController> {
        self.controllers.get_mut(index)
    }

    /// Update all controllers with new measurements
    /// measurements[i] corresponds to controller[i]
    pub fn update_all(&mut self, measurements: &[f64]) -> Vec<(f64, f64)> {
        self.controllers
            .iter_mut()
            .zip(measurements.iter())
            .map(|(controller, &measurement)| {
                let (output, error, _dt) = controller.update(measurement);
                (output, error)
            })
            .collect()
    }

    /// Set setpoints for all controllers
    pub fn set_setpoints(&mut self, setpoints: &[f64]) {
        for (controller, &setpoint) in self.controllers.iter_mut().zip(setpoints.iter()) {
            controller.set_setpoint(setpoint);
        }
    }

    /// Reset all controllers
    pub fn reset_all(&mut self) {
        for controller in &mut self.controllers {
            controller.reset();
        }
    }

    /// Get all controller names
    pub fn get_names(&self) -> Vec<&str> {
        self.controllers.iter().map(|c| c.get_name()).collect()
    }

    /// Update PID gains from configuration
    pub fn update_gains_from_config(&mut self, kp: f64, ki: f64, kd: f64) {
        for controller in &mut self.controllers {
            controller.set_gains(kp, ki, kd);
        }
    }
}

impl Default for PidControllerBank {
    fn default() -> Self {
        Self::new()
    }
}

// ----------------------------------------------------------------------------
// Predictive Control Extension
// ----------------------------------------------------------------------------

/// A simple predictive controller that extends PID with prediction
pub struct PredictiveController {
    /// Base PID controller
    pid: PidController,
    /// History of measurements for prediction
    measurement_history: Vec<f64>,
    /// Maximum history size
    history_size: usize,
    /// Prediction horizon (number of steps ahead)
    prediction_horizon: usize,
}

impl PredictiveController {
    /// Create a new predictive controller
    pub fn new(name: &str, kp: f64, ki: f64, kd: f64, horizon: usize) -> Self {
        Self {
            pid: PidController::new(name, kp, ki, kd),
            measurement_history: Vec::with_capacity(10),
            history_size: 10,
            prediction_horizon: horizon,
        }
    }

    /// Set the target setpoint
    pub fn set_setpoint(&mut self, setpoint: f64) {
        self.pid.set_setpoint(setpoint);
    }

    /// Update with prediction-based control
    pub fn update(&mut self, measurement: f64) -> (f64, f64, f64) {
        // Add to history
        self.measurement_history.push(measurement);
        if self.measurement_history.len() > self.history_size {
            self.measurement_history.remove(0);
        }

        // Predict future value using simple linear extrapolation
        let predicted = self.predict_value();
        
        // Use predicted value for control if we have enough history
        let control_value = if self.measurement_history.len() >= 3 {
            // Blend current and predicted values
            measurement * 0.7 + predicted * 0.3
        } else {
            measurement
        };

        self.pid.update(control_value)
    }

    /// Predict future value using linear regression
    fn predict_value(&self) -> f64 {
        let n = self.measurement_history.len();
        if n < 2 {
            return *self.measurement_history.last().unwrap_or(&0.0);
        }

        // Simple linear regression
        let x_mean = (n - 1) as f64 / 2.0;
        let y_mean: f64 = self.measurement_history.iter().sum::<f64>() / n as f64;

        let mut numerator = 0.0;
        let mut denominator = 0.0;

        for (i, &y) in self.measurement_history.iter().enumerate() {
            let x = i as f64;
            numerator += (x - x_mean) * (y - y_mean);
            denominator += (x - x_mean).powi(2);
        }

        if denominator.abs() < 1e-10 {
            return y_mean;
        }

        let slope = numerator / denominator;
        let intercept = y_mean - slope * x_mean;

        // Predict value at future timestep
        let future_x = (n + self.prediction_horizon) as f64;
        slope * future_x + intercept
    }

    /// Reset the controller
    pub fn reset(&mut self) {
        self.pid.reset();
        self.measurement_history.clear();
    }

    /// Get the underlying PID controller
    pub fn get_pid(&mut self) -> &mut PidController {
        &mut self.pid
    }
}
