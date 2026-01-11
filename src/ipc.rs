// ============================================================================
// IPC Module - Inter-Process Communication
// ============================================================================
// Implements communication channels between sensor and actuator
// modules. Uses std::sync::mpsc channels for message passing.
// ============================================================================

use crate::config::*;
use crate::types::*;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc::{self, Receiver, SyncSender, TryRecvError, TrySendError};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

// ----------------------------------------------------------------------------
// Channel Types
// ----------------------------------------------------------------------------

fn decrement_queue(queue: &AtomicUsize) {
    let _ = queue.fetch_update(Ordering::AcqRel, Ordering::Acquire, |v| v.checked_sub(1));
}

fn make_channel<T>(capacity: usize) -> (SyncSender<T>, Arc<Mutex<Receiver<T>>>, Arc<AtomicUsize>) {
    let (sender, receiver) = mpsc::sync_channel(capacity);
    (
        sender,
        Arc::new(Mutex::new(receiver)),
        Arc::new(AtomicUsize::new(0)),
    )
}

/// Channel for sending processed sensor data from sensor to actuator module
pub struct SensorDataChannel {
    sender: SyncSender<ProcessedSensorData>,
    receiver: Arc<Mutex<Receiver<ProcessedSensorData>>>,
    queued: Arc<AtomicUsize>,
}

impl SensorDataChannel {
    /// Create a new bounded channel with specified capacity
    pub fn new(capacity: usize) -> Self {
        let (sender, receiver, queued) = make_channel(capacity);
        Self {
            sender,
            receiver,
            queued,
        }
    }

    /// Create an unbounded channel (simulated with large bounded capacity)
    pub fn unbounded() -> Self {
        let capacity = CHANNEL_BUFFER_SIZE.saturating_mul(10);
        let (sender, receiver, queued) = make_channel(capacity);
        Self {
            sender,
            receiver,
            queued,
        }
    }

    /// Get a clone of the sender
    pub fn get_sender(&self) -> SensorDataSender {
        SensorDataSender {
            sender: self.sender.clone(),
            queued: Arc::clone(&self.queued),
        }
    }

    /// Get a clone of the receiver
    pub fn get_receiver(&self) -> SensorDataReceiver {
        SensorDataReceiver {
            receiver: Arc::clone(&self.receiver),
            queued: Arc::clone(&self.queued),
        }
    }
}

/// Sender end of the sensor data channel
#[derive(Clone)]
pub struct SensorDataSender {
    sender: SyncSender<ProcessedSensorData>,
    queued: Arc<AtomicUsize>,
}

impl SensorDataSender {
    /// Send sensor data (blocking if channel is full)
    pub fn send(&self, data: ProcessedSensorData) -> Result<(), String> {
        self.sender
            .send(data)
            .map_err(|e| format!("Failed to send sensor data: {}", e))?;
        self.queued.fetch_add(1, Ordering::AcqRel);
        Ok(())
    }

    /// Try to send without blocking
    pub fn try_send(&self, data: ProcessedSensorData) -> Result<(), String> {
        self.sender
            .try_send(data)
            .map_err(|e| format!("Failed to send sensor data: {}", e))?;
        self.queued.fetch_add(1, Ordering::AcqRel);
        Ok(())
    }

    /// Send with timeout
    pub fn send_timeout(&self, data: ProcessedSensorData, timeout: Duration) -> Result<(), String> {
        let start = Instant::now();
        let mut pending = Some(data);

        loop {
            match self.sender.try_send(pending.take().expect("pending data")) {
                Ok(()) => {
                    self.queued.fetch_add(1, Ordering::AcqRel);
                    return Ok(());
                }
                Err(TrySendError::Full(data)) => {
                    if start.elapsed() >= timeout {
                        return Err("Send timeout: channel full".to_string());
                    }
                    pending = Some(data);
                    std::thread::yield_now();
                }
                Err(TrySendError::Disconnected(_)) => {
                    return Err("Failed to send sensor data: channel disconnected".to_string());
                }
            }
        }
    }
}

/// Receiver end of the sensor data channel
#[derive(Clone)]
pub struct SensorDataReceiver {
    receiver: Arc<Mutex<Receiver<ProcessedSensorData>>>,
    queued: Arc<AtomicUsize>,
}

impl SensorDataReceiver {
    /// Receive sensor data (blocking)
    pub fn recv(&self) -> Result<ProcessedSensorData, String> {
        let receiver = self
            .receiver
            .lock()
            .map_err(|_| "Sensor receiver lock poisoned".to_string())?;
        match receiver.recv() {
            Ok(data) => {
                decrement_queue(&self.queued);
                Ok(data)
            }
            Err(e) => Err(format!("Failed to receive sensor data: {}", e)),
        }
    }

    /// Try to receive without blocking
    pub fn try_recv(&self) -> Result<ProcessedSensorData, TryRecvError> {
        let receiver = match self.receiver.lock() {
            Ok(receiver) => receiver,
            Err(_) => return Err(TryRecvError::Disconnected),
        };
        match receiver.try_recv() {
            Ok(data) => {
                decrement_queue(&self.queued);
                Ok(data)
            }
            Err(e) => Err(e),
        }
    }

    /// Receive with timeout
    pub fn recv_timeout(&self, timeout: Duration) -> Result<ProcessedSensorData, String> {
        let receiver = self
            .receiver
            .lock()
            .map_err(|_| "Sensor receiver lock poisoned".to_string())?;
        match receiver.recv_timeout(timeout) {
            Ok(data) => {
                decrement_queue(&self.queued);
                Ok(data)
            }
            Err(e) => Err(format!("Receive timeout: {}", e)),
        }
    }

    /// Check if there are pending messages
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Get number of pending messages
    pub fn len(&self) -> usize {
        self.queued.load(Ordering::Acquire)
    }
}

// ----------------------------------------------------------------------------
// Feedback Channel (Actuator to Sensor)
// ----------------------------------------------------------------------------

/// Channel for sending feedback from actuator back to sensor module
pub struct FeedbackChannel {
    sender: SyncSender<ActuatorFeedback>,
    receiver: Arc<Mutex<Receiver<ActuatorFeedback>>>,
    queued: Arc<AtomicUsize>,
}

impl FeedbackChannel {
    /// Create a new bounded feedback channel
    pub fn new(capacity: usize) -> Self {
        let (sender, receiver, queued) = make_channel(capacity);
        Self {
            sender,
            receiver,
            queued,
        }
    }

    /// Get a clone of the sender
    pub fn get_sender(&self) -> FeedbackSender {
        FeedbackSender {
            sender: self.sender.clone(),
            queued: Arc::clone(&self.queued),
        }
    }

    /// Get a clone of the receiver
    pub fn get_receiver(&self) -> FeedbackReceiver {
        FeedbackReceiver {
            receiver: Arc::clone(&self.receiver),
            queued: Arc::clone(&self.queued),
        }
    }
}

/// Sender end of feedback channel
#[derive(Clone)]
pub struct FeedbackSender {
    sender: SyncSender<ActuatorFeedback>,
    queued: Arc<AtomicUsize>,
}

impl FeedbackSender {
    /// Send feedback (blocking)
    pub fn send(&self, feedback: ActuatorFeedback) -> Result<(), String> {
        self.sender
            .send(feedback)
            .map_err(|e| format!("Failed to send feedback: {}", e))?;
        self.queued.fetch_add(1, Ordering::AcqRel);
        Ok(())
    }

    /// Try to send without blocking
    pub fn try_send(&self, feedback: ActuatorFeedback) -> Result<(), String> {
        self.sender
            .try_send(feedback)
            .map_err(|e| format!("Failed to send feedback: {}", e))?;
        self.queued.fetch_add(1, Ordering::AcqRel);
        Ok(())
    }

    /// Send with timeout (for real-time constraints)
    pub fn send_timeout(&self, feedback: ActuatorFeedback, timeout: Duration) -> Result<(), String> {
        let start = Instant::now();
        let mut pending = Some(feedback);

        loop {
            match self.sender.try_send(pending.take().expect("pending feedback")) {
                Ok(()) => {
                    self.queued.fetch_add(1, Ordering::AcqRel);
                    return Ok(());
                }
                Err(TrySendError::Full(feedback)) => {
                    if start.elapsed() >= timeout {
                        return Err("Feedback send timeout: channel full".to_string());
                    }
                    pending = Some(feedback);
                    std::thread::yield_now();
                }
                Err(TrySendError::Disconnected(_)) => {
                    return Err("Failed to send feedback: channel disconnected".to_string());
                }
            }
        }
    }
}

/// Receiver end of feedback channel
#[derive(Clone)]
pub struct FeedbackReceiver {
    receiver: Arc<Mutex<Receiver<ActuatorFeedback>>>,
    queued: Arc<AtomicUsize>,
}

impl FeedbackReceiver {
    /// Receive feedback (blocking)
    pub fn recv(&self) -> Result<ActuatorFeedback, String> {
        let receiver = self
            .receiver
            .lock()
            .map_err(|_| "Feedback receiver lock poisoned".to_string())?;
        match receiver.recv() {
            Ok(feedback) => {
                decrement_queue(&self.queued);
                Ok(feedback)
            }
            Err(e) => Err(format!("Failed to receive feedback: {}", e)),
        }
    }

    /// Try to receive without blocking
    pub fn try_recv(&self) -> Result<ActuatorFeedback, TryRecvError> {
        let receiver = match self.receiver.lock() {
            Ok(receiver) => receiver,
            Err(_) => return Err(TryRecvError::Disconnected),
        };
        match receiver.try_recv() {
            Ok(feedback) => {
                decrement_queue(&self.queued);
                Ok(feedback)
            }
            Err(e) => Err(e),
        }
    }

    /// Receive with timeout
    pub fn recv_timeout(&self, timeout: Duration) -> Result<ActuatorFeedback, String> {
        let receiver = self
            .receiver
            .lock()
            .map_err(|_| "Feedback receiver lock poisoned".to_string())?;
        match receiver.recv_timeout(timeout) {
            Ok(feedback) => {
                decrement_queue(&self.queued);
                Ok(feedback)
            }
            Err(e) => Err(format!("Feedback receive timeout: {}", e)),
        }
    }

    /// Check if there are pending messages
    pub fn is_empty(&self) -> bool {
        self.queued.load(Ordering::Acquire) == 0
    }
}

// ----------------------------------------------------------------------------
// Command Channel (for control commands)
// ----------------------------------------------------------------------------

/// Commands that can be sent to control the system
#[derive(Debug, Clone)]
pub enum ControlCommand {
    /// Start the system
    Start,
    /// Stop the system gracefully
    Stop,
    /// Update configuration
    UpdateConfig(String, f64),
    /// Request status report
    RequestStatus,
    /// Emergency shutdown
    EmergencyStop,
}

/// Channel for sending control commands
pub struct CommandChannel {
    sender: SyncSender<ControlCommand>,
    receiver: Arc<Mutex<Receiver<ControlCommand>>>,
}

impl CommandChannel {
    pub fn new() -> Self {
        let (sender, receiver) = mpsc::sync_channel(10);
        Self {
            sender,
            receiver: Arc::new(Mutex::new(receiver)),
        }
    }

    pub fn get_sender(&self) -> CommandSender {
        CommandSender {
            sender: self.sender.clone(),
        }
    }

    pub fn get_receiver(&self) -> CommandReceiver {
        CommandReceiver {
            receiver: Arc::clone(&self.receiver),
        }
    }
}

impl Default for CommandChannel {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone)]
pub struct CommandSender {
    sender: SyncSender<ControlCommand>,
}

impl CommandSender {
    pub fn send(&self, cmd: ControlCommand) -> Result<(), String> {
        self.sender
            .send(cmd)
            .map_err(|e| format!("Failed to send command: {}", e))
    }
}

#[derive(Clone)]
pub struct CommandReceiver {
    receiver: Arc<Mutex<Receiver<ControlCommand>>>,
}

impl CommandReceiver {
    pub fn try_recv(&self) -> Option<ControlCommand> {
        let receiver = self.receiver.lock().ok()?;
        receiver.try_recv().ok()
    }
}

// ----------------------------------------------------------------------------
// IPC Manager - Coordinates all channels
// ----------------------------------------------------------------------------

/// Central manager for all IPC channels
pub struct IpcManager {
    /// Sensor data channel
    pub sensor_channel: SensorDataChannel,
    /// Feedback channel
    pub feedback_channel: FeedbackChannel,
    /// Command channel
    pub command_channel: CommandChannel,
    /// Transmission timing statistics
    pub transmission_times: Arc<Mutex<Vec<u64>>>,
}

impl IpcManager {
    /// Create a new IPC manager with all channels
    pub fn new() -> Self {
        Self {
            sensor_channel: SensorDataChannel::new(CHANNEL_BUFFER_SIZE),
            feedback_channel: FeedbackChannel::new(CHANNEL_BUFFER_SIZE),
            command_channel: CommandChannel::new(),
            transmission_times: Arc::new(Mutex::new(Vec::new())),
        }
    }

    /// Get sensor data sender
    pub fn get_sensor_sender(&self) -> SensorDataSender {
        self.sensor_channel.get_sender()
    }

    /// Get sensor data receiver
    pub fn get_sensor_receiver(&self) -> SensorDataReceiver {
        self.sensor_channel.get_receiver()
    }

    /// Get feedback sender
    pub fn get_feedback_sender(&self) -> FeedbackSender {
        self.feedback_channel.get_sender()
    }

    /// Get feedback receiver
    pub fn get_feedback_receiver(&self) -> FeedbackReceiver {
        self.feedback_channel.get_receiver()
    }

    /// Get command sender
    pub fn get_command_sender(&self) -> CommandSender {
        self.command_channel.get_sender()
    }

    /// Get command receiver
    pub fn get_command_receiver(&self) -> CommandReceiver {
        self.command_channel.get_receiver()
    }

    /// Record a transmission time
    pub fn record_transmission_time(&self, time_ns: u64) {
        self.transmission_times.lock().unwrap().push(time_ns);
    }

    /// Get transmission statistics
    pub fn get_transmission_stats(&self) -> PerformanceStats {
        let times = self.transmission_times.lock().unwrap();
        let deadline_ns = TRANSMISSION_DEADLINE.as_nanos() as u64;
        let missed = times.iter().filter(|&&t| t > deadline_ns).count();
        let total_time = times.iter().sum::<u64>() as f64 / 1_000_000_000.0;
        PerformanceStats::from_measurements(&times, missed, total_time)
    }
}

impl Default for IpcManager {
    fn default() -> Self {
        Self::new()
    }
}

// ----------------------------------------------------------------------------
// Timed Send/Receive Wrappers
// ----------------------------------------------------------------------------

/// Wrapper that times how long a send operation takes
pub fn timed_send<T>(
    sender: &SyncSender<T>,
    data: T,
    deadline: Duration,
) -> (Result<(), String>, u64, bool)
where
    T: Send,
{
    let start = Instant::now();
    let result = sender.send(data);
    let elapsed = start.elapsed();
    let elapsed_ns = elapsed.as_nanos() as u64;
    let met_deadline = elapsed <= deadline;

    (
        result.map_err(|e| format!("Send failed: {}", e)),
        elapsed_ns,
        met_deadline,
    )
}

/// Wrapper that times how long a receive operation takes
pub fn timed_recv<T>(receiver: &Receiver<T>, timeout: Duration) -> (Result<T, String>, u64)
where
    T: Send,
{
    let start = Instant::now();
    let result = receiver.recv_timeout(timeout);
    let elapsed = start.elapsed();
    let elapsed_ns = elapsed.as_nanos() as u64;

    (
        result.map_err(|e| format!("Receive failed: {}", e)),
        elapsed_ns,
    )
}
