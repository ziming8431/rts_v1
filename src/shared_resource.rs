// ============================================================================
// Shared Resource Module
// ============================================================================
// Implements shared resources that both sensor and actuator modules access.
// Uses proper synchronization to prevent data races and corruption.
// Demonstrates mutex usage, atomic operations, and lock-free structures.
// ============================================================================

use crate::config::*;
use crate::types::*;
use std::sync::{Mutex, RwLock};
use std::collections::VecDeque;
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::sync::Arc;

// ----------------------------------------------------------------------------
// Diagnostic Log Buffer (Shared Resource with Mutex)
// ----------------------------------------------------------------------------

/// A thread-safe diagnostic log buffer that both sensor and actuator
/// modules can write to. Uses a mutex for exclusive access.
pub struct DiagnosticLog {
    /// The log entries stored in a circular buffer
    entries: Mutex<VecDeque<DiagnosticEntry>>,
    /// Maximum number of entries to keep
    max_entries: usize,
    /// Total number of entries written (for statistics)
    total_writes: AtomicU64,
    /// Number of times lock was contended
    contention_count: AtomicU64,
}

impl DiagnosticLog {
    /// Create a new diagnostic log with specified capacity
    pub fn new(max_entries: usize) -> Self {
        Self {
            entries: Mutex::new(VecDeque::with_capacity(max_entries)),
            max_entries,
            total_writes: AtomicU64::new(0),
            contention_count: AtomicU64::new(0),
        }
    }

    /// Add a new log entry (blocking if lock is held)
    pub fn log(&self, level: LogLevel, source: &str, message: &str) {
        let entry = DiagnosticEntry::new(level, source, message);
        
        // Try to acquire lock - track contention
        let mut entries = self.entries.lock().unwrap();
        
        // Add entry, removing oldest if at capacity
        if entries.len() >= self.max_entries {
            entries.pop_front();
        }
        entries.push_back(entry);
        
        self.total_writes.fetch_add(1, Ordering::Relaxed);
    }

    /// Try to add a log entry with timeout (returns false if couldn't acquire lock)
    pub fn try_log(&self, level: LogLevel, source: &str, message: &str) -> bool {
        let entry = DiagnosticEntry::new(level, source, message);
        
        // Try to acquire lock without blocking
        if let Ok(mut entries) = self.entries.try_lock() {
            if entries.len() >= self.max_entries {
                entries.pop_front();
            }
            entries.push_back(entry);
            self.total_writes.fetch_add(1, Ordering::Relaxed);
            true
        } else {
            // Lock was contended
            self.contention_count.fetch_add(1, Ordering::Relaxed);
            false
        }
    }

    /// Get recent log entries (for reading)
    pub fn get_recent(&self, count: usize) -> Vec<DiagnosticEntry> {
        let entries = self.entries.lock().unwrap();
        entries.iter().rev().take(count).cloned().collect()
    }

    /// Get contention statistics
    pub fn get_stats(&self) -> (u64, u64) {
        (
            self.total_writes.load(Ordering::Relaxed),
            self.contention_count.load(Ordering::Relaxed),
        )
    }
}

// ----------------------------------------------------------------------------
// Configuration Buffer (Shared Resource with RwLock)
// ----------------------------------------------------------------------------

/// Runtime configuration that can be read frequently and updated occasionally.
/// Uses RwLock to allow multiple simultaneous readers.
pub struct ConfigBuffer {
    /// Current configuration values
    config: RwLock<SystemConfig>,
    /// Version number for change detection
    version: AtomicU64,
    /// Read count for statistics
    read_count: AtomicU64,
    /// Write count for statistics
    write_count: AtomicU64,
}

/// Dynamic system configuration parameters
#[derive(Debug, Clone)]
pub struct SystemConfig {
    /// Sensor calibration offsets
    pub sensor_offsets: [f64; NUM_SENSOR_TYPES],
    /// Actuator gain multipliers
    pub actuator_gains: [f64; NUM_ACTUATORS],
    /// Anomaly detection threshold
    pub anomaly_threshold: f64,
    /// System operating mode
    pub mode: SystemMode,
    /// PID parameters (Kp, Ki, Kd)
    pub pid_params: (f64, f64, f64),
}

impl Default for SystemConfig {
    fn default() -> Self {
        Self {
            sensor_offsets: [0.0; NUM_SENSOR_TYPES],
            actuator_gains: [1.0; NUM_ACTUATORS],
            anomaly_threshold: ANOMALY_THRESHOLD,
            mode: SystemMode::Normal,
            pid_params: (PID_KP, PID_KI, PID_KD),
        }
    }
}

impl ConfigBuffer {
    /// Create a new configuration buffer with default values
    pub fn new() -> Self {
        Self {
            config: RwLock::new(SystemConfig::default()),
            version: AtomicU64::new(0),
            read_count: AtomicU64::new(0),
            write_count: AtomicU64::new(0),
        }
    }

    /// Read the current configuration (multiple readers allowed)
    pub fn read(&self) -> SystemConfig {
        self.read_count.fetch_add(1, Ordering::Relaxed);
        self.config.read().unwrap().clone()
    }

    /// Update the configuration (exclusive access required)
    pub fn update<F>(&self, updater: F)
    where
        F: FnOnce(&mut SystemConfig),
    {
        let mut config = self.config.write().unwrap();
        updater(&mut config);
        self.version.fetch_add(1, Ordering::Release);
        self.write_count.fetch_add(1, Ordering::Relaxed);
    }

    /// Get the current configuration version
    pub fn get_version(&self) -> u64 {
        self.version.load(Ordering::Acquire)
    }

    /// Get read/write statistics
    pub fn get_stats(&self) -> (u64, u64) {
        (
            self.read_count.load(Ordering::Relaxed),
            self.write_count.load(Ordering::Relaxed),
        )
    }
}

impl Default for ConfigBuffer {
    fn default() -> Self {
        Self::new()
    }
}

// ----------------------------------------------------------------------------
// Status Memory (Lock-Free Atomic Status)
// ----------------------------------------------------------------------------

/// Lock-free status indicators using atomic operations.
/// Provides the fastest possible access for critical status checks.
pub struct StatusMemory {
    /// System mode encoded as integer
    mode: AtomicUsize,
    /// Number of consecutive missed deadlines
    missed_deadline_count: AtomicUsize,
    /// Number of anomalies detected
    anomaly_count: AtomicUsize,
    /// Total cycles completed
    cycle_count: AtomicU64,
    /// Emergency stop flag
    emergency_stop: AtomicUsize,
}

impl StatusMemory {
    /// Create new status memory
    pub fn new() -> Self {
        Self {
            mode: AtomicUsize::new(SystemMode::Normal as usize),
            missed_deadline_count: AtomicUsize::new(0),
            anomaly_count: AtomicUsize::new(0),
            cycle_count: AtomicU64::new(0),
            emergency_stop: AtomicUsize::new(0),
        }
    }

    /// Get current system mode (lock-free)
    pub fn get_mode(&self) -> SystemMode {
        match self.mode.load(Ordering::Acquire) {
            0 => SystemMode::Normal,
            _ => SystemMode::Shutdown,
        }
    }

    /// Set system mode (lock-free)
    pub fn set_mode(&self, mode: SystemMode) {
        self.mode.store(mode as usize, Ordering::Release);
    }

    /// Increment missed deadline counter (lock-free)
    pub fn increment_missed_deadlines(&self) -> usize {
        self.missed_deadline_count.fetch_add(1, Ordering::AcqRel) + 1
    }

    /// Reset missed deadline counter (lock-free)
    pub fn reset_missed_deadlines(&self) {
        self.missed_deadline_count.store(0, Ordering::Release);
    }

    /// Get missed deadline count (lock-free)
    pub fn get_missed_deadlines(&self) -> usize {
        self.missed_deadline_count.load(Ordering::Acquire)
    }

    /// Increment anomaly counter (lock-free)
    pub fn increment_anomalies(&self) -> usize {
        self.anomaly_count.fetch_add(1, Ordering::AcqRel) + 1
    }

    /// Reset anomaly counter (lock-free)
    pub fn reset_anomalies(&self) {
        self.anomaly_count.store(0, Ordering::Release);
    }

    /// Get anomaly count (lock-free)
    pub fn get_anomalies(&self) -> usize {
        self.anomaly_count.load(Ordering::Acquire)
    }

    /// Increment cycle counter (lock-free)
    pub fn increment_cycles(&self) -> u64 {
        self.cycle_count.fetch_add(1, Ordering::AcqRel) + 1
    }

    /// Get cycle count (lock-free)
    pub fn get_cycles(&self) -> u64 {
        self.cycle_count.load(Ordering::Acquire)
    }

    /// Trigger emergency stop (lock-free)
    pub fn emergency_stop(&self) {
        self.emergency_stop.store(1, Ordering::Release);
        self.mode.store(SystemMode::Shutdown as usize, Ordering::Release);
    }

    /// Check if emergency stop is active (lock-free)
    pub fn is_emergency_stop(&self) -> bool {
        self.emergency_stop.load(Ordering::Acquire) != 0
    }

    /// Clear emergency stop (lock-free)
    pub fn clear_emergency_stop(&self) {
        self.emergency_stop.store(0, Ordering::Release);
    }
}

impl Default for StatusMemory {
    fn default() -> Self {
        Self::new()
    }
}

// ----------------------------------------------------------------------------
// Combined Shared Resources Container
// ----------------------------------------------------------------------------

/// Container holding all shared resources for easy distribution
pub struct SharedResources {
    /// Diagnostic log (mutex-based)
    pub diagnostic_log: Arc<DiagnosticLog>,
    /// Configuration buffer (RwLock-based)
    pub config_buffer: Arc<ConfigBuffer>,
    /// Status memory (atomic/lock-free)
    pub status_memory: Arc<StatusMemory>,
}

impl SharedResources {
    /// Create a new set of shared resources
    pub fn new() -> Self {
        Self {
            diagnostic_log: Arc::new(DiagnosticLog::new(1000)),
            config_buffer: Arc::new(ConfigBuffer::new()),
            status_memory: Arc::new(StatusMemory::new()),
        }
    }

    /// Print synchronization statistics
    pub fn print_sync_stats(&self) {
        println!("\n=== Shared Resource Synchronization Statistics ===");
        
        let (log_writes, log_contention) = self.diagnostic_log.get_stats();
        println!("Diagnostic Log:");
        println!("  Total Writes:     {}", log_writes);
        println!("  Lock Contentions: {}", log_contention);
        if log_writes > 0 {
            println!("  Contention Rate:  {:.2}%", 
                (log_contention as f64 / log_writes as f64) * 100.0);
        }
        
        let (config_reads, config_writes) = self.config_buffer.get_stats();
        println!("Configuration Buffer:");
        println!("  Total Reads:  {}", config_reads);
        println!("  Total Writes: {}", config_writes);
        println!("  Read/Write Ratio: {:.1}", 
            if config_writes > 0 { config_reads as f64 / config_writes as f64 } else { 0.0 });
        
        println!("Status Memory (Lock-Free):");
        println!("  Total Cycles:       {}", self.status_memory.get_cycles());
        println!("  Missed Deadlines:   {}", self.status_memory.get_missed_deadlines());
        println!("  Anomalies Detected: {}", self.status_memory.get_anomalies());
        println!("  Current Mode:       {:?}", self.status_memory.get_mode());
    }
}

impl Default for SharedResources {
    fn default() -> Self {
        Self::new()
    }
}

impl Clone for SharedResources {
    fn clone(&self) -> Self {
        Self {
            diagnostic_log: Arc::clone(&self.diagnostic_log),
            config_buffer: Arc::clone(&self.config_buffer),
            status_memory: Arc::clone(&self.status_memory),
        }
    }
}

// ----------------------------------------------------------------------------
// Lock Contention Benchmark Helper
// ----------------------------------------------------------------------------

/// Measures lock contention under concurrent access
pub struct ContentionBenchmark {
    /// Timing samples for lock acquisition
    pub lock_times_ns: Vec<u64>,
    /// Number of contentions detected
    pub contentions: usize,
}

impl ContentionBenchmark {
    pub fn new() -> Self {
        Self {
            lock_times_ns: Vec::new(),
            contentions: 0,
        }
    }

    /// Record a lock acquisition time
    pub fn record(&mut self, time_ns: u64, was_contended: bool) {
        self.lock_times_ns.push(time_ns);
        if was_contended {
            self.contentions += 1;
        }
    }

    /// Get statistics about lock contention
    pub fn get_stats(&self) -> PerformanceStats {
        let missed = self.contentions;
        let total_time = self.lock_times_ns.iter().sum::<u64>() as f64 / 1_000_000_000.0;
        PerformanceStats::from_measurements(&self.lock_times_ns, missed, total_time)
    }
}

impl Default for ContentionBenchmark {
    fn default() -> Self {
        Self::new()
    }
}
