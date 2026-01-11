// ============================================================================
// Benchmark Module
// ============================================================================
// Implements comprehensive benchmarking for:
// - Individual component performance
// - End-to-end latency
// - Throughput under various loads
// - Comparison between sync and async implementations
// ============================================================================

use crate::config::*;
use std::time::{Duration, Instant};

// ----------------------------------------------------------------------------
// Benchmark Timer
// ----------------------------------------------------------------------------

/// High-precision timer for benchmarking
pub struct BenchmarkTimer {
    /// Start time
    start: Option<Instant>,
    /// Recorded durations in nanoseconds
    durations: Vec<u64>,
    /// Name of what's being benchmarked
    name: String,
}

impl BenchmarkTimer {
    /// Create a new benchmark timer
    pub fn new(name: &str) -> Self {
        Self {
            start: None,
            durations: Vec::with_capacity(BENCHMARK_ITERATIONS),
            name: name.to_string(),
        }
    }

    /// Start timing
    pub fn start(&mut self) {
        self.start = Some(Instant::now());
    }

    /// Stop timing and record duration
    pub fn stop(&mut self) -> u64 {
        if let Some(start) = self.start.take() {
            let duration = start.elapsed().as_nanos() as u64;
            self.durations.push(duration);
            duration
        } else {
            0
        }
    }

    /// Record a precomputed duration
    pub fn record_duration(&mut self, duration_ns: u64) {
        self.durations.push(duration_ns);
    }

    /// Get all recorded durations
    pub fn get_durations(&self) -> &[u64] {
        &self.durations
    }

    /// Calculate statistics
    pub fn get_stats(&self) -> BenchmarkStats {
        if self.durations.is_empty() {
            return BenchmarkStats::default();
        }

        let mut sorted = self.durations.clone();
        sorted.sort_unstable();

        let count = sorted.len();
        let min = sorted[0];
        let max = sorted[count - 1];
        let sum: u64 = sorted.iter().sum();
        let mean = sum as f64 / count as f64;

        // Calculate standard deviation
        let variance: f64 = sorted
            .iter()
            .map(|&x| (x as f64 - mean).powi(2))
            .sum::<f64>()
            / count as f64;
        let std_dev = variance.sqrt();

        // Percentiles
        let p50 = sorted[count / 2];
        let p95 = sorted[(count as f64 * 0.95) as usize];
        let p99 = sorted[(count as f64 * 0.99) as usize];

        BenchmarkStats {
            name: self.name.clone(),
            count,
            min_ns: min,
            max_ns: max,
            mean_ns: mean,
            std_dev_ns: std_dev,
            p50_ns: p50,
            p95_ns: p95,
            p99_ns: p99,
            jitter_ns: max - min,
        }
    }

    /// Clear recorded durations
    pub fn clear(&mut self) {
        self.durations.clear();
    }
}

/// Benchmark statistics
#[derive(Debug, Clone, Default)]
pub struct BenchmarkStats {
    pub name: String,
    pub count: usize,
    pub min_ns: u64,
    pub max_ns: u64,
    pub mean_ns: f64,
    pub std_dev_ns: f64,
    pub p50_ns: u64,
    pub p95_ns: u64,
    pub p99_ns: u64,
    pub jitter_ns: u64,
}

impl BenchmarkStats {
    /// Print formatted statistics
    pub fn print(&self) {
        println!("\n=== {} Benchmark Results ===", self.name);
        println!("  Iterations:  {}", self.count);
        println!("  Min:         {:.3} µs", self.min_ns as f64 / 1000.0);
        println!("  Max:         {:.3} µs", self.max_ns as f64 / 1000.0);
        println!("  Mean:        {:.3} µs", self.mean_ns / 1000.0);
        println!("  Std Dev:     {:.3} µs", self.std_dev_ns / 1000.0);
        println!("  P50:         {:.3} µs", self.p50_ns as f64 / 1000.0);
        println!("  P95:         {:.3} µs", self.p95_ns as f64 / 1000.0);
        println!("  P99:         {:.3} µs", self.p99_ns as f64 / 1000.0);
        println!("  Jitter:      {:.3} µs", self.jitter_ns as f64 / 1000.0);
    }

    /// Convert to JSON-friendly format
    pub fn to_json(&self) -> String {
        format!(
            r#"{{"name":"{}","count":{},"min_us":{:.3},"max_us":{:.3},"mean_us":{:.3},"std_dev_us":{:.3},"p50_us":{:.3},"p95_us":{:.3},"p99_us":{:.3},"jitter_us":{:.3}}}"#,
            self.name,
            self.count,
            self.min_ns as f64 / 1000.0,
            self.max_ns as f64 / 1000.0,
            self.mean_ns / 1000.0,
            self.std_dev_ns / 1000.0,
            self.p50_ns as f64 / 1000.0,
            self.p95_ns as f64 / 1000.0,
            self.p99_ns as f64 / 1000.0,
            self.jitter_ns as f64 / 1000.0,
        )
    }
}

// ----------------------------------------------------------------------------
// System Benchmark Suite
// ----------------------------------------------------------------------------

/// Complete benchmark suite for the real-time system
pub struct SystemBenchmark {
    /// Sensor generation benchmark
    pub sensor_generation: BenchmarkTimer,
    /// Data processing benchmark
    pub data_processing: BenchmarkTimer,
    /// Data transmission benchmark
    pub data_transmission: BenchmarkTimer,
    /// Actuator reception benchmark
    pub actuator_reception: BenchmarkTimer,
    /// PID control benchmark
    pub pid_control: BenchmarkTimer,
    /// Feedback transmission benchmark
    pub feedback_transmission: BenchmarkTimer,
    /// End-to-end latency benchmark
    pub end_to_end: BenchmarkTimer,
    /// Lock contention benchmark
    pub lock_contention: BenchmarkTimer,
    /// Overall system throughput
    pub throughput_samples: Vec<f64>,
}

impl SystemBenchmark {
    /// Create a new benchmark suite
    pub fn new() -> Self {
        Self {
            sensor_generation: BenchmarkTimer::new("Sensor Generation"),
            data_processing: BenchmarkTimer::new("Data Processing"),
            data_transmission: BenchmarkTimer::new("Data Transmission"),
            actuator_reception: BenchmarkTimer::new("Actuator Reception"),
            pid_control: BenchmarkTimer::new("PID Control"),
            feedback_transmission: BenchmarkTimer::new("Feedback Transmission"),
            end_to_end: BenchmarkTimer::new("End-to-End Latency"),
            lock_contention: BenchmarkTimer::new("Lock Contention"),
            throughput_samples: Vec::new(),
        }
    }

    /// Record throughput sample
    pub fn record_throughput(&mut self, ops_per_second: f64) {
        self.throughput_samples.push(ops_per_second);
    }

    /// Get throughput statistics
    pub fn get_throughput_stats(&self) -> (f64, f64, f64) {
        if self.throughput_samples.is_empty() {
            return (0.0, 0.0, 0.0);
        }

        let min = self.throughput_samples.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = self.throughput_samples.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let avg = self.throughput_samples.iter().sum::<f64>() / self.throughput_samples.len() as f64;

        (min, max, avg)
    }

    /// Print complete benchmark report
    pub fn print_report(&self) {
        println!("\n{}", "=".repeat(60));
        println!("           SYSTEM BENCHMARK REPORT");
        println!("{}", "=".repeat(60));

        self.sensor_generation.get_stats().print();
        self.data_processing.get_stats().print();
        self.data_transmission.get_stats().print();
        self.actuator_reception.get_stats().print();
        self.pid_control.get_stats().print();
        let feedback_stats = self.feedback_transmission.get_stats();
        if feedback_stats.count > 0 {
            feedback_stats.print();
        }
        self.end_to_end.get_stats().print();
        let contention_stats = self.lock_contention.get_stats();
        if contention_stats.count > 0 {
            contention_stats.print();
        }

        let (min_tp, max_tp, avg_tp) = self.get_throughput_stats();
        println!("\n=== Throughput Statistics ===");
        println!("  Min:  {:.2} ops/sec", min_tp);
        println!("  Max:  {:.2} ops/sec", max_tp);
        println!("  Avg:  {:.2} ops/sec", avg_tp);

        // Deadline analysis
        self.print_deadline_analysis();
    }

    /// Print deadline analysis
    fn print_deadline_analysis(&self) {
        println!("\n=== Deadline Analysis ===");
        
        let proc_stats = self.data_processing.get_stats();
        let proc_deadline_ns = PROCESSING_DEADLINE.as_nanos() as u64;
        let proc_violations = self.data_processing.get_durations()
            .iter()
            .filter(|&&d| d > proc_deadline_ns)
            .count();
        println!("  Processing Deadline ({:.1} µs):", proc_deadline_ns as f64 / 1000.0);
        println!("    Violations: {} / {} ({:.2}%)", 
            proc_violations, proc_stats.count,
            if proc_stats.count > 0 { proc_violations as f64 / proc_stats.count as f64 * 100.0 } else { 0.0 });

        let tx_stats = self.data_transmission.get_stats();
        let tx_deadline_ns = TRANSMISSION_DEADLINE.as_nanos() as u64;
        let tx_violations = self.data_transmission.get_durations()
            .iter()
            .filter(|&&d| d > tx_deadline_ns)
            .count();
        println!("  Transmission Deadline ({:.1} µs):", tx_deadline_ns as f64 / 1000.0);
        println!("    Violations: {} / {} ({:.2}%)", 
            tx_violations, tx_stats.count,
            if tx_stats.count > 0 { tx_violations as f64 / tx_stats.count as f64 * 100.0 } else { 0.0 });

        let act_deadline_ns = ACTUATOR_DEADLINE.as_nanos() as u64;
        let act_violations = self.pid_control.get_durations()
            .iter()
            .filter(|&&d| d > act_deadline_ns)
            .count();
        let act_stats = self.pid_control.get_stats();
        println!("  Actuator Deadline ({:.1} ms):", act_deadline_ns as f64 / 1_000_000.0);
        println!("    Violations: {} / {} ({:.2}%)", 
            act_violations, act_stats.count,
            if act_stats.count > 0 { act_violations as f64 / act_stats.count as f64 * 100.0 } else { 0.0 });

        let fb_deadline_ns = FEEDBACK_DEADLINE.as_nanos() as u64;
        let fb_violations = self.feedback_transmission.get_durations()
            .iter()
            .filter(|&&d| d > fb_deadline_ns)
            .count();
        let fb_stats = self.feedback_transmission.get_stats();
        println!("  Feedback Deadline ({:.1} µs):", fb_deadline_ns as f64 / 1000.0);
        println!("    Violations: {} / {} ({:.2}%)", 
            fb_violations, fb_stats.count,
            if fb_stats.count > 0 { fb_violations as f64 / fb_stats.count as f64 * 100.0 } else { 0.0 });
    }

    /// Export results to JSON
    pub fn export_json(&self) -> String {
        format!(
            r#"{{"sensor_generation":{},"data_processing":{},"data_transmission":{},"actuator_reception":{},"pid_control":{},"feedback_transmission":{},"end_to_end":{},"lock_contention":{}}}"#,
            self.sensor_generation.get_stats().to_json(),
            self.data_processing.get_stats().to_json(),
            self.data_transmission.get_stats().to_json(),
            self.actuator_reception.get_stats().to_json(),
            self.pid_control.get_stats().to_json(),
            self.feedback_transmission.get_stats().to_json(),
            self.end_to_end.get_stats().to_json(),
            self.lock_contention.get_stats().to_json(),
        )
    }

    /// Clear all benchmarks
    pub fn clear(&mut self) {
        self.sensor_generation.clear();
        self.data_processing.clear();
        self.data_transmission.clear();
        self.actuator_reception.clear();
        self.pid_control.clear();
        self.feedback_transmission.clear();
        self.end_to_end.clear();
        self.lock_contention.clear();
        self.throughput_samples.clear();
    }
}

impl Default for SystemBenchmark {
    fn default() -> Self {
        Self::new()
    }
}

// ----------------------------------------------------------------------------
// Load Generator (for CPU load simulation - Advanced Feature)
// ----------------------------------------------------------------------------

/// Generates artificial CPU load for testing
pub struct LoadGenerator {
    /// Target CPU usage (0.0 to 1.0)
    target_load: f64,
    /// Running flag
    running: bool,
    /// Load thread handle
    load_threads: Vec<std::thread::JoinHandle<()>>,
}

impl LoadGenerator {
    /// Create a new load generator
    pub fn new() -> Self {
        Self {
            target_load: 0.0,
            running: false,
            load_threads: Vec::new(),
        }
    }

    /// Start generating load
    pub fn start(&mut self, target_load: f64, num_threads: usize) {
        self.target_load = target_load.clamp(0.0, 1.0);
        self.running = true;

        for _ in 0..num_threads {
            let load = self.target_load;
            let handle = std::thread::spawn(move || {
                let mut counter: u64 = 0;
                let work_duration = Duration::from_micros((load * 1000.0) as u64);
                let sleep_duration = Duration::from_micros(((1.0 - load) * 1000.0) as u64);

                loop {
                    // Work phase
                    let start = Instant::now();
                    while start.elapsed() < work_duration {
                        counter = counter.wrapping_add(1);
                        // Prevent optimization
                        std::hint::black_box(counter);
                    }

                    // Sleep phase
                    if sleep_duration > Duration::ZERO {
                        std::thread::sleep(sleep_duration);
                    }

                    // Check for stop (simplified - in real code use atomic flag)
                    if counter > 1_000_000_000 {
                        break;
                    }
                }
            });
            self.load_threads.push(handle);
        }
    }

    /// Stop generating load
    pub fn stop(&mut self) {
        self.running = false;
        // Note: threads will eventually stop due to counter limit
        // For proper implementation, use atomic flag
    }

    /// Get current target load
    pub fn get_target_load(&self) -> f64 {
        self.target_load
    }
}

impl Default for LoadGenerator {
    fn default() -> Self {
        Self::new()
    }
}

// ----------------------------------------------------------------------------
// Comparison Helper
// ----------------------------------------------------------------------------

/// Compares two benchmark results
pub fn compare_benchmarks(baseline: &BenchmarkStats, test: &BenchmarkStats) -> ComparisonResult {
    let mean_diff = test.mean_ns - baseline.mean_ns;
    let mean_diff_percent = if baseline.mean_ns > 0.0 {
        (mean_diff / baseline.mean_ns) * 100.0
    } else {
        0.0
    };

    let p99_diff = test.p99_ns as i64 - baseline.p99_ns as i64;
    let jitter_diff = test.jitter_ns as i64 - baseline.jitter_ns as i64;

    ComparisonResult {
        baseline_name: baseline.name.clone(),
        test_name: test.name.clone(),
        mean_diff_ns: mean_diff,
        mean_diff_percent,
        p99_diff_ns: p99_diff,
        jitter_diff_ns: jitter_diff,
        is_improvement: mean_diff < 0.0,
    }
}

/// Result of comparing two benchmarks
#[derive(Debug, Clone)]
pub struct ComparisonResult {
    pub baseline_name: String,
    pub test_name: String,
    pub mean_diff_ns: f64,
    pub mean_diff_percent: f64,
    pub p99_diff_ns: i64,
    pub jitter_diff_ns: i64,
    pub is_improvement: bool,
}

impl ComparisonResult {
    /// Print comparison result
    pub fn print(&self) {
        println!("\n=== Comparison: {} vs {} ===", self.baseline_name, self.test_name);
        println!("  Mean Difference: {:.3} µs ({:+.2}%)", 
            self.mean_diff_ns / 1000.0, self.mean_diff_percent);
        println!("  P99 Difference:  {:+.3} µs", self.p99_diff_ns as f64 / 1000.0);
        println!("  Jitter Difference: {:+.3} µs", self.jitter_diff_ns as f64 / 1000.0);
        println!("  Result: {}", if self.is_improvement { "IMPROVEMENT" } else { "REGRESSION" });
    }
}
