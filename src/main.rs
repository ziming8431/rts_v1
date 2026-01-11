// ============================================================================
// RTS Manufacturing System - Main Entry Point
// ============================================================================
// Real-Time Sensor-Actuator System for Automated Manufacturing
// 
// This program demonstrates:
// - Multi-threaded sensor-actuator integration
// - PID control with predictive algorithms
// - Shared resource synchronization (mutex, RwLock, atomics)
// - Fault injection mechanisms
// - Comprehensive performance benchmarking
// - COMPARISON BENCHMARKS for assignment requirements
// ============================================================================

use rts_manufacturing::actuator::*;
use rts_manufacturing::benchmark::*;
use rts_manufacturing::config::*;
use rts_manufacturing::fault_injection::*;
use rts_manufacturing::ipc::*;
use rts_manufacturing::sensor::*;
use rts_manufacturing::shared_resource::*;

use std::sync::atomic::AtomicBool;
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};
use std::fs::File;
use std::io::Write;

// ----------------------------------------------------------------------------
// Main System Integration
// ----------------------------------------------------------------------------

fn main() {
    println!("============================================================");
    println!("  RTS Manufacturing System - Real-Time Control Simulation");
    println!("============================================================");
    println!();

    // Run the main demonstration
    run_demonstration();
}

/// Main demonstration function
fn run_demonstration() {
    println!("Starting system demonstration...\n");

    // 1. Run basic multi-threaded integration
    println!("=== Part 1: Multi-Threaded Integration ===");
    run_multithreaded_system(DEMO_INTEGRATION_CYCLES);

    // 2. Run with fault injection enabled
    println!("\n=== Part 2: Fault Injection Testing ===");
    run_with_fault_injection(DEMO_FAULT_INJECTION_CYCLES);

    // 3. Run under CPU load simulation (COMPARISON: Different Load Levels)
    println!("\n=== Part 3: CPU Load Comparison ===");
    run_under_load(DEMO_LOAD_CYCLES);

    // 4. Run detailed benchmarks
    println!("\n=== Part 4: Performance Benchmarking ===");
    run_benchmarks();

    // 9. Save all benchmark results to files
    println!("\n=== Part 9: Saving Results to Files ===");
    save_benchmark_results();

    println!("\n============================================================");
    println!("  Demonstration Complete");
    println!("============================================================");
}

// ----------------------------------------------------------------------------
// Multi-Threaded System Integration
// ----------------------------------------------------------------------------

/// Run the complete multi-threaded system
fn run_multithreaded_system(num_cycles: usize) {
    println!("Initializing multi-threaded system...");

    let shared = SharedResources::new();
    let ipc = IpcManager::new();
    let running = Arc::new(AtomicBool::new(true));

    let sensor_sender = ipc.get_sensor_sender();
    let feedback_receiver = ipc.get_feedback_receiver();
    let sensor_shared = shared.clone();
    let sensor_running = Arc::clone(&running);

    let sensor_receiver = ipc.get_sensor_receiver();
    let feedback_sender = ipc.get_feedback_sender();
    let actuator_shared = shared.clone();
    let actuator_running = Arc::clone(&running);

    let start_time = Instant::now();

    let sensor_handle = thread::Builder::new()
        .name("sensor-module".into())
        .spawn(move || {
            let mut sensor = SensorModule::new(
                sensor_sender,
                feedback_receiver,
                sensor_shared,
                sensor_running,
            );
            
            for _ in 0..num_cycles {
                if let Err(e) = sensor.run_cycle() {
                    eprintln!("Sensor error: {}", e);
                }
                thread::sleep(Duration::from_millis(1));
            }
            
            sensor.get_stats()
        })
        .expect("Failed to spawn sensor thread");

    let actuator_handle = thread::Builder::new()
        .name("actuator-module".into())
        .spawn(move || {
            let mut actuator = ActuatorModule::new(
                sensor_receiver,
                feedback_sender,
                actuator_shared,
                actuator_running,
            );
            
            for _ in 0..num_cycles {
                if let Err(e) = actuator.run_cycle() {
                    eprintln!("Actuator error: {}", e);
                }
                thread::sleep(Duration::from_micros(500));
            }
            
            actuator.get_stats()
        })
        .expect("Failed to spawn actuator thread");

    let sensor_stats = sensor_handle.join().expect("Sensor thread panicked");
    let actuator_stats = actuator_handle.join().expect("Actuator thread panicked");
    let total_time = start_time.elapsed();

    println!("\n--- Multi-Threaded System Results ---");
    println!("Total Runtime: {:.2} seconds", total_time.as_secs_f64());
    println!("Total Sensor Cycles: {}", sensor_stats.total_cycles);
    println!("Total Actuator Cycles: {}", actuator_stats.total_cycles);
    
    println!("\nSensor Performance:");
    println!("  Avg Generation Time:   {:.3} µs", sensor_stats.generation.avg_latency_ns / 1000.0);
    println!("  Avg Processing Time:   {:.3} µs", sensor_stats.processing.avg_latency_ns / 1000.0);
    println!("  Avg Transmission Time: {:.3} µs", sensor_stats.transmission.avg_latency_ns / 1000.0);
    println!("  Missed Deadlines:      {}", sensor_stats.missed_deadlines);

    println!("\nActuator Performance:");
    println!("  Avg Reception Time:    {:.3} µs", actuator_stats.reception.avg_latency_ns / 1000.0);
    println!("  Avg Control Time:      {:.3} µs", actuator_stats.control.avg_latency_ns / 1000.0);
    println!("  Avg Feedback Time:     {:.3} µs", actuator_stats.feedback.avg_latency_ns / 1000.0);
    println!("  Missed Deadlines:      {}", actuator_stats.missed_deadlines);
    shared.print_sync_stats();
}

// ----------------------------------------------------------------------------
// Fault Injection Testing
// ----------------------------------------------------------------------------

fn run_with_fault_injection(num_cycles: usize) {
    println!("Initializing fault injection test...");

    let shared = SharedResources::new();
    let ipc = IpcManager::new();
    let running = Arc::new(AtomicBool::new(true));

    let mut fault_injector = FaultInjector::new();
    fault_injector.set_probabilities(0.05, 0.03, 0.02, 0.05);

    let mut fault_detector = FaultDetector::new(NUM_SENSOR_TYPES);

    let mut sensor = SensorModule::new(
        ipc.get_sensor_sender(),
        ipc.get_feedback_receiver(),
        shared.clone(),
        Arc::clone(&running),
    );

    let mut actuator = ActuatorModule::new(
        ipc.get_sensor_receiver(),
        ipc.get_feedback_sender(),
        shared.clone(),
        Arc::clone(&running),
    );

    let mut faults_injected = 0;
    let mut faults_detected = 0;

    for _cycle in 0..num_cycles {
        if let Ok(processed_data) = sensor.run_cycle() {
            for data in processed_data {
                if let Some((faulty_data, record)) = fault_injector.apply_fault(data) {
                    if record.fault_type != FaultType::None {
                        faults_injected += 1;
                    }
                    let issues = fault_detector.check_data(&faulty_data);
                    if !issues.is_empty() {
                        faults_detected += 1;
                    }
                }
            }
        }
        let _ = actuator.run_cycle();
        thread::sleep(Duration::from_millis(1));
    }

    println!("\n--- Fault Injection Results ---");
    fault_injector.print_stats();
    println!("\nFault Detection:");
    println!("  Faults Injected: {}", faults_injected);
    println!("  Faults Flagged: {}", faults_detected);
    println!("  Faults Detected (seq/time): {}", fault_detector.get_fault_count());
    
}

// ----------------------------------------------------------------------------
// CPU Load Simulation (COMPARISON: Normal vs High Load)
// ----------------------------------------------------------------------------

fn run_under_load(num_cycles: usize) {
    println!("Starting CPU load simulation...");
    println!("COMPARISON: System performance under varying CPU loads\n");

    let load_levels = [0.0, 0.3, 0.6, 0.8];
    let mut results: Vec<(f64, f64, f64, usize, f64)> = Vec::new();

    for &load in &load_levels {
        println!("Testing at {:.0}% CPU load...", load * 100.0);

        let num_load_threads = if load > 0.0 { 2 } else { 0 };
        let load_handles: Vec<_> = (0..num_load_threads)
            .map(|_| {
                let target_load = load;
                thread::spawn(move || {
                    let start = Instant::now();
                    let mut counter: u64 = 0;
                    while start.elapsed() < Duration::from_secs(2) {
                        for _ in 0..(target_load * 10000.0) as usize {
                            counter = counter.wrapping_add(1);
                            std::hint::black_box(counter);
                        }
                        thread::sleep(Duration::from_micros(100));
                    }
                })
            })
            .collect();

        let shared = SharedResources::new();
        let ipc = IpcManager::new();
        let running = Arc::new(AtomicBool::new(true));

        let mut sensor = SensorModule::new(
            ipc.get_sensor_sender(),
            ipc.get_feedback_receiver(),
            shared.clone(),
            Arc::clone(&running),
        );

        let mut actuator = ActuatorModule::new(
            ipc.get_sensor_receiver(),
            ipc.get_feedback_sender(),
            shared.clone(),
            Arc::clone(&running),
        );

        let mut latencies: Vec<u64> = Vec::new();
        let mut missed = 0;

        for _ in 0..num_cycles {
            let cycle_start = Instant::now();
            let _ = sensor.run_cycle();
            let _ = actuator.run_cycle();
            let cycle_time = cycle_start.elapsed().as_nanos() as u64;
            latencies.push(cycle_time);
            
            if cycle_time > ACTUATOR_DEADLINE.as_nanos() as u64 {
                missed += 1;
            }
            thread::sleep(Duration::from_millis(1));
        }

        for handle in load_handles {
            let _ = handle.join();
        }

        let avg_latency = latencies.iter().sum::<u64>() as f64 / latencies.len() as f64;
        let max_latency = *latencies.iter().max().unwrap_or(&0) as f64;
        let mut sorted = latencies.clone();
        sorted.sort_unstable();
        let p99_idx = (sorted.len() as f64 * 0.99) as usize;
        let p99_latency = sorted[p99_idx.min(sorted.len() - 1)] as f64;
        
        results.push((load, avg_latency, max_latency, missed, p99_latency));
    }

    println!("\n╔═══════════════════════════════════════════════════════════════════════════╗");
    println!("║                    CPU LOAD COMPARISON RESULTS                            ║");
    println!("╠═══════════════════════════════════════════════════════════════════════════╣");
    println!("║ {:^10} │ {:^15} │ {:^15} │ {:^15} │ {:^10} ║", 
        "Load %", "Avg Lat (µs)", "Max Lat (µs)", "P99 Lat (µs)", "Missed");
    println!("╠═══════════════════════════════════════════════════════════════════════════╣");
    for (load, avg, max, missed, p99) in &results {
        println!("║ {:^10.0} │ {:>15.3} │ {:>15.3} │ {:>15.3} │ {:^10} ║", 
            load * 100.0, avg / 1000.0, max / 1000.0, p99 / 1000.0, missed);
    }
    println!("╚═══════════════════════════════════════════════════════════════════════════╝");

    // Analysis
    let baseline = &results[0];
    let high_load = &results[3];
    let latency_increase = ((high_load.1 - baseline.1) / baseline.1) * 100.0;
    
    println!("\n--- Analysis ---");
    println!("  • At 80% CPU load, latency increased by {:.1}% compared to 0% load", latency_increase);
    println!("  • Missed deadlines increased from {} to {} under high load", baseline.3, high_load.3);
    println!("  • P99 latency shows worst-case behavior critical for real-time systems");
}

// ----------------------------------------------------------------------------
// Performance Benchmarking
// ----------------------------------------------------------------------------

fn run_benchmarks() {
    println!("Running performance benchmarks...");
    
    let mut benchmark = SystemBenchmark::new();
    let iterations = 1000;

    println!("  Benchmarking sensor generation...");
    let mut sensor_sim = rts_manufacturing::sensor::SensorSimulator::new(0, "Test");
    for _ in 0..iterations {
        benchmark.sensor_generation.start();
        let _ = sensor_sim.generate_reading();
        benchmark.sensor_generation.stop();
    }

    println!("  Benchmarking data processing...");
    let mut processor = rts_manufacturing::sensor::DataProcessor::new(NUM_SENSOR_TYPES);
    for i in 0..iterations {
        let reading = rts_manufacturing::types::SensorReading::new(
            0, "Test".to_string(), 50.0 + (i as f64 * 0.1), i as u64
        );
        benchmark.data_processing.start();
        let _ = processor.process(&reading);
        benchmark.data_processing.stop();
    }

    println!("  Benchmarking PID control...");
    let mut pid = rts_manufacturing::pid_controller::PidController::with_defaults("Test");
    pid.set_setpoint(50.0);
    for i in 0..iterations {
        let measurement = 45.0 + (i as f64 * 0.01);
        benchmark.pid_control.start();
        let _ = pid.update(measurement);
        benchmark.pid_control.stop();
    }

    println!("  Benchmarking channel operations...");
    let (tx, rx) = std::sync::mpsc::sync_channel::<u64>(100);
    for i in 0..iterations {
        benchmark.data_transmission.start();
        let _ = tx.send(i as u64);
        benchmark.data_transmission.stop();
        
        benchmark.actuator_reception.start();
        let _ = rx.recv();
        benchmark.actuator_reception.stop();
    }

    println!("  Benchmarking end-to-end cycle...");
    let shared = SharedResources::new();
    let ipc = IpcManager::new();
    let running = Arc::new(AtomicBool::new(true));

    let mut sensor = SensorModule::new(
        ipc.get_sensor_sender(),
        ipc.get_feedback_receiver(),
        shared.clone(),
        Arc::clone(&running),
    );

    let mut actuator = ActuatorModule::new(
        ipc.get_sensor_receiver(),
        ipc.get_feedback_sender(),
        shared,
        running,
    );

    let end_to_end_iterations = 200;
    let start = Instant::now();
    for _ in 0..end_to_end_iterations {
        benchmark.end_to_end.start();
        let _ = sensor.run_cycle();
        let _ = actuator.run_cycle();
        benchmark.end_to_end.stop();
    }
    let elapsed = start.elapsed();
    if elapsed.as_secs_f64() > 0.0 {
        benchmark.record_throughput(end_to_end_iterations as f64 / elapsed.as_secs_f64());
    }

    benchmark.print_report();
}

// ----------------------------------------------------------------------------
// Save Benchmark Results
// ----------------------------------------------------------------------------

fn save_benchmark_results() {
    println!("Saving benchmark results to files...");
    
    let mut benchmark = SystemBenchmark::new();
    let iterations = 500;
    
    let mut sensor_sim = rts_manufacturing::sensor::SensorSimulator::new(0, "Test");
    for _ in 0..iterations {
        benchmark.sensor_generation.start();
        let _ = sensor_sim.generate_reading();
        benchmark.sensor_generation.stop();
    }
    
    let mut processor = rts_manufacturing::sensor::DataProcessor::new(NUM_SENSOR_TYPES);
    for i in 0..iterations {
        let reading = rts_manufacturing::types::SensorReading::new(
            0, "Test".to_string(), 50.0 + (i as f64 * 0.1), i as u64
        );
        benchmark.data_processing.start();
        let _ = processor.process(&reading);
        benchmark.data_processing.stop();
    }
    
    let mut pid = rts_manufacturing::pid_controller::PidController::with_defaults("Test");
    pid.set_setpoint(50.0);
    for i in 0..iterations {
        benchmark.pid_control.start();
        let _ = pid.update(45.0 + (i as f64 * 0.01));
        benchmark.pid_control.stop();
    }
    
    let (tx, rx) = std::sync::mpsc::sync_channel::<u64>(100);
    for i in 0..iterations {
        benchmark.data_transmission.start();
        let _ = tx.send(i as u64);
        benchmark.data_transmission.stop();
        benchmark.actuator_reception.start();
        let _ = rx.recv();
        benchmark.actuator_reception.stop();
    }

    let shared = SharedResources::new();
    let ipc = IpcManager::new();
    let running = Arc::new(AtomicBool::new(true));

    let mut sensor = SensorModule::new(
        ipc.get_sensor_sender(),
        ipc.get_feedback_receiver(),
        shared.clone(),
        Arc::clone(&running),
    );

    let mut actuator = ActuatorModule::new(
        ipc.get_sensor_receiver(),
        ipc.get_feedback_sender(),
        shared,
        running,
    );

    let end_to_end_iterations = 200;
    let start = Instant::now();
    for _ in 0..end_to_end_iterations {
        benchmark.end_to_end.start();
        let _ = sensor.run_cycle();
        let _ = actuator.run_cycle();
        benchmark.end_to_end.stop();
    }
    let elapsed = start.elapsed();
    if elapsed.as_secs_f64() > 0.0 {
        benchmark.record_throughput(end_to_end_iterations as f64 / elapsed.as_secs_f64());
    }
    
    let json = benchmark.export_json();
    match File::create("benchmark_results.json") {
        Ok(mut file) => {
            if let Err(e) = writeln!(file, "{}", json) {
                eprintln!("Failed to write benchmark JSON: {}", e);
            } else {
                println!("  Saved benchmark_results.json");
            }
        }
        Err(e) => eprintln!("Failed to create benchmark JSON file: {}", e),
    }
    
    save_timing_log(&benchmark);
}

fn save_timing_log(benchmark: &SystemBenchmark) {
    let mut log = String::new();

    log.push_str("============================================================
");
    log.push_str("  RTS Manufacturing System - Timing Log
");
    log.push_str("============================================================
");
    log.push_str(&format!("Generated at: {:?}

", std::time::SystemTime::now()));

    fn append_timer(log: &mut String, name: &str, stats: &BenchmarkStats) {
        log.push_str(&format!("=== {} ===
", name));
        log.push_str(&format!("  Iterations: {}
", stats.count));
        log.push_str(&format!("  Min: {:.3} us
", stats.min_ns as f64 / 1000.0));
        log.push_str(&format!("  Max: {:.3} us
", stats.max_ns as f64 / 1000.0));
        log.push_str(&format!("  Mean: {:.3} us
", stats.mean_ns / 1000.0));
        log.push_str(&format!("  Std Dev: {:.3} us
", stats.std_dev_ns / 1000.0));
        log.push_str(&format!("  P50: {:.3} us
", stats.p50_ns as f64 / 1000.0));
        log.push_str(&format!("  P95: {:.3} us
", stats.p95_ns as f64 / 1000.0));
        log.push_str(&format!("  P99: {:.3} us
", stats.p99_ns as f64 / 1000.0));
        log.push_str(&format!("  Jitter: {:.3} us

", stats.jitter_ns as f64 / 1000.0));
    }

    fn append_deadline(
        log: &mut String,
        name: &str,
        durations: &[u64],
        deadline_ns: u64,
    ) {
        let count = durations.len();
        let violations = durations.iter().filter(|&&d| d > deadline_ns).count();
        let percent = if count > 0 {
            violations as f64 / count as f64 * 100.0
        } else {
            0.0
        };

        log.push_str(&format!(
            "  {} Deadline ({:.1} us):
",
            name,
            deadline_ns as f64 / 1000.0
        ));
        log.push_str(&format!(
            "    Violations: {} / {} ({:.2}%)
",
            violations,
            count,
            percent
        ));
    }

    append_timer(
        &mut log,
        "Sensor Generation",
        &benchmark.sensor_generation.get_stats(),
    );
    append_timer(
        &mut log,
        "Data Processing",
        &benchmark.data_processing.get_stats(),
    );
    append_timer(
        &mut log,
        "Data Transmission",
        &benchmark.data_transmission.get_stats(),
    );
    append_timer(
        &mut log,
        "Actuator Reception",
        &benchmark.actuator_reception.get_stats(),
    );
    append_timer(
        &mut log,
        "PID Control",
        &benchmark.pid_control.get_stats(),
    );
    append_timer(
        &mut log,
        "Feedback Transmission",
        &benchmark.feedback_transmission.get_stats(),
    );
    append_timer(
        &mut log,
        "End-to-End Latency",
        &benchmark.end_to_end.get_stats(),
    );
    append_timer(
        &mut log,
        "Lock Contention",
        &benchmark.lock_contention.get_stats(),
    );

    let (min_tp, max_tp, avg_tp) = benchmark.get_throughput_stats();
    log.push_str("=== Throughput Summary ===
");
    log.push_str(&format!("  Min: {:.2} ops/sec
", min_tp));
    log.push_str(&format!("  Max: {:.2} ops/sec
", max_tp));
    log.push_str(&format!("  Avg: {:.2} ops/sec

", avg_tp));

    log.push_str("=== Deadline Analysis ===
");
    append_deadline(
        &mut log,
        "Processing",
        benchmark.data_processing.get_durations(),
        PROCESSING_DEADLINE.as_nanos() as u64,
    );
    append_deadline(
        &mut log,
        "Transmission",
        benchmark.data_transmission.get_durations(),
        TRANSMISSION_DEADLINE.as_nanos() as u64,
    );
    append_deadline(
        &mut log,
        "Actuator Control",
        benchmark.pid_control.get_durations(),
        ACTUATOR_DEADLINE.as_nanos() as u64,
    );
    append_deadline(
        &mut log,
        "Feedback",
        benchmark.feedback_transmission.get_durations(),
        FEEDBACK_DEADLINE.as_nanos() as u64,
    );

    log.push_str("============================================================
");

    match File::create("timing_log.txt") {
        Ok(mut file) => {
            if let Err(e) = write!(file, "{}", log) {
                eprintln!("Failed to write timing log: {}", e);
            } else {
                println!("  Saved timing_log.txt");
            }
        }
        Err(e) => eprintln!("Failed to create timing log file: {}", e),
    }
}

// ----------------------------------------------------------------------------
// Unit Tests
// ----------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use rts_manufacturing::types::*;

    #[test]
    fn test_sensor_generation() {
        let mut sensor = rts_manufacturing::sensor::SensorSimulator::new(0, "Force");
        let reading = sensor.generate_reading();
        assert_eq!(reading.sensor_id, 0);
        assert!(reading.value > 0.0);
    }

    #[test]
    fn test_pid_controller() {
        let mut pid = rts_manufacturing::pid_controller::PidController::with_defaults("Test");
        pid.set_setpoint(100.0);
        let (output, error, _) = pid.update(50.0);
        assert!(output > 0.0);
        assert!((error - 50.0).abs() < 0.001);
    }

    #[test]
    fn test_shared_resource_sync() {
        let shared = SharedResources::new();
        shared.status_memory.increment_cycles();
        assert_eq!(shared.status_memory.get_cycles(), 1);
        
        shared.diagnostic_log.log(LogLevel::Info, "Test", "Test message");
        let recent = shared.diagnostic_log.get_recent(1);
        assert_eq!(recent.len(), 1);
    }

    #[test]
    fn test_channel_communication() {
        let channel = SensorDataChannel::new(10);
        let sender = channel.get_sender();
        let receiver = channel.get_receiver();

        let data = ProcessedSensorData::new(
            0, "Test".to_string(), 50.0, 50.0, false, 1.0, 100, 1
        );

        sender.send(data.clone()).unwrap();
        let received = receiver.recv().unwrap();
        
        assert_eq!(received.sensor_id, 0);
        assert!((received.filtered_value - 50.0).abs() < 0.001);
    }
}
