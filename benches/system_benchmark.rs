//! # Criterion Benchmarks for RTS V2 (Optimized)
//!
//! These benchmarks test the ACTUAL V2 system modules - the same code that
//! main.rs uses. This ensures benchmarks reflect real system performance.
//!
//! Run with: cargo bench
//! View reports in: target/criterion/report/index.html

use criterion::{criterion_group, criterion_main, Criterion, Throughput};
use std::hint::black_box;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::thread;
use std::time::{Duration, Instant};

// Import ACTUAL V2 modules (same as main.rs)
use rts_manufacturing::sensor::{SensorModule, SensorSimulator, DataProcessor};
use rts_manufacturing::actuator::ActuatorModule;
use rts_manufacturing::pid_controller::PidController;
use rts_manufacturing::shared_resource::*;
use rts_manufacturing::ipc::*;
use rts_manufacturing::config::*;
use rts_manufacturing::types::*;

// ============================================================================
// ACTUAL SYSTEM BENCHMARKS
// ============================================================================
// These benchmark the EXACT same code paths that main.rs uses

/// Benchmark the ACTUAL SensorModule::run_cycle() - same as main.rs
fn bench_sensor_module_run_cycle(c: &mut Criterion) {
    let mut group = c.benchmark_group("v2_actual_system");

    group.bench_function("sensor_module_run_cycle", |b| {
        // Setup exactly like main.rs does
        let shared = SharedResources::new();
        let ipc = IpcManager::new();
        let running = Arc::new(AtomicBool::new(true));

        let sender = ipc.get_sensor_sender();
        let feedback_receiver = ipc.get_feedback_receiver();

        let mut sensor_module = SensorModule::new(
            sender,
            feedback_receiver,
            shared,
            running,
        );

        b.iter(|| {
            // Benchmark the ACTUAL run_cycle - same as main.rs calls
            black_box(sensor_module.run_cycle())
        })
    });

    group.finish();
}

/// Benchmark the ACTUAL ActuatorModule::run_cycle() - same as main.rs
fn bench_actuator_module_run_cycle(c: &mut Criterion) {
    let mut group = c.benchmark_group("v2_actual_system");

    group.bench_function("actuator_module_run_cycle", |b| {
        // Setup exactly like main.rs does
        let shared = SharedResources::new();
        let ipc = IpcManager::new();
        let running = Arc::new(AtomicBool::new(true));

        let sensor_receiver = ipc.get_sensor_receiver();
        let feedback_sender = ipc.get_feedback_sender();
        let sensor_sender = ipc.get_sensor_sender();

        // Pre-populate channel with test data so actuator has data to process
        for i in 0..10 {
            let data = ProcessedSensorData::new(
                0, "Force".to_string(), 50.0, 50.0 + (i as f64 * 0.1), 
                false, 1.0, 1000, i as u64
            );
            let _ = sensor_sender.try_send(data);
        }

        let mut actuator_module = ActuatorModule::new(
            sensor_receiver,
            feedback_sender,
            shared,
            running,
        );

        b.iter(|| {
            // Benchmark the ACTUAL run_cycle - same as main.rs calls
            black_box(actuator_module.run_cycle())
        })
    });

    group.finish();
}

/// Benchmark COMPLETE end-to-end cycle - sensor → channel → actuator
fn bench_complete_system_cycle(c: &mut Criterion) {
    let mut group = c.benchmark_group("v2_actual_system");
    group.throughput(Throughput::Elements(1));

    group.bench_function("complete_end_to_end_cycle", |b| {
        // Setup EXACTLY like main.rs
        let shared = SharedResources::new();
        let ipc = IpcManager::new();
        let running = Arc::new(AtomicBool::new(true));

        let sensor_sender = ipc.get_sensor_sender();
        let sensor_receiver = ipc.get_sensor_receiver();
        let feedback_sender = ipc.get_feedback_sender();
        let feedback_receiver = ipc.get_feedback_receiver();

        let mut sensor_module = SensorModule::new(
            sensor_sender,
            feedback_receiver,
            shared.clone(),
            Arc::clone(&running),
        );

        let mut actuator_module = ActuatorModule::new(
            sensor_receiver,
            feedback_sender,
            shared.clone(),
            Arc::clone(&running),
        );

        b.iter(|| {
            // 1. Sensor generates and sends data (ACTUAL run_cycle)
            let _ = sensor_module.run_cycle();

            // 2. Actuator receives and processes (ACTUAL run_cycle)
            let result = actuator_module.run_cycle();

            black_box(result)
        })
    });

    group.finish();
}

/// Benchmark complete multi-threaded system (sensor + actuator threads)
fn bench_multithreaded_system_end_to_end(c: &mut Criterion) {
    let mut group = c.benchmark_group("v2_actual_system");
    group.sample_size(20);
    group.measurement_time(Duration::from_secs(3));
    group.throughput(Throughput::Elements(1));

    group.bench_function("multi_threaded_end_to_end", |b| {
        b.iter_custom(|iters| {
            let cycles = iters as usize;
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

            let start = Instant::now();

            let sensor_handle = thread::spawn(move || {
                let mut sensor = SensorModule::new(
                    sensor_sender,
                    feedback_receiver,
                    sensor_shared,
                    sensor_running,
                );

                for _ in 0..cycles {
                    let _ = sensor.run_cycle();
                    thread::sleep(Duration::from_millis(1));
                }

                sensor.get_stats()
            });

            let actuator_handle = thread::spawn(move || {
                let mut actuator = ActuatorModule::new(
                    sensor_receiver,
                    feedback_sender,
                    actuator_shared,
                    actuator_running,
                );

                for _ in 0..cycles {
                    let _ = actuator.run_cycle();
                    thread::sleep(Duration::from_micros(500));
                }

                actuator.get_stats()
            });

            let sensor_stats = sensor_handle.join().expect("Sensor thread panicked");
            let actuator_stats = actuator_handle.join().expect("Actuator thread panicked");

            black_box(sensor_stats);
            black_box(actuator_stats);

            start.elapsed()
        })
    });

    group.finish();
}

// ============================================================================
// COMPONENT TIMING BENCHMARKS
// ============================================================================
// Individual component timing to verify deadlines

/// Benchmark sensor generation (0.2ms deadline check)
fn bench_sensor_generation(c: &mut Criterion) {
    let mut group = c.benchmark_group("v2_component_timing");

    group.bench_function("sensor_generation", |b| {
        let mut sensor = SensorSimulator::new(SENSOR_FORCE, "Force");

        b.iter(|| {
            black_box(sensor.generate_reading())
        })
    });

    group.finish();
}

/// Benchmark data processing with filtering (0.2ms deadline)
fn bench_data_processing(c: &mut Criterion) {
    let mut group = c.benchmark_group("v2_component_timing");

    group.bench_function("data_processing_filter", |b| {
        let mut processor = DataProcessor::new(NUM_SENSOR_TYPES);
        let mut sensor = SensorSimulator::new(SENSOR_FORCE, "Force");

        b.iter(|| {
            let reading = sensor.generate_reading();
            black_box(processor.process(&reading))
        })
    });

    group.finish();
}

/// Benchmark PID controller update (part of actuator 1-2ms deadline)
fn bench_pid_update(c: &mut Criterion) {
    let mut group = c.benchmark_group("v2_component_timing");

    group.bench_function("pid_update", |b| {
        let mut pid = PidController::with_defaults("Test");
        pid.set_setpoint(50.0);

        b.iter(|| {
            black_box(pid.update_with_dt(black_box(45.0), black_box(0.001)))
        })
    });

    group.finish();
}

/// Benchmark channel transmission (0.1ms deadline)
fn bench_channel_transmission(c: &mut Criterion) {
    let mut group = c.benchmark_group("v2_component_timing");

    // Test actual SensorDataChannel (crossbeam bounded)
    group.bench_function("sensor_data_channel", |b| {
        let channel = SensorDataChannel::new(CHANNEL_BUFFER_SIZE);
        let sender = channel.get_sender();
        let receiver = channel.get_receiver();

        let data = ProcessedSensorData::new(
            0, "Force".to_string(), 50.0, 50.0, false, 1.0, 1000, 1
        );

        b.iter(|| {
            let _ = sender.try_send(data.clone());
            black_box(receiver.try_recv())
        })
    });

    // Test actual FeedbackChannel
    group.bench_function("feedback_channel", |b| {
        let channel = FeedbackChannel::new(CHANNEL_BUFFER_SIZE);
        let sender = channel.get_sender();
        let receiver = channel.get_receiver();

        let feedback = ActuatorFeedback::new(0, 1, ActuatorState::new(0));

        b.iter(|| {
            let _ = sender.try_send(feedback.clone());
            black_box(receiver.try_recv())
        })
    });

    group.finish();
}

/// Benchmark shared resource operations
fn bench_shared_resources(c: &mut Criterion) {
    let mut group = c.benchmark_group("v2_component_timing");

    // Atomic counter (what V2 uses for counters)
    group.bench_function("status_memory_increment", |b| {
        let status = StatusMemory::new();

        b.iter(|| {
            black_box(status.increment_cycles())
        })
    });

    // RwLock config read (what V2 uses for config)
    group.bench_function("config_buffer_read", |b| {
        let config = ConfigBuffer::new();

        b.iter(|| {
            black_box(config.read())
        })
    });

    // parking_lot::Mutex log write
    group.bench_function("diagnostic_log_write", |b| {
        let log = DiagnosticLog::new(1000);  // Use direct value

        b.iter(|| {
            log.try_log(LogLevel::Info, "Bench", "Test message")
        })
    });

    group.finish();
}

// ============================================================================
// SYNCHRONIZATION COMPARISON BENCHMARKS
// ============================================================================
// Compare V2's optimized primitives vs alternatives

fn bench_sync_primitives(c: &mut Criterion) {
    let mut group = c.benchmark_group("v2_sync_comparison");
    group.throughput(Throughput::Elements(10000));

    // What V2 uses: parking_lot::Mutex
    group.bench_function("parking_lot_mutex", |b| {
        let mutex = parking_lot::Mutex::new(0u64);

        b.iter(|| {
            for _ in 0..10000 {
                let mut guard = mutex.lock();
                *guard += 1;
                black_box(*guard);
            }
        })
    });

    // What V2 uses: AtomicU64 for counters
    group.bench_function("atomic_u64", |b| {
        let atomic = std::sync::atomic::AtomicU64::new(0);

        b.iter(|| {
            for _ in 0..10000 {
                let val = atomic.fetch_add(1, Ordering::SeqCst);
                black_box(val);
            }
        })
    });

    // What V2 uses: parking_lot::RwLock for config
    group.bench_function("rwlock_read", |b| {
        let rwlock = parking_lot::RwLock::new(0u64);

        b.iter(|| {
            for _ in 0..10000 {
                let guard = rwlock.read();
                black_box(*guard);
            }
        })
    });

    group.finish();
}

// ============================================================================
// SHARED RESOURCE CONTENTION BENCHMARKS
// ============================================================================

fn bench_shared_resource_contention(c: &mut Criterion) {
    const THREADS: usize = 4;
    const OPS_PER_THREAD: usize = 200;

    let mut group = c.benchmark_group("v2_shared_resource_contention");
    group.sample_size(20);
    group.measurement_time(Duration::from_secs(3));

    group.throughput(Throughput::Elements((THREADS * OPS_PER_THREAD) as u64));
    group.bench_function("diagnostic_log_contended", |b| {
        let log = Arc::new(DiagnosticLog::new(1000));

        b.iter(|| {
            let handles: Vec<_> = (0..THREADS)
                .map(|_| {
                    let log = Arc::clone(&log);
                    thread::spawn(move || {
                        for _ in 0..OPS_PER_THREAD {
                            log.log(LogLevel::Info, "Bench", "Contention");
                        }
                    })
                })
                .collect();

            for h in handles {
                h.join().unwrap();
            }

            black_box(log.get_stats());
        })
    });

    const READERS: usize = 3;
    const WRITERS: usize = 1;
    group.throughput(Throughput::Elements(((READERS + WRITERS) * OPS_PER_THREAD) as u64));
    group.bench_function("config_buffer_read_write", |b| {
        let config = Arc::new(ConfigBuffer::new());

        b.iter(|| {
            let mut handles = Vec::with_capacity(READERS + WRITERS);

            for _ in 0..READERS {
                let config = Arc::clone(&config);
                handles.push(thread::spawn(move || {
                    for _ in 0..OPS_PER_THREAD {
                        black_box(config.read());
                    }
                }));
            }

            for _ in 0..WRITERS {
                let config = Arc::clone(&config);
                handles.push(thread::spawn(move || {
                    for _ in 0..OPS_PER_THREAD {
                        config.update(|cfg| {
                            cfg.anomaly_threshold += 0.001;
                            if cfg.anomaly_threshold > ANOMALY_THRESHOLD * 2.0 {
                                cfg.anomaly_threshold = ANOMALY_THRESHOLD;
                            }
                        });
                    }
                }));
            }

            for h in handles {
                h.join().unwrap();
            }

            black_box(config.get_stats());
        })
    });

    group.finish();
}

// ============================================================================
// PRIORITY INVERSION SIMULATION BENCHMARKS
// ============================================================================

fn bench_priority_inversion_simulated(c: &mut Criterion) {
    const HOLD_TIME_US: u64 = 500;

    let mut group = c.benchmark_group("v2_priority_inversion");
    group.sample_size(20);
    group.measurement_time(Duration::from_secs(3));
    group.throughput(Throughput::Elements(1));

    group.bench_function("config_buffer_priority_inversion", |b| {
        b.iter_custom(|iters| {
            let mut total_wait_ns: u128 = 0;

            for _ in 0..iters {
                let config = Arc::new(ConfigBuffer::new());
                let low_locked = Arc::new(AtomicBool::new(false));
                let high_done = Arc::new(AtomicBool::new(false));
                let wait_ns = Arc::new(AtomicU64::new(0));

                let low_config = Arc::clone(&config);
                let low_locked_flag = Arc::clone(&low_locked);
                let low = thread::spawn(move || {
                    low_config.update(|cfg| {
                        low_locked_flag.store(true, Ordering::Release);
                        thread::sleep(Duration::from_micros(HOLD_TIME_US));
                        cfg.mode = SystemMode::Normal;
                    });
                });

                let high_config = Arc::clone(&config);
                let low_locked_flag = Arc::clone(&low_locked);
                let high_done_flag = Arc::clone(&high_done);
                let wait_ns_flag = Arc::clone(&wait_ns);
                let high = thread::spawn(move || {
                    while !low_locked_flag.load(Ordering::Acquire) {
                        std::hint::spin_loop();
                    }
                    let start = Instant::now();
                    let value = high_config.read();
                    black_box(value);
                    let elapsed = start.elapsed().as_nanos() as u64;
                    wait_ns_flag.store(elapsed, Ordering::Release);
                    high_done_flag.store(true, Ordering::Release);
                });

                let low_locked_flag = Arc::clone(&low_locked);
                let high_done_flag = Arc::clone(&high_done);
                let medium = thread::spawn(move || {
                    while !low_locked_flag.load(Ordering::Acquire) {
                        std::hint::spin_loop();
                    }
                    let mut burn: u64 = 0;
                    while !high_done_flag.load(Ordering::Acquire) {
                        burn = burn.wrapping_add(1);
                        black_box(burn);
                    }
                });

                let _ = low.join();
                let _ = high.join();
                let _ = medium.join();

                total_wait_ns += wait_ns.load(Ordering::Acquire) as u128;
            }

            let capped = if total_wait_ns > u128::from(u64::MAX) {
                u64::MAX
            } else {
                total_wait_ns as u64
            };

            Duration::from_nanos(capped)
        })
    });

    group.finish();
}

// ============================================================================
// CPU LOAD IMPACT BENCHMARKS
// ============================================================================

fn simulate_cpu_load(load_percent: f64, duration_us: u64) {
    if load_percent <= 0.0 {
        return;
    }

    let busy_time = (duration_us as f64 * load_percent) as u64;
    let start = Instant::now();

    let mut x: u64 = 0;
    while start.elapsed().as_micros() < busy_time as u128 {
        x = x.wrapping_add(1);
        black_box(x);
    }
}

fn bench_cpu_load_impact(c: &mut Criterion) {
    let mut group = c.benchmark_group("v2_cpu_load_impact");
    group.throughput(Throughput::Elements(100));

    // 0% load (baseline)
    group.bench_function("load_0_percent", |b| {
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

        b.iter(|| {
            for _ in 0..100 {
                simulate_cpu_load(0.0, 100);
                let _ = sensor.run_cycle();
                let _ = actuator.run_cycle();
            }
        })
    });

    // 30% load
    group.bench_function("load_30_percent", |b| {
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

        b.iter(|| {
            for _ in 0..100 {
                simulate_cpu_load(0.3, 100);
                let _ = sensor.run_cycle();
                let _ = actuator.run_cycle();
            }
        })
    });

    // 60% load
    group.bench_function("load_60_percent", |b| {
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

        b.iter(|| {
            for _ in 0..100 {
                simulate_cpu_load(0.6, 100);
                let _ = sensor.run_cycle();
                let _ = actuator.run_cycle();
            }
        })
    });

    // 80% load
    group.bench_function("load_80_percent", |b| {
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

        b.iter(|| {
            for _ in 0..100 {
                simulate_cpu_load(0.8, 100);
                let _ = sensor.run_cycle();
                let _ = actuator.run_cycle();
            }
        })
    });

    group.finish();
}

// ============================================================================
// CRITERION GROUPS
// ============================================================================

// ACTUAL SYSTEM benchmarks (tests real code from main.rs)
criterion_group!(
    actual_system_benches,
    bench_sensor_module_run_cycle,
    bench_actuator_module_run_cycle,
    bench_complete_system_cycle,
    bench_multithreaded_system_end_to_end
);

// Component timing benchmarks
criterion_group!(
    component_benches,
    bench_sensor_generation,
    bench_data_processing,
    bench_pid_update,
    bench_channel_transmission,
    bench_shared_resources
);

// Sync comparison benchmarks
criterion_group!(
    sync_benches,
    bench_sync_primitives
);

// Shared resource contention benchmarks
criterion_group!(
    shared_resource_contention_benches,
    bench_shared_resource_contention
);

// Priority inversion simulation benchmarks
criterion_group!(
    priority_inversion_benches,
    bench_priority_inversion_simulated
);

// Load impact benchmarks
criterion_group!(
    load_benches,
    bench_cpu_load_impact
);

criterion_main!(
    actual_system_benches,
    component_benches,
    sync_benches,
    shared_resource_contention_benches,
    priority_inversion_benches,
    load_benches
);
