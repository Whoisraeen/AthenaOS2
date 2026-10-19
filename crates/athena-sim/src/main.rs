use athena_core::{
    bus::{SensoryBus, Snapshot},
    drive, *,
};
use athena_memory::{EpisodeKind, EpisodicMemory, WorkingBuffer};
use std::{env, fs, io, path::PathBuf};

fn main() {
    if let Err(error) = run() {
        eprintln!("Athena: {error}");
        std::process::exit(1);
    }
}

fn run() -> io::Result<()> {
    let mut args = env::args().skip(1);
    let command = args.next().unwrap_or_else(|| "demo".to_owned());
    if command == "--help" || command == "help" {
        println!(
            "Athena embodied runtime\n\n  cargo run -p athena-sim -- demo [journal-path]\n  cargo run -p athena-sim -- inspect <journal-path>\n\nDemo: task -> withdrawal -> cortex outage -> recharge -> replay proposal."
        );
        return Ok(());
    }
    if command != "demo" && command != "inspect" {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "expected demo or inspect; use --help",
        ));
    }
    let path = match args.next() {
        Some(path) => PathBuf::from(path),
        None if command == "inspect" => {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "inspect requires a journal path",
            ));
        }
        None => {
            fs::create_dir_all("runs")?;
            PathBuf::from("runs/demo.athena")
        }
    };
    if args.next().is_some() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "too many arguments",
        ));
    }
    if command == "inspect" && !path.is_file() {
        return Err(io::Error::new(
            io::ErrorKind::NotFound,
            "journal does not exist",
        ));
    }
    let mut memory = EpisodicMemory::open(&path)?;
    if command == "inspect" {
        for e in memory.episodes() {
            println!(
                "#{:<4} {:>10}us {:?} salience={:.2} parent={:?}  {}",
                e.id, e.timestamp_us, e.kind, e.salience, e.parent_id, e.summary
            );
        }
        return Ok(());
    }
    let offset = memory.last_timestamp_us().saturating_add(1);
    let mut working = WorkingBuffer::new(20_001, 5_000_000);
    let mut bus = SensoryBus::<64>::new();
    let (mut sensors, mut ingress) = bus.split();
    let mut snapshots = Snapshot::new(BodyFrame::REST);
    let (mut body_writer, mut body_reader) = snapshots.split();
    let mut kernel = SpinalKernel::new(BodyConfig::fixture()).expect("valid fixture");
    let mut body = BodyFrame::REST;
    let mut last_drive = None;
    let mut last_mode = None;
    let mut last_watchdog = false;
    let mut parent = None;
    let mut replay_done = false;
    println!("ATHENA / embodied runtime / fixture body, logical time");
    println!("Journal: {}", path.display());
    println!(" time(ms)  spinal / workspace transition");
    for tick in 0..2_400_u64 {
        let now = tick * TICK_US;
        body.sampled_at_us = now;
        if tick == 40 {
            sensors
                .send(Spike {
                    timestamp_us: now,
                    source_id: 7,
                    intensity: u16::MAX,
                    limb_mask: Limb::LeftArm.mask(),
                    modality: Modality::Nociception,
                })
                .expect("fixture capacity");
        }
        if tick >= 1_600 {
            body.battery_soc = 0.12;
        }
        if tick >= 2_000 {
            body.battery_soc = 0.8;
            body.docked = true;
            body.task_demand = 0.0;
        }
        let intent = if tick % 40 == 0 && !(1_000..1_400).contains(&tick) {
            Some(MotorIntent {
                goal: MotorGoal::JointPose([0.3; JOINTS]),
                ..MotorIntent::hold(tick + 1, now)
            })
        } else {
            None
        };
        let output = kernel.tick(now, &body, intent, &mut ingress);
        if output.faults.any() {
            return Err(io::Error::other(format!(
                "unexpected spinal fault: {:?}",
                output.faults
            )));
        }
        body_writer.publish(body);
        working.push(body)?;
        let mode = output.modes[0];
        if last_mode != Some(mode) {
            println!(
                " {:>8.2}  left arm -> {mode:?}; torque={:.2} Nm",
                now as f64 / 1_000.0,
                output.torque_nm[0]
            );
            parent = Some(memory.record(
                offset + now,
                if mode == ControlMode::Withdrawal {
                    EpisodeKind::Reflex
                } else {
                    EpisodeKind::Observation
                },
                if mode == ControlMode::Withdrawal {
                    1.0
                } else {
                    0.25
                },
                parent,
                format!(
                    "left arm {mode:?}; mask=0x{:02x}",
                    output.active_reflex_mask
                ),
            )?);
            last_mode = Some(mode);
        }
        if output.watchdog_expired != last_watchdog {
            println!(
                " {:>8.2}  cortical watchdog expired={}",
                now as f64 / 1_000.0,
                output.watchdog_expired
            );
            parent = Some(memory.record(
                offset + now,
                EpisodeKind::Watchdog,
                0.9,
                parent,
                format!("cortical watchdog expired={}", output.watchdog_expired),
            )?);
            last_watchdog = output.watchdog_expired;
        }
        // The advisor normally runs every 10ms; spinal notification preempts that cadence.
        if tick % 40 == 0 || output.notify_cortex {
            let snapshot = body_reader.latest();
            let decision = drive::evaluate(&snapshot, &output);
            if last_drive != Some(decision.drive) {
                println!(
                    " {:>8.2}  workspace -> {:?}, urgency={:.2}",
                    now as f64 / 1_000.0,
                    decision.drive,
                    decision.urgency
                );
                parent = Some(memory.record(
                    offset + now,
                    EpisodeKind::DriveChange,
                    decision.urgency,
                    parent,
                    format!("homeostatic drive {:?}", decision.drive),
                )?);
                last_drive = Some(decision.drive);
            }
            if !replay_done {
                if let Some(proposal) = memory.propose_replay(&snapshot, &output, 8) {
                    println!(
                        " {:>8.2}  replay proposal: episodes {:?}; model weights unchanged",
                        now as f64 / 1_000.0,
                        proposal.episode_ids
                    );
                    replay_done = true;
                }
            }
        }
    }
    println!(
        "\nCompleted 2,400 logical spinal ticks; {} working frames; {} resident episodes.",
        working.len(),
        memory.episodes().len()
    );
    println!("No wall-clock deadline or physical stability claim. Replay is a proposal only.");
    Ok(())
}
