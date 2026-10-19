use athena_core::{bus::SensoryBus, *};
use athena_memory::{EpisodeKind, EpisodicMemory, WorkingBuffer};
use std::{
    fs,
    path::PathBuf,
    sync::atomic::{AtomicUsize, Ordering},
};

struct Temp(PathBuf);
impl Temp {
    fn new() -> Self {
        static COUNTER: AtomicUsize = AtomicUsize::new(0);
        let path = std::env::temp_dir().join(format!(
            "athena-{}-{}.journal",
            std::process::id(),
            COUNTER.fetch_add(1, Ordering::Relaxed)
        ));
        assert!(!path.exists());
        Self(path)
    }
}
impl Drop for Temp {
    fn drop(&mut self) {
        let _ = fs::remove_file(&self.0);
    }
}

#[test]
fn journal_restarts_with_causal_links_and_unicode_intact() {
    let temp = Temp::new();
    let expected;
    {
        let mut memory = EpisodicMemory::open(&temp.0).unwrap();
        let parent = memory
            .record(
                1,
                EpisodeKind::Reflex,
                1.0,
                None,
                "hot pipe → withdraw\nleft arm",
            )
            .unwrap();
        memory
            .record(2, EpisodeKind::DriveChange, 0.5, Some(parent), "caution")
            .unwrap();
        expected = memory.episodes().clone();
    }
    let mut memory = EpisodicMemory::open(&temp.0).unwrap();
    assert_eq!(memory.episodes(), &expected);
    assert_eq!(
        memory
            .record(3, EpisodeKind::Observation, 0.1, Some(2), "cooled")
            .unwrap(),
        3
    );
}

#[test]
fn corruption_is_reported_without_modifying_existing_data() {
    let temp = Temp::new();
    let mut memory = EpisodicMemory::open(&temp.0).unwrap();
    memory
        .record(1, EpisodeKind::Reflex, 1.0, None, "hazard")
        .unwrap();
    drop(memory);
    let mut bytes = fs::read(&temp.0).unwrap();
    bytes[20] ^= 1;
    fs::write(&temp.0, &bytes).unwrap();
    assert!(EpisodicMemory::open(&temp.0).is_err());
    assert_eq!(fs::read(&temp.0).unwrap(), bytes);
}

#[test]
fn a_torn_append_is_reported_without_silent_history_loss() {
    let temp = Temp::new();
    let mut memory = EpisodicMemory::open(&temp.0).unwrap();
    memory
        .record(1, EpisodeKind::Observation, 0.5, None, "event")
        .unwrap();
    drop(memory);
    let mut bytes = fs::read(&temp.0).unwrap();
    bytes.pop();
    fs::write(&temp.0, &bytes).unwrap();
    assert!(EpisodicMemory::open(&temp.0).is_err());
    assert_eq!(fs::read(&temp.0).unwrap(), bytes);
}

#[test]
fn future_causes_nan_salience_and_regressing_time_are_rejected() {
    let temp = Temp::new();
    let mut memory = EpisodicMemory::open(&temp.0).unwrap();
    assert!(
        memory
            .record(1, EpisodeKind::Reflex, 1.0, Some(1), "self cause")
            .is_err()
    );
    assert!(
        memory
            .record(1, EpisodeKind::Reflex, f32::NAN, None, "bad salience")
            .is_err()
    );
    memory
        .record(2, EpisodeKind::Observation, 0.5, None, "valid")
        .unwrap();
    assert!(
        memory
            .record(1, EpisodeKind::Observation, 0.5, None, "past")
            .is_err()
    );
    assert_eq!(memory.episodes().len(), 1);
}

#[test]
fn working_memory_respects_both_time_and_capacity_limits() {
    let mut buffer = WorkingBuffer::new(4, 500);
    for tick in 0..10 {
        buffer
            .push(BodyFrame {
                sampled_at_us: tick * 250,
                ..BodyFrame::REST
            })
            .unwrap();
    }
    assert_eq!(buffer.len(), 3);
    assert_eq!(buffer.oldest().unwrap().sampled_at_us, 1_750);
    assert!(buffer.push(BodyFrame::REST).is_err());
}

#[test]
fn replay_requires_a_healthy_docked_idle_body_and_ranks_salient_experiences() {
    let temp = Temp::new();
    let mut memory = EpisodicMemory::open(&temp.0).unwrap();
    memory
        .record(0, EpisodeKind::Observation, 0.1, None, "wall")
        .unwrap();
    memory
        .record(1, EpisodeKind::Reflex, 1.0, Some(1), "hot pipe")
        .unwrap();
    memory
        .record(2, EpisodeKind::Watchdog, 0.8, Some(2), "advisor outage")
        .unwrap();
    let mut bus = SensoryBus::<4>::new();
    let (_, mut ingress) = bus.split();
    let mut kernel = SpinalKernel::new(BodyConfig::fixture()).unwrap();
    let mut body = BodyFrame {
        docked: true,
        task_demand: 0.0,
        ..BodyFrame::REST
    };
    let output = kernel.tick(0, &body, Some(MotorIntent::hold(1, 0)), &mut ingress);
    assert_eq!(
        memory
            .propose_replay(&body, &output, 8)
            .unwrap()
            .episode_ids,
        [2, 3]
    );
    body.docked = false;
    assert!(memory.propose_replay(&body, &output, 8).is_none());
    body.docked = true;
    body.joints[0].temperature_c = 75.0;
    assert!(memory.propose_replay(&body, &output, 8).is_none());
}
