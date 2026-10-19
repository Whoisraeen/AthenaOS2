//! Host-side episodic journal and replay proposals. Never called from a spinal tick.
use athena_core::{
    BodyFrame, MotorOutput,
    drive::{self, Drive},
};
use std::{
    collections::VecDeque,
    fs::{File, OpenOptions},
    io::{self, Read, Seek, SeekFrom, Write},
    path::Path,
};

const MAGIC: &[u8; 8] = b"ATHENA01";
const MAX_PAYLOAD: usize = 4_096;
pub const EPISODE_CAPACITY: usize = 4_096;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum EpisodeKind {
    Observation = 0,
    Reflex = 1,
    Watchdog = 2,
    DriveChange = 3,
    Fault = 4,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Episode {
    pub id: u64,
    pub timestamp_us: u64,
    pub parent_id: Option<u64>,
    pub kind: EpisodeKind,
    pub salience: f32,
    pub summary: String,
}

pub struct WorkingBuffer {
    frames: VecDeque<BodyFrame>,
    capacity: usize,
    retention_us: u64,
}

impl WorkingBuffer {
    pub fn new(capacity: usize, retention_us: u64) -> Self {
        assert!(capacity > 0);
        Self {
            frames: VecDeque::with_capacity(capacity),
            capacity,
            retention_us,
        }
    }

    pub fn push(&mut self, frame: BodyFrame) -> io::Result<()> {
        if !frame.is_valid()
            || self
                .frames
                .back()
                .is_some_and(|last| frame.sampled_at_us < last.sampled_at_us)
        {
            return Err(invalid("invalid or regressing working-memory frame"));
        }
        while self
            .frames
            .front()
            .is_some_and(|first| frame.sampled_at_us - first.sampled_at_us > self.retention_us)
            || self.frames.len() == self.capacity
        {
            self.frames.pop_front();
        }
        self.frames.push_back(frame);
        Ok(())
    }

    pub fn len(&self) -> usize {
        self.frames.len()
    }
    pub fn is_empty(&self) -> bool {
        self.frames.is_empty()
    }
    pub fn oldest(&self) -> Option<&BodyFrame> {
        self.frames.front()
    }
}

pub struct EpisodicMemory {
    file: File,
    episodes: VecDeque<Episode>,
    next_id: u64,
    last_timestamp: u64,
    /// An append/sync failure requires reopening and checking the journal.
    writable: bool,
}

#[derive(Clone, Debug)]
pub struct ReplayProposal {
    pub episode_ids: Vec<u64>,
    pub total_salience: f32,
}

impl EpisodicMemory {
    /// Replays a checksummed, versioned append-only journal. Corruption or a torn
    /// tail is reported without modifying the existing file.
    pub fn open(path: impl AsRef<Path>) -> io::Result<Self> {
        let mut file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(path)?;
        if file.metadata()?.len() == 0 {
            file.write_all(MAGIC)?;
            file.sync_data()?;
        }
        file.seek(SeekFrom::Start(0))?;
        let mut magic = [0; 8];
        file.read_exact(&mut magic)?;
        if &magic != MAGIC {
            return Err(invalid("unknown Athena journal version"));
        }
        let mut memory = Self {
            file,
            episodes: VecDeque::new(),
            next_id: 1,
            last_timestamp: 0,
            writable: true,
        };
        loop {
            let mut first = [0; 1];
            if memory.file.read(&mut first)? == 0 {
                break;
            }
            let mut length = [0; 4];
            length[0] = first[0];
            memory.file.read_exact(&mut length[1..])?;
            let length = u32::from_le_bytes(length) as usize;
            if !(29..=MAX_PAYLOAD).contains(&length) {
                return Err(invalid("invalid journal record length"));
            }
            let mut bytes = vec![0; length];
            memory.file.read_exact(&mut bytes)?;
            let mut checksum = [0; 4];
            memory.file.read_exact(&mut checksum)?;
            if u32::from_le_bytes(checksum) != checksum32(&bytes) {
                return Err(invalid("journal checksum mismatch"));
            }
            let episode = decode(&bytes)?;
            memory.validate(&episode)?;
            memory.remember(episode);
        }
        Ok(memory)
    }

    fn validate(&self, episode: &Episode) -> io::Result<()> {
        if episode.id != self.next_id
            || episode.id == u64::MAX
            || episode.timestamp_us < self.last_timestamp
            || !episode.salience.is_finite()
            || !(0.0..=1.0).contains(&episode.salience)
            || episode.parent_id.is_some_and(|p| p == 0 || p >= episode.id)
            || episode.summary.len() > MAX_PAYLOAD - 29
        {
            return Err(invalid(
                "invalid episode sequence, timestamp, salience, parent, or size",
            ));
        }
        Ok(())
    }

    fn remember(&mut self, episode: Episode) {
        self.next_id = episode.id + 1;
        self.last_timestamp = episode.timestamp_us;
        if self.episodes.len() == EPISODE_CAPACITY {
            self.episodes.pop_front();
        }
        self.episodes.push_back(episode);
    }

    pub fn record(
        &mut self,
        timestamp_us: u64,
        kind: EpisodeKind,
        salience: f32,
        parent_id: Option<u64>,
        summary: impl Into<String>,
    ) -> io::Result<u64> {
        if !self.writable {
            return Err(invalid("reopen journal after write failure"));
        }
        let episode = Episode {
            id: self.next_id,
            timestamp_us,
            kind,
            salience,
            parent_id,
            summary: summary.into(),
        };
        self.validate(&episode)?;
        let bytes = encode(&episode);
        let result = (|| {
            self.file.seek(SeekFrom::End(0))?;
            self.file.write_all(&(bytes.len() as u32).to_le_bytes())?;
            self.file.write_all(&bytes)?;
            self.file.write_all(&checksum32(&bytes).to_le_bytes())?;
            self.file.sync_data()
        })();
        if let Err(error) = result {
            self.writable = false;
            return Err(error);
        }
        let id = episode.id;
        self.remember(episode);
        Ok(id)
    }

    pub fn episodes(&self) -> &VecDeque<Episode> {
        &self.episodes
    }
    pub fn last_timestamp_us(&self) -> u64 {
        self.last_timestamp
    }

    /// Produces a replay curriculum only; it never changes weights or motor policy.
    pub fn propose_replay(
        &self,
        body: &BodyFrame,
        spinal: &MotorOutput,
        limit: usize,
    ) -> Option<ReplayProposal> {
        if drive::evaluate(body, spinal).drive != Drive::Consolidate {
            return None;
        }
        let mut candidates: Vec<_> = self.episodes.iter().filter(|e| e.salience >= 0.5).collect();
        candidates.sort_by(|a, b| {
            b.salience
                .total_cmp(&a.salience)
                .then_with(|| b.id.cmp(&a.id))
        });
        candidates.truncate(limit.min(64));
        Some(ReplayProposal {
            episode_ids: candidates.iter().map(|e| e.id).collect(),
            total_salience: candidates.iter().map(|e| e.salience).sum(),
        })
    }
}

fn invalid(message: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message)
}

// FNV-1a detects accidental damage; it does not authenticate untrusted journals.
fn checksum32(bytes: &[u8]) -> u32 {
    bytes.iter().fold(2_166_136_261_u32, |sum, byte| {
        (sum ^ u32::from(*byte)).wrapping_mul(16_777_619)
    })
}

fn encode(episode: &Episode) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(29 + episode.summary.len());
    bytes.extend_from_slice(&episode.id.to_le_bytes());
    bytes.extend_from_slice(&episode.timestamp_us.to_le_bytes());
    bytes.extend_from_slice(&episode.parent_id.unwrap_or(0).to_le_bytes());
    bytes.push(episode.kind as u8);
    bytes.extend_from_slice(&episode.salience.to_le_bytes());
    bytes.extend_from_slice(episode.summary.as_bytes());
    bytes
}

fn decode(bytes: &[u8]) -> io::Result<Episode> {
    let id = u64::from_le_bytes(bytes[0..8].try_into().unwrap());
    let timestamp_us = u64::from_le_bytes(bytes[8..16].try_into().unwrap());
    let parent = u64::from_le_bytes(bytes[16..24].try_into().unwrap());
    let kind = match bytes[24] {
        0 => EpisodeKind::Observation,
        1 => EpisodeKind::Reflex,
        2 => EpisodeKind::Watchdog,
        3 => EpisodeKind::DriveChange,
        4 => EpisodeKind::Fault,
        _ => return Err(invalid("unknown episode kind")),
    };
    let salience = f32::from_le_bytes(bytes[25..29].try_into().unwrap());
    let summary = std::str::from_utf8(&bytes[29..])
        .map_err(|_| invalid("episode is not UTF-8"))?
        .to_owned();
    Ok(Episode {
        id,
        timestamp_us,
        parent_id: (parent != 0).then_some(parent),
        kind,
        salience,
        summary,
    })
}
