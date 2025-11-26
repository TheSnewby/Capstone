use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::Arc;
use tokio::net::UdpSocket;
use tokio::sync::{broadcast, RwLock};
use tracing::Instrument;
use uuid::Uuid;

/// 3D position in meters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Position {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
/// orientation in radians
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Velocity {
    pub vx: f64,
	pub vy: f64,
    pub vz: f64,
}

/// state of a single UAV sent to WebSocket clients
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UavState {
    pub id: Uuid,
    pub position: Position,
    pub velocity: Velocity,
    pub timestamp: DateTime<Utc>,
}

/// Telemetry sent over UDP as JSON from telemetry_encoder.cpp
#[derive(Debug, Clone, Deserialize)]
pub struct TelemetryFrame {
    pub id: Uuid,
    pub position: Position,
    pub velocity: Velocity,
	pub timestamp: DateTime<Utc>,
}

/// Swarm Behavior settings sent from the UI over WebSocket
#[derive(Debug, Clone, Deserialize)]
pub struct SwarmSettings {
    pub cohesion: f64,
    pub separation: f64,
    pub alignment: f64,
    #[serde(rename = "maxSpeed")]
    pub max_speed: f64,
    #[serde(rename = "targetAltitude")]
    pub target_altitude: f64,
}

/// state of UAV swarm
#[derive(Clone)]
pub struct SwarmState {
    inner: Arc<RwLock<HashMap<Uuid, UavState>>>,
    settings: Arc<RwLock<SwarmSettings>>,
    last_command: Arc<RwLock<Option<serde_json::Value>>>,
}

impl SwarmState {
    pub fn new() -> Self {
        SwarmState {
            inner: Arc::new(RwLock::new(HashMap::new())),
            settings: Arc::new(RwLock::new(SwarmSettings {
                cohesion: 1.0,
                separation: 1.0,
                alignment: 1.0,
                max_speed: 10.0,
                target_altitude: 10.0,
            })),
            last_command: Arc::new(RwLock::new(None)),
        }
    }

    pub async fn upsert_uav(&self, state: UavState) {
        let mut map = self.inner.write().await;
        map.insert(state.id, state);
    }

    pub async fn get_uav(&self, id: Uuid) -> Option<UavState> {
        let map = self.inner.read().await;
        map.get(&id).cloned()
    }

    pub async fn list_uavs(&self) -> Vec<UavState> {
        let map = self.inner.read().await;
        map.values().cloned().collect()
    }

    /// command from UI to control swarm behavior
    pub async fn apply_command(&self, cmd: serde_json::Value) {
        {
            let mut last = self.last_command.write().await;
            *last = Some(cmd.clone());
        }

        tracing::info!(swarm_command_from_ui = %cmd);
    }

    /// apply swarm behavior settings from UI
    pub async fn apply_settings(&self, settings: SwarmSettings) {
        {
            let mut current = self.settings.write().await;
            *current = settings.clone();
        }

        tracing::info!(
            cohesion = settings.cohesion,
            separation = settings.separation,
            alignment = settings.alignment,
            max_speed = settings.max_speed,
            target_altitude = settings.target_altitude,
            updated_swarm_settings_from_ui = true,
        );
    }

    /// get the current swarm settings snapshot
    pub async fn current_settings(&self) -> SwarmSettings {
        let s = self.settings.read().await;
        s.clone()
    }

    /// get the last command received from the UI, if any
    pub async fn last_command(&self) -> Option<serde_json::Value> {
        let c = self.last_command.read().await;
        c.clone()
    }
}

/// shared handles used by both UDP listener and WebSocket server
#[derive(Clone)]
pub struct TelemetryShared {
    pub swarm: SwarmState,
    pub tx: broadcast::Sender<UavState>,
}

/// start UDP listener that receives telemetry frames, updates swarm state,
/// and broadcasts each update to WebSocket clients
pub async fn run_udp_listener(bind_addr: SocketAddr, shared: TelemetryShared) {
    let socket = match UdpSocket::bind(bind_addr).await {
        Ok(s) => s,
        Err(err) => {
            tracing::error!("Failed to bind UDP socket at {}: {}", bind_addr, err);
            return;
        }
    };

    tracing::info!("UDP telemetry listener bound at {}", bind_addr);

    loop {
        let mut buf = vec![0u8; 2048];
        let span = tracing::info_span!("udp_recv");
        let shared = shared.clone();
        let socket = &socket;
        let addr = bind_addr;

        let fut = async move {
            let (len, src) = match socket.recv_from(&mut buf).await {
                Ok(res) => res,
                Err(err) => {
                    tracing::warn!("Error receiving UDP packet: {}", err);
                    return;
                }
            };

            let data = &buf[..len];
            tracing::debug!("Received {} bytes from {}", len, src);

            match decode_frame(data) {
                Ok(uav) => {
                    let id = uav.id;
                    shared.swarm.upsert_uav(uav.clone()).await;

                    // broadcast the update; ignore if there are no listeners.
                    if let Err(e) = shared.tx.send(uav) {
                        tracing::debug!(
                            "No WebSocket subscribers for update {}: {}",
                            id,
                            e
                        );
                    }
                }
                Err(err) => {
                    tracing::warn!(
                        "Failed to decode telemetry frame from {}: {}",
                        src,
                        err
                    );
                }
            }
        };

        fut.instrument(span).await;
        // loop continues
        tracing::trace!("Waiting for next UDP telemetry packet on {}", addr);
    }
}

/// decode a UDP datagram into a `UavState`.
///
/// (for now) this expects a single JSON object matching `TelemetryFrame`
fn decode_frame(data: &[u8]) -> Result<UavState, serde_json::Error> {
    let frame: TelemetryFrame = serde_json::from_slice(data)?;
    Ok(UavState {
        id: frame.id,
        position: frame.position,
        velocity: frame.velocity,
        timestamp: frame.timestamp,
    })
}
