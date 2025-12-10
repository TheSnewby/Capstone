use crate::telemetry::{Goal, ObstacleType, SwarmSettings, TelemetryShared, UavState};
use axum::{
    extract::{
        ws::{Message, WebSocket, WebSocketUpgrade},
        Path, State,
    },
    http::StatusCode,
    response::IntoResponse,
    routing::get,
    Json, Router,
};
use futures::{sink::SinkExt, stream::StreamExt};
use serde::Deserialize;
use std::env;
use tokio::net::UdpSocket;
use tracing::Instrument;

fn sim_addr() -> String {
    env::var("SKYWEAVE_SIM_ADDR").unwrap_or_else(|_| "127.0.0.1:6001".to_string())
}

/// message sent from the UI or bridge over WebSocket
#[derive(Debug, Deserialize)]
#[serde(tag = "type", content = "payload")]
enum ClientMessage {
    #[serde(rename = "command")]
    Command(serde_json::Value),
    #[serde(rename = "swarm_settings")]
    SwarmSettings(SwarmSettings),
    #[serde(rename = "telemetry")]
    Telemetry(UavState),
}

/// build the Axum router that serves:
/// - GET /health   : simple health check
/// - GET /uavs     : snapshot of current swarm state
/// - GET /uavs/:id : get a single UAV by id
/// - GET /ws       : WebSocket streaming telemetry updates
pub fn router(shared: TelemetryShared) -> Router {
    Router::new()
        .route("/health", get(health))
        .route("/uavs", get(list_uavs))
        .route("/uavs/:id", get(get_uav))
        .route("/ws", get(ws_handler))
        .with_state(shared)
}

async fn health() -> &'static str {
    "ok"
}

async fn list_uavs(State(shared): State<TelemetryShared>) -> Json<Vec<UavState>> {
    let list = shared.swarm.list_uavs().await;
    Json(list)
}

async fn get_uav(State(shared): State<TelemetryShared>, Path(id): Path<u64>) -> impl IntoResponse {
    match shared.swarm.get_uav(id).await {
        Some(uav) => Json(uav).into_response(),
        None => (StatusCode::NOT_FOUND, "not found").into_response(),
    }
}

async fn ws_handler(
    State(shared): State<TelemetryShared>,
    ws: WebSocketUpgrade,
) -> impl IntoResponse {
    ws.on_upgrade(move |socket| handle_ws(socket, shared))
}

async fn send_formation_command_to_sim(formation: &str) {
    let msg = match formation {
        "line" => Some("line"),
        "vee" => Some("vee"),
        "circle" => Some("circle"),
        _ => None,
    };

    let Some(msg) = msg else {
        tracing::warn!("Unknown formation command from client: {}", formation);
        return;
    };

    let addr = sim_addr();

    match UdpSocket::bind("[::]:0").await {
        Ok(socket) => {
            tracing::info!("sending formation UDP to sim {}: {}", addr, msg);
            if let Err(err) = socket.send_to(msg.as_bytes(), &addr).await {
                tracing::warn!("Failed to send formation command to sim: {}", err);
            }
        }
        Err(err) => {
            tracing::warn!("Failed to bind UDP socket for formation command: {}", err);
        }
    }
}

/// Send a generic control command (e.g., leader movement / altitude change) to the simulator over UDP.
async fn send_control_command_to_sim(command: &str) {
    let addr = sim_addr();

    match UdpSocket::bind("[::]:0").await {
        Ok(socket) => {
            tracing::info!("sending control UDP to sim {}: {}", addr, command);
            if let Err(err) = socket.send_to(command.as_bytes(), &addr).await {
                tracing::warn!(
                    "Failed to send control command to sim: {} (cmd = {})",
                    err,
                    command
                );
            }
        }
        Err(err) => {
            tracing::warn!(
                "Failed to bind UDP socket for control command `{}`: {}",
                command,
                err
            );
        }
    }
}

/// Send updated swarm settings to the simulator over UDP as JSON.
async fn send_swarm_settings_to_sim(settings: &SwarmSettings) {
    let addr = sim_addr();

    let payload = serde_json::json!({
        "type": "swarm_settings",
        "payload": {
            "cohesion": settings.cohesion,
            "separation": settings.separation,
            "alignment": settings.alignment,
            // C++ sim expects snake_case keys
            "max_speed": settings.max_speed,
            "target_altitude": settings.target_altitude,
            "swarm_size": settings.swarm_size,
        }
    });

    let msg = match serde_json::to_string(&payload) {
        Ok(m) => m,
        Err(err) => {
            tracing::warn!(
                "Failed to serialize swarm_settings for sim: {} (settings = {:?})",
                err,
                settings
            );
            return;
        }
    };

    match UdpSocket::bind("[::]:0").await {
        Ok(socket) => {
            tracing::info!("sending swarm_settings UDP to sim {}: {}", addr, msg);
            if let Err(err) = socket.send_to(msg.as_bytes(), &addr).await {
                tracing::warn!("Failed to send swarm_settings to sim: {}", err);
            }
        }
        Err(err) => {
            tracing::warn!(
                "Failed to bind UDP socket for swarm_settings command: {}",
                err
            );
        }
    }
}

/// handle a single WebSocket client connection
async fn handle_ws(socket: WebSocket, shared: TelemetryShared) {
    let span = tracing::info_span!("ws_client");
    async move {
        let (mut sender, mut receiver) = socket.split();

        // subscribe to telemetry updates
        let mut rx = shared.tx.subscribe();

        // send a snapshot of all UAVs on connect
        if let Ok(initial) = serde_json::to_string(&shared.swarm.list_uavs().await) {
            let _ = sender.send(Message::Text(initial)).await;
        }

        // send environmental obstacles on connect (if any)
        {
            let obstacles_guard = shared.obstacles.read().await;
            let goal_guard = shared.goal.read().await;

            if !obstacles_guard.is_empty() || goal_guard.is_some() {
                let env_msg = serde_json::json!({
                    "type": "environment",
                    "payload": {
                        "obstacles": &*obstacles_guard,
                        "goal": &*goal_guard,
                    }
                });

                if let Ok(txt) = serde_json::to_string(&env_msg) {
                    let _ = sender.send(Message::Text(txt)).await;
                }
            }
        }

        loop {
            tokio::select! {
                maybe_msg = receiver.next() => {
                    match maybe_msg {
                        Some(Ok(msg)) => {
                            match msg {
                                Message::Text(text) => {
                                    // First, handle environment messages coming from the UDP→WS bridge:
                                    // { "type": "environment", "obstacles": [ ... ] }
                                    if let Ok(raw) = serde_json::from_str::<serde_json::Value>(&text) {
                                        if raw.get("type").and_then(|v| v.as_str()) == Some("environment") {
                                            let mut updated_obstacles = false;
                                            if let Some(obstacles_val) = raw.get("obstacles") {
                                                match serde_json::from_value::<Vec<ObstacleType>>(obstacles_val.clone()) {
                                                    Ok(obstacles) => {
                                                        let count = obstacles.len();
                                                        {
                                                            let mut guard = shared.obstacles.write().await;
                                                            *guard = obstacles;
                                                        }
                                                        tracing::info!(
                                                            "ws_recv: updated environment from bridge with {} obstacles",
                                                            count
                                                        );
                                                        updated_obstacles = true;
                                                    }
                                                    Err(err) => {
                                                        tracing::warn!(
                                                            "Failed to decode environment obstacles from WS: {} (raw: {})",
                                                            err,
                                                            text
                                                        );
                                                    }
                                                }
                                            }
                                            if let Some(goal_val) = raw.get("goal") {
                                                match serde_json::from_value(goal_val.clone()) {
                                                    Ok(goal) => {
                                                        let mut guard = shared.goal.write().await;
                                                        *guard = goal;
                                                        tracing::info!("ws_recv: updated goal from bridge");
                                                    }
                                                    Err(err) => {
                                                        tracing::warn!("Failed to decode goal from WS: {} (raw: {})", err, text);
                                                    }
                                                }
                                            }
                                            if !updated_obstacles && !raw.get("goal").is_some() {
                                                tracing::warn!("Environment message missing obstacles/goal fields: {}", text);
                                            }

                                            // We've fully handled this message; skip normal ClientMessage parsing.
                                            continue;
                                        }
                                    }

                                    // Non-environment messages use the normal typed ClientMessage flow.
                                    match serde_json::from_str::<ClientMessage>(&text) {
                                        Ok(ClientMessage::Command(cmd)) => {
                                            tracing::info!("swarm_command_from_ui={}", cmd);

                                            if let Some(cmd_type) = cmd.get("type").and_then(|v| v.as_str()) {
                                                match cmd_type {
                                                    // formation commands from UI
                                                    "formation" => {
                                                        if let Some(mode) = cmd.get("mode").and_then(|v| v.as_str()) {
                                                            // mode: "line" | "vee" | "circle"
                                                            send_formation_command_to_sim(mode).await;
                                                        } else {
                                                            tracing::warn!("formation command missing `mode` field: {:?}", cmd);
                                                        }
                                                    }

                                                    // leader movement
                                                    "move_leader" => {
                                                        if let Some(direction) = cmd.get("direction").and_then(|v| v.as_str()) {
                                                            let command = format!("move_leader {}", direction.to_lowercase());
                                                            send_control_command_to_sim(&command).await;
                                                        } else {
                                                            tracing::warn!("move_leader command missing `direction` field: {:?}", cmd);
                                                        }
                                                    }

                                                    // altitude change
                                                    "altitude_change" => {
                                                        if let Some(amount) = cmd.get("amount").and_then(|v| v.as_f64()) {
                                                            let command = format!("altitude_change {}", amount);
                                                            send_control_command_to_sim(&command).await;
                                                        } else {
                                                            tracing::warn!("altitude_change command missing `amount` field: {:?}", cmd);
                                                        }
                                                    }

                                                    // flight mode toggle: autonomous vs controlled
                                                    "flight_mode" => {
                                                        if let Some(mode) = cmd.get("mode").and_then(|v| v.as_str()) {
                                                            tracing::info!("flight_mode command from UI: {}", mode);
                                                            // Forward to sim so it can enable/disable autopilot on the leader.
                                                            let command = format!("flight_mode {}", mode.to_lowercase());
                                                            send_control_command_to_sim(&command).await;
                                                        } else {
                                                            tracing::warn!("flight_mode command missing `mode` field: {:?}", cmd);
                                                        }
                                                    }

                                                    // anything else: apply locally
                                                    _ => {
                                                        shared.swarm.apply_command(cmd).await;
                                                    }
                                                }
                                            } else if let Some(formation) = cmd.get("formation").and_then(|v| v.as_str()) {
                                                // legacy shape: { "formation": "line" }
                                                send_formation_command_to_sim(formation).await;
                                            } else {
                                                shared.swarm.apply_command(cmd).await;
                                            }
                                        }

                                        Ok(ClientMessage::SwarmSettings(settings)) => {
                                            // apply settings to the in-memory swarm model
                                            let settings_clone = settings.clone();
                                            shared.swarm.apply_settings(settings).await;

                                            // forward updated settings to the simulator over UDP so the
                                            // C++ side can update its boids weights in real time
                                            send_swarm_settings_to_sim(&settings_clone).await;

                                            // echo settings update back to this WS client for UI sync
                                            let payload = serde_json::json!({
                                                "type": "settings_update",
                                                "payload": {
                                                    "cohesion": settings_clone.cohesion,
                                                    "separation": settings_clone.separation,
                                                    "alignment": settings_clone.alignment,
                                                    "max_speed": settings_clone.max_speed,
                                                    "target_altitude": settings_clone.target_altitude,
                                                }
                                            });

                                            if let Ok(txt) = serde_json::to_string(&payload) {
                                                let _ = sender.send(Message::Text(txt)).await;
                                            }
                                        }

                                        Ok(ClientMessage::Telemetry(uav)) => {
                                            let id = uav.id;
                                            tracing::info!("ws_recv: telemetry frame for UAV {} via WebSocket", id);
                                            // mirror UDP path: update swarm and broadcast
                                            shared.swarm.upsert_uav(uav.clone()).await;

                                            if let Err(e) = shared.tx.send(uav) {
                                                tracing::debug!(
                                                    "No WebSocket subscribers for telemetry update {}: {}",
                                                    id,
                                                    e
                                                );
                                            }
                                        }

                                        Err(err) => {
                                            tracing::warn!(
                                                "Invalid WS client message: {} (raw: {})",
                                                err,
                                                text
                                            );
                                        }
                                    }
                                }

                                Message::Close(_) => {
                                    tracing::info!("WebSocket client closed connection");
                                    break;
                                }

                                _ => {
                                    // ignore non-text messages for now
                                }
                            }
                        }

                        Some(Err(err)) => {
                            tracing::warn!("WebSocket receive error: {}", err);
                            break;
                        }

                        None => {
                            tracing::info!("WebSocket client stream ended");
                            break;
                        }
                    }
                }

                recv_result = rx.recv() => {
                    match recv_result {
                        Ok(update) => {
                            let msg = match serde_json::to_string(&update) {
                                Ok(m) => m,
                                Err(err) => {
                                    tracing::warn!("Failed to serialize update: {}", err);
                                    continue;
                                }
                            };

                            if sender.send(Message::Text(msg)).await.is_err() {
                                tracing::info!("WebSocket client disconnected");
                                break;
                            }
                        }
                        Err(err) => {
                            tracing::warn!("WebSocket broadcast recv error: {}", err);
                            break;
                        }
                    }
                }
            }
        }
    }
    .instrument(span)
    .await;
}
