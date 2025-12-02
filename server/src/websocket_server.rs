use crate::telemetry::{SwarmSettings, TelemetryShared, UavState};
use axum::{
    extract::{
        ws::{Message, WebSocket, WebSocketUpgrade},
        State,
        Path,
    },
    http::StatusCode,
    response::IntoResponse,
    routing::get,
    Json, Router,
};
use futures::{sink::SinkExt, stream::StreamExt};
use serde::Deserialize;
use tokio::net::UdpSocket;
use tracing::Instrument;
use std::env;

fn sim_addr() -> String {
    env::var("SKYWEAVE_SIM_ADDR").unwrap_or_else(|_| "127.0.0.1:6001".to_string())
}

/// message sent from the UI over WebSocket
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

async fn get_uav(
    State(shared): State<TelemetryShared>,
    Path(id): Path<u64>,
) -> impl IntoResponse {
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
        "line" => Some("LINE"),
        "vee" => Some("VEE"),
        "circle" => Some("CIRCLE"),
        _ => None,
    };

    let Some(msg) = msg else {
        tracing::warn!("Unknown formation command from client: {}", formation);
        return;
    };

    let addr = sim_addr();

    match UdpSocket::bind("0.0.0.0:0").await {
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

    match UdpSocket::bind("0.0.0.0:0").await {
        Ok(socket) => {
            tracing::info!("sending control UDP to sim {}: {}", addr, command);
            if let Err(err) = socket.send_to(command.as_bytes(), &addr).await {
                tracing::warn!("Failed to send control command to sim: {} (cmd = {})", err, command);
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

        loop {
            tokio::select! {
                maybe_msg = receiver.next() => {
                    match maybe_msg {
                        Some(Ok(msg)) => {
                            match msg {
                                Message::Text(text) => {
                                    match serde_json::from_str::<ClientMessage>(&text) {
                                        Ok(ClientMessage::Command(cmd)) => {
                                            tracing::info!("swarm_command_from_ui={}", cmd);

                                            if let Some(formation) = cmd.get("formation").and_then(|v| v.as_str()) {
                                                // UI is asking for a formation change; forward to the C++ simulator
                                                send_formation_command_to_sim(formation).await;
                                            } else if let Some(cmd_type) = cmd.get("type").and_then(|v| v.as_str()) {
                                                // Typed control commands from the UI (move leader, altitude, etc.)
                                                match cmd_type {
                                                    "move_leader" => {
                                                        if let Some(direction) = cmd.get("direction").and_then(|v| v.as_str()) {
                                                            // Match the C++ parser: "move_leader north|south|east|west"
                                                            let command = format!("move_leader {}", direction.to_lowercase());
                                                            send_control_command_to_sim(&command).await;
                                                        } else {
                                                            tracing::warn!("move_leader command missing `direction` field: {:?}", cmd);
                                                        }
                                                    }
                                                    "altitude_change" => {
                                                        if let Some(amount) = cmd.get("amount").and_then(|v| v.as_f64()) {
                                                            // Match the C++ parser: "altitude_change <delta>"
                                                            let command = format!("altitude_change {}", amount);
                                                            send_control_command_to_sim(&command).await;
                                                        } else {
                                                            tracing::warn!("altitude_change command missing `amount` field: {:?}", cmd);
                                                        }
                                                    }
                                                    _ => {
                                                        // Unknown typed command: apply locally for now
                                                        shared.swarm.apply_command(cmd).await;
                                                    }
                                                }
                                            } else {
                                                // No special routing; treat it as a normal swarm command
                                                shared.swarm.apply_command(cmd).await;
                                            }
                                        }
                                        Ok(ClientMessage::SwarmSettings(settings)) => {
                                            // apply and then echo settings update back to this client
                                            let settings_clone = settings.clone();
                                            shared.swarm.apply_settings(settings).await;

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
