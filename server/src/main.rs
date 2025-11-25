mod config;
mod telemetry;
mod websocket_server;

use crate::config::AppConfig;
use crate::telemetry::{SwarmState, TelemetryShared};
use crate::websocket_server::router;
use std::net::SocketAddr;
use tokio::net::TcpListener;
use tracing_subscriber::EnvFilter;

#[tokio::main]
async fn main() {
    init_tracing();

    let cfg = AppConfig::from_env();
    tracing::info!("Loaded config: {:?}", cfg);

    // initialize shared telemetry state
    let swarm = SwarmState::new();
    let (tx, _rx) = tokio::sync::broadcast::channel(1024);
    let shared = TelemetryShared { swarm, tx };

    // start UDP telemetry listener
    let udp_addr: SocketAddr = format!("{}:{}", cfg.udp_bind_addr, cfg.udp_port)
        .parse()
        .expect("Invalid UDP bind address");
    {
        let shared_clone = shared.clone();
        tokio::spawn(async move {
            telemetry::run_udp_listener(udp_addr, shared_clone).await;
        });
    }

    // start HTTP/WebSocket server
    // Fly.io will inject $PORT for the container's HTTP service
    let port_env = std::env::var("PORT").ok();
    let port = port_env
        .as_deref()
        .unwrap_or(&cfg.http_port.to_string())
        .parse::<u16>()
        .expect("Invalid PORT env var");

    let http_addr: SocketAddr = format!("0.0.0.0:{}", port)
        .parse()
        .expect("Invalid HTTP bind address");

    let listener = TcpListener::bind(http_addr)
        .await
        .expect("Failed to bind HTTP listener");

    tracing::info!("HTTP/WebSocket listening on {}", http_addr);

    let app = router(shared);

    if let Err(err) = axum::serve(listener, app).await {
        tracing::error!("Server error: {}", err);
    }
}

/// initialize tracing subscriber for logging
fn init_tracing() {
    let env_filter =
        EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));

    tracing_subscriber::fmt()
        .with_env_filter(env_filter)
        .init();
}
