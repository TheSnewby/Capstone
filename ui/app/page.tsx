/**
 * SkyWeave TelemetryPage
 *
 * data flow:
 *   C++ sim → UDP → Rust telemetry server → WebSocket (/ws) → useTelemetry hook → this UI.
 *
 * This page handles:
 *   - Splash screen and navigation
 *   - Live UAV 3D scene (UavScene)
 *   - Swarm controls and command panel
 *   - Telemetry table and WebSocket connection status
 */
"use client";

import { useEffect, useState } from "react";
import { useRouter } from "next/navigation";

import { useTelemetry } from "./hooks/useTelemetry";
import UavScene from "./components/UavScene";
import TelemetryTable from "./components/TelemetryTable";
import SwarmControls, { SwarmSettings } from "./components/SwarmControls";
import CommandPanel, { Command } from "./components/CommandPanel";

export default function TelemetryPage() {
  const { uavs, status, settings, send } = useTelemetry();
  const router = useRouter();
  const [showSplash, setShowSplash] = useState(true);
  const [showTrails, setShowTrails] = useState(true);
  const [swarmSettings, setSwarmSettings] = useState<SwarmSettings>({
    cohesion: 1.0,
    separation: 1.0,
    alignment: 1.0,
    maxSpeed: 30,
    targetAltitude: 150,
  });

  // sync local swarmSettings when server-sourced settings arrive over WebSocket
  useEffect(() => {
    if (!settings) return;

    setSwarmSettings((prev) => ({
      cohesion: settings.cohesion ?? prev.cohesion,
      separation: settings.separation ?? prev.separation,
      alignment: settings.alignment ?? prev.alignment,
      maxSpeed: settings.max_speed ?? settings.maxSpeed ?? prev.maxSpeed,
      targetAltitude:
        settings.target_altitude ?? settings.targetAltitude ?? prev.targetAltitude,
    }));
  }, [settings]);

  const handleCommand = (cmd: Command) => {
    // send command over the telemetry WebSocket to the Rust backend
    send({ type: "command", payload: cmd });
  };

  useEffect(() => {
    // broadcast swarm behavior settings to the backend via WebSocket
    send({ type: "swarm_settings", payload: swarmSettings });
  }, [swarmSettings, send]);

  // keyboard controls: WASD to move leader, Q/E altitude, T toggle trails
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      // don’t hijack typing in inputs/textareas/selects
      const target = event.target as HTMLElement | null;
      if (target && ["INPUT", "TEXTAREA", "SELECT"].includes(target.tagName)) {
        return;
      }

      switch (event.key.toLowerCase()) {
        case "w":
          handleCommand({ type: "move_leader", direction: "north" });
          break;
        case "s":
          handleCommand({ type: "move_leader", direction: "south" });
          break;
        case "a":
          handleCommand({ type: "move_leader", direction: "west" });
          break;
        case "d":
          handleCommand({ type: "move_leader", direction: "east" });
          break;
        case "q":
          handleCommand({ type: "altitude_change", amount: 10 });
          break;
        case "e":
          handleCommand({ type: "altitude_change", amount: -10 });
          break;
        case "t":
          setShowTrails((prev) => !prev);
          break;
        default:
          break;
      }
    };

    window.addEventListener("keydown", handleKeyDown);
    return () => window.removeEventListener("keydown", handleKeyDown);
  }, [send]);

  if (showSplash) {
    return (
      <main className="min-h-screen w-full bg-black text-white flex justify-center items-center">
        <div className="max-w-xl w-full px-6 text-center crt-scanlines mc-panel py-10 flex flex-col items-center justify-center">
          <div className="mb-6 flex justify-center items-center w-full">
            <svg
              viewBox="0 0 200 60"
              className="w-64 h-20 mx-auto block"
              aria-hidden="true"
            >
              <g>
                <polygon points="46,10 66,20 46,30" fill="#22d3ee" />
                <polygon points="78,10 98,20 78,30" fill="#22d3ee" />
                <polygon points="110,10 130,20 110,30" fill="#22d3ee" />
                <polygon points="142,5 162,20 142,35" fill="#facc15" />
              </g>
            </svg>
          </div>
          <h1 className="text-2xl md:text-3xl font-semibold nasa-text mb-2 mx-auto pl-2">
            SkyWeave
          </h1>
          <p className="text-zinc-400 text-sm md:text-base mb-6">Control autonomous UAV swarms with real-time telemetry and commands.</p>
          <div className="mt-8 w-full grid grid-cols-3 place-items-center gap-4">
            <button
              className="mc-button nasa-text text-xs px-5"
              onClick={() => router.push("/guide")}
            >
              Guide
            </button>

            <button
              className="mc-button nasa-text text-base px-10 py-3"
              onClick={() => setShowSplash(false)}
            >
              Launch
            </button>

            <button
              className="mc-button nasa-text text-xs px-5"
              onClick={() => router.push("/developers")}
            >
              Developers
            </button>
          </div>
        </div>
      </main>
    );
  }
  return (
    <main className="min-h-screen w-full bg-black text-white flex justify-center items-start py-8">
      <div className="w-full max-w-6xl px-4 crt-scanlines mc-panel">
        <header className="flex items-baseline justify-between mb-6">
          <div>
            <h1 className="text-2xl md:text-3xl font-semibold tracking-tight nasa-text">
              SkyWeave
            </h1>
            <p className="text-zinc-400 mt-1 text-sm">
              Live UAV Swarm Telemetry • C++ sim → Rust server → WebSocket → Next.js
            </p>
            <p className="text-zinc-500 mt-1 text-xs nasa-text">
              W-A-S-D : move leader • Q / E : change altitude • T : toggle trails
            </p>
          </div>

          <div className="flex flex-col items-end">
            <button
              className="mc-button nasa-text text-[0.65rem] mb-2 mr-1 btn-glow"
              onClick={() => setShowSplash(true)}
            >
              ← Back to Main
            </button>

            <div className="text-right text-xs nasa-text mc-panel-inner bg-black/40 rounded-lg px-3 py-2">
              <div className="flex items-center gap-2 crt-flicker">
                <span
                  className={
                    "ws-dot " +
                    (status === "open"
                      ? "ws-dot--ok"
                      : status === "connecting"
                      ? "ws-dot--warn"
                      : "ws-dot--err")
                  }
                />
                <span>
                  WS:{" "}
                  {status === "open"
                    ? "CONNECTED"
                    : status === "connecting"
                    ? "CONNECTING..."
                    : status === "closed"
                    ? "DISCONNECTED"
                    : "ERROR"}
                </span>
              </div>
              <div className="mt-1">UAVS: {uavs.length}</div>
            </div>
          </div>
        </header>

        <section className="mb-6 relative mc-panel">
          {/* HUD overlays over the 3D view */}
          <div className="absolute top-2 left-3 z-10 nasa-text text-xs crt-flicker">
            SWARM: {uavs.length > 0 ? "ACTIVE" : "STANDBY"}
          </div>
          <div className="absolute top-2 right-3 z-10 flex flex-col items-end gap-1">
            <div className="nasa-text text-xs">VIEW: 3D ORBIT</div>
            <button
              className="mc-button nasa-text text-[0.65rem]"
              onClick={() => setShowTrails((prev) => !prev)}
            >
              TRAILS: {showTrails ? "ON" : "OFF"}
            </button>
          </div>

          <UavScene uavs={uavs} showTrails={showTrails} />
        </section>

        <section className="mb-6 grid gap-4 md:grid-cols-2">
          <CommandPanel onCommand={handleCommand} status={status} />
          <SwarmControls settings={swarmSettings} onChange={setSwarmSettings} />
        </section>

        <section>
          <TelemetryTable uavs={uavs} />
        </section>

        <section className="mt-4">
          <h2 className="text-sm nasa-text mb-2">Swarm Telemetry (debug)</h2>
          <pre className="text-[0.6rem] bg-black/60 rounded-lg p-3 overflow-x-auto max-h-64">
            {JSON.stringify(uavs, null, 2)}
          </pre>
        </section>
      </div>
    </main>
  );
}
