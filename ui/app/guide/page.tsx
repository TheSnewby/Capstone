"use client";

import Link from "next/link";
import Image from "next/image";

export default function GuidePage() {
  return (
    <main className="min-h-screen w-full bg-black text-white flex justify-center items-start py-8">
      <div className="w-full max-w-4xl px-4 crt-scanlines mc-panel">
        <div className="mb-4 flex justify-end">
          <Link href="/" className="mc-button btn-glow text-xs px-4 py-1 text-emerald-200">
            ← Back to Main
          </Link>
        </div>
        <header className="mb-6">
          <h1 className="text-3xl font-semibold nasa-text tracking-widest mb-2 text-emerald-300">
            SkyWeave Guide
          </h1>
          <p className="text-emerald-100 mb-4 font-sans tracking-wide">
            A simple overview of what SkyWeave does and how to use it
          </p>
        </header>

        <div className="w-full h-px bg-zinc-700/60 mb-6"></div>

        <section className="mc-panel-inner bg-gradient-to-b from-black/80 to-black/95 font-sans leading-relaxed tracking-wide text-base mb-8 border border-emerald-700/40 shadow-[0_0_12px_rgba(16,185,129,0.25)] ring-1 ring-emerald-600/40">
          <h2 className="text-xl font-semibold nasa-text tracking-widest uppercase mb-3 text-emerald-200">
            System Architecture
          </h2>
          <p className="text-emerald-100 mb-4">
            This diagram shows how the SkyWeave system flows from the Next.js dashboard
            to the Rust middleware and into the C++ simulation engine.
          </p>
          <div className="w-full flex justify-center">
            <Image
              src="/skyweave_flowchart.png"
              alt="SkyWeave Architecture Diagram"
              width={1200}
              height={600}
              className="rounded-lg border border-zinc-700 shadow-lg"
            />
          </div>
        </section>

        <section className="mc-panel-inner bg-gradient-to-b from-black/80 to-black/95 font-sans leading-relaxed tracking-wide text-base mb-6 border border-emerald-700/40 shadow-[0_0_12px_rgba(16,185,129,0.25)]">
          <h2 className="text-xl font-semibold nasa-text tracking-widest uppercase mb-3 text-emerald-200">
            What SkyWeave Does
          </h2>
          <p className="text-emerald-100 mb-3">
            SkyWeave is a real-time UAV swarm interface that visualizes autonomous
            drones inside a 3D environment. It connects to a C++ simulation that
            models UAV movement, formation behavior, and telemetry.
          </p>
          <p className="text-emerald-100 mb-3">
            The Rust backend receives UDP telemetry from the simulator, processes
            it, and broadcasts the data to the browser over WebSockets. The
            Next.js/React dashboard renders the swarm, displays sensor values,
            and lets you control formation parameters.
          </p>
        </section>

        <section className="mc-panel-inner bg-gradient-to-b from-black/80 to-black/95 font-sans leading-relaxed tracking-wide text-base mb-6 border border-emerald-700/40 shadow-[0_0_12px_rgba(16,185,129,0.25)]">
          <h2 className="text-xl font-semibold nasa-text tracking-widest uppercase mb-3 text-emerald-200">
            How the Simulation Works
          </h2>
          <p className="text-emerald-100 mb-3">
            The C++ simulation updates each UAV's position every frame,
            calculates heading, velocity, altitude, and formation alignment, and
            sends the resulting telemetry packets to the Rust server. The sim
            includes support for leader–follower behavior, collision‑avoidance,
            and configurable swarm spacing.
          </p>
          <p className="text-emerald-100 mb-3">
            Each drone is represented as a simple point-mass model with basic
            physics constraints. The server collects all UAV states into a single
            swarm snapshot and distributes it to connected clients.
          </p>
        </section>

        <section className="mc-panel-inner bg-gradient-to-b from-black/80 to-black/95 font-sans leading-relaxed tracking-wide text-base mb-6 border border-emerald-700/40 shadow-[0_0_12px_rgba(16,185,129,0.25)]">
          <h2 className="text-xl font-semibold nasa-text tracking-widest uppercase mb-3 text-emerald-200">
            How to Use SkyWeave
          </h2>
          <p className="text-emerald-100 mb-3">
            When the simulation and server are running, open the dashboard to view
            live telemetry. Each UAV is visualized as a triangle in 3D space, with
            color-coded trails that show recent motion. The leader is gold, while
			its followers are cyan.
          </p>
          <p className="text-emerald-100 mb-3">
            Use the on‑screen Mission Control panels to adjust formation spacing,
            toggle flight trails, or send high‑level commands to the swarm leader.
          </p>
        </section>

        <section className="mc-panel-inner bg-gradient-to-b from-black/80 to-black/95 font-sans leading-relaxed tracking-wide text-base mb-10 border border-emerald-700/40 shadow-[0_0_12px_rgba(16,185,129,0.25)]">
          <h2 className="text-xl font-semibold nasa-text tracking-widest uppercase mb-3 text-emerald-200">
            Keyboard Controls
          </h2>
          <p className="text-emerald-100 mb-3">
            In the 3D environment, use the following keys to control the swarm leader:
			W-A-S-D for movement, Q/E for altitude, and T to toggle flight trails.
          </p>
        </section>

      </div>
    </main>
  );
}