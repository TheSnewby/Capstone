"use client";

import { useTelemetry } from "../hooks/useTelemetry";
import TelemetryTable from "../components/TelemetryTable";

export default function TelemetryPage() {
	const { uavs, status } = useTelemetry();

	return (
		<main className="min-h-screen w-full bg-black text-white flex justify-center items-start py-8">
			<div className="w-full max-w-5xl px-4 crt-scanlines mc-panel">
				<header className="flex items-start justify-between mb-4">
					<div>
						<h1 className="text-lg md:text-2xl font-semibold nasa-text">
							UAV Swarm Telemetry
						</h1>
						<p className="text-zinc-400 mt-1 text-xs md:text-sm">
							Live data from the C++ simulator via Rust UDP + WebSocket backend.
						</p>
					</div>

					<div className="text-right text-xs nasa-text mc-panel-inner bg-black/40 rounded-lg px-3 py-2">
						<div className="flex items-center gap-2 crt-flicker">
							<span
								className={
									"ws-dot " +
									(status.toString().toLowerCase() === "connected"
											? "ws-dot--ok"
										: status.toString().toLowerCase() === "connecting"
											? "ws-dot--warn"
											: "ws-dot--err")
								}
							/>
							WS: {status.toString().toUpperCase()}
						</div>
						<div className="mt-1">UAVS: {uavs.length}</div>
					</div>
				</header>

				<section>
					<h2 className="text-xs font-semibold text-emerald-400 uppercase tracking-wide mb-2 nasa-text">
						Telemetry Console
					</h2>
					<TelemetryTable uavs={uavs} />
				</section>
			</div>
		</main>
	);
}
