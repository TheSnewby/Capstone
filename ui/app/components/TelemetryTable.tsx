"use client";

import { UavState } from "../hooks/useTelemetry";

type Props = {
	uavs: UavState[];
};

/**
 * TelemetryTable
 *
 * displays a table of all UAV telemetry data.
 * uses global .nasa-text styling for green glowing HUD text
 */
export default function TelemetryTable({ uavs }: Props) {
	return (
		<section className="mc-panel mc-panel-inner overflow-hidden bg-gradient-to-b from-black/80 to-black/95 border border-emerald-700/40 shadow-[0_0_12px_rgba(16,185,129,0.25)]">
			<h2 className="mb-4">
				<div className="inline-flex items-center px-3 py-2 rounded-sm border border-emerald-400/70 bg-emerald-900/40 shadow-[0_0_18px_rgba(16,185,129,0.7)]">
					<span className="uppercase tracking-[0.3em] text-[0.8rem] font-semibold text-emerald-100 drop-shadow-[0_0_8px_rgba(16,185,129,0.95)]">
						Telemetry Feed
					</span>
				</div>
			</h2>
			<table className="w-full text-xs md:text-sm font-sans">
				<thead className="bg-black/40">
					<tr>
						<th className="px-4 py-2 border-b border-emerald-700/60 text-left nasa-text text-emerald-300 tracking-widest uppercase">
							ID
						</th>
						<th className="px-4 py-2 border-b border-emerald-700/60 text-left nasa-text text-emerald-300 tracking-widest uppercase">
							X
						</th>
						<th className="px-4 py-2 border-b border-emerald-700/60 text-left nasa-text text-emerald-300 tracking-widest uppercase">
							Y
						</th>
						<th className="px-4 py-2 border-b border-emerald-700/60 text-left nasa-text text-emerald-300 tracking-widest uppercase">
							Altitude
						</th>
						<th className="px-4 py-2 border-b border-emerald-700/60 text-left nasa-text text-emerald-300 tracking-widest uppercase">
							Velocity (m/s)
						</th>
						<th className="px-4 py-2 border-b border-emerald-700/60 text-left nasa-text text-emerald-300 tracking-widest uppercase">
							Timestamp
						</th>
					</tr>
				</thead>
				<tbody>
					{uavs.length === 0 ? (
						<tr>
							<td
								className="px-4 py-6 text-center nasa-text text-emerald-300 tracking-widest uppercase"
								colSpan={6}
							>
								NO TELEMETRY // STANDBY
							</td>
						</tr>
					) : (
						uavs.map((uav) => {
							const speed = Math.sqrt(
								uav.velocity.vx * uav.velocity.vx +
								uav.velocity.vy * uav.velocity.vy +
								uav.velocity.vz * uav.velocity.vz
							);

							const isLeader = uav.id === 0;

							return (
								<tr
									key={uav.id}
									className={
										(isLeader
											? "bg-emerald-900/20 hover:bg-emerald-900/20 shadow-[0_-2px_12px_rgba(16,185,129,0.45),0_2px_12px_rgba(16,185,129,0.45)]"
											: "border-b border-emerald-900/40 last:border-b-0 hover:bg-emerald-900/10")
									}
								>
									<td className="px-4 py-2 nasa-text text-emerald-100">
										{uav.id}
									</td>
									<td className="px-4 py-2 nasa-text text-emerald-100">
										{uav.position.x.toFixed(1)}
									</td>
									<td className="px-4 py-2 nasa-text text-emerald-100">
										{uav.position.y.toFixed(1)}
									</td>
									<td className="px-4 py-2 nasa-text text-emerald-100">
										{uav.position.z.toFixed(1)}
									</td>
									<td className="px-4 py-2 nasa-text text-emerald-100">
										{speed.toFixed(1)}
									</td>
									<td className="px-4 py-2 nasa-text text-emerald-100">
										{uav.timestamp
											? new Date(uav.timestamp).toLocaleTimeString()
											: "—"}
									</td>
								</tr>
							);
						})
					)}
				</tbody>
			</table>
		</section>
	);
}
