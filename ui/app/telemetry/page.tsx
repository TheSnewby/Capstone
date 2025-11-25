"use client";

import { useTelemetry } from "../hooks/useTelemetry";
import React from "react";

function formatLastUpdate(iso?: string) {
	if (!iso) return "—";
	const d = new Date(iso);
	if (Number.isNaN(d.getTime())) return "—";
	return d.toLocaleTimeString();
}

export default function TelemetryPage() {
	const { uavs, status } = useTelemetry();

	const statusColor =
		status === "open"
			? "text-green-600"
			: status === "connecting"
				? "text-yellow-600"
				: status === "error"
					? "text-red-600"
					: "text-gray-600";

	return (
		<main style={{ padding: "1.5rem", fontFamily: "system-ui, sans-serif" }}>
			<header
				style={{
					display: "flex",
					justifyContent: "space-between",
					marginBottom: "1rem",
					alignItems: "baseline",
				}}
			>
				<div>
					<h1 style={{ fontSize: "1.75rem", fontWeight: 600 }}>
						UAV Swarm Telemetry
					</h1>
					<p style={{ color: "#555", marginTop: "0.25rem" }}>
						Live data from the C++ simulator via Rust UDP + WebSocket backend.
					</p>
				</div>

				<div style={{ textAlign: "right" }}>
					<div className={statusColor} style={{ fontWeight: 600 }}>
						WS Status: {status.toUpperCase()}
					</div>
					<div style={{ color: "#555", marginTop: "0.25rem" }}>
						Active UAVs: {uavs.length}
					</div>
				</div>
			</header>

			<section
				style={{
					border: "1px solid #ddd",
					borderRadius: "0.5rem",
					overflow: "hidden",
				}}
			>
				<table
					style={{
						width: "100%",
						borderCollapse: "collapse",
						fontSize: "0.9rem",
					}}
				>
					<thead style={{ backgroundColor: "#f8f8f8" }}>
						<tr>
							<th style={thStyle}>ID</th>
							<th style={thStyle}>Callsign</th>
							<th style={thStyle}>X</th>
							<th style={thStyle}>Y</th>
							<th style={thStyle}>Z</th>
							<th style={thStyle}>Speed (m/s)</th>
							<th style={thStyle}>Last Update</th>
						</tr>
					</thead>
					<tbody>
						{uavs.length === 0 ? (
							<tr>
								<td colSpan={7} style={emptyCellStyle}>
									{status === "open"
										? "Connected, waiting for telemetry…"
										: "No telemetry yet…"}
								</td>
							</tr>
						) : (
							uavs.map((uav) => (
								<tr key={uav.id}>
									<td style={tdStyle}>{uav.id}</td>
									<td style={tdStyle}>{uav.callsign}</td>
									<td style={tdStyle}>{uav.position.x.toFixed(1)}</td>
									<td style={tdStyle}>{uav.position.y.toFixed(1)}</td>
									<td style={tdStyle}>{uav.position.z.toFixed(1)}</td>
									<td style={tdStyle}>{uav.velocity_mps.toFixed(1)}</td>
									<td style={tdStyle}>{formatLastUpdate(uav.last_update)}</td>
								</tr>
							))
						)}
					</tbody>
				</table>
			</section>
		</main>
	);
}

const thStyle: React.CSSProperties = {
	textAlign: "left",
	padding: "0.5rem 0.75rem",
	borderBottom: "1px solid #ddd",
};

const tdStyle: React.CSSProperties = {
	padding: "0.5rem 0.75rem",
	borderBottom: "1px solid #eee",
	whiteSpace: "nowrap",
};

const emptyCellStyle: React.CSSProperties = {
	padding: "1rem",
	textAlign: "center",
	color: "#777",
};
