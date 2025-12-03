"use client";

import { useCallback, useEffect, useRef, useState } from "react";

// Coordinate convention:
// - x, y: horizontal plane
// - z: altitude (up/down)
export type Position = {
	x: number;
	y: number;
	z: number;
};

export type Velocity = {
	vx: number;
	vy: number;
	vz: number;
};

export type UavState = {
	id: number;
	position: Position;
	velocity: Velocity;
	timestamp: string;
};

export type ConnectionStatus = "connecting" | "open" | "closed" | "error";

type TelemetryState = {
	uavs: UavState[];
	status: ConnectionStatus;
	settings: any | null;
	send: (message: unknown) => void;
};

const isUavState = (data: any): data is UavState => {
	return (
		data &&
		typeof data === "object" &&
		typeof data.id === "number" &&
		data.position &&
		typeof data.position.x === "number" &&
		typeof data.position.y === "number" &&
		typeof data.position.z === "number" &&
		data.velocity &&
		typeof data.velocity.vx === "number" &&
		typeof data.velocity.vy === "number" &&
		typeof data.velocity.vz === "number"
	);
};

const isSettingsUpdate = (data: any): data is { type: string; payload: any } => {
	return (
		data &&
		typeof data === "object" &&
		data.type === "settings_update" &&
		"payload" in data
	);
};

/**
 * useTelemetry
 * - opens a WebSocket to the rust server
 * - receives either:
 *   - array of UavState (initial snapshot)
 *   - single UavState (incremental update)
 */
export function useTelemetry(): TelemetryState {
	const [uavs, setUavs] = useState<UavState[]>([]);
	const [status, setStatus] = useState<ConnectionStatus>("connecting");
	const [settings, setSettings] = useState<any | null>(null);
	const wsRef = useRef<WebSocket | null>(null);

	const send = useCallback(
		(message: unknown) => {
			const ws = wsRef.current;
			if (!ws || ws.readyState !== WebSocket.OPEN) {
				console.warn("WebSocket not open; cannot send message", message);
				return;
			}

			try {
				const payload =
					typeof message === "string" ? message : JSON.stringify(message);
				ws.send(payload);
			} catch (err) {
				console.error("Failed to send WS message", err);
			}
		},
		[]
	);

	useEffect(() => {
		// only run on client
		if (typeof window === "undefined") return;

		// explicit env var or fall back to localhost:8080 in dev
		const envWsUrl = process.env.NEXT_PUBLIC_TELEMETRY_WS_URL;
		let wsUrl: string;

		if (envWsUrl && typeof envWsUrl === "string") {
			wsUrl = envWsUrl;
		} else {
			const protocol = window.location.protocol === "https:" ? "wss" : "ws";
			const host = window.location.hostname || "localhost";
			const port = 8080;
			wsUrl = `${protocol}://${host}:${port}/ws`;
		}

		const ws = new WebSocket(wsUrl);
		wsRef.current = ws;

		ws.onopen = () => {
			setStatus("open");
			// console.log("WS connected");
		};

		ws.onclose = () => {
			setStatus("closed");
			// console.log("WS closed");
		};

		ws.onerror = () => {
			setStatus("error");
			// console.error("WS error", event);
		};

		ws.onmessage = (event) => {
			try {
				const data = JSON.parse(event.data);

				// settings update from server
				if (isSettingsUpdate(data)) {
					setSettings(data.payload);
					return;
				}

				// initial snapshot: array of UavState
				if (Array.isArray(data)) {
					const valid = data.filter(isUavState);
					if (valid.length > 0) {
						setUavs(valid);
					} else {
						console.warn("Received array snapshot with no valid UAVs", data);
					}
					return;
				}

				// server might wrap telemetry like this: { type: 'telemetry', payload: UavState }
				if (
					data &&
					typeof data === "object" &&
					data.type === "telemetry" &&
					isUavState(data.payload)
				) {
					const update = data.payload;
					setUavs((prev) => {
						const idx = prev.findIndex((u) => u.id === update.id);
						if (idx === -1) return [...prev, update];
						const copy = [...prev];
						copy[idx] = update;
						return copy;
					});
					return;
				}

				// PROBABLY an incremental update: single UavState
				if (isUavState(data)) {
					const update = data;
					setUavs((prev) => {
						const idx = prev.findIndex((u) => u.id === update.id);
						if (idx === -1) return [...prev, update];
						const copy = [...prev];
						copy[idx] = update;
						return copy;
					});
					return;
				}

				// otherwise, who knows
				console.warn("Unknown telemetry message", data);
			} catch (err) {
				console.error("Failed to parse WS message", err, event.data);
			}
		};

		return () => {
			ws.close();
			wsRef.current = null;
		};
	}, []);

	// Debug helper: log current UAV positions when they change
	useEffect(() => {
		if (uavs.length === 0) return;

		console.log(
			"UAV positions from useTelemetry (z = altitude):",
			uavs.map((u) => ({
				id: u.id,
				x: u.position.x,
				y: u.position.y,
				z: u.position.z,
			}))
		);
	}, [uavs]);

	return { uavs, status, settings, send };
}
