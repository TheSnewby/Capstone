"use client";

import { useCallback, useEffect, useRef, useState } from "react";

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
	id: string;
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

		// Prefer an explicit env var in production, fall back to localhost:8080 in dev
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

				// settings updates
				if (data?.type === "settings_update") {
					setSettings(data.payload);
					return;
				}

				// initial UAV snapshot
				if (Array.isArray(data)) {
					setUavs(data);
					return;
				}

				// single UAV update
				const update: UavState = data;
				setUavs((prev) => {
					const idx = prev.findIndex((u) => u.id === update.id);
					if (idx === -1) return [...prev, update];
					const copy = [...prev];
					copy[idx] = update;
					return copy;
				});
			} catch (err) {
				console.error("Failed to parse WS message", err);
			}
		};

		return () => {
			ws.close();
			wsRef.current = null;
		};
	}, []);

	return { uavs, status, settings, send };
}
