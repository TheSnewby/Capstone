import dgram from "dgram";
import WebSocket from "ws";

const UDP_PORT = 6000; // Sim sends UDP locally into the bridge
const WS_URL = "wss://server-green-silence-3042.fly.dev/ws";

const udp = dgram.createSocket("udp4");
let ws;

function connectWs() {
	ws = new WebSocket(WS_URL);

	ws.on("open", () => {
		console.log("WS connected to", WS_URL);
	});

	ws.on("close", () => {
		console.log("WS closed, reconnecting in 2s...");
		setTimeout(connectWs, 2000);
	});

	ws.on("error", (err) => {
		console.error("WS error:", err.message);
	});

	ws.on("message", (data) => {
		const text = data.toString();
		console.log("WS message from server:", text);

		let msg;
		try {
			msg = JSON.parse(text);
		} catch (e) {
			// non-JSON or irrelevant messages are just logged and ignored
			return;
		}

		// Forward swarm_settings messages from server back to the sim via UDP
		if (msg.type === "swarm_settings") {
			const buf = Buffer.from(JSON.stringify(msg), "utf8");
			udp.send(buf, UDP_PORT, "127.0.0.1", (err) => {
				if (err) {
					console.error("Error sending swarm_settings to sim via UDP:", err.message);
				} else {
					console.log("Forwarded swarm_settings to sim via UDP:", msg);
				}
			});
		}
	});
}

connectWs();

udp.on("message", (msg, rinfo) => {
	const text = msg.toString("utf8");
	console.log("UDP from sim:", rinfo.address + ":" + rinfo.port, text);

	if (!ws || ws.readyState !== WebSocket.OPEN) {
		console.warn("WS not open, dropping packet");
		return;
	}

	let parsed;
	try {
		parsed = JSON.parse(text);
	} catch (e) {
		console.error("Failed to parse UDP JSON:", e.message);
		return;
	}

	// ----- NEW LOGIC: Detect environment vs telemetry -----
	let wrapper;

	// Environment packets include: { "type": "environment", "obstacles": [...] }
	if (parsed.type === "environment") {
		wrapper = {
			type: "environment",
			obstacles: parsed.obstacles ?? [],
			goal: parsed.goal ?? null,
		};
		console.log("Forwarding ENV to WS:", wrapper);
	} else {
		// Telemetry is all other packets (per-UAV)
		wrapper = {
			type: "telemetry",
			payload: parsed,
		};
	}

	ws.send(JSON.stringify(wrapper));
});

udp.bind(UDP_PORT, "127.0.0.1", () => {
	console.log(`Listening for sim UDP on 127.0.0.1:${UDP_PORT}`);
});
