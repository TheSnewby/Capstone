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
		console.log("WS message from server:", data.toString());
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