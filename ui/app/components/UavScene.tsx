"use client";
import { useRef } from "react";

import { Canvas, useFrame } from "@react-three/fiber";
import { OrbitControls, Line } from "@react-three/drei";
import * as THREE from "three";
import { UavState, ObstacleType, GoalMarker } from "../hooks/useTelemetry";

type CameraTarget = {
	x: number;
	y: number;
	z: number;
};

type Props = {
	uavs: UavState[];
	showTrails?: boolean;
	cameraTarget?: CameraTarget;
	formationMode?: string;
	obstacles?: ObstacleType[];
	goal?: GoalMarker | null;
};

function LeaderOrbitControls({ target }: { target: [number, number, number] }) {
	const controlsRef = useRef<any>(null);

	useFrame(() => {
		if (!controlsRef.current) return;

		const [x, y, z] = target;
		// smoothly lerp the controls target toward the leader position
		const desired = new THREE.Vector3(x, y, z);
		controlsRef.current.target.lerp(desired, 0.15);
		controlsRef.current.update();
	});

	return (
		<OrbitControls
			ref={controlsRef}
			enableDamping
			enablePan={false}
		/>
	);
}

/**
 * UavScene
 *
 * lightweight 3D visualization of the UAV swarm
 * - draws a nice green grid
 * - places glowing spheres for each UAV, colored by role
 * - allows camera orbiting
 * - draws simple trails from UAVs
 * - renders server-synced obstacles (cylinders, boxes, spheres)
 */

export default function UavScene({
	uavs,
	showTrails = true,
	cameraTarget,
	formationMode,
	obstacles = [],
	goal = null,
}: Props) {
	const scale = 1.0; // shrinks world into view
	const obstacleVisualScale = 1.0; // align visuals with collision footprint

	const trailsRef = useRef<Map<number, [number, number, number][]>>(new Map());

	// leader is ID 0, or first UAV if no explicit leader
	const leader = uavs.find((u) => u.id === 0) ?? uavs[0];
	const uavCount = uavs.length;

	// center frame on leader (or cameraTarget if provided)
	const worldLeaderX = cameraTarget?.x ?? (leader ? leader.position.x : 0);
	const worldLeaderY = cameraTarget?.y ?? (leader ? leader.position.y : 0);

	const originX = worldLeaderX * scale;
	const originZ = worldLeaderY * scale;


	// get some leader stats for HUD
	const headingDeg = leader
		? (Math.atan2(leader.velocity.vy, leader.velocity.vx) * 180) / Math.PI + 90
		: null;
	const velocity = leader ? leader.velocity : null;
	const altitude = leader ? leader.position.z : null;

	const formation = formationMode ? formationMode.toUpperCase() : "N/A";
	const goalDistance =
		goal && leader
			? Math.sqrt(
				(goal.x - leader.position.x) ** 2 +
				(goal.y - leader.position.y) ** 2 +
				(goal.z - leader.position.z) ** 2
			)
			: null;

	// leader position in the centered frame (used for camera targeting)
	const leaderTarget: [number, number, number] | null = leader
		? [
			leader.position.x * scale - originX,
			leader.position.z * scale,
			leader.position.y * scale - originZ,
		]
		: null;

	return (
		<div className="w-full h-96 mc-panel mc-panel-inner overflow-hidden relative bg-gradient-to-b from-black/80 to-black/95 border border-emerald-700/40 shadow-[0_0_12px_rgba(16,185,129,0.25)]">
			{/* HUD: obstacle count */}
			<div className="absolute bottom-2 left-3 z-20 nasa-text text-[0.65rem] text-emerald-300 bg-black/60 px-2 py-1 rounded">
				Obstacle Count: {obstacles.length}
			</div>
			{goal && (
				<div className="absolute bottom-2 right-3 z-20 nasa-text text-[0.65rem] text-emerald-300 bg-black/60 px-2 py-1 rounded text-right space-y-[2px]">
					<div>Goal: {goal.x.toFixed(1)}, {goal.y.toFixed(1)}, {goal.z.toFixed(1)}</div>
					{goalDistance !== null && <div>Dist: {goalDistance.toFixed(1)} m</div>}
				</div>
			)}
			{!leader && (
				<div className="absolute inset-0 z-10 flex items-center justify-center pointer-events-none">
					<div className="mc-panel-inner nasa-text text-xs tracking-widest text-emerald-300 bg-black/60 px-4 py-2 rounded">
						NO TELEMETRY // STANDBY
					</div>
				</div>
			)}
			{leader && (
				<div className="absolute top-3 left-3 z-10 mc-panel-inner nasa-text text-[0.65rem] bg-black/40 rounded-lg">
					<div className="flex flex-col gap-1">
						<div className="flex gap-4">
							<div>
								<div className="uppercase tracking-wide text-[0.6rem] text-emerald-300">
									UAVs
								</div>
								<div className="text-[0.75rem]">{uavCount}</div>
							</div>
							<div>
								<div className="uppercase tracking-wide text-[0.6rem] text-emerald-300">
									Heading
								</div>
								<div className="text-[0.75rem]">
									{headingDeg !== null ? `${headingDeg.toFixed(0)}°` : "—"}
								</div>
							</div>
						</div>
						<div className="flex gap-4">
							<div>
								<div className="uppercase tracking-wide text-[0.6rem] text-emerald-300">
									Velocity
								</div>
								<div className="text-[0.75rem]">
									{velocity !== null
										? `${Math.sqrt(
											velocity.vx ** 2 +
											velocity.vy ** 2 +
											velocity.vz ** 2
										).toFixed(1)} m/s`
										: "—"}
								</div>
							</div>
							<div>
								<div className="uppercase tracking-wide text-[0.6rem] text-emerald-300">
									Altitude
								</div>
								<div className="text-[0.75rem]">
									{altitude !== null ? `${altitude.toFixed(1)} m` : "—"}
								</div>
							</div>
							<div>
								<div className="uppercase tracking-wide text-[0.6rem] text-emerald-300">
									Formation
								</div>
								<div className="text-[0.75rem]">{formation}</div>
							</div>
						</div>
					</div>
				</div>
			)}
			<Canvas camera={{ position: [0, 75, 100], fov: 50 }}>
				<ambientLight intensity={0.6} />
				<directionalLight position={[8, 15, 5]} intensity={1.0} />

				{/* ground grid, axes, origin, and obstacles centered on leader-relative origin */}
				<group position={[-originX, 0, -originZ]}>
					<gridHelper args={[750, 750, "#00ff00", "#008800"]} />
					<axesHelper args={[5]} />

					{/* scene origin marker */}
					<mesh position={[0, 0.01, 0]}>
						<cylinderGeometry args={[0.2, 0.2, 0.02, 16]} />
						<meshStandardMaterial color="#3b82f6" />
					</mesh>

					{/* server-synced obstacles */}
					{goal && <GoalBeacon goal={goal} scale={scale} />}

					{/* server-synced obstacles */}
					{obstacles.map((obs, idx) => {
						// base altitude from sim (z), then lift by half-height / radius
						if (obs.type === "cylinder") {
							const centerY = obs.z * scale;
							const visualHeight = obs.height * scale * obstacleVisualScale;

							return (
								<group
									key={`obs-${idx}`}
									position={[obs.x * scale, centerY, obs.y * scale]}
								>
									<mesh>
										<cylinderGeometry
											args={[
												obs.radius * scale * obstacleVisualScale,
												obs.radius * scale * obstacleVisualScale,
												visualHeight,
												24,
											]}
										/>
										<meshStandardMaterial
											color="#00ff00"
											emissive="#00ff00"
											emissiveIntensity={1.5}
											transparent
											opacity={0.6}
											wireframe
										/>
									</mesh>
								</group>
							);
						}

						if (obs.type === "box") {
							const centerY = obs.z * scale;
							const visualHeight = obs.height * scale * obstacleVisualScale;

							return (
								<group
									key={`obs-${idx}`}
									position={[obs.x * scale, centerY, obs.y * scale]}
								>
									<mesh>
										<boxGeometry
											args={[
												obs.width * scale * obstacleVisualScale,
												visualHeight,
												obs.depth * scale * obstacleVisualScale,
											]}
										/>
										<meshStandardMaterial
											color="#00ff00"
											emissive="#00ff00"
											emissiveIntensity={1.5}
											transparent
											opacity={0.6}
											wireframe
										/>
									</mesh>
								</group>
							);
						}

						if (obs.type === "sphere") {
							const centerY = obs.z * scale;
							const visualRadius = obs.radius * scale * obstacleVisualScale;

							return (
								<group
									key={`obs-${idx}`}
									position={[obs.x * scale, centerY, obs.y * scale]}
								>
									<mesh>
										<sphereGeometry args={[visualRadius, 24, 24]} />
										<meshStandardMaterial
											color="#00ff00"
											emissive="#00ff00"
											emissiveIntensity={1.5}
											transparent
											opacity={0.6}
											wireframe
										/>
									</mesh>
								</group>
							);
						}

						return null;
					})}
				</group>
				{/* UAVs */}
				{uavs.map((uav) => {
					const isLeader = uav.id === 0; // UAV with ID 0 is leader

					// render positions in a frame centered on the leader so the grid moves with the swarm
					const headX = uav.position.x * scale - originX;
					const headY = uav.position.z * scale; // altitude uses z, no extra lift
					const headZ = uav.position.y * scale - originZ; // depth uses y, centered on leader

					// scale UAV size slightly based on altitude: higher = larger, lower = smaller
					const rawAlt = uav.position.z; // altitude now uses z
					const altNorm = Math.max(0, Math.min(1, rawAlt / 100)); // assume 0–100m typical band
					const sizeFactor = 0.7 + altNorm * 0.8; // range ~0.7–1.5

					// maintain trail history
					let trail = trailsRef.current.get(uav.id);
					if (!trail) {
						trail = [];
						trailsRef.current.set(uav.id, trail);
					}

					const last = trail[trail.length - 1];
					const nextPoint: [number, number, number] = [headX, headY, headZ];

					// only add to trail if position changed
					if (
						!last ||
						last[0] !== nextPoint[0] ||
						last[1] !== nextPoint[1] ||
						last[2] !== nextPoint[2]
					) {
						trail.push(nextPoint);
						if (trail.length > 80) {
							trail.shift(); // keep a finite tail length
						}
					}

					// fade trail colors
					const trailPoints = trail;
					const trailColors: [number, number, number][] = trail.map(
						(_, idx) => {
							// t = 0 for the newest point (end of trail), 1 for the oldest (tail)
							const t = trail.length > 1 ? 1 - idx / (trail.length - 1) : 0;

							if (isLeader) {
								// leader: brightest at head, dark amber at tail
								if (t > 0.66) {
									// bright gold
									return [0.98, 0.85, 0.08];
								}
								if (t > 0.33) {
									// mid gold
									return [0.82, 0.66, 0.05];
								}
								// dark amber
								return [0.25, 0.18, 0.02];
							} else {
								// followers: bright cyan at head, deep teal at tail
								if (t > 0.66) {
									// bright cyan
									return [0.13, 0.83, 0.93];
								}
								if (t > 0.33) {
									// mid cyan
									return [0.03, 0.57, 0.70];
								}
								// dark teal
								return [0.01, 0.18, 0.20];
							}
						}
					);

					return (
						<group key={uav.id}>
							<group
								position={[headX, headY, headZ]}
								scale={[sizeFactor, sizeFactor, sizeFactor]}
							>
								{/* core glowing sphere */}
								<mesh>
									<sphereGeometry args={[1.2, 24, 24]} />
									<meshStandardMaterial
										color={isLeader ? "#facc15" : "#22d3ee"} // gold for leader, cyan for followers
										emissive={isLeader ? "#eab308" : "#06b6d4"}
										emissiveIntensity={1.5}
									/>
								</mesh>

								{/* soft halo around each UAV for extra glow */}
								<mesh>
									<sphereGeometry args={[2.1, 24, 24]} />
									<meshStandardMaterial
										color={isLeader ? "#facc15" : "#22d3ee"}
										transparent
										opacity={0.15}
									/>
								</mesh>
							</group>

							{/* animated, fading trail line using drei's Line (world-space points) */}
							{showTrails && trail.length >= 2 && (
								<Line
									points={trailPoints}
									vertexColors={trailColors}
									lineWidth={(isLeader ? 2.5 : 1.5) * (0.8 + altNorm * 0.4)}
								/>
							)}
						</group>
					);
				})}

				{leaderTarget ? (
					<LeaderOrbitControls target={leaderTarget} />
				) : (
					<OrbitControls enableDamping enablePan={false} />
				)}
			</Canvas>
		</div>
	);
}
function GoalBeacon({ goal, scale }: { goal: GoalMarker; scale: number }) {
	const coreRef = useRef<THREE.Mesh>(null);
	const haloRef = useRef<THREE.Mesh>(null);

	// pulse the goal marker so it's easy to spot
	useFrame(({ clock }) => {
		const t = clock.getElapsedTime();
		const pulse = 1 + 0.08 * Math.sin(t * 2.0);
		if (coreRef.current) {
			coreRef.current.scale.setScalar(pulse);
		}
		if (haloRef.current) {
			const haloScale = 1.2 + 0.25 * Math.sin(t * 1.5);
			haloRef.current.scale.setScalar(haloScale);
		}
	});

	return (
		<group position={[goal.x * scale, goal.z * scale, goal.y * scale]}>
			<mesh ref={coreRef}>
				<sphereGeometry args={[goal.radius * scale, 32, 32]} />
				<meshStandardMaterial
					color="#ffd000" // bright yellow
					emissive="#ffd000"
					emissiveIntensity={6.8}
				/>
			</mesh>
			<mesh ref={haloRef}>
				<sphereGeometry args={[goal.radius * 1.6 * scale, 32, 32]} />
				<meshStandardMaterial
					color="#ffd000"
					transparent
					opacity={0.2}
					emissive="#ffd000"
					emissiveIntensity={5.0}
				/>
			</mesh>
			<mesh rotation={[-Math.PI / 2, 0, 0]}>
				<ringGeometry args={[goal.radius * 1.1 * scale, goal.radius * 1.4 * scale, 48]} />
				<meshStandardMaterial
					color="#ffd000"
					emissive="#ffd000"
					emissiveIntensity={4.8}
					side={THREE.DoubleSide}
					transparent
					opacity={0.35}
				/>
			</mesh>
		</group>
	);
}
