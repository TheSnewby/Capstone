"use client";
import { useRef } from "react";

import { Canvas } from "@react-three/fiber";
import { OrbitControls, Line } from "@react-three/drei";
import { UavState } from "../hooks/useTelemetry";

type Props = {
  uavs: UavState[];
  showTrails?: boolean;
};

/**
 * UavScene
 *
 * lightweight 3D visualization of the UAV swarm
 * - draws a nice green grid
 * - places a triangle for each UAV, colored by role
 * - allows camera orbiting!!!
 * - draws simple trails from UAVs
 */

export default function UavScene({ uavs, showTrails = true }: Props) {
  const scale = 0.1; // shrinks world into view
  const trailsRef = useRef<Map<number, [number, number, number][]>>(new Map());

  // leader is first in list
  const leader = uavs[0];
  const uavCount = uavs.length;

  // center frame on leader
  const originX = leader ? leader.position.x * scale : 0;
  const originZ = leader ? leader.position.y * scale : 0;

  // get some leader stats for HUD
  const headingDeg = leader ?
	(Math.atan2(leader.velocity.vy, leader.velocity.vx) * 180) / Math.PI + 90
	: null;
  const velocity = leader ? leader.velocity : null;
  const altitude = leader ? leader.position.z : null;
  const formation = "N/A"; // placeholder for future formation modes

  return (
    <div className="w-full h-96 mc-panel mc-panel-inner overflow-hidden relative bg-gradient-to-b from-black/80 to-black/95 border border-emerald-700/40 shadow-[0_0_12px_rgba(16,185,129,0.25)]">
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
              </div>
              <div>
                <div className="uppercase tracking-wide text-[0.6rem] text-emerald-300">UAVs</div>
                <div className="text-[0.75rem]">{uavCount}</div>
              </div>
              <div>
                <div className="uppercase tracking-wide text-[0.6rem] text-emerald-300">Heading</div>
                <div className="text-[0.75rem]">
                  {headingDeg !== null ? `${headingDeg.toFixed(0)}°` : "—"}
                </div>
              </div>
            </div>
            <div className="flex gap-4">
              <div>
                <div className="uppercase tracking-wide text-[0.6rem] text-emerald-300">Velocity</div>
                <div className="text-[0.75rem]">
                  {velocity !== null ? `${Math.sqrt(velocity.vx**2 + velocity.vy**2 + velocity.vz**2).toFixed(1)} m/s` : "—"}
                </div>
              </div>
              <div>
                <div className="uppercase tracking-wide text-[0.6rem] text-emerald-300">Altitude</div>
                <div className="text-[0.75rem]">
                  {altitude !== null ? `${altitude.toFixed(1)} m` : "—"}
                </div>
              </div>
              <div>
                <div className="uppercase tracking-wide text-[0.6rem] text-emerald-300">Formation</div>
                <div className="text-[0.75rem]">{formation}</div>
              </div>
            </div>
          </div>
	  	)}
      <Canvas camera={{ position: [0, 12, 22], fov: 50 }}>
        <ambientLight intensity={0.6} />
        <directionalLight position={[8, 15, 5]} intensity={1.0} />

        {/* ground grid & axes centered on the leader-relative origin */}
        <group position={[-originX, 0, -originZ]}>
          <gridHelper args={[50, 50, "#00ff00", "#008800"]} />
          <axesHelper args={[5]} />

          {/* scene origin marker */}
          <mesh position={[0, 0.01, 0]}>
            <cylinderGeometry args={[0.2, 0.2, 0.02, 16]} />
            <meshStandardMaterial color="#3b82f6" />
          </mesh>
        </group>

        {/* UAVs */}
        {uavs.map((uav, index) => {
          const isLeader = index === 0; // first UAV is leader

          // render positions in a frame centered on the leader so the grid moves with the swarm
          const headX = uav.position.x * scale - originX;
          const headY = uav.position.z * scale + 0.75; // altitude now uses z
          const headZ = uav.position.y * scale - originZ; // horizontal depth now uses y

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
          if (!last || last[0] !== nextPoint[0] || last[1] !== nextPoint[1] || last[2] !== nextPoint[2]) {
            trail.push(nextPoint);
            if (trail.length > 60) {
              trail.shift(); // keep a finite tail length
            }
          }

          // fade trail colors
          const trailPoints = trail;
          const trailColors: [number, number, number][] = trail.map((_, idx) => {
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
          });

          return (
            <group key={uav.id}>
              {/* UAV glyph (core + halo) scaled by altitude */}
              <group
                position={[headX, headY, headZ]}
                scale={[sizeFactor, sizeFactor, sizeFactor]}
              >
                {/* core glowing sphere */}
                <mesh>
                  <sphereGeometry args={[0.4, 24, 24]} />
                  <meshStandardMaterial
                    color={isLeader ? "#facc15" : "#22d3ee"} // gold for leader, cyan for followers
                    emissive={isLeader ? "#eab308" : "#06b6d4"}
                    emissiveIntensity={1.5}
                  />
                </mesh>

                {/* soft halo around each UAV for extra glow */}
                <mesh>
                  <sphereGeometry args={[0.7, 24, 24]} />
                  <meshStandardMaterial
                    color={isLeader ? "#facc15" : "#22d3ee"}
                    transparent
                    opacity={0.15}
                  />
                </mesh>
              </group>

              {/* animated, fading trail line using drei's Line */}
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

        <OrbitControls enableDamping />
      </Canvas>
    </div>
  );
}
