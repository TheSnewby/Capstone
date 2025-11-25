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
    <section className="mc-panel overflow-hidden">
      <table className="w-full text-xs md:text-sm">
        <thead className="bg-black/40">
          <tr>
            <th className="px-4 py-2 border-b border-zinc-700 text-left nasa-text">
              ID
            </th>
            <th className="px-4 py-2 border-b border-zinc-700 text-left nasa-text">
              Callsign
            </th>
            <th className="px-4 py-2 border-b border-zinc-700 text-left nasa-text">
              X
            </th>
            <th className="px-4 py-2 border-b border-zinc-700 text-left nasa-text">
              Y
            </th>
            <th className="px-4 py-2 border-b border-zinc-700 text-left nasa-text">
              Z
            </th>
            <th className="px-4 py-2 border-b border-zinc-700 text-left nasa-text">
              Speed (m/s)
            </th>
            <th className="px-4 py-2 border-b border-zinc-700 text-left nasa-text">
              Last Update
            </th>
          </tr>
        </thead>
        <tbody>
          {uavs.length === 0 ? (
            <tr>
              <td
                className="px-4 py-4 text-center nasa-text"
                colSpan={7}
              >
                No telemetry yet…
              </td>
            </tr>
          ) : (
            uavs.map((uav) => (
              <tr
                key={uav.id}
                className="border-b border-zinc-800/50 last:border-b-0"
              >
                <td className="px-4 py-2 nasa-text">{uav.id}</td>
                <td className="px-4 py-2 nasa-text">{uav.callsign}</td>
                <td className="px-4 py-2 nasa-text">
                  {uav.position.x.toFixed(1)}
                </td>
                <td className="px-4 py-2 nasa-text">
                  {uav.position.y.toFixed(1)}
                </td>
                <td className="px-4 py-2 nasa-text">
                  {uav.position.z.toFixed(1)}
                </td>
                <td className="px-4 py-2 nasa-text">
                  {uav.velocity_mps.toFixed(1)}
                </td>
                <td className="px-4 py-2 nasa-text">
                  {uav.last_update
                    ? new Date(uav.last_update).toLocaleTimeString()
                    : "—"}
                </td>
              </tr>
            ))
          )}
        </tbody>
      </table>
    </section>
  );
}
