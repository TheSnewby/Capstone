"use client";

import React from "react";

export type Command =
  | { type: "move_leader"; direction: "north" | "south" | "east" | "west" }
  | { type: "altitude_change"; amount: number }
  | { type: "formation"; mode: "line" | "vee" | "circle" }
  | { type: "pause" }
  | { type: "resume" }
  | { type: "rtb" }; // return to base (leader)

type CommandPanelProps = {
  onCommand: (cmd: Command) => void;
};

/**
 * CommandPanel
 *
 * Mission Control command interface for UAV swarm operations.
 * - Leader movement buttons
 * - Altitude adjustments
 * - Formation switches
 * - Global pause/resume/RTB
 */
export default function CommandPanel({ onCommand }: CommandPanelProps) {
  return (
    <section className="mc-panel nasa-text text-xs">
      <h2 className="mb-3 uppercase tracking-wide text-[0.7rem] text-zinc-400">
        Command Console
      </h2>

      {/* Leader Movement */}
      <div className="mb-4">
        <div className="mb-1 tracking-wide">LEADER MOVEMENT</div>
        <div className="grid grid-cols-3 gap-2 text-center">
          <button
            className="mc-button nasa-text text-[0.7rem]"
            onClick={() => onCommand({ type: "move_leader", direction: "north" })}
          >
            ▲
          </button>
          <div />
          <button
            className="mc-button nasa-text text-[0.7rem]"
            onClick={() => onCommand({ type: "move_leader", direction: "south" })}
          >
            ▼
          </button>
          <button
            className="mc-button nasa-text text-[0.7rem]"
            onClick={() => onCommand({ type: "move_leader", direction: "west" })}
          >
            ◀
          </button>
          <div />
          <button
            className="mc-button nasa-text text-[0.7rem]"
            onClick={() => onCommand({ type: "move_leader", direction: "east" })}
          >
            ▶
          </button>
        </div>
      </div>

      {/* Altitude Controls */}
      <div className="mb-4">
        <div className="mb-1 tracking-wide">ALTITUDE</div>
        <div className="flex gap-2">
          <button
            className="flex-1 mc-button nasa-text text-[0.7rem]"
            onClick={() => onCommand({ type: "altitude_change", amount: +10 })}
          >
            +10m
          </button>
          <button
            className="flex-1 mc-button nasa-text text-[0.7rem]"
            onClick={() => onCommand({ type: "altitude_change", amount: -10 })}
          >
            −10m
          </button>
        </div>
      </div>

      {/* Formation Controls */}
      <div className="mb-4">
        <div className="mb-1 tracking-wide">FORMATION</div>
        <div className="grid grid-cols-3 gap-2">
          <button
            className="mc-button nasa-text text-[0.7rem]"
            onClick={() => onCommand({ type: "formation", mode: "line" })}
          >
            LINE
          </button>
          <button
            className="mc-button nasa-text text-[0.7rem]"
            onClick={() => onCommand({ type: "formation", mode: "vee" })}
          >
            VEE
          </button>
          <button
            className="mc-button nasa-text text-[0.7rem]"
            onClick={() => onCommand({ type: "formation", mode: "circle" })}
          >
            CIRCLE
          </button>
        </div>
      </div>

      {/* Global Controls */}
      <div className="mb-1 tracking-wide">GLOBAL COMMANDS</div>
      <div className="grid grid-cols-3 gap-2">
        <button
          className="mc-button nasa-text text-[0.7rem]"
          onClick={() => onCommand({ type: "pause" })}
        >
          PAUSE
        </button>
        <button
          className="mc-button nasa-text text-[0.7rem]"
          onClick={() => onCommand({ type: "resume" })}
        >
          RESUME
        </button>
        <button
          className="mc-button nasa-text text-[0.7rem]"
          onClick={() => onCommand({ type: "rtb" })}
        >
          RTB
        </button>
      </div>
    </section>
  );
}
