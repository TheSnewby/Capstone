"use client";

import React, { useEffect, useState } from "react";
import type { ConnectionStatus } from "../hooks/useTelemetry";

export type Command =
	| { type: "move_leader"; direction: "accelerate" | "decelerate" | "left" | "right" }
	| { type: "altitude_change"; amount: number }
	| { type: "formation"; mode: "line" | "vee" | "circle" }
	| { type: "pause" }
	| { type: "resume" }
	| { type: "rtb" }; // return to base (leader)

type CommandPanelProps = {
	onCommand: (cmd: Command) => void;
	status: ConnectionStatus;
};

/**
 * CommandPanel
 *
 * Mission Control command interface for UAV swarm operations
 * - leader movement buttons
 * - altitude adjustments
 * - formation switches
 * - global pause/resume/RTB
 */
export default function CommandPanel({ onCommand, status }: CommandPanelProps) {
	const isDisabled = status !== "open";

	const [keyPress, setKeyPress] = useState<string | null>(null);

	// Keyboard bindings for visual feedback only (commands are handled in page.tsx)
	useEffect(() => {
		const handleKeyDown = (e: KeyboardEvent) => {
			if (isDisabled) return;
			if (e.repeat) return;

			const key = e.key.toLowerCase();
			if (["w", "a", "s", "d", "q", "e"].includes(key)) {
				switch (key) {
					case "w":
						setKeyPress("accelerate");
						break;
					case "s":
						setKeyPress("decelerate");
						break;
					case "a":
						setKeyPress("left");
						break;
					case "d":
						setKeyPress("right");
						break;
					case "q":
						setKeyPress("alt_up");
						break;
					case "e":
						setKeyPress("alt_down");
						break;
				}
			}
		};

		const handleKeyUp = (e: KeyboardEvent) => {
			const key = e.key.toLowerCase();
			if (["w", "a", "s", "d", "q", "e"].includes(key)) {
				setKeyPress(null);
			}
		};

		window.addEventListener("keydown", handleKeyDown);
		window.addEventListener("keyup", handleKeyUp);
		return () => {
			window.removeEventListener("keydown", handleKeyDown);
			window.removeEventListener("keyup", handleKeyUp);
		};
	}, [isDisabled]);

	return (
		<section className="mc-panel mc-panel-inner bg-gradient-to-b from-emerald-900/60 to-black/85 nasa-text text-xs border border-emerald-400/60 shadow-[0_0_22px_rgba(16,185,129,0.55)]">
			<h2 className="mb-4">
				<div className="inline-flex items-center px-3 py-2 rounded-sm border border-emerald-400/70 bg-emerald-900/40 shadow-[0_0_18px_rgba(16,185,129,0.7)]">
					<span className="uppercase tracking-[0.3em] text-[0.8rem] font-semibold text-emerald-200 drop-shadow-[0_0_8px_rgba(16,185,129,0.95)]">
						Command Console
					</span>
				</div>
			</h2>

			{/* leader movement */}
			<div className="mb-4">
				<div className="mb-1 tracking-wide">LEADER MOVEMENT</div>
				<div className="flex flex-col items-center gap-2 text-center">
					<button
						className={
							"mc-button btn-glow nasa-text font-semibold tracking-widest text-[0.85rem] px-8 py-3 min-w-[7rem] bg-emerald-900/50 hover:bg-emerald-500/30 border border-emerald-300/70 text-emerald-200 shadow-[0_0_20px_rgba(16,185,129,0.9)] drop-shadow-[0_0_6px_rgba(16,185,129,0.9)] disabled:opacity-40 disabled:cursor-not-allowed active:translate-y-[2px] active:brightness-90 transition-all" +
							(keyPress === "accelerate"
								? " ring-2 ring-emerald-400 translate-y-[2px] brightness-90"
								: "")
						}
						disabled={isDisabled}
						onClick={() => onCommand({ type: "move_leader", direction: "accelerate" })}
					>
						▲
					</button>

					<div className="flex gap-2">
						<button
							className={
								"mc-button btn-glow nasa-text font-semibold tracking-widest text-[0.85rem] px-8 py-3 min-w-[7rem] bg-emerald-900/50 hover:bg-emerald-500/30 border border-emerald-300/70 text-emerald-200 shadow-[0_0_20px_rgba(16,185,129,0.9)] drop-shadow-[0_0_6px_rgba(16,185,129,0.9)] disabled:opacity-40 disabled:cursor-not-allowed active:translate-y-[2px] active:brightness-90 transition-all" +
								(keyPress === "west"
									? " ring-2 ring-emerald-400 translate-y-[2px] brightness-90"
									: "")
							}
							disabled={isDisabled}
							onClick={() => onCommand({ type: "move_leader", direction: "left" })}
						>
							◀
						</button>

						<button
							className={
								"mc-button btn-glow nasa-text font-semibold tracking-widest text-[0.85rem] px-8 py-3 min-w-[7rem] bg-emerald-900/50 hover:bg-emerald-500/30 border border-emerald-300/70 text-emerald-200 shadow-[0_0_20px_rgba(16,185,129,0.9)] drop-shadow-[0_0_6px_rgba(16,185,129,0.9)] disabled:opacity-40 disabled:cursor-not-allowed active:translate-y-[2px] active:brightness-90 transition-all" +
								(keyPress === "east"
									? " ring-2 ring-emerald-400 translate-y-[2px] brightness-90"
									: "")
							}
							disabled={isDisabled}
							onClick={() => onCommand({ type: "move_leader", direction: "right" })}
						>
							▶
						</button>
					</div>

					<button
						className={
							"mc-button btn-glow nasa-text font-semibold tracking-widest text-[0.85rem] px-8 py-3 min-w-[7rem] bg-emerald-900/50 hover:bg-emerald-500/30 border border-emerald-300/70 text-emerald-200 shadow-[0_0_20px_rgba(16,185,129,0.9)] drop-shadow-[0_0_6px_rgba(16,185,129,0.9)] disabled:opacity-40 disabled:cursor-not-allowed active:translate-y-[2px] active:brightness-90 transition-all" +
							(keyPress === "decelerate"
								? " ring-2 ring-emerald-400 translate-y-[2px] brightness-90"
								: "")
						}
						disabled={isDisabled}
						onClick={() => onCommand({ type: "move_leader", direction: "decelerate" })}
					>
						▼
					</button>
				</div>
			</div>

			{/* altitude controls */}
			<div className="mb-4">
				<div className="mb-1 tracking-wide">ALTITUDE</div>
				<div className="flex gap-2">
					<button
						className={
							"flex-1 mc-button btn-glow nasa-text font-semibold tracking-widest text-[0.85rem] px-3 py-2 bg-emerald-900/50 hover:bg-emerald-500/30 border border-emerald-300/70 text-emerald-200 shadow-[0_0_20px_rgba(16,185,129,0.9)] drop-shadow-[0_0_6px_rgba(16,185,129,0.9)] disabled:opacity-40 disabled:cursor-not-allowed active:translate-y-[2px] active:brightness-90 transition-all" +
							(keyPress === "alt_up"
								? " ring-2 ring-emerald-400 translate-y-[2px] brightness-90"
								: "")
						}
						disabled={isDisabled}
						onClick={() => onCommand({ type: "altitude_change", amount: +10 })}
					>
						+10m
					</button>
					<button
						className={
							"flex-1 mc-button btn-glow nasa-text font-semibold tracking-widest text-[0.85rem] px-3 py-2 bg-emerald-900/50 hover:bg-emerald-500/30 border border-emerald-300/70 text-emerald-200 shadow-[0_0_20px_rgba(16,185,129,0.9)] drop-shadow-[0_0_6px_rgba(16,185,129,0.9)] disabled:opacity-40 disabled:cursor-not-allowed active:translate-y-[2px] active:brightness-90 transition-all" +
							(keyPress === "alt_down"
								? " ring-2 ring-emerald-400 translate-y-[2px] brightness-90"
								: "")
						}
						disabled={isDisabled}
						onClick={() => onCommand({ type: "altitude_change", amount: -10 })}
					>
						−10m
					</button>
				</div>
			</div>

			{/* formation controls */}
			<div className="mb-4">
				<div className="mb-1 tracking-wide">FORMATION</div>
				<div className="grid grid-cols-3 gap-2">
					<button
						className="mc-button btn-glow nasa-text font-semibold tracking-widest text-[0.85rem] px-3 py-2 bg-emerald-900/50 hover:bg-emerald-500/30 border border-emerald-300/70 text-emerald-200 shadow-[0_0_20px_rgba(16,185,129,0.9)] drop-shadow-[0_0_6px_rgba(16,185,129,0.9)] disabled:opacity-40 disabled:cursor-not-allowed active:translate-y-[2px] active:brightness-90 transition-all"
						disabled={isDisabled}
						onClick={() => onCommand({ type: "formation", mode: "line" })}
					>
						LINE
					</button>
					<button
						className="mc-button btn-glow nasa-text font-semibold tracking-widest text-[0.85rem] px-3 py-2 bg-emerald-900/50 hover:bg-emerald-500/30 border border-emerald-300/70 text-emerald-200 shadow-[0_0_20px_rgba(16,185,129,0.9)] drop-shadow-[0_0_6px_rgba(16,185,129,0.9)] disabled:opacity-40 disabled:cursor-not-allowed active:translate-y-[2px] active:brightness-90 transition-all"
						disabled={isDisabled}
						onClick={() => onCommand({ type: "formation", mode: "vee" })}
					>
						VEE
					</button>
					<button
						className="mc-button btn-glow nasa-text font-semibold tracking-widest text-[0.85rem] px-3 py-2 bg-emerald-900/50 hover:bg-emerald-500/30 border border-emerald-300/70 text-emerald-200 shadow-[0_0_20px_rgba(16,185,129,0.9)] drop-shadow-[0_0_6px_rgba(16,185,129,0.9)] disabled:opacity-40 disabled:cursor-not-allowed active:translate-y-[2px] active:brightness-90 transition-all"
						disabled={isDisabled}
						onClick={() => onCommand({ type: "formation", mode: "circle" })}
					>
						CIRCLE
					</button>
				</div>
			</div>

			{/* global controls */}
			<div className="mb-1 tracking-wide">GLOBAL COMMANDS</div>
			<div className="grid grid-cols-3 gap-2">
				<button
					className="mc-button btn-glow nasa-text font-semibold tracking-widest text-[0.85rem] px-3 py-2 bg-emerald-900/50 hover:bg-emerald-500/30 border border-emerald-300/70 text-emerald-200 shadow-[0_0_20px_rgba(16,185,129,0.9)] drop-shadow-[0_0_6px_rgba(16,185,129,0.9)] disabled:opacity-40 disabled:cursor-not-allowed active:translate-y-[2px] active:brightness-90 transition-all"
					disabled={isDisabled}
					onClick={() => onCommand({ type: "pause" })}
				>
					PAUSE
				</button>
				<button
					className="mc-button btn-glow nasa-text font-semibold tracking-widest text-[0.85rem] px-3 py-2 bg-emerald-900/50 hover:bg-emerald-500/30 border border-emerald-300/70 text-emerald-200 shadow-[0_0_20px_rgba(16,185,129,0.9)] drop-shadow-[0_0_6px_rgba(16,185,129,0.9)] disabled:opacity-40 disabled:cursor-not-allowed active:translate-y-[2px] active:brightness-90 transition-all"
					disabled={isDisabled}
					onClick={() => onCommand({ type: "resume" })}
				>
					RESUME
				</button>
				<button
					className="mc-button btn-glow nasa-text font-semibold tracking-widest text-[0.85rem] px-3 py-2 bg-emerald-900/50 hover:bg-emerald-500/30 border border-emerald-300/70 text-emerald-200 shadow-[0_0_20px_rgba(16,185,129,0.9)] drop-shadow-[0_0_6px_rgba(16,185,129,0.9)] disabled:opacity-40 disabled:cursor-not-allowed active:translate-y-[2px] active:brightness-90 transition-all"
					disabled={isDisabled}
					onClick={() => onCommand({ type: "rtb" })}
				>
					RTB
				</button>
			</div>
		</section>
	);
}
