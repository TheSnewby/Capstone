"use client";

import React from "react";

export type SwarmSettings = {
  cohesion: number; // how strongly UAVs pull toward group center
  separation: number; // how strongly they avoid crowding
  alignment: number; // how strongly they match neighbor heading
  maxSpeed: number; // cap on speed (m/s)
  targetAltitude: number; // preferred cruise altitude
};

type SwarmControlsProps = {
  settings: SwarmSettings;
  onChange: (next: SwarmSettings) => void;
};

/**
 * SwarmControls
 *
 * panel for tuning swarm behavior parameters
 * - exposes cohesion / separation / alignment weights
 * - controls max speed and target altitude
 * - sliders update in real-time and send changes upstream
 */
export default function SwarmControls({ settings, onChange }: SwarmControlsProps) {
  const updateField = (field: keyof SwarmSettings, value: number) => {
    onChange({ ...settings, [field]: value });
  };

  return (
    <section className="mc-panel mc-panel-inner bg-gradient-to-b from-black/80 to-black/95 nasa-text text-xs border border-emerald-700/40 shadow-[0_0_12px_rgba(16,185,129,0.25)]">
      <h2 className="mb-4">
        <div className="inline-flex items-center px-3 py-2 rounded-sm border border-emerald-400/70 bg-emerald-900/40 shadow-[0_0_18px_rgba(16,185,129,0.7)]">
          <span className="uppercase tracking-[0.3em] text-[0.85rem] font-bold text-emerald-200 drop-shadow-[0_0_10px_rgba(16,185,129,1)]">
            Swarm Behavior Controls
          </span>
        </div>
      </h2>

      <div className="space-y-4">
        {/* cohesion */}
        <ControlRow
          label="COHESION"
          min={0}
          max={2}
          step={0.05}
          value={settings.cohesion}
          onChange={(v) => updateField("cohesion", v)}
        />

        {/* separation */}
        <ControlRow
          label="SEPARATION"
          min={0}
          max={2}
          step={0.05}
          value={settings.separation}
          onChange={(v) => updateField("separation", v)}
        />

        {/* alignment */}
        <ControlRow
          label="ALIGNMENT"
          min={0}
          max={2}
          step={0.05}
          value={settings.alignment}
          onChange={(v) => updateField("alignment", v)}
        />

        <div className="h-px bg-zinc-800 my-2" />

        {/* max Speed */}
        <ControlRow
          label="MAX SPEED (m/s)"
          min={0}
          max={60}
          step={1}
          value={settings.maxSpeed}
          onChange={(v) => updateField("maxSpeed", v)}
        />

        {/* target Altitude */}
        <ControlRow
          label="TARGET ALT (m)"
          min={0}
          max={500}
          step={10}
          value={settings.targetAltitude}
          onChange={(v) => updateField("targetAltitude", v)}
        />
      </div>
    </section>
  );
}

type ControlRowProps = {
  label: string;
  min: number;
  max: number;
  step: number;
  value: number;
  onChange: (value: number) => void;
};

function ControlRow({ label, min, max, step, value, onChange }: ControlRowProps) {
  const formatted = step >= 1 ? value.toFixed(0) : value.toFixed(2);

  return (
    <div>
      <div className="flex justify-between mb-1">
        <span className="text-[0.85rem] tracking-widest text-emerald-300 font-medium">{label}</span>
        <span className="text-[0.85rem] text-emerald-400 font-medium">{formatted}</span>
      </div>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={value}
        onChange={(e) => onChange(parseFloat(e.target.value))}
        className="mc-slider accent-emerald-400 cursor-pointer hover:brightness-110"
      />
    </div>
  );
}
