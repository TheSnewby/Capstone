"use client";

import Link from "next/link";
import Image from "next/image";

export default function DevelopersPage() {
	return (
		<main className="min-h-screen w-full bg-black text-white flex justify-center items-start py-8">
			<div className="w-full max-w-4xl px-4 crt-scanlines mc-panel">
				<header className="flex items-baseline justify-between mb-6">
					<div>
						<h1 className="text-3xl md:text-4xl font-semibold nasa-text tracking-widest mb-1 text-emerald-300">
							SkyWeave Developers
						</h1>
						<p className="text-emerald-200 text-base font-sans tracking-wide">
							The Team Behind the Swarm
						</p>
					</div>

					<div className="text-xs nasa-text">
						<Link href="/" className="mc-button btn-glow px-4 py-1 text-emerald-200">
							⟵ Back to SkyWeave
						</Link>
					</div>
				</header>
				<div className="w-full h-px bg-emerald-700/50 shadow-[0_0_10px_rgba(16,185,129,0.35)] mb-8"></div>

				<section className="grid gap-4 md:grid-cols-2 relative">
					<article className="mc-panel-inner font-sans text-base leading-relaxed tracking-wide bg-gradient-to-b from-black/80 to-black/95 border border-emerald-700/40 shadow-[0_0_12px_rgba(16,185,129,0.25)] p-6">
						<div className="w-full flex justify-center mb-4">
						  <div className="transition-transform hover:scale-105 hover:shadow-[0_0_40px_rgba(16,185,129,0.75)] rounded-full">
						    <Image
						      src="/nash.jpg"
						      alt="Nash Thames"
						      width={160}
						      height={160}
						      className="w-40 h-40 rounded-full object-cover border-2 border-emerald-400 shadow-[0_0_28px_rgba(16,185,129,0.55)]"
						    />
						  </div>
						</div>
						<h2 className="text-xl font-semibold nasa-text tracking-widest uppercase mb-2 text-emerald-200">Nash Thames</h2>
						<p className="text-emerald-400 text-sm mb-6 font-sans tracking-wide">
							Backend & Telemetry • Rust • WebSockets • Next.js
						</p>
						<p className="text-emerald-100 mb-4">
							Nash is a systems-focused software developer specializing in Rust,
							blockchain, and low-level architecture. He designed and implemented
							the SkyWeave Rust telemetry server, bridging the C++ simulator to
							the real-time 3D dashboard.
						</p>
						<p className="text-emerald-300 text-xs mb-4">
							Key Contributions: telemetry pipeline, WebSocket API, 3D HUD
							wiring, UI state & controls, Next.js site architecture
						</p>
						<div className="flex gap-3 mt-3">
						  <a href="https://github.com/internashionalist" target="_blank" className="mc-button btn-glow text-xs px-3 py-1">GitHub</a>
						  <a href="https://linkedin.com/in/nashthames" target="_blank" className="mc-button btn-glow text-xs px-3 py-1">LinkedIn</a>
						</div>
					</article>

					<div className="block md:hidden w-full h-px bg-emerald-700/40 my-6"></div>

					{/* Partner */}
					<article className="mc-panel-inner font-sans text-base leading-relaxed tracking-wide bg-gradient-to-b from-black/80 to-black/95 border border-emerald-700/40 shadow-[0_0_12px_rgba(16,185,129,0.25)] p-6">
						<div className="w-full flex justify-center mb-4">
						  <div className="transition-transform hover:scale-105 hover:shadow-[0_0_40px_rgba(16,185,129,0.75)] rounded-full">
						    <Image
						      src="/stephen.jpg"
						      alt="Stephen Newby"
						      width={160}
						      height={160}
						      className="w-40 h-40 rounded-full object-cover border-2 border-emerald-400 shadow-[0_0_28px_rgba(16,185,129,0.55)]"
						    />
						  </div>
						</div>
						<h2 className="text-xl font-semibold nasa-text tracking-widest uppercase mb-2 text-emerald-200">Stephen Newby</h2>
						<p className="text-emerald-400 text-sm mb-6 font-sans tracking-wide">
							Simulation & Flight Dynamics • C++ • UDP • JSON
						</p>
						<p className="text-emerald-100 mb-4">
							Stephen is an aerospace software engineer and low-level systems architect focused
							on development and real-time data transmission. He constructed the UAV simulation,
							movement logic, and telemetry encoder that powers SkyWeave.
						</p>
						<p className="text-emerald-300 text-xs mb-4">
							Key Contributions: C++ sim core, UDP broadcaster, swarm behavior
							parameters, flight dynamics, multithreaded simulation loop
						</p>
						<div className="flex gap-3 mt-3">
						  <a href="https://github.com/TheSnewby" target="_blank" className="mc-button btn-glow text-xs px-3 py-1">GitHub</a>
						  <a href="https://www.linkedin.com/in/stephenjnewby/" target="_blank" className="mc-button btn-glow text-xs px-3 py-1">LinkedIn</a>
						</div>
					</article>
					<div className="hidden md:block absolute left-1/2 top-0 h-full w-px bg-zinc-700/50"></div>
				</section>
				<div className="w-full h-px bg-emerald-700/40 my-8"></div>

				<section className="mt-10 font-sans text-sm text-emerald-200 leading-relaxed tracking-wide px-2">
					<p>
						SkyWeave was built as a capstone project for Atlas School to explore real-time
						systems: C++ simulation, Rust networking, and a 3D telemetry
						dashboard in Next.js/React Three Fiber.
					</p>
				</section>
			</div>
		</main>
	);
}
