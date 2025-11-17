# Capstone
Atlas Capstone - UAV Swarm Simulator with Advanced Capabilities

# Description
This is the capstone project that caps a 20 month program at Atlas School - a intensive software engineering school in Tulsa, OK that uses the Holberton School curriculum. 

The aim of this project is to simulate UAV swarms by maintaining flight cohesion through autonomous path-finding, real-time multithreaded communications-handling, and to represent this through a web app as a live telemtry visual.

# Tech Stack
C++ for Simulations
Rust Backend network layer
next.js Frontend visualization and user interaction

UAV Process 1 (Port 8001) ←→ Direct C++ networking ←→ UAV Process 2 (Port 8002)
       ↓                                                        ↓
   HTTP to Rust                                           HTTP to Rust  
       ↓                                                        ↓
    Rust Server ←→ Aggregates all UAV data ←→ WebSocket ←→ Next.js UI

Roll of Rust:
- Collect telemetry from multiple UAV processes

- Aggregate swarm state from distributed sources

- Provide centralized view to Next.js dashboard

- Optional: Act as mission command center (send high-level goals)

# Stretch Goals
Allow Users to take control of an individual UAV and have all other UAVs respond in a way that maintains cohesion. 

Provide obstacles that the UAV will have to safely pathfind around (No-Fly zones, buildings, mountains)

Create a stunning visual interface.

Make 3D

Collision detection.

Allow users to drop waypoints.

Path history with a ghost trail.

Cohesion sliders ( separation distance, max speed, number of drones, etc)

Wind/Turbulence

Formation Choices (Flying V, line, wedge, smiley face, etc)

Battery tracking with return-to-base if battery too low.

Inter-Drone Messaging Issues (latency, packet-loss, limited comm range)

Add more autonomy modes

Limit awareness of each drone such that there's no "global" knowledge.

Adding Tilt/Roll and Yaw visuals

Add a malfunctioning or malicious UAV for "fun"

Have "Missions" for the UAV (start at A, go to B in x time, got to C in y time, orbit a target location, return to base in z time)

Add a red vs blue mechanic where a red team chases a blue team

Incorporate actual drones that are controlled via this(Probably in a future project as it would require a full rewrite to handle the hardware capabilities and limitations)

# Authors
Stephen Newby
Nash Thames