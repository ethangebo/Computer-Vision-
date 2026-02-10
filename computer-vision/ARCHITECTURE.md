# Lunabotics Perception & Navigation Stack

**Target:** NASA Lunabotics 2026 — lunar-sim arena (UCF Exolith Lab, KSC Artemis Arena).  
**Goal:** Detect boulders and craters, navigate around them autonomously, send drive commands to myRIO.

---

## 1. High-Level Pipeline

Your flow is correct. A complete stack looks like this:

```
┌─────────────┐     ┌──────────────┐     ┌─────────────────┐     ┌──────────────┐     ┌─────────────┐
│  RealSense  │ ──► │   Process    │ ──► │  Point cloud +  │ ──► │  Occupancy   │ ──► │  Path       │
│  D455       │     │  (depth etc) │     │  obstacles map  │     │  grid        │     │  plan       │
└─────────────┘     └──────────────┘     └─────────────────┘     └──────────────┘     └──────┬──────┘
         │                    │                      │                     │                  │
         │                    │                      │                     │                  ▼
         │                    │                      │                     │           ┌─────────────┐
         │                    │                      │                     │           │  Navigate   │
         │                    │                      │                     │           │  → myRIO    │
         │                    │                      │                     │           │  (motors)   │
         │                    │                      │                     │           └─────────────┘
         │                    │                      │                     │
         ▼                    ▼                      ▼                     ▼
┌─────────────────────────────────────────────────────────────────────────────────────────────────────┐
│  Localization (UWB beacons)  ──►  pose in arena frame  ──►  optional global map / goal-directed nav  │
└─────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

**What you have now:** Image → process → point cloud → **boulders + craters** → occupancy grid → path plan.  
**What’s missing:** (1) **Localization** (UWB), (2) **Comms** to myRIO (scaffold in `comms.py`), (3) optional **goal-directed** nav using UWB pose.

---

## 2. Module Layout (What the Codebase Should Look Like)

```
computer-vision/
├── config.py              # All tunables (camera, grid, planner, comms, UWB)
├── main.py                # Main loop: sense → map → plan → send to myRIO (optional viz)
├── ARCHITECTURE.md        # This file
│
├── perception/            # Depth → obstacles (boulders + craters)
│   ├── realsense_io.py    # D455 depth stream
│   ├── ground_plane.py    # RANSAC ground plane
│   ├── mapping.py         # Point cloud → obstacle endpoints (positive + negative)
│   └── detection.py       # Optional: boulder vs crater labels (if needed)
│
├── mapping/               # Occupancy / costmap
│   └── occupancy_grid.py  # 2D log-odds grid, ray cast, inflation
│
├── planning/
│   └── planner.py         # Grid → steer, speed (and later: goals from UWB)
│
├── control/
│   └── comms.py           # Send (steer, speed) or (waypoints) to myRIO
│
├── localization/          # UWB (when you add it)
│   └── uwb.py             # Beacons → rover pose in arena frame
│
└── viz/                   # Optional; can disable on Beelink at competition
    ├── viz.py
    └── hud.py
```

Right now most of this lives as flat files (`realsense_io`, `mapping`, `occupancy_grid`, `planner`, etc.). Functionally that’s fine; you can either keep that structure or gradually move modules into `perception/`, `mapping/`, etc. as you grow.

---

## 3. Lunar Arena: Boulders vs Craters

| Type        | Meaning              | Current stack                         | What to add                    |
|-------------|----------------------|----------------------------------------|--------------------------------|
| **Boulders**| Above ground         | ✅ Height above plane → obstacles      | Tuning for regolith, lighting  |
| **Craters** | Below ground (holes) | ✅ Implemented (`CRATER_*` config, `mark_occupied_cells`) | Tune `CRATER_HEIGHT_BELOW_M`, `MAX_CRATER_DEPTH_M` in arena |

**Craters** are “negative obstacles”: depressions the rover can fall into. The stack now:

- Uses the **RANSAC ground plane** as the reference surface.
- Marks **below-ground** points (height < `-CRATER_HEIGHT_BELOW_M`, within `MAX_CRATER_DEPTH_M`) as crater.
- Projects them to 2D and uses **`mark_occupied_cells`** on the occupancy grid (no raycast) so the planner avoids crater footprints.

`mapping` outputs both **boulder** endpoints (raycast) and **crater** footprint points (direct mark). Tune `CRATER_HEIGHT_BELOW_M` and `MAX_CRATER_DEPTH_M` in config for your arena.

---

## 4. UWB Localization

- **Role:** UWB beacons → rover pose (x, y, θ) in arena frame.
- **Use:** Optional global map, “go to goal” behavior, or simple arena-relative logging.
- **Today:** Not integrated. Add a `localization` module that:
  - Reads UWB ranging (or pose) from your hardware/API.
  - Outputs `(x, y, theta)` and optionally covariance.
- **Planning:** For now you can stay **reactive** (local obstacle avoidance only). Later, use UWB pose to build a global map and do goal-directed navigation.

---

## 5. myRIO Comms

- **Role:** Send **drive commands** from Beelink to myRIO; myRIO drives motors.
- **Typical flow:** `(steer, speed)` or `(v_left, v_right)` at a fixed rate (e.g. 10–20 Hz).
- **Transport:** Depends on your myRIO setup (e.g. TCP, UDP, or serial). Your myRIO program must implement the **same protocol** (message format, units, rate).
- **Robustness:** Timeouts, heartbeat, and **E-stop** (e.g. “speed = 0” or dedicated E-stop message) are important.

`comms.py` should define a **clear interface** (e.g. `send(steer, speed)` or `send_velocity(v_left, v_right)`) and hide transport details. Start with a **mock** implementation that logs commands, then swap in real myRIO comms when hardware is ready.

---

## 6. Beelink MINI S12 Pro

- Run perception + planning on the Beelink; keep myRIO for low-level motor control.
- Your current pipeline (RealSense + occupancy grid + sector planner) is lightweight and should be fine.
- At competition, you may want to **disable** OpenCV windows (`SHOW_DEPTH_VIEW`, `SHOW_GRID_VIEW`) to avoid display overhead and run headless.

---

## 7. Recommended Order of Work

1. ~~**Craters (negative obstacles)**~~ — **Done.** Config: `CRATER_HEIGHT_BELOW_M`, `MAX_CRATER_DEPTH_M`.
2. **myRIO comms**  
   - Use `comms.create_comms(mock=False, ...)` when hardware is ready; implement real TCP/serial in `comms.py`.
3. **Wire main loop to comms**  
   - After `plan_from_grid`, call `comms.send(steer, speed)` (optionally rate-limited). E-stop on key or timeout.
4. **UWB localization**  
   - Implement real driver in `localization/uwb.py`; plug pose into optional global map or goal-directed nav.
5. **Goal-directed nav (optional)**  
   - Use UWB pose + goal waypoints; plan toward goal while avoiding obstacles in the local grid.

---

## 8. Quick Reference: Config → Behavior

| Config / module   | Purpose |
|-------------------|---------|
| `ROVER_RADIUS_M`  | Inflation radius for planner (rover footprint). |
| `Z_MIN_M`, `Z_MAX_M` | Depth range used for mapping. |
| `MIN_OBSTACLE_HEIGHT_M` | Min height above ground to count as boulder. |
| `OBSTACLE_MIN_CLUSTER_SIZE` | Reduces noise; increase if too many false obstacles. |
| `SECTOR_DEGS`, `LOOKAHEAD_M` | Planner resolution and lookahead. |
| `STOP_DIST_M`, `SLOW_DIST_M` | Safety distances for stop/slow. |

Tune these on real regolith and lighting (e.g. at UCF) before competition.

---

## 9. What “Done” Looks Like for Perception + Nav

- [x] D455 depth → point cloud → **boulders** (above ground) and **craters** (below ground).
- [x] Occupancy grid updated from both positive and negative obstacles.
- [x] Planner outputs `(steer, speed)` every frame.
- [ ] `comms` sends those commands to myRIO; rover drives accordingly.
- [ ] (Optional) UWB provides pose; (optional) goal-directed nav uses it.
- [ ] Runs headless on Beelink (`RUN_HEADLESS = True`) in lunar sim.

Remaining gaps: **wire main → comms**, **real myRIO driver**, **UWB**, and **headless** for competition.