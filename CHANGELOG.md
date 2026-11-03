# Changelog

## ubuntu-22.04 branch — 2026-11-03

- Target Ubuntu 22.04 / ROS 2 Humble with OSRF Gazebo Harmonic.
- Make Jazzy vendor-package discovery optional, with Humble defaulting to system libraries.
- Default launchers to Humble while respecting an explicitly selected ROS environment.
- Support Jammy's Pillow 9.0 API in QR generation and map comparisons.
- Keep 24.04 evidence labelled as the historical baseline; Jammy runtime validation remains pending.

## 0.5.0 — 2026-11-03

- Replace continuous gray-area obstacle sampling with 13 reference-image candidate slots.
- Add random, solvable, blocked, stress and manual slot modes with seeded replay.
- Share positions across SDF, maps, records and fixed-heading A*.
- Add visual-only candidate labels and actual Gazebo acceptance evidence.
- Validate 1000 seeds, all 13 physical contacts and existing station/material behavior.
