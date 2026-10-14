# Execution refactor baseline — 15 October 2026

HEAD f6dfdb9 (1.7.3-safe-recovery), earlier comparison bf0fbb3.
The incoming working tree already modified Ortho-Sim.py, east_odrive.py,
east_reference.py and east_runtime.py and added analysis/diagnose_east_timing.py.
Those changes are carried forward and reviewed. Original experimental data and
both untracked P3022 bench-report files are excluded from implementation commits.

Incoming offline suite: 207 tests, two failures and three errors (partial changes
were not yet integrated with the legacy test fixtures). These are offline results.

Contracts: 2.055 nominal degrees per motor turn; torque uses motor-derived angle;
rotary plot uses accepted calibrated fixture angle; zero, tare and moving baseline
remain distinct; no automatic reference continuity, watchdog enable or test restart.
150 ms capture / 250 ms freshness production budgets remain unchanged. A transport
call's elapsed deadline does not interrupt Fibre. Windows physical behaviour is
NOT TESTED. Existing logs show both successful and failed runs; they do not isolate
CPU, USB or storage as the sole cause.
