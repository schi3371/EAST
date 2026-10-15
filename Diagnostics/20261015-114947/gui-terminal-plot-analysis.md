# EAST stationary GUI Terminal/Plot comparison — 2026-10-15

## Scope and operator observations

The operator confirmed the fixture was stationary and competing hardware applications
were closed before GUI launch. EAST 1.8.0 was launched with Python 3.10.21 at 11:47:59
AEDT. Normal Connect settled at **CONNECTED / MACHINE-ZERO SETUP REQUIRED**. Unlike the
earlier strictly read-only standalone probes, normal Connect requested and confirmed
controller IDLE. It did not establish machine zero, tare, arm, move, clear errors, or
save controller configuration.

During both the Session Terminal and Plot periods, the operator exercised tab
switching, resizing, and scrolling where available and reported that all remained
responsive. No visible pause, blocked operation, feedback fault, or exception was
observed. No motion, test, jog, return-to-zero, machine-zero, or tare action was used.

- Windows scaling: 150% (`AppliedDPI` 144)
- Display metrics available to the process: 1280 x 720
- Configured launch size: 1040 x 496
- The window was resized during the checks; exact intermediate/final sizes were not
  captured because the GUI window handle was not visible across the diagnostic process
  boundary
- Runtime-state storage: `C:\Users\schi3371\AppData\Local\EAST`
- Preserved-copy storage: `Diagnostics\20261015-114947`

## Original and preserved evidence

| Stage | Wall time | Original runtime path | Preserved copy | SHA-256 |
|---|---|---|---|---|
| Baseline | 11:48:41 AEDT | `C:\Users\schi3371\AppData\Local\EAST\stationary_20261015T114841.json` | `baseline_stationary_20261015T114841.json` | `664CC01BC38ECB4714116DC0E54DC31D9B40ADEC00E2C98833D25F8B0A473732` |
| Terminal | 11:51:31 AEDT | `C:\Users\schi3371\AppData\Local\EAST\stationary_20261015T115131.json` | `terminal_stationary_20261015T115131.json` | `545F129C2B7A31875ACED0C56CD3C808A9EE219CC9DC6E7A8EFDE2919C93BA4A` |
| Plot | 11:57:19 AEDT | `C:\Users\schi3371\AppData\Local\EAST\stationary_20261015T115719.json` | `plot_stationary_20261015T115719.json` | `4F399D95D9A28682E09A3C6E9C479A738E9687125C97E651AEABAB37EBD7177D` |

## Window-selection method

`monitor_timings` is a rolling history capped at 2,048 observations, and GUI timing
maxima are cumulative from application launch. For motor comparisons, each result below
uses only observations in the final 20 motor-seconds ending at that capture. This gives
a complete 20-second Terminal window and a complete 20-second Plot window without
double-counting the rolling history. The baseline was saved shortly after Connect and
contains only 17.906 seconds, so all 566 available baseline samples are reported.

The Terminal and Plot JSONs were saved several minutes apart; their rolling histories
do not represent just the requested observation periods. Cumulative GUI maxima are
therefore reported as cumulative and are not attributed to a tab.

## Motor timing by capture window

| Metric | Baseline (17.906 s) | Terminal (20 s) | Plot (20 s) |
|---|---:|---:|---:|
| Samples | 566 | 794 | 699 |
| Achieved monitor rate | 31.55 Hz | 39.65 Hz | 34.90 Hz |
| Gap median | 31 ms | 31 ms | 31 ms |
| Gap p95 | 47 ms | 46 ms | 47 ms |
| Gap p99 | 109 ms | 62 ms | 63 ms |
| Gap maximum | 125 ms | 93 ms | 78 ms |
| Snapshot median | 0 ms | 0 ms | 0 ms |
| Snapshot p95 | 16 ms | 16 ms | 16 ms |
| Snapshot p99 | 93 ms | 47 ms | 46 ms |
| Snapshot maximum | 109 ms | 62 ms | 47 ms |
| Queue-wait p99 | 0 ms | 0 ms | 0 ms |
| Queue-wait maximum | 0 ms | 16 ms | 0 ms |
| Lock-wait p99 | 0 ms | 0 ms | 0 ms |
| Lock-wait maximum | 16 ms | 0 ms | 0 ms |

The baseline includes connection settling and has the largest tails. Terminal versus
Plot had comparable medians and p95/p99 values; the idle Plot window's achieved rate was
4.75 Hz lower than Terminal. One operator-interactive window per tab is insufficient to
attribute that difference to the tab, especially because resizing/tab switching occurred
and GUI monitoring intentionally waits 8 ms after each complete capture.

## Property-read timing (p95 / p99 / maximum)

| Property | Baseline | Terminal | Plot |
|---|---:|---:|---:|
| Position | 16 / 78 / 93 ms | 15 / 17 / 62 ms | 16 / 31 / 47 ms |
| Velocity | 0 / 16 / 16 ms | 15 / 16 / 47 ms | 15 / 16 / 16 ms |
| Active errors | 15 / 16 / 16 ms | 15 / 16 / 31 ms | 15 / 16 / 32 ms |
| Current state | 11 / 16 / 16 ms | 15 / 16 / 47 ms | 15 / 16 / 16 ms |
| Disarm reason | 0 / 16 / 78 ms | 15 / 16 / 31 ms | 15 / 16 / 16 ms |
| Uptime | 15 / 16 / 16 ms | 15 / 16 / 47 ms | 16 / 16 / 31 ms |

## GUI timing, sensors, and activity

| Capture | Output tab | Cumulative drains | Cumulative max drain | Cumulative max drain gap | Rotary rate | Rotary state |
|---|---|---:|---:|---:|---:|---|
| Baseline | Session Terminal | 639 | 16 ms | 735 ms | 124.63 Hz | connected, valid, ready, within fit range |
| Terminal | Session Terminal | 3,403 | 16 ms | 735 ms | 124.57 Hz | connected, valid, ready, within fit range |
| Plot | Plot | 8,902 | 16 ms | 735 ms | 124.57 Hz | connected, valid, ready, within fit range |

The 735 ms GUI drain-gap maximum already existed in the baseline and did not increase,
so it cannot be attributed to Terminal or Plot. The maximum drain duration stayed at
16 ms. All captures reported engine state `ready`, continuing motor observations, and
fresh valid rotary health with the same connection identity.

Normal Connect opened the load-cell interface, but continuous force acquisition was
not active. No force samples or force recording were requested, and no Bridge CSV was
created by this GUI check. Rotary health callbacks were active, but rotary recording was
not active. Selecting/resizing Plot caused ordinary idle redraws; no test data entered
the plot queue, so live plot updates during acquisition were not tested.

No monitor, persistence, or worker error was visible to the operator. All three JSON
saves completed. The stationary diagnostic schema does not contain a full internal
error ledger, so this observation is not expanded beyond the visible status, `ready`
engine state, continuing samples, and successful persistence.

## Scheduling comparison and limitations

The standalone probe uses deadline scheduling at an 8 ms requested interval. The GUI
feedback monitor waits 8 ms *after* each complete motor capture, making its period the
capture duration plus the wait. The GUI also runs transport dispatch, rotary callbacks,
Tk queue drains, and operator interactions. Its 31.55–39.65 Hz window rates must not be
compared with the standalone 69.10–72.17 Hz as though both implemented the same schedule.

This was an idle/empty Plot check. It does not verify responsiveness during live force
acquisition, recording, powered motion, or populated plotting, and it does not resolve
the reported freezing problem.

## Remaining stages

- Connection failure and stationary disconnect/reconnect recovery: **NOT TESTED**
- Stop/Escape: **NOT TESTED**
- Quit and visible isolation-exit workflow: **NOT TESTED**
- Physical machine zero, empty tare, early-Stop motion, and complete motion runs:
  **NOT TESTED**

The next outstanding stage is connection failure and stationary reconnect/recovery.
It was not started. The GUI was not closed because shutdown recovery is a separate
pending check.
