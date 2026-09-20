# EAST Tracker speed analysis

This tool converts Tracker Point Mass exports into neutral-relative AFO angles,
detects complete cycles, fits angular speed over the central −8° to +8° region
of every complete endpoint-to-endpoint sweep, and produces combined results.

It uses only the Python standard library.

## Tracker export

Export these columns for every video:

```text
t, frame, x, y, θr
```

`frame` is recommended but optional. `θr` must be **position angle in degrees**.
Keep Tracker's coordinate origin at the physical rotation pivot and lock the
coordinate system before tracking.

In the supplied `Speed_1.txt`, dorsiflexion makes position angle decrease from
about 90° toward 80°. With the same camera and axes setup, use
`dorsiflexion_direction=decreasing`, which applies:

```text
AFO angle = 90° − Tracker position angle
```

This produces positive dorsiflexion and negative plantarflexion. If a future
camera/axes setup makes position angle increase during dorsiflexion, record
`increasing` in that trial's manifest row.

## Batch use

Copy `manifest_template.csv`, add one row per video, and keep each `video_id`
unique. Relative file paths are resolved from the manifest's directory.

Run:

```bash
python analyze_tracker_speed.py \
  --manifest my_manifest.csv \
  --output Results
```

For one file:

```bash
python analyze_tracker_speed.py \
  --input Speed_1.txt \
  --video-id EXP03_S01_R01 \
  --commanded-speed 1 \
  --trial 1 \
  --expected-cycles 3 \
  --dorsiflexion-direction decreasing \
  --output Results
```

## Analysis rules

- Physical neutral is fixed at 90°. It is not estimated from the first frames.
- A five-frame rolling median is used only for endpoint detection.
- Endpoints are detected at ±9° to tolerate small tracking/position error.
- Speed is fitted to the raw AFO angle data between −8° and +8°.
- Startup neutral-to-dorsiflexion motion is identified as cycle 0 and excluded.
- A numbered cycle is complete only after DF→PF and PF→DF sweeps both finish.
- A trial is included in the primary results only when all expected cycles were
  detected and the segment regression has R² ≥ 0.995.
- Incomplete trials and segments remain in the outputs with exclusion reasons.

## Outputs

The output directory contains:

- `combined_segment_results.csv`: one row per startup, complete or incomplete movement;
- `combined_trial_summary.csv`: one row per video/trial;
- `analysis_failures.json`: files that could not be processed;
- one folder per video with cleaned data, segment results, quality-control JSON,
  and an SVG angle-time plot.

Review every SVG plot before accepting the batch results. Green overlays are
fit regions included by the protocol; orange overlays were fitted for diagnosis
but excluded from the primary result.

## Tests

From this directory:

```bash
python -m unittest discover -s tests -v
```
