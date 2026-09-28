# EAST ArUco and colour-marker video analysis guide

## What this setup measures

The four **ArUco markers (IDs 0–3)** are mounted separately around the visible movement area on the stationary EAST frame. Together they define a stable measurement plane. The **yellow and magenta targets** are attached to the same rigid moving component. By default, the hybrid detector locates each large coloured circle and then tracks its small black centre dot. It records both locations and falls back to the colour centroid only when the dot is unavailable. The script measures the orientation of the line from the selected yellow point to the selected magenta point, subtracts the initial machine-zero orientation, and reports angle against time. It then fits angular speed through the central **−8° to +8°** window.

This is an independent optical check. It does not replace machine zero, load-cell tare, the physical E-stop, or the EAST operating procedure.

## 1. Print and prepare the targets

1. Open `EAST_ArUco_A4_Marker_Kit.pdf`.
2. Print both pages on **A4, portrait, Actual Size / 100%**. Disable “Fit”, “Scale to fit”, borderless enlargement, and booklet modes.
3. Use high-quality colour printing on matte white paper. Gloss causes reflections.
4. Measure the printed 100 mm line with a ruler. Accept the sheet only if it measures **100 mm within ±0.5 mm**.
5. Measure one black ArUco square. It must be **45 mm within ±0.5 mm**.
6. Cut out the four individual ArUco cards along the dashed outlines. Mount each one flat on rigid card without covering the black square or its surrounding white border.
7. Cut one 40 mm magenta target and one 40 mm yellow target from page 2. Mount them on thin stiff card or use equivalent matte vinyl stickers. Keep the coloured circle intact.

## 2. Install the four fixed ArUco markers

1. Isolate the machine before placing anything inside the enclosure.
2. Mount the four cards around the motion area: ID 0 top-left, ID 1 top-right, ID 2 bottom-right and ID 3 bottom-left.
3. Attach every marker to a **stationary, rigid frame member**. Do not use the enclosure door or the moving linkage.
4. Spread the markers widely enough to surround the complete coloured-target path, while keeping them in approximately the same depth plane as the targets.
5. Make sure all four markers remain visible throughout the full ROM. Cables, hands, the AFO, linkage, and reflections must not cover them.
6. Measure each marker centre position in millimetres. Use ID 0 as `(0, 0)`. For a rectangular layout, ID 1 is `(width, 0)`, ID 2 is `(width, height)` and ID 3 is `(0, height)`.
7. Copy `reference_layout_template.json`, enter the measured centre coordinates, and keep it with the experiment record.

## 3. Install the moving colour targets

1. Choose the single rigid moving component whose angle represents the EAST output/AFO fixture angle.
2. Put **both targets on that same component**. Do not put one on the frame and one on the moving part.
3. Put yellow nearer one end and magenta farther along the member. The analysis vector runs **yellow → magenta**.
4. Separate their centres by at least **80 mm**; 100–150 mm is preferable if both remain visible.
5. Keep the targets flat, facing the camera, and away from joints or flexible AFO material.
6. Move the fixture slowly through the complete ROM by the approved setup method and confirm that neither target is hidden.

If the moving member is too narrow, attach a light rigid target plate to it. Confirm that the plate cannot contact the frame or change the mechanism’s motion.

## 4. Position the iPhone

1. Mount the iPhone rigidly on a tripod; do not hand-hold it.
2. Use the rear **1× camera**. Avoid 0.5× because its wider perspective and lens distortion increase error.
3. Start approximately **1.5–2.5 m away**, then adjust so the four ArUcos and both colour targets fill most of the frame with margin at full ROM.
4. Place the camera lens perpendicular to the plane of motion. Centre the lens approximately on the moving-marker plane.
5. Use landscape orientation and record at **1080p/60 fps or 4K/60 fps** if storage allows.
6. Lock the tripod, focus, and exposure. Use even diffuse lighting and remove strong reflections.
7. Do not move or touch the camera, board, or targets between trials. If anything moves, treat the next recording as a new setup.

## 5. Record each trial

1. Establish and verify EAST machine zero at physical 90° using the approved square procedure.
2. Bring the fixture to machine zero.
3. Start video recording.
4. Keep the fixture stationary at machine zero for at least **2 seconds**. The script uses this interval as the optical zero.
5. Run the EAST test through the required cycles.
6. After the final return to machine zero, keep recording for another 2 seconds.
7. Stop recording and name the file with speed, trial, and date, for example `Speed_05deg_s_Trial_01_20260928.MOV`.
8. Do not edit, trim, slow, or export the original before analysis. Copy it intact.

## 6. Install the software

Use Python 3.10 or newer. In Terminal, change to the folder containing the supplied files and run:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements_aruco_video.txt
```

On Windows, activate with `.venv\Scripts\activate`.

## 7. Analyse a video

Run:

```bash
python east_aruco_video_analysis.py \
  "/path/to/Speed_05deg_s_Trial_01.MOV" \
  --output "/path/to/results/Speed_05deg_s_Trial_01" \
  --reference-layout "/path/to/reference_layout.json" \
  --reference-mode aruco \
  --point-source hybrid \
  --require-black-dots \
  --neutral-seconds 2 \
  --fit-limit-deg 8 \
  --overlay
```

This formal path requires all four ArUcos by default. The layout JSON must contain the measured marker-centre positions from the installed machine; do not substitute dimensions from the printable sheet.

If one fixed ArUco is temporarily unusable, a measured three-marker affine correction can be explored with `--minimum-aruco-markers 3`. It is classified as exploratory and should not replace a successful four-marker validation.

To run the requested yellow/magenta-only exploratory analysis, with no ArUco detection or plane correction, use:

```bash
python east_aruco_video_analysis.py \
  "/path/to/video.mp4" \
  --output "/path/to/results/marker_only" \
  --reference-mode marker-only \
  --point-source hybrid \
  --require-black-dots \
  --neutral-seconds 2 \
  --fit-limit-deg 8 \
  --overlay
```

Marker-only angles come directly from image pixels. They can be useful for comparison when the camera is fixed and nearly perpendicular, but they do not correct camera movement or perspective and are always labelled exploratory.

If dorsiflexion appears negative when it should be positive, repeat with:

```bash
--dorsiflexion-direction decreasing
```

The results folder contains:

- `frame_data.csv`: time, colour centroids, black-dot centres, selected point source, ArUco transform, orientation, neutral-relative angle, and frame validity;
- `sweep_summary.csv`: central-window fitted speed and R² for each detected sweep;
- `analysis_summary.json`: overall speed, detection rate, thresholds, and QC result;
- `detection_overlay.mp4`: visual evidence showing detected ArUcos and coloured targets.

## 8. Review before accepting a result

1. Watch the complete overlay video.
2. Confirm the colour-circle indicators and centre-dot crosses stay on the correct targets. For final analysis, use `--require-black-dots` so a fallback cannot pass unnoticed.
3. Confirm all four ArUco IDs are detected for nearly every frame.
4. Require at least **95% valid frames** and a valid speed fit. The script reports this as `qc_pass`.
5. Review each sweep’s R². For formal speed verification, investigate any fit below **0.995**.
6. Confirm the angle begins near 0°, reaches the intended endpoints, and returns near 0°.
7. Retain the original video, marker sheet version, script version, output CSV/JSON, and overlay together.

## 9. If detection fails

- **ArUco misses:** move closer, increase resolution/light, reduce glare, flatten the board, and keep more white space around every marker.
- **Yellow is lost:** reduce bright reflections and white clipping; improve saturation by using matte printed targets or matte vinyl.
- **Magenta is lost:** remove similarly coloured objects from the view and improve lighting.
- **Wrong coloured object selected:** crop the camera view physically or adjust the hue thresholds shown by `--help`.
- **Angle is noisy:** increase target separation, use a firmer mount, shorten exposure with brighter light, and verify both targets are on one rigid part.
- **Systematic angle error remains:** reduce the depth difference between the four references and moving targets, make the camera more perpendicular, and perform a static check at known −10°, 0°, and +10° positions.

## 10. Minimum validation before thesis data collection

Record a short static validation video at machine positions −10°, 0°, and +10° using the same camera setup. Compare optical readings against an independent inclinometer or angle reference. Then record one trial at 1, 5, 10, and 20°/s and compare the new pipeline with the existing Tracker workflow. Do not treat the computer-vision result as validated until static angle error, repeatability, and overlay review are acceptable.

## 11. Calculating the experimental degrees-per-turn factor

For each accepted constant-speed sweep, calculate:

```text
degrees per ODrive turn = optical angular speed (deg/s) / ODrive turn speed (turns/s)
```

Also calculate it independently from static displacement:

```text
degrees per ODrive turn = optical angle change (deg) / ODrive position change (turns)
```

Use both directions, all cycles, and all four commanded speeds. Report the mean, standard deviation, confidence interval, direction difference, and speed dependence. Do not replace the software value merely because one video produces a different factor. Accept a new factor only if the static and dynamic methods agree and the result remains stable across camera repetitions and speeds.
