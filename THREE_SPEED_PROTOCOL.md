# Three-speed AFO protocol

**v1.12.0 update:** Select Original or Updated cubic using the main Motion method menu. Presets retain +/-10 degrees; the model-specific +/-9-degree restriction has been removed. Extrapolated cubic movements are flagged in metadata. See [motion selection and extrapolation](MOTION_METHODS.md). Speeds remain requested nominal values.

EAST v1.10.0 provides **Three-Speed AFO Protocol** and **Three-Speed Empty-Machine Baseline** presets. Both run 0.5, 5 and 20 nominal degrees/s, in that order. Defaults are three numbered cycles per speed (nine total), ±10 nominal degrees, and 100 degrees/s² acceleration. These are provisional experimental settings pending bench verification; the existing motor/angle conversion is unchanged.

## Operator workflow

1. Before fitting an AFO, prepare the empty machine using the normal connection, rotary homing and unloaded session tare workflow. Select **Three-Speed Empty-Machine Baseline**, then **Load Preset**. Check the common settings.
2. Press the main **Start** button to open the sequence panel. Check the mounting/setup ID, press **Run Sequence**, and review the single confirmation covering all three runs. The baseline uses the reserved `EMPTY_MACHINE_BASELINE` specimen identity.
3. After the baseline finishes, fit the AFO using the normal mounting and safety procedure. Restore valid reference/connection state if mounting interrupted power. Preserve the valid unloaded tare; do not tare away the AFO load.
4. Enter the actual AFO identity, select **Three-Speed AFO Protocol**, and **Load Preset**. Main **Start** opens its sequence panel. Use a new mounting/setup ID when remounting; reuse an ID only for the same mounting. Optionally enter a baseline batch ID. This is an operator-supplied, unverified provenance link, not an automatic matching check or correction.
5. Press **Run Sequence** and review all settings once. Identity, settings, tare and reference are frozen for the batch. Each speed runs through the normal GUI test, return, settling, idle confirmation and recording cleanup. The next run starts only after these finish and its manifest is saved. No additional inter-run rest is imposed. Actual timestamps/gaps are retained.

The scalar speed entry shows the first speed and is read-only for a sequence. The full sequence is shown in the protocol summary and confirmation. Common cycles, range and acceleration can be changed before confirmation and are marked as modified. Existing single-speed presets remain available.

## Records and interpretation

Each speed produces the normal strain CSV, rotary CSV and metadata JSON, with unique filenames. A separate batch manifest records the entire plan, shared setup identity, protocol version, per-run settings, timestamps, file links and completed/failed/cancelled/skipped outcomes. Per-run metadata links back to the batch and its files. Empty and loaded batches have different batch IDs and test types.

Three speeds on one mounting are not three independent remounts or a reliability study. There is no automatic baseline subtraction, stiffness calculation, or claim that different settings are mechanically equivalent. The known nominal-speed/angle conversion discrepancy remains unchanged.

## Stop and fault handling

Stop/Escape, child-window closure, application shutdown, device faults, recording failures, invalid setup or failed cleanup cancel the remaining plan. Parent inputs and competing motion remain blocked between runs. No automatic retry or resume occurs. Resolve the cause and start a new batch; partial records are preserved. If idle cannot be confirmed, follow the normal physical safe-stop procedure before touching the fixture.

## Verification

Offline tests exercise both complete batches, default nine cycles, metadata and normal recorders, setup freezing, admission failures, recording/manifest faults, Stop during/between runs, child closure and application shutdown. These tests use fake devices and do not establish hardware performance at 20 degrees/s. Verify the configuration on the bench before collecting study data.
