import ast
import unittest
from pathlib import Path

from east_core import CSV_COLUMNS


PROJECT_DIR = Path(__file__).resolve().parents[1]


class GuiContractTests(unittest.TestCase):
    def test_main_gui_has_persistent_test_parameter_labels(self):
        source = (PROJECT_DIR / "Ortho-Sim.py").read_text(encoding="utf-8")
        tree = ast.parse(source)
        displayed_strings = {
            node.value
            for node in ast.walk(tree)
            if isinstance(node, ast.Constant) and isinstance(node.value, str)
        }
        expected_labels = {
            "Test Parameters",
            "File Name Prefix",
            "Number of Cycles",
            "Speed (\N{DEGREE SIGN}/s)",
            "Acceleration (\N{DEGREE SIGN}/s\N{SUPERSCRIPT TWO})",
            "Minimum Angle (\N{DEGREE SIGN})",
            "Maximum Angle (\N{DEGREE SIGN})",
            "Operator ID",
            "AFO ID",
            "Fixture ID",
            "Calibration ID",
        }
        self.assertTrue(expected_labels.issubset(displayed_strings))
        self.assertIn('"acceleration_deg_s2": self.acceleration_input.get()', source)
        self.assertTrue(any(value.startswith("Commanded:") for value in displayed_strings))

    def test_preview_matches_main_input_labels(self):
        source = (
            PROJECT_DIR / "Testing Scripts" / "gui-layout-preview.py"
        ).read_text(encoding="utf-8")
        tree = ast.parse(source)
        displayed_strings = {
            node.value
            for node in ast.walk(tree)
            if isinstance(node, ast.Constant) and isinstance(node.value, str)
        }
        for label in (
            "Test Parameters",
            "Speed (\N{DEGREE SIGN}/s)",
            "Acceleration (\N{DEGREE SIGN}/s\N{SUPERSCRIPT TWO})",
            "Minimum Angle (\N{DEGREE SIGN})",
            "Maximum Angle (\N{DEGREE SIGN})",
        ):
            self.assertIn(label, displayed_strings)

    def test_main_and_diagnostic_csv_rows_match_shared_schema(self):
        main_tree = ast.parse((PROJECT_DIR / "Ortho-Sim.py").read_text(encoding="utf-8"))
        main_rows = [
            node.value
            for node in ast.walk(main_tree)
            if isinstance(node, ast.Assign)
            and any(isinstance(target, ast.Name) and target.id == "data_row" for target in node.targets)
            and isinstance(node.value, ast.List)
        ]
        self.assertEqual(len(main_rows), 1)
        self.assertEqual(len(main_rows[0].elts), len(CSV_COLUMNS))

        diagnostic_tree = ast.parse(
            (PROJECT_DIR / "Testing Scripts" / "afo-strain-test-script.py").read_text(
                encoding="utf-8"
            )
        )
        diagnostic_rows = [
            node.args[0]
            for node in ast.walk(diagnostic_tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "writerow"
            and node.args
            and isinstance(node.args[0], ast.List)
        ]
        self.assertEqual(len(diagnostic_rows), 1)
        self.assertEqual(len(diagnostic_rows[0].elts), len(CSV_COLUMNS))

    def test_main_has_no_fixed_relative_zero_or_automatic_error_clear(self):
        source = (PROJECT_DIR / "Ortho-Sim.py").read_text(encoding="utf-8")
        config_source = (PROJECT_DIR / "tester_config.json").read_text(encoding="utf-8")
        self.assertNotIn('"neutral_position_turns"', config_source)
        self.assertNotIn("fixed 90 degree neutral", source)
        self.assertNotIn("clear_errors()", source)
        self.assertNotIn("starting_position", source)
        self.assertIn("Set Machine Zero \\N{EM DASH} Fixture at 90\\N{DEGREE SIGN}", source)
        self.assertIn("reference_manager.target_for_angle", source)

    def test_machine_zero_and_tare_workflow_contract(self):
        source = (PROJECT_DIR / "Ortho-Sim.py").read_text(encoding="utf-8")
        tree = ast.parse(source)
        functions = {
            node.name: node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)
        }
        start_source = ast.get_source_segment(source, functions["start_strain_test"])
        set_zero_source = ast.get_source_segment(source, functions["set_machine_zero"])
        return_source = ast.get_source_segment(source, functions["return_to_machine_zero"])
        recovery_source = ast.get_source_segment(source, functions["recovery_jog"])
        acknowledgement_source = ast.get_source_segment(
            source, functions["_accept_recovery_session_acknowledgement"]
        )
        refresh_source = ast.get_source_segment(source, functions["_refresh_motion_controls"])

        self.assertNotIn("tare_scale", source)
        self.assertIn("validate_empty_machine_tare", start_source)
        self.assertNotIn("getVoltageRatio", start_source)
        self.assertNotIn("_submit_position", set_zero_source)
        self.assertIn("invalidate_empty_machine_tare", set_zero_source)
        self.assertNotIn("establish_at_physical_neutral", start_source)
        self.assertIn("Return to Machine Zero", start_source)
        self.assertNotIn("manual_mode.get", return_source)
        self.assertNotIn("recovery_timeout", recovery_source)
        self.assertIn("recovery_jog_maximum_cumulative_deg", recovery_source)
        self.assertIn("invalidate_empty_machine_tare", recovery_source)
        self.assertIn("recovery_session_acknowledged", acknowledgement_source)
        self.assertIn(
            'state="normal" if connected and idle_ui else "disabled"', refresh_source
        )
        self.assertIn("Tare Empty Machine", source)
        self.assertIn("Machine zero: SETUP REQUIRED", source)

        for name in (
            "update_parameter_summary",
            "start_strain_test",
            "create_plot_window",
        ):
            function_source = ast.get_source_segment(source, functions[name])
            self.assertNotIn("reference_manager.invalidate", function_source)

    def test_protocol_preset_and_non_destructive_reset_contract(self):
        source = (PROJECT_DIR / "Ortho-Sim.py").read_text(encoding="utf-8")
        tree = ast.parse(source)
        functions = {
            node.name: node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)
        }
        reset_source = ast.get_source_segment(source, functions["reset_test_fields"])
        identity_source = ast.get_source_segment(source, functions["_on_tare_identity_edit"])
        fixture_source = ast.get_source_segment(
            source, functions["_require_matching_reference_fixture"]
        )
        clear_source = ast.get_source_segment(
            source, functions["clear_session_tare"]
        )

        self.assertIn('ctk.StringVar(value="Custom")', source)
        self.assertIn('text="Protocol:"', source)
        self.assertIn('text="Load Preset"', source)
        self.assertIn('text="Reset Test Fields"', source)
        self.assertIn('text="Clear Session Tare"', source)
        self.assertNotIn("Reset Form", source)
        self.assertIn('self.protocol_var.set("Custom")', reset_source)
        self.assertIn("self.file_name_input", reset_source)
        self.assertIn("self.afo_id_input", reset_source)
        for preserved_field in (
            "self.operator_input",
            "self.fixture_id_input",
            "self.calibration_id_input",
            "self.empty_machine_tare = None",
            "reference_manager.invalidate",
            "safe_idle_motor",
        ):
            self.assertNotIn(preserved_field, reset_source)

        self.assertNotIn("invalidate_empty_machine_tare", identity_source)
        self.assertIn("update_tare_display", identity_source)
        self.assertNotIn("reference_manager.invalidate", fixture_source)
        self.assertIn("CTkMessagebox", clear_source)
        self.assertLess(
            clear_source.index('confirmation.get() != "Clear Session Tare"'),
            clear_source.index("invalidate_empty_machine_tare"),
        )
        for preserved_state in (
            "reference_manager.invalidate",
            "operator_input.delete",
            "fixture_id_input.delete",
            "calibration_id_input.delete",
            "file_name_input.delete",
        ):
            self.assertNotIn(preserved_state, clear_source)

    def test_pending_protocol_selection_does_not_change_loaded_protocol(self):
        source = (PROJECT_DIR / "Ortho-Sim.py").read_text(encoding="utf-8")
        tree = ast.parse(source)
        functions = {
            node.name: node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)
        }
        status_source = ast.get_source_segment(source, functions["update_protocol_status"])
        collect_source = ast.get_source_segment(source, functions["collect_test_parameters"])
        metadata_source = ast.get_source_segment(source, functions["current_preset_metadata"])
        start_source = ast.get_source_segment(source, functions["start_strain_test"])

        self.assertIn("command=lambda _selection: self.update_protocol_status()", source)
        self.assertIn("self.protocol_var.get()", status_source)
        self.assertIn("self.loaded_preset_key", status_source)
        self.assertIn("Selected:", status_source)
        self.assertIn("press Load Preset to apply", status_source)
        self.assertIn("Active protocol remains:", status_source)
        status_assignments = {
            target.attr
            for node in ast.walk(functions["update_protocol_status"])
            if isinstance(node, (ast.Assign, ast.AnnAssign))
            for target in (
                node.targets if isinstance(node, ast.Assign) else [node.target]
            )
            if isinstance(target, ast.Attribute)
        }
        self.assertNotIn("loaded_preset_key", status_assignments)

        self.assertIn("preset = self._loaded_preset()", collect_source)
        self.assertNotIn("self.protocol_var", collect_source)
        self.assertIn("preset = self._loaded_preset()", metadata_source)
        self.assertNotIn("self.protocol_var", metadata_source)

        self.assertIn('"afo_test": "AFO TEST"', start_source)
        self.assertIn(
            '"empty_machine_baseline": "EMPTY-MACHINE BASELINE"', start_source
        )
        self.assertIn('f"{test_type_label}  |  {active_protocol}', start_source)
        self.assertIn("self.show_resizable_confirmation", start_source)

    def test_start_confirmation_is_resizable_and_scrollable(self):
        source = (PROJECT_DIR / "Ortho-Sim.py").read_text(encoding="utf-8")
        tree = ast.parse(source)
        function = next(
            node for node in ast.walk(tree)
            if isinstance(node, ast.FunctionDef)
            and node.name == "show_resizable_confirmation"
        )
        dialog_source = ast.get_source_segment(source, function)
        self.assertIn("dialog.resizable(True, True)", dialog_source)
        self.assertIn("ctk.CTkTextbox", dialog_source)
        self.assertIn('text="Cancel"', dialog_source)
        self.assertIn("text=confirm_text", dialog_source)
        self.assertIn("self.master.wait_window(dialog)", dialog_source)

    def test_baseline_and_protocol_provenance_are_logged(self):
        source = (PROJECT_DIR / "Ortho-Sim.py").read_text(encoding="utf-8")
        self.assertIn('parameters.test_type == "empty_machine_baseline"', source)
        self.assertIn("Remove the AFO and all removable loads before starting", source)
        self.assertIn('self.run_metadata["baseline_matching"]', source)
        self.assertIn("automatic_subtraction_applied", source)
        self.assertIn("preset_metadata=preset_metadata", source)
        for column in ("Test Type", "Preset Name", "Preset Version", "Preset Modified"):
            self.assertIn(column, CSV_COLUMNS)

    def test_preview_includes_protocol_and_session_controls(self):
        source = (
            PROJECT_DIR / "Testing Scripts" / "gui-layout-preview.py"
        ).read_text(encoding="utf-8")
        self.assertIn('ctk.StringVar(value="Custom")', source)
        self.assertIn('text="Load Preset"', source)
        self.assertIn('text="Reset Test Fields"', source)
        self.assertIn('text="Clear Session Tare"', source)
        self.assertIn("Active protocol: Custom", source)
        self.assertIn("Active protocol remains:", source)
        self.assertIn("tester_config.json", source)

    def test_main_uses_one_gui_event_system(self):
        source = (PROJECT_DIR / "Ortho-Sim.py").read_text(encoding="utf-8")
        self.assertNotIn("PyQt5", source)
        self.assertNotIn("pyqtgraph", source)
        self.assertNotIn("QApplication", source)
        self.assertNotIn("_process_qt_events", source)
        self.assertIn("tk.Canvas", source)

    def test_stop_path_does_not_call_neutral_return(self):
        tree = ast.parse((PROJECT_DIR / "Ortho-Sim.py").read_text(encoding="utf-8"))
        stop_function = next(
            node for node in ast.walk(tree)
            if isinstance(node, ast.FunctionDef) and node.name == "stop_logging"
        )
        calls = {
            node.func.attr
            for node in ast.walk(stop_function)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        }
        self.assertIn("safe_idle_motor", calls)
        self.assertNotIn("return_to_neutral", calls)
        assignments = {
            target.attr
            for node in ast.walk(stop_function)
            if isinstance(node, ast.Assign)
            for target in node.targets
            if isinstance(target, ast.Attribute)
        }
        self.assertNotIn("strain_test_active", assignments)

    def test_position_writes_use_the_coordinator_submission_gate(self):
        source = (PROJECT_DIR / "Ortho-Sim.py").read_text(encoding="utf-8")
        self.assertEqual(source.count(".command_position("), 1)
        self.assertIn("motion_coordinator.submit_command", source)
        self.assertIn('self.plot_container.bind("<Escape>"', source)

    def test_shutdown_tracks_all_motion_workers_before_clean_checkpoint(self):
        source = (PROJECT_DIR / "Ortho-Sim.py").read_text(encoding="utf-8")
        tree = ast.parse(source)
        functions = {
            node.name: node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)
        }
        close_source = ast.get_source_segment(source, functions["on_close"])
        safe_idle_source = ast.get_source_segment(source, functions["safe_idle_motor"])

        for worker in (
            "strain_thread",
            "data_collection_thread",
            "neutral_thread",
            "manual_step_thread",
            "continuous_thread",
            "recovery_thread",
        ):
            self.assertIn(f'"{worker}"', close_source)
        self.assertIn("continuity_ready", close_source)
        self.assertIn("self.motion_coordinator.owner is None", close_source)
        self.assertIn('self.ui_message_queue.put(("reference",))', safe_idle_source)


if __name__ == "__main__":
    unittest.main()
