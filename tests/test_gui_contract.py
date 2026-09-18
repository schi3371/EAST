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
        self.assertIn("Set Current Physical Position as 90 deg Neutral", source)
        self.assertIn("reference_manager.target_for_angle", source)

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


if __name__ == "__main__":
    unittest.main()
