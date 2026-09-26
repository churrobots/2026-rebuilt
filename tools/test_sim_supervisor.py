import tempfile
import unittest
from pathlib import Path

from sim_supervisor import changed_paths, find_java_home, project_year, snapshot


class SnapshotTests(unittest.TestCase):
    def test_finds_explicit_java_home_and_project_year(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            (root / "build.gradle").write_text(
                'id "edu.wpi.first.GradleRIO" version "2026.2.1"'
            )
            java_home = root / "jdk"
            executable = java_home / "bin" / "java"
            executable.parent.mkdir(parents=True)
            executable.touch()

            self.assertEqual(project_year(root), "2026")
            self.assertEqual(find_java_home(root, java_home), java_home)

    def test_tracks_sources_and_ignores_generated_build_constants(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source = root / "src/main/java/frc/robot/Robot.java"
            generated = source.with_name("BuildConstants.java")
            source.parent.mkdir(parents=True)
            source.write_text("class Robot {}")
            generated.write_text("class BuildConstants {}")
            (root / "build.gradle").write_text("plugins {}")

            state = snapshot(root)

            self.assertIn(Path("src/main/java/frc/robot/Robot.java"), state)
            self.assertIn(Path("build.gradle"), state)
            self.assertNotIn(
                Path("src/main/java/frc/robot/BuildConstants.java"), state
            )

    def test_reports_creates_modifications_and_deletes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source = root / "src/main/java/Robot.java"
            source.parent.mkdir(parents=True)
            source.write_text("one")
            before = snapshot(root)

            source.write_text("a longer version")
            deploy = root / "src/main/deploy/config.json"
            deploy.parent.mkdir(parents=True)
            deploy.write_text("{}")
            after = snapshot(root)

            self.assertEqual(
                changed_paths(before, after),
                [
                    Path("src/main/deploy/config.json"),
                    Path("src/main/java/Robot.java"),
                ],
            )

            source.unlink()
            self.assertEqual(
                changed_paths(after, snapshot(root)),
                [Path("src/main/java/Robot.java")],
            )


if __name__ == "__main__":
    unittest.main()
