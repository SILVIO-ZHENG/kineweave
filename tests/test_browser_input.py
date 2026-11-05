"""Validate important semantics in the real UI script."""

from pathlib import Path
import shutil
import subprocess

import pytest


def test_browser_input_freshness_and_deadman():
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node is optional; needed only for browser input logic regression")
    result = subprocess.run([node, str(Path(__file__).with_name("browser_input_smoke.cjs"))],
                            capture_output=True, text=True, encoding="utf-8", timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr
