"""Tests for machine-readable report contracts."""

from __future__ import annotations

from pathlib import Path

from bagx.anomaly import detect_anomalies
from bagx.batch import batch_anomaly
from bagx.compare import compare_bags
from bagx.contracts import REPORT_SCHEMA_VERSION
from bagx.eval import evaluate_bag
from bagx.scenario import detect_scenarios
from bagx.scene import extract_scene
from bagx.sync import analyze_sync


def _assert_contract_metadata(data: dict, report_type: str) -> None:
    assert data["schema_version"] == REPORT_SCHEMA_VERSION
    assert data["report_type"] == report_type
    assert isinstance(data["bagx_version"], str)
    assert data["bagx_version"]


class TestReportContracts:
    def test_eval_contract_metadata(self, multi_bag: Path):
        _assert_contract_metadata(evaluate_bag(str(multi_bag)).to_dict(), "eval")

    def test_compare_contract_metadata(self, gnss_bag: Path, gnss_bag_degraded: Path):
        _assert_contract_metadata(compare_bags(str(gnss_bag), str(gnss_bag_degraded)).to_dict(), "compare")

    def test_sync_contract_metadata(self, multi_bag: Path):
        _assert_contract_metadata(analyze_sync(str(multi_bag), "/gnss", "/imu").to_dict(), "sync")

    def test_anomaly_contract_metadata(self, multi_bag: Path):
        _assert_contract_metadata(detect_anomalies(str(multi_bag)).to_dict(), "anomaly")

    def test_scenario_contract_metadata(self, multi_bag: Path):
        _assert_contract_metadata(detect_scenarios(str(multi_bag)).to_dict(), "scenario")

    def test_scene_contract_metadata(self, nav2_bag: Path):
        _assert_contract_metadata(extract_scene(str(nav2_bag)).to_dict(), "scene")

    def test_batch_anomaly_contract_metadata(self, gnss_bag: Path, imu_bag: Path):
        _assert_contract_metadata(batch_anomaly([str(gnss_bag), str(imu_bag)]), "batch_anomaly")
