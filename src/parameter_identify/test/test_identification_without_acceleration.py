"""无加速度辨识的回归用例：只用 q/q̇/τ，验收判据为基参数范数相对误差 ≤1%。

fixture `sim_joint6_frozen_500hz.csv` 是仿真激励轨迹的测量结果（500 Hz、10 s、
4 谐波、joint6 冻结模拟实机无电机、每关节叠加 0.002×额定力矩噪声），**不含 acc_* 列**，
与实机 CSV 的形状一致。URDF 就是仿真真值，因此 `base_parameter_relative_error_norm`
可以直接当作误差使用（docs/bugs.md §9）。
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

pytest.importorskip("pinocchio", reason="参数辨识依赖 pinocchio（ROS jazzy 环境）")

from parameter_identify.identify_arm import identify  # noqa: E402

FIXTURE = Path(__file__).parent / "fixtures" / "sim_joint6_frozen_500hz.csv"
URDF = Path(__file__).resolve().parents[2] / "arm_model" / "model" / "robotic_arm.urdf"

# 判据：docs/bugs.md §9 的验收线（O8 的 0.39% 基准用的是同一个范数度量）。
MAX_RELATIVE_ERROR = 0.01
MAX_RMSE = 0.01


def _config() -> dict:
    return {
        "model": {"active_joints": [f"joint{index}" for index in range(1, 7)]},
        "data": {
            "sample_time": 0.002,  # fixture 实际 500 Hz，sample_time 从 time 列自动覆盖
            "acceleration_source": "recorded",  # fixture 无 acc_* → 返回 None，正好测「没有加速度」
            "max_samples": 0,
            "decimate": False,
        },
        "filter": {
            "enabled": True,
            "cutoff_hz": 2.0,
            "drop_initial_seconds": 0.5,
            "finite_difference_step": 1.0e-6,
            "derivative_scheme": "central",
            "qr_tolerance": 1.0e-6,
        },
        "identification": {
            "zero_tolerance": 1.0e-8,
            "qr_tolerance": 1.0e-8,
            "weighted_least_squares": False,
            "has_friction": True,
            "has_actuator_inertia": False,
            "has_joint_offset": True,
        },
        # 本用例只验证回归量本身；SDP/投影依赖 picos，留给实机流程。
        # 关掉重建后 status 会是 "disabled"，必须显式 require_physical=false，
        # 否则 identify_arm 仍按「重建未成功」拒绝写文件。
        "reconstruction": {"enabled": False, "require_physical": False},
    }


@pytest.mark.skipif(not FIXTURE.is_file(), reason=f"缺少 fixture：{FIXTURE}")
def test_identify_without_acceleration_reaches_one_percent(tmp_path: Path) -> None:
    result = identify(
        csv_path=FIXTURE,
        input_urdf=URDF,
        output_urdf=tmp_path / "identified.urdf",
        config=_config(),
        report_path=tmp_path / "identified.identify.yaml",
    )

    # 确认确实走了滤波路径，而且 CSV 没有被读到加速度（否则这条用例就失去意义）。
    assert result["filter"]["enabled"] is True
    assert result["filter"]["derivative_scheme"] == "central"
    assert result["filter"].get("recorded_acceleration_ignored") is None

    assert result["base_parameter_relative_error_norm"] is not None
    assert result["base_parameter_relative_error_norm"] <= MAX_RELATIVE_ERROR, (
        f"基参数范数相对误差 {result['base_parameter_relative_error_norm']:.4%} "
        f"超过 {MAX_RELATIVE_ERROR:.0%}"
    )
    assert result["rmse"] < MAX_RMSE, f"力矩 rmse={result['rmse']:.6g} 超过 {MAX_RMSE}"

    with Path(result["report"]).open("r") as stream:
        report = yaml.safe_load(stream)
    assert report["filter"]["kept_samples"] == report["samples"] - 250  # 500 Hz 丢 0.5 s
