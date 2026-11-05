"""
Scenario-to-model routing.

`mujoco_model_for_scenario` is the ROS-free rule `sim.launch.py` uses to pick the
MuJoCo world: `sst_*` scenarios get the Strange Situation Test world, everything
else (and an empty scenario) keeps the original. Keeping the 12 shipped scenarios
on the original world is the backward-compatibility guarantee, so it is checked
against every real scenario file as well as the prefix rule.
"""

from pathlib import Path

from mecanumbot_sim.sim_scenarios import (
    DEFAULT_MUJOCO_MODEL,
    SST_MUJOCO_MODEL,
    mujoco_model_for_scenario,
)

SCENARIO_ROOT = Path(__file__).resolve().parents[1] / "config" / "sim_scenarios"


def test_empty_scenario_uses_default_world():
    assert mujoco_model_for_scenario("") == DEFAULT_MUJOCO_MODEL


def test_sst_prefix_selects_sst_world():
    assert mujoco_model_for_scenario("sst_episode_1_owner_and_dog") == SST_MUJOCO_MODEL
    assert (
        mujoco_model_for_scenario("sst_episode_4_dog_alone.yaml") == SST_MUJOCO_MODEL
    )


def test_sst_prefix_works_on_a_full_path():
    full = "/abs/share/config/sim_scenarios/sst_episode_2_owner_stranger_dog.yaml"
    assert mujoco_model_for_scenario(full) == SST_MUJOCO_MODEL


def test_non_sst_names_use_default_world():
    for name in (
        "single_human_follow",
        "human_close_follow.yaml",
        "/abs/path/exercise_mat_stationary.yaml",
        "fear_hri_brick_carry",
    ):
        assert mujoco_model_for_scenario(name) == DEFAULT_MUJOCO_MODEL


def test_every_shipped_non_sst_scenario_keeps_the_default_world():
    for path in sorted(SCENARIO_ROOT.glob("*.yaml")):
        if path.name.startswith("sst_"):
            continue
        assert (
            mujoco_model_for_scenario(str(path)) == DEFAULT_MUJOCO_MODEL
        ), f"{path.name} unexpectedly routed off the default world"
