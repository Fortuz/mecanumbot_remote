"""
Drift guard for the SST world.

`mujoco/mecanumbot_sst.xml` is a full copy of `mujoco/mecanumbot.xml` with the
Strange Situation Test furniture added and the two figures recoloured. Until the
two models are deduplicated (an `<include>` refactor), this test keeps the shared
robot and actor/prop pool in step: every body, joint and actuator in the base
model must still exist, by name, in the SST model.

The SST model is allowed to add bodies/geoms (the furniture); it is not allowed to
drop or rename anything the backend node looks up by name.
"""

from pathlib import Path

import pytest

mujoco = pytest.importorskip("mujoco")

MUJOCO_ROOT = Path(__file__).resolve().parents[1] / "mujoco"
BASE_MODEL = MUJOCO_ROOT / "mecanumbot.xml"
SST_MODEL = MUJOCO_ROOT / "mecanumbot_sst.xml"

# Furniture the SST world adds and the base world must not have.
SST_FURNITURE_GEOMS = (
    "chair_1_seat",
    "chair_2_seat",
    "door_marker",
    "water_bowl",
    "location_1",
    "location_2",
)


def _names(model, obj_type) -> set[str]:
    names = set()
    count = {
        mujoco.mjtObj.mjOBJ_BODY: model.nbody,
        mujoco.mjtObj.mjOBJ_JOINT: model.njnt,
        mujoco.mjtObj.mjOBJ_ACTUATOR: model.nu,
        mujoco.mjtObj.mjOBJ_GEOM: model.ngeom,
    }[obj_type]
    for index in range(count):
        name = mujoco.mj_id2name(model, obj_type, index)
        if name:
            names.add(name)
    return names


@pytest.fixture(scope="module")
def base_model():
    return mujoco.MjModel.from_xml_path(str(BASE_MODEL))


@pytest.fixture(scope="module")
def sst_model():
    return mujoco.MjModel.from_xml_path(str(SST_MODEL))


def test_both_models_compile(base_model, sst_model):
    assert base_model.nbody > 0
    assert sst_model.nbody > 0


def test_sst_keeps_every_base_body(base_model, sst_model):
    base_bodies = _names(base_model, mujoco.mjtObj.mjOBJ_BODY)
    sst_bodies = _names(sst_model, mujoco.mjtObj.mjOBJ_BODY)
    missing = base_bodies - sst_bodies
    assert not missing, f"SST model dropped base bodies: {sorted(missing)}"


def test_sst_keeps_every_base_joint(base_model, sst_model):
    base_joints = _names(base_model, mujoco.mjtObj.mjOBJ_JOINT)
    sst_joints = _names(sst_model, mujoco.mjtObj.mjOBJ_JOINT)
    missing = base_joints - sst_joints
    assert not missing, f"SST model dropped base joints: {sorted(missing)}"


def test_sst_keeps_every_base_actuator(base_model, sst_model):
    base_actuators = _names(base_model, mujoco.mjtObj.mjOBJ_ACTUATOR)
    sst_actuators = _names(sst_model, mujoco.mjtObj.mjOBJ_ACTUATOR)
    assert base_actuators == sst_actuators


def test_sst_keeps_pool_and_robot_bodies(sst_model):
    sst_bodies = _names(sst_model, mujoco.mjtObj.mjOBJ_BODY)
    for required in (
        "base_link",
        "base_scan",
        "sim_human_0",
        "sim_human_1",
        "sim_wall_panel_0",
        "sim_wall_panel_1",
        "sim_mat_0",
        "sim_mat_1",
        "sim_mat_2",
        "sim_cube_0",
        "sim_cube_1",
        "sim_cube_2",
        "sim_cube_3",
    ):
        assert required in sst_bodies, f"SST model is missing pool body {required}"


def test_sst_adds_furniture_absent_from_base(base_model, sst_model):
    base_geoms = _names(base_model, mujoco.mjtObj.mjOBJ_GEOM)
    sst_geoms = _names(sst_model, mujoco.mjtObj.mjOBJ_GEOM)
    for furniture in SST_FURNITURE_GEOMS:
        assert furniture in sst_geoms, f"SST model is missing furniture {furniture}"
        assert (
            furniture not in base_geoms
        ), f"Base model must not have SST furniture {furniture}"


def test_chairs_reach_into_the_lidar_plane_and_collide(sst_model):
    """Chair legs must be seen by the 2D lidar (~0.13 m) and be solid."""
    lidar_z = 0.13
    for chair in ("chair_1", "chair_2"):
        for leg in ("leg_fl", "leg_fr", "leg_bl", "leg_br"):
            geom_name = f"{chair}_{leg}"
            geom_id = mujoco.mj_name2id(
                sst_model, mujoco.mjtObj.mjOBJ_GEOM, geom_name
            )
            assert geom_id >= 0, f"missing {geom_name}"
            # Group 0 and non-visual means mj_ray (lidar) sees it.
            assert sst_model.geom_group[geom_id] == 0, f"{geom_name} not in lidar group"
            assert sst_model.geom_contype[geom_id] != 0, f"{geom_name} does not collide"
            # The leg straddles the lidar plane: body z + geom top >= lidar_z.
            body_id = sst_model.geom_bodyid[geom_id]
            top_z = (
                sst_model.body_pos[body_id][2]
                + sst_model.geom_pos[geom_id][2]
                + sst_model.geom_size[geom_id][2]
            )
            assert top_z >= lidar_z, f"{geom_name} top {top_z:.3f} below lidar plane"


def test_floor_markings_stay_below_the_lidar_and_do_not_collide(sst_model):
    for marking in ("door_marker", "water_bowl", "location_1", "location_2"):
        geom_id = mujoco.mj_name2id(sst_model, mujoco.mjtObj.mjOBJ_GEOM, marking)
        assert geom_id >= 0, f"missing {marking}"
        assert sst_model.geom_contype[geom_id] == 0, f"{marking} should not collide"
        top_z = (
            sst_model.geom_pos[geom_id][2] + sst_model.geom_size[geom_id][2]
        )
        assert top_z < 0.05, f"{marking} top {top_z:.3f} reaches the lidar band"
