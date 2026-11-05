"""
Strange Situation Test scenario files.

One test per episode plus a sweep that loads every `sst_*` file, following the
"shipped scenarios load" pattern in `test_sim_scenarios.py`. These guard the
things the episodes rely on: who the subject is (the owner, when present), who
walks off the arena (a leaving person's last waypoint), and the dog-alone
episodes carrying no actors at all.
"""

from pathlib import Path

from mecanumbot_sim.sim_scenarios import load_sim_scenario

SCENARIO_ROOT = Path(__file__).resolve().parents[1] / "config" / "sim_scenarios"

#: Anything with y below this is off the arena floor (south wall is at y = -2.0).
OUTSIDE_Y = -2.2


def sst(name: str) -> str:
    return str(SCENARIO_ROOT / name)


def test_all_sst_scenarios_load():
    sst_files = sorted(SCENARIO_ROOT.glob("sst_*.yaml"))
    assert sst_files, "no sst_* scenarios found"
    for path in sst_files:
        scenario = load_sim_scenario(str(path))
        assert scenario.name == path.stem


def test_episode_1_owner_is_the_subject_and_tours_the_room():
    scenario = load_sim_scenario(sst("sst_episode_1_owner_and_dog.yaml"))

    assert scenario.name == "sst_episode_1_owner_and_dog"
    # One human, no stranger, and the owner is the subject.
    humans = [a for a in scenario.actors if a.kind == "human"]
    assert len(humans) == 1
    owner = scenario.subject_actor
    assert owner is not None
    assert owner.body_name == "sim_human_0"
    assert owner.name == "owner"

    # The owner walks a route (Chair 1 -> Location 1 -> Location 2 -> ...): more
    # than one waypoint, patrol motion, and does not loop (a one-way tour).
    assert owner.motion_mode == "patrol"
    assert owner.motion_loop is False
    assert len(owner.motion_waypoints) >= 4
    # Every pose stays inside the arena - the owner never leaves in Ep1.
    assert all(w.y > OUTSIDE_Y for w in owner.motion_waypoints)

    # The building blocks are present as props on Location 1.
    cubes = [p for p in scenario.props if p.kind == "cube"]
    assert len(cubes) == 4
    assert all(p.x > 1.0 for p in cubes)


def test_episode_2_owner_is_subject_and_leaves_stranger_stays():
    scenario = load_sim_scenario(sst("sst_episode_2_owner_stranger_dog.yaml"))

    humans = [a for a in scenario.actors if a.kind == "human"]
    assert len(humans) == 2

    owner = scenario.subject_actor
    assert owner is not None
    assert owner.body_name == "sim_human_0"
    # The owner is the one who leaves: a final waypoint off the arena.
    assert owner.motion_waypoints[-1].y < OUTSIDE_Y

    stranger = next(a for a in scenario.actors if not a.is_subject)
    assert stranger.body_name == "sim_human_1"
    # The stranger stays: every waypoint is inside the room.
    assert all(w.y > OUTSIDE_Y for w in stranger.motion_waypoints)


def test_episode_3_has_no_subject_and_only_the_stranger():
    scenario = load_sim_scenario(sst("sst_episode_3_stranger_dog.yaml"))

    humans = [a for a in scenario.actors if a.kind == "human"]
    assert len(humans) == 1
    assert humans[0].body_name == "sim_human_1"
    # Owner absent -> no subject -> detection evaluator reports subject_missing.
    assert scenario.subject_actor is None
    # The stranger leaves at the end: a final waypoint off the arena.
    assert humans[0].motion_waypoints[-1].y < OUTSIDE_Y


def test_dog_alone_episodes_have_no_actors_but_keep_the_blocks():
    for name in ("sst_episode_4_dog_alone.yaml", "sst_episode_6_dog_alone.yaml"):
        scenario = load_sim_scenario(sst(name))
        assert scenario.actors == ()
        assert scenario.subject_actor is None
        cubes = [p for p in scenario.props if p.kind == "cube"]
        assert len(cubes) == 4


def test_episode_5_owner_returns_and_leaves_again():
    scenario = load_sim_scenario(sst("sst_episode_5_owner_dog.yaml"))

    owner = scenario.subject_actor
    assert owner is not None
    assert owner.body_name == "sim_human_0"
    # Re-enters from the door (starts off/near the south edge) ...
    assert owner.y < 0.0
    # ... visits Chair 1 (a waypoint up in the +y half) ...
    assert any(w.y > 0.3 for w in owner.motion_waypoints)
    # ... and leaves again at the end (final waypoint off the arena).
    assert owner.motion_waypoints[-1].y < OUTSIDE_Y


def test_episode_7_has_no_subject_and_stranger_leaves_with_the_dog():
    scenario = load_sim_scenario(sst("sst_episode_7_stranger_dog.yaml"))

    humans = [a for a in scenario.actors if a.kind == "human"]
    assert len(humans) == 1
    assert humans[0].body_name == "sim_human_1"
    # Owner absent -> no subject.
    assert scenario.subject_actor is None
    # The stranger leaves at the end (final waypoint off the arena).
    assert humans[0].motion_waypoints[-1].y < OUTSIDE_Y


def test_every_episode_has_the_toy_ball_in_the_toys_area():
    for path in sorted(SCENARIO_ROOT.glob("sst_episode_*.yaml")):
        scenario = load_sim_scenario(str(path))
        balls = [p for p in scenario.props if p.body_name == "sim_ball_0"]
        assert len(balls) == 1, f"{path.stem} should have exactly one toy ball"
        ball = balls[0]
        assert ball.kind == "ball"
        # The Toys area is top-centre, well up in the +y half between the chairs.
        assert ball.y > 1.0
        assert abs(ball.x) < 0.6


def test_all_seven_episodes_are_present():
    names = {p.stem for p in SCENARIO_ROOT.glob("sst_episode_*.yaml")}
    assert names == {
        "sst_episode_1_owner_and_dog",
        "sst_episode_2_owner_stranger_dog",
        "sst_episode_3_stranger_dog",
        "sst_episode_4_dog_alone",
        "sst_episode_5_owner_dog",
        "sst_episode_6_dog_alone",
        "sst_episode_7_stranger_dog",
    }
