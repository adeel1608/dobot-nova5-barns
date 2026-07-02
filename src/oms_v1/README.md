# oms_v1 Robot Sequence Package

`oms_v1` contains the BARNS robot sequence layer used by the robot container. It wraps low-level Dobot/ROS motion calls and exposes higher-level coffee automation sequences.

## Important files

- `oms_v1/manipulate_node.py` - low-level robot motion/service wrapper and `run_skill` dispatcher.
- `oms_v1/params.py` - shared constants, gripper settings, cup parsing helpers, and taught robot poses.
- `oms_v1/sequences/espresso.py` - espresso portafilter/grinder/tamper/pitcher flows.
- `oms_v1/sequences/paper_cups.py` - paper cup pickup/place/serve flows.
- `oms_v1/sequences/plastic_cups.py` - plastic cup dispense/stage/ice/sauces/milk flows.
- `oms_v1/sequences/milk_frothing.py` - frother pickup/mount/swirl/pour/clean/return flows.
- `oms_v1/sequences/cleaning.py` - hard/soft brush portafilter cleaning flows.
- `oms_v1/sequences/slush.py` - slush dispense/place flows.
- `oms_v1/sequences/home.py` - home positioning, calibration helpers, and robot reset helpers.
- `oms_v1/sequences/computer_vision.py` - cup detection from depth camera topics.
- `oms_v1/gui/` - Streamlit/teaching/debug GUI helpers.

## Build/run

```bash
cd services/robot_container/ros_ws
source /opt/ros/<distro>/setup.bash
colcon build --packages-select oms_v1
source install/setup.bash
```

Run commands depend on the active robot container launch setup. Use the container compose/launch files and confirm Dobot services are available before running physical motion.

## Logging and trace style

Sequence modules use lightweight trace helpers similar to:

```python
_trace_step(scope, message)
run_skill(...)
_fail(reason="...")
```

Use these for non-obvious movement branches, cache hit/miss decisions, retries, validation failures, and gripper verification. Avoid logging trivial assignments or high-frequency loops unless debugging a specific issue.

## Gripper verification

`manipulate_node.robot_motion.set_gripper_position(...)` supports optional `verify_position=True`. Default calls preserve the existing stable-read behavior. Use `expected_position=...` for object-contact grips where the final readback is expected to be lower than the commanded close value.

## Safety notes

- `params.py` should remain the source of truth for taught joint poses.
- Do not change robot motion order, ROS service names, topics, or calibration targets without a test plan.
- Cache invalidation functions are important after calibration/pose changes.
- Always confirm the robot is clear and emergency stop is accessible before running new sequence changes.

## TODOs

- Add dry-run sequence tests that monkey-patch `run_skill` and print the command order.
- Add a generated sequence catalog/flowchart bundle as part of release docs.
- Document the expected machine calibration order for production startup.