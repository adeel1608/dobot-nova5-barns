# `computer_vision.py` flowcharts

- Sequence/exported functions: 1
- Support/helper functions: 0

## Sequence/exported functions

### `detect_cup_gripper`

- **Mermaid file:** [../mermaid/computer_vision/detect_cup_gripper.mmd](../mermaid/computer_vision/detect_cup_gripper.mmd)
- **Parameter scenarios observed:** none directly read
- **Branch/decision scenarios:**
  - `not rclpy.ok()`
  - `node.latest_depth_image is None`
  - `not node.depth_info_received`
  - `node.latest_depth_image is None`
  - `not isinstance(depth_image, np.ndarray)`
  - `len(depth_data_raw.shape) > 2`
  - `total_roi_pixels > 0`
  - `depth_data_filtered[center_y, center_x] > INVALID_DEPTH_THRESHOLD_MM`
  - `current_time - last_print_time >= PRINT_INTERVAL`
  - `percentage_invalid >= CUP_DETECTION_INVALID_PERCENTAGE_THRESHOLD`

```mermaid
flowchart TD
  N0(["START detect_cup_gripper(**params)"])
  N1["Call: _trace_step('detect_cup_gripper', 'START')"]
  N2{"IF not rclpy.ok()?"}
  N3["State/cache: cup_detected = False"]
  N4["Call: try: # Wait for first depth image and camera info print('(CV) Waiting for depth camera topics...') while (node...."]
  N5(["END detect_cup_gripper"])
  N0 --> N1
  N1 --> N2
  N2 --> N3
  N2 --> N3
  N3 --> N4
  N4 --> N5
```
