# ArUco perception fix for K8s (color/depth resolution mismatch)

## Problem
On host, color and depth may be aligned or timing is such that depth at marker corners is valid. In K8s, depth image resolution (848x480) differs from color (1280x720), so color (u,v) used to index depth often goes out of bounds or hits wrong pixels → no valid depth → TF never published.

## Solution
Map color-image coordinates to depth-image coordinates before sampling depth, so it works when resolutions differ (K8s) and unchanged when they match (host).

## File to edit
`services/robot_container/ros_ws/src/pickn_place/pickn_place/aruco_perception.py`

## Change

**Find this block (around lines 274-290):**

```python
                cam_pts = []
                valid = True
                for (uf, vf) in corners[i][0]:
                    u, v = int(round(uf)), int(round(vf))
                    if not (0 <= u < self.depth_width and 0 <= v < self.depth_height):
                        valid = False; break
                    d = float(self.latest_depth_image[v, u]) / 1000.0
                    if d <= 0:
                        valid = False; break
                    fx, fy = self.depth_camera_matrix[0,0], self.depth_camera_matrix[1,1]
                    cx, cy = self.depth_camera_matrix[0,2], self.depth_camera_matrix[1,2]
                    X = (u - cx) * d / fx
                    Y = (v - cy) * d / fy
                    cam_pts.append([X, Y, d])
                if not valid or len(cam_pts) < 4:
                    continue
```

**Replace with:**

```python
                cam_pts = []
                valid = True
                # Map color coords to depth coords when resolutions differ (e.g. K8s: color 1280x720, depth 848x480)
                scale_u = self.depth_width / float(self.image_width) if self.image_width else 1.0
                scale_v = self.depth_height / float(self.image_height) if self.image_height else 1.0
                for (uf, vf) in corners[i][0]:
                    u, v = int(round(uf)), int(round(vf))
                    u_d = int(round(uf * scale_u))
                    v_d = int(round(vf * scale_v))
                    if not (0 <= u_d < self.depth_width and 0 <= v_d < self.depth_height):
                        valid = False; break
                    d = float(self.latest_depth_image[v_d, u_d]) / 1000.0
                    if d <= 0:
                        valid = False; break
                    fx, fy = self.depth_camera_matrix[0,0], self.depth_camera_matrix[1,1]
                    cx, cy = self.depth_camera_matrix[0,2], self.depth_camera_matrix[1,2]
                    X = (u_d - cx) * d / fx
                    Y = (v_d - cy) * d / fy
                    cam_pts.append([X, Y, d])
                if not valid or len(cam_pts) < 4:
                    continue
```

Summary:
- Add scale_u, scale_v to map color (u,v) to depth pixel (u_d, v_d).
- Use (u_d, v_d) for bounds check and for reading depth.
- Use (u_d, v_d) and depth intrinsics for X,Y (depth is in depth camera frame).

## Optional: fix label depth (avoids IndexError when depth res < color res)

**Find (around lines 352-356):**
```python
                c = pts2d.mean(axis=0).astype(int)
                dv = self.latest_depth_image[c[1], c[0]]
                label = f"{self.marker_name_mapping.get(mid, mid)} {dv:.1f}mm"
```

**Replace with:**
```python
                c = pts2d.mean(axis=0).astype(int)
                c_d_u = int(round(c[0] * scale_u))
                c_d_v = int(round(c[1] * scale_v))
                if 0 <= c_d_u < self.depth_width and 0 <= c_d_v < self.depth_height:
                    dv = self.latest_depth_image[c_d_v, c_d_u]
                else:
                    dv = 0
                label = f"{self.marker_name_mapping.get(mid, mid)} {dv:.1f}mm"
```

## Apply on Linux
```bash
cd ~/BARNS
# Edit the file
nano services/robot_container/ros_ws/src/pickn_place/pickn_place/aruco_perception.py
# Then rebuild image (with cache is enough)
cd services/robot-arm-k8s
sudo docker build -f Dockerfile.robot1 -t barns-robot1:latest ../..
# Push and rollout
sudo docker tag barns-robot1:latest me-central2-docker.pkg.dev/qss-development-project/barns/barns-robot1:latest
sudo docker push me-central2-docker.pkg.dev/qss-development-project/barns/barns-robot1:latest
kubectl rollout restart deployment/robot1 -n barns
```
