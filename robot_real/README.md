# Robot model: Flexiv Rizon 4s + Grav (GN01)

`rizon4s_gn01.xml` (MuJoCo) is the real robot in the API world frame. Meshes are in
`assets/`; the `part_*.xml` files are included by it. Source: the lab's
newton_handoff_20260921 model, without its table, camera and keyframe.

## Frames

- **World (API) frame:** z up, table top is the plane z = 0, origin directly below the
  robot base. x points away from the robot towards the camera, y to the robot's left.
- **Robot base:** body `base`, at (0, 0, 0.0134): the base sits 13.4 mm above the table top.
- **TCP (`tcp` in the API):** site `gn01_grasp_center_site`, the fingertip point between
  the fingers, 0.198 m from the flange along the tool axis.
- **Tool orientation:** always pointing straight down. At yaw 0 the tool x axis points
  along world -x and the fingers open along world y; yaw turns it about world z.
  Tool rotation = Rz(yaw) * diag(-1, 1, -1), so tool z points down.
- **Flange:** 0.198 m above the TCP along the tool axis.

## Wrench

`/v1/execute` returns `wrench` = [Fx Fy Fz Tx Ty Tz]: the external force and torque on the
tool, about the flange, in tool axes (as above), tared over 0.1 s at the start of each
action. It is estimated by the arm from its joint torques; there is no separate
force/torque sensor. Expect noise of order 1 N, more while the arm accelerates.

## How the server moves the arm

Every primitive is a sequence of straight TCP segments in the world frame. Each segment
follows a minimum-jerk time profile (peak speed = the segment's speed limit), streamed at
100 Hz as TCP targets to the arm controller: inverse kinematics with collision avoidance
(off during contact segments), then joint impedance control on the robot at 1 kHz. The
TCP speed is capped at 0.03 m/s. Tracking is within a few millimetres at rest; during
motion the arm may lag the commanded path slightly. `tcp_cmd` and `tcp_meas` in each
response show both.

Gripper: width 0-0.100 m, speed 0.05 m/s, closing force capped at 25 N. Width changes
happen before the arm moves on.

## Joints

`joint1` ... `joint7` are the arm; `gn01_finger_width_joint` is the gripper opening
(the six linkage joints follow it through equality constraints).
