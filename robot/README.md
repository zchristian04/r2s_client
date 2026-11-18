# Robot model

The robot in the sim-to-sim exam: a Flexiv Grav (GN01) parallel gripper on a
floating stage with x, y, z, roll, pitch and yaw. The stage stands in for the
Rizon 4s arm. The GN01 geometry comes from the lab model; the stage, controller,
masses of the stage links and force limits are simulation settings, not the
Flexiv controller.

- `gn01_stage.usda`: the exact USD articulation the source world uses (PhysX
  joints, drives, mimic joints, colliders, masses, wrist camera), at the home
  pose. It contains no physics scene, table or lights; add your own.
- `gn01_stage.xml`: a MuJoCo model with the same kinematics, masses, colliders,
  gains and wrist camera, on a bare table plane. It loads as is
  (`mujoco.MjModel.from_xml_path("gn01_stage.xml")`). Its link poses match the
  USD model to 1e-15 m at any pose and width. Add your own scene to it.
- `gn01/`: the gripper meshes both use.

The two files describe the same robot, but the engines differ (contact model,
solver, joint constraints); do not assume either reproduces the source world's
contacts exactly.

## Frames and units

SI units, right-handed, world z up, table top at z = 0.

- Orientation rpy = [roll, pitch, yaw], tool rotation R = Rz(yaw) Ry(pitch)
  Rx(roll) in the world frame. |roll|, |pitch| <= pi/2; yaw in [-pi, pi] in
  commands (the joint allows ±3.2).
- Joint chain: x → y → z → yaw → pitch → roll → tool. The x, y, z joint values
  are the TCP position, and the three rotations turn about the TCP, so tilting
  never moves the fingertips' closed point.
- Tool frame (= F/T sensor frame, site `s2s_ft`): the flange, 0.32212 m from the
  TCP along tool +z. At rpy = 0 tool z points up, the fingers point along
  tool −z (straight down) and open along tool ±y.
- TCP (site `s2s_tcp`): the bottom of the closed fingertips. Open fingertips sit
  higher (about 16 mm at full opening) because the GN01 fingers move on a
  linkage arc.
- Width: the GN01 width coordinate, 0 to 0.100 m; it matches the gap between the
  fingertip pads to within about 1 mm. Knuckle angle = −0.155 + 9.404 · width
  (rad); every other knuckle copies it (finger mounts mirror it).
- Wrist camera `wrist`: on the tool, position (−0.065, 0, −0.155) m in the tool
  frame, looking at (0.02, 0, −0.36), vertical field of view 70°. It moves and
  tilts with the gripper. /v1/calibration gives its pose relative to the flange.

## Masses, collisions, friction

- Gripper below the sensor: 0.900 kg (Flexiv spec), COM about 0.183 m below
  the flange along the tool axis. Per-link values are in both model files.
- Gravity does not act on the robot links in the source world (as if fully
  gravity-compensated; MuJoCo `gravcomp="1"`), so the drives never sag.
- The whole gripper collides with the scene: fingertip pad boxes, the gripper
  base, the bars and the finger mounts (convex hulls) and the flange and adapter
  cylinders. The gripper does not collide with itself.
- Friction coefficient 1.0 on the pads and 0.5 on the rest of the gripper;
  contacts take the larger of the two surfaces' coefficients.

## Controller

Each command (parameters in the prompt, Section 5) is planned as straight-line
segments of TCP position, orientation and width.

- Every segment follows a minimum-jerk profile s(u) = 10u³ − 15u⁴ + 6u⁵ over
  T = max(1.875·|Δp|/v, 1.875·max|Δrpy|/1.5, 1.875·|Δwidth|/0.05, 0.02 s),
  with v the segment speed (free travel 0.10 m/s). Position moves in a straight
  line; roll, pitch and yaw each move in joint space on the same profile.
- Drives: position servos on x, y, z (kp 4000 N/m, damping 150 N s/m) and on
  roll, pitch, yaw (kp 60 N m/rad, damping 3 N m s/rad); a width servo (kp 5000
  N/m, kv 50 N s/m) capped at 25 N, which is the grip force. `stiffness` and
  `rot_stiffness` replace the kp values for compliant segments; damping stays.
  After a compliant command, the controller holds the measured pose for 0.2 s
  before restoring full stiffness.
- Force/torque `wrench` = [Fx Fy Fz Tx Ty Tz] in the tool frame about the
  flange: the total force the environment applies to the gripper (contacts) plus
  the gripper's own weight, minus its inertial force while it accelerates, as a
  real wrist sensor sees it. It is tared at the start of every action, from the
  last 0.05 s of a 0.1 s rest. Tilting during an action therefore shows up as a
  change of the weight between the axes. In `gn01_stage.xml`, the untared value
  is minus the `s2s_ft_force` / `s2s_ft_torque` sensors. Readings are noisy and
  keep a small per-action bias.
- stop_force: a segment with a stop force ends when |F| exceeds it; the robot
  then holds where it actually is (not where it was commanded) and the remaining
  waypoints are skipped. A touch then backs off 0.02 m from where it stopped.
- Protective stop: when |F| averaged over 5 ms exceeds 30 N, the action stops
  and the robot holds its measured pose for 0.2 s.
- Workspace (TCP): x, y in [−0.35, 0.35] m, z in [0.002, 0.40] m; guarded
  moves (with a stop force) may aim down to z = −0.02 m. Home: TCP (−0.25, 0,
  0.25), rpy 0, fingers closed.

## Shortcuts, exactly

Each starts by rising straight up to the travel height z = 0.38 m (if below
it), then travels above its start point while turning to its orientation.

- move_to: one segment to the target pose.
- home: to home at the travel height, then down to the home pose, closed.
- push: descend to 0.03 m before contact_point (closed), push to
  contact_point + distance · direction at `speed`, back off 0.02 m at `speed`,
  rise. Default orientation: yaw facing the push direction.
- touch: descend to `point`, move along `direction` for up to max_travel at
  `speed` until |F| > stop_force, back off 0.02 m from where it stopped, rise.
- grasp_lift: open to open_width at the travel height above the approach
  start, descend to 0.06 m back from `center` along the finger direction,
  approach `center` at 0.03 m/s, close (hold 0.3 s, then wait up to 1 s until
  the width changes by less than 0.2 mm over 0.1 s), lift by `lift` at
  0.03 m/s and hold `hold` s, lower at 0.03 m/s towards 0.015 m below `center`
  until the force changes by 3 N from the held load (contact), open (hold
  0.2 s), withdraw to the approach start at 0.03 m/s, rise, close.
