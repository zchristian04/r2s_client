# Robot model

The robot in every seed: a Flexiv Grav (GN01) parallel gripper on a floating
Cartesian stage with yaw (x, y, z, yaw). The stage stands in for the Rizon 4s arm.

This is a simulated GN01: its geometry comes from the lab model; the stage,
controller and force limits are simulation settings, not the Flexiv Grav
controller.

- `gn01_stage.xml`: MuJoCo model of the stage, gripper, wrist camera and wrist
  force/torque sensor, on a bare table plane. It loads as is
  (`mujoco.MjModel.from_xml_path("gn01_stage.xml")`). Add your own scene to it.
- `gn01/`: the gripper meshes it uses.

## Frames and units

SI units, right-handed, world z up, table top at z = 0.

- Tool frame: on the stage, rotated by yaw about world z, z up. The fingers open
  along tool ±y.
- Flange: the tool frame origin, where the F/T sensor sits (site `r2s_ft`).
- TCP (site `r2s_tcp`): the bottom of the closed fingertips, 0.32212 m below the
  flange. Open fingertips sit higher (about 16 mm at full opening), because the
  GN01 fingers move on a linkage arc.
- Width: the GN01 width joint, 0 to 0.100 m, which matches the gap between the
  fingertips to within about 1 mm.

## Controller

Each primitive (parameters in the prompt, Section 5) is planned as straight-line
segments of TCP position, yaw and width. Every primitive first rises to the
travel height, moves above its start point at that height, does its work and
returns to the travel height.

- Every segment follows a minimum-jerk profile, s(u) = 10u³ − 15u⁴ + 6u⁵, over
  duration T = max(1.875·|Δp|/v, 1.875·|Δyaw|/1.5, 1.875·|Δwidth|/0.05, 0.02 s),
  where v is the segment speed (free travel 0.10 m/s).
- Travel height 0.20 m. A push starts 0.03 m before its contact point and backs
  off 0.02 m after it. A touch backs off 0.02 m from where it stopped.
- grasp_lift: open above the centre, descend, close (hold 0.3 s), lift and lower
  at 0.03 m/s (hold at the top for `hold`), open (hold 0.2 s), rise, close.
- Drives: position servos on x, y, z (kp 4000) and yaw (kp 60) with joint damping,
  and a width servo (kp 5000, kv 50) whose force is capped at 25 N, which is the
  grip force.
- Force/torque: the wrench the environment applies to the tool, in the tool
  frame at the flange, `[Fx Fy Fz Tx Ty Tz]`. It is tared in free space during
  0.1 s at the start of every action. Readings are noisy and may keep a small
  residual bias.
- Protective stop: if |F| exceeds 30 N, the action stops and holds for 0.2 s.
  A touch stops when |F| exceeds its `stop_force`.
- Workspace (TCP): x, y in [−0.35, 0.35] m, z in [0.002, 0.40] m. Guarded touches
  may aim down to z = −0.02 m. Home: TCP (−0.25, 0, 0.25), yaw 0, fingers closed.
