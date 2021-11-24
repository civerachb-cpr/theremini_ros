theremini_ros
==============

This repository is a simple demo that uses MoveIt! to control a dual-armed robot to play a Moog Theremini.


Usage
------

Add the Theremini to your URDF.  The parent can be a fixed link on the robot (e.g. `base_link` or `base_footprint`)
if the instrument is being manually positioned relative to the robot, or it can be a link in the world frame
relative to `odom` or `map` if the robot will localize itself with the instrument.

```xml
<xacro:include filename="$(find theremini_description)/urdf/theremini.urdf.xacro" />
<xacro:theremini parent="base_footprint" prefix="theremin">
  <origin xyz="1 0 0" rpy="0 0 0" />
</xacro:theremini>
```

The robot itself will require two arms, configured with MoveIt! move groups.  The default names for these groups are
`left_manipulator` and `right_manipulator`, but these can be modified by editing `theremini_player/config/theremin.yaml`

Parameters
-----------

All necessary parameters should be defined within the `theremin` namespace.  `theremin` must contain configurations
for `pitch_control` and `volume_control`, as shown below:

```yaml
theremin:
  pitch_control:
    move_group: right_manipulator
    antenna_frame: theremin_pitch_antenna
    min_distance: 0.05
    max_distance: 0.5
    control_axis: x
    control_mode: linear
    x_offset: 0.0
    y_offset: 0.05
    z_offset: 0.15
    pitch_offset: 0.0
    roll_offset: 0.0
    yaw_offset: 0.0

  volume_control:
    move_group: left_manipulator
    antenna_frame: theremin_volume_antenna
    min_distance: 0.05
    max_distance: 0.5
    control_axis: z
    control_mode: linear
    x_offset: -0.03
    y_offset: 0.02
    z_offset: 0.0
    pitch_offset: 0.0
    roll_offset: 0.0
    yaw_offset: 0.0
```

**Antenna Control Parameters**

- `move_group` the name of the MoveIt! move-group that governs the arm that interacts with the given antenna
- `antenna_frame` the name of the frame for the antenna on the Theremin URDF
- `min_distance` the minimum distance between the end-effector link and the antenna
- `max_distance` the maximum distance between the end-effector link and the antenna
- `control_axis` one of `x`, `y`, or `z` to define the axis along with the end-effector moves relative to the antenna
  the pitch antenna should normally move vertically and the pitch antenna should be horizontally
- `control_mode` unused, but reserved for future expansion to handle interpolation between desired notes
- `{x|y|z}_offset` a fixed offset along each axis relative to the `antenna_frame`
- `{roll|pitch|yaw}_offset` a fixed offset around each axis relative to the `antenna_frame`
