# Trajectory input

Each non-comment CSV row contains:

```text
x,y,z
```

With `trajectory_relative_to_start: true`, coordinates use the robot pose at
the moment trajectory mode starts:

- `x`: forward in metres
- `y`: left in metres
- `z`: vertical offset from the initial B2 base in metres

The follower anchors the complete path in the MuJoCo world, searches forward
for the closest segment, and uses separate look-ahead targets for the base and
end effector. Duplicate adjacent XY points are ignored.

For future vision integration, either transform camera points into this start
frame before writing the CSV, or set `trajectory_relative_to_start: false` and
provide MuJoCo world-frame XYZ points directly.
