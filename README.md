# map_origin_broadcaster

Maintains the origin of the `map` frame and statically broadcasts that origin.
The program aims to achieve global consistency of the map in the GPS frame and
was tested using
[slam_gmapping](https://github.com/ros-perception/slam_gmapping).

## Caveats

This node simply listens to TF transformations between `gps_origin` and
`gps_base_link`. This means that there needs to be a node that broadcasts the
transformations between the GPS origin and the GPS mobile base (for example,
[gnss_broadcaster](https://github.com/tado-aev/gnss_broadcaster)).
Additionally, the GPS mobile base location needs to be accurate (e.g. RTK fix,
FloatRTK fix etc.) because only one coordinate location is taken (without
taking the average of N samples etc.).

Once it receives a transformation, it publishes that as a static
transformation between `gps_origin` and `map`. On doing so, the difference
between `odom` and `base_link` (i.e. the
potentially-inaccurate-but-the-only-best-guess distance traveled since the
odometry node was brought up) is subtracted because the `map` is set to the
origin of the `odom` frame upon
[creation](https://github.com/ros-perception/slam_gmapping/blob/ef88b991132ab71897fe3d866e450ad9f6bf40ba/gmapping/src/slam_gmapping.cpp#L130),
meaning that the "map origin" is equal to the current `odom` origin.

## Parameters

In the parentheses are the default values for each parameter.

- `gps_origin_frame` (`gps_origin`): The frame attached to the GPS origin.
- `gps_base_frame` (`gps_base_link`): The frame attached to the base frame, calculated using GNSS instruments.
- `map_frame` (`map`): The frame attached to the map.
- `odom_frame` (`odom`): The frame attached to the odometry system.
- `base_frame` (`base_link`): The frame attached to the mobile base.
- `save_on_exit` (`true`): Whether to save the location of the map origin to a file on exiting.

## Saved file

When the `save_on_exit` parameter is set to true, the program saves the
location of the map origin on exit. The file is a CSV file with the file name
being the date and time when the coordinate information received. The file
contains the following information in the respective order:

1. Date and time when the coordinate information was received
2. Frame name of the GPS origin
3. Frame name of the map origin
4. Translation in the X direction
5. Translation in the Y direction
6. Translation in the Z direction
7. Rotation quaternion X
8. Rotation quaternion Y
9. Rotation quaternion Z
10. Rotation quaternion W

When the node is launched using the launch file, the file is saved in
`~/.ros`. When invoked with `rosrun`, it should save the file to the current
directory (i.e. wherever the `rosrun` was invoked).

## License

MIT

## Author

Naoki Mizuno (mizuno.naoki@rm.is.tohoku.ac.jp)
