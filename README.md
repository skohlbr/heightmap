# heightmap

Computes a height map from OctoMap and PointCloud messages, and serves it through a ROS service.

## Building

The usual procedure for building ROS packages works for heightmap as
well. Just download the source tree in a catkin workspace and do:

```sh
	catkin_make
```

Please note that the performance of the node is very sensitive to
compiler optimizations. It's possible to observe a ~50% performance
increase when compiling with optimizations turned on. When they
aren't, the node may not run fast enough to function well as a
component of a robot's operating system. For this reason, it's
recommended to do a debug build *only* when there's any debugging to
do, and enable optimization in all other cases, by invoking the
compilation like this:

```sh
	catkin_make -DCMAKE_BUILD_TYPE=Release
```

## Running

You can run the node with `rosrun`, like any other ROS node.

The initial height map can be loaded from a file by append `load
<filename>` to the command line. The image file `filename` will be
loaded and written to the height map with the corner at the origin
(`0,0` coordinates). Please note that the image's pixels are mapped
1:1 to the elements of the heightmap: no resolution adaptation is
performed at all.
