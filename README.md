# Juniper

## What is Juniper?

Juniper is a collection of utilities for processing and visualizing massive point cloud datasets using [OpenSceneGraph](http://www.openscenegraph.org).

[![IMAGE ](https://img.youtube.com/vi/lUeF4Y8yGNI/0.jpg)](https://www.youtube.com/watch?v=lUeF4Y8yGNI)

This video is a great example of what Juniper can do.  The datasets in that video is around 5 billion points, which is obviously too big to be stored in memory.  Using Juniper's command line utilities we can take the input files, sort them into an optimized structure for streaming and then we can visualize them at runtime, only streaming in points as needed.

## QuickStart

Tiling a dataset

```
mkdir tiled
cd tiled
osgjuniper_split /path/to/points.laz
osgjuniper_downsample tileset.lastile
```

Visualizing a tiled pointcloud
```
osgviewer tileset.lastile
```


* [Building Juniper from source](docs/building.md)
* [Command line utilities](docs/utilities.md)









