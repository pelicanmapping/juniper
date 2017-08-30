# Utilities

This document describes the usage of the command line utilities in Juniper.

## osgjuniper_split
```
Usage: osgjuniper_split [options] file.laz [file2.laz ...]
Options:
  --dest            The destination srs
  --directory       Loads a directory of laz or las files
  --driver drivername
                    The driver to use for output (filesystem, directory,
                    rocksdb)
  --filter level x y z
                    The octree cell to filter the input data to.  Used in
                    multiprocess builds.
  --geocentric      Generates a geocentric output
  --level           The initial split level.  Default is automatically computed
                    from the source data
  --out             The path to write tiles to
  --src             The source srs
  --target          The target number of points for a leaf node octree cell.
                    Tiles with more points than target will be further refined
  -h or --help      Display command line parameters
```


## osgjuniper_downsample
```
Usage: osgjuniper_downsample [options] tileset.lastile
Options:
  --innerLevel level
                    The octree level to use for the internal box filter for each
                    downsampled cell
  --threads numThreads
                    The number of threads to use for downsampling
```

## osgjuniper_tile
```
Usage: osgjuniper_tile [options] file.laz [file2.laz ...]
Options:
  --dest            The destination srs
  --directory       Loads a directory of laz or las files
  --geocentric      Generates a geocentric output
  --innerLevel level
                    The octree level to use for the internal box filter for each
                    downsampled cell
  --maxLevel maxLevel
                    The maximum level of subdivision for the tileset
  --src             The source srs
  --target          The target number of points in a cell
  --threads numThreads
                    The number of threads to use
  -h or --help      Display command line parameters
```
