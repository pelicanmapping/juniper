# Tiling Point Clouds

This document describes the usage of the command line utilities in Juniper and the algorithms behind them. 

At it's core, Juniper's tiler utilities take a massive number of points as input and sorts them into an octree structure that can be streamed in efficiently for real time rendering.  There are two differnet tiling algorithms implemented in Juniper:  split/downsample and tile.

## Split/Downsample
The Split/Downsample algorithm takes a multistep approach to tiling point clouds in order to make it more parallel and also allow it to run across machines is desired (using the --filter option and shared storage).  The first step in this algorithm is splitting.  To split a point cloud dataset you can run:
```
mkdir tiled
cd tiled
osgjuniper_split /path/to/file1.laz /path/to/file2.laz ...
```

osgjuniper_split will look at the metadata of the file and compute an estimated max split level.  You can also specify a --split argument to manually choose the split level.  Juniper then reads all of the points in the input files, determines which cell at the split level the point belongs in and writes it to that cell.  While it's doing this writing, Juniper is also keeping track of how many points actually end up in a cell.

These cells that have more than the specified target number of points will go through a process called refinement where they will be further split until their leaf nodes contain less than the target number of points.  Because we performed the initial splitting of points in a single threaded, we can refine each cell in parallel b/c each cell doesn't requirement state or input from any other cell.

Once refinement is complete, you're left with all leaf nodes of an octree.  To produce the lower levels of the octree we use the osgjuniper_downsample command on the output tileset.lastile file.

```
osgjuniper_downsample tileset.lastile
```
osgjuniper_downsample uses the tileset.lastile file to get metadata information about the tileset and builds lower lods from the leaf level up.  The cells in each level of the octree can be built completely in parallel because their input is the parent cells children.  For example, to build a level 6 cell, the osgjuniper_downsample application will load all of the potential 8 children of the level 6 cell and select a subsample of points to include as the level 6 representation.  No other level 6 cells use those children as input, so it's safe to run everything parallel.  Once level 6 is built, then level 5 can be built using the level 6 cells as input.

## Tile

Tile is the original Juniper tiling algorithm and is similar to the split/downsample algorithm except that it works from the top down instead of the bottom up.  It can still run in parallel, but only starts becoming highly parallelized once many of the top level tiles are built.

For each cell in the octree, starting at the top, osgjuniper_tile will read all of the points from the input files, and it will keep a portion of the them that fits it's "inner octree" level to produce a reasonable lower lod of what the overall point cloud looks like.  This is basically a box filter that is applied to the points.  Any points that aren't accepted in this cell are written to a temporary file on disk that corresponds to one of the 8 children of this cell.  Once the cell is complete, it's 8 children are built in the same manner.  Once the original input is split into it's children, then each child cell can be built in parallel.  

This algorithm produces very regularly looking datasets, but it can be much slower for large datasets than the split/downsample algorithm due to the fact that it has to spend so much time reading data at each cell.  For example, lod 0 might have to read 1 billion points just to create a tile that contains 50K points, leaving lod 1 to have to read 999980000 points as input.  For this reason it is recommend to use the split/downsample algorithm.


# Application Usage

### osgjuniper_split
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

### osgjuniper_downsample
```
Usage: osgjuniper_downsample [options] tileset.lastile
Options:
  --innerLevel level
                    The octree level to use for the internal box filter for each
                    downsampled cell
  --threads numThreads
                    The number of threads to use for downsampling
```

### osgjuniper_tile
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
