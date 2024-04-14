# odin-pathgrid
An implementation of an A* 2D pathfinding grid in Odin. Designed for minimal allocation.

The design of this implementation is heavily inspired by the Godot engine's AStarGrid2D code. If you're familiar with it at all, using this will be a piece of cake!

## Example Usage
```odin
package main

import "core:fmt"
import pg "./include/odin_pathgrid" // Or wherever you keep the package :P

main :: proc() {
	astar: pg.AStar_Grid
	pg.astar_grid_init(&astar)
	defer pg.astar_grid_destroy(&astar)

	// Set the region to 12x12 for example. (Using a min and max vector allows it to support offset coordinates.)
	astar.region.min = {0, 0}
	astar.region.max = {12, 12}

	// `astar_grid_clear` clears out any and all blocked/added-cost points in the grid.
	pg.astar_grid_clear(&astar)

	// `astar_block` sets points on the grid as impassable.
    pg.astar_block(&astar, {0,1})
    pg.astar_block(&astar, {2,1})
	// `astar_set_cost` can be used to add an additional cost value to a point, making it a less desirable point to visit.
    pg.astar_set_cost(&astar, {2,0}, 3.)
    pg.astar_set_cost(&astar, {1,1}, 1.)
    pg.astar_set_cost(&astar, {3,1}, 100.)

	// `astar_get_path` gets you the set of points that A* calculates as the best path from your start point to your end point.
	// The result of `get_path` is just an array of `[2]i32`s. The second return value will return false if no path could be found.
	//
	// NOTE: By default, `get_path` allocates memory using `context.allocator`. You can pass your own allocator as an argument to change this.
	// This allocator is only used for the output slice, and not for anything internal to the pathfinding algorithm.
	path, ok := pg.astar_get_path(&astar, {0,0}, {11,5})
    defer delete(path)}

	
	if ok {
		for p,i in path {
			fmt.printf("{}: {}", i, p)
		}
	}
}

```
