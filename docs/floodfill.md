# Flood-Fill Maze Solving

Mansoyanno uses flood-fill path planning for autonomous maze exploration.

## Basic Idea

Every maze cell is assigned a distance value representing the estimated number of moves required to reach the current target.

The target is initially the 2×2 center goal.

The robot then examines its neighboring cells and selects an accessible cell with a lower flood-fill distance.

```text
        ┌───┬───┬───┐
        │ 4 │ 3 │ 4 │
        ├───┼───┼───┤
        │ 3 │ G │ 3 │
        ├───┼───┼───┤
        │ 4 │ 3 │ 4 │
        └───┴───┴───┘
```

As the robot discovers walls, its internal maze representation is updated and the flood-fill distances are recalculated.

## Wall Mapping

The four IR sensors determine whether walls are present around the current cell.

The maze stores wall information for each direction.

When a wall is detected, the corresponding neighboring cell is also updated so that the shared wall remains consistent.

## Search Phase

The robot starts at `(0,0)` and initially faces North.

It repeatedly:

1. Reads the current walls.
2. Updates the maze.
3. Calculates flood-fill distances.
4. Selects the best valid direction.
5. Turns if necessary.
6. Moves one cell.
7. Updates its coordinates.
8. Repeats.

## Goal Detection

The center of the maze is represented as a 2×2 goal region.

The search phase ends when the robot enters any cell inside the goal region.

## Return Phase

Instead of stopping at the goal, the robot changes the flood-fill target to `(0,0)`.

It then calculates a new route back to the start using the maze information accumulated during exploration.

This is important because the return route does not have to be identical to the route used to reach the goal.

## Fast Path

After returning to the starting cell, the robot generates a fast path from the accumulated maze information.

The path is stored in EEPROM so that it can be reused after power cycling.
