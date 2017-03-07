This package creates and executes concentric or rectilinear sweeping patterns

Functions:
    -accept user input of polygon wall boundary (similar to frontier_explorer)
    -follow walls at a set distance (custom local/global planner? use of extra sensors?)
    -generate sweeping paths to conform to patterns
    -generate costmap (layer) to track swept/unswept space

Objectives:

    minimize sweeping time
    minimize computational load
    minimize "missed" space
    maximize use of existing packages (move_base, global/local planners, etc)
    minimize additional hardware required

Constraints:

    no crashing
    hardware
    computational
    latency
