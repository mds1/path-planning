# Path Planning

Hierarchical D* Lite (HD*) is a real-time path planning algorithm for use in unknown environments. It first plans a coarse path, then refines it to improve path quality.

## How to Use :  

To run HD\*, run main_hdstar.py. Run main_dstar.py if you want to use normal D\* Lite (this is good for finding the optimal path on a given map). Once either of those functions are run, it imports config_user.py, config_program.py, and all_functions.py

In config_user.py, you can modify the settings that affect oepration. Each variable is explained below.
- `testingMode`: Suppresses figure generation, outputs from main_hdstar.py (or main_dstar.py) are not printed.
- `makeFigure`: When `True`, the final path and environment is displayed upon completion.
- `makeMovie`: When `True`, the figure of then current path and environment is saved after each iteration, and at the end combined to create a movie. 
- `startWithEmptyMap`: When `True`, the agent (UAV) has no initial knowledge of the environment. When `False`, the agent is aware of the random fixed individual obstacles (FIO) and fixed rectangular obstacles (FRO). Obstacle configuration is explained more later.
- `makeRandObs`: When `True`, additional random obstacles are generated during each iteration.
  - `minObs, maxObs`: Upper and lower bounds on the number of random obstacles generated during each iteration.
  - `maxPercent`: Stop generating random obstacles when `n` percent of all nodes are filled with obstacles, where `n` is the value entered here.
  - `seedDyn`: If desired, seed the random number generator used to generated random obstacles at each iteration.
- `useMovingGoals`: Set to `True` to simulate moving goals. The current configuration of the moving goals function, `movingGoal`, moves the goal randomly, but this can be modified if desired. Note that this random movement is seeded to ensure deterministic results, so the `movingGoal` function must be modified for true randomness.
- `restrictVerticalMovement`: Set to `True` if the agent cannot travel vertically. This setting prevents vertical successor nodes from being considered.
- `useHierarchicalPlanning`: Set to `False` if you do not want to use the default hierarchical planning approach. This is useful if you have a known map, or just want more flexibility of the number of hierarchical levels used. Used in conjunction with `numHierLevels`.
- `numHierLevels`: use this when `useHierarchicalPlanning` is False to allow control over the tradeoff between path length and run time. A value of 0 will use no hierarchical levels and find the shortest path (for the given heuristic), while a value of 1 will reduce run time at the expense of path cost. A value of 2 will further reduce run time at the expense of path length, and so on.
- `percentFixedRandomObstacles`: Set the percentage of the map to initially cover in randomly generated FIO which are of size 5x5x5.
  - `seedStatic`: If desired, seed the random number generator used to create the initial fixed obstacles.
- `safetyMargin`: Prevents the agent from traveling within `n` nodes of obstacles, where `n` is the value entered here. Works by extending the footprint of obstacles by `n` nodes in each direction.
- `cX, cY, cZ`: Directional cost factors for the x-, y-, and z-directions. Scales the cost of travel by the value entered here. Note that if `cX` or `cY` are not set to 1, the `computeCost` function will need to be modified, as its current configuration assumes `cX = cY = 1`.
- `heuristicScale`: Multiplies the heuristic by the value entered here. Larger values can find paths faster, but may find less optimal paths. A value sightly above 1 is recommended, such as 1.01.
- `searchRadius`: The sensor range of the agent, as the number of nodes.
- `refinementDistace`: Refines the segment of the path within `n` nodes of the agent, where `n` is the value entered here. Larger values find shorter and more realistic paths but increases computation time.
- `t_max`: A soft limit on planning time. Path refinement is terminated when this value is exceeded.
- `sizeX, sizeY, sizeX`: Dimensions of the map.
- `mapscale`: Multiples map dimensions and FRO by this value.
- `start`: Start location of the agent, entered as (x,y,z) coordinates.
- `goals`: All goals to travel to, where each goal is its own row. Entered as (x,y,z,0), where 0 is a placeholder used when there are moving goals (it stores a unique integer generated from the (x,y,z) values in order keep track of the goals).
  - When there are multiple goals, the goal location selected is that with the shortest Euclidean distance from the current location.
- `initX, initY, initZ`: Lists containing intial coordinates of moving goals.
- `T`: Moving goals will not move every `n` iterations, where `n` is the value entered here. This is to ensure the goals can be caught by the agent.
- `obstacles`: Enter locations of any FIO as `(x,y,z)`.
- `rXstart, rYstart, rZstart`: Lists containing the start coordinates of FRO.
- `rXdim, rYdim, rZdim`: `(x,y,z)` dimensions of FRO, describes how far to extend FRO from the coordinates entered in `rXstart, rYstart, rZstart`.
- `vidName`: If creating a video, give it a name here.
- `fps`: Frames per second for the video. Higher value results in faster playback but slower runtime.
- `dpi`: Resolution of each frame of the video. Higher value results in better quality but slower runtime.
- `imgformat`: Image format for figure. Should be left as `png`.


Worth noting is the default approach is to create splines to smooth out the straight line paths to generate more realistic paths. If the splines are not desired, the can be turned off by commenting out `path = fcn.CatmullRomSpline(path)` in main_hdstar.py. When main_hdstar.py is ran, config_user.py is imported. This function sets up obstacles, figures, etc. You generally will not need to edit this function, but there a few things you can change here.
- Modify the approach used to choose the next goal (when there are multiple goals) beginning at the line where `hyp = []` is declared. Currently, the goal with the shortest Euclidean distance from the start location is used. 
  - If changing this, you also need to change the section beginning at `if len(gl.goals) > 1:` in main_hdstar.py
- Under the comment `# Generating random fixed obstacles` is the line `rLoc = fcn.rectObs(newXFixed, newYFixed, newZFixed, 5,5,5)`. The 5,5,5 portion is what configures the fixed random obstacles to be 5x5x5. So modify this line if you want to change those dimensions.
- Plot setting can be modified under the comment `# Configure plot settings`

The last imported file is all_functions.py, which contains all the functions needed to run the script. Then the main file, main_hdstar.py, is run. This file plans a path, and the UAV follows it until the path becomes invalidated. The path becomes invalidated when one of three conditions occur:

1. An obstacle blocks the current path
2. The goal moves
3. The distance travelled since the last replan equals half of the refinement distance

Regardless of which condition occurs, a new path is planned. The process is repeated until the goal is reached.
