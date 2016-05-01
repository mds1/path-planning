# Path Planning

Hierarchical D* Lite (HD*) is a real-time path planning algorithm for use in unknown environments. It first plans a coarse path, then refines it to improve path quality.

## How to Use :  

Open config_user.py. This file is where you can modify settings that affect oepration. Each variable is explained below.
- `testingMode`: Suppresses figure generation, outputs from main_hdstar.py (or main_dstar.py) are not printed.
- `makeFigure`: When `True`, the final path and environment is displayed upon completion.
- `makeMovie`: When `True`, the figure of then current path and environment is saved after each iteration, and at the end combined to create a movie. 
- `startWithEmptyMap`: When `True`, the agent (UAV) has no initial knowledge of the environment. When `False`, the agent is aware of the random fixed individual obstacles (FIO) and fixed rectangular obstacles (FRO). Obstacle configuration is explained more later.
- `makeRandObs`: When `True`, additional random obstacles are generated during each iteration.
  - `minObs, maxObs`: Upper and lower bounds on the number of random obstacles generated during each iteration.
  - `maxPercent`: Stop generating random obstacles when `n` percent of all nodes are filled with obstacles, where `n` is the value entered here.
  - `seedDyn`: If desired, seed the random number generator used to generated random obstacles at each iteration.
- `useMovingGoals`: Set to `True` to simulate moving goals. The current configuration of the moving goals function, `movingGoal`, moves the goal randomly, but this can be modified if desired.
- `restrictVerticalMovement`: Set to `True` if the agent cannot travel vertically. This setting prevents vertical successor nodes from being considered.
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
- `initX, initY, initZ`: Lists containing intial coordinates of moving goals.
- `T`: Moving goals will not move every `n` iterations, where `n` is the value entered here. This is to ensure the goals can be caught by the agent.
- `obstacles`: Enter locations of any FIO as `(x,y,z)`.
- `rXstart, rYstart, rZstart`: Lists containing the start coordinates of FRO.
- `rXdim, rYdim, rZdim`: `(x,y,z)` dimensions of FRO, describes how far to extend FRO from the coordinates entered in `rXstart, rYstart, rZstart`.
- `vidName`: If creating a video, give it a name here.
- `fps`: Frames per second for the video. Higher value results in faster playback but slower runtime.
- `dpi`: Resolution of each frame of the video. Higher value results in better quality but slower runtime.
- `imgformat`: Image format for figure. Should be left as `png`.

### Files
main_hdstar.py: 3D Hierarchical D* Lite algorithm  
main_dstar.py: regular D* Lite algorithm  
config_user.py: user configures all settings here  
config_program.py: sets up obstacles, figures, etc.  
all_function.py: contains all functions needed by the script  

## 
More details regarding usage to come...
