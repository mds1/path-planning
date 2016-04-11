# Configuration File
from __future__ import division
import math
import collections
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D



testingMode = False

# Visual Settings
makeFigure = True
makeMovie = False
startWithEmptyMap = True
makeRandObs = False
useMovingGoals = False
restrictVerticalMovement = True




vidname = 'dstarVid'
fps = 5            # higher = faster playback speed
dpi = 100           # higher = better quality, slower runtime (300
imgformat = 'png'   # currently only works for png


# Global Cost Scale Factors / Other Settings
alpha = 0.5
splinePoints = 5        # Enter 2 to not use splines, otherwise 5 is recommended
t_max = float('inf')             # Max time to spend on path-finding, in milliseconds. Enter high value to prevent restriction

mapscale = 4
percentFixedRandomObstacles = 0
safetymargin = 0
cX, cY, cZ = 1, 1, 2
heuristicScale = 1.01

searchRadius = 20
refinementDistance = math.ceil(searchRadius * 2)

zf1, zf2 = 1, 0             # provides more flexibility over coarse z-movement; zf1 = multiplier, zf2 = added constant
                                # use (1,0) for default, or (0,x) to set coarse z-successors at a distance of x
distancerequirement = 7     # determines cluster size used for coarse paths, shorter = faster, but may have longer paths
                                # distance >= distancerequirement*maxclusterdimension
                                # too small and it runs for very long and may not find a path, >=6 recommended






# Map Settings
sizeX = 64 * mapscale
sizeY = 64 * mapscale
sizeZ = 64 * mapscale
start = (3*mapscale , 4*mapscale, 6*mapscale)
#start = (5*mapscale , 5*mapscale, sizeZ/2*mapscale)
#goals = np.array([[sizeX-5., sizeY-5., sizeZ/2., 0.]])  * mapscale
goals = np.array([[62., 60., 6.,    0.]])  * mapscale
# 0 placeholders are for cantor function

# Configure Moving Goals
initX = [60, 60]# [12, 6]
initY = [50, 49]#[3, 2]
initZ = [6, 6]#[4, 7]
T = [5, 5]#[5, 2]



# Fixed Individual Obstacles
obstacles = []

# Fixed Rectangular Obstacles
rXstart = [8,  12, 15,  35, 41, 49]
rYstart = [2,  15, 35, 10, 20, 47]
rZstart = [1,  1,  1,  1,  1,  1]
rXdim =   [4,  20, 30, 5,  8,  6]
rYdim =   [9,  12, 8,  5,  8,  6]
rZdim =   [30,  8, 15, 28, 20, 28]
# rXstart = []
# rYstart = []
# rZstart = []
# rXdim =   []
# rYdim =   []
# rZdim =   []




# Generate Random Dynamic Obstacles
randomint = np.random.random_integers

minObs = 5
maxObs = 50
maxPercent = 0.05
seedDyn = np.random.randint(0,1000)
#seedDyn = np.random.randint(0,10)
seedDyn = 432


# Generate Random Fixed Obstacles
num2gen = int(round(percentFixedRandomObstacles/100 * sizeX*sizeY*sizeZ))
seedStatic = np.random.random_integers(0,1000)
#seedStatic = np.random.random_integers(0,10
seedStatic = 141




"""
====================================================================================
================== Variables below this line are not user inputs ===================
======== They are here since they're being implemented as global variables =========
====================================================================================
"""

# Modifying by scale factor
initX = [mapscale*point for point in initX]
initY = [mapscale*point for point in initY]
initZ = [mapscale*point for point in initZ]

rXstart = [mapscale*(point) for point in rXstart if point >= 1]
rYstart = [mapscale*(point) for point in rYstart if point >= 1]
rZstart = [point for point in rZstart if point >= 1]
rXdim = [mapscale*(point) for point in rXdim if point <= sizeX]
rYdim = [mapscale*(point) for point in rYdim if point <= sizeY]
rZdim = [mapscale*(point) for point in rZdim if point <= sizeZ]


if testingMode:
    makeFigure = False
    makeMove = False

if makeMovie:
    makeFigure = True

if not useMovingGoals:
    initX = []
    initY = []
    initZ = []
    T = []



goalsVisited, goalhandles, numGoals, goal = [], [], [], []
stepCount = 1              # number of total iterations
number_of_obstacles = 0    # for genRandObs function
numNodes = sizeX*sizeY*sizeZ
goalMoved = False
numlevels = 0
# For marking priority queue elements removed
removed = '<removed-task>'

# Set up UAV map and plot
map_ = collections.defaultdict(lambda : 0)
costMatrix = collections.defaultdict(lambda: 1)

if makeFigure:
    fig1 = plt.figure()
    #ax1 = fig1.add_subplot(111, projection='3d')
    ax1 = fig1.gca(projection='3d')

# Used to save some variables
hdl = []
closed_list = 0
output = {}



# Deprecated variables
distBetweenL0Paths = 8      # the max distance in x, y, or z-direction between level 0 path calculations
                                # shorter = faster on-line computation, but more jagged paths (recommended between 4-16)
minclustersize = 4  # (still used) dimension of a cluster is this many L0 nodes (4 recommended for shorter paths)