# Configuration File
from __future__ import division
import math
import collections
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


# Visual Settings
makeFigure = True
makeMovie = False
watchPlot = False
pauseDuration = np.spacing(1)

vidname = 'dstarVid'
fps = 5            # higher = faster playback speed
dpi = 100           # higher = better quality, slower runtime (300
imgformat = 'png'   # currently only works for png

# Cluster Settings
minclustersize = 8      # min dimension of a cluster is 8 L0 nodes
mostcoarsecluster = 4   # successive clusters have 4x fewer nodes in each direction

# Global Cost Scale Factors / Other Settings
searchRadius = 20
cX, cY, cZ = 1, 1, 1
heuristicScale = 1.01
mapscale = 2
zf1, zf2 = 1, 0             # provides more flexibility over z-movement; zf1 = multiplier, zf2 = added constant
distBetweenL0Paths = 8     # the max distance in x, y, or z-direction between level 0 path calculations
                            # shorter = faster on-line computation, but more jagged paths (recommended between 4-16)

startWithEmptyMap = True
smoothPath = False
makeRandObs = False
useMovingGoals =False
restrictVerticalMovement = False


# Map Settings
sizeX = 64 * mapscale
sizeY = 64 * mapscale
sizeZ = 32 * mapscale
startX = 3 * mapscale
startY = 4 * mapscale
startZ = 7 * mapscale
goals = np.array([[62., 60., 6.,    0., 0.]])  * mapscale
#goals = np.array([[15., 8., 10.,    0., 0.], [23., 23., 19.,    0., 0.]])  * mapscale
# 0 placeholders are for node and cantor function

# Generate Moving Goals
initX = []# [12, 6]
initY = []#[3, 2]
initZ = []#[4, 7]
T = []#[5, 2]

initX = [mapscale*point for point in initX]
initY = [mapscale*point for point in initY]
initZ = [mapscale*point for point in initZ]

# Fixed Individual Obstacles
obstacles = np.array([])

# Fixed Rectangular Obstacles
rXstart = [8,  12, 15,  35, 41, 49]
rYstart = [3,  15, 35, 10, 20, 47]
rZstart = [1,  1,  1,  1,  1,  1]
rXdim =   [4,  20, 30, 5,  8,  6]
rYdim =   [8,  12, 8,  5,  8,  6]
rZdim =   [30,  8, 15, 28, 20, 28]

rXstart = [mapscale*point for point in rXstart]  # modify by scale factor
rYstart = [mapscale*point for point in rYstart]
rZstart = [mapscale*point for point in rZstart]
rXdim = [mapscale*point for point in rXdim]
rYdim = [mapscale*point for point in rYdim]
rZdim = [mapscale*point for point in rZdim]


# Generate Random Dynamic Obstacles
minObs = 5
maxObs = 50
maxPercent = 0.05
seedDyn = np.random.randint(0,1000)
seedDyn = 432

# Generate Random Fixed Obstacles
num2gen = 0
seedStatic = np.random.random_integers(0,1000)
seedStatic = 141
np.random.seed(seedStatic)



"""
====================================================================================
================== Variables below this line are not user inputs ===================
======== They are here since they're being implemented as global variables =========
====================================================================================
"""

if makeMovie:
    makeFigure = True

goalsVisited, goalhandles, numGoals, start, goal = [], [], [], [], []
goalX, goalY, goalZ = [], [], []
stepCount = 1              # number of total iterations
number_of_obstacles = 0    # for genRandObs function
numNodes = sizeX*sizeY*sizeZ
goalMoved = False
numlevels = 0
# For marking priority queue elements removed
removed = '<removed-task>'

# Set up UAV map and plot

#map_ = np.zeros((sizeX*sizeY*sizeZ+1))
#costMatrix = {i: 1 for i in xrange(1,sizeX*sizeY*sizeZ+1)}
map_ = collections.defaultdict(lambda : 0)
costMatrix = collections.defaultdict(lambda: 1)

fig1 = plt.figure()
#ax1 = fig1.add_subplot(111, projection='3d')
ax1 = fig1.gca(projection='3d')

# Used to save some variables
output = {}





