from __future__ import division
import time
import math
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import config_user as gl
import all_functions as fcn
# Create local copies of constants
sizeX, sizeY, sizeZ, zMove = gl.sizeX, gl.sizeY, gl.sizeZ, gl.sizeX * gl.sizeY
rXstart, rYstart, rZstart, rXdim, rYdim, rZdim = gl.rXstart, gl.rYstart, gl.rZstart, gl.rXdim, gl.rYdim, gl.rZdim




# Check user inputs
fcn.clusterDimCheck()


# Add moving goals to goals array
if len(gl.initX) > 0:
    newgoals = np.zeros((len(gl.initX), 5))
    for w in xrange(0, len(gl.initX)):
        newgoals[w, 0] = gl.initX[w]
        newgoals[w, 1] = gl.initY[w]
        newgoals[w, 2] = gl.initZ[w]
    gl.goals = np.vstack((gl.goals, newgoals))

gl.numGoals = gl.goals.shape[0]


# Get start and goal nodes
gl.start = fcn.general_c2n(gl.startX, gl.startY, gl.startZ)

hyp = []
for i in xrange(0, gl.numGoals):
    gX, gY, gZ = gl.goals[i, 0], gl.goals[i, 1], gl.goals[i, 2]
    xdist, ydist, zdist = gX - gl.startX, gY - gl.startY, gZ - gl.startZ
    hyp.append(math.sqrt(xdist**2 + ydist**2 + zdist**2))

goalindex = hyp.index(min(hyp))
gl.goalX, gl.goalY, gl.goalZ = gl.goals[goalindex, 0], gl.goals[goalindex, 1], gl.goals[goalindex, 2]


# Ensure start coordinate are valid
if gl.startX > sizeX or gl.startY > sizeY or gl.startX < 1 or gl.startY < 1:
    raise Exception('Start coordinates are outside of map_ boundary')


# Add node number and cantor id to goals array
hyp = []
for i in xrange(0, gl.numGoals):
    gX, gY, gZ = gl.goals[i, 0], gl.goals[i, 1], gl.goals[i, 2]
    gl.goals[i, 3], gl.goals[i, 4] = fcn.general_c2n(gX, gY, gZ), fcn.cantor(gX, gY, gZ)

gl.goal = gl.goals[goalindex, 3]



# Generating random fixed obstacles
np.random.seed(gl.seedStatic)
for i in xrange(0, gl.num2gen):
    newXFixed = np.random.random_integers(1, sizeX)
    newYFixed = np.random.random_integers(1, sizeY)
    newZFixed = np.random.random_integers(1, sizeZ)

    obsnode = fcn.general_c2n(newXFixed, newYFixed, newZFixed)
    newObsArray = [newXFixed, newYFixed, newZFixed, obsnode]

    if len(gl.obstacles) > 0:
        gl.obstacles = np.vstack((gl.obstacles, newObsArray))
    else:
        gl.obstacles = np.array(newObsArray)

"""

# Update map with obstacle coordinates, and update cost matrices such that cells
#   with obstacles have cost of infinity
#
# 0   = Open space
# 1   = UAV (not used)
# 2   = Goal (not used)
# -1  = Obstacle
# -2  = Newly detected / undetected obstacle

"""

# Generate plot with start and goal nodes
if gl.makeFigure:
    gl.goalhandles = {}
    gl.ax1.scatter(gl.startX, gl.startY, gl.startZ, c='g')
    for idx,eachgoal in enumerate(gl.goals):
        # This is done to save and remove scatter points for moving goals
        q = eachgoal[-1]
        gl.goalhandles[q] = gl.ax1.scatter(eachgoal[0], eachgoal[1], eachgoal[2], c='r')


# Plot individual obstacles
for idx, obs in enumerate(gl.obstacles):
    gl.obstacles[idx, 3] = fcn.general_c2n(obs[0], obs[1], obs[2])
    obsLoc = gl.obstacles[idx, 3]
    if gl.makeFigure:
        fcn.plotRectObs(obs[0], obs[1], obs[2], 1, 1, 1, gl.ax1)


# Plot rectangular obstacles and them to obstacles array
for i in xrange(0,len(rXstart)):
    # if i == 2:  # skip this obstacle for now
    #     continue

    # Define each pair of x,y,z coordinates. Each column is a vertex
    if gl.makeFigure:
        fcn.plotRectObs(rXstart[i], rYstart[i], rZstart[i], rXdim[i], rYdim[i], rZdim[i], gl.ax1)

    rLoc = fcn.rectObs(rXdim[i], rYdim[i], rZdim[i], rXstart[i], rYstart[i], rZstart[i])

    if len(gl.obstacles) > 0:
        gl.obstacles = np.vstack((gl.obstacles, rLoc))
    else:
        gl.obstacles = np.array(rLoc)
gl.number_of_obstacles =gl.obstacles.shape[0]


# Update cost matrix if needed
for obs in gl.obstacles:
    obsLoc = obs[3]
    if gl.startWithEmptyMap:
        gl.map_[obsLoc] = -2                     # mark as undetected obstacle
    elif not gl.startWithEmptyMap:
        gl.map_[obsLoc] = -1                     # mark as known obstacle
        gl.costMatrix[obsLoc] = float('inf')
    else:
        raise ValueError('\'startWithEmptymap\' must be equal to True or False')


# Configure plot settings
xMin, xMax = 0.5, sizeX+0.5
yMin, yMax = 0.5, sizeY+0.5
zMin, zMax = 0.5, sizeZ+0.5
if gl.makeFigure:
    gl.ax1.set_xlabel('x'), gl.ax1.set_ylabel('y'), gl.ax1.set_zlabel('z')
    gl.ax1.set_xlim(xMin, xMax), gl.ax1.set_ylim(yMin, yMax), gl.ax1.set_zlim(zMin, zMax)
    gl.ax1.view_init(elev=44., azim= -168)

#gl.ax1.set_aspect('equal','box')
"""
Scaling is done from here...
"""
x_scale=sizeX
y_scale=sizeY
z_scale=sizeZ

scale=np.diag([x_scale, y_scale, z_scale, 1.0])
scale=scale*(1.0/scale.max())
scale[3,3]=1.0

def modified_proj():
  return np.dot(Axes3D.get_proj(gl.ax1), scale)

gl.ax1.get_proj=modified_proj
"""
to here
"""

# Create clusters


# Determine number of levels
gl.numlevels = 1
maxdim = max(sizeX, sizeY, sizeZ)
while maxdim > gl.minclustersize:
    maxdim = maxdim / gl.mostcoarsecluster
    gl.numlevels += 1




# print len(L)
#
# L = {}
# L[1] = fcn.CreateLevel(1,8,8,8)
# L[1].preprocessing()
# print len(L[1].S), len(L[1].E)

# # Define outputs
# g.output = {'map_dimensions' : (sizeX, sizeY, sizeZ)}
# g.output['start_coords'] = g.startX, g.startY, g.startZ
# g.output['start_node'] = g.start
# g.output['goal_nodes'] = g.goals
# g.output['seed_dyn'] = g.seedDyn
# g.output['seed_static'] = g.seedStatic
# g.output['nodePath'] = []
# g.output['cost_scale_factors'] = g.cX, g.cY, g.cZ
