from __future__ import division, print_function
import heapq
import time
from math import sqrt, ceil, floor, isinf, pi, isnan
import random
import itertools
import collections
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import config_user as gl


from sys import getsizeof, stderr
from itertools import chain
from collections import deque
try:
    from reprlib import repr
except ImportError:
    pass



# Creating local copies of constants
sizeX, sizeY, sizeZ = gl.sizeX, gl.sizeY, gl.sizeZ
cX, cY, cZ, zMove = gl.cX, gl.cY, gl.cZ, sizeX*sizeY*sizeZ

searchRadius,searchRadiusSquared = gl.searchRadius, gl.searchRadius**2
sr = searchRadius
numNodes = sizeX*sizeY*sizeZ
heuristicScale = gl.heuristicScale
makeFigure = gl.makeFigure
makeMovie = gl.makeMovie
restrictVerticalMovement = gl.restrictVerticalMovement
distancerequirement = gl.distancerequirement
refinementDistance, refinementDistanceSquared = gl.refinementDistance, gl.refinementDistance**2
safetymargin = gl.safetymargin
alpha, splinePoints = gl.alpha, gl.splinePoints
pathComputationLimit = gl.t_max / 1000          # convert to seconds
zf1, zf2 = gl.zf1, gl.zf2
#maxTurnAngle = gl.maxTurnAngle



def succ(s):
    """ Find which nodes can be moved to next from node s. Used to randomly move any moving goals """
    x, y, z = s

    # Define successor states
    sDel = []
    succNode = [
        (x,   y+1, z-1),
        (x+1, y+1, z-1),
        (x+1, y,   z-1),
        (x+1, y-1, z-1),
        (x,   y-1, z-1),
        (x-1, y-1, z-1),
        (x-1, y,   z-1),
        (x-1, y+1, z-1),
        (x,   y,   z-1),
        (x,   y+1, z),
        (x+1, y+1, z),
        (x+1, y,   z),
        (x+1, y-1, z),
        (x,   y-1, z),
        (x-1, y-1, z),
        (x-1, y,   z),
        (x-1, y+1, z),
        (x,   y+1, z+1),
        (x+1, y+1, z+1),
        (x+1, y,   z+1),
        (x+1, y-1, z+1),
        (x,   y-1, z+1),
        (x-1, y-1, z+1),
        (x-1, y,   z+1),
        (x-1, y+1, z+1),
        (x,   y,   z+1),
    ]

    # Nodes to delete when on a boundary
    if x == sizeX:
        sDel += 1,2,3,10,11,12,18,19,20
    elif x == 1:
        sDel +=5,6,7,14,15,16,22,23,24

    if y == sizeY:
        sDel += 0,1,7,9,10,16,17,18,24
    elif y == 1:
        sDel += 3,4,5,12,13,14,20,21,22

    if z == sizeZ:
        sDel += 17,18,19,20,21,22,23,24,25
    elif z == 1:
        sDel += 0,1,2,3,4,5,6,7,8

    if sDel:
        sDel = set(sDel)
        for i in sorted(sDel, reverse=True):
            del succNode[i]

    return succNode


def cantor(x,y,z):
    """
    :param x, y, z: node coordinates
    :return: single unique integer of the 3 coordinates using cantor pairing function
    """
    x = 0.5 * (x + y) * (x + y + 1) + y
    x = 0.5 * (x + z) * (x + z + 1) + z
    return x


def rectObs(locX, locY, locZ, dimX, dimY, dimZ):
    """
    Generate rectangular obstacles
    :param dimX, dimY, dimZ: specifies the dimensions of the obstacle
    :param locX, locY, locZ: defines the bottom left corner of the obstacle
    :return: array of the individual nodes which compose the larger cuboid
    """
    obsLocX, obsLocY, obsLocZ = [],[],[]
    appX, appY, appZ = obsLocX.append, obsLocY.append, obsLocZ.append
    obsLocs = []
    obs_append = obsLocs.append
    for dx in xrange(0, dimX):
        for dy in xrange(0, dimY):
            for dz in xrange(0, dimZ):
                appX(locX + dx), appY(locY + dy), appZ(locZ + dz)
                obs_append((locX + dx, locY + dy, locZ + dz))

    return obsLocs


def plotRectObs(x, y, z, xd, yd, zd, alpha, axisname):
    """
    :param x, y, z: coordinates of the obstacle
    :param xd, yd, zd: width of the obstacle
    :param axisname: figure axis on which to plot
    :return: add the obstacle to plot
    """

    # Define each pair of x,y,z coordinates. Each column is a vertex
    xvec = [x, x+xd, x+xd, x,    x,    x+xd, x+xd, x]
    yvec = [y, y,    y+yd, y+yd, y,    y,    y+yd, y+yd]
    zvec = [z, z,    z,    z,    z+zd, z+zd, z+zd, z+zd]

     # Specify order in which to connect each vertex to make the 6 faces
    vertlist = [[0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7], [0, 1, 2, 3], [4, 5, 6, 7]]
    tupleList = zip(xvec, yvec, zvec)

    # Add polygon to axis
    poly3d = [[tupleList[vertlist[ix][iy]] for iy in xrange(len(vertlist[0]))] for ix in xrange(len(vertlist))]
    collection = Poly3DCollection(poly3d, linewidth=1, alpha=alpha)
    collection.set_color([0, 0, 0, alpha])
    axisname.add_collection3d(collection)


def heuristic(us,ut):
    """
    :param us: source node
    :param ut: target node
    :return: Euclidean distance between them
    """
    sx, sy, sz = us
    tx, ty, tz = ut

    dx, dy, dz = sx-tx, sy-ty, sz-tz

    return heuristicScale * sqrt(dx*dx + dy*dy + dz*dz)


def lineOfSight(*args):
    """
    :param us OR x1,y1,z1: source node, given as node number or coordinates
    :param ut OR x2,y2,z2: target node, given as node number or coordinates
    :return: boolean, whether or not line of sight exists between the two nodes
    """

    x1, y1, z1 = args[0]
    x2, y2 ,z2 = args[1]

    dx, dy, dz = x2 - x1,       y2 - y1,        z2 - z1
    ax, ay, az = abs(dx)*2,     abs(dy)*2,      abs(dz)*2
    sx, sy, sz = cmp(dx,0),     cmp(dy,0),      cmp(dz,0)

    if ax >= max(ay,az):
        yD = ay - ax/2
        zD = az - ax/2

        while x1 != x2:

            if yD >= 0:
                y1 += sy
                yD -= ax
            if zD >= 0:
                z1 += sz
                zD -= ax

            x1 += sx; yD += ay; zD += az

            if isinf(gl.costMatrix[(x1,y1,z1)]):
                return False

    elif ay >= max(ax,az):
        xD = ax - ay/2
        zD = az - ay/2

        while y1 != y2:

            if xD >= 0:
                x1 += sx
                xD -= ay
            if zD >= 0:
                z1 += sz
                zD -= ay

            y1 += sy; xD += ax; zD += az

            if isinf(gl.costMatrix[(x1,y1,z1)]):
                return False

    elif az > max(ax,ay):
        xD = ax - az/2
        yD = ay - az/2

        while z1 != z2:

            if xD >= 0:
                x1 += sx
                xD -= az
            if yD >= 0:
                y1 += sy
                yD -= az

            z1 += sz; xD += ax; yD += ay

            if isinf(gl.costMatrix[(x1,y1,z1)]):
                return False

    return True


def lineOfSight4SAU(*args):
    """
    This function is only called by searchAndUpdate. It checks for new obstacles, where as the other lineOfSight
    function only checks for known obstacles

    :param us OR x1,y1,z1: source node, given as node number or coordinates
    :param ut OR x2,y2,z2: target node, given as node number or coordinates
    :return: boolean, whether or not line of sight exists between the two nodes
    """

    x1, y1, z1 = args[0]
    x2, y2 ,z2 = args[1]

    dx, dy, dz = x2 - x1,       y2 - y1,        z2 - z1
    ax, ay, az = abs(dx)*2,     abs(dy)*2,      abs(dz)*2
    sx, sy, sz = cmp(dx,0),     cmp(dy,0),      cmp(dz,0)

    if ax >= max(ay,az):
        yD = ay - ax/2
        zD = az - ax/2

        while x1 != x2:

            if yD >= 0:
                y1 += sy
                yD -= ax
            if zD >= 0:
                z1 += sz
                zD -= ax

            x1 += sx; yD += ay; zD += az

            if gl.map_[(x1,y1,z1)] == - 2 or gl.map_[(x1,y1,z1)] == -1:
                return False, (x1,y1,z1)

    elif ay >= max(ax,az):
        xD = ax - ay/2
        zD = az - ay/2

        while y1 != y2:

            if xD >= 0:
                x1 += sx
                xD -= ay
            if zD >= 0:
                z1 += sz
                zD -= ay

            y1 += sy; xD += ax; zD += az

            if gl.map_[(x1,y1,z1)] == - 2 or gl.map_[(x1,y1,z1)] == -1:
                return False, (x1,y1,z1)

    elif az > max(ax,ay):
        xD = ax - az/2
        yD = ay - az/2

        while z1 != z2:

            if xD >= 0:
                x1 += sx
                xD -= az
            if yD >= 0:
                y1 += sy
                yD -= az

            z1 += sz; xD += ax; yD += ay

            if gl.map_[(x1,y1,z1)] == - 2 or gl.map_[(x1,y1,z1)] == -1:
                return False, (x1,y1,z1)

    return True, None


def postSmoothPath(pathArray):
        """
        :param pathArray: current path stored as a series of nodes
        :return: Path smoothed in directions of uniform cost
        """
        k, t = 0, [pathArray[0]]

        for i in xrange(1,len(pathArray)-1):
            x1,y1,z1 = t[k]
            x2,y2,z2 = pathArray[i+1]
            if (abs(z1-z2)>0.01 and cZ > 1) or not lineOfSight(t[k],pathArray[i+1]):
                k += 1
                t.append(pathArray[i])

        k += 1
        t.append(pathArray[-1])

        return t


def genRandObs(minObs, maxObs, maxPercent, seed):
    """
    Generates random 1x1 obstacles during traversal

    :param minObs: min mumber number of randomly generated obstacles
    :param maxObs: max mumber number of randomly generated obstacles
    :param maxPercent: max percent of map that can contain obstacles
    :param seed: used to create deterministic results
    :return: updates UAV map and plots obstacles
    """
    np.random.seed(seed + gl.stepCount)
    num2gen = np.random.random_integers(minObs,maxObs)
    randomint = np.random.random_integers
    for i in xrange(num2gen):
        # Stop generating obstacles if limit is reached
        obsFraction = len(gl.map_ ) / numNodes
        if obsFraction*100 > maxPercent:
            break

        # Generate obstacle location
        newX, newY, newZ = randomint(1,sizeX), randomint(1,sizeY), randomint(1,sizeZ)
        s_obs = (newX,newY,newZ)

        # Don't place obstacle at start, goal, other obstacles, or within searchRadius
        if s_obs == gl.start:
            continue
        if s_obs in gl.goals:
            continue

        curX, curY, curZ = gl.start
        if max([abs(curX - newX),abs(curY - newY),abs(curZ - newZ)]) < searchRadius:
            continue

        # Update map
        gl.map_[s_obs] = -2              # -2 indicated undetected obstacle
        gl.number_of_obstacles += 1

        # Add new obstacle to plot
        if makeFigure:
            plotRectObs(newX, newY, newZ, 1, 1, 1, 0.2, gl.ax1)


def movingGoal(initX, initY, initZ, T):
    """
    Generate and execute moving goals
    :param initX, initY, initZ: initial location of goal vertex
    :param T: movement frequency, don't move every T iterations
    :return: 1. Moves the goal specified by input parameters; 2. boolean, whether or not current goal has moved
    """
    goalMoved = False
    if gl.stepCount % T == 0:  # move every T iterations

        q = cantor(initX, initY, initZ)                                     # find unique cantor id
        if q in gl.goalsVisited:                                            # break if we've visited this goal already
            return
        idx = np.where(gl.goals[:, 3] == q)[0][0]                           # get current location of goal
        mgs_old = (gl.goals[idx, 0], gl.goals[idx, 1], gl.goals[idx, 2])    # get current node number of that goal

        random.seed(q+3)
        mgs = random.choice(succ(mgs_old))                                  # pick random successor to move to
        newseed = q + 4
        while mgs in gl.obstacles:
            # pick another random successor if we end up in an obstacle
            random.seed(newseed)
            mgs = random.choice(succ(mgs_old))
            newseed += 1

        mgx, mgy, mgz = mgs                                                 # get coordinates of that location
        gl.goals[idx, 0:3] = mgx, mgy, mgz                                  # update location of node in goals array

        if mgs_old == gl.goal:                                      # if old goal was current goal, update current goal
            gl.goal = mgs
            goalMoved = True

        if makeFigure:
            gl.goalhandles[q].remove()                                      # remove scatter point and add new one
            gl.goalhandles[q] = gl.ax1.scatter(mgx, mgy, mgz, c='r')

    return goalMoved


def setupLevels():
    """
    :return: Dictionary of all hierarchical levels
    """
    L = {}

    for level in xrange(gl.numlevels,-1,-1):
    # Get dimensions for each level
        if level == 0:
            lsizeX, lsizeY, lsizeZ = sizeX, sizeY, sizeZ
            L[level] = CL(level, int(lsizeX), int(lsizeY), int(lsizeZ))
        else:
            sf = 2+(level-1)   # scale factor
            lsize = max(sizeX/(2**sf), sizeY/(2**sf), sizeZ/(2**sf))
            lsizeX, lsizeY, lsizeZ = lsize, lsize, lsize

            if lsizeX > sizeX/gl.minclustersize:  lsizeX = sizeX/gl.minclustersize
            if lsizeY > sizeY/gl.minclustersize:  lsizeY = sizeY/gl.minclustersize
            if lsizeZ > sizeZ/gl.minclustersize:  lsizeZ = sizeZ/gl.minclustersize

            L[level] = CL(level, int(lsizeX), int(lsizeY), int(lsizeZ))

    return L


def searchAndUpdate(xNew,yNew,zNew,*args):
    """
    New method for faster searching and updating of nodes. Uses line-of-sight checks instead of searching all nodes
    that are within the bounding search cube (original "searchAndUpdate" function is now called "searchAndUpdate_old")

    This and markSafetyMargin are the major bottlenecks
    Modifications to speed them up would be very useful

    :param xNew, yNew, zNew: current location of UAV
    :param args: checks node of pathToFollow to ensure they still have line-of-sight
    :return: boolean, whether or not new obstacles exist nearby
    """

    cellsToUpdate = []
    cellappend = cellsToUpdate.append
    validPath = True

    """ Get endpoints of bounding search cube """
    searchRange = []
    sr_append = searchRange.append
    x,y,z = int(round(xNew)), int(round(yNew)), int(round(zNew))
    xmin, xmax = max(x-sr, 1), min(x+sr, sizeX)
    ymin, ymax = max(y-sr, 1), min(y+sr, sizeY)
    zmin, zmax = max(z-sr, 1), min(z+sr, sizeZ)

    """ Get nodes that make up the 6 faces """

    # Face 1: vary x,y at zmin
    [sr_append((dx,dy,zmin)) for dx in xrange(xmin, xmax+1) for dy in xrange(ymin, ymax+1)]
    # Face 2: vary x,y at zmax
    [sr_append((dx,dy,zmax)) for dx in xrange(xmin, xmax+1) for dy in xrange(ymin, ymax+1)]
    # Face 3: vary x,z at ymin
    [sr_append((dx,ymin,dz)) for dx in xrange(xmin, xmax+1) for dz in xrange(zmin, zmax+1)]
    # Face 4: vary x,z at ymax
    [sr_append((dx,ymax,dz)) for dx in xrange(xmin, xmax+1) for dz in xrange(zmin, zmax+1)]
    # Face 5: vary y,z at xmin
    [sr_append((xmin,dy,dz)) for dy in xrange(ymin, ymax+1) for dz in xrange(zmin, zmax+1)]
    # Face 6: vary y,z at xmax
    [sr_append((xmax,dy,dz)) for dy in xrange(ymin, ymax+1) for dz in xrange(zmin, zmax+1)]

    """ Run line-of-sight checks """
    for node in searchRange:
        los, blkdNode = lineOfSight4SAU((x,y,z), node)
        if not los:
            cellappend(blkdNode)
            gl.costMatrix[blkdNode] = float('inf')

    if cellsToUpdate:
        markSafetyMargin(cellsToUpdate,safetymargin)
    del cellsToUpdate, searchRange   # free up memory

    if args:
        path = args[0]
        path = [(round(pt[0]), round(pt[1]), round(pt[2])) for pt in reversed(path)]

        # Check line of sight between nodes in path
        if len(path) > 0:
            # Extract portion within search radius
            path_section = []
            x1,y1,z1 = gl.start
            x2,y2,z2 = path[0]
            while max([abs(x1-x2), abs(y1-y2), abs(z1-z2)]) <= max(refinementDistance,searchRadius):
                path_section.append(path.pop(0))
                if len(path) < 1:
                    break
                x2,y2,z2 = path[0]

            # For each node in path_section:
            for idx in xrange(len(path_section)-1):
                if not lineOfSight(path_section[idx],path_section[idx+1]):
                    validPath = False
                    break

            del path, path_section  # free up memory

    return validPath


def searchAndUpdate_old(xNew,yNew,zNew,*args):
    """
    This function is no longer used but is left for posterity
    It has been replaced by the more realistic and efficient "searchAndUpdate"

    :param xNew, yNew, zNew: current location of UAV
    :param args: checks node of pathToFollow to ensure they still have line-of-sight
    :return: boolean, whether or not new obstacles exist nearby
    """

    cellsToUpdate = []
    cellappend = cellsToUpdate.append
    validPath = True

    # Generate list of points to search
    searchRange = []
    sr_append = searchRange.append
    x,y,z = int(round(xNew)), int(round(yNew)), int(round(zNew))
    xmin, xmax = max(x-sr, 1), min(x+sr, sizeX)
    ymin, ymax = max(y-sr, 1), min(y+sr, sizeY)
    zmin, zmax = max(z-sr, 1), min(z+sr, sizeZ)

    [sr_append((dx,dy,dz)) for dx in xrange(xmin, xmax+1) for dy in xrange(ymin, ymax+1) for dz in xrange(zmin, zmax+1)]

    # Search them
    for obsLoc in searchRange:
        if gl.map_[obsLoc] == - 2 or gl.map_[obsLoc] == -1:
            # -1  = Known obstacle
            # -2  = Newly detected/undetected obstacle

            cellappend(obsLoc)      # Marking obstacles within search radius

            gl.map_[obsLoc] = -1
            gl.costMatrix[obsLoc] = float('inf')

    if cellsToUpdate:
        markSafetyMargin(cellsToUpdate,safetymargin)

    del cellsToUpdate, searchRange   # free up memory

    if args:
        path = args[0]
        path = [(round(pt[0]), round(pt[1]), round(pt[2])) for pt in reversed(path)]

        # Check line of sight between nodes in path
        if len(path) > 0:
            # Extract portion within search radius
            path_section = []
            x1,y1,z1 = gl.start
            x2,y2,z2 = path[0]
            while max([abs(x1-x2), abs(y1-y2), abs(z1-z2)]) <= max(refinementDistance,searchRadius):
                path_section.append(path.pop(0))
                if len(path) < 1:
                    break
                x2,y2,z2 = path[0]

            # For each node in path_section:
            for idx in xrange(len(path_section)-1):
                if not lineOfSight(path_section[idx],path_section[idx+1]):
                    validPath = False
                    break

            del path, path_section  # free up memory

    return validPath


def simulateUAVmovement(pathToFollow):
    """
    :param pathToFollow: series of nodes for UAV to follow
    :return: list of coordinates to move to
    """
    nextcoords = []
    for k in xrange(len(pathToFollow)-1):
        prevS, nextS = pathToFollow[k], pathToFollow[k+1]

        prevX, prevY, prevZ = prevS
        nextX, nextY, nextZ = nextS

        dX, dY, dZ = nextX-prevX, nextY-prevY, nextZ-prevZ
        maxDist = max(abs(dist) for dist in [dX, dY, dZ])

        if maxDist <=1:
            nextcoords.append((nextX, nextY, nextZ))
        else:
            for jj in xrange(1, int(maxDist+1)):
                xFrac, yFrac, zFrac = dX/maxDist, dY/maxDist, dZ/maxDist
                newX, newY, newZ = prevX + jj*xFrac, prevY + jj*yFrac, prevZ + jj*zFrac
                nextcoords.append((newX, newY, newZ))

    # Next node to go to is last node in array
    nextcoords.reverse()
    return nextcoords


def euclideanDistance(us,ut):
    """
    :param us: source node
    :param ut: target node
    :return: Euclidean distance between the nodes
    """
    xs,ys,zs = us
    xt,yt,zt = ut
    dx,dy,dz = xs-xt, ys-yt, zs-zt

    return sqrt(dx*dx + dy*dy + dz*dz)


def findPath(L):
    startTime = time.clock()
    distance = euclideanDistance(gl.start,gl.goal)
    path = [gl.start, gl.goal]
    first_path = True


    if distance <= distancerequirement*4: # path distance too short
        path = L[0].computeShortestPath(path,False)
        return path

    # Get most coarse path first
    for level in xrange(gl.numlevels,-1,-1):
        if distance >= distancerequirement*L[level].maxlength:
            try:
                path = L[level].computeShortestPath(path,first_path)
                first_path = False

                break
            except (KeyError, IndexError):
                # if there's no path at that level, go a level smaller
                continue



    if level != 0 and time.clock()-startTime < pathComputationLimit:
        refined_path = []
        x1,y1,z1 = gl.start
        x2,y2,z2 = path[0]
        while max([abs(x1-x2), abs(y1-y2), abs(z1-z2)]) <= refinementDistance:
            refined_path.append(path.pop(0))
            if len(path) == 1:
                    break
            x2,y2,z2 = path[0]

        refined_path.append(path.pop(0))
        # "path" now only contains nodes outside of the search radius

        for newlevel in xrange(level-1,-1,-1):
            if time.clock()-startTime < pathComputationLimit:
                refined_path = L[newlevel].computeShortestPath(refined_path, first_path)
            else:
                break

        path[:0] = refined_path     # insert refined path into the beginning of the original path

    else:
        return path

    return path


def parameterValues(ti, p1, p2):
    """
    Used to get parametric value t for Catmull-Rom spline
    """
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    return (((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2 )**0.5)**alpha + ti


def CatmullRomPoints(p0, p1, p2, p3, numPts):
    """
    :param p0, p1, p2, p3:  (x,y,z) coordinates
    :param numPts:  number of points to include in this curve segment.
    :return: [p1, generated intermediary points, p2]
    """

    # Convert points to numpy for array multiplication
    p0, p1, p2, p3 = map(np.array, [p0, p1, p2, p3])

    # Calculate t0 to t3
    t0 = 0
    t1 = parameterValues(t0, p0, p1)
    t2 = parameterValues(t1, p1, p2)
    t3 = parameterValues(t2, p2, p3)

    if t1==t0:
        t1 = 1e-8

    if t3==t2:
        t3 = t2+1e-8

    # Only find points between p1 and p2
    t = np.linspace(t1, t2, numPts)

    # Get point for each t-value using equation in: http://faculty.cs.tamu.edu/schaefer/research/catmull_rom.pdf
    t = t.reshape(len(t),1)

    L01 = (t1-t)/(t1-t0) * p0 + (t - t0) / (t1 - t0) * p1
    L12 = (t2-t)/(t2-t1) * p1 + (t - t1) / (t2 - t1) * p2
    L23 = (t3-t)/(t3-t2) * p2 + (t - t2) / (t3 - t2) * p3

    L012 = (t2-t)/(t2-t0)*L01 + (t-t0)/(t2-t0)*L12
    L123 = (t3-t)/(t3-t1)*L12 + (t-t1)/(t3-t1)*L23

    C  = (t2-t)/(t2-t1)*L012 + (t-t1)/(t2-t1)*L123
    return C


def CatmullRomSpline(pts):
    """
    Generate Catmull-Rom splines for a path and return the new path
    """

    if len(pts) > 2:
        # Duplicate first and last nodes to get their splines
        pts.insert(0, pts[0])
        pts.append(pts[-1])

        sz = len(pts)

        # Create list of (x,y,z) coordinates
        C = []

        for i in range(sz-3):
            c = CatmullRomPoints(pts[i], pts[i+1], pts[i+2], pts[i+3], splinePoints)
            C.extend(c.tolist())

        return C



    else:
        # Cannot generate spline through two nodes
        return pts


def plotResultingWaypoints(waypoints,color,size,delete):
    """
    Useful for debugging, plots nodes used to form path, then removes them and shows the new ones
    for subsequent paths
    :param waypoints: vector of nodes
    :return: updates plot with scatter points of each waypoint
    """
    if makeFigure:
        X,Y,Z = [], [], []
        if gl.stepCount > 1 and delete==True:
            gl.hdl.remove()
        for node in waypoints:
            x,y,z = node
            X.append(x), Y.append(y), Z.append(z)

        X.pop(0), Y.pop(0), Z.pop(0)        # don't plot start node
        gl.hdl = gl.ax1.scatter(X,Y,Z, c=color, s=size)


def succ6(s):
    """
    Find which nodes can be moved to next from node s, excluding diagonals
    Used to mark nodes within safety margin since its faster than using all 26 successors
    """
    x, y, z = s

    # Define successor states, one down in z-direction
    sDel = []
    succNode = [
        (x,   y,   z-1),  # Keep - 8    0
        (x,   y+1, z),    # Keep - 9    1
        (x+1, y,   z),    # Keep - 11   2
        (x,   y-1, z),    # Keep - 13   3
        (x-1, y,   z),    # Keep - 15   4
        (x,   y,   z+1),   # Keep - 25  5
    ]

    # Nodes to delete when on a boundary
    if x == sizeX:
        sDel.append(2)
    elif x == 1:
        sDel.append(4)

    if y == sizeY:
        sDel.append(1)
    elif y == 1:
        sDel.append(3)

    if z == sizeZ:
        sDel.append(5)
    elif z == 1:
        sDel.append(0)

    if sDel:
        sDel = set(sDel)
        for i in sorted(sDel, reverse=True):
            del succNode[i]

    return succNode


def markSafetyMargin(cellsToUpdate,sm):
    """
    New method for marking nodes within the safety margin, a bit faster than the old one

    This and searchAndUpdate are the major bottlenecks
    Modifications to speed them up would be very useful

    :param cellsToUpdate: list of nodes containing obstacles
    :param sm: safe distance to remain from obstacles
    :return: recursively mark successors of nodes as obstacles until safety margin is met
    """

    if sm == 0:
        return
    else:
        # First, get a list of the immediate successors
        allsucc = set()
        asa = allsucc.update

        for node in cellsToUpdate:
            succS = succ6(node)
            asa(succS)

        # Now repeat for remaining successors
        if sm > 1:
            for i in xrange(sm-1):
                cellsToAdd = []
                ce = cellsToAdd.extend
                for node in allsucc:
                    succS = succ6(node)
                    ce(succS)

                allsucc.update(cellsToAdd)

        for node in list(allsucc):
            #gl.map_[node] = -1
            gl.costMatrix[node] = float('inf')

        del allsucc


def markSafetyMargin_old(cellsToUpdate,sm):
    """
    This function is no longer used but is left for posterity
    It has been replaced by the more realistic and efficient "markSafetyMargin"

    :param cellsToUpdate: list of nodes containing obstacles
    :param sm: safe distance to remain from obstacles
    :return: recursively mark successors of nodes as obstacles until safety margin is met
    """

    if sm == 0:
        return
    else:
        # First, get a list of the immediate successors
        allsucc = []
        asa = allsucc.extend
        for node in cellsToUpdate:
            succS = succ6(node)
            asa(succS)

        # Remove duplicates
        allsucc = list(set(allsucc))

        # Now repeat for remaining successors
        if sm > 1:
            for i in xrange(sm-1):
                cellsToAdd = []
                ce = cellsToAdd.extend
                for node in allsucc:
                    succS = succ6(node)
                    ce(succS)

                allsucc.extend(list(set(cellsToAdd)))
                allsucc = list(set(allsucc))

        for node in allsucc:
            #gl.map_[node] = -1
            gl.costMatrix[node] = float('inf')


        del allsucc




class CL:   # Create level
    """ Creates a class containing the map properties for a given level """

    def __init__(self, levelnumber, lsizeX, lsizeY, lsizeZ):
        self.levelnumber = levelnumber

        self.lengthX = int(sizeX/lsizeX)        # distance of successor nodes
        self.lengthY = int(sizeY/lsizeY)
        self.lengthZ = int(sizeZ/lsizeZ)
        self.maxlength = max(self.lengthX, self.lengthY, self.lengthZ)
        self.minlength = min(self.lengthX, self.lengthY, self.lengthZ)


    def initialize(self, startnode, goalnode):
        """
        :param startnode: start location
        :param goalnode: goal location
        :return: initialization to perform computeShortestPath
        """

        CL.U = []
        CL.entry_finder = {}      # mapping of tasks to entries
        CL.km = 0
        CL.g = collections.defaultdict(lambda: float('inf'))
        CL.rhs = collections.defaultdict(lambda: float('inf'))
        CL.bptr = collections.defaultdict(None)

        CL.g[goalnode] = float('inf')
        CL.rhs[goalnode] = 0

        self.add_node(heuristic(startnode, goalnode), 0, goalnode)    # add goal to queue


    def calcKey(self,s, startnode):
        """ Calculates the key values for vertex s based on a given startnode"""
        key1 = min(CL.g[s], CL.rhs[s]) + CL.km + heuristic(startnode, s)
        key2 = min(CL.g[s], CL.rhs[s])

        return key1, key2


    def add_node(self, key1, key2, u):
        """ Add new node or update priority of existing node """
        if u in CL.entry_finder:
            self.remove_node(u)
        CL.entry_finder[u] = [key1, key2, u]
        heapq.heappush(CL.U, [key1, key2, u])


    def remove_node(self, u):
        """ Mark an existing node as removed """
        del CL.entry_finder[u]


    def pop_node(self):
        """ Remove and return lowest priority task. Raise KeyError if empty """
        while True:
            key1, key2, u = heapq.heappop(CL.U)
            if u in CL.entry_finder:
                del CL.entry_finder[u]
                return key1, key2, u
        raise KeyError('Attempted to pop from an empty priority queue')


    def updateVertex(self, u, startnode):
        """ Inserts, or removes keys/entries of u in U """
        if CL.g[u] != CL.rhs[u]:
            k1, k2 = self.calcKey(u, startnode)
            self.add_node(k1, k2, u)
        elif CL.g[u] == CL.rhs[u]:
            if u in CL.entry_finder:
                self.remove_node(u)


    def computeCost(self, us, ut, first_path):
        """
        :param us: source node
        :param ut: target node
        :return: cost of moving from us to ut for level 0
        """

        if isinf(gl.costMatrix[ut]):
            return float('inf')

        if first_path:
            # Check line of sight if closest node is within search radius of current location
            dx1, dy1, dz1 = gl.start[0]-us[0], gl.start[1]-us[1], gl.start[2]-us[2]
            dx2, dy2, dz2 = gl.start[0]-ut[0], gl.start[1]-ut[1], gl.start[2]-ut[2]

            if max([abs(dx1),abs(dy1),abs(dz1)]) <= searchRadius or max([abs(dx2),abs(dy2),abs(dz2)]) <= searchRadius:
                if not lineOfSight(us,ut):
                    return float('inf')

        dx, dy, dz = us[0]-ut[0], us[1]-ut[1], us[2]-ut[2]

        # Modify these lines if cX and cY are not 1
        if dz != 0:
            sf = cZ     # scale factor
        else:
            sf = 1

        # try:
        #     if turnAngle(ut,us,CL.bptr[us]) != 0:
        #         return sf * sqrt(dx*dx + dy*dy + dz*dz) + 0.5
        #     else:
        #         return sf * sqrt(dx*dx + dy*dy + dz*dz)
        # except KeyError:
        #     return sf * sqrt(dx*dx + dy*dy + dz*dz)
        return sf * sqrt(dx*dx + dy*dy + dz*dz)


    def succ(self, s, startnode):
        """
        :param s: node to find successors for
        :param startnode: current location of agent
        :return: list of successors nodes
        """

        x, y, z = s

        # Define successor states, one down in z-direction
        sDel = []
        succNode = [
            (x,              y+self.lengthY, (z-self.lengthZ)*zf1 + zf2),
            (x+self.lengthX, y+self.lengthY, (z-self.lengthZ)*zf1 + zf2),
            (x+self.lengthX, y,              (z-self.lengthZ)*zf1 + zf2),
            (x+self.lengthX, y-self.lengthY, (z-self.lengthZ)*zf1 + zf2),
            (x,              y-self.lengthY, (z-self.lengthZ)*zf1 + zf2),
            (x-self.lengthX, y-self.lengthY, (z-self.lengthZ)*zf1 + zf2),
            (x-self.lengthX, y,              (z-self.lengthZ)*zf1 + zf2),
            (x-self.lengthX, y+self.lengthY, (z-self.lengthZ)*zf1 + zf2),
            (x,              y,              (z-self.lengthZ)*zf1 + zf2),
            (x,              y+self.lengthY, z),
            (x+self.lengthX, y+self.lengthY, z),
            (x+self.lengthX, y,              z),
            (x+self.lengthX, y-self.lengthY, z),
            (x,              y-self.lengthY, z),
            (x-self.lengthX, y-self.lengthY, z),
            (x-self.lengthX, y,              z),
            (x-self.lengthX, y+self.lengthY, z),
            (x,              y+self.lengthY, (z+self.lengthZ)*zf1 + zf2),
            (x+self.lengthX, y+self.lengthY, (z+self.lengthZ)*zf1 + zf2),
            (x+self.lengthX, y,              (z+self.lengthZ)*zf1 + zf2),
            (x+self.lengthX, y-self.lengthY, (z+self.lengthZ)*zf1 + zf2),
            (x,              y-self.lengthY, (z+self.lengthZ)*zf1 + zf2),
            (x-self.lengthX, y-self.lengthY, (z+self.lengthZ)*zf1 + zf2),
            (x-self.lengthX, y,              (z+self.lengthZ)*zf1 + zf2),
            (x-self.lengthX, y+self.lengthY, (z+self.lengthZ)*zf1 + zf2),
            (x,              y,              (z+self.lengthZ)*zf1 + zf2)
            ]


        if restrictVerticalMovement:
            sDel += 8, 25

        # Nodes to delete when on a boundary
        if x > sizeX - self.lengthX:
            sDel += 1,2,3,10,11,12,18,19,20
        elif x < 1 + self.lengthX:
            sDel +=5,6,7,14,15,16,22,23,24

        if y > sizeY - self.lengthY:
            sDel += 0,1,7,9,10,16,17,18,24
        elif y < 1 + self.lengthY:
            sDel += 3,4,5,12,13,14,20,21,22

        if z > sizeZ - self.lengthZ:
            sDel += 17,18,19,20,21,22,23,24,25
        elif z < 1 + self.lengthZ:
            sDel += 0,1,2,3,4,5,6,7,8

        if sDel:
            sDel = set(sDel)
            for i in sorted(sDel, reverse=True):
                del succNode[i]

        # Check if start node is a successor
        startx, starty, startz = startnode
        dx, dy, dz = abs(x-startx), abs(y-starty), abs(z-startz)
        if max(dx,dy,dz) < 2*self.maxlength:
            succNode.append(startnode)

        # succNode2 = [node for node in succNode if turnAngle(node,startnode,gl.oldstart) <= maxTurnAngle+1e-6]
        # print(turnAngle(node,startnode,gl.oldstart)*180/pi)
        # return succNode2
        return succNode


    def computeShortestPath(self, waypoints, first_path):
        """
        :param waypoints: list of points to computeShortestPath for
        (i.e. from waypoint[0] to waypoint [1], from waypoint[1] to waypoint[2],..., from waypoint[n-1] to waypoint[n])
        :return: new set of waypoints to either send to lower level planner, or to smooth and follow
        """

        new_waypoints, startnode, goalnode = [], [], []

        for idx, node in enumerate(waypoints[0:-1]):
            startnode, goalnode = node, waypoints[idx+1]

            #if isinf(gl.costMatrix[startnode]):
            # if startnode in gl.obstacles:
            #     raise Exception('In an obstacle')

            self.initialize(startnode, goalnode)

            # kOld comparisons generally not used, but do not affect operation
            kOld1, kOld2, u = self.pop_node()
            k1, k2 = self.calcKey(startnode, startnode)
            gl.closed_list += 1     # counting number of nodes expanded

            while kOld1 < k1 or (kOld1 == k1 and kOld2 < k2) or CL.rhs[startnode] > CL.g[startnode]:

                kNew1,kNew2 = self.calcKey(u, startnode)
                if kOld1 < kNew1 or (kOld1 == kNew1 and kOld2 < kNew2):
                    self.add_node(kNew1, kNew2, u)

                elif CL.g[u] > CL.rhs[u]:
                    CL.g[u] = CL.rhs[u]
                    succU = self.succ(u, startnode)

                    for s in succU:
                        theCost = self.computeCost(u, s, first_path)
                        if s != goalnode and CL.rhs[s] > CL.g[u] + theCost:
                            CL.bptr[s] = u
                            CL.rhs[s] = CL.g[u] + theCost
                            self.updateVertex(s, startnode)
                else:
                    CL.g[u] = float('inf')
                    succU = self.succ(u, startnode)
                    for s in succU:
                        if s != goalnode and CL.bptr[s] == u:
                            succS = self.succ(s, startnode)

                            minArray = {}
                            for sp in succS:
                                minArray[sp] = CL.g[sp] + self.computeCost(sp, s, first_path)

                            # Find min by comparing second element of each tuple
                            CL.bptr[s], CL.rhs[s] = min(minArray.items(), key=lambda x:x[1])

                            if isinf(CL.rhs[s]):
                                CL.bptr[s] = float('NaN')

                        self.updateVertex(s, startnode)

                kOld1, kOld2, u = self.pop_node()
                k1, k2 = self.calcKey(startnode, startnode)
                gl.closed_list += 1

            if isinf(gl.costMatrix[CL.rhs[startnode]]):
                raise Exception('No path exists')

            nextpos = [startnode]
            while nextpos[-1] != goalnode:
                nextpos.append(CL.bptr[nextpos[-1]])

            new_waypoints.extend(nextpos[0:-1])

        new_waypoints.append(goalnode)
        return new_waypoints


# Recursively get size of an object
def total_size(o, handlers={}, verbose=False):
    """ Returns the approximate memory footprint an object and all of its contents.

    Automatically finds the contents of the following builtin containers and
    their subclasses:  tuple, list, deque, dict, set and frozenset.
    To search other containers, add handlers to iterate over their contents:

        handlers = {SomeContainerClass: iter,
                    OtherContainerClass: OtherContainerClass.get_elements}

    Source: http://code.activestate.com/recipes/577504/ (this link was found in the Python3 documentation)
    """

    dict_handler = lambda d: chain.from_iterable(d.items())
    all_handlers = {tuple: iter,
                    list: iter,
                    deque: iter,
                    dict: dict_handler,
                    set: iter,
                    frozenset: iter,
                   }
    all_handlers.update(handlers)     # user handlers take precedence
    seen = set()                      # track which object id's have already been seen
    default_size = getsizeof(0)       # estimate sizeof object without __sizeof__

    def sizeof(o):
        if id(o) in seen:       # do not double count the same object
            return 0
        seen.add(id(o))
        s = getsizeof(o, default_size)

        if verbose:
            print(s, type(o), repr(o), file=stderr)

        for typ, handler in all_handlers.items():
            if isinstance(o, typ):
                s += sum(map(sizeof, handler(o)))
                break
        return s

    return sizeof(o)