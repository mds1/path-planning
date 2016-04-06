from __future__ import division
import heapq
import time
from math import sqrt, ceil, floor, isinf
import random
import itertools
import collections
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import config_user as gl



# Creating local copies of unchanging variables
sizeX, sizeY, sizeZ = gl.sizeX, gl.sizeY, gl.sizeZ
cX, cY, cZ, zMove = gl.cX, gl.cY, gl.cZ, sizeX*sizeY*sizeZ

searchRadius,searchRadiusSquared = gl.searchRadius, gl.searchRadius**2
sr = searchRadius
numNodes = sizeX*sizeY*sizeZ
heuristicScale = gl.heuristicScale
makeFigure = gl.makeFigure
makeMovie = gl.makeMovie
restrictVerticalMovement = gl.restrictVerticalMovement
minclustersize = gl.minclustersize
distBetweenL0Paths = gl.distBetweenL0Paths
distancerequirement = gl.distancerequirement
refinementDistance = gl.refinementDistance


zf1, zf2 = gl.zf1, gl.zf2


def succ(s):
    """ Find which nodes can be moved to next from node s"""
    x, y, z = s

    # Define successor states, one down in z-direction
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


def rectObs(dimX, dimY, dimZ, locX, locY, locZ,):
    """
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

    #return np.column_stack((obsLocX, obsLocY, obsLocZ))
    return obsLocs


def plotRectObs(x, y, z, xd, yd, zd, alpha, axisname):
    """
    :param x, y, z: coordinates of the obstacle
    :param xd, yd, zd: width of the obstacle
    :param axisname: figure axis on which to plot
    :return: add the obstacle to plot
    """

    #x, y, z = x-0.5, y-0.5, z-0.5

    # Define each pair of x,y,z coordinates. Each column is a vertex
    xvec = [x, x+xd, x+xd, x,    x,    x+xd, x+xd, x]
    yvec = [y, y,    y+yd, y+yd, y,    y,    y+yd, y+yd]
    zvec = [z, z,    z,    z,    z+zd, z+zd, z+zd, z+zd]

     # Specify order in which to connect each vertex to make the 6 faces
    vertlist = [[0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7], [0, 1, 2, 3], [4, 5, 6, 7]]
    tupleList = zip(xvec, yvec, zvec)

    # Add polygon to axis
    poly3d = [[tupleList[vertlist[ix][iy]] for iy in xrange(len(vertlist[0]))] for ix in xrange(len(vertlist))]
    #axisname.scatter(xvec, yvec, zvec)
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

    if len(args) == 6:
        x1, y1, z1, x2, y2, z2 = args
    elif len(args) == 2:
        x1, y1, z1 = args[0]
        x2, y2 ,z2 = args[1]
    else:
        raise TypeError('lineOfSight() take either 2 or 6 arguments')


    dx, dy, dz = x2 - x1,       y2 - y1,        z2 - z1
    ax, ay, az = abs(dx)*2,     abs(dy)*2,      abs(dz)*2
    sx, sy, sz = cmp(dx,0),     cmp(dy,0),      cmp(dz,0)

    if ax >= max(ay,az):
        yD = ay - ax/2
        zD = az - ax/2

        while x1 != x2:
            xk, yk, zk = x1, y1, z1

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
            xk, yk, zk = x1, y1, z1

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
            xk, yk, zk = x1, y1, z1

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


def postSmoothPath(pathArray):
        """
        :param pathArray: current path stored as a series of nodes
        :return: Path smoothed in directions of uniform cost
        """
        k, t = 0, [pathArray[0]]


        for i in xrange(1,len(pathArray)-1):
            if not lineOfSight(t[k],pathArray[i+1]):
                x1,y1,z1 = t[k]
                x2,y2,z2 = pathArray[i+1]
                if z1 == z2 or cZ == 1:
                    k += 1
                    t.append(pathArray[i])

        k += 1
        t.append(pathArray[-1])

        return t


def genRandObs(minObs, maxObs, maxPercent, seed):
    """
    Generate random obstacles
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
        obsFraction = gl.map_[gl.map_ < 0].size / numNodes
        if obsFraction > maxPercent:
            break


        # Generate obstacle location
        newX, newY, newZ = randomint(1,sizeX), randomint(1,sizeY), randomint(1,sizeZ)
        s_obs = (newX,newY,newZ)
        #print 'node: ' + str(s_obs) + ', stepCount: ' + str(g.stepCount)

        # Don't place obstacle at start, goal, other obstacles, or within searchRadius
        if s_obs == gl.start:
            continue
        if s_obs in gl.goals:
            continue

        curX, curY, curZ = (gl.start)     # S[g.start]  # S[1,g.start], S[2,g.start], S[3,g.start]
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
    Generate ane execute moving goals
    :param initX, initY, initZ: initial location of goal vertex
    :param T: movement frequency, don't move every T iterations
    :return: 1. Moves the goal specified by input parameters; 2. boolean, whether or not goal has moved
    """
    goalMoved = False
    if gl.stepCount % T:  # don't move every T iterations

        q = cantor(initX, initY, initZ)                 # find unique cantor id
        if q in gl.goalsVisited:                        # break if we've visited it already
            return
        idx = np.where(gl.goals[:, 4] == q)[0][0]       # get current location of goal
        mgs_old = gl.goals[idx, 3]                      # get current node number of that goal

        random.seed(q+3)
        mgs = random.choice(succ(mgs_old))              # pick random successor to move to
        mgx, mgy, mgz = mgs                             # get coordinates of that location
        gl.goals[idx, 0:3] = mgx, mgy, mgz              # update location of node in goals array

        if mgs_old == gl.goal:                          # if old goal was current goal, update current goal
            gl.goal = mgs
            goalMoved = True

        if makeFigure:
            gl.goalhandles[q].remove()               # remove scatter point and add new one
            gl.goalhandles[q] = gl.ax1.scatter(mgx, mgy, mgz, c='r')

    return goalMoved


def isPowerOfTwo(x):
    """
    :param x: any number, used for map dimensions
    :return: boolean, whether or not it's a power of 2
    """
    return x != 0 and ((x & (x - 1)) == 0)


def clusterDimCheck():
    """
    :return: boolean, makes sure all appropriate values are a function of 2
    """
    if not (isPowerOfTwo(sizeX) and isPowerOfTwo(sizeY) and isPowerOfTwo(sizeZ)):
        raise ValueError('Map dimensions must be a power of 2')

    if not (isPowerOfTwo(gl.minclustersize)):
        raise ValueError('Value of all \'minclustersize\' must be a power of 2')


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
            # number of clusters in each dimension = number of clusters in largest dimension
            sf = 2+(level-1)   # scale factor
            lsize = max(sizeX/(2**sf), sizeY/(2**sf), sizeZ/(2**sf))
            lsizeX, lsizeY, lsizeZ = lsize, lsize, lsize

            if lsizeX > sizeX/minclustersize:  lsizeX = sizeX/minclustersize
            if lsizeY > sizeY/minclustersize:  lsizeY = sizeY/minclustersize
            if lsizeZ > sizeZ/minclustersize:  lsizeZ = sizeZ/minclustersize

            L[level] = CL(level, int(lsizeX), int(lsizeY), int(lsizeZ))

    return L


def searchAndUpdate(xNew,yNew,zNew,*args):
    """
    :param xNew, yNew, zNew: current location of UAV
    :param nextcoords: set of coordinates indicating where it will move next
    :param args: arg1 checks new_waypoints_nodes, arg2 checks nextcoords_nodes
    :return: boolean, whether or not new obstacles exist nearby
    """
    cellsToUpdate = []
    cellappend = cellsToUpdate.append
    validCoarsePath, validL0Path = True, True

    # Generate list of points to search
    searchRange = []
    sr_append = searchRange.append
    x,y,z = int(round(xNew)), int(round(yNew)), int(round(zNew))
    xmin, xmax = max(x-sr, 1), min(x+sr, sizeX)
    ymin, ymax = max(y-sr, 1), min(y+sr, sizeY)
    zmin, zmax = max(z-sr, 1), min(z+sr, sizeZ)
    # for dx in xrange(xmin, xmax+1):
    #     for dy in xrange(ymin, ymax+1):
    #         for dz in xrange(zmin, zmax+1):
    #             sr_append((dx,dy,dz))
    [sr_append((dx,dy,dz)) for dx in xrange(xmin, xmax+1) for dy in xrange(ymin, ymax+1) for dz in xrange(zmin, zmax+1)]

    # Search them
    for obsLoc in searchRange:
        if gl.map_[obsLoc] == - 2 or gl.map_[obsLoc] == -1:

            # Mark obstacles within search radius
            if max([abs(obsLoc[0]-xNew), abs(obsLoc[1]-yNew), abs(obsLoc[2]-zNew)]) <= searchRadius:
                cellappend(obsLoc)

                gl.map_[obsLoc] = - 1
                gl.costMatrix[obsLoc] = float('inf')

                # See if any are on current path, and if so, recalculate path
                if args:
                    new_waypoints_su = args[0]
                    nextcoords_su = args[1]
                    if obsLoc in new_waypoints_su:
                        validCoarsePath = False
                    if obsLoc in nextcoords_su:
                        validL0Path = False


    return validCoarsePath, validL0Path


def fromCoarseToWaypoints(nextpos):
    """
    :param nextpos: smoothed coarse path to follow
    :return: set of coordinates to travel to between waypoints, in order to simulate UAV movemement
    """
    new_waypoints = []
    for i in xrange(len(nextpos)-1):
        prevS, nextS = nextpos[i], nextpos[i+1]

        prevX, prevY, prevZ = prevS
        nextX, nextY, nextZ = nextS

        dX, dY, dZ = nextX-prevX, nextY-prevY, nextZ-prevZ

        maxDist = max(abs(dist) for dist in [dX, dY, dZ])
        numpoints =  int(round(maxDist/distBetweenL0Paths))
        if numpoints == 0:
            numpoints = 1
        for jj in xrange(1, numpoints+1):
            xFrac, yFrac, zFrac = dX/numpoints, dY/numpoints, dZ/numpoints
            new_waypoints.append((prevX+jj*xFrac, prevY+jj*yFrac, prevZ+jj*zFrac))

    return new_waypoints


def fromNodesToCoordinates(pathToFollow):
    """
    :param pathToFollow: series of nodes for UAV to follow
    :return: list of coordinates to move to
    """
    nextcoords = []      # Split next vertex into coordinates to travel on (for obstacle detection)
    for k in xrange(len(pathToFollow)-1):
        prevS, nextS = pathToFollow[k], pathToFollow[k+1]

        prevX, prevY, prevZ = prevS
        nextX, nextY, nextZ = nextS

        dX, dY, dZ = nextX-prevX, nextY-prevY, nextZ-prevZ

        maxDist = max(abs(dist) for dist in [dX, dY, dZ])

        for jj in xrange(1, int(maxDist+1)):
            xFrac, yFrac, zFrac = dX/maxDist, dY/maxDist, dZ/maxDist
            nextcoords.append((prevX + jj*xFrac, prevY + jj*yFrac, prevZ + jj*zFrac))

    # Next node to go to is last node in array (easier to splice in new paths this way)
    nextcoords.reverse()
    return nextcoords


def euclideanDistance(us,ut):
    """
    :param us: source node
    :param ut: target node
    :return: euclidean distance between the nodes
    """
    xs,ys,zs = us
    xt,yt,zt = ut
    dx,dy,dz = xs-xt, ys-yt, zs-zt

    return sqrt(dx*dx + dy*dy + dz*dz)


def findCoarsePath(L):
    distance = euclideanDistance(gl.start,gl.goal)
    new_waypoints = [gl.start, gl.goal]

    for level in xrange(gl.numlevels,-1,-1):        # for all levels
        if distance > distancerequirement*L[level].maxlength:     # only for big enough distances
            try:
                new_waypoints = L[level].computeShortestCoarsePath(new_waypoints)
                #new_waypoints = L[level].computeShortestPathWithWaypoints(new_waypoints)
                break
            except KeyError:    # if there's no path at that level, go a level smaller
                continue

    if level != 1:
        refined_waypoints = list(new_waypoints)
        for idx,waypoint in enumerate(new_waypoints):
            if euclideanDistance(gl.start,waypoint) <= refinementDistance:
                new_waypoints.pop(0)
            else:
                new_waypoints.pop(0)
                final_point = waypoint
                refined_waypoints = refined_waypoints[0:idx+1]
                break

        for newlevel in xrange(level-1,0,-1):
            refined_waypoints = L[newlevel].computeRefinedCoarsePath(refined_waypoints)

        new_waypoints[:0] = refined_waypoints
    return new_waypoints


def plotResultingWaypoints(waypoints,color,size):
    """
    Useful for debugging, plots coarse waypoints for current coarse path, then removes them and shows the new ones
    for subsequent coarse paths
    :param waypoints: vector of coarse nodes
    :return: updates plot with scatter points of each waypoint
    """
    X,Y,Z = [], [], []
    if gl.stepCount > 1:
        gl.hdl.remove()
    for node in waypoints:
        x,y,z = node
        X.append(x), Y.append(y), Z.append(z)
    gl.hdl = gl.ax1.scatter(X,Y,Z, c=color, s=size)


def updateTotalCost(us,ut):
    """
    :param us: source node
    :param ut: target node
    :return: additional cost added by traveling between those two nodes
    """


""" Creating classes """

class CL:   # Create level
    """ Creates a class containing the map properties for a given level """

    def __init__(self, levelnumber, lsizeX, lsizeY, lsizeZ):
        self.levelnumber = levelnumber

        self.sizeX = lsizeX
        self.sizeY = lsizeY
        self.sizeZ = lsizeZ
        self.zMove = self.sizeX*self.sizeY
        self.numclusters = self.sizeX*self.sizeY*self.sizeZ

        self.lengthX = int(sizeX/lsizeX)
        self.lengthY = int(sizeY/lsizeY)
        self.lengthZ = int(sizeZ/lsizeZ)
        self.maxlength = max(self.lengthX, self.lengthY, self.lengthZ)
        self.minlength = min(self.lengthX, self.lengthY, self.lengthZ)
        self.numNodes = self.sizeX*self.sizeY*self.sizeZ

        self.E = []
        self.appE = self.E.append
        self.S = {gl.start:[], gl.goal:[]}
        # self.clusterStart = (gl.start)
        # self.clusterGoal  = (gl.goal)


    def initialize(self,startnode,goalnode):
        """
        :param startnode: start location
        :param goalnode: goal location
        :return: resets class variables to perform computeShortestPath
        """
        CL.U = []
        CL.km = 0
        CL.g = collections.defaultdict(lambda: float('inf'))
        CL.rhs = collections.defaultdict(lambda: float('inf'))
        CL.bptr = collections.defaultdict(None)

        CL.entry_finder = {}      # mapping of tasks to entries
        CL.g[goalnode] = float('inf')
        CL.rhs[goalnode] = 0
        CL.bptr[goalnode] = None

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
       # entry = CL.entry_finder.pop(u)
        del CL.entry_finder[u]
      #  entry[-1] = '<removed-task>'


    def pop_node(self):
        """ Remove and return lowest priority task. Raise KeyError if empty """
        while True:
            key1, key2, u = heapq.heappop(CL.U)
            if u in CL.entry_finder:
                #if u is not '<removed-task>':
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


    """
    Hierarchical Functions
    """

    def computeCoarseCost(self, us, ut):
        """
        :param xs,ys,zs: source node, us, coordinates
        :param ut: target node number
        :return: cost of moving from us to ut for abstract levels
        """

        # Check line of sight if closest node is within search radius of current location


        # We search from goal to start
        dx1, dy1, dz1 = gl.start[0]-us[0], gl.start[1]-us[1], gl.start[2]-us[2]
        dx2, dy2, dz2 = gl.start[0]-ut[0], gl.start[1]-ut[1], gl.start[2]-ut[2]

        if isinf(gl.costMatrix[ut]):#  or isinf(gl.costMatrix[us]):
            return float('inf')
        elif restrictVerticalMovement:
            if abs(us[2]-ut[2])==1 and us[0] == ut[0] and us[1] == ut[1]:
                return float('inf')
        elif dx2*dx2 + dy2*dy2 + dz2*dz2 <= searchRadiusSquared or dx1*dx1 + dy1*dy1 + dz1*dz1 <= searchRadiusSquared:
            if not lineOfSight(us,ut):
                return float('inf')

        dx, dy, dz = us[0]-ut[0], us[1]-ut[1], us[2]-ut[2]
        if us[2] != ut[2]:
            sf = cZ     # scale factor
        else:
            sf = max(cX, cY)

        return sf * sqrt(dx*dx + dy*dy + dz*dz)


    def computeRefinedCost(self, us, ut):
        """
        :param xs,ys,zs: source node, us, coordinates
        :param ut: target node number
        :return: cost of moving from us to ut for abstract levels, disregarding line of sight
        """

        if isinf(gl.costMatrix[ut]):#  or isinf(gl.costMatrix[us]):
            return float('inf')
        elif restrictVerticalMovement:
            if abs(us[2]-ut[2])==1 and us[0] == ut[0] and us[1] == ut[1]:
                return float('inf')

        dx, dy, dz = us[0]-ut[0], us[1]-ut[1], us[2]-ut[2]
        if us[2] != ut[2]:
            sf = cZ     # scale factor
        else:
            sf = max(cX, cY)

        return sf * sqrt(dx*dx + dy*dy + dz*dz)


    def computeL0Cost(self, us, ut):
        """
        :param us: source node
        :param ut: target node
        :return: cost of moving from us to ut for level 0
        """


        if isinf(gl.costMatrix[ut]):#  or isinf(gl.costMatrix[us]):
            return float('inf')
        elif restrictVerticalMovement:
            if abs(us[2]-ut[2])==1 and us[0] == ut[0] and us[1] == ut[1]:
                return float('inf')

        # if us[2] == 1 or ut[2] == 1 or us[2] == 0 or ut[2] == 0:
        #     return float('inf')

        dx, dy, dz = us[0]-ut[0], us[1]-ut[1], us[2]-ut[2]

        if us[2] != ut[2]:
            sf = cZ     # scale factor
        else:
            sf = max(cX, cY)

        return sf * sqrt(dx*dx + dy*dy + dz*dz)


    def coarse_succ(self,s,startnode):
        """ Find which nodes can be moved to next from node s"""
        x, y, z = s #S[s]

        # Define successor states, one down in z-direction
        sDel = []
        succNode = [
            (x+self.lengthX, y+self.lengthY, (z-self.lengthZ)*zf1 + zf2),
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
            (x,              y,              (z+self.lengthZ)*zf1 + zf2),
            ]


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

        # check if start node is a successor
        startx, starty, startz = startnode
        dx, dy, dz = abs(x-startx), abs(y-starty), abs(z-startz)
        if max(dx,dy,dz) <= self.maxlength:
            succNode.append(startnode)

        return succNode  # [sn for sn in succNode if sn > 0]


    def computeShortestCoarsePath(self,waypoints):
        """
        :param waypoints: list of points to computeShortestPath for
        (i.e. from waypoint[0] to waypoint [1], from waypoint[1] to waypoint[2],..., from waypoint[n-1] to waypoint[n]
        :return: new set of waypoints for the refinement planner
        """

        new_waypoints, startnode, goalnode = [], [], []
        numloscalls = 0
        for idx, node in enumerate(waypoints[0:-1]):
            startnode, goalnode = node, waypoints[idx+1]
            self.initialize(startnode,goalnode)

            kOld1, kOld2, u = self.pop_node()
            k1, k2 = self.calcKey(startnode, startnode)
            gl.closed_coarse += 1

            while kOld1 < k1 or (kOld1 == k1 and kOld2 < k2) or CL.rhs[startnode] > CL.g[startnode]:

                kNew1,kNew2 = self.calcKey(u, startnode)
                if kOld1 < kNew1 or (kOld1 == kNew1 and kOld2 < kNew2):
                    self.add_node(kNew1, kNew2, u)

                elif CL.g[u] > CL.rhs[u]:
                    CL.g[u] = CL.rhs[u]
                    succU = self.coarse_succ(u,startnode)

                    for s in succU:
                        theCost = self.computeCoarseCost(u, s)
                        if s != goalnode and CL.rhs[s] > CL.g[u] + theCost:
                            numloscalls += 1
                            CL.bptr[s] = u
                            CL.rhs[s] = CL.g[u] + theCost
                            self.updateVertex(s, startnode)
                else:
                    CL.g[u] = float('inf')
                    succU = self.coarse_succ(u,startnode)
                    for s in succU:
                        if s != goalnode and CL.bptr[s] == u:
                            succS = self.coarse_succ(s,startnode)

                            minArray = {}
                            for sp in succS:
                                minArray[sp] = CL.g[sp] + self.computeCoarseCost(s, sp) #self.getCoarseCost(sp, s)

                            # Find min by comparing second element of each tuple
                            CL.bptr[s], CL.rhs[s] = min(minArray.items(), key=lambda x:x[1])

                            if isinf(CL.rhs[s]):
                                CL.bptr[s] = float('NaN')

                        self.updateVertex(s, startnode)

                kOld1, kOld2, u = self.pop_node()
                k1, k2 = self.calcKey(startnode, startnode)
                gl.closed_coarse += 1

            nextpos = [startnode]
            while nextpos[-1] != goalnode:
                nextpos.append(CL.bptr[nextpos[-1]])

            new_waypoints.extend(nextpos[0:-1])
        new_waypoints.append(goalnode)
        return new_waypoints


    def computeRefinedCoarsePath(self,waypoints):
        """
        :param waypoints: list of points to computeShortestPath for
        (i.e. from waypoint[0] to waypoint [1], from waypoint[1] to waypoint[2],..., from waypoint[n-1] to waypoint[n]
        :return: new set of waypoints for a lower level planner
        """

        new_waypoints, startnode, goalnode = [], [], []
        numloscalls = 0
        for idx, node in enumerate(waypoints[0:-1]):
            startnode, goalnode = node, waypoints[idx+1]
            self.initialize(startnode,goalnode)

            kOld1, kOld2, u = self.pop_node()
            k1, k2 = self.calcKey(startnode, startnode)
            gl.closed_refined += 1

            while kOld1 < k1 or (kOld1 == k1 and kOld2 < k2) or CL.rhs[startnode] > CL.g[startnode]:

                kNew1,kNew2 = self.calcKey(u, startnode)
                if kOld1 < kNew1 or (kOld1 == kNew1 and kOld2 < kNew2):
                    self.add_node(kNew1, kNew2, u)

                elif CL.g[u] > CL.rhs[u]:
                    CL.g[u] = CL.rhs[u]
                    succU = self.coarse_succ(u,startnode)

                    for s in succU:
                        theCost = self.computeRefinedCost(u, s)
                        if s != goalnode and CL.rhs[s] > CL.g[u] + theCost:
                            numloscalls += 1
                            CL.bptr[s] = u
                            CL.rhs[s] = CL.g[u] + theCost
                            self.updateVertex(s, startnode)
                else:
                    CL.g[u] = float('inf')
                    succU = self.coarse_succ(u,startnode)
                    for s in succU:
                        if s != goalnode and CL.bptr[s] == u:
                            succS = self.coarse_succ(s,startnode)

                            minArray = {}
                            for sp in succS:
                                minArray[sp] = CL.g[sp] + self.computeRefinedCost(s, sp) #self.getCoarseCost(sp, s)

                            # Find min by comparing second element of each tuple
                            CL.bptr[s], CL.rhs[s] = min(minArray.items(), key=lambda x:x[1])

                            if isinf(CL.rhs[s]):
                                CL.bptr[s] = float('NaN')

                        self.updateVertex(s, startnode)

                kOld1, kOld2, u = self.pop_node()
                k1, k2 = self.calcKey(startnode, startnode)
                gl.closed_refined += 1

            # Get the backpointers
            # nextpos = np.array([startnode])
            # while nextpos[-1] != goalnode:
            #     nextpos = np.append(nextpos, CL.bptr[nextpos[-1]])
            nextpos = [startnode]
            while nextpos[-1] != goalnode:
                nextpos.append(CL.bptr[nextpos[-1]])

            new_waypoints.extend(nextpos[0:-1])
        new_waypoints.append(goalnode)
        return new_waypoints


    def computeShortestL0Path(self, waypoints):
        """
        Only use for level 0 map. The only difference is the successor function
        :param waypoints: list of points to computeShortestPath to, in that order
        :return: set of waypoints to smooth and then travel on
        """

        new_waypoints, startnode, goalnode = [], [], []
        for idx, node in enumerate(waypoints[0:-1]):
            startnode, goalnode = node, waypoints[idx+1]
            self.initialize(startnode,goalnode)


            kOld1, kOld2, u = self.pop_node()
            k1, k2 = self.calcKey(startnode, startnode)
            gl.closed_L0 += 1

            while kOld1 < k1 or (kOld1 == k1 and kOld2 < k2) or CL.rhs[startnode] > CL.g[startnode]:


                kNew1,kNew2 = self.calcKey(u, startnode)
                if kOld1 < kNew1 or (kOld1 == kNew1 and kOld2 < kNew2):
                    self.add_node(kNew1, kNew2, u)

                elif CL.g[u] > CL.rhs[u]:
                    CL.g[u] = CL.rhs[u]
                    succU = succ(u)

                    for s in succU:
                        theCost = self.computeL0Cost(u, s)
                        if s != goalnode and CL.rhs[s] > CL.g[u] + theCost:
                            CL.bptr[s] = u
                            CL.rhs[s] = CL.g[u] + theCost
                            self.updateVertex(s, startnode)
                else:
                    CL.g[u] = float('inf')
                    succU = succ(u)
                    for s in succU:
                        if s != goalnode and CL.bptr[s] == u:
                            succS = succ(s)

                            minArray = {}
                            for sp in succS:
                                minArray[sp] = CL.g[sp] + self.computeL0Cost(sp, s)

                            # Find min by comparing second element of each tuple
                            CL.bptr[s], CL.rhs[s] = min(minArray.items(), key=lambda x:x[1])

                            if isinf(CL.rhs[s]):
                                CL.bptr[s] = float('NaN')

                        self.updateVertex(s, startnode)

                kOld1, kOld2, u = self.pop_node()
                k1, k2 = self.calcKey(startnode, startnode)
                gl.closed_L0 += 1

            nextpos = [startnode]
            while nextpos[-1] != goalnode:
                nextpos.append(CL.bptr[nextpos[-1]])
            new_waypoints.extend(nextpos[0:-1])
        new_waypoints.append(goalnode)

        return new_waypoints