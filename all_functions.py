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
cX, cY, cZ, zMove = gl.cX, gl.cY, gl.cZ, sizeX * sizeY

searchRadius,searchRadiusSquared = gl.searchRadius, gl.searchRadius**2
sr = searchRadius
numNodes = sizeX, sizeY, sizeZ
heuristicScale = gl.heuristicScale
mapsize = len(gl.map_)
makeFigure = gl.makeFigure
makeMovie = gl.makeMovie
restrictVerticalMovement = gl.restrictVerticalMovement
minclustersize = gl.minclustersize
distBetweenL0Paths = gl.distBetweenL0Paths
distancerequirement = gl.distancerequirement

zf1, zf2 = gl.zf1, gl.zf2


def general_n2c(s):
    """
    Converts from node number s to (x,y,z) coordinates
        z equation derived intuitively
        y equation derived from:  s = (y-1)*(sizeX-1) + y + (sizeX*sizeY)*(z-1)
            and that was derived by inspection
        x equation derived by rearranging the equation in the c2n function
    """

    z = ceil(s / zMove)
    y = floor((s + sizeX - 1 - zMove * (z - 1)) / sizeX)
    x = s - sizeX * (y - 1) - zMove * (z - 1)

    return x,y,z


def succ(s):
    """ Find which nodes can be moved to next from node s"""
    xs, ys, zs = general_n2c(s) #S[s]

    # Define successor states, one down in z-direction
    sDel = []
    succNode = [
        s + sizeX     - zMove,
        s + sizeX + 1 - zMove,
        s + 1         - zMove,
        s - sizeX + 1 - zMove,
        s - sizeX     - zMove,
        s - sizeX - 1 - zMove,
        s - 1         - zMove,
        s + sizeX - 1 - zMove,
        s             - zMove,
        s + sizeX,
        s + sizeX + 1,
        s + 1,
        s - sizeX + 1,
        s - sizeX,
        s - sizeX - 1,
        s - 1,
        s + sizeX - 1,
        s + sizeX     + zMove,
        s + sizeX + 1 + zMove,
        s + 1         + zMove,
        s - sizeX + 1 + zMove,
        s - sizeX     + zMove,
        s - sizeX - 1 + zMove,
        s - 1         + zMove,
        s + sizeX - 1 + zMove,
        s             + zMove,
    ]

    # Nodes to delete when on a boundary
    if xs == sizeX:
        sDel += 1,2,3,10,11,12,18,19,20
    elif xs == 1:
        sDel +=5,6,7,14,15,16,22,23,24

    if ys == sizeY:
        sDel += 0,1,7,9,10,16,17,18,24
    elif ys == 1:
        sDel += 3,4,5,12,13,14,20,21,22

    if zs == sizeZ:
        sDel += 17,18,19,20,21,22,23,24,25
    elif zs == 1:
        sDel += 0,1,2,3,4,5,6,7,8

    if sDel:
        sDel = set(sDel)
        for i in sorted(sDel, reverse=True):
            del succNode[i]

    return succNode


def general_c2n(x,y,z):
    """ Converts from (x,y,z) coordinates to node number s """

    x,y,z = float(x), float(y), float(z)

    if x % 1 != 0 or y % 1 != 0 or z % 1 != 0:
        x, y, z = round(x), round(y), round(z)

    if x > sizeX or y > sizeY or z > sizeZ or x < 1 or y < 1 or z < 1:
        s = float("NaN")
    else:
        s = x + sizeX * (y - 1) + zMove * (z - 1)

    return s


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
    obsLocX, obsLocY, obsLocZ, obsLocS = [],[],[], []
    appX, appY, appZ, appS = obsLocX.append, obsLocY.append, obsLocZ.append, obsLocS.append
    for dx in xrange(0, dimX):
        for dy in xrange(0, dimY):
            for dz in xrange(0, dimZ):
                appX(locX + dx), appY(locY + dy), appZ(locZ + dz), appS(general_c2n(locX+dx, locY+dy, locZ+dz))

    return np.column_stack((obsLocX, obsLocY, obsLocZ, obsLocS))


def plotRectObs(x, y, z, xd, yd, zd, axisname):
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
    alpha = 0.2
    collection = Poly3DCollection(poly3d, linewidth=1, alpha=alpha)
    collection.set_color([0, 0, 0, alpha])
    axisname.add_collection3d(collection)


def heuristic(us,ut):
    """
    :param us: source node
    :param ut: target node
    :return: Euclidean distance between them
    """
    sx, sy, sz = general_n2c(us)    # S[us] #S[1, s], S[2, s], S[3, s]
    tx, ty, tz = general_n2c(ut)    # S[ut] # S[1, t], S[2, t], S[3, t]

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
        x1, y1, z1 = general_n2c(args[0])
        x2, y2 ,z2 = general_n2c(args[1])
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

            xCheck = round(0.5*(xk+x1))
            yCheck = round(0.5*(yk+y1))
            zCheck = round(0.5*(zk+z1))

            #voxCost = general_c2n(xCheck,yCheck,zCheck)
            voxCost = xCheck + sizeX * (yCheck - 1) + zMove * (zCheck - 1)  # c2n
            voxCost = gl.costMatrix[voxCost]

            if isinf(voxCost):
                return False
    else:
        if ay >= max(ax,az):
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

                xCheck = round(0.5*(xk+x1))
                yCheck = round(0.5*(yk+y1))
                zCheck = round(0.5*(zk+z1))

                #voxCost = general_c2n(xCheck,yCheck,zCheck)
                voxCost = xCheck + sizeX * (yCheck - 1) + zMove * (zCheck - 1)  # c2n
                voxCost = gl.costMatrix[voxCost]
                if isinf(voxCost):
                    return False

        else:
            if az > max(ax,ay):
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

                    xCheck = round(0.5*(xk+x1))
                    yCheck = round(0.5*(yk+y1))
                    zCheck = round(0.5*(zk+z1))

                    #voxCost = general_c2n(xCheck,yCheck,zCheck)
                    voxCost = xCheck + sizeX * (yCheck - 1) + zMove * (zCheck - 1)  # c2n
                    voxCost = gl.costMatrix[voxCost]
                    if isinf(voxCost):
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
                x1,y1,z1 = general_n2c(t[k])
                x2,y2,z2 = general_n2c(pathArray[i+1])
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

    for i in xrange(num2gen):
        # Stop generating obstacles if limit is reached
        obsFraction = gl.map_[gl.map_ < 0].size / mapsize
        if obsFraction > maxPercent:
            break


        # Generate obstacle location
        newX = np.random.random_integers(1,sizeX)
        newY = np.random.random_integers(1,sizeY)
        newZ = np.random.random_integers(1,sizeZ)

        s_obs = general_c2n(newX,newY,newZ)
        #print 'node: ' + str(s_obs) + ', stepCount: ' + str(g.stepCount)

        # Don't place obstacle at start, goal, other obstacles, or within searchRadius
        if s_obs == gl.start:
            continue
        if any(s_obs == gl.goals[:, 3]):
            continue
        if any(s_obs == gl.obstacles[:, 3]):
            continue

        curX, curY, curZ = general_n2c(gl.start)     # S[g.start]  # S[1,g.start], S[2,g.start], S[3,g.start]
        if max([abs(curX - newX),abs(curY - newY),abs(curZ - newZ)]) < searchRadius:
            continue

        # Update map
        gl.map_[s_obs] = -2              # -2 indicated undetected obstacle
        gl.number_of_obstacles += 1

        # Add new obstacle to plot
        if makeFigure:
            plotRectObs(newX, newY, newZ, 1, 1, 1, gl.ax1)


def movingGoal(initX, initY, initZ, T):
    """
    Generate ane execute moving goals
    :param initX, initY, initZ: initial location of goal vertex
    :param T: movement frequency, don't move every T iterations
    :return: 1. Moves the goal specified by input parameters; 2. boolean, whether or not goal has moved
    """
    goalMoved = False
    if gl.stepCount % T:  # don't move every T iterations

        q = cantor(initX, initY, initZ)             # find unique cantor id
        if q in gl.goalsVisited:                     # break if we've visited it already
            return
        idx = np.where(gl.goals[:, 4] == q)[0][0]       # get current location of goal
        mgs_old = gl.goals[idx, 3]                    # get current node number of that goal

        random.seed(q+3)
        mgs = random.choice(succ(mgs_old))          # pick random successor to move to
        mgx, mgy, mgz = general_n2c(mgs) #S[mgs]    # get coordinates of that location
        gl.goals[idx, 0:4] = mgx, mgy, mgz, mgs          # update location of node in goals array

        if mgs_old == gl.goal:                       # if old goal was current goal, update current goal
            gl.goal = mgs
            gl.goalX, gl.goalY, gl.goalZ = mgx, mgy, mgz
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

    if not isPowerOfTwo(gl.mostcoarsecluster):
        raise ValueError('Value of \'mostcoarsecluster\' must be a power of 2')


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

            if lsizeX < minclustersize:  lsizeX = minclustersize
            if lsizeY < minclustersize:  lsizeY = minclustersize
            if lsizeZ < minclustersize:  lsizeZ = minclustersize

            L[level] = CL(level, int(lsizeX), int(lsizeY), int(lsizeZ))
          #  L[level].preprocessing()



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
    for dx in xrange(xmin, xmax+1):
        for dy in xrange(ymin, ymax+1):
            for dz in xrange(zmin, zmax+1):
                sr_append((dx,dy,dz))

    # Search them
    for point in searchRange:
        oX, oY, oZ = point
        #obsLoc = general_c2n(oX,oY,oZ)
        obsLoc = oX + sizeX * (oY - 1) + zMove * (oZ - 1)
        if gl.map_[obsLoc] == - 2:

            # Mark obstacles within search radius
            if max([abs(oX-xNew), abs(oY-yNew), abs(oZ-zNew)]) <= searchRadius:
                xR, yR, zR = round(xNew), round(yNew), round(zNew)
                cellappend(obsLoc)

                gl.map_[obsLoc] = - 1
                cOld = gl.costMatrix[obsLoc]
                gl.costMatrix[obsLoc] = float('inf')

                # See if any are on current path, and if so, recalculate path
                if args:
                    new_waypoints_nodes = args[0]
                    nextcoords_nodes = args[1]
                    if obsLoc in new_waypoints_nodes:
                        validCoarsePath = False
                    if obsLoc in nextcoords_nodes:
                        validL0Path = False


    return validCoarsePath, validL0Path


def fromCoarseToWaypoints(nextpos,L):
    """
    :param nextpos: smoothed coarse path to follow
    :param L: Levels
    :return: set of coordinates to travel to
    """
    q, new_waypoints = 0, np.zeros((500, 3))
    for i in xrange(len(nextpos)-1):
        prevS, nextS = nextpos[i], nextpos[i+1]

        prevX, prevY, prevZ = general_n2c(prevS)    # [prevS]
        nextX, nextY, nextZ = general_n2c(nextS)    # S[nextS]

        dX, dY, dZ = nextX-prevX, nextY-prevY, nextZ-prevZ

        maxDist = max(abs(dist) for dist in [dX, dY, dZ])
        #minLDist = min(dist for dist in [L[1].lengthX, L[1].lengthY, L[1].lengthZ])
        numpoints =  int(round(maxDist/distBetweenL0Paths))
        if numpoints == 0:   numpoints = 1
        for jj in xrange(1, numpoints+1):
            xFrac, yFrac, zFrac = dX/numpoints, dY/numpoints, dZ/numpoints
            new_waypoints[q,0], new_waypoints[q,1], new_waypoints[q,2] = prevX+jj*xFrac, prevY+jj*yFrac, prevZ+jj*zFrac
            q += 1
    return new_waypoints[0:q, :]


def fromNodesToCoordinates(pathToFollow):
    """
    :param pathToFollow: series of nodes for UAV to follow
    :return: list of coordinates to move to
    """
    q, nextcoords = 0, np.zeros((500, 3))      # Split next vertex into coordinates to travel on (for obstacle detection)
    for k in xrange(len(pathToFollow)-1):
        prevS, nextS = pathToFollow[k], pathToFollow[k+1]

        prevX, prevY, prevZ = general_n2c(prevS)
        nextX, nextY, nextZ = general_n2c(nextS)

        dX, dY, dZ = nextX-prevX, nextY-prevY, nextZ-prevZ

        maxDist = max(abs(dist) for dist in [dX, dY, dZ])

        for jj in xrange(1, int(maxDist+1)):
            xFrac, yFrac, zFrac = dX/maxDist, dY/maxDist, dZ/maxDist
            nextcoords[q,0] = prevX + jj*xFrac
            nextcoords[q,1] = prevY + jj*yFrac
            nextcoords[q,2] = prevZ + jj*zFrac
            q += 1

    nextcoords = nextcoords[0:q, :]

    # Next node to go to is last node in array (easier to splice in new paths this way)
    return np.flipud(nextcoords)


def euclideanDistance(us,ut):
    """
    :param us: source node
    :param ut: target node
    :return: euclidean distance between the nodes
    """
    xs,ys,zs = general_n2c(us)
    xt,yt,zt = general_n2c(ut)
    dx,dy,dz = xs-xt, ys-yt, zs-zt

    return sqrt(dx*dx + dy*dy + dz*dz)


def findCoarsePath(L):
    distance = euclideanDistance(gl.start,gl.goal)
    new_waypoints = [gl.start, gl.goal]

    for level in xrange(gl.numlevels,0,-1):        # for all levels except level 0
        if distance > distancerequirement*L[level].maxlength:     # only for big enough distances
            try:
                new_waypoints = L[level].computeShortestCoarsePath(new_waypoints)
                #new_waypoints = L[level].computeShortestPathWithWaypoints(new_waypoints)

            except KeyError:    # if there's no path at that level
                continue

    return new_waypoints



""" Creating classes """

class CL:   # Create level
    """ Creates a class containing the map properties for a given level """

    num_nodes = gl.sizeX*gl.sizeY*gl.sizeZ
    # U = []
    # km = 0

#    entry_finder = {}      # mapping of tasks to entries

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
        self.clusterStart = self.fromL0ToCluster(gl.start)
        self.clusterGoal  = self.fromL0ToCluster(gl.goal)


    def c2n(self,x,y,z):
        """ Converts from (x,y,z) coordinates to node number s """

        x,y,z = float(x), float(y), float(z)

        if x % 1 != 0 or y % 1 != 0 or z % 1 != 0:
            x, y, z = floor(x), floor(y), floor(z)

        if x > self.sizeX or y > self.sizeY or z > self.sizeZ or x < 1 or y < 1 or z < 1:
            s = float("NaN")
        else:
            s = x + self.sizeX * (y - 1) + self.zMove * (z - 1)

        return s


    def n2c(self,s):
        """
        Converts from node number s to (x,y,z) coordinates

        # z equation derived intuitively
        # y equation derived from:  s = (y-1)*(sizeX-1) + y + (sizeX*sizeY)*(z-1)
        #     and that was derived by inspection
        # x equation derived by rearraging the equation in the c2n function
        """

        z = ceil(s / self.zMove)
        y = floor((s + self.sizeX - 1 - self.zMove * (z - 1)) / self.sizeX)
        x = s - self.sizeX * (y - 1) - self.zMove * (z - 1)

        return x,y,z


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
        heapq.heappush(CL.U, [key1,key2,u])


    def remove_node(self, u):
        """ Mark an existing node as removed """
        entry = CL.entry_finder.pop(u)
        entry[-1] = '<removed-task>'


    def pop_node(self):
        """ Remove and return lowest priority task. Raise KeyError if empty """
        while CL.U:
            key1, key2, u = heapq.heappop(CL.U)
            if u in CL.entry_finder:
                if u is not '<removed-task>':
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


    def getCoarseCost(self,xs,ys,zs,ut):
        """
        :param us: source node
        :param ut: target node
        :return: cost of moving from us to ut for abstract levels
        """

        # Check line of sight if closest node is within search radius of current location
     #  xs, ys, zs = general_n2c(us)
        xt, yt, zt = general_n2c(ut)

        # We search from goal to start
        dx1, dy1, dz1 = gl.startX-xs, gl.startY-ys, gl.startZ-zs
        dx2, dy2, dz2 = gl.startX-xt, gl.startY-yt, gl.startZ-zt

        if dx2*dx2 + dy2*dy2 + dz2*dz2 <= searchRadiusSquared or dx1*dx1 + dy1*dy1 + dz1*dz1 <= searchRadiusSquared:
            if not lineOfSight(xs,ys,zs,xt,yt,zt):
                return float('inf')

        dx, dy, dz = xs-xt, ys-yt, zs-zt
        if zs != zt:
            sf = cZ     # scale factor
        else:
            sf = max(cX, cY)

        return sf * sqrt(dx*dx + dy*dy + dz*dz)


    def getCost(self,us,ut):
        """
        :param us: source node
        :param ut: target node
        :return: cost of moving from us to ut for level 0
        """

        if isinf(gl.costMatrix[ut]):#  or isinf(gl.costMatrix[us]):
            return float('inf')
        elif restrictVerticalMovement and abs(us-ut) == zMove:
            return float('inf')
        elif abs(us-ut) == 1:
            return cX*1
        elif abs(us-ut) == sizeX:
            return cY*1
        elif abs(us-ut) == sizeX+1 or abs(us-ut) == sizeX-1:
            return cX*1.414213562 # 1.414213562 = sqrt(2)  # used cX, since cX=cY
        else:
            return cZ*1.732050808 # 1.732050808 = sqrt(3)



    """

    Hierarchical Functions

    """

    def succ_noDiag(self,s,*args):
        """
        :param s: node to find successors for
        :return: array of nodes that can be moved to next, excluding diagonal successors
        """
        xs, ys, zs = self.n2c(s) #S[s]

        # Define successor states, one down in z-direction
        sDel = []
        succNode = [s+1, s-1, s+self.sizeX, s-self.sizeX, s+self.zMove, s-self.zMove]

        if args:
            startnode, goalnode = args[0], args[1]
            if self.fromL0ToCluster(goalnode) == self.fromL0ToCluster(s):
                succNode.append(goalnode)
            if self.fromL0ToCluster(startnode) == self.fromL0ToCluster(s):
                succNode.append(goalnode)

        # Nodes to delete when on a boundary
        if xs == self.sizeX:
            sDel.append(0)
        elif xs == 1:
            sDel.append(1)

        if ys == self.sizeY:
            sDel.append(2)
        elif ys == 1:
            sDel.append(3)

        if zs == self.sizeZ:
            sDel.append(4)
        elif zs == 1:
            sDel.append(5)

        if sDel:
            #sDel = set(sDel)
            for i in sorted(sDel, reverse=True):
                del succNode[i]

        return succNode  # [sn for sn in succNode if sn > 0]


    def computeShortestPathWithWaypoints(self,waypoints):
        """
        :param waypoints: list of points to computeShortestPath for
        (i.e. from waypoint[0] to waypoint [1], from waypoint[1] to waypoint[2],..., from waypoint[n-1] to waypoint[n]
        :return: new set of waypoints for a lower level planner
        """

        new_waypoints, startnode, goalnode = [], [], []
        for idx, node in enumerate(waypoints[0:-1]):
            startnode, goalnode = node, waypoints[idx+1]
            self.initialize(startnode,goalnode)



            kOld1, kOld2, u = self.pop_node()
            k1, k2 = self.calcKey(startnode, startnode)

            while kOld1 < k1 or (kOld1 == k1 and kOld2 < k2) or CL.rhs[startnode] > CL.g[startnode]:

                kNew1,kNew2 = self.calcKey(u, startnode)
                if kOld1 < kNew1 or (kOld1 == kNew1 and kOld2 < kNew2):
                    self.add_node(kNew1, kNew2, u)

                elif CL.g[u] > CL.rhs[u]:
                    CL.g[u] = CL.rhs[u]
                    succU = self.S[u]

                    for s in succU:
                        theCost = self.getCost(u, s)
                        if s != goalnode and CL.rhs[s] > CL.g[u] + theCost:
                            CL.bptr[s] = u
                            CL.rhs[s] = CL.g[u] + theCost
                            self.updateVertex(s, startnode)
                else:
                    CL.g[u] = float('inf')
                    succU = self.S[u]
                    for s in succU:
                        if s != goalnode and CL.bptr[s] == u:
                            succS = self.succ_noDiag(s,startnode,goalnode)

                            minArray = {}
                            for sp in succS:
                                minArray[sp] = CL.g[sp] + self.getCost(sp, s)

                            # Find min by comparing second element of each tuple
                            CL.bptr[s], CL.rhs[s] = min(minArray.items(), key=lambda x:x[1])

                            if isinf(CL.rhs[s]):
                                CL.bptr[s] = float('NaN')

                        self.updateVertex(s, startnode)

                kOld1, kOld2, u = self.pop_node()
                k1, k2 = self.calcKey(startnode, startnode)

            # Get the backpointers
            nextpos = np.array([startnode])
            while nextpos[-1] != goalnode:
                nextpos = np.append(nextpos, CL.bptr[nextpos[-1]])

            new_waypoints.extend(nextpos[0:-1])

        new_waypoints.append(goalnode)

        return new_waypoints


    def computeShortestPathWithWaypoints_L0(self,waypoints):
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

            while kOld1 < k1 or (kOld1 == k1 and kOld2 < k2) or CL.rhs[startnode] > CL.g[startnode]:


                kNew1,kNew2 = self.calcKey(u, startnode)
                if kOld1 < kNew1 or (kOld1 == kNew1 and kOld2 < kNew2):
                    self.add_node(kNew1, kNew2, u)

                elif CL.g[u] > CL.rhs[u]:
                    CL.g[u] = CL.rhs[u]
                    succU = succ(u)

                    for s in succU:
                        theCost = self.getCost(u, s)
                        if s != goalnode and CL.rhs[s] > CL.g[u] + theCost:
                            CL.bptr[s] = u
                            CL.rhs[s] = CL.g[u] + theCost
                            self.updateVertex(s, startnode)
                else:
                    CL.g[u] = float('inf')
                    succU = succ(u)
                    for s in succU:
                        if s != goalnode and CL.bptr[s] == u:
                            succS = self.succ_noDiag(s,startnode,goalnode)

                            minArray = {}
                            for sp in succS:
                                minArray[sp] = CL.g[sp] + self.getCost(sp, s)

                            # Find min by comparing second element of each tuple
                            CL.bptr[s], CL.rhs[s] = min(minArray.items(), key=lambda x:x[1])

                            if isinf(CL.rhs[s]):
                                CL.bptr[s] = float('NaN')

                        self.updateVertex(s, startnode)

                kOld1, kOld2, u = self.pop_node()
                k1, k2 = self.calcKey(startnode, startnode)

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


    def fromL0ToCluster(self,s):
        """ Convert from Level 0 node to cluster number of current level """
        x,y,z = general_n2c(s)
        clusterX = ceil(x/self.lengthX)
        clusterY = ceil(y/self.lengthY)
        clusterZ = ceil(z/self.lengthZ)

        return self.c2n(clusterX, clusterY, clusterZ)


    def fromClusterToL0(self,s):
        """ Output Level 0 coordinates of the center of a cluster"""
        clusterX, clusterY, clusterZ = self.n2c(s)
        minx, miny, minz = clusterX*self.lengthX, clusterY*self.lengthY, clusterZ*self.lengthZ
        centerX, centerY, centerZ = minx+self.lengthX/2, miny+self.lengthY/2, minz+self.lengthZ/2

        return centerX, centerY, centerZ


    def coarse_succ(self,s,startnode,startx,starty,startz):
        """ Find which nodes can be moved to next from node s"""
        xs, ys, zs = general_n2c(s) #S[s]

        # Define successor states, one down in z-direction
        sDel = []
        succNode = [
            s + sizeX*self.lengthX - (zMove*self.lengthZ*zf1+zf2),
            s + sizeX*self.lengthX + 1*self.lengthY   - (zMove*self.lengthZ*zf1 + zMove*zf2),
            s                      + 1*self.lengthY   - (zMove*self.lengthZ*zf1 + zMove*zf2),
            s - sizeX*self.lengthX + 1*self.lengthY   - (zMove*self.lengthZ*zf1 + zMove*zf2),
            s - sizeX*self.lengthX                    - (zMove*self.lengthZ*zf1 + zMove*zf2),
            s - sizeX*self.lengthX - 1*self.lengthY   - (zMove*self.lengthZ*zf1 + zMove*zf2),
            s                      - 1*self.lengthY   - (zMove*self.lengthZ*zf1 + zMove*zf2),
            s + sizeX*self.lengthX - 1*self.lengthY   - (zMove*self.lengthZ*zf1 + zMove*zf2),
            s                                         - (zMove*self.lengthZ*zf1 + zMove*zf2),
            s + sizeX*self.lengthX,
            s + sizeX*self.lengthX + 1*self.lengthY,
            s                      + 1*self.lengthY,
            s - sizeX*self.lengthX + 1*self.lengthY,
            s - sizeX*self.lengthX,
            s - sizeX*self.lengthX - 1*self.lengthY,
            s                      - 1*self.lengthY,
            s + sizeX*self.lengthX - 1*self.lengthY,
            s + sizeX*self.lengthX                  + (zMove*self.lengthZ*zf1 + zMove*zf2),
            s + sizeX*self.lengthX + 1*self.lengthY + (zMove*self.lengthZ*zf1 + zMove*zf2),
            s                      + 1*self.lengthY + (zMove*self.lengthZ*zf1 + zMove*zf2),
            s - sizeX*self.lengthX + 1*self.lengthY + (zMove*self.lengthZ*zf1 + zMove*zf2),
            s - sizeX*self.lengthX                  + (zMove*self.lengthZ*zf1 + zMove*zf2),
            s - sizeX*self.lengthX - 1*self.lengthY + (zMove*self.lengthZ*zf1 + zMove*zf2),
            s                      - 1*self.lengthY + (zMove*self.lengthZ*zf1 + zMove*zf2),
            s + sizeX*self.lengthX - 1*self.lengthY + (zMove*self.lengthZ*zf1 + zMove*zf2),
            s                                       + (zMove*self.lengthZ*zf1 + zMove*zf2)
        ]


        # Nodes to delete when on a boundary
        if xs > sizeX - self.lengthX:
            sDel += 1,2,3,10,11,12,18,19,20
        elif xs < 1 + self.lengthX:
            sDel +=5,6,7,14,15,16,22,23,24

        if ys > sizeY - self.lengthY:
            sDel += 0,1,7,9,10,16,17,18,24
        elif ys < 1 + self.lengthY:
            sDel += 3,4,5,12,13,14,20,21,22

        if zs > sizeZ - self.lengthZ:
            sDel += 17,18,19,20,21,22,23,24,25
        elif zs < 1 + self.lengthZ:
            sDel += 0,1,2,3,4,5,6,7,8

        if sDel:
            sDel = set(sDel)
            for i in sorted(sDel, reverse=True):
                del succNode[i]

        # check if start node is a successor
        dx, dy, dz = abs(xs-startx), abs(ys-starty), abs(zs-startz)
        if max(dx,dy,dz) <= self.maxlength:
            succNode.append(startnode)

        return succNode  # [sn for sn in succNode if sn > 0]


    def computeShortestCoarsePath(self,waypoints):
        """
        :param waypoints: list of points to computeShortestPath for
        (i.e. from waypoint[0] to waypoint [1], from waypoint[1] to waypoint[2],..., from waypoint[n-1] to waypoint[n]
        :return: new set of waypoints for a lower level planner
        """

        new_waypoints, startnode, goalnode = [], [], []
        numloscalls = 0
        for idx, node in enumerate(waypoints[0:-1]):
            startnode, goalnode = node, waypoints[idx+1]
            startx, starty, startz = general_n2c(startnode)
            self.initialize(startnode,goalnode)

            kOld1, kOld2, u = self.pop_node()
            gl.closed_coarse.append(u)
            k1, k2 = self.calcKey(startnode, startnode)


            while kOld1 < k1 or (kOld1 == k1 and kOld2 < k2) or CL.rhs[startnode] > CL.g[startnode]:

                kNew1,kNew2 = self.calcKey(u, startnode)
                if kOld1 < kNew1 or (kOld1 == kNew1 and kOld2 < kNew2):
                    self.add_node(kNew1, kNew2, u)

                elif CL.g[u] > CL.rhs[u]:
                    CL.g[u] = CL.rhs[u]
                    succU = self.coarse_succ(u,startnode,startx, starty, startz)

                    x,y,z = general_n2c(u)
                    for s in succU:
                        theCost = self.getCoarseCost(x,y,z, s)
                        if s != goalnode and CL.rhs[s] > CL.g[u] + theCost:
                            numloscalls += 1
                            CL.bptr[s] = u
                            CL.rhs[s] = CL.g[u] + theCost
                            self.updateVertex(s, startnode)
                else:
                    CL.g[u] = float('inf')
                    succU = self.coarse_succ(u,startnode,startx, starty, startz)
                    for s in succU:
                        if s != goalnode and CL.bptr[s] == u:
                            succS = self.coarse_succ(s,startnode,startx, starty, startz)

                            minArray = {}
                            x,y,z = general_n2c(s)
                            for sp in succS:
                                minArray[sp] = CL.g[sp] + self.getCoarseCost(x,y,z, sp) #self.getCoarseCost(sp, s)

                            # Find min by comparing second element of each tuple
                            CL.bptr[s], CL.rhs[s] = min(minArray.items(), key=lambda x:x[1])

                            if isinf(CL.rhs[s]):
                                CL.bptr[s] = float('NaN')

                        self.updateVertex(s, startnode)

                kOld1, kOld2, u = self.pop_node()
                k1, k2 = self.calcKey(startnode, startnode)
                gl.closed_coarse.append(u)

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








