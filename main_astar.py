from __future__ import division
import time
import os
import subprocess
import collections
from math import isinf, sqrt
import heapq
import numpy as np
import matplotlib.pyplot as plt
import config_program
import all_functions as fcn
import config_user as gl


sizeX, sizeY, sizeZ, cX, cY, cZ = gl.sizeX, gl.sizeY, gl.sizeZ, gl.cX, gl.cY, gl.cZ
restrictVerticalMovement = gl.restrictVerticalMovement




def add_node(fvalue, u):
    """ Add new node or update priority of existing node """
    if u in entry_finder:
        remove_node(u)
    entry_finder[u] = [fvalue, u]
    heapq.heappush(U, [fvalue,u])


def remove_node(u):
    """ Mark an existing node as removed """
    entry = entry_finder.pop(u)
    entry[-1] = '<removed-task>'


def pop_node():
    """ Remove and return lowest priority task. Raise KeyError if empty """
    while U:
        fvalue, u = heapq.heappop(U)
        if u in entry_finder:
            if u is not '<removed-task>':
                del entry_finder[u]
                return fvalue, u
    raise KeyError('Attempted to pop from an empty priority queue')


def getCost(us, ut):
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


        dx, dy, dz = us[0]-ut[0], us[1]-ut[1], us[2]-ut[2]
        if us[2] != ut[2]:
            sf = cZ     # scale factor
        else:
            sf = max(cX, cY)

        return sf * sqrt(dx*dx + dy*dy + dz*dz)


L = fcn.CL(0, sizeX, sizeY, sizeZ)

""" Initialize """
U = []
entry_finder = {}       # mapping of tasks to entries
closed = []
closed_append = closed.append
g = collections.defaultdict(lambda: float('inf'))
f = collections.defaultdict(lambda: float('inf'))
bptr = collections.defaultdict(None)


g[gl.start] = 0
bptr[gl.start] = None
add_node(fcn.heuristic(gl.start, gl.goal), gl.start)

while U:
    f_u, u = pop_node()
    if u == gl.goal:
        path = []
        while u != gl.start:
            path.append(u)
            u = bptr[u]

        break

    closed_append(u)

    for s in fcn.succ(u):
        if s in closed:
            continue  # ignore successors which are already evaluated

        fvalue = g[u] + getCost(u,s)
        if s not in g or fvalue < g[s]:
            g[s] = fvalue
            fvalue += getCost(gl.goal,s)
            add_node(fvalue,s)
            bptr[s] = u



path.append(gl.start)

path = fcn.postSmoothPath(path)

for node in path:
    xOld, yOld, zOld = gl.goal
    xNew, yNew, zNew = node

    gl.ax1.plot([xOld,xNew], [yOld,yNew], [zOld,zNew], linewidth=2, c='#60BD68')
    gl.goal = (xNew, yNew, zNew)


print 'Run succeeded!\n'
print 'Level 0 Expansions: ' + str(len(closed))

#print '\nElapsed time: ' + str(time.time() - tic) + ' seconds'
#print 'Mean findCoarsePath Time: ' + str(mean_time_findCoarsePath*1000) + ' ms'
#print time_findCoarsePath


plt.show()