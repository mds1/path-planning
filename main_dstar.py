from __future__ import division
import time
import os
import subprocess
import math
import numpy as np
import matplotlib.pyplot as plt
import config_program
import all_functions as fcn
import config_user as gl


"""
USER CONFIGURATION
    use config_user.py to modify user inputs
"""


"""
PROGRAM CONFIGURATION
    done in config_program.py
    creating local copies of constants here
"""

if gl.testingMode:
    reload(gl)
    reload(config_program)
    reload(fcn)

sizeX, sizeY, sizeZ, cX, cY, cZ = gl.sizeX, gl.sizeY, gl.sizeZ, gl.cX, gl.cY, gl.cZ

searchRadius, useMovingGoals = gl.searchRadius, gl.useMovingGoals
makeRandObs, makeFigure, makeMovie, numlevels = gl.makeRandObs, gl.makeFigure, gl.makeMovie, gl.numlevels

minObs, maxObs, maxPercent, seedDyn, seedStatic = gl.minObs, gl.maxObs, gl.maxPercent, gl.seedDyn, gl.seedStatic
initX, initY, initZ, T, rXdim, rYdim, rZdim = gl.initX, gl.initY, gl.initZ, gl.T, gl.rXdim, gl.rYdim, gl.rZdim
rXstart, rYstart, rZstart = gl.rXstart, gl.rYstart, gl.rZstart

tic = time.time()
if makeMovie:   frames = []



""" Setup abstract levels """
L = fcn.CL(0, sizeX, sizeY, sizeZ)
time_findPath = []
total_cost = 0


#numlevels = 0
""" Begin main algorithm """
for idx in xrange(0, gl.numGoals):                      # for each goal
    xNew, yNew, zNew = gl.start                         # get current location
 #   fcn.searchAndUpdate(xNew,yNew,zNew)                 # search for obstacles
    while gl.start != gl.goal:

        tic1 = time.clock()
        path = L.computeShortestPath([gl.start, gl.goal], False)
        path = fcn.postSmoothPath(path)
     #   path = fcn.CatmullRomSpline(path)
        path = fcn.simulateUAVmovement(path)

        findPathTime = time.clock() - tic1  # end timer

        time_findPath.append(findPathTime)  # record time
        if gl.stepCount == 1:
            initialFindPathTime = findPathTime




        xOrig, yOrig, zOrig = gl.start     # to identify when leaving refinement region



        dfs = 0                 # distance from start, used to identify when path needs to be updated
        goalMoved = False       # indicates whether or not the goal has moved
        validPath = True        # indicates whether or not path being followed is still valid

        while not goalMoved and validPath and gl.start != gl.goal:

            # 4. Follow those points until path is invalidated or we reach end of refinement region
            for point in path:

                # Save current position, then move to next point
                xOld, yOld, zOld = xNew, yNew, zNew
                xNew, yNew, zNew = path.pop()
                gl.start = (round(xNew), round(yNew), round(zNew))   # update start coordinate

                # if isinf(gl.costMatrix[gl.start]):
                #     raise Exception('Current location is in an obstacle')

                # Update distance from start
                dx, dy, dz = xOrig-xNew, yOrig-yNew, zOrig-zNew
                dfs = math.sqrt(dx**2 + dy**2 + dz**2)


                # Plot movement and save figure
                if makeMovie:
                    fname = ('_tmp%05d.'+gl.imgformat) %gl.stepCount
                    plt.savefig(fname,dpi=gl.dpi,bbox_inches='tight')
                    frames.append(fname)
                if gl.makeFigure:
                    gl.ax1.plot([xOld,xNew], [yOld,yNew], [zOld,zNew], linewidth=2, c='#5DA5DA')

                # Update total cost of path
                total_cost += L.computeCost((xOld, yOld, zOld), (xNew, yNew, zNew), False)


                # Generate random obstacles
                if makeRandObs:
                    fcn.genRandObs(minObs,maxObs,maxPercent,seedDyn)

                # Moving goal execution
                if useMovingGoals:
                    for i in xrange(0,len(initX)):  goalMoved = fcn.movingGoal(initX[i], initY[i], initZ[i], T[i])

                # Update counter used for the two preceding functions
                gl.stepCount += 1

                # Check if there's any obstacles within search radius
                # if not fcn.searchAndUpdate(xNew, yNew, zNew, path):
                #     validPath=False
                #     break








    k = np.nonzero(gl.goals[:,0]==gl.goal[0] and gl.goals[:,1]==gl.goal[1] and gl.goals[:,2]==gl.goal[2]) # find current goal
    gl.goalsVisited.append(gl.goals[k,3])           # save its goal ID
    gl.goals = np.delete(gl.goals, k, 0)            # remove that row

    if len(gl.goals) > 0:
        print 'finding next goal...'

        # Find next closest goal w.r.t. straight line distance
        hyp = {}
        for i in gl.goals:
            gx, gy, gz = i[0], i[1], i[2]
            xdist, ydist, zdist = gx-gl.start[0], gy-gl.start[1], gz-gl.start[2]
            hyp[i] = math.sqrt(xdist**2 + ydist**2 + zdist**2)

        gl.goal, __ = min(hyp.items(), key=lambda x:x[1])

        # Reconfigure levels (needed to add in successors for new goal node)
        L = fcn.setupLevels()



if not gl.testingMode:
    print 'Run succeeded!\n'
    print 'Total cost: ' + str(total_cost)

# Get averages, in milliseconds
mean_time_findPath = 1000*sum(time_findPath)/len(time_findPath)


def dstar_outputs():
    return total_cost, gl.closed_list, mean_time_findPath, initialFindPathTime



if makeFigure:
    print 'Figure is open. Close figure to end script'
    plt.savefig('dstarFig.pdf',bbox_inches='tight')
    plt.show()






