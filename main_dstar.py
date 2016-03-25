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

sizeX, sizeY, sizeZ, cX, cY, cZ = gl.sizeX, gl.sizeY, gl.sizeZ, gl.cX, gl.cY, gl.cZ

searchRadius, useMovingGoals, smoothPath = gl.searchRadius, gl.useMovingGoals, gl.smoothPath
makeRandObs, makeFigure, makeMovie, numlevels = gl.makeRandObs, gl.makeFigure, gl.makeMovie, gl.numlevels

minObs, maxObs, maxPercent, seedDyn, seedStatic = gl.minObs, gl.maxObs, gl.maxPercent, gl.seedDyn, gl.seedStatic
initX, initY, initZ, T, rXdim, rYdim, rZdim = gl.initX, gl.initY, gl.initZ, gl.T, gl.rXdim, gl.rYdim, gl.rZdim
rXstart, rYstart, rZstart = gl.rXstart, gl.rYstart, gl.rZstart

tic = time.time()
if makeMovie:   frames = []



""" Setup abstract levels """
L = fcn.CL(0, sizeX, sizeY, sizeZ)
time_findL0Path = []

#numlevels = 0
""" Begin main algorithm """
for idx in xrange(0, gl.numGoals):                      # for each goal
    xNew, yNew, zNew = gl.start                         # get current location
    fcn.searchAndUpdate(xNew,yNew,zNew)                 # search for obstacles
    while gl.start != gl.goal:
        oldstart, oldgoal = gl.start, gl.goal           # Line 48 of Moving Target D*Lite


        tic = time.clock()
        pathToFollow = L.computeShortestL0Path([gl.start, gl.goal])   # get path
        time_findL0Path.append(time.clock()-tic)


        pathToFollow = fcn.postSmoothPath(pathToFollow)
        nextcoords = fcn.fromNodesToCoordinates(pathToFollow)
        nextcoords_su = [(round(pt[0]), round(pt[1]), round(pt[2])) for pt in nextcoords]

        # Line 53 of Moving Target D* Lite is computed within below while loop
        # Line 54 of Moving Target D* Lite begins here
        goalMoved, validL0Path = False, True
        while not goalMoved and validL0Path:
            # Save current position, then move to next point
            xOld, yOld, zOld = xNew, yNew, zNew
            xNew, yNew, zNew = nextcoords.pop()
            gl.start = (round(xNew), round(yNew), round(zNew))   # update start coordinate

            # Plot movement and save figure
            if makeMovie:
                fname = ('_tmp%05d.'+gl.imgformat) %gl.stepCount
                plt.savefig(fname,dpi=gl.dpi,bbox_inches='tight')
                frames.append(fname)
            gl.ax1.plot([xOld,xNew], [yOld,yNew], [zOld,zNew], linewidth=2, c='#5DA5DA')

            # Generate random obstacles
            if makeRandObs:
                fcn.genRandObs(minObs,maxObs,maxPercent,seedDyn)

            # Moving goal execution
            if useMovingGoals:
                for i in xrange(0,len(initX)):  goalMoved = fcn.movingGoal(initX[i], initY[i], initZ[i], T[i])

            gl.stepCount += 1

            # Search time
            __, validL0Path = fcn.searchAndUpdate(xNew, yNew, zNew, nextcoords_su, nextcoords_su)

            # If we reached the goal
            if gl.start == gl.goal:
                goalMoved = True
                break








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





mean_time_findL0Path = sum(time_findL0Path)/len(time_findL0Path)



print 'Run succeeded!\n'

print 'Level 0 Expansions: ' + str(gl.closed_L0)

print '\nElapsed time: ' + str(time.time() - tic) + ' seconds'
print 'Mean findL0Path Time: ' + str(mean_time_findL0Path*1000) + ' ms'
print time_findL0Path


if makeFigure:
    print 'Figure is open. Close figure to end script'
    plt.savefig('dstarFig.pdf',bbox_inches='tight')
    plt.show()






