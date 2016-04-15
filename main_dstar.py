from __future__ import division
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import config_program
import all_functions as fcn
import config_user as gl

""" Used with known map to find shortest path, set startWithEmptyMap = False """

# To reload settings for multiple trials
if gl.testingMode:
    reload(gl)
    reload(config_program)
    reload(fcn)

# Creating local copies of constants
sizeX, sizeY, sizeZ, cX, cY, cZ = gl.sizeX, gl.sizeY, gl.sizeZ, gl.cX, gl.cY, gl.cZ
searchRadius, useMovingGoals = gl.searchRadius, gl.useMovingGoals
makeRandObs, makeFigure, makeMovie, numlevels = gl.makeRandObs, gl.makeFigure, gl.makeMovie, gl.numlevels
minObs, maxObs, maxPercent, seedDyn, seedStatic = gl.minObs, gl.maxObs, gl.maxPercent, gl.seedDyn, gl.seedStatic
initX, initY, initZ, T, rXdim, rYdim, rZdim = gl.initX, gl.initY, gl.initZ, gl.T, gl.rXdim, gl.rYdim, gl.rZdim
rXstart, rYstart, rZstart = gl.rXstart, gl.rYstart, gl.rZstart


if makeMovie:   frames = []



""" Setup one level """
L = fcn.CL(0, sizeX, sizeY, sizeZ)
time_findPath = []
total_cost = 0


""" Begin main algorithm """
for idx in xrange(0, gl.numGoals):   # for each goal
    xNew, yNew, zNew = gl.start      # get current location

    while gl.start != gl.goal:

        tic = time.clock()
        path = L.computeShortestPath([gl.start, gl.goal], False)
        path = fcn.postSmoothPath(path)
     #   path = fcn.CatmullRomSpline(path)
        path = fcn.simulateUAVmovement(path)

        findPathTime = time.clock() - tic   # end timer

        time_findPath.append(findPathTime)  # record time
        if gl.stepCount == 1:
            initialFindPathTime = findPathTime

        xOrig, yOrig, zOrig = gl.start     # to identify when leaving refinement region

        dfs = 0                 # distance from start, used to identify when path needs to be updated
        goalMoved = False       # indicates whether or not the goal has moved
        validPath = True        # indicates whether or not path being followed is still valid

        while not goalMoved and validPath and gl.start != gl.goal:

            # Follow those points until path is invalidated or we reach end of refinement region
            for point in path:

                # Save current position, then move to next point
                xOld, yOld, zOld = xNew, yNew, zNew
                xNew, yNew, zNew = path.pop()
                gl.start = (round(xNew), round(yNew), round(zNew))   # update start coordinate

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

    # Identify rows in goals array matching current goal
    k1 = np.where(gl.goals[:,0]==gl.goal[0])
    k2 = np.where(gl.goals[:,1]==gl.goal[1])
    k3 = np.where(gl.goals[:,2]==gl.goal[2])

    # Whichever row shows up in k1, k2, and k3 is the row with the current goal
    k = np.intersect1d(np.intersect1d(k1,k2),k3)

    gl.goalsVisited.append(gl.goals[k,3])           # save its goal ID
    gl.goals = np.delete(gl.goals, k, 0)            # delete that row

    if len(gl.goals) > 0:
        print 'finding next goal...'

        # Find next closest goal w.r.t. straight line distance
        hyp = {}
        for i in gl.goals:
            gx, gy, gz = i[0], i[1], i[2]
            xdist, ydist, zdist = gx-gl.start[0], gy-gl.start[1], gz-gl.start[2]
            hyp[i] = math.sqrt(xdist**2 + ydist**2 + zdist**2)

        gl.goal, __ = min(hyp.items(), key=lambda x:x[1])


# Get averages, in milliseconds
mean_time_findPath = 1000*sum(time_findPath)/len(time_findPath)

if not gl.testingMode:
    print 'Run succeeded!\n'
    print 'Elapsed time: ' + str(time.time() - tic) + ' seconds'
    print 'Total cost: ' + str(total_cost)
    print 'Path-finding Time: ' + str(mean_time_findPath)
    print 'Expanded nodes: ' + str(gl.closed_list)



def dstar_outputs():
    return total_cost, gl.closed_list, mean_time_findPath, initialFindPathTime


if makeFigure:
    print 'Figure is open. Close figure to end script'
    plt.savefig('dstarFig.pdf',bbox_inches='tight')
    plt.show()






