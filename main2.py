# Main file to test our code
# import time # commented out in this script since it wasn't being used
import Astar as ast
from maze import Maze


def main():
    # totalTraj = 0 # commented out here
    # totalCells = 0
    totalFinal = 0
    totalFull = 0
    
    iterations = 100 # desired number of mazes to generate
    density = 0.315 # likliehood of a cell being blocked
    # need to reset astar variables since previous iterations will be included
    ast.trajectoryUnknown = 0
    ast.cellsProcessed = 0
    
    for i in range(iterations):
        m = Maze(101, density)
        
        solvable = ast.DFSSolve(m, (0,0), (100,100)) # use DFS to see if solution exists
        while ~(solvable): # if not solvable regenerate till it is
            m = Maze(101,density)
            if(ast.DFSSolve(m, (0,0), (100,100))): # break when solvable
                break

        resK = ast.AStarKnown(m, (0, 0), (100, 100), ast.mDistance, True)
        resU, trajectory, cells = ast.AStarUnknown(m, (0, 0), (100, 100), ast.mDistance, True)

        totalFinal += len(resU)
        totalFull += len(resK)
        
        print("Iteration: " + str(i+1))

    print("Density = " + str(density))
    print("Average Trajectory Length = " + str(trajectory/iterations))
    print("Average Cells Processed = " + str(cells/iterations))
    print("Path Length Final Gridworld = " + str(totalFinal/iterations))
    print("Path Length Full Gridworld = " + str(totalFull/iterations))

if __name__ == "__main__":
    main()
