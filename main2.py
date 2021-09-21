# Main file to test our code
import time
import Astar as ast
from maze import Maze


def main():
    # Q6
    timeM = 0 # time for Manhattan distance
    atl = 0 # average trajectory length
    spDisGW = 0
    spFullGW = 0
    astarCells = 0
    

    for i in range(100):
        m = Maze(101, .33)

        start = time.time() # time started
        solve = ast.AStar(m, (0, 0), (100, 100), ast.mDistance, False)
        while ~solve: # if maze isn't solvable then
            m = Maze(101, .33) # recreate maze again
            start = time.time() # restart timer
            solve = ast.AStar(m, (0, 0), (100, 100), ast.mDistance, False)
            if solve: # break when it finally is solvalbe
                break
        end = time.time() # time ended
        timeM += (end - start) # total time ran

    print(timeM / 100) #prints the run time
    print(atl)
    print(spDisGW)
    print(spFullGW)
    print(astarCells)

if __name__ == "__main__":
    main()
