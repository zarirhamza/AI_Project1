# Main file to test our code
import time
import Astar as ast
from maze import Maze


def main():
    # Q6
    timeM = 0

    for i in range(100):
        m = Maze(101, 0.3)

        start = time.time()
        ast.AStar(m, (0, 0), (100, 100), ast.mDistance, False)
        end = time.time()
        timeM += (end - start)

    print(timeM / 100)

if __name__ == "__main__":
    main()
