# robotic-navigation-system-
1.	TITLE : The title of my project  is robotic navigation system using BFS ( breadth first search ), DFS( depth  first search ), UCS (uniform cost search ) and heuristic search .

2.	INTRODUCTION : Robotic navigation occupies a special area in artificial intelligence . In this project a robot moves from one location to another by avoiding barriers . This mainly uses BFS,DFS ,UCS  types of search algorithm .This will help the robot  to find the best possible path in a environment like a map.

3.	MOTIVATION OF THE PROJECT : Nowadays robots are widely used n industries ,education and healthcare systems . we need to navigate efficiently because this will save the time energy and various resources . This project will compare the different AI search algorithm for ideal pathfinding .

4.	PROBLEM STATEMENT :  The main problem is to design a robot system which can find the shortest and the most efficient path from start node to the end node by using different search algorithms.

5.	OBJECTIVE OF THE PROJECT : 

•	To implement  BFS , DFS, UCS , A*algorithms   .
•	To compare the performance .
•	To find shortest and optimal path .
•	To show robot movement in a grid environment .

6) EXISTING METHODS : Traditional navigation includes only simple problem solutions it does not contain the solution of complex problem . New technologies include the modern AI approaches like searching algorithm and decision making .

7) PROS AND CONS OF THE STATED METHODS : 
BFS :
•	Finds shortest path 
•	High memory usage is there .

DFS : 
•	First go deeply into the solution and then if it is wrong then come back and try again 
•	Basically its backtracking .
•	May not find the shortest path 
UCS :
•	It is uniform cost search 
•	It suggests the cheapest and the optimal solution of the path.
•	It needs good heuristic search 

8)  HARDWARE AND SOFTWARE REQUIREMENTS : 
HARDWARE : 
•	Laptop  
•	minimum 4GB RAM
SOFTWARE : 
•	Python 
•	Google collab 
•	Libraries : NumPy , Matplotlib



9) METHODOLOGY AND GOAL : This project uses a grid based environment on which the robot moves step by step. The main goal of this project is to compare the result of path length ,time taken and efficiency .

10) FUNCTIONAL MODULE DESIGN AND ANALYSIS :
•	Input module 
•	 Search algorithm module 
•	Pathfinding module 
•	Visualization  module 
•	Output modules 

11) ALGORITHM DEVELOPMENT : 
      1) Start 
      2) Define grid with barriers 
      3)  Apply search algorithm 
      4) Track the visited nodes 
      5) Generate the path  developed 
      6) Output result 
      7) Stop 

12)  SOFTWARE ARCHITECTURAL DIAGRAM :
 

13) CODING : This project uses python language for each algorithm . It explores nodes  uniquely and returns the final path .
from collections import deque
import heapq

# حركة directions (up, down, left, right)
directions = [(-1,0), (1,0), (0,-1), (0,1)]

# Check valid cell
def is_valid(grid, x, y, visited):
    return (0 <= x < len(grid) and
            0 <= y < len(grid[0]) and
            grid[x][y] == 0 and
            (x, y) not in visited)

# ---------------- BFS ----------------
def bfs(grid, start, goal):
    queue = deque([(start, [start])])
    visited = set()

    while queue:
        (x, y), path = queue.popleft()

        if (x, y) == goal:
            return path

        visited.add((x, y))

        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if is_valid(grid, nx, ny, visited):
                queue.append(((nx, ny), path + [(nx, ny)]))

    return None

# ---------------- DFS ----------------
def dfs(grid, start, goal):
    stack = [(start, [start])]
    visited = set()

    while stack:
        (x, y), path = stack.pop()

        if (x, y) == goal:
            return path

        if (x, y) in visited:
            continue

        visited.add((x, y))

        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if is_valid(grid, nx, ny, visited):
                stack.append(((nx, ny), path + [(nx, ny)]))

    return None

# ---------------- UCS ----------------
def ucs(grid, start, goal):
    pq = []
    heapq.heappush(pq, (0, start, [start]))
    visited = set()

    while pq:
        cost, (x, y), path = heapq.heappop(pq)

        if (x, y) == goal:
            return path, cost

        if (x, y) in visited:
            continue

        visited.add((x, y))

        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if is_valid(grid, nx, ny, visited):
                heapq.heappush(pq, (cost + 1, (nx, ny), path + [(nx, ny)]))

    return None

# ---------------- A* ----------------
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid, start, goal):
    pq = []
    heapq.heappush(pq, (0, 0, start, [start]))
    visited = set()

    while pq:
        f, g, (x, y), path = heapq.heappop(pq)

        if (x, y) == goal:
            return path, g

        if (x, y) in visited:
            continue

        visited.add((x, y))

        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if is_valid(grid, nx, ny, visited):
                new_g = g + 1
                new_f = new_g + heuristic((nx, ny), goal)
                heapq.heappush(pq, (new_f, new_g, (nx, ny), path + [(nx, ny)]))

    return None

# ---------------- MAIN ----------------
if __name__ == "__main__":
    grid = [
        [0, 0, 0, 1, 0],
        [1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1],
        [0, 0, 0, 0, 0]
    ]

    start = (0, 0)
    goal = (4, 4)

    print("===== ROBOT NAVIGATION RESULTS =====\n")

    print("BFS Path:")
    print(bfs(grid, start, goal))

    print("\nDFS Path:")
    print(dfs(grid, start, goal))

    print("\nUCS Path:")
    result_ucs = ucs(grid, start, goal)
    print(result_ucs)

    print("\nA* Path:")
    result_astar = astar(grid, start, goal)
    print(result_astar)

14) OUTPUT :  The output compare the path distance ,time taken  and efficiency of different search algorithms .
 

15) KEY IMPLEMENTATION : 
•	Grid based system 
•	Implementation of search algorithms 
•	Comparision of efficiency 
•	Results visuals 

16) SIGNIFICANT PROJECT OUTCOMES :
•	Understanding of search algorithms 
•	Improved skills in problem solving 
•	Knowledge about navigation system 

17)  TESTING AND REFINEMENT :  This system is tested with different parameters like Barries and gird sizes . It can be improved by optimizing code and smart search .

18) REAL WORLD APPLICATIONS : 
	Self driving cars 
	Delivery system
	Game ai navigation 


19)  CONTRIBUTION OF THE PROJECT : This project helps in better understanding ai tools . and also how ai is used in real world applications and providing a base for robotic navigation system .

20) LIMITATIONS OF THE SYSTEM :
•	Works in static environment 
•	It depends on grid size
•	Heuristic accuracy is affected .

21) CONCLUSIONS : It demonstrates the different search algorithms ,and A* perform best in all the available cases due to its efficiency and optimality .
22) FUTUTRE ENHANCEMENT:
•	Uses real robotic hardware 
•	Implement ML for better learning 
•	3d environment navigation 
•	Dynamic barriers 

23) REFERENCES :
•	Artificial intelligence a modern approach 
•	Python documentation 
•	Research papers on finding algorithms .







