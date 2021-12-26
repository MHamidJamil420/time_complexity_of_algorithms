from queue import PriorityQueue
import timeit as t
import matplotlib.pyplot as ploter

class Graph:

    def __init__(self):

        self.graph = {
            "A": [(146, ("A", "O")), (140, ("A", "S")), (494, ("A", "C"))],
            "O": [(146, ("O", "A")), (151, ("O", "S"))],
            "S": [(151, ("S", "O")), (140, ("S", "A")), (80, ("S", "R")), (99, ("S", "F"))],
            "C": [(494, ("C", "A")), (146, ("C", "R"))],
            "R": [(80, ("R", "S")), (146, ("R", "C")), (97, ("R", "P"))],
            "F": [(99, ("F", "S")), (211, ("F", "B"))],
            "B": [(211, ("B", "F")), (101, ("B", "P"))],
            "P": [(101, ("P", "B")), (97, ("P", "R")), (138, ("P", "C"))]}
        self.heristics = {
            "A": 10,
            "O": 9,
            "S": 7,
            "C": 8,
            "R": 6,
            "F": 5,
            "P": 3,
            "B": 0
        }
        self.edges = {}
        self.weights = {}
        self.populate_edges()
        self.populate_weights()

        print("edges : ", self.edges)
        print("____________________________________")
        print("weights  : ", self.weights)

    def populate_edges(self):
        for key in self.graph:
            neighbours = []
            for each_tuple in self.graph[key]:
                neighbours.append(each_tuple[1][1])
            
            self.edges[key] = neighbours

    def populate_weights(self):
        for key in self.graph:
            neighbours = self.graph[key]
            for each_tuple in neighbours:
                self.weights[each_tuple[1]] = each_tuple[0]

    def neighbors(self, node):
        return self.edges[node]

    def get_cost(self, from_node, to_node):
        return self.weights[(from_node, to_node)]

    def get_heuristic(self, node):
        return self.heristics[node]

def astar(graph, start, goal):
    visited = []
    queue = PriorityQueue()
    queue.put((0, start))
    while queue:
        cost, node = queue.get()
        if node not in visited:
            visited.append(node)
            if node == goal:
                break
            for i in graph.neighbors(node):
                if i not in visited:
                    total_cost = graph.get_cost(
                        node, i) + graph.get_heuristic(node)
                    queue.put((total_cost, i))
    print("----------> Node visited in Astar : ",len(visited))
    return visited

def ucs(graph, start, goal):
    visited = []
    queue = PriorityQueue()
    queue.put((0, start))
    while queue:
        cost, node = queue.get()
        if node not in visited:
            visited.append(node)
            if node == goal:
                break
            for i in graph.neighbors(node):
                if i not in visited:
                    total_cost = cost + graph.get_cost(node, i)
                    queue.put((total_cost, i))
    print("----------> Node visited UCS : ",len(visited))
    return visited

def greedy(graph, start, goal):
    visited = []
    queue = PriorityQueue()
    queue.put((0, start))
    while queue:
        cost, node = queue.get()
        if node not in visited:
            visited.append(node)
            if node == goal:
                break
            for i in graph.neighbors(node):
                if i not in visited:
                    total_cost = graph.get_heuristic(i)
                    queue.put((total_cost, i))
    print("----------> Node visited in greedy: ",len(visited))
    return visited

def dfs(graph, start, goal):
    explored = []
    queue = PriorityQueue()
    queue.put((0, start))
    while queue:
        cost, node = queue.get()
        print('node being explored: ', node)
        print("explored : ", explored)
        if node not in explored:
            explored.append(node)
            if node == goal:
                break
            neighbours = graph.neighbors(node)
            neighbours.reverse()
            for neighbour in neighbours:
                total_cost = graph.get_cost(node, neighbour)
                queue.put((total_cost, neighbour))
    print("----------> Node visited in DFS: ",len(explored))            
    return explored

def bfs(graph, start, goal):
    explored = []
    queue = PriorityQueue()
    queue.put((0, start))
    while queue:
        cost, node = queue.get()
        print('node being explored: ', node)
        print("explored : ", explored)
        if node not in explored:
            explored.append(node)
            if node == goal:
                break
            neighbours = graph.neighbors(node)

            for neighbour in neighbours:
                total_cost = graph.get_cost(node, neighbour)
                queue.put((total_cost, neighbour))
    print("----------> Node visited in BFS: ",len(explored))  
    return explored

startTime = t.default_timer()
print("Traversal of astar : ", astar(Graph(), "A", "B"))
endTime = t.default_timer()
totalTime = [endTime-startTime]

startTime = t.default_timer()
print("Traversal of UCS : ", ucs(Graph(), "A", "B"))
endTime = t.default_timer()
totalTime.append(endTime-startTime)

startTime = t.default_timer()
print("Traversal of greedy : ", greedy(Graph(), "A", "B"))
endTime = t.default_timer()
totalTime.append(endTime-startTime)

startTime = t.default_timer()
print("Traversal of dfs : ", dfs(Graph(), "A", "B"))
endTime = t.default_timer()
totalTime.append(endTime-startTime)

startTime = t.default_timer()
print("Traversal of bfs: ", bfs(Graph(), "A", "B"))
endTime = t.default_timer()
totalTime.append(endTime-startTime)
tick_label = ["Astar", "UCS", "Greedy", "DFS", "BFS"]

ploter.bar([1, 2, 3, 4, 5], totalTime, tick_label = tick_label,
           width=0.6)
ploter.xlabel('Algorithm')
ploter.ylabel('Time')
ploter.show()
