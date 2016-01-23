# RouteMapGraphs
RouteMapGraphs is the project deployed for the course Advanced Data Structures in Java of the San Diego University

# Details of A-star algorithm implementation

I implemented a decorator pattern in order to compute the edge weight and the heuristic weight used inside the A* algorithm.
AstarHelper is the interface that define the methods:
- computeEdgeWeight --> to compute the real weight of an edge 
- computeHeuristicWeight --> to compute the ehuristic weight of an edge

AstarHelperDecorator is an abstract class and it contains a AstarHelper as a field.

BasicAstarHelper is the concrete basic implementation of the AstarHelper (the only root for the decorator), it finds the shortest path, in other words it is the implementation of A* developed in the previous week, where computeEdgeWeight return the length of the edge and computeHeuristicWeight return the linear distance between the current node and the goal node.

RoadTypeAstarHelper is the concrete decorator and it finds the fastest path, to do this it overrides the computeEdgeWeight method in order to return the time duration of the edge, it uses "roadType" to make assumptions about speed limits.
computeHeuristicWeight returns the time duration of linear path between the current node and the goal node with the assumption that the connection is the fastest, that is the motorway. This assumption is necessary because the heuristic function must be admissible (https://en.wikipedia.org/wiki/A*_search_algorithm).

WeightedAstarHelper is another concrete decorator and it makes the heuristic function ε-admissible, in other words it speeds up the search by relaxing the admissibility criterion.  The weighted A* algorithm guarantees that the path found will have a cost of at most 1 + ε times the cost of the optimal path in the graph.

TryToAvoidRoadTypeAstarHelper is the last concrete decorator. It tries to find a path without edge with a particular streetType. It is possible to obtain this result by incrementing the weight of the edge with a particular streetType. This decorator overrides only the computeEdgeWeight, and it multiply by 20 the weight of edge of the selected type.

I extended the user interface so it is possible to see the route found by each A* algorithm, and to see the visited nodes.

The decorato pattern opens the possibility to combine the variations of the A*, open a total of 8 possible combination using the interface and much more with the code.



