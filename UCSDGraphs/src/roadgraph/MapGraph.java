/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import roadgraph.astarfunctions.AstarHelper;
import roadgraph.astarfunctions.BasicAstarHelper;
import roadgraph.astarfunctions.RoadTypeAstarHelper;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 *         A class which represents a graph of geographic locations Nodes in the
 *         graph are intersections between
 *
 */
public class MapGraph {
	private HashMap<GeographicPoint, MapNode> nodes;
	private HashSet<MapEdge> edges;

	/**
	 * Create a new empty MapGraph
	 */
	public MapGraph() {
		nodes = new HashMap<>();
		edges = new HashSet<MapEdge>();

	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * 
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		return nodes.values().size();
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * 
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		return nodes.keySet();
	}

	/**
	 * Get the number of road segments in the graph
	 * 
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
		return edges.size();

	}

	// For us in DEBUGGING. Print the Nodes in the graph
	public void printNodes() {
		System.out.println("****PRINTING NODES ********");
		System.out.println("There are " + getNumVertices() + " Nodes: \n");
		for (GeographicPoint pt : nodes.keySet()) {
			MapNode n = nodes.get(pt);
			System.out.println(n);
		}
	}

	/**
	 * Add a node corresponding to an intersection
	 *
	 * @param latitude
	 *            The latitude of the location
	 * @param longitude
	 *            The longitude of the location
	 */
	public void addVertex(double latitude, double longitude) {
		GeographicPoint pt = new GeographicPoint(latitude, longitude);
		this.addVertex(pt);
	}

	/**
	 * Add a node corresponding to an intersection at a Geographic Point If the
	 * location is already in the graph or null, this method does not change the
	 * graph.
	 * 
	 * @param location
	 *            The location of the intersection
	 * @return true if a node was added, false if it was not (the node was
	 *         already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {
		if (location == null) {
			return false;
		}
		MapNode node = new MapNode(location);
		return nodes.put(location, node) != null;
	}

	/**
	 * Add an edge representing a segment of a road. Precondition: The
	 * corresponding Nodes must have already been added to the graph.
	 * 
	 * @param roadName
	 *            The name of the road
	 * @param roadType
	 *            The type of the road
	 */
	public void addEdge(double lat1, double lon1, double lat2, double lon2, String roadName, String roadType) {
		// Find the two Nodes associated with this edge.
		GeographicPoint pt1 = new GeographicPoint(lat1, lon1);
		GeographicPoint pt2 = new GeographicPoint(lat2, lon2);

		MapNode n1 = nodes.get(pt1);
		MapNode n2 = nodes.get(pt2);

		// check nodes are valid
		if (n1 == null)
			throw new NullPointerException("addEdge: pt1:" + pt1 + "is not in graph");
		if (n2 == null)
			throw new NullPointerException("addEdge: pt2:" + pt2 + "is not in graph");

		addEdge(n1, n2, roadName, roadType, MapEdge.DEFAULT_LENGTH);

	}

	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType)
			throws IllegalArgumentException {
		MapNode startNode = nodes.get(from);
		MapNode endNode = nodes.get(to);

		if (from == null || to == null || roadName == null || roadType == null || startNode == null
				|| endNode == null) {
			throw new IllegalArgumentException();
		}

		addEdge(startNode, endNode, roadName, roadType, MapEdge.DEFAULT_LENGTH);
	}

	public void addEdge(GeographicPoint pt1, GeographicPoint pt2, String roadName, String roadType, double length) {
		MapNode n1 = nodes.get(pt1);
		MapNode n2 = nodes.get(pt2);

		// check nodes are valid
		if (n1 == null)
			throw new NullPointerException("addEdge: pt1:" + pt1 + "is not in graph");
		if (n2 == null)
			throw new NullPointerException("addEdge: pt2:" + pt2 + "is not in graph");

		addEdge(n1, n2, roadName, roadType, length);
	}

	// Add an edge when you already know the nodes involved in the edge
	private void addEdge(MapNode n1, MapNode n2, String roadName, String roadType, double length) {
		MapEdge edge = new MapEdge(roadName, roadType, n1, n2, length);
		edges.add(edge);
		n1.addEdge(edge);
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return bfs(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// Hook for visualization. See writeup.
		// nodeSearched.accept(next.getLocation());
		// Setup - check validity of inputs
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		MapNode startNode = nodes.get(start);
		MapNode goalNode = nodes.get(goal);
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (goalNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}

		HashMap<MapNode, MapNode> parentMap = new HashMap<>();

		if (!bfsSearch(startNode, goalNode, nodeSearched, parentMap)) {
			System.out.println("No path exists");
			return null;
		}
		// reconstruct the path
		return reconstructPath(startNode, goalNode, parentMap);

	}

	/**
	 * 
	 * @param startNode
	 * @param goalNode
	 * @param parentMap
	 * @return
	 */
	public List<GeographicPoint> reconstructPath(MapNode startNode, MapNode goalNode,
			HashMap<MapNode, MapNode> parentMap) {
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		double totalDistance = 0.00;
		MapNode curr = goalNode;
		while (curr != startNode) {
			path.addFirst(curr.getLocation());
			curr = parentMap.get(curr);
			totalDistance += curr.getActualWeight();
		}
		path.addFirst(startNode.getLocation());
		System.out.println("Route weight: " + totalDistance);
		return path;
	}

	/**
	 * 
	 * @param startNode
	 * @param goalNode
	 * @param nodeSearched
	 * @param parentMap
	 * @return
	 */
	public boolean bfsSearch(MapNode startNode, MapNode goalNode, Consumer<GeographicPoint> nodeSearched,
			HashMap<MapNode, MapNode> parentMap) {

		HashSet<MapNode> visited = new HashSet<>();
		Queue<MapNode> toExplore = new LinkedList<>();

		toExplore.add(startNode);
		boolean found = false;

		while (!toExplore.isEmpty()) {
			MapNode currentNode = toExplore.remove();
			if (currentNode == goalNode) {
				found = true;
				break;
			}
			Set<MapNode> neighbors = currentNode.getNeighbors();
			for (MapNode next : neighbors) {
				nodeSearched.accept(next.getLocation());
				if (!visited.contains(next)) {
					visited.add(next);
					parentMap.put(next, currentNode);
					toExplore.add(next);
				}
			}
		}

		return found;
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return dijkstra(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// TODO: Implement this method in WEEK 3

		// Hook for visualization. See writeup.
		// nodeSearched.accept(next.getLocation());

		// Setup - check validity of inputs
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		MapNode startNode = nodes.get(start);
		MapNode goalNode = nodes.get(goal);
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (goalNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}

		HashMap<MapNode, MapNode> parentMap = new HashMap<>();

		for (MapNode node : nodes.values()) {
			node.setWeight(Double.POSITIVE_INFINITY);
		}

		// printNodes();

		if (!dijkstraSearch(startNode, goalNode, nodeSearched, parentMap)) {
			System.out.println("No path exists");
			return null;
		}
		// reconstruct the path
		return reconstructPath(startNode, goalNode, parentMap);

	}

	public boolean dijkstraSearch(MapNode startNode, MapNode goalNode, Consumer<GeographicPoint> nodeSearched,
			HashMap<MapNode, MapNode> parentMap) {
		int removedNodeFromQueue = 0;
		HashSet<MapNode> visited = new HashSet<>();
		PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>();

		startNode.setWeight((Double) 0.0);
		toExplore.add(startNode);
		boolean found = false;

		while (!toExplore.isEmpty()) {
			MapNode currentNode = toExplore.remove();
			removedNodeFromQueue ++;
			currentNode.setWeight(currentNode.getWeight());
			if (!visited.contains(currentNode)) {
				visited.add(currentNode);
				if (currentNode == goalNode) {
					found = true;
					break;
				}
				Set<MapEdge> outEdges = currentNode.getEdges();
				for (MapEdge nextEdge : outEdges) {
					MapNode next = nextEdge.getEndNode();
					nodeSearched.accept(next.getLocation());
					if (!visited.contains(next)) {
						double currentDistance = currentNode.getWeight() + nextEdge.getDistance();
						if (currentDistance < next.getWeight()) {

							next.setWeight(currentDistance);
							parentMap.put(next, currentNode);
							toExplore.add(next);
						}
					}
				}
			}
		}
		System.out.println("dijkstraSearch removed node from queue :" + removedNodeFromQueue);
		return found;
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal, AstarHelper astarHelper) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return aStarSearch(start, goal, temp, astarHelper);
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched, AstarHelper astarHelper) {
		// TODO: Implement this method in WEEK 3

		// Hook for visualization. See writeup.
		// nodeSearched.accept(next.getLocation());

		// Setup - check validity of inputs
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		MapNode startNode = nodes.get(start);
		MapNode goalNode = nodes.get(goal);
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (goalNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}
		astarHelper.setGoalNode(goalNode);
		
		HashMap<MapNode, MapNode> parentMap = new HashMap<>();

		for (MapNode node : nodes.values()) {
			node.setWeight(Double.POSITIVE_INFINITY);
			node.setActualWeight(Double.POSITIVE_INFINITY);
		}

		// printNodes();

		if (!aStarSearchPath(startNode, goalNode, nodeSearched, parentMap, astarHelper)) {
			System.out.println("No path exists");
			return null;
		}
		// reconstruct the path
		return reconstructPath(startNode, goalNode, parentMap);

	}

	public boolean aStarSearchPath(MapNode startNode, MapNode goalNode, Consumer<GeographicPoint> nodeSearched,
			HashMap<MapNode, MapNode> parentMap, AstarHelper astarHelper) {
		int removedNodeFromQueue = 0;
		HashSet<MapNode> visited = new HashSet<>();
		PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>();

		startNode.setWeight((Double) 0.0);
		startNode.setActualWeight((Double) 0.0);
		toExplore.add(startNode);
		boolean found = false;

		while (!toExplore.isEmpty()) {
			MapNode currentNode = toExplore.remove();
			removedNodeFromQueue ++;
			if (!visited.contains(currentNode)) {
				visited.add(currentNode);
				if (currentNode == goalNode) {
					found = true;
					break;
				}
				Set<MapEdge> outEdges = currentNode.getEdges();
				for (MapEdge nextEdge : outEdges) {
					MapNode next = nextEdge.getEndNode();
					nodeSearched.accept(next.getLocation());
					if (!visited.contains(next)) {
						next.setActualWeight(currentNode.getActualWeight()  + astarHelper.computeEdgeWeight(nextEdge));
						double nextDistance = next.getActualWeight() + astarHelper.computeHeuristicWeight(next);
						
						if (nextDistance < next.getWeight()) {

							next.setWeight(nextDistance);
							parentMap.put(next, currentNode);
							toExplore.add(next);
						}
					}
				}
			}
		}
		System.out.println("AStarSearch removed node from queue :" + removedNodeFromQueue);
		return found;
	}

	public static void main(String[] args) {
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");

		// You can use this method for testing.

		/*
		 * Use this code in Week 3 End of Week Quiz MapGraph theMap = new
		 * MapGraph(); System.out.print("DONE. \nLoading the map...");
		 */
		GraphLoader.loadRoadMap("data/maps/hollywood_large.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(34.1078988, -118.3361954);
		GeographicPoint end = new GeographicPoint(34.1121014, -118.3061978);

		List<GeographicPoint> route = theMap.dijkstra(start, end);
//		AstarHelper astarHelper = new BasicAstarHelper();
		AstarHelper astarHelper = new RoadTypeAstarHelper(new BasicAstarHelper());
//		AstarHelper astarHelper = new WeightedAstarHelper(new BasicAstarHelper(), 1.5);
		
		List<GeographicPoint> route2 = theMap.aStarSearch(start, end, astarHelper);
		
		System.out.println("dijkstra " + route.size() + " aStarSearch "+ route2.size());

	}

}
