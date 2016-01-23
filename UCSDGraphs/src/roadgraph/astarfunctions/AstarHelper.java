package roadgraph.astarfunctions;

import roadgraph.MapEdge;
import roadgraph.MapNode;

/**
 * Helper class of Astar algorithm
 * @author sonia pini
 *
 */
public interface AstarHelper {
	/**
	 * compute the weight of the current edge
	 * @param edgeToAdd current edge
	 * @return the weight of the edgeToAdd
	 */
	double computeEdgeWeight(MapEdge edgeToAdd);
	
	/**
	 * compute the heuristic weight of the path between 
	 * the current node and the goal node
	 * @param inputNode the current node
	 * @return the heuristic weight of the path between inputNode and goalNode
	 */
	double computeHeuristicWeight(MapNode inputNode);
	
	/**
	 * set the goal node of the algorithm
	 * @param goalNode the target node
	 */
	void setGoalNode(MapNode goalNode);
}
