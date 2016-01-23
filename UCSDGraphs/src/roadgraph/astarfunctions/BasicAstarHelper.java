package roadgraph.astarfunctions;

import roadgraph.MapEdge;
import roadgraph.MapNode;

/**
 * Default implementation of AstarHelper used by Astar algorithm to find the shortest path.
 * It uses distance as edge cost
 * @author sonia pini
 *
 */
public class BasicAstarHelper implements AstarHelper {
	/**
	 * contains the goal node 
	 */
	private MapNode goalNode;
	
	@Override
	/**
	 * compute the distance of the current edge
	 * @param edgeToAdd current edge
	 * @return the length in Km of the edgeToAdd
	 */
	public double computeEdgeWeight(MapEdge edgeToAdd) {
		return edgeToAdd.getDistance();
	}
	
	@Override
	/**
	 * compute the heuristic weight of the path between 
	 * the current node and the goal node
	 * @param inputNode the current node
	 * @return the linear distance between inputNode and goalNode
	 */
	public double computeHeuristicWeight(MapNode inputNode) {
		if(goalNode == null) {
			throw new IllegalStateException("To compute A* heristic the goal node is needed");
		}
		return inputNode.getLocation().distance(goalNode.getLocation());
	}
	
	@Override
	public final void setGoalNode(MapNode goalNode) {
		this.goalNode = goalNode;
	}

}