package roadgraph.astarfunctions;

import roadgraph.MapEdge;
import roadgraph.MapNode;
/**
 * Abstract Decorator class
 * @author sonia pini
 *
 */
public abstract class AstarHelperDecorator implements AstarHelper {
	private final AstarHelper internalHelper;
	
	public AstarHelperDecorator(AstarHelper astarHelper) {
		if(astarHelper == null) {
			throw new NullPointerException("astarHelper cannot be null");				
		}
		
		this.internalHelper = astarHelper;
	}

	@Override
	public double computeEdgeWeight(MapEdge edgeToAdd) {
		return internalHelper.computeEdgeWeight(edgeToAdd);
	}

	@Override
	public double computeHeuristicWeight(MapNode inputNode) {
		return internalHelper.computeHeuristicWeight(inputNode);
	}
	
	@Override
	public final void setGoalNode(MapNode goalNode) {
		internalHelper.setGoalNode(goalNode);
	}
}