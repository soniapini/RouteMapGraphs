package roadgraph.astarfunctions;

import roadgraph.MapEdge;

/**
 * Concrete A-star helper decorator
 * @author sonia pini
 *
 */
public class TryToAvoidRoadTypeAstarHelper extends AstarHelperDecorator {
	private String avoidRoad;
	
	public TryToAvoidRoadTypeAstarHelper(AstarHelper astarHelper, String avoidRoad) {
		super(astarHelper);
		this.avoidRoad = avoidRoad;
	}
	
	@Override
	/**
	 *  It try to found a path without edge with streetType = motorway. It is possible to obtain this result by incrementing the weight of the edge with streetType = motorway. This decorator overrides only the computeEdgeWeight, and it multiply by 20 the length of edge  with streetType = motorway.
	 * compute the weight of the current edge
	 * @param edgeToAdd current edge
	 * @return the weight of the edgeToAdd if streetType != avoidRoad, otherwise the weight of the edgeToAdd * 20
	 */
	public double computeEdgeWeight(MapEdge edgeToAdd) {
		return edgeToAdd.getStreetType().equals(avoidRoad) ? super.computeEdgeWeight(edgeToAdd) * 20D : super.computeEdgeWeight(edgeToAdd);
	}

}
