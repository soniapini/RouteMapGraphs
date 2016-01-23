package roadgraph.astarfunctions;

import java.util.HashMap;
import java.util.Map;

import roadgraph.MapEdge;
import roadgraph.MapNode;

/**
 * Concrete A-star helper decorator
 * @author sonia pini
 *
 */
public class RoadTypeAstarHelper extends AstarHelperDecorator {

	private Map<String, Double> speedLimits;
	private double defaultSpeed;
	private String heuristicRoadType;
	
	public RoadTypeAstarHelper(AstarHelper astarHelper) {
		super(astarHelper);
		
		speedLimits = new HashMap<>();
		setDefaultSpeedLimits();
		
		defaultSpeed = 40D;
		heuristicRoadType = "motorway";
	}

	private void setDefaultSpeedLimits() {
		speedLimits.put("motorway", 65D);
		speedLimits.put("primary", 55D);
		speedLimits.put("secondary", 55D);
		speedLimits.put("tertiary", 55D);
		speedLimits.put("unclassified", 55D);
		speedLimits.put("residential", 30D);
		speedLimits.put("living_street", 20D);
		speedLimits.put("motorway_link", 30D);
		speedLimits.put("secondary_link", 20D);
	}

	/**
	 * 
	 * @param roadType street type
	 * @return the speed limit for this type of street
	 */
	private double getSpeedFromRoadType(String roadType) {
		Double speed = speedLimits.get(roadType);
		if(speed == null) {
			return defaultSpeed;
		}
		
		return speed;
	}
	
	/**
	 * compute the time needed to pass through the edge
	 * @param distance length in Km
	 * @param streetType type of the street
	 * @return the time needed
	 */
	private double computeNeededTime(double distance, String streetType) {
		return distance / getSpeedFromRoadType(streetType);
	}
	
	@Override
	/**
	 * compute the time of the current edge
	 * @param edgeToAdd current edge
	 * @return the time in h of the edgeToAdd
	 */
	public double computeEdgeWeight(MapEdge edgeToAdd) {
		return computeNeededTime(
				super.computeEdgeWeight(edgeToAdd), 
				edgeToAdd.getStreetType());
	}
	
	@Override
	/**
	 * compute the heuristic weight of the path between 
	 * the current node and the goal node
	 * @param inputNode the current node
	 * @return the time needed to go from inputNode to the goalNode
	 *  with the assumption that the connection be the fastest, that is the motorway. 
	 */
	public double computeHeuristicWeight(MapNode inputNode) {
		return computeNeededTime(
				super.computeHeuristicWeight(inputNode), 
				heuristicRoadType);
	}
}