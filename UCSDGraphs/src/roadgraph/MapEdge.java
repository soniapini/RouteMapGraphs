package roadgraph;

/** @author Sonia Pini
 *  a class which represents a edge between nodes graph
 *  node graph are geographic location
 */
public class MapEdge {
	
	/** the two end points of the edge */
	private MapNode startNode;
	private MapNode endNode;
	
	/** the name of the road */
	private String streetName;
	/** the type of the road */
	private String streetType; //may be an Enum
	/** the length of the road segment in km */
	private double distance;
	
	static final double DEFAULT_LENGTH = 0.01;
	/** Create a new MapEdge object
	 * 
	 * @param roadName
	 * @param n1  The point at one end of the segment
	 * @param n2  The point at the other end of the segment
	 * 
	 */
	MapEdge(String roadName, MapNode n1, MapNode n2) 
	{
		this(roadName, "", n1, n2, DEFAULT_LENGTH);
	}
	
	MapEdge(String roadName, String roadType, MapNode n1, MapNode n2) 
	{
		this(roadName, roadType, n1, n2, DEFAULT_LENGTH);
	}
	
	MapEdge(String roadName, String roadType,
			MapNode n1, MapNode n2, double length) 
	{
		this.streetName = roadName;
		startNode = n1;
		endNode = n2;
		this.streetType = roadType;
		this.distance = length;
	}

	/**
	 * @return the start node of the edge
	 */
	public MapNode getStartNode() {
		return startNode;
	}
	
	/**
	 * 
	 * @return the end node of the edge
	 */
	public MapNode getEndNode() {
		return endNode;
	}

	/**
	 * 
	 * @return the street name associated with the edge
	 */
	public String getStreetName() {
		return streetName;
	}

	/**
	 * 
	 * @return the distance in Km assocaited with the edge
	 */
	public double getDistance() {
		return distance;
	}

	/**
	 * 
	 * @return the street type associated with the edge
	 */
	public String getStreetType() {
		return streetType;
	}
	
	/** given on point in the edge returns the other point */
	MapNode getOtherNode(MapNode node) {
		if(node.equals(startNode) ) {
			return endNode;
		} else if(node.equals(endNode)) {
			return startNode;
		}
		throw new IllegalArgumentException("Looking for " +
				"a point that is not in the edge");

	}

	@Override
	public String toString() {
		String toReturn = "[EDGE between ";
		toReturn += "\n\t" + startNode.getLocation();
		toReturn += "\n\t" + endNode.getLocation();
		toReturn += "\nRoad name: " + streetName + " Road type: " + streetType +
				" Segment length: " + String.format("%.3g", distance) + "km";
		
		return toReturn;	}
	
	

}
