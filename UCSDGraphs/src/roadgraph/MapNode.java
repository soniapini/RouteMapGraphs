package roadgraph;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import geography.GeographicPoint;

/** @author Sonia Pini
 *  a class which represents a node inside the graph
 */
public class MapNode implements Comparable<MapNode> {
	/** the latitude and longitude of this node*/
	private GeographicPoint location;
	
	/** the list of outgoing edges from this node */
	private HashSet<MapEdge> edges;
	
	/** the predicted distance of this node */
	private double weight;
	
	/** the actual distance of this node from start */
	private double actualWeight;

	
	/** 
	 * Create a new MapNode with outgoing edges 
	 */
	MapNode(GeographicPoint location, HashSet<MapEdge> edges) {
		super();
		this.location = location;
		this.edges = edges;
		setWeight(0.0);
		setActualWeight(0.0);
		
	}
	
	/** 
	 * Create a new MapNode without outgoing edges 
	 */
	 MapNode(GeographicPoint location) {
		super();
		this.location = location;
		edges = new HashSet<MapEdge>();
		setWeight(0.0);
		setActualWeight(0.0);

	}

	/**
	 * Get the geographic location of a MapNode 
	 * @return The MapNode as GeographicPoints
	 */
	public GeographicPoint getLocation() {
		return location;
	}

	/**
	 * Get the outgoing edges from the node
	 * @return the outgoing edges from the node as MapEdge
	 */
	public HashSet<MapEdge> getEdges() {
		return edges;
	}
	
	/**
	 * get node weight (predicted)
	 * @return
	 */
	public double getWeight() {
		return weight;
	}

	/**
	 * set node weight (predicted)
	 * @param weight
	 */
	public void setWeight(double weight) {
		this.weight = weight;
	}
	
	public double getActualWeight() {
		return actualWeight;
	}

	public void setActualWeight(double actualWeight) {
		this.actualWeight = actualWeight;
	}

	/**
	 * Get the number of outgoing edges from the node
	 * @return The number of outgoing edges from the node.
	 */
	public int getNumberOfEdges() {
		return edges.size();
	}

	/**
	 * Add a new outgoing edge from the node
	 * @param edge the outgoing edge to be added to the node
	 */
	public void addEdge(MapEdge edge) {
		edges.add(edge);
	}
	
	/**
	 * Add a set of new outgoing edge from the node
	 * @param edges the outgoing edges to be added to the node
	 */
	public void addEdges(List<MapEdge> edges) {
		this.edges.addAll(edges);
	}
	
	/** Return the neighbors of this MapNode */
	Set<MapNode> getNeighbors() {
		Set<MapNode> neighbors = new  HashSet<MapNode>();
		for (MapEdge edge: edges) {
			neighbors.add(edge.getOtherNode(this));
		}
		
		return neighbors;
	}


	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((location == null) ? 0 : location.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		MapNode other = (MapNode) obj;
		if (location == null) {
			if (other.location != null)
				return false;
		} else if (!location.equals(other.location))
			return false;
		return true;
	}

//	@Override
//	public int compareTo(Object o) {
//		MapNode node = (MapNode) o;
//		return ((Double) this.getDistance()).compareTo((Double) node.getDistance());
//	}

	@Override
	public String toString() {
		return "MapNode [location=" + location + ", edges=" + edges + ", distance=" + weight + ", actualDistance="
				+ actualWeight + "]";
	}

	@Override
	public int compareTo(MapNode other) {
		return ((Double) this.getWeight()).compareTo((Double) other.getWeight());
	}

	

	
	
}
