package roadgraph.astarfunctions;

import java.security.InvalidParameterException;

import roadgraph.MapNode;

/**
 * Concrete A-star helper decorator
 * @author sonia pini
 *
 */
public class WeightedAstarHelper extends AstarHelperDecorator {
	private final double weight;
	
	public WeightedAstarHelper(AstarHelper astarHelper, double weight) {
		super(astarHelper);
		
		if(weight < 1) {
			throw new InvalidParameterException("The weight must be greater than 1");
		}
		this.weight = weight;
	}

	@Override
	 /* * 
	 * compute the ε-admissible heuristic weight of the path between 
	 * the current node and the goal node
	 * @param inputNode the current node
	 * @return the heuristic weight * weight where weight > 1
	 */
	public double computeHeuristicWeight(MapNode inputNode) {
		return weight * super.computeHeuristicWeight(inputNode);
	}
}