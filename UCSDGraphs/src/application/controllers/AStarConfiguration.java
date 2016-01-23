package application.controllers;

/**
 * utility class used by User Interface to know which check-boxs are selected by the user
 * @author sonia pini
 *
 */
public class AStarConfiguration {
	private boolean weighted;
	private boolean fastestRoute;
	private boolean avoidRoute;
	
	public boolean isWeighted() {
		return weighted;
	}
	
	public void setWeighted(boolean weighted) {
		this.weighted = weighted;
	}
	
	public boolean isFastestRoute() {
		return fastestRoute;
	}
	
	public void setFastestRoute(boolean fastestRoute) {
		this.fastestRoute = fastestRoute;
	}

	public boolean isAvoidRoute() {
		return avoidRoute;
	}

	public void setAvoidRoute(boolean avoidRoute) {
		this.avoidRoute = avoidRoute;
	}
	
}
