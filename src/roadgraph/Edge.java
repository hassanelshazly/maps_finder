package roadgraph;

import geography.GeographicPoint;

public class Edge {
	
	private GeographicPoint endPoint;
	private String roadName;
	private String roadType; 
	private double length;
	
	public Edge(GeographicPoint endPoint, String roadName, String roadType, double length) {
		this.endPoint = endPoint;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
	}
	
	public Edge(GeographicPoint endPoint) {
		this.endPoint = endPoint;
		this.roadName = "No Set";
		this.roadType = "No Set";
		this.length = 0;
	}
	
	public GeographicPoint getEnd() {
		return endPoint;
	}
	
	public double getLength() {
		return length;
	}
	
}