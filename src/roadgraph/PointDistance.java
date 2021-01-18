package roadgraph;

import geography.GeographicPoint;

public class PointDistance implements Comparable<PointDistance> {
	private GeographicPoint point;
	private double distance;
	
	public PointDistance(GeographicPoint point, double distance) {
		this.point = point;
		this.distance = distance;
	}
	
	public GeographicPoint getPoint() {
		return point;
	}
	
	public double getDistance() {
		return distance;
	}
	
	public void setPoint(GeographicPoint point) {
		this.point = point;
	}
	
	public void serDistance(double distance) {
		this.distance = distance;
	}

	@Override
	public int compareTo(PointDistance point) {
		if (this.getDistance() > point.getDistance()) {
			return 1;
		} else if (this.getDistance() < point.getDistance()) {
			return -1;
		} else {
			return 0;
		}
	}
}