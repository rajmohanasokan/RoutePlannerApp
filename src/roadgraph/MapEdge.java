package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	MapNode start;
	MapNode end;
	String roadName;
	String roadType; 
	double length;
	
	MapEdge(MapNode start, MapNode end,
			String roadName, String roadType, double length){
		this.start = start;
		this.end = end;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
	}

	public MapNode getStart() {
		return start;
	}

	public void setStart(MapNode start) {
		this.start = start;
	}

	public MapNode getEnd() {
		return end;
	}

	public void setEnd(MapNode end) {
		this.end = end;
	}

	public String getRoadName() {
		return roadName;
	}

	public void setRoadName(String roadName) {
		this.roadName = roadName;
	}

	public String getRoadType() {
		return roadType;
	}

	public void setRoadType(String roadType) {
		this.roadType = roadType;
	}

	public double getLength() {
		return length;
	}

	public void setLength(double length) {
		this.length = length;
	}
	
}
