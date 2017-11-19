package roadgraph;

import java.util.LinkedList;
import java.util.List;

import geography.GeographicPoint;

public class MapNode {
	GeographicPoint node = null;
	List<MapEdge> neighbors = null;
	
	MapNode(GeographicPoint node){
		this.node = node;
		neighbors = new LinkedList<MapEdge>();
	}

	public GeographicPoint getNode() {
		return node;
	}

	public void setNode(GeographicPoint node) {
		this.node = node;
	}

	public List<MapEdge> getNeighbors() {
		return neighbors;
	}

	public void addNeighbor(MapEdge neighbor) {
		this.neighbors.add(neighbor);
	}
	
}
