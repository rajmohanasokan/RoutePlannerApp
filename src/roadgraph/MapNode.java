package roadgraph;

import java.util.LinkedList;
import java.util.List;

import geography.GeographicPoint;

public class MapNode implements Comparable<MapNode> {
	GeographicPoint node = null;
	List<MapEdge> neighbors = null;
	double cost;
	
	MapNode(GeographicPoint node){
		this.node = node;
		neighbors = new LinkedList<MapEdge>();
		cost = Double.MAX_VALUE;
	}

	public double getCost() {
		return cost;
	}

	public void setCost(double cost) {
		this.cost = cost;
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

	@Override
	public int compareTo(MapNode other) {
		// TODO Auto-generated method stub
		return cost < other.getCost()? -1 : cost > other.getCost()? 1 : 0;
	}
	
}
