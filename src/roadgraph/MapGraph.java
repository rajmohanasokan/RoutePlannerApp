/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
	//Set of vertices in the graph
	Set<GeographicPoint> vertices = null;
	Set<GeographicPoint> reverseVertices = null;
	//List of edges of type MapEdge
	List<MapEdge> edges = null;
	List<MapEdge> reverseEdges = null;
	//A collection mapping geographic points to objects of type MapNode
	Map<GeographicPoint, MapNode> locationNode = null;
	Map<GeographicPoint, MapNode> reverseLocationNode = null;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		vertices = new HashSet<GeographicPoint>();
		reverseVertices = new HashSet<GeographicPoint>();
		edges = new LinkedList<MapEdge>();
		reverseEdges = new LinkedList<MapEdge>();
		locationNode = new HashMap<GeographicPoint, MapNode>();
		reverseLocationNode = new HashMap<GeographicPoint, MapNode>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return vertices.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		
		return vertices;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return edges.size();
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3		
		if(!vertices.contains(location)){
			vertices.add(location);
			//Create a MapNode object, node, using the GeographicPoint object, location
			MapNode node = new MapNode(location);
			MapNode revNode = new MapNode(location);
			//Map location to node
			locationNode.put(location, node);
			reverseLocationNode.put(location,  revNode);
			return true;
		}
		return false;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 3
		if(!vertices.contains(from) || !vertices.contains(to) ||
				roadName == null || roadType == null){
			throw new IllegalArgumentException();
		}
		MapNode fromNode = locationNode.get(from);
		MapNode toNode = locationNode.get(to);
		//Create a MapEdge object, edge, using the MapNode objects, fromNode and toNode
		MapEdge edge = new MapEdge(fromNode, toNode, roadName, roadType, length);
		//Add edge to the list of edges
		edges.add(edge);
		//Add edge as a neighbor in fromNode, so that fromNode can access toNode using this edge object
		fromNode.addNeighbor(edge);
		
		MapNode revFromNode = reverseLocationNode.get(to);
		MapNode revToNode = reverseLocationNode.get(from);
		MapEdge revEdge = new MapEdge(revFromNode, revToNode, roadName, roadType, length);
		reverseEdges.add(revEdge);
		revFromNode.addNeighbor(revEdge);
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		Map<MapNode, MapNode> parent = new HashMap<MapNode, MapNode>();
		//Maintain a set, visited, of type MapNode to track the MapNode objects that have been explored
		Set<MapNode> visited = new HashSet<MapNode>();
		MapNode startNode = locationNode.get(start);
		MapNode goalNode = locationNode.get(goal);
		Queue<MapNode> bfsQueue = new LinkedList<MapNode>();
		bfsQueue.offer(startNode);
		while(!bfsQueue.isEmpty()){
			MapNode node = bfsQueue.poll();
			nodeSearched.accept(node.getNode());
			if(node == goalNode){
				return constructPath(parent, goalNode, startNode);
			}else{
				for(MapEdge edge : node.getNeighbors()){
					if(!visited.contains(edge.getEnd())){
						bfsQueue.offer(edge.getEnd());
						parent.put(edge.getEnd(), node);
					}					
				}
			}
			visited.add(node);
		}
		return null;
	}
	

	private List<GeographicPoint> constructPath(Map<MapNode, MapNode> parent, MapNode goalNode, MapNode startNode) {
		// TODO Auto-generated method stub
		List<GeographicPoint> path = new LinkedList<GeographicPoint>();
		//Start from goalNode, traverse through the map and reach the startNode, while also capturing the nodes in this traversal.
		path.add(goalNode.getNode());
		while(goalNode != startNode){
			goalNode = parent.get(goalNode);
			path.add(goalNode.getNode());
		}		
		Collections.reverse(path);
		return path;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	public List<GeographicPoint> bidijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return bidijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bidijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		Map<MapNode, Double> fwdDistance = new HashMap<MapNode, Double>();
		Map<MapNode, Double> bwdDistance = new HashMap<MapNode, Double>();
		
		Map<MapNode, MapNode> parent = new HashMap<MapNode, MapNode>();
		Map<MapNode, MapNode> reverseParent = new HashMap<MapNode, MapNode>();
		//Maintain a set, visited, of type MapNode to track the MapNode objects that have been explored
		Set<MapNode> visited = new HashSet<MapNode>();
		Set<MapNode> revVisited = new HashSet<MapNode>();
		MapNode startNode = locationNode.get(start);
		MapNode revStartNode = reverseLocationNode.get(goal);
		startNode.setCost(0.0);
		fwdDistance.put(startNode, 0.0);
		revStartNode.setCost(0.0);
		bwdDistance.put(revStartNode, 0.0);
//		MapNode goalNode = locationNode.get(goal);
//		MapNode revGoalNode = reverseLocationNode.get(start);
		Queue<MapNode> dkQueue = new PriorityQueue<MapNode>();
		Queue<MapNode> revDkQueue = new PriorityQueue<MapNode>();
		dkQueue.offer(startNode);
		revDkQueue.offer(revStartNode);
		System.out.println("The following nodes were visited in Dijkstra");
		do{
			MapNode currNode = dkQueue.poll();
			if(visited.contains(currNode)){
				continue;
			}
			else{
				nodeSearched.accept(currNode.getNode());
				System.out.println(currNode.getNode().getX() + ", " + currNode.getNode().getY());
				visited.add(currNode);
			}
			traverseEdgesOfCurrentNode(currNode, fwdDistance, parent, dkQueue);
			if(revVisited.contains(reverseLocationNode.get(currNode.getNode()))){
				return bidirectionalPath(parent, reverseParent, visited, revVisited, fwdDistance, bwdDistance, start, goal);
			}
			
			// Reverse Dijkstra			
			currNode = revDkQueue.poll();
			if(revVisited.contains(currNode)){
				continue;
			}
			else{
				nodeSearched.accept(currNode.getNode());
				System.out.println(currNode.getNode().getX() + ", " + currNode.getNode().getY());
				revVisited.add(currNode);
			}
			traverseEdgesOfCurrentNode(currNode, bwdDistance, reverseParent, revDkQueue);			
			if(visited.contains(locationNode.get(currNode.getNode()))){
				return bidirectionalPath(parent, reverseParent, visited, revVisited, fwdDistance, bwdDistance, start, goal);
			}
			
		}while(true);
	}
	
	private void traverseEdgesOfCurrentNode(MapNode currNode, Map<MapNode, Double> fwdDistance,
			Map<MapNode, MapNode> parent, Queue<MapNode> dkQueue) {
		// TODO Auto-generated method stub
		for(MapEdge edge : currNode.getNeighbors()){
			double currLength = currNode.getCost() + edge.getLength();
			MapNode destNode = edge.getEnd();
			if(currLength < destNode.getCost()){
				destNode.setCost(currLength);
				fwdDistance.put(destNode, currLength);
				dkQueue.offer(destNode);
				parent.put(destNode, currNode);
			}
		}
	}

	private List<GeographicPoint> bidirectionalPath(Map<MapNode, MapNode> parent, Map<MapNode, MapNode> reverseParent,
			Set<MapNode> visited, Set<MapNode> revVisited, Map<MapNode, Double> fwdDistance,
			Map<MapNode, Double> bwdDistance, GeographicPoint start, GeographicPoint goal) {
		// TODO Auto-generated method stub		
		MapNode bestNode = null;
		List<GeographicPoint> path = new LinkedList<GeographicPoint>();
		System.out.println("Entering path reconstruction");
		bestNode = findMeetingNode(visited, fwdDistance, bwdDistance);		
		if(bestNode != null){
			MapNode tempNode = bestNode;
			MapNode startNode = locationNode.get(start);
			MapNode goalNode = reverseLocationNode.get(goal);			
			//Start from goalNode, traverse through the map and reach the startNode, while also capturing the nodes in this traversal.			
			reconstructPathFromBidirectionalDijkstra(parent, tempNode, startNode, path);					
			Collections.reverse(path);
			tempNode = reverseLocationNode.get(bestNode.getNode());
			tempNode = reverseParent.get(tempNode);
			reconstructPathFromBidirectionalDijkstra(reverseParent, tempNode, goalNode, path);			
		}
		return path;
	}


	private void reconstructPathFromBidirectionalDijkstra(Map<MapNode, MapNode> parent, MapNode tempNode,
			MapNode targetNode, List<GeographicPoint> path) {
		// TODO Auto-generated method stub
		while(tempNode != targetNode){
			path.add(tempNode.getNode());
			tempNode = parent.get(tempNode);			
		}
		path.add(tempNode.getNode());
	}

	private MapNode findMeetingNode(Set<MapNode> visited, Map<MapNode, Double> fwdDistance,
			Map<MapNode, Double> bwdDistance) {
		// TODO Auto-generated method stub
		Double bestDistance = Double.MAX_VALUE;
		Double localDistance = 0.0;
		MapNode bestNode = null;
		for(MapNode node : visited){
			MapNode revNode = reverseLocationNode.get(node.getNode());
			localDistance = fwdDistance.get(node) + (bwdDistance.containsKey(revNode)?
															bwdDistance.get(revNode) : Double.MAX_VALUE);
			if(localDistance < bestDistance){
				bestDistance = localDistance;
				bestNode = node;
			}
		}
		return bestNode;
	}

	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// TODO: Implement this method in WEEK 4

		// Hook for visualization. See writeup.
		// nodeSearched.accept(next.getLocation());
		Map<MapNode, MapNode> parent = new HashMap<MapNode, MapNode>();
		// Maintain a set, visited, of type MapNode to track the MapNode objects
		// that have been explored
		Set<MapNode> visited = new HashSet<MapNode>();
		MapNode startNode = locationNode.get(start);
		startNode.setCost(0);
		MapNode goalNode = locationNode.get(goal);
		Queue<MapNode> dkQueue = new PriorityQueue<MapNode>();
		dkQueue.offer(startNode);
		System.out.println("The following nodes were visited in Dijkstra");
		while (!dkQueue.isEmpty()) {
			MapNode currNode = dkQueue.poll();
			if (visited.contains(currNode)) {
				continue;
			} else {
				nodeSearched.accept(currNode.getNode());
				System.out.println(currNode.getNode().getX() + ", " + currNode.getNode().getY());
				visited.add(currNode);
			}
			if (currNode == goalNode) {
				return constructPath(parent, goalNode, startNode);
			} else {
				for (MapEdge edge : currNode.getNeighbors()) {
					double currLength = currNode.getCost() + edge.getLength();
					MapNode destNode = edge.getEnd();
					if (currLength < destNode.getCost()) {
						destNode.setCost(currLength);
						dkQueue.offer(destNode);
						parent.put(destNode, currNode);
					}
				}
			}
		}
		return null;
	}
	

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		Map<MapNode, MapNode> parent = new HashMap<MapNode, MapNode>();
		//Maintain a set, visited, of type MapNode to track the MapNode objects that have been explored
		Set<MapNode> visited = new HashSet<MapNode>();
		MapNode startNode = locationNode.get(start);
		startNode.setCost(start.distance(goal));
		MapNode goalNode = locationNode.get(goal);		
		Queue<MapNode> dkQueue = new PriorityQueue<MapNode>();
		dkQueue.offer(startNode);
		System.out.println("The following nodes were visited in A*");
		while(!dkQueue.isEmpty()){
			MapNode currNode = dkQueue.poll();
			if(visited.contains(currNode)){
				continue;
			}
			else{
				nodeSearched.accept(currNode.getNode());
				System.out.println(currNode.getNode().getX() + ", " + currNode.getNode().getY());
				visited.add(currNode);
			}
			if(currNode == goalNode){							
				return constructPath(parent, goalNode, startNode);
			}
			else{
				for(MapEdge edge : currNode.getNeighbors()){
					MapNode destNode = edge.getEnd();
					double currLength = currNode.getCost() - currNode.getNode().distance(goal) + edge.getLength() + destNode.getNode().distance(goal);					
					if(currLength < destNode.getCost()){
						destNode.setCost(currLength);
						dkQueue.offer(destNode);
						parent.put(destNode, currNode);
					}
				}
			}
		}
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		/*List<GeographicPoint> testroute = firstMap.aStarSearch(testStart, testEnd);
		System.out.println("The route is as follows");
		for(GeographicPoint location : testroute){
			System.out.println(location.getX() + ", " + location.getY());
		}
		System.out.println("DONE.");
		testroute.clear();*/
		List<GeographicPoint> testroute = firstMap.bidijkstra(testStart, testEnd);
		System.out.println("The route is as follows");
		for(GeographicPoint location : testroute){
			System.out.println(location.getX() + ", " + location.getY());
		}
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
