/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
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
	//Done: Add your member variables here in WEEK 3
	private Map<GeographicPoint, List<Edge>> adjList ;
	private int numOfVertices;
	private int numOfEdgs;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// Done: Implement in this constructor in WEEK 3
		numOfVertices = 0;
		numOfEdgs = 0;
		adjList = new HashMap<GeographicPoint, List<Edge>>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//Done: Implement this method in WEEK 3
		return numOfVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//Done: Implement this method in WEEK 3
		Set<GeographicPoint> vertices = new HashSet<GeographicPoint>();
		for(GeographicPoint vertex : adjList.keySet()) {
			vertices.add(vertex);
		}
		return vertices;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//Done: Implement this method in WEEK 3
		return numOfEdgs;
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
		// Done: Implement this method in WEEK 3
		List<Edge> neighbors = new ArrayList<Edge>();
		adjList.put(location, neighbors);
		numOfVertices++;
		return true;
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

		//Done: Implement this method in WEEK 3
		if(adjList.get(from) == null || adjList.get(to) == null) {
			throw new IllegalArgumentException();
		}
		numOfEdgs++;
		adjList.get(from).add(new Edge(to, roadName, roadType, length));
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
	
	/** Helper function to Find the path from start to goal using breadth first search
	 * 
	 * @param toExplore Queue
	 * @param visited   HashSet to store the visited nodes
	 * @param parentMap A Map with parent nodes
	 * @param nodeSearched A hook for visualization.
	 * @return true if found, false if not;
	 */
	
	private boolean bfsSearch(GeographicPoint start, GeographicPoint goal,
							  Map<GeographicPoint, GeographicPoint> parentMap, 
							  Consumer<GeographicPoint> nodeSearched)
	{
		boolean found = false;
		
		Queue<GeographicPoint> toExplore = new LinkedList<GeographicPoint>();
        Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
        
        toExplore.add(start);
		while(!toExplore.isEmpty()) {
			GeographicPoint from = toExplore.remove();
			if (goal.equals(from)) {
				found = true;
				nodeSearched.accept(from);
				break;
			}
			for(Edge road : adjList.get(from)) {
				GeographicPoint to = road.getEnd();
				if(!visited.contains(to)) {
					parentMap.put(to, from);
					visited.add(to);
					toExplore.add(to);
					nodeSearched.accept(to);
				}
			}
		}
		return found;
	}
	
	
	/** Construct the path from a given parent map
	 * 
	 * @param  start The starting location
	 * @param  goal The goal location
	 * @param  parentMap A Map with parent nodes
	 * @return path from start to goal (including both start and goal).
	 */
	private List<GeographicPoint> constructPath(GeographicPoint start, GeographicPoint goal,
												Map<GeographicPoint, GeographicPoint> parentMap) 
	{
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		path.addFirst(goal);
		while(start != goal) {
			goal = parentMap.get(goal);
			path.addFirst(goal);
		}
		return path;
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
		// Done: Implement this method in WEEK 3
		
        Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
        if(bfsSearch(start, goal, parentMap, nodeSearched)) {
        	return constructPath(start, goal, parentMap);
        }
		return null;
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
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
			  							  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// Done: Implement this method in WEEK 4
		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		if(dijkstraSearch(start, goal, parentMap, nodeSearched)) {
        	return constructPath(start, goal, parentMap);
        }
		return null;
		
	}
	
	public boolean dijkstraSearch(GeographicPoint start, GeographicPoint goal,
								  Map<GeographicPoint, GeographicPoint> parentMap,  
								  Consumer<GeographicPoint> nodeSearched)
	{
		PriorityQueue<PointDistance> toExplore = 
				new PriorityQueue<PointDistance>();
        Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
       
        boolean found = false;
        double currDistance = 0;
        int num = 0;
        Map<GeographicPoint, Double> distanceMap = new HashMap<GeographicPoint, Double>(); 
        for(GeographicPoint point : adjList.keySet()) {
        	distanceMap.put(point, Double.POSITIVE_INFINITY);
        }
        
        toExplore.add(new PointDistance(start, 0));
        distanceMap.replace(start, 0.0);
		while(!toExplore.isEmpty()) {
			PointDistance curr = toExplore.poll();
			GeographicPoint from = curr.getPoint();
			num++;
			if (!visited.contains(from)) {
				visited.add(from);
				
				if (goal.equals(from)) {
					found = true;
					nodeSearched.accept(from);
					break;
				}
				for(Edge road : adjList.get(from)) {
					GeographicPoint to = road.getEnd();
					currDistance = distanceMap.get(from) + road.getLength();
					if (currDistance < distanceMap.get(to)) {
						distanceMap.replace(to, currDistance);
						if (parentMap.replace(to, from) == null) {
							parentMap.put(to, from);
						}
						toExplore.add(new PointDistance(to, currDistance));
						nodeSearched.accept(to);
					}
				}
			}
		}

		System.out.println("dijkstra  " + num);
		return found;
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
		// Done: Implement this method in WEEK 4
		
		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		PriorityQueue<PointDistance> toExplore = 
				new PriorityQueue<PointDistance>(numOfVertices);
        Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
       
        boolean found = false;
        double currDistance = 0;
        double estimatedDistance = 0;
       
        int num = 0;
        Map<GeographicPoint, Double> distanceMap = new HashMap<GeographicPoint, Double>();
        for(GeographicPoint point : adjList.keySet()) {
        	distanceMap.put(point, Double.MAX_VALUE);
        }
        
        toExplore.add(new PointDistance(start, 0));
        distanceMap.replace(start, 0.0);
		
        while(!toExplore.isEmpty()) {
			PointDistance curr = toExplore.poll();
			GeographicPoint from = curr.getPoint();
			num++;
			if (!visited.contains(from)) {
				visited.add(from);
				
				if (goal.equals(from)) {
					found = true;
					nodeSearched.accept(from);
					break;
				}
				for(Edge road : adjList.get(from)) {
					GeographicPoint to = road.getEnd();
					currDistance = distanceMap.get(from) + road.getLength();
					estimatedDistance = currDistance + to.distance(goal);
					if (currDistance < distanceMap.get(to)) {
						distanceMap.replace(to, currDistance);
						if (parentMap.replace(to, from) == null) {
							parentMap.put(to, from);
						}
						toExplore.add(new PointDistance(to, estimatedDistance));
						nodeSearched.accept(to);
					}
				}
			}
		}
		System.out.println("Astar  " + num);
		if(found) {
        	return constructPath(start, goal, parentMap);
        }
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
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
		
		
		
		/* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		
		
	}
	
}
