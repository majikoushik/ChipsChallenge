package com.chipschallenge.agent;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Stack;

import com.chipschallenge.environment.Action;
import com.chipschallenge.environment.Environment;
import com.chipschallenge.environment.Position;
import com.chipschallenge.environment.TileStatus;

/**
 * Represents a planning agent within an environment modeled after
 * the Chip's Challenge Windows 95 game. This agent must develop a
 * plan for navigating the environment to collect chips and keys
 * in order to reach the environment's portal (goal condition).
 * 
 * Chip Mccallahan is AI planning agent who navigate the environment to collect chips scattered across the
 * map. In order to reach the portal (goal condition), the agent must collect
 * all the chips first. In order to do this, the agent will also need to collect
 * assorted keys that can be used to unlock doors blocking some of the chips.
 * 
 * Map difficulties increase by the number of subgoals that the agent must
 * complete.
 */

public class Robot {
	private Environment env;

	/**
	 * Initializes a Robot on a specific tile in the environment.
	 * 
	 * @param env - The Environment
	 */
	public Robot(Environment env) {
		this.env = env;
	}

	Queue<Action> foundActionList = new LinkedList<Action>();

	/** To track all the visited nodes, in terms of path **/
	Map<Node, Node> cameFromPositionList = new HashMap<>();
	Stack<GoalItem> goalStackList = new Stack<GoalItem>();

	/** Allows to run the a method only once in the get action method call **/
	boolean planExecutionOn = false;
	boolean initialGoalAddCompleted = false;

	/**
	 * The method called by Environment to retrieve the agent's actions.
	 * 
	 * @return should return a single Action from the Action class.
	 *         - Action.DO_NOTHING
	 *         - Action.MOVE_UP
	 *         - Action.MOVE_DOWN
	 *         - Action.MOVE_LEFT
	 *         - Action.MOVE_RIGHT
	 */
	public Action getAction() {
		Action actionReturned = Action.DO_NOTHING;
		//This code will only execute once
		
		if (!initialGoalAddCompleted) {
			Map<TileStatus, ArrayList<Position>> envPositions = env.getEnvironmentPositions();
			Position goal = envPositions.get(TileStatus.GOAL).get(0);
			Position doorGoal = envPositions.get(TileStatus.DOOR_GOAL).get(0);

			goalStackList.push(new GoalItem(goal, TileStatus.GOAL));
			goalStackList.push(new GoalItem(doorGoal, TileStatus.DOOR_GOAL));
			addNewGoalByProximity(TileStatus.CHIP);
			initialGoalAddCompleted = true;
		}
		
		if (!planExecutionOn) {			
			if(foundActionList.isEmpty())
			{
				planExecutionOn = true;
				Map<TileStatus, ArrayList<Position>> envPositions = env.getEnvironmentPositions();
				ArrayList<Position> chips = envPositions.get(TileStatus.CHIP);
				//goalStack having 2 goal and there are chips left in the environment, then add the chip as goal
				if (goalStackList.size() == 2 && chips.size() > 0) {
					addNewGoalByProximity(TileStatus.CHIP);
				}
				while (!goalStackList.isEmpty()) {
					GoalItem goal = goalStackList.peek();
					AStarGoalPathFinding(env.getRobotPosition(this), goal);
					if (!foundActionList.isEmpty()) {
						goalStackList.pop();
						planExecutionOn = false;
						break;
					}
				}
			}
			else
			{
				actionReturned = foundActionList.poll();
			}			
		}
		return actionReturned;
	}

	private void addNewGoalByProximity(TileStatus goalType) {

		Position sourcePosition = env.getRobotPosition(this);
		PriorityQueue<Node> genericPriorityQueue = new PriorityQueue<>();
		Map<TileStatus, ArrayList<Position>> envPositions = env.getEnvironmentPositions();
		ArrayList<Position> goalList = null;
		if (goalType == TileStatus.CHIP) {
			goalList = envPositions.get(TileStatus.CHIP);
		} else if (goalType == TileStatus.KEY_BLUE) {
			goalList = envPositions.get(TileStatus.KEY_BLUE);
		} else if (goalType == TileStatus.KEY_GREEN) {
			goalList = envPositions.get(TileStatus.KEY_GREEN);
		} else if (goalType == TileStatus.KEY_RED) {
			goalList = envPositions.get(TileStatus.KEY_RED);
		} else if (goalType == TileStatus.KEY_YELLOW) {
			goalList = envPositions.get(TileStatus.KEY_YELLOW);
		}
		for (int i = 0; i < goalList.size(); i++) {
			Position destinationPosition = goalList.get(i);
			if (sourcePosition != destinationPosition) {
				genericPriorityQueue.add(new Node(destinationPosition, heuristic(sourcePosition, destinationPosition), ""));
			}

		}

		if (!genericPriorityQueue.isEmpty()) {
			goalStackList.push(new GoalItem(genericPriorityQueue.poll().position, goalType));
		}

	}

	private boolean isSubGoalAdded(TileStatus sourceTileStatus, TileStatus currentTileStatus) {
		boolean newGoalAdded = false;
		Map<TileStatus, ArrayList<Position>> envPositions = env.getEnvironmentPositions();
		if (currentTileStatus == TileStatus.DOOR_BLUE
				&& envPositions.get(TileStatus.KEY_BLUE).size() == envPositions.get(TileStatus.DOOR_BLUE).size() &&
				sourceTileStatus != TileStatus.KEY_BLUE) {
			addNewGoalByProximity(TileStatus.KEY_BLUE);
			newGoalAdded = true;
		} else if (currentTileStatus == TileStatus.DOOR_GREEN
				&& envPositions.get(TileStatus.KEY_GREEN).size() == envPositions.get(TileStatus.DOOR_GREEN).size()
				&& sourceTileStatus != TileStatus.KEY_GREEN) {
			addNewGoalByProximity(TileStatus.KEY_GREEN);
			newGoalAdded = true;
		} else if (currentTileStatus == TileStatus.DOOR_YELLOW
				&& envPositions.get(TileStatus.KEY_YELLOW).size() == envPositions.get(TileStatus.DOOR_YELLOW).size()
				&& sourceTileStatus != TileStatus.KEY_YELLOW) {
			addNewGoalByProximity(TileStatus.KEY_YELLOW);
			newGoalAdded = true;
		}
		else if (currentTileStatus == TileStatus.DOOR_RED
				&& envPositions.get(TileStatus.KEY_RED).size() == envPositions.get(TileStatus.DOOR_RED).size()
				&& sourceTileStatus != TileStatus.KEY_RED) {
			addNewGoalByProximity(TileStatus.KEY_RED);
			newGoalAdded = true;
		}
		return newGoalAdded;
	}

	private void AStarGoalPathFinding(Position selfPosition, GoalItem targetGoal) {

		// The Position class has also been updated to include an equals method
		// if starting and end position is the same - meaning target is reached, then a
		// star is not required
		Position targetPos = targetGoal.getGoalPosition();
		if (selfPosition.equals(targetPos)) {
			foundActionList.add(Action.DO_NOTHING);
		}

		// a star is responsible for keeping the open nodes in ascending order based on
		// priority
		// open nodes: the nodes that need to be visited
		// priority is the f function which is g + h
		// the top element in the priority queue (ascending order) is the first node to
		// be visited
		PriorityQueue<Node> frontier = new PriorityQueue<>();

		/** To track all the visited nodes, in terms of path **/
		cameFromPositionList.clear();
		foundActionList.clear();

		// Adding the start node, pick up the first node at the top of the priority
		// queue
		frontier.add(new Node(selfPosition, 0, ""));

		// stores the cost with already visited nodes
		Map<Position, Integer> costSoFar = new HashMap<>();

		// creating the node from the starting position , converting the starting
		// position to the node, cost is 0 because we are starting here - no direction
		Node start = new Node(selfPosition, 0, "");

		// starting node is already visited so put in the bucket of nodes already
		// visited
		cameFromPositionList.put(start, null);

		// the cost of the starting node is 0
		costSoFar.put(selfPosition, 0);

		// while there are more nodes to be visited meaning the open bucket has more
		// nodes to be visited
		while (!frontier.isEmpty()) {
			// pick up the top node to be explored (where the priority is smaller - meaning
			// the path cost is smaller), the node is then removed
			// from the bucket
			Node current = frontier.poll();

			// if the current node that we just explored is the target then we are done
			if (current.getPosition().equals(targetPos)) {
				findingPath(current); // find the path
				break;
			}
			Map<String, Position> neighboringPositions = env.getNeighborPositions(current.getPosition());

			// traverse through the neighboring four positions
			for (Map.Entry<String, Position> entry : neighboringPositions.entrySet()) {

				// Calculating the cost for the path from the starting point to the current
				// positions's neighbor node
				int new_cost = costSoFar.get(current.getPosition()) + 1; // step cost is 1

				// extract the position object
				Position nextPosition = entry.getValue();

				// the position picked up is not boundary and the position is not impassable and
				// the position was never visited
				TileStatus nextPositionTileStatus = env.getTiles().get(nextPosition).getStatus();
				if (nextPosition != null && nextPositionTileStatus != TileStatus.WALL
						&& nextPositionTileStatus != TileStatus.WATER
						&& costSoFar.containsKey(nextPosition) == false
						&& !makeDoorAsWall(nextPositionTileStatus, targetGoal)) {

					// put the position in the visited bucket
					costSoFar.put(nextPosition, new_cost);

					// figuring out that which neighbor node cost is the smallest from the starting
					// position to that particular neighbor node
					if (new_cost <= costSoFar.get(nextPosition)) {
						// calculating the total cost which is f = g + h.
						int priority = new_cost + heuristic(nextPosition, targetPos);
						// create the node object which would contain the new neighboring position, the
						// total cost, and the direction involved
						Node neighborNode = new Node(nextPosition, priority, entry.getKey());
						// add the node object to frontier bucket - so it will sit in the priority queue
						frontier.add(neighborNode);
						// so add that node into the visited nodes bucket - to track the direction
						cameFromPositionList.put(neighborNode, current);					

					}

				}

			}
		}

	}
    
	// In the path if the challenge and goal type match then make that tile as impassable so that new goal cannot be added
	private boolean makeDoorAsWall(TileStatus tileStatus, GoalItem moveAction) {
		if ((tileStatus == TileStatus.DOOR_BLUE && moveAction.getGoalType() == TileStatus.KEY_BLUE) ||
				(tileStatus == TileStatus.DOOR_GREEN && moveAction.getGoalType() == TileStatus.KEY_GREEN) ||
				(tileStatus == TileStatus.DOOR_RED && moveAction.getGoalType() == TileStatus.KEY_RED) ||
				(tileStatus == TileStatus.DOOR_YELLOW && moveAction.getGoalType() == TileStatus.KEY_YELLOW)) {
			return true;
		}
		return false;
	}

	private void findingPath(Node target) {
		// Storing the node in the backward order
		ArrayList<Node> backwardPath = new ArrayList<Node>();

		// The array list will first contain the starting position which is the target
		// node
		backwardPath.add(target);		

		// Go through all the visited nodes and store it in the array list **/
		while (cameFromPositionList.get(target) != null) {
			target = cameFromPositionList.get(target);
			backwardPath.add(target);
			
		}
		TileStatus sourceTileStatus = env.getTiles().get(backwardPath.get(backwardPath.size() - 1).position).getStatus();
		// Running the for loop in the backward order to figure out the direction
		for (int i = 0; i < backwardPath.size(); i++) {

			/** Start off by getting the last index **/
			int pathIndex = backwardPath.size() - 1 - i;

			/** Get the direction name of that specific backward path step **/
			Node pathNode = backwardPath.get(pathIndex);
			String direction = pathNode.getDirection();
			TileStatus currentTileStatus = env.getTiles().get(pathNode.position).getStatus();
			if(isSubGoalAdded(sourceTileStatus, currentTileStatus))
			{
                foundActionList.clear();
				return;
			}
			// make actions based on the direction in backward order
			// add all the actions in to queue where get Action will pick each action and
			// move
			if (direction == "above") {
				foundActionList.add(Action.MOVE_UP);
			}
			if (direction == "below") {
				foundActionList.add(Action.MOVE_DOWN);
			}

			if (direction == "right") {
				foundActionList.add(Action.MOVE_RIGHT);
			}
			if (direction == "left") {
				foundActionList.add(Action.MOVE_LEFT);
			}
		}

	}

	/**
	 * GoalItem class to store the information of goal type like Key, Chip and position of the goal
	 * 
	 */
	private class GoalItem {
		private Position goalPosition;
		private TileStatus goalType;

		public GoalItem(Position goalPosition, TileStatus goalType) {
			this.goalPosition = goalPosition;
			this.goalType = goalType;
		}

		public Position getGoalPosition() {
			return goalPosition;
		}

		public TileStatus getGoalType() {
			return goalType;
		}

	}

	/**
	 * Node class to store the information required for the a star logic, compares
	 * the priorities
	 * 
	 */
	private class Node implements Comparable<Node> {

		// Stores the position info for the node
		private Position position;

		// Stores the priority info for the node
		private int priority;

		// Stores the direction info for the node
		private String direction;

		// Constructor that initializes the node fields
		public Node(Position position, int priority, String direction) {
			this.position = position;
			this.priority = priority;
			this.direction = direction;
		}

		/**
		 * Extracting position from node class
		 * 
		 * @return the position
		 */
		public Position getPosition() {
			return position;

		}

		/**
		 * Extracting direction from node class
		 * 
		 * @return the direction
		 */
		public String getDirection() {
			return direction;
		}

		/**
		 * Comparing the priorities of two nodes that will be used
		 * in the priority queue
		 */
		@Override
		public int compareTo(Node o) {
			return Integer.compare(priority, o.priority);
		}

	}
    

	private int heuristic(Position sourcePosition, Position targePosition) {
		int manHattanDistance = Math.abs(targePosition.getRow() - sourcePosition.getRow()) + Math.abs(targePosition.getCol() - sourcePosition.getCol());
		return manHattanDistance;
	}

	@Override
	public String toString() {
		return "Robot [pos=" + env.getRobotPosition(this) + "]";
	}

}