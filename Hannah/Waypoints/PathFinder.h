#pragma once

#include <iostream>
#include <map>


#include <opencv/cv.h>
#include <opencv/highgui.h>


#include "PriorityQueueClass.h"


// Definition of a node for the graph
struct NodeType
{
	const int i, j;	// Position
	unsigned char pixel; // Grey value of pixel
	bool visited;
	bool walkable;
	unsigned int cost_accumulator;	 // The cost to reach this node
	std::vector<NodeType*> outgoing; // Nodes that can be reached from this node
	std::vector<NodeType*> incoming; // Nodes that lead to this node	

	NodeType(const int _i, const int _j) :
		i(_i), j(_j),
		pixel(0), visited(false), walkable(false),
		cost_accumulator(0)
	{}
};

// Renaming to prevent typos
typedef std::map < NodeType*, NodeType*> PathType;

/*
Impletents the A* search algorithm to search a given graph for the optimal path.
*/
class PathFinder
{
public:
	PathFinder();
	~PathFinder();

	/*The map is processed with OpenCV to create a costmap.*/
	void create_costmap(const cv::Mat* imag);

	/*Searche the graph starting at 'start'. Finished when a optimal path to 'goal' is found. The path is appended at the end of 'nodes'*/
	void Search(NodeType* start, NodeType* goal, std::vector<NodeType*>* nodes);

private:
		
	// Takes the current node and its valid successor and calculates the cost for the action
	int cost_of_action(NodeType* current, NodeType* successor);

	// Converts the path map to a list of nodes
	void path_to_nodes(PathType* path, std::vector<NodeType*>* nodes);

	// Test if the given node is a goal state. The first goal is the top right corner in the map. When it is reached
	// the goal is changed to the bottom right corner. The next goal state would be the left bottom corner and then
	// again the start state -> closed path
	bool goal_test(NodeType* n);

	// Searches a path from start to goal
	void aStarSearch(PathType* path);

	// Call to get the heuristic value for given node n
	int heuristic(NodeType* n);

	NodeType* m_start, *m_goal;

	cv::Mat m_costmap;

};
