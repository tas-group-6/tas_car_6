#include "PathFinder.h"

PathFinder::PathFinder(){}
PathFinder::~PathFinder(){}


void PathFinder::create_costmap(const cv::Mat* image)
{
	cv::Mat image_grey;
	cv::Mat image_thres;
	cv::Mat test, test2;

	cv::cvtColor(*image, image_grey, CV_RGB2GRAY);

	cv::threshold(image_grey, image_thres, 250, 255, CV_THRESH_BINARY);

	//cv::namedWindow("test", CV_WINDOW_FREERATIO);
	//cv::imshow("test", image_thres);
	//cv::waitKey(0);
	
	cv::blur(image_thres, m_costmap, cv::Size(25, 25));
	
	m_costmap = ~m_costmap; // negate -> white == high costs
	
	//cv::threshold(test, m_costmap, 25, 255, CV_THRESH_BINARY);

	//cv::blur(test2, m_costmap, cv::Size(25, 25));

	//cv::imwrite("blur.png", m_costmap);


	//cv::namedWindow("test", CV_WINDOW_FREERATIO);
	//cv::imshow("test", m_costmap);
	//cv::waitKey(0);	

	cv::imwrite("CostMap.png", m_costmap);

}

void PathFinder::Search(NodeType* start, NodeType* goal, std::vector<NodeType*>* nodes)
{
	m_start = start;
	m_goal = goal;
	
	PathType path;
	NodeType dummy(-1, -1);
	path[m_start] = &dummy;

	aStarSearch(&path);

	std::vector<NodeType*> tmp_nodes;

	path_to_nodes(&path, &tmp_nodes);
	
	// Append here so that the order of the nodes is correct -> path from start to goal
	//for (unsigned int i = 0; i < tmp_nodes->size(); i++)
	for (int i = tmp_nodes.size()-1; i >= 0; i--)
	{
		nodes->push_back(tmp_nodes[i]);
	}
}

bool PathFinder::goal_test(NodeType* n)
{
	//bool result;

	//static int counter = 0;

	//if (counter < 100) return false;


	


	return n == m_goal;

	//static int counter = 0;

	//switch (counter)
	//{
	//case 0: // Corner 1 475 255
	//	if (n == goal_node)
	//	{
	//		std::cout << "Temp goal 475 255 reached. Next goal is 457 512" << std::endl;
	//		goal_node = graph[457][512];
	//		counter++;
	//		return false;
	//	}
	//	break;
	//case 1: // Corner 2 457 512
	//	if (n == goal_node)
	//	{
	//		std::cout << "Temp goal 457 512 reached. Next goal is 208 492" << std::endl;
	//		goal_node = graph[208][492];
	//		counter++;
	//		return false;
	//	}
	//	break;
	//case 2: // Corner 3 208 492
	//	if (n == goal_node)
	//	{
	//		std::cout << "Temp goal 208 492 reached. Next goal is 221 235" << std::endl;
	//		goal_node = start_node;

	//		counter++;
	//		return false;
	//	}
	//	break;
	//case 3: // Start state 221 235
	//	if (n == goal_node)
	//	{
	//		std::cout << "Final goal 221 235 reached" << std::endl;

	//		return true;
	//	}
	//	break;
	//default:
	//	throw "Undifined goal state";
	//	break;
	//}

	//return false;
}

int PathFinder::heuristic(NodeType* n)
{
	//Manhatten heuristic
	//return abs(n->i - m_goal->i) + abs(n->j - m_goal->j);

	// Euclidean distance
	int a = (int)round(sqrt((n->i - m_goal->i)*(n->i - m_goal->i) + (n->j - m_goal->j)*(n->j - m_goal->j)));

	// Costmap value
	int b = (int)round( 1000.0 * (double)m_costmap.at<unsigned char>(n->i, n->j) / 255.0);

	return a + b;
}

int PathFinder::cost_of_action(NodeType* current, NodeType* successor)
{
	if (current == successor) return 0;
	
	const int cost_per_single_move = 1;

	if (current->walkable && successor->walkable)
	{
		// Manhatten distance of the two nodes
		int tmp = abs(current->i - successor->i) + abs(current->j - successor->j);

		if (tmp == 1) return cost_per_single_move;
		else return tmp * cost_per_single_move * 10; // Jumping is penelized by factor of ten
		
	}

	if (current->walkable && !successor->walkable)
		return 100000; // Fee for moving into unwalkable parts of the map

	return 0;
}

void PathFinder::aStarSearch(PathType* path)
{
	std::cout << "- searching -" << std::endl;

	PriorityQueue<NodeType*> pq;

	pq.push(m_start, 0); // Start with start node


	NodeType* current = 0;
	NodeType* sucessor = 0;
	NodeType* tmp_node = 0;

	int tmp_costs;
	unsigned int min_goal_cost = -1;
	int cost;		// Cost per action -> moving one node in the graph

	while (!pq.empty())
	{
		current = pq.pop();

		// Test for goal and check if a goal is found with a better path
		if (goal_test(current) && min_goal_cost > current->cost_accumulator)
		{
			min_goal_cost = current->cost_accumulator;

			// no need to expand successors because ucs returns best paths
			// every path starting at a goal node must be longer than the current one
			continue;
		}
		// when the goal is the node with the smallest path cost in the queue,
		// every other remaining path must be longer->stop loop
		else if (min_goal_cost < current->cost_accumulator) break;

		//Mark current state as visited
		current->visited = true;

		for (unsigned int i = 0; i < current->outgoing.size(); i++)
		{
			sucessor = current->outgoing[i];

			cost = cost_of_action(current, sucessor);

			//        = costs to get to this point + cost for the action 
			tmp_costs = current->cost_accumulator + cost; 

			// a node which has not been visited and is not in the queue can be added without danger
			if (!sucessor->visited && !pq.find(sucessor))
			{
				path->operator[](sucessor) = current;
				sucessor->cost_accumulator = tmp_costs;
				pq.push(sucessor, tmp_costs + heuristic(sucessor));

			}
			// otherwise test if node is in queue with higher cost. If true,
			// the longer path can be replaced with the current one
			else if (pq.find(sucessor, tmp_costs + heuristic(sucessor)))
			{
				path->operator[](sucessor) = current;
				sucessor->cost_accumulator = tmp_costs;
				pq.push(sucessor, tmp_costs + heuristic(sucessor));
			}
		}
	}
}


void PathFinder::path_to_nodes(PathType* path, std::vector<NodeType*>* nodes)
{
	NodeType* current = m_goal;
	
	do
	{
		// Check if current has a successor
		//if (path->find(path->at(current)) != path->end())
		//else break;
		
		nodes->push_back(current);
		
		current = path->at(current);
				
		if (current->i == -1 && current->j == -1) break;

	} while (true);

}