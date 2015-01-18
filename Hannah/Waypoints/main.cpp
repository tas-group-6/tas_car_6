#include <iostream>
#include <string>
#include <vector>
#include <time.h>
#include <math.h>
#include <fstream>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "PathFinder.h"

#define CLOCKWISE false

#define PIXEL_TO_METER 0.05 /* Factor for conversion */
#define METER_TO_PIXEL 20.0 /* Factor for conversion */


// The name of the window
const std::string WINDOW_NAME = "Waypoint Generator";

// The floor plan of the building
const cv::Mat map_LSR = cv::imread("LSR_3.png");

// The position of a node on the map. Either one of the four corridors or an edge
enum NodePosition{ TOP, LEFT, BOTTOM, RIGHT, EDGE };

// The graph for searching a way 
std::vector< std::vector< NodeType* >> graph;


void transform_node_to_world(const NodeType* n, double& x, double& y)
{
	x = n->j * PIXEL_TO_METER;
	y = (map_LSR.rows - 1 - n->i) * PIXEL_TO_METER;
}
void transform_world_to_node(const double& x, const double& y, int& i, int& j)
{
	i = (map_LSR.rows - 1 - (int)round(y * METER_TO_PIXEL));
	j = (int)round(x * METER_TO_PIXEL);
}

/*Resize grid of nodes.*/
void initialize_graph()
{
	std::cout << "- initializing -" << std::endl;

	graph.resize(map_LSR.rows);
	for (int i = 0; i < map_LSR.rows; i++)
	{
		graph[i].resize(map_LSR.cols);

		for (int j = 0; j < map_LSR.cols; j++)
			graph[i][j] = 0;
	}
}

/*Load picture of LSR chair and create the undirected graph*/
void convert_map_to_graph()
{
	std::cout << "- converting -" << std::endl;
	cv::Mat image_grey;

	// Convert to greyscale -> known type of matrix ( unsigned char )
	cv::cvtColor(map_LSR, image_grey, CV_RGB2GRAY);

	//cv::Mat image_thres;
	//cv::Mat image_blur;

	//cv::namedWindow("test2", CV_WINDOW_FREERATIO);
	//cv::imshow("test2", image_grey);
	//cv::waitKey(0);


	//cv::threshold(image_grey, image_thres, 250, 255, CV_THRESH_BINARY);
	//cv::blur(image_thres, image_blur, cv::Size(25, 25));
	//cv::threshold(image_blur, image_thres, 235, 255, CV_THRESH_BINARY);

	//cv::namedWindow("test2", CV_WINDOW_FREERATIO);
	//cv::imshow("test2", image_thres);
	//cv::waitKey(0);


	// The one pixel rim of the picture is left out so that no out of range checks are required
	for (int i = 1; i < image_grey.rows - 1; i++)
	{
		for (int j = 1; j < image_grey.cols - 1; j++)
		{
			unsigned char pixel = image_grey.at<unsigned char>(i, j);

			// Pixel is white => walkable
			if (pixel > 250)
			{
				// When already existing, delete node first
				if (graph[i][j]) delete graph[i][j];

				// Then create new node at the current position
				graph[i][j] = new NodeType(i, j);
				graph[i][j]->pixel = pixel;
				graph[i][j]->walkable = true;
			}
		}
	}

	// The one pixel rim of the picture is left out so that no out of range checks are required
	for (int i = 1; i < image_grey.rows - 1; i++)
	{
		for (int j = 1; j < image_grey.cols - 1; j++)
		{
			unsigned char pixel = image_grey.at<unsigned char>(i, j);

			// Pixel is white => walkable
			if (pixel > 250)
			{
				// if Neighbour is existing -> set incoming and outgoing nodes
				// else Create unwalkable neighbour and set up connections => the whole graph is surrounded by unwalkable nodes => walls can be dectected
				if (graph[i][j + 1]) { graph[i][j + 1]->incoming.push_back(graph[i][j]); graph[i][j]->outgoing.push_back(graph[i][j + 1]); }
				else
				{
					graph[i][j + 1] = new NodeType(i, j + 1);
					graph[i][j + 1]->incoming.push_back(graph[i][j]); graph[i][j]->outgoing.push_back(graph[i][j + 1]);
				}

				if (graph[i - 1][j]) { graph[i - 1][j]->incoming.push_back(graph[i][j]); graph[i][j]->outgoing.push_back(graph[i - 1][j]); }
				else
				{
					graph[i - 1][j] = new NodeType(i - 1, j);
					graph[i - 1][j]->incoming.push_back(graph[i][j]); graph[i][j]->outgoing.push_back(graph[i - 1][j]);
				}

				if (graph[i][j - 1]) { graph[i][j - 1]->incoming.push_back(graph[i][j]); graph[i][j]->outgoing.push_back(graph[i][j - 1]); }
				else
				{
					graph[i][j - 1] = new NodeType(i, j - 1);
					graph[i][j - 1]->incoming.push_back(graph[i][j]); graph[i][j]->outgoing.push_back(graph[i][j - 1]);
				}

				if (graph[i + 1][j]) { graph[i + 1][j]->incoming.push_back(graph[i][j]); graph[i][j]->outgoing.push_back(graph[i + 1][j]); }
				else
				{
					graph[i + 1][j] = new NodeType(i + 1, j);
					graph[i + 1][j]->incoming.push_back(graph[i][j]); graph[i][j]->outgoing.push_back(graph[i + 1][j]);
				}
			}
		}
	}
}

/*Release the whole graph*/
void clean_graph()
{
	std::cout << "- cleaning -" << std::endl;

	if (!graph.size()) return;

	for (int i = 0; i < map_LSR.rows; i++)
	{
		if (!graph[i].size()) continue;

		for (int j = 0; j < map_LSR.cols; j++)
		{
			if (graph[i][j])
			{
				delete graph[i][j];
				graph[i][j] = 0;
			}
		}
	}
}

/*Reset the visisted state from the last A* search*/
void clear_visited()
{
	std::cout << "- clearing visited -" << std::endl;

	for (int i = 0; i < map_LSR.rows; i++)
	{
		for (int j = 0; j < map_LSR.cols; j++)
		{
			if (graph[i][j]) graph[i][j]->visited = false;
		}
	}
}

/*Show the optimal path in the picture of the LSR chair */
bool plot_path(std::vector< NodeType*>* nodes)
{
	std::cout << "- plotting -" << std::endl;

	cv::Mat image = map_LSR.clone();

	for (auto it = nodes->begin(); it != nodes->end(); it++)
	{
		image.at<cv::Vec3b>((*it)->i, (*it)->j) = cv::Vec3b(255, 0, 0);
	}

	cv::namedWindow(WINDOW_NAME, CV_WINDOW_FREERATIO);
	cv::imshow(WINDOW_NAME, image);
	cv::waitKey(0);

	cv::imwrite("Plotted_path.png", image);

	return false;
}

/*Determine the position of a node in the map. Either a corridor or a corner.*/
NodePosition get_node_position(NodeType* n)
{
	if (n->i > 265 && n->i < 466)
	{
		if (n->j < 242) return NodePosition::LEFT;
		if (n->j > 441) return NodePosition::RIGHT;
	}

	if (n->j > 242 && n->j < 441)
	{
		if (n->i < 265) return NodePosition::TOP;
		if (n->i > 466) return NodePosition::BOTTOM;
	}

	return NodePosition::EDGE;
}

/*Convert the optimal path to waypoints with orientation. Safe the result into a text file, which can be used by the simple_navigation_goals node*/
void write_path_to_file(std::vector<NodeType*>* nodes)
{
	std::cout << "- writing to file -" << std::endl;

	if (nodes->size() <= 1) { std::cout << "At least two nodes are needed for a path!" << std::endl; return; }

	std::ofstream ofs;
	ofs.open("waypoints.txt", std::ios::out);
	if (!ofs.is_open()) { std::cout << "Output file stream not open" << std::endl; return; }

	double length = 0.0; // In meter
	const double length_threshold = 2.5; // In meter, corridors are roughly 15 meter long
	double x1, x2, y1, y2;

	double x, y;

	for (unsigned int i = 1; i < nodes->size(); i++)
	{
		transform_node_to_world(nodes->at(i), x1, y1);
		transform_node_to_world(nodes->at(i - 1), x2, y2);

		// Euclidean distance
		length += sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));

		if (length > length_threshold)
		{
			length = 0.0;

			// Coordinates of waypoint
			transform_node_to_world(nodes->at(i), x, y);

			// and its orientation
			switch (get_node_position(nodes->at(i)))
			{
			case TOP:
				ofs << x << " " << y << " " << 0.18 << std::endl;
				if (CLOCKWISE)  ofs << 0.0 << " " << 0.0 << " " << -0.011 << " " << 1.0 << std::endl;
				else ofs << 0.0 << " " << 0.0 << " " << 1.0 << " " << -0.002 << std::endl;
				break;
			case LEFT:
				ofs << x << " " << y << " " << 0.18 << std::endl;
				if (CLOCKWISE)  ofs << 0.0 << " " << 0.0 << " " << 0.682 << " " << 0.731 << std::endl;
				else ofs << 0.0 << " " << 0.0 << " " << -0.725 << " " << 0.689 << std::endl;
				break;
			case BOTTOM:
				ofs << x << " " << y << " " << 0.18 << std::endl;
				if (CLOCKWISE)  ofs << 0.0 << " " << 0.0 << " " << 1.0 << " " << 0.007 << std::endl;
				else ofs << 0.0 << " " << 0.0 << " " << -0.27 << " " << 1.0 << std::endl;
				break;
			case RIGHT:
				ofs << x << " " << y << " " << 0.18 << std::endl;
				if (CLOCKWISE)  ofs << 0.0 << " " << 0.0 << " " << -0.7 << " " << 0.715 << std::endl;
				else ofs << 0.0 << " " << 0.0 << " " << 0.689 << " " << 0.725 << std::endl;
				break;
			case EDGE:
				break;
			default:
				break;
			}
		}
	}
}

// Functions for debugging

void debug_draw()
{
	std::cout << "- debug draw -" << std::endl;

	cv::Mat image = map_LSR.clone();

	for (int i = 0; i < map_LSR.rows; i++)
	{
		for (int j = 0; j < map_LSR.cols; j++)
		{
			image.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255); //Background white

			if (graph[i][j])
				image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255); // Existings nodes in red

			if (graph[i][j] && !graph[i][j]->walkable)
			{
				image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0); // Walls in black
			}

			if (graph[i][j] && graph[i][j]->visited) // Visited states in blue
				image.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);
		}
	}

	cv::namedWindow(WINDOW_NAME, CV_WINDOW_FREERATIO);
	cv::imshow(WINDOW_NAME, image);

	int result = cv::waitKey(0);

	cv::imwrite("debug_draw.png", image);

}
void debug_path_to_file(std::vector<NodeType*>* nodes)
{
	std::cout << "- debug path to file -" << std::endl;


	std::ofstream ofs;
	ofs.open("path.txt", std::ios::out);
	if (!ofs.is_open()) { std::cout << "Output file stream not open" << std::endl; return; }
	for (unsigned int i = 0; i < nodes->size(); i++)
	{
		ofs << nodes->at(i)->i << " " << nodes->at(i)->j << std::endl;
	}

	return;
}
void debug_path_from_file(std::vector<NodeType*>* nodes)
{
	std::cout << "- debug path from file -" << std::endl;

	std::ifstream ifs;
	ifs.open("path.txt", std::ios::in);
	if (!ifs.is_open()) { std::cout << "Output file stream not open" << std::endl; return; }

	NodeType* n;
	int x, y;

	while (!ifs.eof())
	{
		ifs >> x >> y;

		n = new NodeType(x, y);
		nodes->push_back(n);
	}

	return;
}
void debug_compare_waypoints()
{
	struct Waypoint	{ int i, j; };
	Waypoint w;
	double x, y, tmp;

	std::vector<Waypoint> points;
	std::vector<Waypoint> points_ref;

	std::ifstream ifs1, ifs2;

	ifs1.open("waypoints.txt", std::ios::in);
	if (!ifs1.is_open()) { std::cout << "Output file stream not open" << std::endl; return; }


	std::string line;

	while (std::getline(ifs1, line))
	{
		if (line == "") continue; //Skip empty lines

		stringstream ss(line);
		ss >> x >> y >> tmp; // Position line

		transform_world_to_node(x, y, w.i, w.j);

		points.push_back(w);

		std::getline(ifs1, line); //Quternion line
	}
	ifs1.close();

	ifs2.open("WaypointsReference.txt", std::ios::in);
	if (!ifs2.is_open()) { std::cout << "Output file stream not open" << std::endl; return; }

	while (std::getline(ifs2, line))
	{
		if (line == "") continue; //Skip empty lines

		stringstream ss(line);
		ss >> x >> y >> tmp; // Position line

		transform_world_to_node(x, y, w.i, w.j);

		points_ref.push_back(w);

		std::getline(ifs2, line); //Quternion line
	}
	ifs2.close();

	cv::Mat image = map_LSR.clone();

	int radius = 3;
	int posI, posJ;

	for (unsigned int i = 0; i < points.size(); i++)
	{
		posI = (int)round(points[i].i);
		posJ = (int)round(points[i].j);

		for (int a = -radius; a < radius; a++)
			for (int b = -radius; b < radius; b++)
				image.at<cv::Vec3b>(posI + a, posJ + b) = cv::Vec3b(0, 0, 255);
	}

	for (unsigned int i = 0; i < points_ref.size(); i++)
	{
		posI = (int)round(points_ref[i].i);
		posJ = (int)round(points_ref[i].j);

		for (int a = -radius; a < radius; a++)
			for (int b = -radius; b < radius; b++)
				image.at<cv::Vec3b>(posI + a, posJ + b) = cv::Vec3b(255, 0, 0);
	}

	cv::namedWindow(WINDOW_NAME, CV_WINDOW_FREERATIO);
	cv::imshow(WINDOW_NAME, image);

	cv::waitKey(0);

	cv::imwrite("compare_path.png", image);
}



int main(int __argc, char* __argv[])
{
	if (!map_LSR.data)
	{
		std::cout << "Could not load picture" << std::endl;
		return -1;
	}

	PathFinder pf;

	// The nodes which lead to the goal, filled by the pathfinder
	std::vector<NodeType*> nodes;

	pf.create_costmap(&map_LSR);

	initialize_graph();

	convert_map_to_graph();

	//First approch: Define corners in the map and search from corner to corner
	NodeType* corner_tl = graph[235][220]; // Top left
	NodeType* corner_tr = graph[250][475]; // Top right
	NodeType* corner_br = graph[500][460]; // Bottom right
	NodeType* corner_bl = graph[490][205]; // Bottom left

	pf.Search(corner_tl, corner_tr, &nodes); /*debug_draw();*/ clear_visited();
	pf.Search(corner_tr, corner_br, &nodes); clear_visited();
	pf.Search(corner_br, corner_bl, &nodes); clear_visited();
	pf.Search(corner_bl, corner_tl, &nodes); clear_visited();

	// Alternative approach: The corridor at the gross labor is closed by hand in the png and the goal and start node are placed at both sides
	// => The A* searches the whole map and finds also the optimal path.
	// => Too bad result, i.e. to close to walls, doors etc.

	//NodeType* start = graph[235][244]; // Top left
	//NodeType* goal  = graph[235][249]; // Top left
	//
	//pf.Search(start, goal, &nodes);
	//
	//debug_draw();
	//clear_visited();

	std::cout << "Path found with length " << nodes.size() << std::endl;

	// To safe and load optimal path -> safe time when debugging
	//debug_path_to_file(&nodes);
	//debug_path_from_file(&nodes);

	plot_path(&nodes);
	write_path_to_file(&nodes);

	//debug_compare_waypoints();

	clean_graph();

	return 1;
}