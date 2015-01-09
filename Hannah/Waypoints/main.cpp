#include <iostream>
#include <string>
#include <vector>
#include <time.h>

#include <math.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "PathFinder.h"

//using namespace std;
//using namespace cv;

const int sizeX = 1280;						// Horizontal size of the picture
const int sizeY = 800;						// Vertical size of the picture
const std::string WINDOW_NAME = "Waypoint Generator";		// The name of the window

// The floor plan of the building
const cv::Mat map_LSR = cv::imread("LSR_3.png");

std::vector< std::vector< NodeType* >> graph; // The graph for searching a way 

void initialize_graph()
{
	std::cout << "- initializing -" << std::endl;

	// Create nodes and set coordinates
	graph.resize(map_LSR.rows);
	for (int i = 0; i < map_LSR.rows; i++)
	{
		graph[i].resize(map_LSR.cols);

		for (int j = 0; j < map_LSR.cols; j++)
			graph[i][j] = 0; // new NodeType(i, j);
	}

	//// Connect nodes of graph
	//for (int i = 0; i < map_LSR.rows; i++)
	//{
	//	for (int j = 0; j < map_LSR.cols; j++)
	//	{
	//		NodeType* cur_node = graph[i][j];

	//		if (j < map_LSR.cols - 1)
	//		{
	//			graph[i][j + 1]->incoming.push_back(cur_node);
	//			cur_node->outgoing.push_back(graph[i][j + 1]);
	//		}
	//		if (i != 0)
	//		{
	//			graph[i - 1][j]->incoming.push_back(cur_node);
	//			cur_node->outgoing.push_back(graph[i - 1][j]);
	//		}
	//		if (j != 0)
	//		{
	//			graph[i][j - 1]->incoming.push_back(cur_node);
	//			cur_node->outgoing.push_back(graph[i][j - 1]);
	//		}
	//		if (i < map_LSR.rows - 1)
	//		{
	//			graph[i + 1][j]->incoming.push_back(cur_node);
	//			cur_node->outgoing.push_back(graph[i + 1][j]);
	//		}
	//	}
	//}
}

void convert_map_to_graph()
{
	std::cout << "- converting -" << std::endl;

	// Convert to greyscale -> known type of matrix ( unsigned char )
	cv::Mat image_grey;
	cv::cvtColor(map_LSR, image_grey, CV_RGB2GRAY);


	// The one pixel rim of the picture is left out so that no out of range checks are required
	for (int i = 1; i < image_grey.rows - 1; i++)
	{
		for (int j = 1; j < image_grey.cols - 1; j++)
		{
			unsigned char pixel = image_grey.at<unsigned char>(i, j);

			if (pixel > 250)
			{
				if (graph[i][j]) delete graph[i][j];

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

			if (pixel > 250)
			{
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

void clean_graph()
{
	std::cout << "- cleaning -" << std::endl;

	for (int i = 0; i < map_LSR.rows; i++)
	{
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

void clear_visited()
{
	std::cout << "- clearing visited-" << std::endl;

	for (int i = 0; i < map_LSR.rows; i++)
	{
		for (int j = 0; j < map_LSR.cols; j++)
		{
			if (graph[i][j]) graph[i][j]->visited = false;
		}
	}
}

bool plot_path(std::vector< NodeType*>* nodes)
{
	std::cout << "- plotting -" << std::endl;

	cv::Mat image = map_LSR.clone();

	for (auto it = nodes->begin(); it != nodes->end(); it++)
	{
		image.at<cv::Vec3b>((*it)->x, (*it)->y) = cv::Vec3b(255, 0, 0);
	}

	cv::namedWindow(WINDOW_NAME, CV_WINDOW_FREERATIO);
	cv::imshow(WINDOW_NAME, image);

	int result = cv::waitKey(0);
	if (result == (int)'q') return true;

	return false;
}

void debug_draw()
{
	std::cout << "- debug draw -" << std::endl;

	cv::Mat image = map_LSR.clone();

	for (int i = 0; i < map_LSR.rows; i++)
	{
		for (int j = 0; j < map_LSR.cols; j++)
		{
			//image.at<cv::Vec3b>(i, j) = cv::Vec3b(0,0, 0);

			if (graph[i][j] && graph[i][j]->visited)
				image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);

			//if (graph[i][j] && !graph[i][j]->walkable)
			//{
			//	image.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);
			//}

		}
	}

	cv::namedWindow(WINDOW_NAME, CV_WINDOW_FREERATIO);
	cv::imshow(WINDOW_NAME, image);

	int result = cv::waitKey(0);

}

int main(int __argc, char* __argv[])
{
	srand((unsigned int)time(NULL));

	if (!map_LSR.data)
	{
		std::cout << "Could not load picture" << std::endl;
		return -1;
	}

	PathFinder pf;

	pf.create_costmap(&map_LSR);
	
	//return 1;

	initialize_graph();

	convert_map_to_graph();

	//NodeType* start_node = graph[221][235];
	//NodeType* goal_node = graph[475][255];

	NodeType* corner_tl = graph[235][220];
	NodeType* corner_tl2 = graph[236][220]; // Two neighboured pixels -> no closed loop easier to plot
	NodeType* corner_tr = graph[250][475];
	NodeType* corner_br = graph[500][460];
	NodeType* corner_bl = graph[490][205];

	std::vector<NodeType*> nodes; // The nodes which lead to the goal

	//path[corner_tl] = graph[0][0]; //Set start node parent to the origin

	pf.Search(corner_tl, corner_tr, &nodes); clear_visited();
	pf.Search(corner_tr, corner_br, &nodes); clear_visited();
	pf.Search(corner_br, corner_bl, &nodes); clear_visited();
	pf.Search(corner_bl, corner_tl, &nodes); clear_visited();

	std::cout << "Path Found with length " << nodes.size() << std::endl;

	plot_path(&nodes);

	clean_graph();

	//cv::Mat image = cv::Mat::zeros(sizeY, sizeX, CV_8UC3);
	//cv::namedWindow(WINDOW_NAME, CV_WINDOW_FREERATIO);
	//cv::imshow(WINDOW_NAME, image);

	//int result = cv::waitKey(0);
	//if (result == (int)'q') return true;

	return 1;
}