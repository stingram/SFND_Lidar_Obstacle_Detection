/* \author Aaron Brown */
// Quiz on implementing kd tree

#include <pcl/common/common.h>
#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(std::vector<float> point, int id, Node*& node, int depth)
	{
		// Got to end, insert
		//if(*node == NULL)
		if(node == NULL)
		{
			std::cout << "Depth: " << depth << "\n";
			// *node = new Node(point, id);
			node = new Node(point, id);
		}
		else
		{
			std::cout << "Depth: " << depth << "\n";
			// check which dimension to split on
			if(depth % 2 == 0)
			{
				// split on x
				// if(point.at(0) < (*node)->point.at(0)){
					//insertHelper(point, id, &((*node)->left), depth+1);
				std::cout << "node Address: " << node << " \n"; 
				std::cout << "Node point: " << node->point.at(0) << " \n"; 
				std::cout << "Point: " << point.at(0) << "\n";
				if(point.at(0) < node->point.at(0)){
					std::cout << "Here!\n";
					insertHelper(point, id, node->left, depth+1);
				}
				else{
					//insertHelper(point, id, &((*node)->right), depth+1);
					insertHelper(point, id, node->right, depth+1);
				}
			}
			else
			{
				// split on y
				// if(point.at(1) < (*node)->point.at(1)){
					// insertHelper(point, id, &((*node)->left), depth+1);
				if(point.at(1) < node->point.at(1)){
					insertHelper(point, id, node->left, depth+1);
				}
				else{
					// insertHelper(point, id, &((*node)->right), depth+1);
					insertHelper(point, id, node->right, depth+1);
				}
			}
		}
		
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root

		// check if root is null
		if(root == NULL)
		{
			Node* node = new Node(point, id);
			root = node;
			std::cout << "Root set\n";
		}
		else
		{
			// insertHelper(point, id, &root, 0);
			insertHelper(point, id, root, 0);
		}

	}


	// close enough
	bool close_enough(const std::vector<float>& target, const std::vector<float>& point, const float distanceTol)
	{
		float sum = 0;
		float dist = 0;
		int i = 0;
		for(float t: target)
		{
			sum += std::pow(t-point.at(i),2);
			++i;
		}
		dist = std::sqrt(sum);
		if(dist < distanceTol)
			return true;
		return false;
	}


	bool within_boundary(const std::vector<std::pair<float, float>>& boundaries, const std::vector<float> & node_point)
	{
		int i = 0;
		for(float val: node_point)
		{
			if(val < boundaries.at(i).first || val > boundaries.at(i).second)
			{
				// along som dimension the current node point is too far from target
				// check for overlap of bounding box with two dividing zones for this node
				return false;
			}
			++i;
		}
		return true;	
	}


	bool in_region(const std::vector<std::pair<float,float>> & boundaries, Node* & node, int depth, int REGION_FLAG)
	{
		// check "smaller region"
		if(REGION_FLAG==0)
		{
			// check x value
			if(depth % 2 == 0)
			{
				// check leftmost boundary
				if(boundaries.at(0).first < node->point.at(0))
					return true;

			}
			// check y value
			else
			{
				// check lower boundary
				if(boundaries.at(1).first < node->point.at(1))
					return true;
			}
			
		}
		// check "bigger region"
		else if(REGION_FLAG==1)
		{
			// check x value
			if(depth % 2 == 0)
			{
				// check rightmost boundary
				if(boundaries.at(0).second > node->point.at(0))
					return true;

			}
			// check y value
			else
			{
				// check upper boundary
				if(boundaries.at(1).second > node->point.at(1))
					return true;

			}
		}
		return false;
	}

	// search helper
	void search_helper(const std::vector<std::pair<float, float>>& boundaries, Node* & node,
						const std::vector<float> & target,std::vector<int>& ids, const float& distanceTol, int depth)
		{
			if(node == NULL)
			{
				return;
			}
			else{

				// check if current node is within boundary
				if(within_boundary(boundaries, node->point))
				{

					// Never got outside of box so we now need to calculate distance
					if(close_enough(target, node->point, distanceTol))
					{
						// add target to list of ids
						ids.push_back(node->id);
					}
				}
				// Check if box boundary overlaps regions
				if(in_region(boundaries, node, depth, 0))
					search_helper(boundaries,node->left, target, ids, distanceTol, depth+1);
				if(in_region(boundaries, node, depth, 1))
					search_helper(boundaries,node->right, target, ids, distanceTol, depth+1);
			}
		}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		Node* curr = root;
		int i;
		float dist;

		// compute box boundary lists
		std::vector<std::pair<float,float>> boundaries;
		for(float val: target)
		{
			boundaries.push_back(std::pair<float,float>(val-distanceTol,val+distanceTol));
		}

		search_helper(boundaries, root, target, ids, distanceTol, 0);
		return ids;
	}
	

};




