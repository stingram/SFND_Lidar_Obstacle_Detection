// KD TREE PROJECT

#include <pcl/common/common.h>

// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node<PointT>* left;
	Node<PointT>* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(PointT point, int id, Node<PointT>*& node, int depth)
	{
		// Got to end, insert
		if(node == NULL)
		{
			node = new Node<PointT>(point, id);
		}
		else
		{
			// check which dimension to split on
			if(depth % 3 == 0)
			{
				// split on x
				if(point.x < node->point.x){
					insertHelper(point, id, node->left, depth+1);
				}
				else{
					insertHelper(point, id, node->right, depth+1);
				}
			}
			else if(depth % 3 == 1)
			{
				// split on y
				if(point.y < node->point.y){
					insertHelper(point, id, node->left, depth+1);
				}
				else{
					insertHelper(point, id, node->right, depth+1);
				}
			}
			else if(depth % 3 == 2)
			{
				// split on z
				if(point.z < node->point.z){
					insertHelper(point, id, node->left, depth+1);
				}
				else{
					insertHelper(point, id, node->right, depth+1);
				}
			}
		}
		
	}

	void insert(PointT point, int id)
	{
		// check if root is null
		if(root == NULL)
		{
			Node<PointT>* node = new Node<PointT>(point, id);
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
	bool close_enough(const PointT& target, const PointT& point, const float distanceTol)
	{
		float sum = 0;
		float dist = 0;

		sum += std::pow(target.x-point.x,2);
		sum += std::pow(target.y-point.y,2);
		sum += std::pow(target.z-point.z,2);

		dist = std::sqrt(sum);
		if(dist < distanceTol)
			return true;
		return false;
	}


	bool within_boundary(const std::vector<std::pair<float, float>>& boundaries, const PointT & node_point)
	{
		if((node_point.x < boundaries.at(0).first || node_point.x > boundaries.at(0).second) || 
			(node_point.y < boundaries.at(1).first || node_point.y > boundaries.at(1).second) || 
				(node_point.z < boundaries.at(2).first || node_point.z > boundaries.at(2).second))
		{
			// along some dimension the current node point is too far from target
			// check for overlap of bounding box with two dividing zones for this node
			return false;
		}
		return true;	
	}


	bool in_region(const std::vector<std::pair<float,float>> & boundaries, Node<PointT>* & node, int depth, int REGION_FLAG)
	{
		// check "smaller region"
		if(REGION_FLAG==0)
		{
			// check x value
			if(depth % 3 == 0)
			{
				// check leftmost boundary
				if(boundaries.at(0).first < node->point.x)
					return true;
			}
			// check y value
			else if (depth % 3 == 1)
			{
				// check lower boundary
				if(boundaries.at(1).first < node->point.y)
					return true;
			}
			// check z value
			else if (depth % 3 == 2)
			{
				// check lower boundary
				if(boundaries.at(2).first < node->point.z)
					return true;
			}
		}
		// check "bigger region"
		else if(REGION_FLAG==1)
		{
			// check x value
			if(depth % 3 == 0)
			{
				// check rightmost boundary
				if(boundaries.at(0).second > node->point.x)
					return true;
			}
			// check y value
			else if(depth % 3 == 1)
			{
				// check upper boundary
				if(boundaries.at(1).second > node->point.y)
					return true;
			}
			// check z value
			else if(depth % 3 == 2)
			{
				// check upper boundary
				if(boundaries.at(2).second > node->point.z)
					return true;
			}
		}
		return false;
	}

	// search helper
	void search_helper(const std::vector<std::pair<float, float>>& boundaries, Node<PointT>* & node,
						const PointT & target, pcl::PointIndices::Ptr& ids, const float& distanceTol, int depth)
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
						ids->indices.push_back(node->id);

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
	pcl::PointIndices::Ptr search(PointT target, float distanceTol)
	{
		pcl::PointIndices::Ptr ids (new pcl::PointIndices());

		Node<PointT>* curr = root;
		int i;
		float dist;

		// compute box boundary lists
		std::vector<std::pair<float,float>> boundaries;

		//  3D boundaries
		boundaries.push_back(std::pair<float,float>(target.x-distanceTol,target.x+distanceTol));
		boundaries.push_back(std::pair<float,float>(target.y-distanceTol,target.y+distanceTol));
		boundaries.push_back(std::pair<float,float>(target.z-distanceTol,target.z+distanceTol));

		search_helper(boundaries, root, target, ids, distanceTol, 0);
		return ids;
	}
	

};




