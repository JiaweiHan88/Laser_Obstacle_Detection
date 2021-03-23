/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"
#include "pcl/point_types.h"
// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node *left;
	Node *right;

	Node(PointT arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}

	~Node()
	{
		delete left;
		delete right;
	}
};
template<typename PointT>
struct KdTree
{
	Node<PointT> *root;

	KdTree()
		: root(NULL)
	{
	}

	~KdTree()
	{
		delete root;
	}
	void insertrec(Node<PointT> *&node, PointT point, int id, int depth)
	{
		if (node == NULL)
		{
			node = new Node<PointT>(point, id);
		}
		else
		{
			uint cdepth = depth % 3;

			if (point.data[cdepth] < node->point.data[cdepth])
			{
				insertrec(node->left, point, id, depth + 1);
			}
			else
			{
				insertrec(node->right, point, id, depth + 1);
			}
		}
	}
	void insert(PointT point, int id)
	{
		insertrec(root, point, id, 0);
	}
	void searchrec(Node<PointT> *node, PointT target, float distanceTol, int depth, std::vector<int> &ids)
	{
		if (node == NULL)
			return;
		uint cdepth = depth % 3;
		//check if in box
		if (node->point.x >= (target.x - distanceTol) && node->point.x <= (target.x + distanceTol) 
		&& node->point.y >= (target.y - distanceTol) && node->point.y <= (target.y + distanceTol)
		&& node->point.z >= (target.z - distanceTol) && node->point.z <= (target.z + distanceTol))
		{
			//if in radius
			float dist = sqrt(pow(target.x - node->point.x, 2) + pow(target.y - node->point.y, 2) + pow(target.z - node->point.z, 2));
			if (dist <= distanceTol)
			{
				ids.push_back(node->id);
			}
		}

		if ((target.data[cdepth] - distanceTol) < node->point.data[cdepth])
		{
			searchrec(node->left, target, distanceTol, depth + 1, ids);
		}
		if ((target.data[cdepth] + distanceTol) > node->point.data[cdepth])
		{
			searchrec(node->right, target, distanceTol, depth + 1, ids);
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;

		searchrec(root, target, distanceTol, 0, ids);

		return ids;
	}
};
