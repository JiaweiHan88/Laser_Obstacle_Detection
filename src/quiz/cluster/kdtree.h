/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node *root;

	KdTree()
		: root(NULL)
	{
	}

	~KdTree()
	{
		delete root;
	}
	void insertrec(Node *&node, std::vector<float> point, int id, int depth)
	{
		if (node == NULL)
		{
			node = new Node(point, id);
		}
		else
		{
			uint cdepth = depth % 2;

			if (point[cdepth] < node->point[cdepth])
			{
				insertrec(node->left, point, id, depth + 1);
			}
			else
			{
				insertrec(node->right, point, id, depth + 1);
			}
		}
	}
	void insert(std::vector<float> point, int id)
	{
		insertrec(root, point, id, 0);
	}
	void searchrec(Node *node, std::vector<float> target, float distanceTol, int depth, std::vector<int> &ids)
	{
		if (node == NULL)
			return;
		uint cdepth = depth % 2;
		//check if in box
		if (node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol) && node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol))
		{
			//if in radius
			float dist = sqrt(pow(target[0] - node->point[0], 2) + pow(target[1] - node->point[1], 2));
			if (dist <= distanceTol)
			{
				ids.push_back(node->id);
			}
		}

		if ((target[cdepth] - distanceTol) < node->point[cdepth])
		{
			searchrec(node->left, target, distanceTol, depth + 1, ids);
		}
		if ((target[cdepth] + distanceTol) > node->point[cdepth])
		{
			searchrec(node->right, target, distanceTol, depth + 1, ids);
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchrec(root, target, distanceTol, 0, ids);

		return ids;
	}
};
