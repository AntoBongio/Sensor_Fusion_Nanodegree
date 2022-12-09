/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <math.h>

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

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insert(std::vector<float> point, int id)
	{
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);
	}

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id) 
	{
		if(*node == NULL)
		{
			*node = new Node(point, id);
		}
		else
		{
			// Calculate current depth
			uint cd = depth % 2;

			if(point[cd] < ((*node)->point[cd]))
				insertHelper(&((*node)->left), depth+1, point, id);
			else
				insertHelper(&((*node)->right), depth+1, point, id); 	
		}		
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}

	inline float distance(std::vector<float> point1, std::vector<float> point2)
	{
		return std::sqrt(std::pow(point1[0] - point2[0], 2) + std::pow(point1[1] - point2[1], 2));
	}

	void searchHelper(std::vector<float> target, Node* node, uint depth, float distanceTol, std::vector<int>& ids)
	{
		if(node != NULL)
		{
			if((target[0] - distanceTol <= node->point[0]) and (target[0] + distanceTol >= node->point[0]))
			{
				if((target[1] - distanceTol <= node->point[1]) and (target[1] + distanceTol >= node->point[1]))
				{
					if (distance(target, node->point) <= distanceTol)
					{
						ids.push_back(node->id);
					}
				}
			}
			uint cd = depth % 2;
			/* 
			if(target[cd] - distanceTol < node->point[cd]) means:
			The point considered is on the left of the node->point, so there's 
			more room for possible points. So keep searching in this direction.			
			*/
			if(target[cd] - distanceTol < node->point[cd])
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			if(target[cd] + distanceTol > node->point[cd])
				searchHelper(target, node->right, depth+1, distanceTol, ids); 	
		
		}
	}
};