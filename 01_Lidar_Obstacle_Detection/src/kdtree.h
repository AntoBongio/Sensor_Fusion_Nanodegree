/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"
#include <math.h>

// Structure to represent node of kd tree
template <typename PointT> struct Node
{
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT point, int setId)
	:	point(point), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

template <typename PointT> struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insert(PointT point, int id)
	{
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);
	}

	void insertHelper(Node<PointT>** node, uint depth, PointT point, int id) 
	{
		if(*node == NULL)
		{
			*node = new Node<PointT>(point, id);
		}
		else
		{
			// Calculate current depth
			uint cd = depth % 3;

			switch(cd)
			{
				case 0:
					if(point.x < (*node)->point.x)
						insertHelper(&((*node)->left), depth+1, point, id);
					else
						insertHelper(&((*node)->right), depth+1, point, id); 	
					break;
				case 1:
					if(point.y < (*node)->point.y)
						insertHelper(&((*node)->left), depth+1, point, id);
					else
						insertHelper(&((*node)->right), depth+1, point, id); 	
					break;
				case 2:
					if(point.z < (*node)->point.z)
						insertHelper(&((*node)->left), depth+1, point, id);
					else
						insertHelper(&((*node)->right), depth+1, point, id); 	
					break;
			}
		}		
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}

	inline float distance(PointT point1, PointT point2)
	{
		return std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2) + std::pow(point1.z - point2.z, 2));
	}

	void searchHelper(PointT target, Node<PointT>* node, uint depth, float distanceTol, std::vector<int>& ids)
	{
		if(node != NULL)
		{
			if((target.x - distanceTol <= node->point.x) and (target.x + distanceTol >= node->point.x) and
				(target.y - distanceTol <= node->point.y) and (target.y + distanceTol >= node->point.y) and
				(target.z - distanceTol <= node->point.z) and (target.z + distanceTol >= node->point.z))
			{
				if (distance(target, node->point) <= distanceTol)
					ids.push_back(node->id);
			}
			uint cd = depth % 3;

			switch(cd)
			{
				case 0:
					if(target.x - distanceTol < node->point.x)
						searchHelper(target, node->left, depth+1, distanceTol, ids);
					if(target.x + distanceTol > node->point.x)
						searchHelper(target, node->right, depth+1, distanceTol, ids); 	
					break;
				case 1:
					if(target.y - distanceTol < node->point.y)
						searchHelper(target, node->left, depth+1, distanceTol, ids);
					if(target.y + distanceTol > node->point.y)
						searchHelper(target, node->right, depth+1, distanceTol, ids); 		
					break;
				case 2:
					if(target.z - distanceTol < node->point.z)
						searchHelper(target, node->left, depth+1, distanceTol, ids);
					if(target.z + distanceTol > node->point.z)
						searchHelper(target, node->right, depth+1, distanceTol, ids); 	
					break;
			}
		}
	}
};

