/* \author Aaron Brown */
// Quiz on implementing kd tree

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

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insert(&root, point, id, 0);
	}
	void insert(Node **node, std::vector<float> point, int id, uint depth){
		uint cd = depth %2;
		if(*node == NULL){
        *node = new Node(point, id);
		}else if ((*node)->point[cd]>point[cd])
		{
			insert(&(*node)->left, point, id, depth+1);
		}else{
			insert(&(*node)->right, point, id, depth+1);
		}
	  
	}


	void search(Node *node, std::vector<float> target, int depth, float distancetol, std::vector<int> &ids){
		if(node !=NULL){
			if(node->point[0]>=target[0]-distancetol && node->point[0]<=target[0]+distancetol && node->point[1]>=target[1]-distancetol && node->point[1]<=target[1]+distancetol){
				float deltax = (node->point[0] - target[0])*(node->point[0] - target[0]);
				float deltay = (node->point[1] - target[1])*(node->point[1] - target[1]);
				float distance = sqrt(deltax+deltay);
				if(distance<distancetol)
					ids.push_back(node->id);
			}
			int cd = depth %2;
			if(node->point[cd]>=target[cd]-distancetol){
				search(node->left, target, depth+1, distancetol, ids);
			}
			if(node->point[cd]<=target[cd]+distancetol){
				search(node->right, target, depth+1, distancetol, ids);
			}
			
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search(root, target, 0, distanceTol, ids);
		return ids;
	}
	

};




