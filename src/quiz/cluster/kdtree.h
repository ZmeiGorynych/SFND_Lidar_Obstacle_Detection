/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	int compare_index;
	Node* left;
	Node* right;

	Node(const std::vector<float>& arr, int setId, int cmpIndex)
	:	point(arr), id(setId), compare_index(cmpIndex), left(nullptr), right(nullptr)
	{}

	~Node(){
	    if(left!= nullptr)
	        delete left;
	    if(right!= nullptr)
	        delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(nullptr)
	{}

	~KdTree(){
	    if(root!= nullptr)
	        delete root;
	}

	void insert(const std::vector<float> & point, int id)
	{
		_insert(root, point, id, 0);
	}

	void _insert(Node*&node, const std::vector<float>& point, int id, int cmpIndex){
	    if(node==nullptr){
	        node = new Node(point, id, cmpIndex);
	    } else if(point[node->compare_index] < node->point[node->compare_index]){
	        _insert(node->left, point, id, (cmpIndex + 1)%point.size());
	    } else {
            _insert(node->right, point, id, (cmpIndex + 1)%point.size());
	    }
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(const std::vector<float>&target, float distanceTol)
	{
        std::vector<int> ids;
        _search(ids, root, target, distanceTol);
        return ids;
	}

    void _search(std::vector<int>& ids, Node* node, const std::vector<float>& target, float distanceTol){
	    int ci = node -> compare_index;
	    if(node->left != nullptr && node->point[ci] > target[ci] - distanceTol){
            _search(ids, node->left, target, distanceTol);
	    }
        if(node-> right != nullptr && node->point[ci] < target[ci] + distanceTol){
            _search(ids, node->right, target, distanceTol);
        }
        float dist = 0;
	    for(int i=0; i<target.size(); i++){
	        float d = target[i] - node->point[i];
	        dist += d*d;
	    }
	    if(dist < distanceTol*distanceTol){
	        ids.push_back(node->id);
	    }
	}
	

};




