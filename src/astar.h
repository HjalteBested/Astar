/**	\file astar.h
 * \brief Implementation of the A* search Algorithm.
 *
 * \author Hjalte Bested Møller
 * \date 4. April 2018	
*/

#ifndef HMASTAR_H
#define HMASTAR_H

using namespace std;

const int ALLOW_DIAGONAL_PASSTHROUGH = 1;
const int NODE_FLAG_CLOSED = -1;
const int NODE_FLAG_UNDEFINED = 0;
const int NODE_FLAG_OPEN = 1;

const int NODE_TYPE_ZERO = 0;
const int NODE_TYPE_OBSTACLE = 1;
const int NODE_TYPE_START = 2;
const int NODE_TYPE_END = 3;

const int G_DIRECT_COST   = 100; 
const int G_DIAGONAL_COST = 141;	// ≈ 100 sqrt(2)
const int H_AMITGAIN = 0;

class MapSize {
public:
    unsigned long width = 0;
    unsigned long height = 0;
    unsigned long size = 0;

    MapSize() { }
    MapSize(unsigned long width, unsigned long height) {
        this->width = width;
        this->height = height;
        this->size = width * height;
    }
};

class MapNode {
public:
    int x = -1;	// x-position
    int y = -1; // y-position
    int h = 0;	// Heuristic cost
    int g = 0;  // Cost from start to node
    int type = NODE_TYPE_ZERO;
    int flag = NODE_FLAG_UNDEFINED;
    MapNode *parent = 0;

    MapNode() { }

    MapNode(int x, int y, int type = NODE_TYPE_ZERO, int flag = NODE_FLAG_UNDEFINED, MapNode *parent = 0){
        this->x = x;
        this->y = y;
        this->type = type;
        this->flag = flag;
        this->parent = parent;
    }

    int f() {
        return g + h;
    }
};

class Astar {
public:
	MapSize mapSize;
	vector<MapNode> mapData;
	vector<MapNode *> openList;
	vector<MapNode *> closedList;
	MapNode *startNode;
	MapNode *targetNode;
	
	Astar() { }

    Astar(unsigned long width, unsigned long height) {
    	this->mapSize = MapSize(width,height);
    	this->mapData.resize(mapSize.size);
    	cout << "MapSize(" << mapSize.width << ", " << mapSize.height << ", " << mapSize.size << ")" << endl;
    }

	/**	The standard heuristic is the Manhattan distance. 
	 *	Look at your cost function and see what the least cost is 
	 *	for moving from one space to another. 
	 *	The heuristic should be cost times manhattan distance: */
	inline int manhattan_distance(MapNode* node1, MapNode* node2){
		return abs(node2->x - node1->x) + abs(node2->y - node1->y);
	}

	/**	If on your map you allow diagonal movement, then you need a different heuristic. 
	 *	The Manhattan distance for (4 east, 4 north) will be 8. 
	 *	However, you could simply move (4 northeast) instead, so the heuristic should be 4. 
	 *	This function handles diagonals: */
	inline int diagonal_distance(MapNode* node1, MapNode* node2){
		return max(abs(node2->x - node1->x),abs(node2->y - node1->y));
	}

	/**	If your units can move at any angle (instead of grid directions), 
	 *	then you should probably use a straight line distance: */
	inline int straight_distance(MapNode* node1, MapNode* node2){
		int dx = (node2->x - node1->x);
		int dy = (node2->y - node1->y);
		return sqrt(dx*dx + dy*dy);
	}

	/**	One thing that can lead to poor performance is ties in the heuristic. 
	 *	When several paths have the same f value, they are all explored, even 
	 *	though we only need to explore one of them. To solve this problem, we 
	 *	can add a small tie-breaker to the heuristic. This tie breaker will assign 
	 *  lower cost along the straight line from the starting node to the goal node and
	 *	can give us nicer looking paths. However, it can produce bad results in case of obstacles.
	 *	node1 = start position
	 *	node2 = current position
	 *	node3 = target position */
	inline int amits_modifier(MapNode* node1, MapNode* node2, MapNode* node3){
		int dx1 = node2->x - node3->x;
		int dy1 = node2->y - node3->y;
		int dx2 = node1->x - node3->x;
		int dy2 = node1->y - node3->y;
		int cross = dx1*dy2 - dx2*dy1;
		if( cross<0 ) cross = -cross;
		return cross;
	}

	/** Compute the heuristic cost, or the estimated cost from 
	*	current node (node1) to goal node (node2). */
	inline int computeH(MapNode *node1, MapNode *node2){
	    if (ALLOW_DIAGONAL_PASSTHROUGH) {
	        return diagonal_distance(node1, node2) * G_DIAGONAL_COST;
	    } else {
	        return manhattan_distance(node1, node2) * G_DIRECT_COST;
	    }
	}

	/** Compute the cost from startnode (node1) to current node (node2). */
	inline int computeG(MapNode *node1, MapNode *node2) {
	    int dX = abs(node1->x - node2->x);
	    int dY = abs(node1->y - node2->y);
	    if (dX > dY) {
	        return G_DIAGONAL_COST * dY + G_DIRECT_COST * (dX - dY);
	    } else {
	        return G_DIAGONAL_COST * dX + G_DIRECT_COST * (dY - dX);
	    }
	}

	/** Find the path with the minimal total cost. The actual A* search!  */
	vector<MapNode *> findpath() {
	    vector<MapNode *> path;
	    cout << "Finding started!" << endl;
	    int iteration = 0;
	    MapNode *node;
	    MapNode *reversedPtr = 0;

	    while (openList.size() > 0) {
	        node = openList.at(0);

	        for (int i = 0, max = openList.size(); i < max; i++) {
	            if (openList[i]->f() <= node->f() && openList[i]->h < node->h) {
	                node = openList[i];
	            }
	        }
	        openList.erase(remove(openList.begin(), openList.end(), node), openList.end());
	        node->flag = NODE_FLAG_CLOSED;
	        closedList.push_back(node);

	        cout << iteration++ << endl;
	        cout << "   Current node " << node->x << ", " << node->y << " ..." << endl;
	        if (node->parent != 0)
	            cout << "       ... parent " << node->parent->x << ", " << node->parent->y << endl;
	        if (node == targetNode) {
	            cout << "Reached the target node." << endl;
	            reversedPtr = node;
	            break;
	        }
	        vector<MapNode *> neighborNodes = neighbors(node);
	        cout << "       ... has " << neighborNodes.size() << " neighbors" << endl;
	        for (int i = 0; i < neighborNodes.size(); i++) {
	            MapNode *_node = neighborNodes[i];
	            if (_node->flag == NODE_FLAG_CLOSED || _node->type == NODE_TYPE_OBSTACLE) {
	                continue;
	            }
	            int g = node->g + computeG(_node, node);
	            if (_node->flag == NODE_FLAG_UNDEFINED || g < _node->g) {
	                _node->g = g;
	                _node->h = computeH(_node, targetNode);
	                if(H_AMITGAIN > 0){
	             	   _node->h += amits_modifier(startNode,_node,targetNode)*H_AMITGAIN;
	            	}
	                _node->parent = node;
	                if (_node->flag != NODE_FLAG_OPEN) {
	                    _node->flag = NODE_FLAG_OPEN;
	                    openList.push_back(_node);
	                }
	            }
	        }

	        if (openList.size() <= 0) break;

	    }
	    if (reversedPtr == 0) {
	        cout << "Target node is unreachable." << endl;
	    } else {
	        MapNode *_node = reversedPtr;
	        while (_node->parent != 0) {
	            path.push_back(_node);
	            _node = _node->parent;
	        }
	        reverse(path.begin(), path.end());
	    }
	    return path;
	}

	/** Find all the neighbors to the node and return a vector of MapNode pointers */
	vector<MapNode *> neighbors(MapNode *node){
	    vector<MapNode *> available;
	    MapNode *_node;

	    if ((_node = mapAt(node->x - 1, node->y)) != 0) available.push_back(_node); // L
	    if ((_node = mapAt(node->x, node->y - 1)) != 0) available.push_back(_node); // T
	    if ((_node = mapAt(node->x + 1, node->y)) != 0) available.push_back(_node); // R
	    if ((_node = mapAt(node->x, node->y + 1)) != 0) available.push_back(_node); // B

	    if (ALLOW_DIAGONAL_PASSTHROUGH) {
	        if ((_node = mapAt(node->x - 1, node->y - 1)) != 0) available.push_back(_node); // LT
	        if ((_node = mapAt(node->x + 1, node->y - 1)) != 0) available.push_back(_node); // RT
	        if ((_node = mapAt(node->x + 1, node->y + 1)) != 0) available.push_back(_node); // RB
	        if ((_node = mapAt(node->x - 1, node->y + 1)) != 0) available.push_back(_node); // LB
	    }
	    return available;
	}

	/** Return a pointer to the MapNode at (x,y) */
	MapNode *mapAt(int x, int y){
    	if (x < 0 || y < 0 || x >= mapSize.width || y >= mapSize.height) return 0;
    	return &mapData[y * mapSize.width + x];
	}

	void putNode(MapNode node){
		mapData[node.y * mapSize.width + node.x] = node;
	}

	void printMap(){
	    for (int y = 0; y < mapSize.height; y++) {
	        for (int x = 0; x < mapSize.width; x++) {
	            cout << mapAt(x, y)->type << " ";
	        }
	        cout << endl;
	    }
	}

};



#endif

