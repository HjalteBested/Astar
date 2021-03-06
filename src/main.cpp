#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include "astar.hpp"

using namespace std;
using namespace cv;

const string M50_0[] = {
        "assets/Map50_0.bmp",
        "assets/Map50_0_Out.png",
        "assets/Map50_0_Path.png"
};

const string M50_1[] = {
        "assets/Map50_1.bmp",
        "assets/Map50_1_Out.png",
        "assets/Map50_1_Path.png"
};

const string M50_2[] = {
        "assets/Map50_2.bmp",
        "assets/Map50_2_Out.png",
        "assets/Map50_2_Path.png"
};

const string M50_3[] = {
        "assets/Map50_3.bmp",
        "assets/Map50_3_Out.png",
        "assets/Map50_3_Path.png"
};

const string *FILE_PATH = M50_1;

Astar myAstar;
Mat map;

void drawPath(Mat &map, vector<MapNode *> path);
void drawOpenList();
void drawClosedList();

int main(){
    map = imread(FILE_PATH[0]);
    // cout << "This is the map:\n" << map << endl;
    Mat resized;
    resize(map, resized, Size(500, 500), 0, 0, INTER_NEAREST);
    imwrite(FILE_PATH[1], resized);

    myAstar = Astar(map.cols, map.rows);

    for (int y = 0; y < map.rows; y++) {
        for (int x = 0; x < map.cols; x++) {
            if (map.at<Vec3b>(y, x) == Vec3b(255, 255, 255)) {
                myAstar.putNode(MapNode(x, y, NODE_TYPE_ZERO));
            } else if (map.at<Vec3b>(y, x) == Vec3b(0, 0, 0)) {
                myAstar.putNode(MapNode(x, y, NODE_TYPE_OBSTACLE));
            } else if (map.at<Vec3b>(y, x) == Vec3b(255, 0, 0)) {
                MapNode node(x, y, NODE_TYPE_START);
                myAstar.putNode(node);
                myAstar.startNode = myAstar.mapAt(x,y);
            } else if (map.at<Vec3b>(y, x) == Vec3b(0, 0, 255)) {
                MapNode node(x, y, NODE_TYPE_END);
                myAstar.putNode(node);
                myAstar.targetNode = myAstar.mapAt(x,y);
            } else {
                map.at<Vec3b>(y, x) = Vec3b(0, 0, 0);
                myAstar.putNode(MapNode(x, y, NODE_TYPE_OBSTACLE));
                cout << "***  ** iT Happened **  **** \n";
            }
        }
    }

    // myAstar.printMap();

    cout << "startNode=(" << myAstar.startNode->x  << ", " << myAstar.startNode->y << ")" << endl;
    cout << "endNode=("   << myAstar.targetNode->x << ", " << myAstar.targetNode->y << ")" << endl;

    myAstar.openList.push_back(myAstar.startNode);
    vector<MapNode *> path = myAstar.findpath();
    drawOpenList();
    drawClosedList();
    drawPath(map, path);
    imwrite(FILE_PATH[2], map);

    myAstar.clear();
    return 0;
}

void drawPath(Mat &map, vector<MapNode *> path) {
    cvtColor(map, map, COLOR_BGR2HSV);
    cout << "(" << myAstar.startNode->x << "," << myAstar.startNode->y << ")";
    for (uint i = 0; i < path.size() - 1; i++) {
        MapNode *node = path[i];
        map.at<Vec3b>(node->y, node->x) = Vec3b(20 + (1.0 - ((double) i / path.size())) * 80, 200, 255);
        cout << "->(" << node->x << "," << node->y << ")";
    }
    cout << "->(" << myAstar.targetNode->x << "," << myAstar.targetNode->y << ")" << endl;

    cvtColor(map, map, COLOR_HSV2BGR);
    resize(map, map, Size(500, 500), 0, 0, INTER_NEAREST);
}

void drawOpenList() {
    for (uint i = 0; i < myAstar.openList.size(); i++) {
        MapNode *node = myAstar.openList[i];
        if (node == myAstar.startNode || node == myAstar.targetNode) continue;
        map.at<Vec3b>(node->y, node->x) = Vec3b(210, 210, 210);
    }
}

void drawClosedList() {
    for (uint i = 0; i < myAstar.closedList.size(); i++) {
        MapNode *node = myAstar.closedList[i];
        if (node == myAstar.startNode || node == myAstar.targetNode) continue;
        map.at<Vec3b>(node->y, node->x) = Vec3b(210, 210, 210);
    }
}
