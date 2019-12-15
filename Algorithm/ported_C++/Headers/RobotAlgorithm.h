#pragma once
#include <limits>
#include <queue>
#include <vector>
#include "LowLevel.h"
using namespace HighLevel;

typedef std::vector<NodeDistPair> NodeDistVec;
typedef std::shared_ptr<NodeDistVec> NodeDistVecPtr;

void Init();

MapNodePtr ExploreMap(Junction currJunc, MapNodePtr prev, double errorAdded, Direction cameFrom);
void ExploreDir(Direction dir, MapNodePtr currNode);
void ConnectNode(MapNodePtr currNode, MapNodePtr nextNode, Direction nextDir);
bool IsSameNode(Point p1, Point p2, Junction j1, Junction j2, Point error);

bool FindShortestRoute(MapPosition& dest, std::queue<Command>& commQueue, double& resultDist);
double Dijkstra(int v1, double v1Init, int v2, double v2Init, MapPosition& dest, std::vector<int>& route);

void AddTurningToQueue(std::queue<Command>& q, Direction robotDir, Direction goal);
int CountLeftTurn(Direction before, Direction after);
int CountRightTurn(Direction before, Direction after);
void TurnRobot(Direction dir);
