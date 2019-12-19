#pragma once
#include <limits>
#include <queue>
#include <vector>
#include <string>
#include "LowLevel.h"
using namespace HighLevel;

typedef std::vector<NodeDistPair> NodeDistVec;
typedef std::shared_ptr<NodeDistVec> NodeDistVecPtr;

void Init();
void DeleteNodeData();

// 웹서버와 파일 통신을 하기 위해 새로운 쓰레드를 생성 및 대기시킴.
void ListenFromWeb();
// ListenFromWeb에서 생성한 쓰레드가 실행하는 함수. 지속적으로 서버로부터 오는 명령을 주시함.
void TaskOfListening();
// 웹서버와 통신을 중단함. (생성했던 쓰레드 종료)
void StopListening();
bool IsListening();

std::shared_ptr<std::string> GetNodeJSON();

MapNodePtr ExploreMap(Junction currJunc, MapNodePtr prev, double errorAdded, Direction cameFrom);
void ExploreDir(Direction dir, MapNodePtr currNode);
void ConnectNode(MapNodePtr currNode, MapNodePtr nextNode, Direction nextDir);
bool IsSameNode(Point p1, Point p2, Junction j1, Junction j2, Point error);

bool FindShortestRoute(MapPosition& dest, std::queue<Command>& commQueue, double& resultDist);
double Dijkstra(int v1, double v1Init, int v2, double v2Init, MapPosition& dest, std::vector<int>& route);
// 큐에 있는 명령대로 로봇을 움직임.
std::future<void> ControlRobot_Async(std::queue<Command>* q, bool* running);
void ControlRobot_Raw(std::queue<Command>* q, bool* running);

void AddTurningToQueue(std::queue<Command>& q, Direction robotDir, Direction goal);
int CountLeftTurn(Direction before, Direction after);
int CountRightTurn(Direction before, Direction after);
void TurnRobot(Direction dir);
