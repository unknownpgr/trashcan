#include "RobotAlgorithm.h"
#include "WebInteract.h"
#include <algorithm>
#include <stack>
#include <string>
#include <fstream>
// 리눅스에서 컴파일 시 -pthread 옵션을 추가해야 함.
#include <thread>
#include <iostream>
using namespace WebInteract;

const double ERROR_RATE = 0.05;
const double DIST_INF = std::numeric_limits<double>::max() / 2 - 1;
const Direction DIRS_FOR_DFS[4] = { Direction::North, Direction::East, Direction::South, Direction::West };

std::vector<MapNodePtr> nodeList;
std::vector<NodeDistVecPtr> nodeConnection;
MapPosition robotPos;

int usedDir[4];
bool listening = false;
std::shared_ptr<std::thread> listenThread;

void Init()
{
	Junction currJunc = LowLevel::CheckNode().get();
	MapNodePtr firstNode = ExploreMap(currJunc, nullptr, 0, Direction::None);

	if (nodeList.size() > 1)
	{
		robotPos.Node1 = firstNode;
		robotPos.Node2 = nodeList[1];
		robotPos.DivRatio1 = 0;
		robotPos.DivRatio2 = 1;
	}
	else
		throw "There is only one node. Unable to set robotPos.";
}

void DeleteNodeData()
{
	for (int i = 0; i < nodeConnection.size(); ++i)
	{
		nodeConnection[i]->clear();
		nodeConnection[i].reset();
	}
	nodeConnection.clear();

	for (int i = 0; i < nodeList.size(); ++i)
	{
		nodeList[i]->North.reset();
		nodeList[i]->South.reset();
		nodeList[i]->East.reset();
		nodeList[i]->West.reset();

		nodeList[i].reset();
	}
	nodeList.clear();
}

void ListenFromWeb()
{
	if (!listening)
	{
		listening = true;
		listenThread.reset(new std::thread(TaskOfListening));
	}
}

void TaskOfListening()
{
	std::future<void> controlTask;
	bool running = false;
	std::queue<Command> q;

	while (listening)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		auto data = ReceiveData();

		std::string jsonContent;
		switch (data->first)
		{
		case CommFromWeb::Calibrate:
			std::cout << "Calibrate!" << std::endl;
			// 구현해야 함...
			break;

		case CommFromWeb::Explore:
			std::cout << "Explore!" << std::endl;
			
			if (nodeList.empty())
			{
				Init();
				auto jsonStr = GetNodeJSON();
				WebInteract::SendData("map.json", *jsonStr);

				std::cout << "[DFS Result]" << std::endl;
				for (MapNodePtr node : nodeList)
				{
					std::cout << node->NodeNumber << " : (" << node->Position.X << ", " << node->Position.Y << "), ("
						<< node->Error.X << ", " << node->Error.Y << ")" << std::endl;
				}

				std::cout << std::endl << "[map.json]" << std::endl << *jsonStr << std::endl;
				jsonStr.reset();
			}
			else
			{
				std::cout << "그래프가 이미 탐색되었으므로 명령 무시." << std::endl;
			}
			break;

		case CommFromWeb::GetStatus:
			std::cout << "GetStatus!" << std::endl;

			jsonContent = "{\"isExplored\":";
			if (nodeList.empty())
				jsonContent += "\"0\"}";
			else
				jsonContent += "\"1\"}";
			WebInteract::SendData("status.json", jsonContent);

			std::cout << "[status.json]" << std::endl;
			std::cout << jsonContent << std::endl;
			break;

		case CommFromWeb::Go:
			std::cout << "Go!" << std::endl;

			// 만약 이전에 호출한 ControlRobot_Async가 아직 끝나지 않았다면,
			// 그 함수가 종료될 때까지 기다림.
			while (running)
				std::this_thread::sleep_for(std::chrono::milliseconds(10));

			if (!nodeList.empty())
			{
				MapPosition mp;
				mp.Node1 = nodeList[std::stoi(data->second[0])];
				mp.Node2 = nodeList[std::stoi(data->second[1])];
				mp.DivRatio1 = std::stod(data->second[2]);
				mp.DivRatio2 = (1 - mp.DivRatio1);

				double dist;
				FindShortestRoute(mp, q, dist);
				if (dist != -1)
					controlTask = ControlRobot_Async(&q, &running);
			}
			break;

		case CommFromWeb::Undefined:
		default:
			break;
		}
	}
}

void StopListening()
{
	if (listening)
	{
		listening = false;
		listenThread->join();
		listenThread.reset();
	}
}

bool IsListening()
{
	return listening;
}

std::shared_ptr<std::string> GetNodeJSON()
{
	std::shared_ptr<std::string> result(new std::string());
	*result += "[\n";

	for (int i = 0; i < nodeList.size(); ++i)
	{
		*result += "\t{\"number\":\"";
		*result += std::to_string(nodeList[i]->NodeNumber);
		*result += "\",";

		*result += "\"position\":{";
		*result += "\"x\":\"";
		*result += std::to_string(nodeList[i]->Position.X);
		*result += "\",\"y\":\"";
		*result += std::to_string(nodeList[i]->Position.Y);
		*result += "\"},";
		*result += "\"connected\":[";

		bool written = false;
		for (NodeDistPair ndp : *(nodeConnection[i]))
		{
			if (written)
				*result += ",";
			*result += "\"";
			*result += std::to_string(ndp.Node->NodeNumber);
			*result += "\"";
			written = true;
		}

		*result += "]}";

		if (i != nodeList.size() - 1)
			*result += ",\n";
	}

	*result += "\n]";

	return result;
}

MapNodePtr ExploreMap(Junction currJunc, MapNodePtr prev, double errorAdded, Direction cameFrom)
{
	Point currPoint = LowLevel::GetPos();

	if (prev)
	{
		std::vector<int> nearList;

		for (MapNodePtr& node : nodeList)
		{
			if (IsSameNode(currPoint, node->Position, currJunc, node->Junc, prev->Error + errorAdded))
				nearList.push_back(node->NodeNumber);
		}

		if (!nearList.empty())
		{
			MapNodePtr sameNode = nodeList[nearList[0]];

			for (int nearIndex : nearList)
			{
				MapNodePtr near = nodeList[nearIndex];
				if (sameNode->Position - currPoint > near->Position - currPoint)
					sameNode = near;
			}

			Point newError = sameNode->Error;
			double maxError = std::max(abs(prev->Position.X - sameNode->Position.X), abs(prev->Position.Y - sameNode->Position.Y));
			newError.X += maxError;
			newError.Y += maxError;

			if (newError.X < prev->Error.X || newError.Y < prev->Error.Y)
				prev->Error = newError;

			return sameNode;
		}
	}

	MapNodePtr newNode(new MapNode);
	newNode->Position = currPoint;
	newNode->NodeNumber = nodeList.size();
	newNode->Junc = currJunc;
	newNode->Error = (prev == nullptr ? Point(errorAdded, errorAdded) : prev->Error + errorAdded);

	nodeList.push_back(newNode);
	nodeConnection.push_back(NodeDistVecPtr(new NodeDistVec));

	std::vector<Direction> visitLater;

	for (int i = 0; i < 4; ++i)
	{
		Direction dirToExplore = DIRS_FOR_DFS[i];

		if (currJunc[dirToExplore] && !(cameFrom != Direction::None && cameFrom == dirToExplore) && (*newNode)[dirToExplore] == nullptr)
		{
			if (usedDir[(int)dirToExplore] != 0)
				visitLater.push_back(dirToExplore);
			else
			{
				++usedDir[(int)dirToExplore];
				ExploreDir(dirToExplore, newNode);
				--usedDir[(int)dirToExplore];
			}
		}
	}

	std::sort(visitLater.begin(), visitLater.end(), [](Direction d1, Direction d2) -> bool {return usedDir[(int)d1] < usedDir[(int)d2]; });

	for (int i = 0; i < visitLater.size(); ++i)
	{
		Direction dir = visitLater[i];
		++usedDir[(int)dir];
		ExploreDir(dir, newNode);
		--usedDir[(int)dir];
	}

	return newNode;
}

void ExploreDir(Direction dir, MapNodePtr currNode)
{
	TurnRobot(dir);

	std::pair<bool, Junction> moveResult = LowLevel::MoveToNode().get();
	Junction juncOfNext = moveResult.second;

	double maxError = std::max(abs(LowLevel::GetX() - currNode->Position.X),
		                       abs(LowLevel::GetY() - currNode->Position.Y)) * ERROR_RATE;
	MapNodePtr nextNode = ExploreMap(juncOfNext, currNode, maxError, ToOpposite(dir));

	TurnRobot(ToOpposite(dir));
	auto moveFuture = LowLevel::MoveToNode();

	ConnectNode(currNode, nextNode, dir);
	moveFuture.wait();
}

void ConnectNode(MapNodePtr currNode, MapNodePtr nextNode, Direction nextDir)
{
	(*currNode)[nextDir] = nextNode;
	(*nextNode)[ToOpposite(nextDir)] = currNode;

	double dist = currNode->Position.GetDist(nextNode->Position);

	NodeDistPair currNodeConn = { nextNode, dist };
	NodeDistPair nextNodeConn = { currNode, dist };

	NodeDistVecPtr currNodeConnVec = nodeConnection[currNode->NodeNumber];
	NodeDistVecPtr nextNodeConnVec = nodeConnection[nextNode->NodeNumber];

	if (std::find(currNodeConnVec->begin(), currNodeConnVec->end(), currNodeConn) == currNodeConnVec->end())
		currNodeConnVec->push_back(currNodeConn);

	if (std::find(nextNodeConnVec->begin(), nextNodeConnVec->end(), nextNodeConn) == nextNodeConnVec->end())
		nextNodeConnVec->push_back(nextNodeConn);

	currNode->SetDist(nextDir, dist);
	nextNode->SetDist(ToOpposite(nextDir), dist);
}

bool IsSameNode(Point p1, Point p2, Junction j1, Junction j2, Point error)
{
	return abs(p1.X - p2.X) <= error.X * 2 &&
		abs(p1.Y - p2.Y) <= error.Y * 2 &&
		j1 == j2;
}

bool FindShortestRoute(MapPosition& dest, std::queue<Command>& commQueue, double& resultDist)
{
	if (dest.IsSameSection(robotPos))
	{
		if ((dest.Node1->NodeNumber > dest.Node2->NodeNumber) != (robotPos.Node1->NodeNumber > robotPos.Node2->NodeNumber))
		{
			dest.Node1.swap(dest.Node2);
		}

		if (dest.GetDist1() < robotPos.GetDist1())
		{
			AddTurningToQueue(commQueue, LowLevel::GetDir(), dest.Node2->FindDirOf(dest.Node1));
			commQueue.push(Command(Action::GoStraight, robotPos.GetDist1() - dest.GetDist1(), nullptr));
			resultDist = robotPos.GetDist1() - dest.GetDist1();
		}

		else
		{
			AddTurningToQueue(commQueue, LowLevel::GetDir(), dest.Node1->FindDirOf(dest.Node2));
			commQueue.push(Command(Action::GoStraight, dest.GetDist1() - robotPos.GetDist1(), nullptr));
			resultDist = dest.GetDist1() - robotPos.GetDist1();
		}

		return true;
	}

	std::vector<int> route;

	double dist = Dijkstra(robotPos.Node1->NodeNumber, robotPos.GetDist1(),
		                   robotPos.Node2->NodeNumber, robotPos.GetDist2(),
		                   dest, route);

	if (dist == -1)
	{
		resultDist = -1;
		return false;
	}

	Direction virtualRobotDir;

	// Node1에서 시작하는 경로일 때
	if (route[0] == robotPos.Node1->NodeNumber)
	{
		Direction dirOfNode1 = robotPos.Node2->FindDirOf(robotPos.Node1);
		AddTurningToQueue(commQueue, LowLevel::GetDir(), dirOfNode1);

		virtualRobotDir = dirOfNode1;
	}
	// Node2에서 시작하는 경로일 때
	else
	{
		Direction dirOfNode2 = robotPos.Node1->FindDirOf(robotPos.Node2);
		AddTurningToQueue(commQueue, LowLevel::GetDir(), dirOfNode2);

		virtualRobotDir = dirOfNode2;
	}

	commQueue.push(Command(Action::GoToNextNode, 0, nodeList[route[0]]));

	// route[i]와 route[i + 1]의 위치관계를 파악하여 큐 작성
	for (int i = 0; i < route.size() - 1; ++i)
	{
		Direction nextDir = nodeList[route[i]]->FindDirOf(nodeList[route[i + 1]]);
		AddTurningToQueue(commQueue, virtualRobotDir, nextDir);
		virtualRobotDir = nextDir;
		commQueue.push(Command(Action::GoToNextNode, 0, nodeList[route[i + 1]]));
	}

	AddTurningToQueue(commQueue, virtualRobotDir,
		((route.back()) == dest.Node1->NodeNumber) ? dest.Node1->FindDirOf(dest.Node2) : dest.Node2->FindDirOf(dest.Node1));

	commQueue.push(Command(Action::GoStraight, (route.back() == dest.Node1->NodeNumber ? dest.GetDist1() : dest.GetDist2()), nullptr));

	resultDist = dist;
	return true;
}

double Dijkstra(int v1, double v1Init, int v2, double v2Init, MapPosition& dest, std::vector<int>& route)
{
	std::vector<double> dist(nodeList.size());
	std::vector<int> prev(nodeList.size());
	std::priority_queue<DijkElement, std::vector<DijkElement>, DijkElementComparer> q;

	for (int i = 0; i < nodeList.size(); ++i)
	{
		dist[i] = DIST_INF;
		prev[i] = -1;
	}

	dist[v1] = v1Init;
	dist[v2] = v2Init;

	q.push(DijkElement(v1, v1Init));
	q.push(DijkElement(v2, v2Init));

	while (!q.empty())
	{
		DijkElement now = q.top();
		q.pop();

		if (dist[now.Index] < now.Cost)
			continue;

		for (auto next : *(nodeConnection[now.Index]))
		{
			int nextIndex = next.Node->NodeNumber;
			double nextDist = next.Dist + now.Cost;

			if (dist[nextIndex] > nextDist)
			{
				dist[nextIndex] = nextDist;
				prev[nextIndex] = now.Index;
				q.push(DijkElement(nextIndex, nextDist));
			}
		}
	}

	if (dist[dest.Node1->NodeNumber] == DIST_INF && dist[dest.Node2->NodeNumber] == DIST_INF)
	{
		route.clear();
		return -1;
	}

	std::stack<int> routeTrace;

	double result;
	double n1Dist = dest.GetDist1();
	double n2Dist = dest.GetDist2();
	int currIndex;

	if (n1Dist + dist[dest.Node1->NodeNumber] >= n2Dist + dist[dest.Node2->NodeNumber])
	{
		currIndex = dest.Node2->NodeNumber;
		result = n2Dist + dist[dest.Node2->NodeNumber];
	}
	else
	{
		currIndex = dest.Node1->NodeNumber;
		result = n1Dist + dist[dest.Node1->NodeNumber];
	}

	while (currIndex != v1 && currIndex != v2)
	{
		routeTrace.push(currIndex);
		currIndex = prev[currIndex];
	}
	routeTrace.push(currIndex);

	while (!routeTrace.empty())
	{
		route.push_back(routeTrace.top());
		routeTrace.pop();
	}

	return result;
}

std::future<void> ControlRobot_Async(std::queue<Command>* q, bool* running)
{
	return std::async(std::launch::async, ControlRobot_Raw, q, running);
}

void ControlRobot_Raw(std::queue<Command>* q, bool* running)
{
	*running = true;
	while (!q->empty())
	{
		Command comm = q->front();
		q->pop();

		// 디버그용 출력 코드 -----------------------
		std::cout << "Act = ";
		switch (comm.Act)
		{
		case Action::GoStraight:
			std::cout << "GoStraight";
			break;
		case Action::GoToNextNode:
			std::cout << "GoToNextNode";
			break;
		case Action::TurnLeft:
			std::cout << "TurnLeft";
			break;
		case Action::TurnRight:
			std::cout << "TurnRight";
			break;
		}

		std::cout << ", Dist = " << comm.Dist;

		if (comm.NextExpect != nullptr)
			std::cout << ", NextExpect = " << comm.NextExpect->NodeNumber;

		std::cout << std::endl;
		//----------------------------------------------

		std::cout << "이동 전 로봇 위치: " << robotPos.Node1->NodeNumber << ", " << robotPos.Node2->NodeNumber << ", " << robotPos.DivRatio1 << std::endl;

		Direction currDir = LowLevel::GetDir();
		// 움직이고 난 뒤의 Dist1의 변화량
		double dDist1;
		double dist1, dist2;
		switch (comm.Act)
		{
		case Action::GoStraight:
			// 로봇이 현재 정점 2에 있는데, 가야하는 방향이 구간의 바깥 방향일 때
			if (currDir == robotPos.Node1->FindDirOf(robotPos.Node2) &&
				robotPos.GetDist2() == 0)
			{
				robotPos.Node1 = robotPos.Node2->operator[](currDir);
				robotPos.DivRatio2 = 0;
				robotPos.DivRatio1 = 1;
			}

			else if (currDir == robotPos.Node2->FindDirOf(robotPos.Node1) &&
				robotPos.GetDist1() == 0)
			{
				robotPos.Node2 = robotPos.Node1->operator[](currDir);
				robotPos.DivRatio2 = 1;
				robotPos.DivRatio1 = 0;
			}

			if (currDir == robotPos.Node1->FindDirOf(robotPos.Node2))
				dDist1 = comm.Dist;
			else dDist1 = -comm.Dist;

			dist1 = robotPos.GetDist1() + dDist1;
			dist2 = robotPos.GetDist2() - dDist1;
			robotPos.DivRatio1 = dist1 / (dist1 + dist2);
			robotPos.DivRatio2 = dist2 / (dist1 + dist2);
			LowLevel::Move(comm.Dist).wait();
			break;

		case Action::GoToNextNode:
			// 로봇이 1번 정점 위에 있을 때
			if (robotPos.GetDist1() == 0)
			{
				robotPos.Node2 = robotPos.Node1->operator[](currDir);
				robotPos.DivRatio2 = 0;
				robotPos.DivRatio1 = 1;
			}
			// 로봇이 2번 정점 위에 있을 때
			else if (robotPos.GetDist2() == 0)
			{
				robotPos.Node1 = robotPos.Node2->operator[](currDir);
				robotPos.DivRatio2 = 1;
				robotPos.DivRatio1 = 0;
			}
			// 로봇이 두 정점 사이에 있을 때
			else
			{
				if (currDir == robotPos.Node1->FindDirOf(robotPos.Node2))
				{
					robotPos.DivRatio1 = 1;
					robotPos.DivRatio2 = 0;
				}
				else if (currDir == robotPos.Node2->FindDirOf(robotPos.Node1))
				{
					robotPos.DivRatio1 = 0;
					robotPos.DivRatio2 = 1;
				}
				else
				{
					// 두 정점 사이에 있는데 두 정점 중 하나를 바라보고 있지 않은 예외 상황.
					// 예외 처리 코드를 필요하다면 작성할 것.
				}
			}
			LowLevel::MoveToNode().wait();
			break;

		case Action::TurnLeft:
			LowLevel::TurnLeft().wait();
			break;

		case Action::TurnRight:
			LowLevel::TurnRight().wait();
			break;
		}
		std::cout << "이동 후 로봇 위치: " << robotPos.Node1->NodeNumber << ", " << robotPos.Node2->NodeNumber << ", " << robotPos.DivRatio1 << std::endl;
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	*running = false;
}

void AddTurningToQueue(std::queue<Command>& q, Direction robotDir, Direction goal)
{
	if (goal != robotDir)
	{
		int leftCount = CountLeftTurn(robotDir, goal);
		int rightCount = CountRightTurn(robotDir, goal);

		if (leftCount <= rightCount)
			for (int i = 0; i < leftCount; ++i)
				q.push(Command(Action::TurnLeft, 0, nullptr));

		else q.push(Command(Action::TurnRight, 0, nullptr));
	}
}

int CountLeftTurn(Direction before, Direction after)
{
	if (before == after)
		return 0;

	int count = 1;
	while ((before = TurnLeft(before)) != after)
		++count;

	return count;
}

int CountRightTurn(Direction before, Direction after)
{
	if (before == after)
		return 0;

	int count = 1;
	while ((before = TurnRight(before)) != after)
		++count;

	return count;
}

void TurnRobot(Direction dir)
{
	if (dir != LowLevel::GetDir())
	{
		int leftCount = CountLeftTurn(LowLevel::GetDir(), dir);
		int rightCount = CountRightTurn(LowLevel::GetDir(), dir);

		if (leftCount <= rightCount)
			for (int i = 0; i < leftCount; ++i)
				LowLevel::TurnLeft().wait();

		else LowLevel::TurnRight().wait();
	}
}
