#include "RobotAlgorithm.h"
#include <algorithm>
#include <stack>
#include <string>

const double ERROR_RATE = 0.05;
const double DIST_INF = std::numeric_limits<double>::max() / 2 - 1;
const Direction DIRS_FOR_DFS[4] = { Direction::North, Direction::East, Direction::South, Direction::West };

std::vector<MapNodePtr> nodeList;
std::vector<NodeDistVecPtr> nodeConnection;
MapPosition robotPos;

int usedDir[4];

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

std::shared_ptr<std::string> GetNodeJSON()
{
	std::shared_ptr<std::string> result(new std::string());
	*result += "[\n";

	for (int i = 0; i < nodeList.size(); ++i)
	{
		*result += "\t{\"number\":\"";
		*result += std::to_string(nodeList[i]->NodeNumber);
		*result += "\",";

		*result += "\"position\":\"(";
		*result += std::to_string(nodeList[i]->Position.X);
		*result += ",";
		*result += std::to_string(nodeList[i]->Position.Y);
		*result += ")\",";
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
			AddTurningToQueue(commQueue, LowLevel::GetDir(), dest.Node2->FindDirOf(dest.Node2));
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
