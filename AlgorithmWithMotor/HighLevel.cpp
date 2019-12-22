#include "HighLevel.h"
#include <ctime>
#include <cstdlib>
#include <cmath>
using namespace HighLevel;

bool RandInitialized = false;

Direction HighLevel::ToOpposite(const Direction dir)
{
	switch (dir)
	{
	case Direction::North:
		return Direction::South;
	case Direction::South:
		return Direction::North;
	case Direction::East:
		return Direction::West;
	case Direction::West:
		return Direction::East;
	default:
		return Direction::None;
	}
}

Direction HighLevel::TurnLeft(const Direction dir)
{
	switch (dir)
	{
	case Direction::North:
		return Direction::West;
	case Direction::South:
		return Direction::East;
	case Direction::East:
		return Direction::North;
	case Direction::West:
		return Direction::South;
	default:
		return Direction::None;
	}
}

Direction HighLevel::TurnRight(const Direction dir)
{
	switch (dir)
	{
	case Direction::North:
		return Direction::East;
	case Direction::South:
		return Direction::West;
	case Direction::East:
		return Direction::South;
	case Direction::West:
		return Direction::North;
	default:
		return Direction::None;
	}
}

double HighLevel::GetError(const double errRate, const double value)
{
	if (!RandInitialized)
	{
		srand((unsigned)time(NULL));
		RandInitialized = true;
	}

	return GetAbsError(errRate, value) * (rand() % 2 == 0 ? -1 : 1);
}

double HighLevel::GetAbsError(const double errRate, const double value)
{
	if (!RandInitialized)
	{
		srand((unsigned)time(NULL));
		RandInitialized = true;
	}

	return abs(value * rand() / (double)RAND_MAX * errRate);
}

HighLevel::Point::Point()
{
	X = Y = 0.0;
}

HighLevel::Point::Point(const double x, const double y)
{
	X = x;
	Y = y;
}

double HighLevel::Point::GetDist(const Point& other) const
{
	return sqrt(pow(X - other.X, 2) + pow(Y - other.Y, 2));
}

double HighLevel::Point::operator-(const Point& other) const
{
	return this->GetDist(other);
}

Point HighLevel::Point::operator+(const double value) const
{
	return { X + value, Y + value };
}

Point HighLevel::Point::operator*(const double value) const
{
	return { X * value, Y * value };
}

HighLevel::Junction::Junction()
{
	North = South = East = West = false;
}

HighLevel::Junction::Junction(bool n, bool s, bool e, bool w)
{
	Reset(n, s, e, w);
}

void HighLevel::Junction::Reset(bool n, bool s, bool e, bool w)
{
	North = n;
	South = s;
	East = e;
	West = w;
}

bool& HighLevel::Junction::operator[](const Direction dir)
{
	switch (dir)
	{
	case Direction::North:
		return North;
	case Direction::South:
		return South;
	case Direction::East:
		return East;
	case Direction::West:
		return West;
	}

	throw "Invalid Direction";
}

NodeType HighLevel::Junction::GetType() const
{
	int routeCount = 0;
	if (North) ++routeCount;
	if (South) ++routeCount;
	if (East) ++routeCount;
	if (West) ++routeCount;

	if (routeCount == 4)
		return NodeType::PlusCross;
	if (routeCount == 3)
		return NodeType::TCross;
	if (routeCount == 2)
	{
		if ((North && South) || (East && West))
			return NodeType::Straight;
		else return NodeType::Corner;
	}
	else return NodeType::EndOfLine;
}

bool HighLevel::Junction::operator==(const Junction& j) const
{
	return North == j.North &&
		South == j.South &&
		East == j.East &&
		West == j.West;
}

bool HighLevel::Junction::operator!=(const Junction& j) const
{
	return !(*this == j);
}

HighLevel::MapNode::~MapNode()
{
	North.reset();
	South.reset();
	East.reset();
	West.reset();
}

MapNodePtr& HighLevel::MapNode::operator[](const Direction dir)
{
	switch (dir)
	{
	case Direction::North:
		return North;
	case Direction::South:
		return South;
	case Direction::East:
		return East;
	case Direction::West:
		return West;
	}

	throw "Invalid Direction";
}

double HighLevel::MapNode::GetDist(const Direction dir) const
{
	switch (dir)
	{
	case Direction::North:
		return NorthDist;
	case Direction::South:
		return SouthDist;
	case Direction::East:
		return EastDist;
	case Direction::West:
		return WestDist;
	}

	throw "Invalid Direction";
}

void HighLevel::MapNode::SetDist(const Direction dir, const double value)
{
	switch (dir)
	{
	case Direction::North:
		NorthDist = value;
		return;
	case Direction::South:
		SouthDist = value;
		return;
	case Direction::East:
		EastDist = value;
		return;
	case Direction::West:
		WestDist = value;
		return;
	}

	throw "Invalid Direction";
}

NodeType HighLevel::MapNode::GetType() const
{
	int routeCount = 0;
	if (North) ++routeCount;
	if (South) ++routeCount;
	if (East) ++routeCount;
	if (West) ++routeCount;

	if (routeCount == 4)
		return NodeType::PlusCross;
	if (routeCount == 3)
		return NodeType::TCross;
	if (routeCount == 2)
	{
		if ((North && South) || (East && West))
			return NodeType::Straight;
		else return NodeType::Corner;
	}
	else return NodeType::EndOfLine;
}

Direction HighLevel::MapNode::FindDirOf(MapNodePtr node) const
{
	if (node == North)
		return Direction::North;
	else if (node == South)
		return Direction::South;
	else if (node == East)
		return Direction::East;
	else if (node == West)
		return Direction::West;

	return Direction::None;
}

HighLevel::MapPosition::~MapPosition()
{
	Node1.reset();
	Node2.reset();
}

double HighLevel::MapPosition::GetDistBetween() const
{
	return Node1->Position.GetDist(Node2->Position);
}

double HighLevel::MapPosition::GetDist1() const
{
	return GetDistBetween() * (DivRatio1 / (DivRatio1 + DivRatio2));
}

double HighLevel::MapPosition::GetDist2() const
{
	return GetDistBetween() * (DivRatio2 / (DivRatio1 + DivRatio2));
}

bool HighLevel::MapPosition::IsSameSection(const MapPosition& mp) const
{
	return (Node1->NodeNumber == mp.Node1->NodeNumber && Node2->NodeNumber == mp.Node2->NodeNumber) ||
		(Node1->NodeNumber == mp.Node2->NodeNumber && Node2->NodeNumber == mp.Node1->NodeNumber);
}

HighLevel::NodeDistPair::~NodeDistPair()
{
	Node.reset();
}

HighLevel::NodeDistPair::NodeDistPair(MapNodePtr node, double dist)
{
	Node = node;
	Dist = dist;
}

bool HighLevel::NodeDistPair::operator==(const NodeDistPair& other) const
{
	return Node->NodeNumber == other.Node->NodeNumber && Dist == other.Dist;
}

HighLevel::Command::~Command()
{
	NextExpect.reset();
}

HighLevel::Command::Command(const Action& act, const double& dist, const MapNodePtr& next)
{
	Act = act;
	Dist = dist;
	NextExpect = next;
}

HighLevel::DijkElement::DijkElement(int index, double cost)
{
	Index = index;
	Cost = cost;
}
