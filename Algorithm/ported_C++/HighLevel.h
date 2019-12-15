#pragma once
#include <memory>
#define MapNodePtr std::shared_ptr<MapNode>

namespace HighLevel
{
	enum class Action { GoToNextNode, GoStraight, TurnRight, TurnLeft };
	enum class NodeType { EndOfLine, PlusCross, TCross, Corner, Straight };
	enum class Direction { North = 0, South = 1, East = 2, West = 3, None = -1 };

	Direction ToOpposite(const Direction dir);
	Direction TurnLeft(const Direction dir);
	Direction TurnRight(const Direction dir);

	double GetError(const double errRate, const double value);
	double GetAbsError(const double errRate, const double value);

	struct Point
	{
		double X, Y;

		Point();
		Point(const double x, const double y);
		double GetDist(const Point& other) const;
		double operator-(const Point& other) const;
		Point operator+(const double value) const;
		Point operator*(const double value) const;
	};

	struct Junction
	{
		bool North, South, East, West;

		Junction();

		bool& operator[](const Direction dir);
		NodeType GetType() const;

		bool operator==(const Junction& j) const;
		bool operator!=(const Junction& j) const;
	};

	class MapNode
	{
	public:
		int NodeNumber = -1;
		Point Position;
		Point Error;
		Junction Junc;

		MapNodePtr North, South, East, West;
		double NorthDist = 0, SouthDist = 0, EastDist = 0, WestDist = 0;

		~MapNode();

		MapNodePtr& operator[](const Direction dir);

		double GetDist(const Direction dir) const;
		void SetDist(const Direction dir, const double value);

		NodeType GetType() const;
		Direction FindDirOf(MapNodePtr node) const;
	};

	struct Command
	{
		Action Act;
		double Dist;
		MapNodePtr NextExpect;

		~Command();
		Command(const Action& act, const double& dist, const MapNodePtr& next);
	};

	struct MapPosition
	{
	public:
		MapNodePtr Node1;
		MapNodePtr Node2;
		double DivRatio1 = 0;
		double DivRatio2 = 0;

		~MapPosition();

		double GetDistBetween() const;
		double GetDist1() const;
		double GetDist2() const;

		bool IsSameSection(const MapPosition& mp) const;
	};

	struct NodeDistPair
	{
		MapNodePtr Node;
		double Dist;

		~NodeDistPair();
		NodeDistPair(MapNodePtr node, double dist);

		bool operator==(const NodeDistPair& other) const;
	};

	struct DijkElement
	{
	public:
		int Index;
		double Cost;

		DijkElement(int index, double cost);
	};

	struct DijkElementComparer
	{
		bool operator()(const DijkElement& d1, const DijkElement& d2) const
		{
			return d1.Cost > d2.Cost;
		}
	};
}
