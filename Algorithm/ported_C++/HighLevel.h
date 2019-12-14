#pragma once
namespace HighLevel
{
	enum Action { GoToNextNode, GoStraight, TurnRight, TurnLeft };
	enum NodeType { EndOfLine, PlusCross, TCross, Corner, Straight };
	enum Direction { North = 0, South = 1, East = 2, West = 3, None = -1 };


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

		bool& operator[](const Direction dir);
		NodeType GetType() const;

		bool operator==(const Junction& j) const;
		bool operator!=(const Junction& j) const;
	};

	class MapNode
	{
	public:
		int NodeNumber;
		Point Position;
		Point Error;
		Junction Junc;

		MapNode* North, * South, * East, * West;
		double NorthDist, SouthDist, EastDist, WestDist;

		MapNode*& operator[](const Direction dir);

		double GetDist(const Direction dir) const;
		void SetDist(const Direction dir, const double value);

		NodeType GetType() const;
		Direction FindDirOf(MapNode* node) const;
	};

	struct Command
	{
		Action Act;
		double Dist;
		MapNode* NextExpect;
	};

	struct MapPosition
	{
	public:
		MapNode* Node1;
		MapNode* Node2;
		double DivRatio1;
		double DivRatio2;

		double GetDistBetween() const;
		double GetDist1() const;
		double GetDist2() const;

		bool IsSameSection(const MapPosition& mp) const;
	};

	struct NodeDistPair
	{
		MapNode* Node;
		double Dist;

		NodeDistPair(MapNode* node, double dist);
		bool operator==(const NodeDistPair& other) const;
	};

	struct DijkElement
	{
	public:
		int Index;
		double Cost;

		DijkElement(int index, double cost);

		static bool Compare(const DijkElement& d1, const DijkElement& d2);
	};
}
