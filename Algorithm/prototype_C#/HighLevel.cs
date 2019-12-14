using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotAlgorithm
{
    public enum Action { GoToNextNode, GoStraight, TurnRight, TurnLeft };
    public enum NodeType { EndOfLine, PlusCross, TCross, Corner, Straight };
    public enum Direction { North = 0, South = 1, East = 2, West = 3 };

    public static class DirectionExt
    {
        public static Direction ToOpposite(this Direction dir)
        {
            switch (dir)
            {
                case Direction.North:
                    return Direction.South;
                case Direction.South:
                    return Direction.North;
                case Direction.East:
                    return Direction.West;
                case Direction.West:
                    return Direction.East;
            }

            throw new ArgumentException();
        }

        public static Direction TurnLeft(this Direction dir)
        {
            switch (dir)
            {
                case Direction.North:
                    return Direction.West;
                case Direction.South:
                    return Direction.East;
                case Direction.East:
                    return Direction.North;
                case Direction.West:
                    return Direction.South;
            }

            throw new ArgumentException();
        }

        public static Direction TurnRight(this Direction dir)
        {
            switch (dir)
            {
                case Direction.North:
                    return Direction.East;
                case Direction.South:
                    return Direction.West;
                case Direction.East:
                    return Direction.South;
                case Direction.West:
                    return Direction.North;
            }

            throw new ArgumentException();
        }
    }

    public static class RandomExt
    {
        public static double GetError(this Random rand, double errRate, double value)
        {
            return GetAbsError(rand, errRate, value) * (rand.Next() % 2 == 0 ? -1 : 1);
        }

        public static double GetAbsError(this Random rand, double errRate, double value)
        {
            return Math.Abs(value * rand.NextDouble() * errRate);
        }
    }

    public struct Junction : IEquatable<Junction>
    {
        public bool North { get; set; }
        public bool South { get; set; }
        public bool East { get; set; }
        public bool West { get; set; }

        public bool this[Direction dir]
        {
            get
            {
                switch (dir)
                {
                    case Direction.North:
                        return North;
                    case Direction.South:
                        return South;
                    case Direction.East:
                        return East;
                    case Direction.West:
                        return West;
                }

                throw new ArgumentException();
            }
            set
            {
                switch (dir)
                {
                    case Direction.North:
                        North = value;
                        break;
                    case Direction.South:
                        South = value;
                        break;
                    case Direction.East:
                        East = value;
                        break;
                    case Direction.West:
                        West = value;
                        break;
                }
            }
        }

        public NodeType Type
        {
            get
            {
                int routeCount = 0;
                if (North) ++routeCount;
                if (South) ++routeCount;
                if (East) ++routeCount;
                if (West) ++routeCount;

                if (routeCount == 4)
                    return NodeType.PlusCross;
                if (routeCount == 3)
                    return NodeType.TCross;
                if (routeCount == 2)
                {
                    if ((North && South) || (East && West))
                        return NodeType.Straight;
                    else return NodeType.Corner;
                }
                else return NodeType.EndOfLine;
            }
        }

        public static bool operator ==(Junction l1, Junction l2)
        {
            return l1.North == l2.North &&
                   l1.South == l2.South &&
                   l1.East == l2.East &&
                   l1.West == l2.West;
        }

        public static bool operator !=(Junction l1, Junction l2)
        {
            return !(l1 == l2);
        }

        public bool Equals(Junction other)
        {
            return this == other;
        }
    }


    public struct Command
    {
        public Action Act { get; set; }
        public double Dist { get; set; }
        public MapNode NextExpect { get; set; }

        public override string ToString()
        {
            return $"Action:{Act.ToString()}, Dist:{Dist}, NextExpect:{(NextExpect == null ? "Null" : NextExpect.NodeNumber.ToString())}";
        }
    }

    public class MapNode : IEquatable<MapNode>
    {
        public int NodeNumber { get; set; }
        public Point Position { get; set; }
        public Point Error { get; set; }
        public Junction Junction { get; set; }

        public MapNode North { get; set; }
        public MapNode South { get; set; }
        public MapNode East { get; set; }
        public MapNode West { get; set; }
        public double NorthDist { get; set; } = -1;
        public double SouthDist { get; set; } = -1;
        public double EastDist { get; set; } = -1;
        public double WestDist { get; set; } = -1;

        public MapNode this[Direction dir]
        {
            get
            {
                switch (dir)
                {
                    case Direction.North:
                        return North;
                    case Direction.South:
                        return South;
                    case Direction.East:
                        return East;
                    case Direction.West:
                        return West;
                }

                throw new ArgumentException();
            }

            set
            {
                switch (dir)
                {
                    case Direction.North:
                        North = value;
                        break;
                    case Direction.South:
                        South = value;
                        break;
                    case Direction.East:
                        East = value;
                        break;
                    case Direction.West:
                        West = value;
                        break;
                }
            }
        }

        public double GetDist(Direction dir)
        {
            switch (dir)
            {
                case Direction.North:
                    return NorthDist;
                case Direction.South:
                    return SouthDist;
                case Direction.East:
                    return EastDist;
                case Direction.West:
                    return WestDist;
            }

            throw new ArgumentException();
        }

        public void SetDist(Direction dir, double value)
        {
            switch (dir)
            {
                case Direction.North:
                    NorthDist = value;
                    break;
                case Direction.South:
                    SouthDist = value;
                    break;
                case Direction.East:
                    EastDist = value;
                    break;
                case Direction.West:
                    WestDist = value;
                    break;
            }
        }

        public NodeType Type
        {
            get
            {
                int routeCount = 0;
                if (North != null) ++routeCount;
                if (South != null) ++routeCount;
                if (East != null) ++routeCount;
                if (West != null) ++routeCount;

                if (routeCount == 4)
                    return NodeType.PlusCross;
                if (routeCount == 3)
                    return NodeType.TCross;
                if (routeCount == 2)
                {
                    if ((North != null && South != null) || (East != null && West != null))
                        return NodeType.Straight;
                    else return NodeType.Corner;
                }
                else return NodeType.EndOfLine;
            }
        }

        public Direction? FindDirOf(MapNode node)
        {
            if (node == North)
                return Direction.North;
            else if (node == South)
                return Direction.South;
            else if (node == East)
                return Direction.East;
            else if (node == West)
                return Direction.West;

            return null;
        }

        public override string ToString()
        {
            return $"{NodeNumber}: {Type}, {Position.ToString()}";
        }

        public bool Equals(MapNode other)
        {
            return NodeNumber == other.NodeNumber;
        }
    }

    public class MapPosition
    {
        public MapNode Node1 { get; set; }
        public MapNode Node2 { get; set; }
        public double DivRatio1 { get; set; }
        public double DivRatio2 { get; set; }

        public double DistBetween => Node1.Position.GetDist(Node2.Position);

        public double Dist1 => DistBetween * (DivRatio1 / (DivRatio1 + DivRatio2));

        public double Dist2 => DistBetween * (DivRatio2 / (DivRatio1 + DivRatio2));

        public bool IsSameSection(MapPosition mp)
        {
            return (Node1.NodeNumber == mp.Node1.NodeNumber && Node2.NodeNumber == mp.Node2.NodeNumber) ||
                   (Node1.NodeNumber == mp.Node2.NodeNumber && Node2.NodeNumber == mp.Node1.NodeNumber);
        }
    }

    public struct Point
    {
        public Point(double x, double y)
        {
            X = x;
            Y = y;
        }

        public double X { get; set; }
        public double Y { get; set; }

        public double GetDist(Point other)
        {
            return Math.Sqrt(Math.Pow(X - other.X, 2) + Math.Pow(Y - other.Y, 2));
        }

        public override string ToString()
        {
            return $"({Math.Round(X, 3)}, {Math.Round(Y, 3)})";
        }

        public static double operator-(Point p1, Point p2)
        {
            return Math.Sqrt(Math.Pow(p1.X - p2.X, 2) + Math.Pow(p1.Y - p2.Y, 2));
        }

        public static Point operator+(Point p, double value)
        {
            return new Point(p.X + value, p.Y + value);
        }

        public static Point operator*(Point p, double value)
        {
            return new Point(p.X * value, p.Y * value);
        }
    }

    public struct NodeDistPair : IEquatable<NodeDistPair>
    {
        public NodeDistPair(MapNode node, double dist)
        {
            Node = node ?? throw new ArgumentNullException(nameof(node));
            Dist = dist;
        }

        public MapNode Node { get; set; }
        public double Dist { get; set; }

        public bool Equals(NodeDistPair other)
        {
            return Node.NodeNumber == other.Node.NodeNumber && Dist == other.Dist;
        }
    }

    public class DijkElement : IComparable
    {
        public DijkElement(int index, double cost)
        {
            Index = index;
            Cost = cost;
        }

        public int Index { get; set; }
        public double Cost { get; set; }

        public int CompareTo(object obj)
        {
            if (obj is DijkElement)
                return CompareTo(obj as DijkElement);

            throw new ArgumentException();
        }

        public int CompareTo(DijkElement dij)
        {
            return Cost.CompareTo(dij.Cost);
        }

        public static bool operator <(DijkElement d1, DijkElement d2)
        {
            return d1.Cost < d2.Cost;
        }

        public static bool operator >(DijkElement d1, DijkElement d2)
        {
            return d1.Cost > d2.Cost;
        }
    }

}
