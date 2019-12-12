using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotAlgorithm
{
    public class LowLevelMap
    {
        private bool[,] mGraph;

        public LowLevelMap(bool[,] graph)
        {
            mGraph = graph;
        }

        public LowLevelMap(char way, char notWay, string graph)
        {
            int width = -1, height = 0;
            List<List<bool>> tempGraph = new List<List<bool>>();

            using (StringReader sr = new StringReader(graph))
            {
                string line;
                int colPos;

                while ((line = sr.ReadLine()) != null)
                {
                    colPos = 0;
                    tempGraph.Add(new List<bool>());

                    foreach (char ch in line)
                    {
                        if (ch == way)
                            tempGraph[height].Add(true);
                        else if (ch == notWay)
                            tempGraph[height].Add(false);
                        else
                            throw new ArgumentException();

                        ++colPos;
                    }

                    if (width == -1)
                        width = colPos;
                    else if (width != colPos)
                        throw new ArgumentException();

                    ++height;
                }
            }

            mGraph = new bool[height, width];
            for (int row = 0; row < height; ++row)
                for (int col = 0; col < width; ++col)
                    mGraph[row, col] = tempGraph[row][col];
        }

        public int Width => mGraph.GetLength(1);
        public int Height => mGraph.GetLength(0);

        public bool this[int row, int col]
        {
            get
            {
                return mGraph[row, col];
            }
        }

        public bool Available(int row, int col)
        {
            if (row < 0 || row >= Height)
                return false;
            if (col < 0 || col >= Width)
                return false;
            return true;
        }
    }

    public class LowLevelRobot
    {
        public const double ERROR_RATE = 0.05;
        private static Random Rand = new Random();

        int mRealX, mRealY;
        LowLevelMap mMap;

        public double X { get; set; }

        public double Y { get; set; }

        public Direction Dir { get; private set; }

        public LowLevelRobot(LowLevelMap map, int x, int y)
        {
            mMap = map;
            X = mRealX = x;
            Y = mRealY = y;
        }

        public void TurnLeft()
        {
            Dir = Dir.TurnLeft();
        }

        public void TurnRight()
        {
            Dir = Dir.TurnRight();
        }

        public bool Move(int l)
        {
            int dx = 0, dy = 0;

            switch (this.Dir)
            {
                case Direction.North:
                    dy = -l;
                    break;
                case Direction.South:
                    dy = l;
                    break;
                case Direction.East:
                    dx = l;
                    break;
                case Direction.West:
                    dx = -l;
                    break;
                default:
                    return false;
            }

            if (!mMap.Available(mRealY + dy, mRealX + dx))
                return false;
            else if (!mMap[mRealY + dy, mRealX + dx])
                return false;

            double fakeDx = Rand.GetError(ERROR_RATE, dx) + dx;
            double fakeDy = Rand.GetError(ERROR_RATE, dy) + dy;

            Console.Write($"Robot Moved: ({mRealX}, {mRealY}) ");
            mRealX += dx;
            mRealY += dy;
            Console.WriteLine($"-> ({mRealX}, {mRealY})");
            X += fakeDx;
            Y += fakeDy;

            return true;
        }

        public Junction CheckNode()
        {
            Junction llj = new Junction();

            if (mMap[mRealY - 1, mRealX])
                llj.North = true;
            if (mMap[mRealY + 1, mRealX])
                llj.South = true;
            if (mMap[mRealY, mRealX + 1])
                llj.East = true;
            if (mMap[mRealY, mRealX - 1])
                llj.West = true;

            return llj;
        }

        public bool MoveToNode(out Junction junction)
        {
            Junction currJunc = CheckNode();

            Console.WriteLine($"Fake X Changed: ({X}, {Y})");
            do
            {
                if (!Move(1))
                    break;
            } while ((currJunc = CheckNode()).Type == NodeType.Straight);
            Console.WriteLine($"-> ({X}, {Y})");

            junction = currJunc;
            return true;
        }
    }
}
