using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace RobotAlgorithm
{
    class Program
    {
        const double ERROR_RATE = 0.05;
        const double DIST_INF = double.MaxValue / 2 - 1;
        static readonly Direction[] DIRS_FOR_DFS = new Direction[4] { Direction.North, Direction.East, Direction.South, Direction.West };

        static LowLevelMap lowMap;
        static LowLevelRobot lowRobot;

        static List<MapNode> nodeList = new List<MapNode>();
        static List<List<NodeDistPair>> nodeConnection = new List<List<NodeDistPair>>();
        static MapPosition robotPos = new MapPosition();

        // DFS 탐색에서 갔던 방향을 저장.
        // 이전에 갔던 방향에 대해서는 나중에 탐색하도록 함.
        // 이렇게 하는 이유는 로봇이 가급적 둥글게 탐색하게 함으로써
        // 오차를 최대한 줄이기 위함임.
        static int[] usedDir = new int[4];

        /// <summary>
        /// 로봇이 처음 실행될 때 라인의 전체 경로를 파악하기 위해 DFS 탐색을 함.
        /// </summary>
        static MapNode ExploreMap(Junction currJunc, MapNode prev, double errorAdded, Direction? cameFrom)
        {
            Console.WriteLine("ExploreMap");

            Point currPoint = new Point { X = lowRobot.X, Y = lowRobot.Y };
            Console.WriteLine($"{currPoint}, {(prev == null ? "(Null)" : (prev.Error + errorAdded).ToString())}, {errorAdded}");

            if (prev != null)
            {
                List<MapNode> nearList = new List<MapNode>();
                foreach (MapNode node in nodeList)
                {
                    if (IsSameNode(currPoint, node.Position, currJunc, node.Junction, prev.Error + errorAdded))
                    {
                        nearList.Add(node);
                    }
                }

                // 현재 위치한 정점이 이미 방문한 정점일 때, 로봇의 위치 정보와 오차를 조정하고 return.
                if (nearList.Count > 0)
                {
                    MapNode sameNode = nearList[0];

                    foreach (MapNode near in nearList)
                    {
                        if (sameNode.Position - currPoint > near.Position - currPoint)
                            sameNode = near;
                    }

                    Console.WriteLine($"FoundSame: [{sameNode}]");
                    Console.WriteLine($"(nearList: {string.Join("/", nearList)})");

                    Point newError = sameNode.Error;
                    double maxError = Math.Max(Math.Abs(prev.Position.X - sameNode.Position.X), Math.Abs(prev.Position.Y - sameNode.Position.Y));
                    newError.X += maxError;
                    newError.Y += maxError;

                    if (newError.X < prev.Error.X || newError.Y < prev.Error.Y)
                        prev.Error = newError;

                    return sameNode;
                }
            }

            MapNode newNode = new MapNode();
            newNode.Position = currPoint;
            newNode.NodeNumber = nodeList.Count;
            newNode.Junction = currJunc;
            newNode.Error = prev == null ? new Point(errorAdded, errorAdded) : prev.Error + errorAdded;
            nodeList.Add(newNode);
            nodeConnection.Add(new List<NodeDistPair>());
            Console.WriteLine("Node Added: " + newNode);

            List<Direction> visitLater = new List<Direction>();

            foreach (Direction dirToExplore in DIRS_FOR_DFS)
            {
                if (currJunc[dirToExplore] && !(cameFrom != null && cameFrom.Value == dirToExplore) && newNode[dirToExplore] == null)
                {
                    if (usedDir[(int)dirToExplore] != 0)
                        visitLater.Add(dirToExplore);
                    else
                    {
                        ++usedDir[(int)dirToExplore];
                        ExploreDir(dirToExplore, newNode);
                        --usedDir[(int)dirToExplore];
                    }
                }
            }

            foreach (Direction dir in visitLater.OrderBy(x => usedDir[(int)x]))
            {
                ++usedDir[(int)dir];
                ExploreDir(dir, newNode);
                --usedDir[(int)dir];
            }

            return newNode;
        }

        static void ExploreDir(Direction dir, MapNode currNode)
        {
            Console.WriteLine($"ExploreDir: From [{currNode}] - {dir.ToString()}");
            TurnRobot(dir);

            Junction juncOfNext;
            lowRobot.MoveToNode(out juncOfNext);
            double maxError = Math.Max(Math.Abs(lowRobot.X - currNode.Position.X), Math.Abs(lowRobot.Y - currNode.Position.Y)) * ERROR_RATE;
            MapNode nextNode = ExploreMap(juncOfNext, currNode, maxError, dir.ToOpposite());

            TurnRobot(dir.ToOpposite());
            lowRobot.MoveToNode(out _);

            ConnectNode(currNode, nextNode, dir);
            Console.WriteLine($"Connected: [{currNode}], [{nextNode}]");
        }

        static bool FindShortestRoute(MapPosition dest, Queue<Command> commQueue, out double resultDist)
        {
            if (dest.IsSameSection(robotPos))
            {
                if ((dest.Node1.NodeNumber > dest.Node2.NodeNumber) != (robotPos.Node1.NodeNumber > robotPos.Node2.NodeNumber))
                {
                    MapNode temp = dest.Node2;
                    dest.Node2 = dest.Node1;
                    dest.Node1 = temp;
                }

                if (dest.Dist1 < robotPos.Dist1)
                {
                    AddTurningToQueue(commQueue, lowRobot.Dir, dest.Node2.FindDirOf(dest.Node1).Value);
                    commQueue.Enqueue(new Command() { Act = Action.GoStraight, Dist = robotPos.Dist1 - dest.Dist1 });
                    resultDist = robotPos.Dist1 - dest.Dist1;
                }

                else
                {
                    AddTurningToQueue(commQueue, lowRobot.Dir, dest.Node1.FindDirOf(dest.Node2).Value);
                    commQueue.Enqueue(new Command() { Act = Action.GoStraight, Dist = dest.Dist1 - robotPos.Dist1 });
                    resultDist = dest.Dist1 - robotPos.Dist1;
                }

                return true;
            }

            int[] route;

            // 로봇이 v1과 v2 정점 사이에 있는데, v1과 v2의 거리

            double dist = Dijkstra(robotPos.Node1.NodeNumber, robotPos.Dist1, robotPos.Node2.NodeNumber, robotPos.Dist2, dest, out route);

            if (dist == -1)
            {
                resultDist = -1;
                return false;
            }

            Direction virtualRobotDir;

            // Node1에서 시작하는 경로일 때
            if (route[0] == robotPos.Node1.NodeNumber)
            {
                Direction dirOfNode1 = robotPos.Node2.FindDirOf(robotPos.Node1).Value;
                AddTurningToQueue(commQueue, lowRobot.Dir, dirOfNode1);

                virtualRobotDir = dirOfNode1;
            }
            // Node2에서 시작하는 경로일 때
            else
            {
                Direction dirOfNode2 = robotPos.Node1.FindDirOf(robotPos.Node2).Value;
                AddTurningToQueue(commQueue, lowRobot.Dir, dirOfNode2);

                virtualRobotDir = dirOfNode2;
            }

            commQueue.Enqueue(new Command() { Act = Action.GoToNextNode, NextExpect = nodeList[route[0]] });

            // route[i]와 route[i + 1]의 위치관계를 파악하여 큐 작성
            for (int i = 0; i < route.Length - 1; ++i)
            {
                Direction nextDir = nodeList[route[i]].FindDirOf(nodeList[route[i + 1]]).Value;
                AddTurningToQueue(commQueue, virtualRobotDir, nextDir);
                virtualRobotDir = nextDir;
                commQueue.Enqueue(new Command() { Act = Action.GoToNextNode, NextExpect = nodeList[route[i + 1]] });
            }

            AddTurningToQueue(commQueue, virtualRobotDir, (route.Last() == dest.Node1.NodeNumber ? dest.Node1.FindDirOf(dest.Node2) : dest.Node2.FindDirOf(dest.Node1)).Value);

            commQueue.Enqueue(new Command() { Act = Action.GoStraight, Dist = (route.Last() == dest.Node1.NodeNumber ? dest.Dist1 : dest.Dist2) });

            resultDist = dist;
            return true;
        }

        static double Dijkstra(int v1, double v1Init, int v2, double v2Init, MapPosition dest, out int[] route)
        {
            double[] dist = new double[nodeList.Count];
            int[] prev = new int[nodeList.Count];
            PriorityQueue<DijkElement> q = new PriorityQueue<DijkElement>();

            for (int i = 0; i < nodeList.Count; ++i)
            {
                dist[i] = DIST_INF;
                prev[i] = -1;
            }

            dist[v1] = v1Init;
            dist[v2] = v2Init;

            q.Enqueue(new DijkElement(v1, v1Init));
            q.Enqueue(new DijkElement(v2, v2Init));

            while (q.Count > 0)
            {
                DijkElement now = q.Dequeue();

                if (dist[now.Index] < now.Cost)
                    continue;

                foreach (var next in nodeConnection[now.Index])
                {
                    int nextIndex = next.Node.NodeNumber;
                    double nextDist = next.Dist + now.Cost;

                    if (dist[nextIndex] > nextDist)
                    {
                        dist[nextIndex] = nextDist;
                        prev[nextIndex] = now.Index;
                        q.Enqueue(new DijkElement(nextIndex, nextDist));
                    }
                }
            }

            if (dist[dest.Node1.NodeNumber] == DIST_INF && dist[dest.Node2.NodeNumber] == DIST_INF)
            {
                route = null;
                return -1;
            }

            Stack<int> routeTrace = new Stack<int>();

            double result;
            double n1Dist = dest.Dist1;
            double n2Dist = dest.Dist2;
            int currIndex;

            if (n1Dist + dist[dest.Node1.NodeNumber] >= n2Dist + dist[dest.Node2.NodeNumber])
            {
                currIndex = dest.Node2.NodeNumber;
                result = n2Dist + dist[dest.Node2.NodeNumber];
            }
            else
            {
                currIndex = dest.Node1.NodeNumber;
                result = n1Dist + dist[dest.Node1.NodeNumber];
            }

            while (currIndex != v1 && currIndex != v2)
            {
                routeTrace.Push(currIndex);
                currIndex = prev[currIndex];
            }
            routeTrace.Push(currIndex);

            route = new int[routeTrace.Count];
            for (int i = 0; i < route.Length; ++i)
            {
                route[i] = routeTrace.Pop();
            }

            return result;
        }

        static void AddTurningToQueue(Queue<Command> q, Direction robotDir, Direction goal)
        {
            if (goal != robotDir)
            {
                int leftCount = CountLeftTurn(robotDir, goal);
                int rightCount = CountRightTurn(robotDir, goal);

                if (leftCount <= rightCount)
                    for (int i = 0; i < leftCount; ++i)
                        q.Enqueue(new Command() { Act = Action.TurnLeft });

                else q.Enqueue(new Command() { Act = Action.TurnRight });
            }
        }

        static int CountLeftTurn(Direction before, Direction after)
        {
            if (before == after)
                return 0;

            int count = 1;
            while ((before = before.TurnLeft()) != after)
                ++count;

            return count;
        }

        static int CountRightTurn(Direction before, Direction after)
        {
            if (before == after)
                return 0;

            int count = 1;
            while ((before = before.TurnRight()) != after)
                ++count;

            return count;
        }

        static void TurnRobot(Direction dir)
        {
            Console.WriteLine("TurnRobot");
            while (lowRobot.Dir != dir)
            {
                Console.WriteLine("TurnRobotWhile");
                lowRobot.TurnLeft();
            }
            Console.WriteLine("TurnRobotEnd");
        }

        static void ConnectNode(MapNode currNode, MapNode nextNode, Direction nextDir)
        {
            currNode[nextDir] = nextNode;
            nextNode[nextDir.ToOpposite()] = currNode;

            double dist = currNode.Position.GetDist(nextNode.Position);

            NodeDistPair currNodeConn = new NodeDistPair(nextNode, dist);
            NodeDistPair nextNodeConn = new NodeDistPair(currNode, dist);

            if (!nodeConnection[currNode.NodeNumber].Contains(currNodeConn))
                nodeConnection[currNode.NodeNumber].Add(currNodeConn);

            if (!nodeConnection[nextNode.NodeNumber].Contains(nextNodeConn))
                nodeConnection[nextNode.NodeNumber].Add(nextNodeConn);

            currNode.SetDist(nextDir, dist);
            nextNode.SetDist(nextDir.ToOpposite(), dist);
        }

        static bool IsSameNode(Point p1, Point p2, Junction j1, Junction j2, Point error)
        {
            return Math.Abs(p1.X - p2.X) <= error.X * 2 && 
                   Math.Abs(p1.Y - p2.Y) <= error.Y * 2 &&
                   j1 == j2;
        }

        [STAThread]
        static void Main(string[] args)
        {
            GraphViewer gv;
            string graph1 = "...........\n" + 
                            ".OOOOOOOOO.\n" + 
                            ".O...O...O.\n" + 
                            ".OOOOOOOOO.\n" + 
                            ".O...O...O.\n" + 
                            ".OOOOOOOOO.\n" + 
                            "...........";

            string graph2 = ".......\n" +
                            ".O...O.\n" + 
                            ".O...O.\n" + 
                            ".OOOOO.\n" + 
                            "...O...\n" + 
                            "...O...\n" + 
                            ".......\n";

            string graph3 = "...........\n" +
                            ".OOOOOOOOO.\n" + 
                            ".O...O...O.\n" +
                            ".O...O...O.\n" +
                            ".O...O...O.\n" +
                            ".....O.....\n" +
                            ".....O.....\n" +
                            ".....O.....\n" + 
                            ".OOOOOOOOO.\n" + 
                            "...........\n";

            string graph4 = ".........................\n" +
                            ".O...OOOOOOOOOOOOOOO...O.\n" +
                            ".O.......O.....O.......O.\n" +
                            ".O.......O.....O.......O.\n" +
                            ".O.......O.....O.......O.\n" +
                            ".O.......O.....O.......O.\n" +
                            ".OOOOOOOOO.....OOOOOOOOO.\n" +
                            ".....O.............O.....\n" +
                            ".....O......O......O.....\n" +
                            ".....O......O......O.....\n" +
                            ".....O......O......O.....\n" +
                            ".OOOOOOOOOOOOOOOOOOOOOOO.\n" +
                            ".........................\n";

            string graph5 = "...........................\n" +
                            ".OOOOOOOOOOOOOOOOOOOOOOOOO.\n" +
                            ".O.....O.....O.....O.....O.\n" +
                            ".O.....O.....O.....O.....O.\n" +
                            ".OOOOOOOOOOOOOOOOOOOOOOOOO.\n" +
                            ".O.....O.....O.....O.....O.\n" +
                            ".O.....O.....O.....O.....O.\n" +
                            ".OOOOOOOOOOOOOOOOOOOOOOOOO.\n" +
                            ".O.....O.....O.....O.....O.\n" +
                            ".O.....O.....O.....O.....O.\n" +
                            ".OOOOOOOOOOOOOOOOOOOOOOOOO.\n" +
                            ".O.....O.....O.....O.....O.\n" +
                            ".O.....O.....O.....O.....O.\n" +
                            ".OOOOOOOOOOOOOOOOOOOOOOOOO.\n" +
                            "...........................\n";

            lowMap = new LowLevelMap('O', '.', graph4);
            lowRobot = new LowLevelRobot(lowMap, 1, 1);

            MapNode firstNode = ExploreMap(lowRobot.CheckNode(), null, 0, null);
            double distToSecond;

            Console.WriteLine("[DFS 결과]");
            Console.WriteLine($"Node 수 : {nodeList.Count}");
            foreach (var ele in nodeList)
            {
                Console.WriteLine($"{ele.NodeNumber} : ({ele.Position}), ({ele.Error})");
            }

            gv = new GraphViewer();
            gv.AddPoints(firstNode);
            gv.Show();

            Direction secondDir = firstNode.FindDirOf(nodeList[1]).Value;
            TurnRobot(secondDir);
            distToSecond = firstNode.GetDist(secondDir);

            lowRobot.Move(1);

            robotPos.Node1 = firstNode;
            robotPos.Node2 = nodeList[1];
            robotPos.DivRatio1 = 1;
            robotPos.DivRatio2 = distToSecond - 1;

            Console.Write("최단 거리를 탐색할 위치를 입력하세요. (입력형식: {node1} {node2} {ratio1} {ratio2}) : ");
            string[] token = Console.ReadLine().Split();

            MapPosition dest = new MapPosition();
            dest.Node1 = nodeList[int.Parse(token[0])];
            dest.Node2 = nodeList[int.Parse(token[1])];
            dest.DivRatio1 = double.Parse(token[2]);
            dest.DivRatio2 = double.Parse(token[3]);

            Queue<Command> commands = new Queue<Command>();
            double dist;
            FindShortestRoute(dest, commands, out dist);

            Console.WriteLine($"최단 거리: {dist}");
            while (commands.Count > 0)
            {
                Console.WriteLine(commands.Dequeue().ToString());
            }

            Console.WriteLine("아무키나 누르면 디버그를 종료합니다...");
            Console.ReadKey(true);
        }
    }
}
