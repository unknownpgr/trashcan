#define _CRTDBG_MAP_ALLOC
#include <iostream>
#include <string>
#include <sstream>
#include <crtdbg.h>
#include "RobotAlgorithm.h"

std::string GetGraph1()
{
	std::string str;
	str += "...........\n";
	str += ".OOOOOOOOO.\n";
	str += ".O...O...O.\n";
	str += ".OOOOOOOOO.\n";
	str += ".O...O...O.\n";
	str += ".OOOOOOOOO.\n";
	str += "...........";
	return str;
}

std::string GetGraph2()
{
	std::string str;
	str += ".......\n";
	str += ".O...O.\n";
	str += ".O...O.\n";
	str += ".OOOOO.\n";
	str += "...O...\n";
	str += "...O...\n";
	str += ".......";
	return str;
}

std::string GetGraph3()
{
	std::string str;
	str += ".........................\n";
	str += ".O...OOOOOOOOOOOOOOO...O.\n";
	str += ".O.......O.....O.......O.\n";
	str += ".O.......O.....O.......O.\n";
	str += ".O.......O.....O.......O.\n";
	str += ".O.......O.....O.......O.\n";
	str += ".OOOOOOOOO.....OOOOOOOOO.\n";
	str += ".....O.............O.....\n";
	str += ".....O......O......O.....\n";
	str += ".....O......O......O.....\n";
	str += ".....O......O......O.....\n";
	str += ".OOOOOOOOOOOOOOOOOOOOOOO.\n";
	str += ".........................";
	return str;
}

extern bool graph[100][100];
extern int width, height;
extern std::vector<MapNodePtr> nodeList;

int main()
{
	//_crtBreakAlloc = 1064;
	std::string graphStr = GetGraph3();
	char way = 'O', notWay = '.';

	std::stringstream ss;
	ss.str(graphStr);

	width = -1, height = 0;
	std::string line;
	while (std::getline(ss, line))
	{
		width = line.length();
		for (int col = 0; col < line.length(); ++col)
		{
			if (line[col] == way)
				graph[height][col] = true;
			else graph[height][col] = false;
		}
		++height;
	}

	std::cout << "Width = " << width << ", Height = " << height << std::endl;

	InitForDebug(1, 1);
	Init();
	
	std::cout << "[DFS Result]" << std::endl;
	for (MapNodePtr node : nodeList)
	{
		std::cout << node->NodeNumber << " : (" << node->Position.X << ", " << node->Position.Y << "), ("
			<< node->Error.X << ", " << node->Error.Y << ")" << std::endl;
	}

	int node1Index, node2Index;
	double div1, div2;
	std::cout << "최단 거리를 탐색할 위치를 입력하세요. (입력형식: {node1} {node2} {ratio1} {ratio2}) : ";
	std::cin >> node1Index >> node2Index >> div1 >> div2;

	MapPosition dest;
	dest.Node1 = nodeList[node1Index];
	dest.Node2 = nodeList[node2Index];
	dest.DivRatio1 = div1;
	dest.DivRatio2 = div2;

	std::queue<Command> commands;
	double dist;
	FindShortestRoute(dest, commands, dist);

	std::cout << "최단거리: " << dist << std::endl;

	while (!commands.empty())
	{
		Command comm = commands.front();
		commands.pop();

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

	}

	std::queue<Command>().swap(commands);
	DeleteNodeData();

	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);

	return 0;
}
