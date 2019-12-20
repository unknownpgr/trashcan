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
	// 디버그용으로 그래프를 초기화 시켜주는 코드
	/*std::string graphStr = GetGraph3();
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
	}*/

	// 디버그용으로 설정한 그래프의 열 수와 행 수 출력
	//std::cout << "Width = " << width << ", Height = " << height << std::endl;

	// 디버그용으로 로봇의 위치를 임의로 설정. 실전에서는 쓰일 일 없음.
	//InitForDebug(1, 1);
	//Init();
	
	ListenFromWeb();
	std::cout << "지금부터 웹서버의 파일 명령을 기다림. 프로그램을 종료하려면 아무키나 누르십시오..." << std::endl;

	std::cin.get();

	StopListening();
	DeleteNodeData();

	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);

	return 0;
}
