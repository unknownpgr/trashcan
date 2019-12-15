#include "LowLevel.h"

Point RobotPos;
Direction RobotDir = Direction::North;

//-------------------------------------------------------

bool graph[100][100];
int width, height;
const double VIR_ERROR_RATE = 0.05;

int realX, realY;

void InitForDebug(int x, int y)
{
	RobotPos.X = x;
	RobotPos.Y = y;
	realX = x;
	realY = y;
}

bool Available(int row, int col)
{
	if (row < 0 || row >= height)
		return false;
	if (col < 0 || col >= width)
		return false;
	return true;
}

double LowLevel::GetX()
{
	return RobotPos.X;
}

double LowLevel::GetY()
{
	return RobotPos.Y;
}

Point LowLevel::GetPos()
{
	return RobotPos;
}

Direction LowLevel::GetDir()
{
	return RobotDir;
}

void LowLevel::TurnLeft_Raw()
{
	RobotDir = TurnLeft(RobotDir);
}

void LowLevel::TurnRight_Raw()
{
	RobotDir = TurnRight(RobotDir);
}

bool LowLevel::Move_Raw(double l)
{
	int dx = 0, dy = 0;

	switch (RobotDir)
	{
	case Direction::North:
		dy = -(int)l;
		break;
	case Direction::South:
		dy = (int)l;
		break;
	case Direction::East:
		dx = (int)l;
		break;
	case Direction::West:
		dx = -(int)l;
		break;
	default:
		return false;
	}

	if (!Available(realY + dy, realX + dx))
		return false;
	else if (!graph[realY + dy, realX + dx])
		return false;

	double fakeDx = GetError(VIR_ERROR_RATE, dx) + dx;
	double fakeDy = GetError(VIR_ERROR_RATE, dy) + dy;

	realX += dx;
	realY += dy;
	RobotPos.X += fakeDx;
	RobotPos.Y += fakeDy;

	return true;
}

Junction LowLevel::CheckNode_Raw()
{
	Junction llj;

	if (graph[realY - 1][realX])
		llj.North = true;
	if (graph[realY + 1][realX])
		llj.South = true;
	if (graph[realY][realX + 1])
		llj.East = true;
	if (graph[realY][realX - 1])
		llj.West = true;

	return llj;
}

std::pair<bool, Junction> LowLevel::MoveToNode_Raw()
{
	Junction currJunc = CheckNode_Raw();

	do
	{
		if (!Move_Raw(1))
			break;
	} while ((currJunc = CheckNode_Raw()).GetType() == NodeType::Straight);

	return { true, currJunc };
}

//-------------------------------------------------------

std::future<void> LowLevel::TurnLeft()
{
	return std::async(std::launch::async, TurnLeft_Raw);
}

std::future<void> LowLevel::TurnRight()
{
	return std::async(std::launch::async, TurnRight_Raw);
}

std::future<bool> LowLevel::Move(double l)
{
	return std::async(std::launch::async, Move_Raw, l);
}

std::future<Junction> LowLevel::CheckNode()
{
	return std::async(std::launch::async, CheckNode_Raw);
}

std::future<std::pair<bool, Junction>> LowLevel::MoveToNode()
{
	return std::async(std::launch::async, MoveToNode_Raw);
}
