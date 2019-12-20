#include "LowLevel.h"
#include "control.h"

Point RobotPos = {0, 0};
Direction RobotDir = Direction::North;
Junction RobotJunc(true, false, false, false);

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
	rotate(90);
	RobotDir = TurnLeft(RobotDir);
}

void LowLevel::TurnRight_Raw()
{
	rotate(-90);
	RobotDir = TurnRight(RobotDir);
}

bool LowLevel::Move_Raw(double l)
{
	moveTicks(static_cast<int64_t>(l));
	
	switch (RobotDir)
	{
	case Direction::North:
		RobotPos.Y -= l;
		break;
	case Direction::South:
		RobotPos.Y += l;
		break;
	case Direction::East:
		RobotPos.X += l;
		break;
	case Direction::West:
		RobotPos.X -= l;
		break;
	}
}

Junction LowLevel::CheckNode_Raw()
{
	return RobotJunc;
}

std::pair<bool, Junction> LowLevel::MoveToNode_Raw()
{
	int8_t lowJunc = goUntillNode();
	if (lowJunc & NODE_LEFT)
	{
		RobotJunc.Reset(false, false, false, false);
		RobotJunc[TurnLeft(RobotDir)] = true;
		RobotJunc[ToOpposite(RobotDir)] = true;
	}
	else if (lowJunc & NODE_RIGHT)
	{
		RobotJunc.Reset(false, false, false, false);
		RobotJunc[TurnRight(RobotDir)] = true;
		RobotJunc[ToOpposite(RobotDir)] = true;
	}
	else if (lowJunc & NODE_CROSS)
	{
		RobotJunc.Reset(true, true, true, true);
	}
	else if (lowJunc & NODE_T)
	{
		RobotJunc.Reset(true, true, true, true);
		RobotJunc[RobotDir] = false;
	}
	else if (lowJunc & NODE_TERMINAL)
	{
		RobotJunc.Reset(false, false, false, false);
		RobotJunc[ToOpposite(RobotDir)];
	}

	return { true, RobotJunc };
}

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
