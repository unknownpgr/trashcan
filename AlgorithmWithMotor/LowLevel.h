#pragma once
#include "HighLevel.h"
#include <future>
using namespace HighLevel;

namespace LowLevel
{
	double GetX();
	double GetY();
	Point GetPos();

	Direction GetDir();

	void TurnLeft_Raw();
	void TurnRight_Raw();
	bool Move_Raw(double l);
	Junction CheckNode_Raw();
	std::pair<bool, Junction> MoveToNode_Raw();

	std::future<void> TurnLeft();
	std::future<void> TurnRight();
	std::future<bool> Move(double l);
	std::future<Junction> CheckNode();
	std::future<std::pair<bool, Junction>> MoveToNode();
}
