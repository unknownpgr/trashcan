#include <fstream>
#include <iostream>
#include "WebInteract.h"
using namespace WebInteract;

void WebInteract::SendData(const std::string& fileName, const std::string& content)
{
	std::ofstream sendStream(Path_ToSend / path(fileName));
	sendStream << content;
	sendStream.close();
}

DataFromWebPtr WebInteract::ReceiveData()
{
	DataFromWeb* data = new DataFromWeb;

	for (auto fileIter = directory_iterator(Path_ToReceive); fileIter != directory_iterator(); ++fileIter)
	{
		if (!fileIter->is_regular_file() || 
			fileIter->path().filename().string()[0] == '_')
			continue;

		std::ifstream fileReader(fileIter->path().string());
		if (fileReader.is_open())
		{
			char header;
			fileReader >> header;

			if (header != '#')
			{
				fileReader.close();
				remove(fileIter->path());
				continue;
			}

			std::string command;
			fileReader >> command;

			int tokenCount = 0;
			if (command == "startCalibration")
			{
				data->first = CommFromWeb::Calibrate;
			}
			else if (command == "startExplore")
			{
				data->first = CommFromWeb::Explore;
			}
			else if (command == "getStatus")
			{
				data->first = CommFromWeb::GetStatus;
			}
			else if (command == "go")
			{
				data->first = CommFromWeb::Go;
				tokenCount = 3;
			}
			else
			{
				fileReader.close();
				remove(fileIter->path());
				continue;
			}

			while (tokenCount--)
			{
				std::string token;
				fileReader >> token;
				data->second.push_back(token);
			}

			fileReader.close();
			remove(fileIter->path());
			return DataFromWebPtr(data);
		}
	}

	data->first = CommFromWeb::Undefined;
	return DataFromWebPtr(data);
}
