#include <fstream>
#include <iostream>
#include "WebInteract.h"

void SendData(const std::string& fileName, const std::string& content)
{
	std::ofstream sendStream(Path_ToSend / path(fileName));
	sendStream << content;
	sendStream.close();
}

DataFromWebPtr ReceiveData()
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

			int tokenCount = 0;
			switch (header)
			{
			case (char)CommFromWeb::QueryPath:
				data->first = CommFromWeb::QueryPath;
				tokenCount = 3;
				break;

			case (char)CommFromWeb::Undefined:
			default:
				data->first = CommFromWeb::Undefined;
				tokenCount = 0;
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
