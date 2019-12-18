#pragma once
#include <string>
#include <vector>
// C++ 17 필요
#include <filesystem>
#include <memory>
using namespace std::filesystem;

enum class CommFromWeb { QueryPath = '@', Undefined = '\0' };
typedef std::pair<CommFromWeb, std::vector<std::string>> DataFromWeb;
typedef std::shared_ptr<DataFromWeb> DataFromWebPtr;

const path Path_ToSend = "C:\\WebInteractTest\\Receive";
const path Path_ToReceive = "C:\\WebInteractTest\\Receive";
// 웹으로부터 오는 명령들의 종류를 정의

void SendData(const std::string& fileName, const std::string& content);
// 웹으로부터 온 데이터(파일)를 확인. 처리할 명령이 없다면 Undefined를 반환.
DataFromWebPtr ReceiveData();
