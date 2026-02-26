#pragma once

#include <windows.h>
#include <string>
class ComPort
{
    private:
        HANDLE hCom;
        bool connected;
        COMSTAT status;
        DWORD errors;
    public:
        ComPort(const char* portName,DWORD baudRate);
        ~ComPort();
        bool isConnected();
        bool writeData(std::string& data);
        bool readData(std::string& buffer, unsigned int bufSize);
        void closePort();
};