#include "_comport.hpp"
#include <iostream>

ComPort::ComPort(const char* portName, DWORD baudRate)
{
    this->connected = false;

    // Windows requires the "\\\\.\\" prefix for COM ports > 9. 
    // This safely formats the port name regardless of the number.
    std::string formattedPortName = "\\\\.\\" + std::string(portName);

    this->hCom = CreateFileA(
        formattedPortName.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL
    );

    if (this->hCom == INVALID_HANDLE_VALUE) {
        if (GetLastError() == ERROR_FILE_NOT_FOUND) {
            std::cerr << "Error: COM port " << portName << " not found." << std::endl;
        } else {
            std::cerr << "Error: Unknown error opening " << portName << std::endl;
        }
        return;
    }

    DCB dcbSerialParams = {0};
    if (!GetCommState(this->hCom, &dcbSerialParams)) {
        std::cerr << "Error: Failed to get current serial parameters." << std::endl;
        this->closePort();
        return;
    }

    dcbSerialParams.BaudRate = baudRate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    // Disable DTR/RTS to prevent Arduino auto-reset on connection (optional but recommended)
    dcbSerialParams.fDtrControl = DTR_CONTROL_DISABLE;
    dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;

    if (!SetCommState(this->hCom, &dcbSerialParams)) {
        std::cerr << "Error: Could not set serial parameters." << std::endl;
        this->closePort();
        return;
    }

    // Set non-blocking timeouts
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (!SetCommTimeouts(this->hCom, &timeouts)) {
        std::cerr << "Error: Could not set timeouts." << std::endl;
        this->closePort();
        return;
    }

    // Purge any residual data in the buffer from previous sessions
    PurgeComm(this->hCom, PURGE_RXCLEAR | PURGE_TXCLEAR);
    this->connected = true;
}

ComPort::~ComPort()
{
    this->closePort();
}


bool ComPort::isConnected()
{
    return this->connected;
}   

void ComPort::closePort() {
    if (this->connected) {
        this->connected = false;
        CloseHandle(this->hCom);
    }
}

bool ComPort::writeData(std::string& data) {
    if (!this->connected) return false;

    DWORD bytesWritten;
    ClearCommError(this->hCom, &this->errors, &this->status);

    bool success = WriteFile(
        this->hCom,
        data.c_str(),
        data.length(),
        &bytesWritten,
        NULL
    );

    return success && (bytesWritten == data.length());
}

bool ComPort::readData(std::string& buffer, unsigned int buffer_size) {
    if (!this->connected) return false;

    DWORD bytesRead;
    unsigned int toRead = 0;
    char* tempBuffer = new char[buffer_size + 1]; // +1 for null terminator

    ClearCommError(this->hCom, &this->errors, &this->status);

    if (this->status.cbInQue > 0) {
        // Read either the available bytes or up to the buffer size
        toRead = (this->status.cbInQue > buffer_size) ? buffer_size : this->status.cbInQue;

        if (ReadFile(this->hCom, tempBuffer, toRead, &bytesRead, NULL)) {
            tempBuffer[bytesRead] = '\0'; // Null-terminate the string
            buffer = std::string(tempBuffer);
            delete[] tempBuffer;
            return true;
        }
    }

    delete[] tempBuffer;
    return false;
}

