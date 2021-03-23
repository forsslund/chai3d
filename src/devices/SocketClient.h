#pragma once
// Prevents <Windows.h> from #including <Winsock.h>, as we use <Winsock2.h> instead.
#ifndef _WINSOCKAPI_
#define DID_DEFINE_WINSOCKAPI
#define _WINSOCKAPI_
#endif

#include <winsock2.h>
#include <afunix.h>
#include <list>
#include <thread>
#include <mutex>

template <class T> 
class SocketClient {
public:
	void Start(std::string socketPath);

	SocketClient() : socketPath(""), listenThread() //Default threadummy
	{
		// Initialize Winsock
		int iResult;
		iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
		if (iResult != 0) {
			printf("WSAStartup failed: %d\n", iResult);
		}
	}
	~SocketClient() {
		stopClient = true;
		if (listenSocket != INVALID_SOCKET) closesocket(listenSocket);
		if (listenThread.joinable()) listenThread.join();
	}
	
	void Shutdown() { 
		stopClient = true;
	}	
	T Get() { 
		const std::lock_guard<std::mutex> lock(dataMutex);
		return data; 
	}

private:
	void Listen();
	bool isRunning = false;
	WSADATA wsaData;
	bool stopClient = false;	
	bool connected = false;
	T data = { 0 };
	std::mutex dataMutex;	
	std::thread listenThread;
	std::string socketPath;
	SOCKET listenSocket = INVALID_SOCKET;
};


///////////////////// SocketClient.cpp ////////

template <class T>
void SocketClient<T>::Start(std::string socketPath_) {
	// Fire up a thread to take care of incomming connections. But first check that thread is not running (i.e. not joinable)
	if (!listenThread.joinable()) {
		socketPath = socketPath_;
		listenThread = std::thread(&SocketClient<T>::Listen, this);
	}
		
}

#ifdef _WIN64
using ssize_t = __int64;
#endif

template <class T>
void SocketClient<T>::Listen() {
	printf("\nIn listening thread\n");

	struct sockaddr_un addr;
	int dataSize = sizeof(data);
	char* dataBuffer = new char[dataSize];
	
	while (!stopClient) {
		// Try setting up a connection
		listenSocket = socket(AF_UNIX, SOCK_STREAM, 0);
		if (listenSocket != INVALID_SOCKET) {
			connected = true;

			memset(&addr, 0, sizeof(struct sockaddr_un));
			addr.sun_family = AF_UNIX;
			strncpy_s(addr.sun_path, socketPath.c_str(), sizeof(addr.sun_path) - 1);
			int res = connect(listenSocket, (struct sockaddr*)&addr, sizeof(struct sockaddr_un));
			if (res >= 0) {
				printf("Connected to server \n");
				while (!stopClient && connected) {
					
					// Block untill activity on socket
					fd_set set;
					FD_ZERO(&set);
					FD_SET(listenSocket, &set);
					int activity = select(1, &set, NULL, NULL, NULL);

					// Recive until the whole datapackage is filled
					int byteCounter = 0;
					while (byteCounter < dataSize) {

						// Note: blocking, timeout needed in order to reach connectionLost=true (is it necessary?)
						// if another thread closes connection it returns with 0 immediately however. 
						int res = recvfrom(listenSocket, &dataBuffer[byteCounter], dataSize - byteCounter, 0, NULL, NULL);
						if (res <= 0) { // Since there was activity, we should have data, if no data or error code, connection was lost
							connected = false;
							break;
						}
						else {
							byteCounter += res;
						}
					}

					if (byteCounter == dataSize) {
						// Atomically store data
						const std::lock_guard<std::mutex> lock(dataMutex);
						memcpy((void*)&data, dataBuffer, sizeof(data));						
					}
				}
			}
			shutdown(listenSocket, 0);
			closesocket(listenSocket);
			listenSocket = INVALID_SOCKET;
			connected = false;			
		}
		Sleep(100); // Wait before trying again
	}
	WSACleanup();
	delete [] dataBuffer;
	return;
}
