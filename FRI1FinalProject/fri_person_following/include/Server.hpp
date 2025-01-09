#include <cstring> 
#include <iostream> 
#include <netinet/in.h> 
#include <sys/socket.h> 
#include <unistd.h> 
#include <ros/ros.h>
using namespace std; 
#define PORT 8448


int startServer() {
	ROS_INFO("Creating server socket...");
	// creating socket
	int serverSocket = socket(AF_INET, SOCK_STREAM, 0); 

	// specifying the address 
	sockaddr_in serverAddress; 
	serverAddress.sin_family = AF_INET; 
	serverAddress.sin_port = htons(PORT); 
	serverAddress.sin_addr.s_addr = INADDR_ANY; 

    ROS_INFO("binding socket...");
	// binding socket. 
	bind(serverSocket, (struct sockaddr*)&serverAddress, 
		sizeof(serverAddress)); 

	// listening to the assigned socket 
    ROS_INFO("Listening to socket for connections...");
	listen(serverSocket, 5);
	
	sockaddr_in client_address;
    socklen_t client_address_size = sizeof(client_address);
    int clientSocket = accept(serverSocket, (struct sockaddr *)&client_address, &client_address_size);
	ROS_INFO("ClientSocket inside server file:%i", clientSocket);

	// accepting connection request 
	ROS_INFO("Accepted client...");

	//Stuff needed inside original main of ROS to close connection or receive messages.
	// recieving data 
	// char buffer[100] = { 0 }; 
	// recv(clientSocket, buffer, sizeof(buffer), 0); 
	// cout << "Message from client: " << buffer 
	// 		<< endl; 

	// // closing the socket. 
	close(serverSocket); 
	return clientSocket;
}