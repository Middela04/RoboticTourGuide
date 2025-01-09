#include <cstring> 
#include <iostream> 
#include <netinet/in.h> 
#include <sys/socket.h> 
#include <unistd.h>
#include <arpa/inet.h>

int startClient() { 
  const char* server_ip = "127.0.0.1";
  // Create a socket
  int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
  if (clientSocket == -1) {
    std::cerr << "Error: Connection failed.\n";
    return 0;
  }

  // Connect to the server
  sockaddr_in server_address;
  server_address.sin_family = AF_INET;
  server_address.sin_port = htons(8448);
  inet_pton(AF_INET, server_ip, &server_address.sin_addr);

  if (connect(clientSocket, (struct sockaddr *)&server_address, sizeof(server_address)) == -1) {
    std::cerr << "Error: Connection failed.\n";
    return 0;
  }
  printf("ClientSocket in Kinect: %i\n", clientSocket);
  printf("Connected to server..\n");
  return clientSocket;
}