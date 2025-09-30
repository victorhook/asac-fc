#include "hal.h"

#include <netinet/in.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>


const int BASE_PORT = 5050;

void* tcp_server(void* arg);


typedef struct
{
    uint8_t nbr;
    uint32_t port;
    struct sockaddr_in serv_addr;
    struct sockaddr_in client;
    int sockfd;
    int client_fd;
    bool connected;
    pthread_t thread;
} tcp_server_t;

tcp_server_t servers[4];


int hal_serial_init(const bus_config_serial_t config, const uint8_t nbr)
{
    tcp_server_t* server = &servers[nbr];
    server->nbr = nbr;
    server->client_fd = -1;
    server->connected = false;

    // Create tcp socket
    server->sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (server->sockfd < 0) return -1;

    // Reuse socket on port
    int value = 1;
    if (setsockopt(server->sockfd, SOL_SOCKET, SO_REUSEADDR, &value, sizeof(value)) < 0) return -1;

    // Fill addr struct
    server->port = BASE_PORT + nbr;
    server->serv_addr.sin_family = AF_INET;
    server->serv_addr.sin_port = htons(server->port);
    server->serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(server->sockfd, (struct sockaddr*) &server->serv_addr, sizeof(server->serv_addr)) < 0) return -1;

    if (listen(server->sockfd, 1) < 0) return -1;

    printf("Serial %d initialized on port: %d\n", nbr, server->port);

    // Start server thread
    pthread_create(&server->thread, NULL, tcp_server, server);

    return 0;
}

int hal_serial_do_write(serial_t* serial, const uint8_t* data, const uint32_t len)
{
    tcp_server_t* server = &servers[serial->nbr];
    if (!server->connected) return 0;

    int bytes_written = write(server->client_fd, data, len);
    if (bytes_written == -1)
    {
        printf("WRINTE DISCONNECTED\n");
        server->connected = false;
    }
    return bytes_written;
}

int hal_serial_do_read(serial_t* serial, uint8_t* data, const uint32_t len)
{
    tcp_server_t* server = &servers[serial->nbr];
    if (!server->connected) return 0;

    int bytes_read = recv(server->client_fd, data, len, 0);
    if (
        ((bytes_read == -1) && (errno != EAGAIN && errno != EWOULDBLOCK)) ||
        (bytes_read == 0)
    )
    {
        printf("Read disconnected: %d, %d\n", server->client_fd, bytes_read);
        server->connected = false;
    }
    return bytes_read;
}

void hal_serial_do_update()
{
    
}

void* tcp_server(void* arg)
{
    tcp_server_t* server = (tcp_server_t*) arg;

    while (true)
    {
        socklen_t client_len = sizeof(server->client);
        server->client_fd = accept(server->sockfd, (struct sockaddr*) &server->client, &client_len);
        if (server->client_fd < 0) continue;

        server->connected = true;

        // Set non-blocking
        int flags = fcntl(server->client_fd, F_GETFL, 0);
        if (flags < 0) flags = 0;
        fcntl(server->client_fd, F_SETFL, flags | O_NONBLOCK);

        printf("NEW CLIENT: %d\n", server->client_fd);

        while (server->connected)
        {
            sleep(1);
        }

        close(server->client_fd);
    }
}
