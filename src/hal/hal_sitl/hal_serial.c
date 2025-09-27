#include "hal.h"
#include "util/ringbuf.h"

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

typedef struct {
    int sockfd;
    int clientfd;
    serial_t* serial;
    pthread_t thread;
    int running;
} serial_impl_t;

#define MAX_SERIALS 4
static serial_impl_t g_serials[MAX_SERIALS];
#define TCP_PORT_BASE 5000

static int set_nonblocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1) return -1;
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

static void* serial_thread(void* arg) {
    serial_impl_t* impl = (serial_impl_t*)arg;

    while (impl->running) {
        printf("[HAL] Waiting for TCP client on port %d...\n", TCP_PORT_BASE + impl->serial->nbr);
        int clientfd = accept(impl->sockfd, NULL, NULL);
        if (clientfd < 0) {
            if (errno == EINTR) continue; // interrupted, retry
            perror("accept");
            continue;
        }

        impl->clientfd = clientfd;
        set_nonblocking(clientfd);
        printf("[HAL] Client connected!\n");

        uint8_t buf[256];
        while (impl->running) {
            // --- RX ---
            int n = recv(clientfd, buf, sizeof(buf), MSG_DONTWAIT);
            if (n > 0) {
                ringbuf_add_bytes(&impl->serial->rx_buf, buf, n);
            } else if (n == 0) {
                printf("[HAL] Client disconnected\n");
                break;
            }

            // --- TX ---
            if (!ringbuf_is_empty(&impl->serial->tx_buf)) {
                uint8_t* txptr;
                uint32_t available = ringbuf_peek(&impl->serial->tx_buf, &txptr);
                if (available > 0) {
                    int sent = send(clientfd, txptr, available, MSG_DONTWAIT);
                    if (sent > 0) {
                        ringbuf_advance(&impl->serial->tx_buf, sent);
                    } else if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                        perror("send");
                        break;
                    }
                }
            }

            usleep(1000); // still yields CPU, but less wasteful now
        }

        close(clientfd);
        impl->clientfd = -1;
    }
    return NULL;
}


int hal_serial_init(serial_t* serial, const uint8_t serial_nbr, const uint32_t baudrate) {
    (void)baudrate; // unused
    if (serial_nbr >= MAX_SERIALS) return -1;

    serial_impl_t* impl = &g_serials[serial_nbr];
    impl->serial = serial;
    impl->running = 1;

    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("socket");
        return -1;
    }

    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(TCP_PORT_BASE + serial_nbr);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(sockfd);
        return -1;
    }
    if (listen(sockfd, 1) < 0) {
        perror("listen");
        close(sockfd);
        return -1;
    }

    impl->sockfd = sockfd;

    if (pthread_create(&impl->thread, NULL, serial_thread, impl) != 0) {
        perror("pthread_create");
        close(sockfd);
        return -1;
    }

    return 0;
}

int hal_serial_write(serial_t* serial, const uint8_t* data, const uint16_t len) {
    int written = 0;
    for (int i = 0; i < len; i++) {
        if (ringbuf_add(&serial->tx_buf, data[i])) {
            written++;
        } else {
            break;
        }
    }
    return written;
}

int hal_serial_read(serial_t* serial, uint8_t* data, const uint16_t len) {
    int read = 0;
    for (int i = 0; i < len; i++) {
        if (ringbuf_get(&serial->rx_buf, &data[i])) {
            read++;
        } else {
            break;
        }
    }
    return read;
}

int hal_serial_available(const uint8_t serial_nbr) {
    if (serial_nbr >= MAX_SERIALS) return -1;
    serial_impl_t* impl = &g_serials[serial_nbr];
    return ringbuf_items(&impl->serial->rx_buf);
}
