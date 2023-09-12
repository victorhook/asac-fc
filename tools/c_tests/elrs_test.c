// C library headers
#include <stdio.h>
#include <string.h>
#include "stdint.h"
#include "stdlib.h"

// Custom
#include "rc/crsf.h"
#include "asac_fc.h"

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

// TCP Server
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>


int port = 9090;


int main() {
    printf("** ELRS Test started, server port: %d **\n", port);


    int sockfd, newsockfd, portno;
    socklen_t clilen;
    char buffer[256];
    struct sockaddr_in serv_addr, cli_addr;
    int n;

    // Create a socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        printf("ERROR opening socket");
        exit(0);
    }

    // Clear address structure
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(port);

    // Bind the socket
    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        printf("ERROR on binding");
        exit(0);
    }

    listen(sockfd, 0);

    printf("** Started to listen... **\n");

    // Accept incoming connection
    newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
    if (newsockfd < 0) {
        printf("ERROR on accept");
        exit(0);
    }

    printf("** Client connected! **\n");

    while (1)
    {
        int r = read(newsockfd, buffer, 255);
        if (r < 0)
        {
            printf("** Error.. **\n");
            exit(0);
        }

        for (int i = 0; i < r; i++)
        {
            uint8_t buf = buffer[i];
            bool new_packet = crsf_parse_byte(buf);
            printf("%02x ", buf);
            //new_packet = false;
            if (new_packet)
            {
                //printf("NEW PACKET RECEIVED!\n");
                rx_state_t state;
                crsf_get_last_state(&state);
                printf("\n !!! -> T: %u", state.last_packet.timestamp);
                printf(", Channels: ");
                for (int i = 0; i < RC_MAX_NBR_OF_CHANNELS; i++)
                {
                    printf("%d ", state.last_packet.channels[i]);
                }
                printf(", RSSI: %d, LQ: %d\n",
                    state.statistics.rssi,
                    state.statistics.link_quality
                );
                printf("\n");
            }
        }
    }



    return 0;
  // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  int serial_port = open("/dev/ttyUSB0", O_RDWR);

  // Create new termios struct, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  //tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  const speed_t baud = 420000;
  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  // Allocate memory for read buffer, set size according to your needs
  uint8_t buf;

  //uint8_t data[] = {0xc8 ,0x18 ,0x16 ,0xe0 ,0x23 ,0x1f ,0x2d ,0xc0 ,0xf7 ,0x8b ,0xf2 ,0xfd ,0xa2 ,0x7c ,0xe5 ,0x2b ,0x5f ,0xf9 ,0xca ,0x07 ,0x00 ,0x00 ,0x4c ,0x7c ,0xe2 ,0xda};
  //for (int i = 0; i < 26; i++)
  //{
  //  crsf_parse_byte(data[i]);
  //}
  //return 0;

  system_init();

  while (1)
  {
    int res = read(serial_port, &buf, 1);
    if (res < 0)
    {
        printf("ERROR OCCURED!!");
        exit(0);
    }
    else if (res > 0)
    {
        printf("%02x ", res);
        if (crsf_parse_byte(buf))
        {
            printf("NEW PACKET RECEIVED!\n");
            rx_state_t state;
            crsf_get_last_state(&state);
            printf("T: %u\n", state.last_packet.timestamp);
            printf("Channels: ");
            for (int i = 0; i < RC_MAX_NBR_OF_CHANNELS; i++)
            {
                printf("%d ", state.last_packet.channels[i]);
            }
            printf("RSSI: %d, LQ: %d\n\n",
                state.statistics.rssi,
                state.statistics.link_quality
            );
        }
    }
  }

  close(serial_port);
  return 0; // success
};