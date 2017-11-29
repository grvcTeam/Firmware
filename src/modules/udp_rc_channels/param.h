#ifndef ATTITUDE_H
#define ATTITUDE_H

#define BUFLEN 512  //Max length of buffer
#define PORT 8081   //The port on which to send data
//#define SERVER "192.168.1.30"
#define SERVER "169.254.223.150"
typedef struct{
  float rc_channels[8];
} rc_channels;

#endif
