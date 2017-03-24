#ifndef ATTITUDE_H
#define ATTITUDE_H

#define BUFLEN 512  //Max length of buffer
#define PORT 8080   //The port on which to send data
#define SERVER "192.168.1.30"
typedef struct{
  float roll;
  float pitch;
  float yaw;
  float quat[4];
} attitudeValues;

#endif
