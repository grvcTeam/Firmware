#ifndef ATTITUDE_H
#define ATTITUDE_H

#define BUFLEN 512  //Max length of buffer
#define PORT 8084   //The port on which to send data
//#define SERVER "192.168.1.30"
#define SERVER "169.254.223.150"
typedef struct{
  float lat;
  float lon;
  float alt;
  float vn;
  float ve;
  float vd;
  float pressure_alt;
  uint64_t timestamp;
} global_pos_values;

#endif
