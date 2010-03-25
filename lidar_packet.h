#ifndef LIDAR_PACKET_H
#define LIDAR_PACKET_H

#include <stdio.h>

#define NUM_LIDAR_SENSORS 64

struct lidarReturn {
  unsigned short distance;
  char intensity;
  float theta;
  float phi;
  float x;
  float y;
  float z;
};

struct correctionData {
  float rotCorrection;
  float vertCorrection;
  float distCorrection;
  float vertOffsetCorrection;
  float horizOffsetCorrection;
};

struct lidarFiringData {
  unsigned short      info;
  unsigned short      rotInfo;
  struct lidarReturn  returnData[32];
};

struct lidarPacket {
  struct lidarFiringData  firingData[12];
  char                    status[6];
};

void 
parse_lidar_packet( FILE *ifp, struct lidarPacket* packet );

#endif

