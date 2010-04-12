#ifndef LIDAR_PACKET_H
#define LIDAR_PACKET_H

#include <stdio.h>

#define NUM_LIDAR_SENSORS 64

struct lidarGPUReturn {
  unsigned short distance;
  char intensity;
};

struct lidarGPUPoint {
  float x;
  float y;
  float z;
};

struct lidarGPUPointReturn {
  struct lidarGPUPoint  returnData[32];
};

struct lidarGPUPointPacket {
  struct lidarGPUPointReturn  firingData[12];
};


struct lidarGPUFireData {
  unsigned short      info;
  unsigned short      rotInfo;
  struct lidarGPUReturn  returnData[32];
};

struct lidarGPUFirePacket {
  struct lidarGPUFireData  firingData[12];
};  

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

