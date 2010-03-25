#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lidar_packet.h"


void 
parse_lidar_packet( FILE *ifp, struct lidarPacket* packet )
{
  int jj = 0;
  while( jj < 12 )
  {
    if (fread(&(packet->firingData[jj].info), 2, 1, ifp) != 1)
    {
      printf("unable to read packet info\n");
      exit(1);
    }
    if (fread(&(packet->firingData[jj].rotInfo), 2, 1, ifp) != 1)
    {
      printf("unable to read packet rotInfo\n");
      exit(1);
    }
    
    int ii = 0;
    while( ii < 32 )
    {
      if (fread(&(packet->firingData[jj].returnData[ii].distance), 2, 1, ifp) != 1)
      {
        printf("unable to read packet distance on return %d\n", ii);
        exit(1);
      }
      if (fread(&(packet->firingData[jj].returnData[ii].intensity), 1, 1, ifp) != 1)
      {
        printf("unable to read packet intensity on return %d\n", ii);
        exit(1);
      }
      ii++;    
    }
    jj++;
  }
  if (fread((packet->status), sizeof(packet->status[0]), 6, ifp) != 6)
  {
    printf("unable to read packet status\n");
    exit(1);
  }
}


  

