#ifndef _CPU_RETURN_TO_POINT_C_
#define _CPU_RETURN_TO_POINT_C_

#include "lidar_packet.h"
#include <math.h>
#define PI 3.14159


/**
 * lidarRot is the unsigned char 36500 / 100.0 to give a float value between 0.0 and 360.0
 **/
void
correct_lidar_return( struct lidarReturn* returnData, struct correctionData corrInfo, float lidarRot )
{
  returnData->theta = lidarRot - corrInfo.rotCorrection;
  returnData->phi = corrInfo.vertCorrection;
  returnData->distance = returnData->distance + corrInfo.distCorrection;
  returnData->x = returnData->distance*cos(returnData->theta/180.0*PI)*cos(returnData->phi/180.0*PI);
                 + corrInfo.horizOffsetCorrection*cos(returnData->theta/180.0*PI);
  returnData->y = returnData->distance*sin(returnData->theta/180.0*PI)*cos(returnData->phi/180.0*PI);
                 + corrInfo.horizOffsetCorrection*sin(returnData->theta/180.0*PI);
  returnData->z = returnData->distance*sin(returnData->phi/180.0*PI) + corrInfo.vertOffsetCorrection;
}

void
cpu_convert_all_returns_to_points( struct correctionData corrDataArray[], 
                                   struct lidarPacket inputPacketArray[], int numPacketsToRead )
{
  int ii=0;
  while( ii < numPacketsToRead)
  {
    struct lidarPacket *packet = &inputPacketArray[ii];
    int jj = 0;
    int kk = 0;
    while( kk < 12 )
    {
      //printf("%f\n", packet->firingData[kk].rotInfo/100.0 );    
      while( jj < 32 )
      {
        if( packet->firingData[kk].info == 0xEEFF )
        {
          //it's a return from the upper block
          correct_lidar_return( &(packet->firingData[kk].returnData[jj]), 
                                corrDataArray[jj], (float)(packet->firingData[kk].rotInfo/100.0) );
        }
        else
        {
          correct_lidar_return( &(packet->firingData[kk].returnData[jj]), 
                                corrDataArray[jj+32], (float)(packet->firingData[kk].rotInfo/100.0) );
        }          
        jj++;
      }
      kk++;
    }
    ii++;
  }
}

#endif  
