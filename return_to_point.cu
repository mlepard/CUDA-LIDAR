#ifndef _GPU_RETURN_TO_POINT_CU_
#define _GPU_RETURN_TO_POINT_CU_

#include "lidar_packet.h"
#include "cuda_runtime.h"
#define PI 3.14159


/**
 * lidarRot is the unsigned char 36500 / 100.0 to give a float value between 0.0 and 360.0
 **/
__device__ void
gpu_correct_lidar_return( struct lidarReturn* returnData, struct correctionData corrInfo, float lidarRot )
{
  returnData->theta = lidarRot - corrInfo.rotCorrection;
  returnData->phi = corrInfo.vertCorrection;
  returnData->distance = returnData->distance + corrInfo.distCorrection;
  returnData->x = returnData->distance*__cosf(returnData->theta/180.0*PI)*__cosf(returnData->phi/180.0*PI);
                 + corrInfo.horizOffsetCorrection*__cosf(returnData->theta/180.0*PI);
  returnData->y = returnData->distance*__sinf(returnData->theta/180.0*PI)*__cosf(returnData->phi/180.0*PI);
                 + corrInfo.horizOffsetCorrection*__sinf(returnData->theta/180.0*PI);
  returnData->z = returnData->distance*__sinf(returnData->phi/180.0*PI) + corrInfo.vertOffsetCorrection;
}

__global__ void
gpu_convert_all_returns_to_points( struct correctionData d_corrDataArray[], 
                                   struct lidarPacket d_inputPacketArray[] )
{
  __shared__ struct correctionData s_corrData[32];
  __shared__ struct lidarFiringData  s_fireData;
  unsigned int packetId = blockDim.x/12 ;
  unsigned int firingId = blockDim.x % 12;
  unsigned int tid      = threadIdx.x;

  bool isUpper = d_inputPacketArray[packetId].firingData[firingId].info == 0xEEFF;
  float currentRotation =  d_inputPacketArray[packetId].firingData[firingId].rotInfo;

  unsigned int corrId = (isUpper) ? tid : tid+32;

  //copy global correction and fire data to shared info.
  s_corrData[tid].rotCorrection = d_corrDataArray[corrId].rotCorrection;  
  s_corrData[tid].vertCorrection = d_corrDataArray[corrId].vertCorrection;  
  s_corrData[tid].distCorrection = d_corrDataArray[corrId].distCorrection;  
  s_corrData[tid].vertOffsetCorrection = d_corrDataArray[corrId].vertOffsetCorrection;  
  s_corrData[tid].horizOffsetCorrection = d_corrDataArray[corrId].horizOffsetCorrection;

  s_fireData.returnData[tid].distance = d_inputPacketArray[packetId].firingData[firingId].returnData[tid].distance;
  s_fireData.returnData[tid].intensity = d_inputPacketArray[packetId].firingData[firingId].returnData[tid].intensity;

  gpu_correct_lidar_return( &(s_fireData.returnData[tid]), s_corrData[tid], currentRotation );

  //copy corrected return data back to the global input array
  d_inputPacketArray[packetId].firingData[firingId].returnData[tid].theta = s_fireData.returnData[tid].theta;
  d_inputPacketArray[packetId].firingData[firingId].returnData[tid].phi = s_fireData.returnData[tid].phi;
  d_inputPacketArray[packetId].firingData[firingId].returnData[tid].x = s_fireData.returnData[tid].x;
  d_inputPacketArray[packetId].firingData[firingId].returnData[tid].y = s_fireData.returnData[tid].y;
  d_inputPacketArray[packetId].firingData[firingId].returnData[tid].z = s_fireData.returnData[tid].z;
}

#endif  
