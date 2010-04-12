#ifndef _GPU_RETURN_TO_POINT_CU_
#define _GPU_RETURN_TO_POINT_CU_

#include "lidar_packet.h"
#include "cuda_runtime.h"
#define PI 3.14159f


/**
 * lidarRot is the unsigned char 36500 / 100.0 to give a float value between 0.0 and 360.0
 **/
__device__ void
gpu_correct_lidar_return( struct lidarReturn* returnData, struct correctionData corrInfo, float lidarRot )
{
  returnData->theta = lidarRot - corrInfo.rotCorrection;
  returnData->phi = corrInfo.vertCorrection;
  returnData->distance = returnData->distance + corrInfo.distCorrection;
  returnData->x = returnData->distance*__cosf(returnData->theta/180.0f*PI)*__cosf(returnData->phi/180.0f*PI);
                 + corrInfo.horizOffsetCorrection*__cosf(returnData->theta/180.0f*PI);
  returnData->y = returnData->distance*__sinf(returnData->theta/180.0f*PI)*__cosf(returnData->phi/180.0f*PI);
                 + corrInfo.horizOffsetCorrection*__sinf(returnData->theta/180.0f*PI);
  returnData->z = returnData->distance*__sinf(returnData->phi/180.0f*PI) + corrInfo.vertOffsetCorrection;
}

__device__ void
gpu_correct_lidar_return_opt( const struct lidarGPUReturn* returnData, const struct correctionData corrInfo, 
                              float lidarRot, struct lidarGPUPoint* pointData)
{
  lidarRot = lidarRot - corrInfo.rotCorrection;
  float phi = corrInfo.vertCorrection;
  float distance = returnData->distance + corrInfo.distCorrection;
  pointData->x = distance*__cosf(lidarRot/180.0f*PI)*__cosf(phi/180.0f*PI);
                 + corrInfo.horizOffsetCorrection*__cosf(lidarRot/180.0f*PI);
  pointData->y = distance*__sinf(lidarRot/180.0f*PI)*__cosf(phi/180.0f*PI);
                 + corrInfo.horizOffsetCorrection*__sinf(lidarRot/180.0f*PI);
  pointData->z = distance*__sinf(phi/180.0f*PI) + corrInfo.vertOffsetCorrection;
}

__global__ void
gpu_convert_all_returns_to_points_2( struct correctionData d_corrDataArray[], 
                                     struct lidarPacket d_inputPacketArray[] )
{
  __shared__ struct correctionData s_corrData[32];
  __shared__ struct lidarPacket  s_packetData;
  unsigned int packetId = blockIdx.x;
  unsigned int firingId = threadIdx.y;
  unsigned int returnId = threadIdx.x;

  struct lidarFiringData *s_pFireData = &(s_packetData.firingData[firingId]);
  struct lidarFiringData *g_pFireData = &(d_inputPacketArray[packetId].firingData[firingId]);

  bool isUpper = d_inputPacketArray[packetId].firingData[firingId].info == 0xEEFF;
  float currentRotation =  d_inputPacketArray[packetId].firingData[firingId].rotInfo;

  unsigned int corrId = (isUpper) ? returnId : returnId+32;

  //copy global correction and fire data to shared info.
  s_corrData[returnId].rotCorrection = d_corrDataArray[corrId].rotCorrection;  
  s_corrData[returnId].vertCorrection = d_corrDataArray[corrId].vertCorrection;  
  s_corrData[returnId].distCorrection = d_corrDataArray[corrId].distCorrection;  
  s_corrData[returnId].vertOffsetCorrection = d_corrDataArray[corrId].vertOffsetCorrection;  
  s_corrData[returnId].horizOffsetCorrection = d_corrDataArray[corrId].horizOffsetCorrection;

  s_pFireData->returnData[returnId].distance = g_pFireData->returnData[returnId].distance;
  s_pFireData->returnData[returnId].intensity = g_pFireData->returnData[returnId].intensity;

  gpu_correct_lidar_return( &(s_pFireData->returnData[returnId]), s_corrData[returnId], currentRotation/100.0f );

  //copy corrected return data back to the global input array
  g_pFireData->returnData[returnId].x = s_pFireData->returnData[returnId].x;
  g_pFireData->returnData[returnId].y = s_pFireData->returnData[returnId].y;
  g_pFireData->returnData[returnId].z = s_pFireData->returnData[returnId].z;
  g_pFireData->returnData[returnId].phi = s_pFireData->returnData[returnId].phi;
  g_pFireData->returnData[returnId].theta = s_pFireData->returnData[returnId].theta;
  g_pFireData->returnData[returnId].distance = s_pFireData->returnData[returnId].distance;

}

__global__ void
gpu_convert_all_returns_to_points_opt( const struct correctionData d_corrDataArray[], 
                                       const struct lidarGPUFirePacket d_inputPacketArray[],
                                       struct lidarGPUPointPacket  d_outputPointArray[] )
{
  __shared__ struct correctionData s_corrData[32];
  __shared__ struct lidarGPUFirePacket  s_packetData;
  __shared__ struct lidarGPUPointPacket  s_pointData;

  unsigned int packetId = blockIdx.x;
  unsigned int firingId = threadIdx.y;
  unsigned int returnId = threadIdx.x;

  struct lidarGPUFireData *s_pFireData = &(s_packetData.firingData[firingId]);
  const struct lidarGPUFireData *g_pFireData = &(d_inputPacketArray[packetId].firingData[firingId]);

  struct lidarGPUPointReturn *s_pPointData = &(s_pointData.firingData[firingId]);
  struct lidarGPUPointReturn *g_pPointData = &(d_outputPointArray[packetId].firingData[firingId]);

  bool isUpper = d_inputPacketArray[packetId].firingData[firingId].info == 0xEEFF;
  float currentRotation =  d_inputPacketArray[packetId].firingData[firingId].rotInfo;

  unsigned int corrId = (isUpper) ? returnId : returnId+32;

  //copy global correction and fire data to shared info.
  s_corrData[returnId].rotCorrection = d_corrDataArray[corrId].rotCorrection;  
  s_corrData[returnId].vertCorrection = d_corrDataArray[corrId].vertCorrection;  
  s_corrData[returnId].distCorrection = d_corrDataArray[corrId].distCorrection;  
  s_corrData[returnId].vertOffsetCorrection = d_corrDataArray[corrId].vertOffsetCorrection;  
  s_corrData[returnId].horizOffsetCorrection = d_corrDataArray[corrId].horizOffsetCorrection;

  s_pFireData->returnData[returnId].distance = g_pFireData->returnData[returnId].distance;
  s_pFireData->returnData[returnId].intensity = g_pFireData->returnData[returnId].intensity;

  gpu_correct_lidar_return_opt( &(s_pFireData->returnData[returnId]), s_corrData[returnId], 
                                currentRotation/100.0f, &(s_pPointData->returnData[returnId]) );

  //copy corrected return data back to the global input array
  g_pPointData->returnData[returnId].x = s_pPointData->returnData[returnId].x;
  g_pPointData->returnData[returnId].y = s_pPointData->returnData[returnId].y;
  g_pPointData->returnData[returnId].z = s_pPointData->returnData[returnId].z;
}

#endif  
