#ifndef _GPU_RETURN_TO_POINT_CU_
#define _GPU_RETURN_TO_POINT_CU_

#include "lidar_packet.h"
#include "cuda_runtime.h"
#define PI 3.14159f


/**
 * lidarRot is the unsigned char 36500 / 100.0 to give a float value between 0.0 and 360.0
 **/

__device__ void
gpu_correct_lidar_return_opt_2( unsigned short distance,
                              float rotCorrection, float vertCorrection, 
                              float distCorrection, float horizOffsetCorrection,
                              float vertOffsetCorrection, float lidarRot, 
                              float* x, float* y, float*z )
{
  lidarRot = lidarRot - rotCorrection;
  float phi = vertCorrection;
  distance = distance + distCorrection;
  *x = distance*__cosf(lidarRot/180.0f*PI)*__cosf(phi/180.0f*PI);
                 + horizOffsetCorrection*__cosf(lidarRot/180.0f*PI);
  *y = distance*__sinf(lidarRot/180.0f*PI)*__cosf(phi/180.0f*PI);
                 + horizOffsetCorrection*__sinf(lidarRot/180.0f*PI);
  *z = distance*__sinf(phi/180.0f*PI) + vertOffsetCorrection;
}

__device__ void
gpu_correct_lidar_return_opt_3( unsigned short distance,
                              float rotCorrection, float vertCorrection, 
                              float distCorrection, float horizOffsetCorrection,
                              float vertOffsetCorrection, float lidarRot, 
                              float* x, float* y, float*z )
{
  lidarRot = lidarRot - rotCorrection;
  float phi = vertCorrection;
  distance = distance + distCorrection;
  *x = distance*cos(lidarRot/180.0f*PI)*cos(phi/180.0f*PI);
                 + horizOffsetCorrection*cos(lidarRot/180.0f*PI);
  *y = distance*sin(lidarRot/180.0f*PI)*cos(phi/180.0f*PI);
                 + horizOffsetCorrection*sin(lidarRot/180.0f*PI);
  *z = distance*sin(phi/180.0f*PI) + vertOffsetCorrection;
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
gpu_convert_all_returns_to_points_opt_2( const struct correctionData d_corrDataArray[], 
                                     const struct lidarGPUFirePacket d_inputPacketArray[],
                                     struct lidarGPUPointPacket  d_outputPointArray[] )
{
  __shared__ struct lidarGPUPointPacket  s_pointData;

  struct lidarGPUPointReturn *g_pPointData = &(d_outputPointArray[blockIdx.x].firingData[threadIdx.y]);
  float x,y,z;

  bool isUpper = d_inputPacketArray[blockIdx.x].firingData[threadIdx.y].info == 0xEEFF;
  float currentRotation =  d_inputPacketArray[blockIdx.x].firingData[threadIdx.y].rotInfo;

  unsigned int corrId = (isUpper) ? threadIdx.x : threadIdx.x+32;

  //use registers to keep minimize memory transfer
  gpu_correct_lidar_return_opt_2( d_inputPacketArray[blockIdx.x].firingData[threadIdx.y].returnData[threadIdx.x].distance, 
                                  d_corrDataArray[corrId].rotCorrection,
                                d_corrDataArray[corrId].vertCorrection, d_corrDataArray[corrId].distCorrection,
                                d_corrDataArray[corrId].horizOffsetCorrection, d_corrDataArray[corrId].vertOffsetCorrection,
                                currentRotation/100.0f, &x, &y, &z );

  //copy corrected return data back to the global input array
  g_pPointData->returnData[threadIdx.x].x = x;
  g_pPointData->returnData[threadIdx.x].y = y;
  g_pPointData->returnData[threadIdx.x].z = z;
}

__global__ void
gpu_convert_all_returns_to_points_opt_3( const struct correctionData d_corrDataArray[], 
                                     const struct lidarGPUFirePacket d_inputPacketArray[],
                                     struct lidarGPUPointPacket  d_outputPointArray[] )
{
  __shared__ struct lidarGPUPointPacket  s_pointData;

  struct lidarGPUPointReturn *g_pPointData = &(d_outputPointArray[blockIdx.x].firingData[threadIdx.y]);
  float x,y,z;

  bool isUpper = d_inputPacketArray[blockIdx.x].firingData[threadIdx.y].info == 0xEEFF;
  float currentRotation =  d_inputPacketArray[blockIdx.x].firingData[threadIdx.y].rotInfo;

  unsigned int corrId = (isUpper) ? threadIdx.x : threadIdx.x+32;

  //use registers to keep minimize memory transfer
  gpu_correct_lidar_return_opt_3( d_inputPacketArray[blockIdx.x].firingData[threadIdx.y].returnData[threadIdx.x].distance, 
                                  d_corrDataArray[corrId].rotCorrection,
                                d_corrDataArray[corrId].vertCorrection, d_corrDataArray[corrId].distCorrection,
                                d_corrDataArray[corrId].horizOffsetCorrection, d_corrDataArray[corrId].vertOffsetCorrection,
                                currentRotation/100.0f, &x, &y, &z );

  //copy corrected return data back to the global input array
  g_pPointData->returnData[threadIdx.x].x = x;
  g_pPointData->returnData[threadIdx.x].y = y;
  g_pPointData->returnData[threadIdx.x].z = z;
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
