#ifndef _GPU_RETURN_TO_POINT_CU_
#define _GPU_RETURN_TO_POINT_CU_

#include "lidar_packet.h"
#include "cuda_runtime.h"
#define PI 3.14159f


/**
 * lidarRot is the unsigned char 36500 / 100.0 to give a float value between 0.0 and 360.0
 **/

__device__ void
gpu_correct_lidar_return( unsigned short distance,
                              float rotCorrection, float vertCorrection, 
                              float distCorrection, float horizOffsetCorrection,
                              float vertOffsetCorrection, float lidarRot, 
                              float3* point )
{
  lidarRot = lidarRot - rotCorrection;
  distance = distance + distCorrection;
  point->x = distance*__cosf(lidarRot/180.0f*PI)*__cosf(vertCorrection/180.0f*PI);
                 + horizOffsetCorrection*__cosf(lidarRot/180.0f*PI);
  point->y = distance*__sinf(lidarRot/180.0f*PI)*__cosf(vertCorrection/180.0f*PI);
                 + horizOffsetCorrection*__sinf(lidarRot/180.0f*PI);
  point->z = distance*__sinf(vertCorrection/180.0f*PI) + vertOffsetCorrection;
}

__global__ void
gpu_convert_all_returns_to_points( const struct correctionData d_corrDataArray[], 
                                     const struct lidarGPUFirePacket d_inputPacketArray[],
                                     struct lidarGPUPointPacket  d_outputPointArray[] )
{
  __shared__ float s_data[12*32*3];
  float3 point = ((float3*)s_data)[blockDim.x*threadIdx.y+threadIdx.x];
  
  bool isUpper = d_inputPacketArray[blockIdx.x].firingData[threadIdx.y].info == 0xEEFF;
  float currentRotation =  d_inputPacketArray[blockIdx.x].firingData[threadIdx.y].rotInfo;

  unsigned int corrId = (isUpper) ? threadIdx.x : threadIdx.x+32;

  //use registers to keep minimize memory transfer
  gpu_correct_lidar_return( d_inputPacketArray[blockIdx.x].firingData[threadIdx.y].returnData[threadIdx.x].distance, 
                            d_corrDataArray[corrId].rotCorrection, d_corrDataArray[corrId].vertCorrection, 
                            d_corrDataArray[corrId].distCorrection,d_corrDataArray[corrId].horizOffsetCorrection, 
                            d_corrDataArray[corrId].vertOffsetCorrection, currentRotation/100.0f, &point );

  ((float3*)s_data)[blockDim.x*threadIdx.y+threadIdx.x] = point;
  __syncthreads();
  //copy corrected return data back to the global input array
  float* gp_out = (float*)&(d_outputPointArray[blockIdx.x].firingData[threadIdx.y]);
  //perform a coalesced global memory write on the point data within one
  //firing structure, ie one row of the 12 row block
  gp_out[threadIdx.x] = s_data[blockDim.x*threadIdx.y+threadIdx.x];
  gp_out[threadIdx.x+32] = s_data[blockDim.x*threadIdx.y+threadIdx.x+32];
  gp_out[threadIdx.x+64] = s_data[blockDim.x*threadIdx.y+threadIdx.x+64];
}

#endif  
