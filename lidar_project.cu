#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <cutil.h>

typedef unsigned int uint;
typedef unsigned char uchar;
#define iDivUp(a, b) (((a) % (b) != 0) ? ((a) / (b) + 1) : ((a) / (b)))

#include "dat_reader.h"
#include "lidar_packet.h"
#include "return_to_point_cpu.c"
#include "cuda_utils.cu"
#include "return_to_point.cu"

void
write_out_file( const char* output_file, 
                struct lidarPacket inputPacketArray[], int numPacketsToWrite )
{
  FILE *ofp;
  char *mode = "w";  
  ofp = fopen(output_file, mode);
  for( int ii=0; ii < numPacketsToWrite; ii++ )
  {
    for( int jj=0; jj < 12; jj++ )
    {
      for( int kk=0; kk < 32; kk++ )
      {
        fprintf(ofp, "%f\t%f\t%f\t%f\n", inputPacketArray[ii].firingData[jj].rotInfo/100.0f,
                                         inputPacketArray[ii].firingData[jj].returnData[kk].x,
                                         inputPacketArray[ii].firingData[jj].returnData[kk].y,
                                         inputPacketArray[ii].firingData[jj].returnData[kk].z );
      }
    }
  }
}

void
write_out_file( const char* output_file, 
                struct lidarGPUPointPacket inputPointArray[], 
                struct lidarGPUFirePacket inputFireArray[], 
                int numPacketsToWrite )
{
  FILE *ofp;
  char *mode = "w";  
  ofp = fopen(output_file, mode);
  for( int ii=0; ii < numPacketsToWrite; ii++ )
  {
    for( int jj=0; jj < 12; jj++ )
    {
      for( int kk=0; kk < 32; kk++ )
      {
        fprintf(ofp, "%f\t%f\t%f\t%f\n", inputFireArray[ii].firingData[jj].rotInfo/100.0f,
                                         inputPointArray[ii].firingData[jj].returnData[kk].x,
                                         inputPointArray[ii].firingData[jj].returnData[kk].y,
                                         inputPointArray[ii].firingData[jj].returnData[kk].z );
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Program main
////////////////////////////////////////////////////////////////////////////////
int
main (int argc, char** argv) 
{
    int numPacketsAtOnce = 30;
    int threads_block;
    int blocks;
    dim3 blockSize;
    float time1, time2, time3, time4;

    unsigned int timer1, timer2;
    // load image from disk

    CUT_DEVICE_INIT (argc, argv);

    // parse arguments
    char *filename;
    char *rawFilename = "unit46monterey.raw",
         *dbFilename = "db.csv", 
         *csvFilename = "out.csv";

    if (cutGetCmdLineArgumentstr ( argc, (const char**) argv, "raw", &filename))
    {
        rawFilename = filename;
    }
    if (cutGetCmdLineArgumentstr ( argc, (const char**) argv, "db", &filename))
    {
        dbFilename = filename;
    }
    if (cutGetCmdLineArgumentstr ( argc, (const char**) argv, "out", &filename))
    {
        csvFilename = filename;
    }

    cutCreateTimer (&timer1);
    cutCreateTimer (&timer2);

    struct correctionData corrDataArray[NUM_LIDAR_SENSORS];

    struct lidarPacket    *inputPacketArray;
    struct lidarPacket    *correctedPacketArray;

    struct correctionData *d_corrDataArray;
    //These are optimized data to minimize copy to GPU.
    struct lidarGPUFirePacket *h_packets;
    struct lidarGPUPointPacket  *h_points;
    struct lidarGPUFirePacket *d_packets;
    struct lidarGPUPointPacket  *d_points;

    //First malloc on cuda seems to take a LONG time...
    //This Malloc only needs to occur once, doesn't change with number fo packets to parse.
    parse_correction_file(dbFilename, corrDataArray);
    cudaMalloc ((void **) &d_corrDataArray,    NUM_LIDAR_SENSORS*sizeof(struct correctionData) );
    cudaMemcpy ((void *) d_corrDataArray, (void *) corrDataArray, 
                 NUM_LIDAR_SENSORS*sizeof(struct correctionData), cudaMemcpyHostToDevice);  
    cudaSucceed("Copy correction data to device");

    while( numPacketsAtOnce < 50 )
    {
      printf("%d packets\n", numPacketsAtOnce );

      //create the unoptimized packet arrays for CPU and GPU
      inputPacketArray = (struct lidarPacket*)malloc( numPacketsAtOnce*sizeof(struct lidarPacket) );
      correctedPacketArray = (struct lidarPacket*)malloc( numPacketsAtOnce*sizeof(struct lidarPacket) );

      //create the optimized transfer packet array
      h_packets = (struct lidarGPUFirePacket*)malloc(numPacketsAtOnce*sizeof(struct lidarGPUFirePacket));
      h_points = (struct lidarGPUPointPacket*)malloc(numPacketsAtOnce*sizeof(struct lidarGPUPointPacket));

      parse_data_file( rawFilename, csvFilename, corrDataArray, inputPacketArray, numPacketsAtOnce );

      //Copy data from the unoptimized packet structure to our optimized packet structure.
      //We wouldn't normally do this, but since we're testing different ways to do things it's fine.
      //Once we find the best way to transfer data we'd use that strucutre only.
      for( int ii=0; ii < numPacketsAtOnce; ii++ )
      {
         for( int jj=0; jj < 12; jj++ )
         {
           h_packets[ii].firingData[jj].info = inputPacketArray[ii].firingData[jj].info;
           h_packets[ii].firingData[jj].rotInfo = inputPacketArray[ii].firingData[jj].rotInfo;
           for( int kk = 0; kk < 32; kk++ )
           {
             h_packets[ii].firingData[jj].returnData[kk].distance = inputPacketArray[ii].firingData[jj].returnData[kk].distance;
             h_packets[ii].firingData[jj].returnData[kk].intensity = inputPacketArray[ii].firingData[jj].returnData[kk].intensity;
           }
         }
      }

      cudaMalloc ((void **) &d_packets, numPacketsAtOnce*sizeof(struct lidarGPUFirePacket) );
      cudaMalloc ((void **) &d_points, numPacketsAtOnce*sizeof(struct lidarGPUPointPacket) );
      cudaSucceed("Allocating device memory");
      cudaThreadSynchronize ();

      //Start timing when we copy data across to device
      cutResetTimer (timer1);
      cutStartTimer (timer1);
      cutResetTimer (timer2);
      cutStartTimer (timer2);

      cudaMemcpy ((void *) d_packets, (void *) h_packets, 
                   numPacketsAtOnce*sizeof(struct lidarGPUFirePacket), cudaMemcpyHostToDevice);
      cudaThreadSynchronize ();
      cutStopTimer (timer2);
      time1 = cutGetTimerValue(timer2);
      printf("GPU copy optimal memory = \t\t%f\n", cutGetTimerValue (timer2));

      //Do the GPU return to point conversion.
      blockSize.x = 32;
      blockSize.y = 12;
      blocks = numPacketsAtOnce;
      cutResetTimer (timer2);
      cutStartTimer (timer2);
      gpu_convert_all_returns_to_points <<<blocks, blockSize>>> ( d_corrDataArray, d_packets, d_points );
      cudaSucceed("GPU returns to points");
      cudaThreadSynchronize ();
      cutStopTimer (timer2);
      printf("GPU return to point time = \t%f\n", cutGetTimerValue (timer2) );
      time1 += cutGetTimerValue(timer2);

      //copy the data back to host
      cutResetTimer (timer2);
      cutStartTimer (timer2);
      cudaMemcpy ((void *) h_points, (void *) d_points, 
                   numPacketsAtOnce*sizeof(struct lidarGPUPointPacket), cudaMemcpyDeviceToHost);
      cudaSucceed("Copy device to host");
      cudaThreadSynchronize ();
      cutStopTimer (timer2);
      printf("GPU copy memory back = \t\t%f\n", cutGetTimerValue (timer2));
      write_out_file( "gpu_out.txt", h_points, h_packets, numPacketsAtOnce );
      time1 += cutGetTimerValue(timer2);

      //cutStopTimer (timer1);
      printf("GPU Total time = \t\t%f\n", time1);

      //Perform CPU returns to point
      cutResetTimer (timer1);
      cutStartTimer (timer1);
      cpu_convert_all_returns_to_points( corrDataArray, inputPacketArray, numPacketsAtOnce );
      cutStopTimer (timer1);
      printf ("CPU return to point time = \t%f\n", cutGetTimerValue (timer1));
      write_out_file( "cpu_out.txt", inputPacketArray, numPacketsAtOnce );

      printf("GPU/CPU Improvement = %f\n", cutGetTimerValue(timer1)/time1 );
      cudaFree (d_packets);
      cudaFree (d_points);
      numPacketsAtOnce = numPacketsAtOnce*2;
    }

    
    return 0;
}
