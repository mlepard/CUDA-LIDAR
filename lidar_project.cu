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
    float time1, time2, time3;

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

    struct lidarPacket    *d_inputPacketArray;
    struct correctionData *d_corrDataArray;

    //First malloc on cuda seems to take a LONG time...
    //This Malloc only needs to occur once, doesn't change with number fo packets to parse.
    cudaMalloc ((void **) &d_corrDataArray,    NUM_LIDAR_SENSORS*sizeof(struct correctionData) );

    while( numPacketsAtOnce < 5000 )
    {
      printf("%d packets\n", numPacketsAtOnce );

      inputPacketArray = (struct lidarPacket*)malloc( numPacketsAtOnce*sizeof(struct lidarPacket) );
      correctedPacketArray = (struct lidarPacket*)malloc( numPacketsAtOnce*sizeof(struct lidarPacket) );

      parse_correction_file(dbFilename, corrDataArray);
      parse_data_file( rawFilename, csvFilename, corrDataArray, inputPacketArray, numPacketsAtOnce );

      //Perform GPU returns to point
      cutResetTimer (timer1);
      cutStartTimer (timer1);
      cutResetTimer (timer2);
      cutStartTimer (timer2);

      cudaMalloc ((void **) &d_inputPacketArray, numPacketsAtOnce*sizeof(struct lidarPacket) );
      cudaSucceed("Allocating device memory");
      cudaThreadSynchronize ();

      cudaMemcpy ((void *) d_inputPacketArray, (void *) inputPacketArray, 
                   numPacketsAtOnce*sizeof(struct lidarPacket), cudaMemcpyHostToDevice);
      cudaMemcpy ((void *) d_corrDataArray, (void *) corrDataArray, 
                   NUM_LIDAR_SENSORS*sizeof(struct correctionData), cudaMemcpyHostToDevice);  
      cudaSucceed("Copy host to device");
      cudaThreadSynchronize ();
      cutStopTimer (timer2);
      printf("GPU copy memory = \t\t%f\n", cutGetTimerValue (timer2));

      cutResetTimer (timer2);
      cutStartTimer (timer2);
      threads_block = 32;
      blocks = numPacketsAtOnce*12;
      gpu_convert_all_returns_to_points <<<blocks, threads_block>>> ( d_corrDataArray, d_inputPacketArray );
      cudaSucceed("GPU returns to points");
      cudaThreadSynchronize ();
      cutStopTimer (timer2);
      printf("GPU return to point time = \t%f,  threads = %d  blocks = %d\n", cutGetTimerValue (timer2), threads_block, blocks);
      time1 = cutGetTimerValue(timer2);

      //cutStopTimer (timer1);
      //printf("GPU Total time = \t\t%f\n", cutGetTimerValue (timer1));

      cutResetTimer (timer2);
      cutStartTimer (timer2);
      blockSize.x = 32;
      blockSize.y = 12;
      blocks = numPacketsAtOnce;
      gpu_convert_all_returns_to_points_2 <<<blocks, blockSize>>> ( d_corrDataArray, d_inputPacketArray );
      cudaSucceed("GPU returns to points");
      cudaThreadSynchronize ();
      cutStopTimer (timer2);
      printf("GPU return to point time = \t%f,  threads = %d  blocks = %d\n", cutGetTimerValue (timer2), blockSize.x * blockSize.y, blocks);
      time2 = cutGetTimerValue(timer2);

      printf("Improvement = \t%f\n", time2/time1*100);

      cutResetTimer (timer2);
      cutStartTimer (timer2);
      cudaMemcpy ((void *) correctedPacketArray, (void *) d_inputPacketArray, 
                   numPacketsAtOnce*sizeof(struct lidarPacket), cudaMemcpyDeviceToHost);
      cudaSucceed("Copy device to host");
      cudaThreadSynchronize ();
      cutStopTimer (timer2);
      printf("GPU copy memory back = \t\t%f\n", cutGetTimerValue (timer2));

      //cutStopTimer (timer1);
      //printf("GPU Total time = \t\t%f\n", cutGetTimerValue (timer1));


      //Perform CPU returns to point
      cutResetTimer (timer1);
      cutStartTimer (timer1);
      cpu_convert_all_returns_to_points( corrDataArray, inputPacketArray, numPacketsAtOnce );
      cutStopTimer (timer1);
      printf ("CPU return to point time = \t%f\n", cutGetTimerValue (timer1));

      cudaFree (d_inputPacketArray);
      numPacketsAtOnce = numPacketsAtOnce*2;
    }

    return 0;
}
