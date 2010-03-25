#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dat_reader.h"
#include "lidar_packet.h"

void 
parse_correction_file( const char* input_file, struct correctionData  corrDataArray[] )
{
  FILE *ifp;
  char *mode = "r";
  char line[512];
  char *delim = ",";
  char *result;
  int sensor;
  float rotCorrection;
  float vertCorrection;
  float distCorrection;
  float vertOffsetCorrection;
  float horizOffsetCorrection;

  ifp = fopen(input_file, mode);

  if (ifp == NULL) {
    fprintf(stderr, "Can't open input file %s\n", input_file);
    exit(1);
  }
  
  while( fgets( line, 512, ifp) )
  {
    result = strtok(line, delim);   
    if( result != NULL ) 
    {
      sensor = atoi(result);
      if( sensor < 0 || sensor > 63 )
      {
        printf("error parsing correction file sensor %d\n", sensor);
        exit(1);
      }
      result = strtok(NULL, delim);    
      if( result != NULL ) rotCorrection = atof(result);
      else
      {
        printf( "error parsing correction file at rotCorrect, sensor %d\n",sensor );
        exit(1);
      }
      result = strtok(NULL, delim);
      if( result != NULL ) vertCorrection = atof(result);
      else
      {
        printf( "error parsing correction file at vertCorrection, sensor %d\n",sensor );
        exit(1);
      }    
      result = strtok(NULL, delim);
      if( result != NULL ) distCorrection = atof(result);
      else
      {
        printf( "error parsing correction file at distCorrection, sensor %d\n",sensor );
        exit(1);
      } 	  
      result = strtok(NULL, delim);
      if( result != NULL ) vertOffsetCorrection = atof(result);
      else
      {
        printf( "error parsing correction file at vertOffsetCorrection, sensor %d\n",sensor );
        exit(1);
      }
      result = strtok(NULL, delim);
      if( result != NULL ) horizOffsetCorrection = atof(result);
      else
      {
        printf( "error parsing correction file at horizOffsetCorrection, sensor %d\n",sensor );
        exit(1);
      }
      corrDataArray[sensor].rotCorrection = rotCorrection;    
      corrDataArray[sensor].vertCorrection = vertCorrection;    
      corrDataArray[sensor].distCorrection = distCorrection;    
      corrDataArray[sensor].vertOffsetCorrection = vertOffsetCorrection;    
      corrDataArray[sensor].horizOffsetCorrection = horizOffsetCorrection; 
      //printf( "sensor %d: %f\t%f\t%f\t%f\t%f\n", sensor, rotCorrection, vertCorrection, 
      //                                                   distCorrection, vertOffsetCorrection, 
      //                                                   horizOffsetCorrection );
    }
    else
    {
      continue;
    }
  }
  fclose(ifp);  
}

void
parse_data_file( const char* input_file, const char* output_file, 
                 struct correctionData corrDataArray[],
                 struct lidarPacket inputPacketArray[], int numPacketsToRead )
{
  FILE *ifp, *ofp;
  char *mode = "r";

  ifp = fopen(input_file, mode);

  if (ifp == NULL) {
    fprintf(stderr, "Can't open input file %s\n", input_file);
    exit(1);
  }

  ofp = fopen(output_file, "w");

  if (ofp == NULL) {
    fprintf(stderr, "Can't open output file %s!\n", output_file);
    exit(1);
  }

  int ii=0;
  while( !feof(ifp) && ii < numPacketsToRead)
  {
    struct lidarPacket *packet = &inputPacketArray[ii];
    parse_lidar_packet(ifp, packet);
    /*****
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
                                corrDataArray[jj], (float)(packet.rotInfo/100.0) );
        }
        else
        {
          correct_lidar_return( &(packet->firingData[kk].returnData[jj]), 
                                corrDataArray[jj+32], (float)(packet.rotInfo/100.0) );
        }          
        jj++;
      }
      kk++;
    }*/
    ii++;
  }
  fclose(ifp);
  fclose(ofp);
}


  

