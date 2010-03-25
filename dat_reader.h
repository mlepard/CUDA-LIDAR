#ifndef DAT_READER_H
#define DAT_READER_H

#include "lidar_packet.h"

void 
parse_correction_file( const char* input_file, struct correctionData  corrDataArray[] );

void
parse_data_file( const char* input_file, const char* output_file, 
                 struct correctionData corrDataArray[], 
                 struct lidarPacket inputPacketArray[], int numPacketsToRead );

#endif



  

