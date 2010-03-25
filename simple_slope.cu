#ifndef _BICUBIC_KERNEL_H_
#define _BICUBIC_KERNEL_H_

#include "cutil_math.h"

#define THREAD_X_SIZE 32
#define THREAD_Y_SIZE 16

void
execconfig_setup (void)
{
  blockSize.x = THREAD_X_SIZE;
  blockSize.y = THREAD_Y_SIZE;
  // calculate new grid size
  gridSize = dim3 (iDivUp (imageWidth, blockSize.x), iDivUp (imageHeight, blockSize.y));
}

__device__ void
copyRightBlock(uchar* s_data, uchar* g_input, uint width, uint height)
{
  //Set the x portion of our global memory index to
  //the block to the right of us (blockIdx+1)
  uint x = (blockIdx.x+1) * blockDim.x + threadIdx.x;
  uint y = blockIdx.y * blockDim.y + threadIdx.y;
  uint i = y * width + x;
  //Set the shared memory index to the 3rd 16x16 piece
  //just below the one row of 3x16
  //s_index starts at 3x16 (top row) + 2x16 (left and center blocks).
  uint s_index = (THREAD_X_SIZE*5 + threadIdx.y*THREAD_X_SIZE*3 + threadIdx.x)*4;
  
  //If we end up accessing outside the image size, set to zero.
  if( x < width && y < height )
  {
    s_data[s_index] = g_input[i];
  }
  else
  {
    s_data[s_index] = 0;
  }
}


__device__ void
copyLeftBlock(uchar* s_data, uchar* g_input, uint width, uint height)
{
  //Set the x portion of our global memory index to
  //the block to the left of us (blockIdx-1)
  uint x = (blockIdx.x-1) * blockDim.x + threadIdx.x;
  uint y = blockIdx.y * blockDim.y + threadIdx.y;
  uint i = y * width + x;
  //Set the shared memory index to the 1st 16x16 piece
  //just below the one row of 3x16.
  //s_index starts at 16x3(top row).  
  uint s_index = (THREAD_X_SIZE*3 + threadIdx.y*THREAD_X_SIZE*3 + threadIdx.x)*4;

  //If we end up accessing outside the image size, set to zero.  
  if( x < width && y < height )
  {
    s_data[s_index] = g_input[i];
  }
  else
  {
    s_data[s_index] = 0;
  }
}

__device__ void
copyBottomRow(uchar* s_data, uchar* g_input, uint width, uint height, uint startingTid)
{
  //Set the y portion of our global memory index to
  //the row just below our block. (blockIdx.y+1)
  //We want to start copying data at the zero thread index,
  //regardless of the thread that is actually doing the copy.
  //Thus perform tid - startingTid to start at the desired block in global memory
  uint tid = blockDim.x*threadIdx.y + threadIdx.x;
  uint x = blockIdx.x * blockDim.x + (tid - startingTid);
  uint y = (blockIdx.y+1) * blockDim.y;
  uint i = y * width + x;
  uint threadX = tid - startingTid;
  //Set the shared memory index to the row below the 2nd (center) 16x16 piece
  //s_index starts at 16x3(top row) + 16x16x3(left/right/center blocks) +
  //16.   
  uint s_index = (THREAD_X_SIZE*4 + THREAD_Y_SIZE*THREAD_X_SIZE*3 + threadX)*4; 

  //If we end up accessing outside the image size, set to zero.
  if( x < width && y < height )
  {
    s_data[s_index] = g_input[i];
  }
  else
  {
    s_data[s_index] = 0;
  }
}

__device__ void
copyBottomRightRow(uchar* s_data, uchar* g_input, uint width, uint height, uint startingTid)
{
  //Set the y portion of our global memory index to
  //the row just below our block. (blockIdx.y+1).
  //And the x portion of our global memory index to the block
  //just to the right of our block (blockIdx.x+1)
  //We want to start copying data at the zero thread index,
  //regardless of the thread that is actually doing the copy.
  //Thus perform tid - startingTid to start at the desired block in global memory
  uint tid = blockDim.x*threadIdx.y + threadIdx.x;
  uint x = (blockIdx.x+1) * blockDim.x + (tid - startingTid);
  uint y = (blockIdx.y+1) * blockDim.y;
  uint i = y * width + x;
  uint threadX = tid - startingTid;
  //Set the shared memory index to the row below the 3rd (right) 16x16 piece
  //s_index starts at 16x3(top row) + 16x16x3(left/right/center blocks) +
  //16x2.    
  uint s_index = (THREAD_X_SIZE*5 + THREAD_Y_SIZE*THREAD_X_SIZE*3 + threadX)*4;

  //If we end up accessing outside the image size, set to zero.  
  if( x < width && y < height )
  {
    s_data[s_index] = g_input[i];
  }
  else
  {
    s_data[s_index] = 0;
  }
}

__device__ void
copyBottomLeftRow(uchar* s_data, uchar* g_input, uint width, uint height, uint startingTid)
{
  //Set the y portion of our global memory index to
  //the row just below our block. (blockIdx.y+1).
  //And the x portion of our global memory index to the block
  //just to the left of our block (blockIdx.x-1)
  //We want to start copying data at the zero thread index,
  //regardless of the thread that is actually doing the copy.
  //Thus perform tid - startingTid to start at the desired block in global memory
  uint tid = blockDim.x*threadIdx.y + threadIdx.x;
  uint x = (blockIdx.x-1) * blockDim.x + (tid - startingTid);
  uint y = (blockIdx.y+1) * blockDim.y;
  uint i = y * width + x;
  uint threadX = tid - startingTid;
  //Set the shared memory index to the row below the 1st (left) 16x16 piece
  //s_index starts at 16x3(top row) + 16x16x3(left/right/center blocks).        
  uint s_index = (THREAD_X_SIZE*3 + THREAD_Y_SIZE*THREAD_X_SIZE*3 + threadX)*4;

  //If we end up accessing outside the image size, set to zero.    
  if( x < width && y < height )
  {
    s_data[s_index] = g_input[i];
  }
  else
  {
    s_data[s_index] = 0;
  }
}

__device__ void
copyTopRow(uchar* s_data, uchar* g_input, uint width, uint height, uint startingTid)
{
  //Set the y portion of our global memory index to
  //the row just above our block. (blockIdx.y-1) + 15 rows.
  //We want to start copying data at the zero thread index,
  //regardless of the thread that is actually doing the copy.
  //Thus perform tid - startingTid to start at the desired block in global memory
  uint tid = blockDim.x*threadIdx.y + threadIdx.x;
  uint x = (blockIdx.x) * blockDim.x + (tid - startingTid);
  uint y = (blockIdx.y-1) * blockDim.y + (blockDim.y - 1);
  uint i = y * width + x;
  uint threadX = tid - startingTid;
  //Set the shared memory index to the row above the 2nd (center) 16x16 piece
  //s_index starts at 16 (center row).            
  uint s_index = (THREAD_X_SIZE +  threadX)*4;
  
  if( x < width && y < height )
  {
    s_data[s_index] = g_input[i];
  }
  else
  {
    s_data[s_index] = 0;
  }
}

__device__ void
copyTopRightRow(uchar* s_data, uchar* g_input, uint width, uint height, uint startingTid)
{
  //Set the y portion of our global memory index to
  //the row just above our block. (blockIdx.y-1)+15 rows.
  //And the x portion of our global memory index to the block
  //just to the right of our block (blockIdx.x+1)
  //We want to start copying data at the zero thread index,
  //regardless of the thread that is actually doing the copy.
  //Thus perform tid - startingTid to start at the desired block in global memory
  uint tid = blockDim.x*threadIdx.y + threadIdx.x;
  uint x = (blockIdx.x+1) * blockDim.x + (tid - startingTid);
  uint y = (blockIdx.y-1) * blockDim.y + (blockDim.y - 1);
  uint i = y * width + x;
  uint threadX = tid - startingTid;
  //Set the shared memory index to the row above the 2nd (center) 16x16 piece
  //s_index starts at 16x2 (right row).                
  uint s_index = (THREAD_X_SIZE*2 +  threadX)*4;
  
  if( x < width && y < height )
  {
    s_data[s_index] = g_input[i];
  }
  else
  {
    s_data[s_index] = 0;
  }
}

__device__ void
copyTopLeftRow(uchar* s_data, uchar* g_input, uint width, uint height, uint startingTid)
{
  //Set the y portion of our global memory index to
  //the row just above our block. (blockIdx.y-1)+15 rows.
  //And the x portion of our global memory index to the block
  //just to the right of our block (blockIdx.x-1)
  //We want to start copying data at the zero thread index,
  //regardless of the thread that is actually doing the copy.
  //Thus perform tid - startingTid to start at the desired block in global memory
  uint tid = blockDim.x*threadIdx.y + threadIdx.x;
  uint x = (blockIdx.x-1) * blockDim.x + (tid - startingTid);
  uint y = (blockIdx.y-1) * blockDim.y + (blockDim.y - 1);
  uint i = y * width + x;
  uint threadX = tid - startingTid;
  //Set the shared memory index to the row above the 2nd (center) 16x16 piece
  //s_index starts at 0 (left row).                    
  uint s_index = (threadX)*4;
  
  if( x < width && y < height )
  {
    s_data[s_index] = g_input[i];
  }
  else
  {
    s_data[s_index] = 0;
  }
}

template <unsigned int blockSize>
__global__ void minMax(uchar *g_idata, uchar *g_oMin, uchar* g_oMax, unsigned int n)
{
  //Perform both the min/max in the same thread.
  //create two shared memory arrays.
  //because the data size is uchar (1byte) make the arrays 4 times the size
  //and only reference every 4th element.  To avoid bank conflicts.
  __shared__ uchar s_minData[blockSize*4];
  __shared__ uchar s_maxData[blockSize*4];
  
  unsigned int tid = threadIdx.x;
  unsigned int i = blockIdx.x*(blockSize*2) + tid;
  unsigned int gridSize = blockSize*2*gridDim.x;
  //Reference the shared memory every 4 bytes to avoid bank conflicts.
  unsigned int s_index = tid*4;
  
  do
  {
    //While copying global data in shared memory, do a max/min.
    s_minData[s_index] = min( g_idata[i], g_idata[i+blockSize] );
    s_maxData[s_index] = max( g_idata[i], g_idata[i+blockSize] );
    i += gridSize;
  } while (i < n);
  __syncthreads();

  if (blockSize >= 512)
  {
    if (tid < 256)
    {
      s_minData[s_index] = min( s_minData[s_index], s_minData[s_index + 256*4] );
      s_maxData[s_index] = max( s_maxData[s_index], s_maxData[s_index + 256*4] );
    }
    __syncthreads();
  }
  if (blockSize >= 256)
  {
    if (tid < 128)
    {
      s_minData[s_index] = min( s_minData[s_index], s_minData[s_index + 128*4] );
      s_maxData[s_index] = max( s_maxData[s_index], s_maxData[s_index + 128*4] );
    }
    __syncthreads();
  }
  if (blockSize >= 128)
  {
    if (tid < 64) 
    { 
      s_minData[s_index] = min( s_minData[s_index], s_minData[s_index + 64*4] );
      s_maxData[s_index] = max( s_maxData[s_index], s_maxData[s_index + 64*4] );
    }
    __syncthreads();
  }
  if (tid < 32)
  {
    if (blockSize >= 64)
    {
      s_minData[s_index] = min( s_minData[s_index], s_minData[s_index + 32*4] );
      s_maxData[s_index] = max( s_maxData[s_index], s_maxData[s_index + 32*4] );
    }  
    if (blockSize >= 32)
    {
      s_minData[s_index] = min( s_minData[s_index], s_minData[s_index + 16*4] );
      s_maxData[s_index] = max( s_maxData[s_index], s_maxData[s_index + 16*4] );
    }
    if (blockSize >= 16)
    {
      s_minData[s_index] = min( s_minData[s_index], s_minData[s_index + 8*4] );
      s_maxData[s_index] = max( s_maxData[s_index], s_maxData[s_index + 8*4] );
    }    
    if (blockSize >= 8)
    {
      s_minData[s_index] = min( s_minData[s_index], s_minData[s_index + 4*4] );
      s_maxData[s_index] = max( s_maxData[s_index], s_maxData[s_index + 4*4] );
    }
    if (blockSize >= 4)
    {
      s_minData[s_index] = min( s_minData[s_index], s_minData[s_index + 2*4] );
      s_maxData[s_index] = max( s_maxData[s_index], s_maxData[s_index + 2*4] );
    }
    if (blockSize >= 2)
    {
      s_minData[s_index] = min( s_minData[s_index], s_minData[s_index + 1*4] );
      s_maxData[s_index] = max( s_maxData[s_index], s_maxData[s_index + 1*4] );
    }
  }

  if (tid == 0)
  {
    g_oMin[blockIdx.x] = s_minData[0];
    g_oMax[blockIdx.x] = s_maxData[0];
  }
}

__global__ void
d_normalize (uchar *d_output, uchar *d_input,  uint width, uint height, uint max, uint min)
{
  //Create the block shared memory, but since we are using uchar (1byte)
  //Make it 4x the size to avoid bank conflicts.
  __shared__ uchar s_data[THREAD_X_SIZE*THREAD_Y_SIZE*4];
  uint x = blockIdx.x * blockDim.x + threadIdx.x;
  uint y = blockIdx.y * blockDim.y + threadIdx.y;
  uint i = y * width + x;
  uint tid = blockDim.x*threadIdx.y + threadIdx.x;
  //Access the shared memory every 4 bytes, to avoid bank conflicts.
  uint s_index = tid*4;

  //Copy global memory data into shared memory.
  //Reads should be coalesced by warp of 32 threads (1 byte uchars)
  if( x < width && y < height )
  {
    s_data[s_index] = d_input[i];
  }

  //Perform the normalized filter
  float factor = 255.0/(max - min);
  s_data[s_index] = (s_data[s_index] - min)*factor;

  //Write back to global memory for output
  //Writes should be coalesces by warp of 32 threads.
  if( x < width && y < height )
  {
    d_output[i] = s_data[s_index];
  }
}

__global__ void
d_filter (uchar *d_output, uchar *d_input,  uint width, uint height)
{
  //Create a shared memory array for the section of the image the block will process.
  //The array needs to be larger than 16x16 so that it can get the pixel data for the rows
  //and columns surrounding the actual data.
  //Shared Memory Array is 3x16 (THREAD_X_SIZE) across and has 1 16x3 row above and below
  //the actual block pixel data.
  //Also, because shared data type is uchar (1 byte) we are going to allocate 4 bytes, 
  //but only use  the first one to store data.  
  //This "should" eliminate the four-way bank conflict that occurs
  //when adjacent threads access shared memory of 1 byte size.
   __shared__ uchar s_data[(THREAD_X_SIZE*THREAD_Y_SIZE*3 + THREAD_X_SIZE*6)*4];
  
  uint x = blockIdx.x * blockDim.x + threadIdx.x;
  uint y = blockIdx.y * blockDim.y + threadIdx.y;
  uint i = y * width + x;
  uint tid = blockDim.x*threadIdx.y + threadIdx.x;

  uint threadX = threadIdx.x;
  uint threadY = threadIdx.y;
  uint s_index;

  //Shared memory index of the pixel we are working on is located in the
  //center block.  Starting at 16x3 (top row) + 16x16 (left block)
  s_index = (THREAD_X_SIZE*4 + threadY*THREAD_X_SIZE*3 + threadX)*4;
  
  //Read current block data from global memory into our shared memory
  if( x < width && y < height )
  {
    //Do the blocks pixel first, ensuring we don't go outside the size of the image
    s_data[s_index] = d_input[i];
  }
  else
  {
    //There is not memset(0) and our calculations assumes data outside the image is zero.
    s_data[s_index] = 0;
  }

  //Now copy all the pixels around our blocks that are needed for the filter
  //There is no memset to zero, so these functions will set indexes outside of the
  //image to 0.    
  copyLeftBlock(s_data, d_input, width, height);      
  copyRightBlock(s_data, d_input, width, height);
  //Copy the rows around our pixel data, 16 threads(segments) at a time.    
  //Because we are only copying 16 bytes each row, we will have uncoalesced reads
  //Future improvement would have the first 48 threads read the entire section of data.
  if( tid < blockDim.x )
  {
    copyTopRow(s_data, d_input, width, height, 0);
  }
  else if( tid < 2*blockDim.x  )
  {
    copyBottomRow(s_data, d_input, width, height, blockDim.x );
  }      
  else if( tid < 3*blockDim.x  )
  {
    copyTopLeftRow(s_data, d_input, width, height, 2*blockDim.x );
  }
  else if( tid < 4*blockDim.x  )
  {
    copyBottomLeftRow(s_data, d_input, width, height, 3*blockDim.x );
  }
  else if( tid < 5*blockDim.x  )
  {
    copyTopRightRow(s_data, d_input, width, height, 4*blockDim.x );
  }
  else if( tid < 6*blockDim.x )
  {
    copyBottomRightRow(s_data, d_input, width, height, 5*blockDim.x );
  } 
  __syncthreads();          
  
  //Calculations....
  if( x < width && y < height )
  {
    //Determine the indexes in shared memory of the pixels we are interested in
    //for our 3x3 neighborhood.
    uint top_index = (THREAD_X_SIZE*4 + (threadY-1)*THREAD_X_SIZE*3 + threadX)*4;
    uint bottom_index = (THREAD_X_SIZE*4 + (threadY+1)*THREAD_X_SIZE*3 + threadX)*4;
    uint right_index = (THREAD_X_SIZE*4 + (threadY)*THREAD_X_SIZE*3 + (threadX+1))*4;
    uint left_index = (THREAD_X_SIZE*4 + (threadY)*THREAD_X_SIZE*3 + (threadX-1))*4;
    uint top_right_index = (THREAD_X_SIZE*4 + (threadY-1)*THREAD_X_SIZE*3 + (threadX+1))*4;
    uint top_left_index = (THREAD_X_SIZE*4 + (threadY-1)*THREAD_X_SIZE*3 + (threadX-1))*4;
    uint bottom_right_index = (THREAD_X_SIZE*4 + (threadY+1)*THREAD_X_SIZE*3 + (threadX+1))*4;
    uint bottom_left_index = (THREAD_X_SIZE*4 + (threadY+1)*THREAD_X_SIZE*3 + (threadX-1))*4;
    s_index = (THREAD_X_SIZE*4 + threadY*THREAD_X_SIZE*3 + threadX)*4;
  
    int blurFactor;
    //Determine if the pixels around our current pixel are present
    bool topPr = y & 0xFFFF;
    bool bottomPr = (height-y) & 0xFFFF;
    bool leftPr = x & 0xFFFF;
    bool rightPr = (width-x) & 0xFFFF;
  
    //Create the blurfactor we will use to divide the sum or multiplied pixels by.
    //If the pixel isn't present (from above) then it will not be included in the calculations.
    //Ie, the top left pixel will only have 3+2(right)+2(bottom)+1(bottom_right)=8 as
    //the blur factor.
    blurFactor = 3 + 2*(leftPr + rightPr) 
                   + 2*(topPr + bottomPr)
                   + 1*(topPr*leftPr + topPr*rightPr +
                        bottomPr*leftPr + bottomPr*rightPr);

    //Calculate the modified pixel value.  Get the pixel values from shared memory
    //If a pixel isn't present (top corners etc) then it is zero and doesn't
    //get included in the calculations.
    //This removes any control branching and should speed up calculations.
    //Also bank conflicts should be reduced, because each thread is accessing
    //only one bank (4x1byte) and they access the memory in a row of 16.
    //For those threads on the far left of the block, ie thread 0, they access bank 16 of 
    //the block to the left, but all other threads are accessign threadid-1, so thread 16
    //access block 15.                           
    blurFactor = (3*s_data[s_index] + 
                  2*s_data[left_index] + 2*s_data[right_index] +
                  2*s_data[top_index] + 2*s_data[bottom_index] + 
                  1*s_data[top_right_index] + 1*s_data[top_left_index] +
                  1*s_data[bottom_right_index] + 1*s_data[bottom_left_index])/blurFactor;
    s_data[s_index] = blurFactor;        
  }
    
  //Write back to global memory for output
  //Because we are accessing 4x1ybte there should be no shared memory bank conflicts
  //However writes to global memory may not be fully coalesced as we only write 
  //16 contiguous bytes at a time (the width of one block).  Each row in a block
  //goes to a different location in memory.
  if( x < width && y < height )
  {
    s_index = (THREAD_X_SIZE*4 + threadY*THREAD_X_SIZE*3 + threadX)*4;
    d_output[i] = s_data[s_index];
  } 
}



#endif // _BICUBIC_KERNEL_H_
