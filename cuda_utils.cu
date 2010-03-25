#ifndef _CUDA_UTILS_CU_
#define _CUDA_UTILS_CU_

void 
cudaSucceed(const char *msg)
{
  cudaError_t err;
  cudaThreadSynchronize ();
  err = cudaGetLastError();
    
  if (err == cudaSuccess)
  {
    return; 
  }
  else
  {
    fprintf (stderr, "CUDA error: %s: %s.\n", msg, cudaGetErrorString (err));
    exit(EXIT_FAILURE);
  }
}

#endif
