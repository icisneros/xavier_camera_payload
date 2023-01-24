#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <cuda_runtime.h>
#include <algorithm>
#include <random>
#include <chrono>

namespace RANSAC_cuda_tools{
#define CHECK_CUDA(call)\
{\
  const cudaError_t error=call;\
  if(error!=cudaSuccess)\
  {\
      printf("ERROR: %s:%d,",__FILE__,__LINE__);\
      printf("code:%d,reason:%s\n",error,cudaGetErrorString(error));\
      std::cerr<<(int)error<<","<<cudaGetErrorString(error)<<std::endl;\
      exit(1);\
  }\
}
#define debugging 1
#define INITIAL_ITER 512
#define MAX_ITER 1024
// RANSAC on CUDA
void findFundamentalMat_on_cuda(std::vector<cv::Point2f> &i1, std::vector<cv::Point2f> &i2,double threashold, double confidence,std::vector<uchar> & status);

}