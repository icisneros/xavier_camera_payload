#include "RANSAC_cuda_tools.h"
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <curand.h>
#include <curand_kernel.h>
namespace RANSAC_cuda_tools{


// __constant__ float parameters[10*INITIAL_ITER];

// compute fundamental mat, use shared memory
// find null space for fundamental mat, the dimension should be 1
void test(){
    std::cerr<<"hello world"<<std::endl;
}
__inline__  __device__ void find_null(float* matrix, int row, int col,float *null_space, bool &success)
{
    int pivate_idx = 0;
    success = true;
                    // printf(
                    //     "[%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f \n"
                    //     "%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f \n"
                    //     "%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f ]\n",
                    //     matrix[0*col],matrix[0*col+1],matrix[0*col+2],matrix[0*col+3],matrix[0*col+4],matrix[0*col+5],matrix[0*col+6],matrix[0*col+7],matrix[0*col+8],
                    //     matrix[1*col],matrix[1*col+1],matrix[1*col+2],matrix[1*col+3],matrix[1*col+4],matrix[1*col+5],matrix[1*col+6],matrix[1*col+7],matrix[1*col+8],
                    //     matrix[2*col],matrix[2*col+1],matrix[2*col+2],matrix[2*col+3],matrix[2*col+4],matrix[2*col+5],matrix[2*col+6],matrix[2*col+7],matrix[2*col+8]);
    // reduced to row echelon form
    for (int i=0;i<row-1;i++){
        // printf("pivate row %d\n",i);
        float pivate = matrix[col*i+pivate_idx];
        if (fabs(pivate) >1e-9){
            for (int j=i+1;j<row;j++){
                float m = matrix[col*j+pivate_idx]/pivate;
                matrix[col*j+pivate_idx] = 0;
                for (int k=pivate_idx+1;k<col;k++){
                    matrix[col*j+k] = matrix[col*j+k]-m*matrix[col*i+k];
                }
            }
        }
        else {
            success = false;
            break;
        }
        pivate_idx = pivate_idx + 1;
    }
    // __syncthreads();
    //find null space
    // visualize matrix
        // printf("[%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f \n"
        //                 "%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f \n"
        //                 "%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f \n"
        //                 "%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f \n"
        //                 "%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f \n"
        //                 "%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f \n"
        //                 "%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f \n"
        //                 "%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f ]\n",
        //                 matrix[0*col],matrix[0*col+1],matrix[0*col+2],matrix[0*col+3],matrix[0*col+4],matrix[0*col+5],matrix[0*col+6],matrix[0*col+7],matrix[0*col+8],
        //                 matrix[1*col],matrix[1*col+1],matrix[1*col+2],matrix[1*col+3],matrix[1*col+4],matrix[1*col+5],matrix[1*col+6],matrix[1*col+7],matrix[1*col+8],
        //                 matrix[2*col],matrix[2*col+1],matrix[2*col+2],matrix[2*col+3],matrix[2*col+4],matrix[2*col+5],matrix[2*col+6],matrix[2*col+7],matrix[2*col+8],
        //                 matrix[3*col],matrix[3*col+1],matrix[3*col+2],matrix[3*col+3],matrix[3*col+4],matrix[3*col+5],matrix[3*col+6],matrix[3*col+7],matrix[3*col+8],
        //                 matrix[4*col],matrix[4*col+1],matrix[4*col+2],matrix[4*col+3],matrix[4*col+4],matrix[4*col+5],matrix[4*col+6],matrix[4*col+7],matrix[4*col+8],
        //                 matrix[5*col],matrix[5*col+1],matrix[5*col+2],matrix[5*col+3],matrix[5*col+4],matrix[5*col+5],matrix[5*col+6],matrix[5*col+7],matrix[5*col+8],
        //                 matrix[6*col],matrix[6*col+1],matrix[6*col+2],matrix[6*col+3],matrix[6*col+4],matrix[6*col+5],matrix[6*col+6],matrix[6*col+7],matrix[6*col+8],
        //                 matrix[7*col],matrix[7*col+1],matrix[7*col+2],matrix[7*col+3],matrix[7*col+4],matrix[7*col+5],matrix[7*col+6],matrix[7*col+7],matrix[7*col+8]);
                // printf(
                //         "[%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f \n"
                //         "%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f \n"
                //         "%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f ]\n",
                //         matrix[0*col],matrix[0*col+1],matrix[0*col+2],matrix[0*col+3],matrix[0*col+4],matrix[0*col+5],matrix[0*col+6],matrix[0*col+7],matrix[0*col+8],
                //         matrix[1*col],matrix[1*col+1],matrix[1*col+2],matrix[1*col+3],matrix[1*col+4],matrix[1*col+5],matrix[1*col+6],matrix[1*col+7],matrix[1*col+8],
                //         matrix[2*col],matrix[2*col+1],matrix[2*col+2],matrix[2*col+3],matrix[2*col+4],matrix[2*col+5],matrix[2*col+6],matrix[2*col+7],matrix[2*col+8]);

                        // printf("                                                                 \n"
                        // "[%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f \n"
                        // "%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f \n"
                        // "%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f ]\n",
                        // matrix[5*col],matrix[5*col+1],matrix[5*col+2],matrix[5*col+3],matrix[5*col+4],matrix[5*col+5],matrix[5*col+6],matrix[5*col+7],matrix[5*col+8],
                        // matrix[6*col],matrix[6*col+1],matrix[6*col+2],matrix[6*col+3],matrix[6*col+4],matrix[6*col+5],matrix[6*col+6],matrix[6*col+7],matrix[6*col+8],
                        // matrix[7*col],matrix[7*col+1],matrix[7*col+2],matrix[7*col+3],matrix[7*col+4],matrix[7*col+5],matrix[7*col+6],matrix[7*col+7],matrix[7*col+8]);
    if (success){
        // int back_col = col-1;
        // null_space[back_col--] = 1;
        // while (back_col>0){
        //     for (int i=back_col+1;i<col;i++){
        //         null_space[back_col] = null_space[back_col]-matrix[back_col*row+i]*null_space[i];
        //     }
        //     null_space[back_col--] /= matrix[back_col*row+back_col];
        // }
        null_space[8] = 1;
        null_space[7] = -matrix[71]*null_space[8]/matrix[70];
        null_space[6] = -(matrix[62]*null_space[8]+matrix[61]*null_space[7])/matrix[60];
        null_space[5] = -(matrix[53]*null_space[8]+matrix[52]*null_space[7]+matrix[51]*null_space[6])/matrix[50];
        null_space[4] = -(matrix[44]*null_space[8]+matrix[43]*null_space[7]+matrix[42]*null_space[6]+matrix[41]*null_space[5])/matrix[40];
        null_space[3] = -(matrix[35]*null_space[8]+matrix[34]*null_space[7]+matrix[33]*null_space[6]+matrix[32]*null_space[5]+matrix[31]*null_space[4])/matrix[30];
        null_space[2] = -(matrix[26]*null_space[8]+matrix[25]*null_space[7]+matrix[24]*null_space[6]+matrix[23]*null_space[5]+matrix[22]*null_space[4]+matrix[21]*null_space[3])/matrix[20];
        null_space[1] = -(matrix[17]*null_space[8]+matrix[16]*null_space[7]+matrix[15]*null_space[6]+matrix[14]*null_space[5]+matrix[13]*null_space[4]+matrix[12]*null_space[3]+matrix[11]*null_space[2])/matrix[10];
        null_space[0] = -(matrix[8] * null_space[8]+ matrix[7] *null_space[7]+matrix[6]*null_space[6]+matrix[5]*null_space[5]+matrix[4]*null_space[4]+matrix[3]*null_space[3]+matrix[2]*null_space[2]+matrix[1]*null_space[1])/matrix[0];
        // int back_col = col-1;
    }
    // __syncthreads();
}

/*cur_d: cur pts
    forw_d: forw_pts
    out_null_space: 10*INIT_ITER, first element indicate success state

    one thread uses one group of eight pairs of points
    one thread calculates one model 
    in total 512 threads, i.e. threads' global indices (1-D) indicate which model to compute: iy *  
    each block contains pts.size() threads since we want to use shared memory to accelerate data fetching
    one thread copy one pairs of points to shared memory (dynamic)

*/
__global__ void computeFundamentalMat(float* cur_d,float* forw_d,int max_pts,curandStatePhilox4_32_10_t *state,float *out_null_space)
{
    // first max_pts * 2: cur_d
    // second max_pts *2: forw_d
    extern __shared__ float points[];
    // extern __shared__ float s_forw[];
    //copy data to shared memory to reduce latency
    // coordinate inside the memory
    int pts_idx = threadIdx.x + blockDim.x*threadIdx.y;
    points[pts_idx] = cur_d[pts_idx];
    points[pts_idx+max_pts] = cur_d[pts_idx+max_pts];
    points[pts_idx+max_pts*2] = forw_d[pts_idx];
    points[pts_idx+max_pts*3] = forw_d[pts_idx+max_pts];
    __syncthreads();

    // int ix = threadIdx.x+blockIdx.x*blockDim.x;
    // int iy = threadIdx.y+blockIdx.y*blockDim.y;
    // global coordinate, indicate model number

    int idx = threadIdx.x+blockIdx.x*blockDim.x + (threadIdx.y+blockIdx.y*blockDim.y)*gridDim.x*blockDim.x;
    int seed = idx;
    float4 randomidx1;
    float4 randomidx2;
    curand_init(seed, idx, 0, &state[idx]);
    randomidx1 = curand_uniform4(&state[idx]);
    randomidx2 = curand_uniform4(&state[idx]);
    int indices[8] = {(int)(randomidx1.x*max_pts),(int)(randomidx1.y*max_pts),(int)(randomidx1.z*max_pts)
    ,(int)(randomidx1.w*max_pts),(int)(randomidx2.x*max_pts),(int)(randomidx2.y*max_pts),(int)(randomidx2.z*max_pts),(int)(randomidx2.w*max_pts)};
    // printf("model %d random idx %d %d %d %d %d %d %d %d\n",idx,indices[0],indices[1],indices[2],indices[3],indices[4],indices[5],indices[6],indices[7]);
    // float* cur_row1 = (float*)((char*)cur_d);
    // float* cur_row2 = (float*)((char*)cur_d + cur_pitch);
    // float* forw_row1 = (float*)((char*)forw_d);
    // float* forw_row2 = (float*)((char*)forw_d + forw_pitch);
    // printf("pts_idx:%d cur_pt(%.3f, %.3f) forw_pt(%.3f, %.3f)\n",pts_idx,points[pts_idx], points[pts_idx+max_pts],points[pts_idx+max_pts*2], points[pts_idx+max_pts*3]);
    if (idx<INITIAL_ITER){
        float matrix[72];
        // we do not use a loop here to reduce latency
        //first row
        int p_idx = indices[0];
        float u1 = points[p_idx];
        float v1 = points[p_idx+max_pts];
        float u2 = points[p_idx+max_pts*2];
        float v2 = points[p_idx+max_pts*3];
        matrix[0] = u1*u2;
        matrix[1] = u1*v2;
        matrix[2] = u1;                   
        matrix[3] = v1*u2;
        matrix[4] = v1*v2;
        matrix[5] = v1;
        matrix[6] = u2;
        matrix[7] = v2;
        matrix[8] = 1;

        //second row
        p_idx = indices[1];
        u1 = points[p_idx];
        v1 = points[p_idx+max_pts];
        u2 = points[p_idx+max_pts*2];
        v2 = points[p_idx+max_pts*3];
        matrix[9] = u1*u2;
        matrix[10] = u1*v2;
        matrix[11] = u1;                   
        matrix[12] = v1*u2;
        matrix[13] = v1*v2;
        matrix[14] = v1;
        matrix[15] = u2;
        matrix[16] = v2;
        matrix[17] = 1;
        //third row
        p_idx = indices[2];
        u1 = points[p_idx];
        v1 = points[p_idx+max_pts];
        u2 = points[p_idx+max_pts*2];
        v2 = points[p_idx+max_pts*3];
        matrix[18] = u1*u2;
        matrix[19] = u1*v2;
        matrix[20] = u1;                   
        matrix[21] = v1*u2;
        matrix[22] = v1*v2;
        matrix[23] = v1;
        matrix[24] = u2;
        matrix[25] = v2;
        matrix[26] = 1;
        //forth row
        p_idx = indices[3];
        u1 = points[p_idx];
        v1 = points[p_idx+max_pts];
        u2 = points[p_idx+max_pts*2];
        v2 = points[p_idx+max_pts*3];
        matrix[27] = u1*u2;
        matrix[28] = u1*v2;
        matrix[29] = u1;                   
        matrix[30] = v1*u2;
        matrix[31] = v1*v2;
        matrix[32] = v1;
        matrix[33] = u2;
        matrix[34] = v2;
        matrix[35] = 1;
        //fifth row
        p_idx = indices[4];
        u1 = points[p_idx];
        v1 = points[p_idx+max_pts];
        u2 = points[p_idx+max_pts*2];
        v2 = points[p_idx+max_pts*3];
        matrix[36] = u1*u2;
        matrix[37] = u1*v2;
        matrix[38] = u1;                   
        matrix[39] = v1*u2;
        matrix[40] = v1*v2;
        matrix[41] = v1;
        matrix[42] = u2;
        matrix[43] = v2;
        matrix[44] = 1;
        //sixth row
        p_idx = indices[5];
        u1 = points[p_idx];
        v1 = points[p_idx+max_pts];
        u2 = points[p_idx+max_pts*2];
        v2 = points[p_idx+max_pts*3];
        matrix[45] = u1*u2;
        matrix[46] = u1*v2;
        matrix[47] = u1;                   
        matrix[48] = v1*u2;
        matrix[49] = v1*v2;
        matrix[50] = v1;
        matrix[51] = u2;
        matrix[52] = v2;
        matrix[53] = 1;
        //seventh row
        p_idx = indices[6];
        u1 = points[p_idx];
        v1 = points[p_idx+max_pts];
        u2 = points[p_idx+max_pts*2];
        v2 = points[p_idx+max_pts*3];
        matrix[54] = u1*u2;
        matrix[55] = u1*v2;
        matrix[56] = u1;                   
        matrix[57] = v1*u2;
        matrix[58] = v1*v2;
        matrix[59] = v1;
        matrix[60] = u2;
        matrix[61] = v2;
        matrix[62] = 1;
        //eighth row
        p_idx = indices[7];
        u1 = points[p_idx];
        v1 = points[p_idx+max_pts];
        u2 = points[p_idx+max_pts*2];
        v2 = points[p_idx+max_pts*3];
        matrix[63] = u1*u2;
        matrix[64] = u1*v2;
        matrix[65] = u1;                   
        matrix[66] = v1*u2;
        matrix[67] = v1*v2;
        matrix[68] = v1;
        matrix[69] = u2;
        matrix[70] = v2;
        matrix[71] = 1;

        // __syncthreads();

        // printf cannot support too much elements, print at most 3 rows
                    // printf("#######################################\n"
                    //     "pts_idx:%d global idx:%d\n"
                    //     "[%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f \n"
                    //     "%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f \n"
                    //     "%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f \n"
                    //     "%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f \n"
                    //     "%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f \n"
                    //     "%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f \n"
                    //     "%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f \n"
                    //     "%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f ]\n"
                    //     "###############################################\n",
                    //     pts_idx,idx,
                    //     matrix[0*9],matrix[0*9+1],matrix[0*9+2],matrix[0*9+3],matrix[0*9+4],matrix[0*9+5],matrix[0*9+6],matrix[0*9+7],matrix[0*9+8],
                    //     matrix[1*9],matrix[1*9+1],matrix[1*9+2],matrix[1*9+3],matrix[1*9+4],matrix[1*9+5],matrix[1*9+6],matrix[1*9+7],matrix[1*9+8],
                    //     matrix[2*9],matrix[2*9+1],matrix[2*9+2],matrix[2*9+3],matrix[2*9+4],matrix[2*9+5],matrix[2*9+6],matrix[2*9+7],matrix[2*9+8],
                    //     matrix[3*9],matrix[3*9+1],matrix[3*9+2],matrix[3*9+3],matrix[3*9+4],matrix[3*9+5],matrix[3*9+6],matrix[3*9+7],matrix[3*9+8],
                    //     matrix[4*9],matrix[4*9+1],matrix[4*9+2],matrix[4*9+3],matrix[4*9+4],matrix[4*9+5],matrix[4*9+6],matrix[4*9+7],matrix[4*9+8],
                    //     matrix[5*9],matrix[5*9+1],matrix[5*9+2],matrix[5*9+3],matrix[5*9+4],matrix[5*9+5],matrix[5*9+6],matrix[5*9+7],matrix[5*9+8],
                    //     matrix[6*9],matrix[6*9+1],matrix[6*9+2],matrix[6*9+3],matrix[6*9+4],matrix[6*9+5],matrix[6*9+6],matrix[6*9+7],matrix[6*9+8],
                    //     matrix[7*9],matrix[7*9+1],matrix[7*9+2],matrix[7*9+3],matrix[7*9+4],matrix[7*9+5],matrix[7*9+6],matrix[7*9+7],matrix[7*9+8]);
        float null_space[9] = {0,0,0,0,0,0,0,0,0};
        bool success = true;

        find_null(matrix,8,9,null_space,success);
        //                                         printf("\n"
        // "global idx%d \n"
        // "(u1,v1): (%.2f,%.2f) (u2,v2): (%.2f,%.2f)\n",
        // idx, u1,v1,u2,v2);
        // int col = 9;
                        // printf("                                                                 \n"
                        // "matrix for model %d:\n"
                        // "[%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f \n"
                        // "%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f \n"
                        // "%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f ]\n",
                        // idx,
                        // matrix[5*col],matrix[5*col+1],matrix[5*col+2],matrix[5*col+3],matrix[5*col+4],matrix[5*col+5],matrix[5*col+6],matrix[5*col+7],matrix[5*col+8],
                        // matrix[6*col],matrix[6*col+1],matrix[6*col+2],matrix[6*col+3],matrix[6*col+4],matrix[6*col+5],matrix[6*col+6],matrix[6*col+7],matrix[6*col+8],
                        // matrix[7*col],matrix[7*col+1],matrix[7*col+2],matrix[7*col+3],matrix[7*col+4],matrix[7*col+5],matrix[7*col+6],matrix[7*col+7],matrix[7*col+8]);
        out_null_space[idx*10] = (float)success;
        // for (int i=1;i<10;i++){
        out_null_space[idx*10+1] = null_space[0];
        out_null_space[idx*10+2] = null_space[1];
        out_null_space[idx*10+3] = null_space[2];
        out_null_space[idx*10+4] = null_space[3];
        out_null_space[idx*10+5] = null_space[4];
        out_null_space[idx*10+6] = null_space[5];
        out_null_space[idx*10+7] = null_space[6];
        out_null_space[idx*10+8] = null_space[7];
        out_null_space[idx*10+9] = null_space[8];
        // }

    }
    // __syncthreads();
    //remember to copy the null space parameters to constant memory in host
}
// test the model on each thread, use shared memory
// each block runs one model (512 blocks), each thread test one data (pts thread each)
// points value stored in shared memory
//512*pts_size
__global__ void testModel(float* cur_d,float* forw_d, int max_pts,float threashold,int* inlier_count,float* parameters,uchar* statuses)
{
    // first max_pts * 2: cur_d
    // second max_pts *2: forw_d
    extern __shared__ float points[];
    // extern __shared__ float s_forw[];
    //copy data to shared memory to reduce latency
    // coordinate inside the memory
    int pts_idx = threadIdx.x + blockDim.x*threadIdx.y;
    points[pts_idx] = cur_d[pts_idx];
    points[pts_idx+max_pts] = cur_d[pts_idx+max_pts];
    points[pts_idx+max_pts*2] = forw_d[pts_idx];
    points[pts_idx+max_pts*3] = forw_d[pts_idx+max_pts];
    points[max_pts*4] = 0;
    __syncthreads();
    int model_idx = blockIdx.x + blockDim.x*blockIdx.y;
    if (parameters[10*model_idx] != 0){
            //verify the point
            float u1 = points[pts_idx];
            float v1 = points[pts_idx+max_pts];
            float u2 = points[pts_idx+max_pts*2];
            float v2 = points[pts_idx+max_pts*3];
            float my_vec[9] = {u1*u2,u1*v2,u1,v1*u2,v1*v2,v1,u2,v2,1};
            // float* cur_row1 = (float*)((char*)cur_d);
            // float* cur_row2 = (float*)((char*)cur_d + cur_pitch);
            // float* forw_row1 = (float*)((char*)forw_d);
            // float* forw_row2 = (float*)((char*)forw_d + forw_pitch);
            float result = 0;
            result += my_vec[0]*parameters[10*model_idx + 1];
            result += my_vec[1]*parameters[10*model_idx + 2];
            result += my_vec[2]*parameters[10*model_idx + 3];
            result += my_vec[3]*parameters[10*model_idx + 4];
            result += my_vec[4]*parameters[10*model_idx + 5];
            result += my_vec[5]*parameters[10*model_idx + 6];
            result += my_vec[6]*parameters[10*model_idx + 7];
            result += my_vec[7]*parameters[10*model_idx + 8];
            result += my_vec[8]*parameters[10*model_idx + 9];
            statuses[model_idx*max_pts+pts_idx] = (uchar)(fabs(result)<threashold);
            atomicAdd(&points[max_pts*4],(int)(fabs(result)<threashold));
            // atomicAdd(&inlier_count[model_idx],1);
            // }
    }
    inlier_count[model_idx] = points[max_pts*4];
    // __syncthreads();
}

__host__ void findFundamentalMat_on_cuda(std::vector<cv::Point2f> &i1, std::vector<cv::Point2f> &i2,double threashold, double confidence,std::vector<uchar> & status){
    //Generate M sets of random indices in CPU eight-point-algorithm
    //M: number of iters
    if (i1.size()<32){
        cv::findFundamentalMat(i1, i2, cv::FM_RANSAC, 1,confidence, status);
        return;
    }
    #if DEBUGGING
    std::chrono::time_point<std::chrono::high_resolution_clock> startt, endt;
    startt = std::chrono::high_resolution_clock::now();
    std::cerr<<"init ransac parameters"<<std::endl;
    #endif

    int s = i1.size();
    int global_max_inlier = 0;
    int max_idx = 0;
    float* bestParameter = (float*)malloc(sizeof(float)*9);
    int repeatance = 0;
    // std::vector<unsigned> indices;
    // for (int i =0;i<s;i++){
    //         indices.push_back(i);
    // }
    // int endIdx = indices.size()-8*((int)(indices.size()/8));
    // Copy the points to GPU
    // std::cerr<<"init pts data on GPU"<<std::endl;
    std::chrono::time_point<std::chrono::high_resolution_clock> cvstartt, cvendt;
    cvstartt = std::chrono::high_resolution_clock::now();
    int width = s, height = 2;
    float*cur_pts_d;
    float* forw_pts_d;
    cudaMalloc(&cur_pts_d,sizeof(float)*width*height);
    cudaMalloc(&forw_pts_d,sizeof(float)*width*height);
    // size_t pitch1, pitch2;
    // size_t h_pitch = width*sizeof(float);
    // cudaMallocPitch(&cur_pts_d,&pitch1,width*sizeof(float),height);
    // cudaMallocPitch(&forw_pts_d,&pitch2,width*sizeof(float),height);
    // std::cerr<<"copy pts:"<<s<<std::endl;
    float *forw_pts_h = (float*)malloc(sizeof(float)*width*height);
    float *cur_pts_h = (float*)malloc(sizeof(float)*width*height);
    for (int i =0;i<s;i++){
        cur_pts_h[i] = i1[i].x;
        cur_pts_h[s+i] = i1[i].y;
        forw_pts_h[i] = i2[i].x;
        forw_pts_h[s+i] = i2[i].y;
    }
    cudaMemcpy(cur_pts_d,cur_pts_h,sizeof(float)*width*height,cudaMemcpyHostToDevice);
    cudaMemcpy(forw_pts_d,forw_pts_h,sizeof(float)*width*height,cudaMemcpyHostToDevice);
    // CHECK_CUDA(cudaMemcpy2D(cur_pts_d,pitch1,cur_pts_h,h_pitch,width*sizeof(float),height,cudaMemcpyHostToDevice));
    // CHECK_CUDA(cudaMemcpy2D(forw_pts_d,pitch1,forw_pts_h,h_pitch,width*sizeof(float),height,cudaMemcpyHostToDevice));
    free(cur_pts_h);
    free(forw_pts_h);
    cvendt = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> cvelapsed_seconds = cvendt-cvstartt;
    // std::cout<<"Copying feature points to CUDA costs:"<<std::endl<<std::fixed<<std::setprecision(6)<<cvelapsed_seconds.count() <<" seconds"<<std::endl;
    float* null_space_h = (float*)malloc(sizeof(float)*10*INITIAL_ITER);
    float* null_space_d;
    cudaMalloc(&null_space_d,sizeof(float)*10*INITIAL_ITER);
    int* inlier_count_d;
    int* inlier_count_h = (int*)malloc(sizeof(int)*INITIAL_ITER);
    cudaMalloc(&inlier_count_d,sizeof(int)*INITIAL_ITER);
    uchar* statuses;
    cudaMalloc(&statuses,sizeof(uchar)*s*INITIAL_ITER);
    // std::cerr<<"enter while loop"<<std::endl;
    //main loop to run each parallel operation 512 itteration each
    #ifdef DEBUGGING
        endt = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = endt-startt;
        std::cout<<"Allocate memory on CUDA costs:"<<std::endl<<std::fixed<<std::setprecision(6)<<elapsed_seconds.count() <<" seconds"<<std::endl;
    #endif
    while (true){

        #ifdef DEBUGGING
        startt = std::chrono::high_resolution_clock::now();
        #endif
        bool flip = false;
        curandStatePhilox4_32_10_t  *dev_random;
        cudaMalloc((void**)&dev_random,s*((int)(INITIAL_ITER/s)+1)*sizeof(curandState));
        // std::chrono::duration<double> elapsed_seconds = endt-startt;
        computeFundamentalMat<<<(int)(INITIAL_ITER/s)+1,s,sizeof(float)*s*2*2>>>(cur_pts_d,forw_pts_d,s,dev_random,null_space_d);
        // cudaDeviceSynchronize();
        cudaFree(dev_random);
        // std::cerr<<"finish compute"<<std::endl;
        // CHECK_CUDA(cudaFree(randomIdx));
        // CHECK_CUDA(cudaMemcpy(null_space_h,null_space_d,sizeof(float)*10*INITIAL_ITER,cudaMemcpyDeviceToHost));
        cudaMemcpy(inlier_count_d,inlier_count_h,sizeof(int)*INITIAL_ITER,cudaMemcpyHostToDevice);
        // CHECK_CUDA(cudaMemcpyToSymbol(parameters,null_space_h,sizeof(float)*10*INITIAL_ITER));
        // std::cerr<<"test Model"<<std::endl;
        testModel<<<INITIAL_ITER,s,sizeof(float)*(s*2*2+1)>>>(cur_pts_d,forw_pts_d,s,threashold,inlier_count_d,null_space_d,statuses);
        //  cudaDeviceSynchronize();
        //  std::cerr<<"memcpy to host"<<std::endl;
        cudaMemcpy(inlier_count_h,inlier_count_d,sizeof(int)*INITIAL_ITER,cudaMemcpyDeviceToHost);
        #ifdef DEBUGGING
        endt = std::chrono::high_resolution_clock::now();
        elapsed_seconds = endt-startt;
        std::cout<<"Computing fundamental matrices and verification models costs:"<<std::endl<<std::fixed<<std::setprecision(6)<<elapsed_seconds.count() <<" seconds"<<std::endl;
        #endif
        int max_inlier = global_max_inlier;

        // std::cout<<std::endl;
        for (int i=0;i<INITIAL_ITER;i++){
            // std::cout<<inlier_count_h[i]<<',';
            if (inlier_count_h[i]>max_inlier){
                max_idx = i;
                max_inlier = inlier_count_h[i];
                flip = true;
            }
        }
        if (flip){
            global_max_inlier = max_inlier;
            // CHECK_CUDA(cudaMemcpy(bestParameter,null_space_d+max_idx*10+1,sizeof(float)*9,cudaMemcpyDeviceToHost));
            // for (int k=0;k<9;k++){
            //     bestParameter[k] = null_space_h[10*max_idx+1+k];
            // }
            // CHECK_CUDA(cudaMemcpyFromSymbol(bestParameter,parameters,sizeof(float)*9,(max_idx*10+1)*sizeof(float)));
                // std::cout<<std::endl;
            // for (int i =0;i<9;i++){
            //     std::cout<<bestParameter[i]<<',';
            // }
        }
        // std::cout<<std::endl<<"best idx"<<max_idx<<std::endl;
        // std::cout<<"max inlier:"<<max_inlier<<"/"<<s<<std::endl;
        // compute iteration time 
        confidence = MAX(confidence, 0.);
        confidence = MIN(confidence, 1.);
        float numerator = std::log(1-confidence);
        // std::cerr<<"numerator"<< numerator<<std::endl;
        // std::cerr<<((float)max_inlier/s)<<std::endl;
        float denominator = std::log(1-std::pow((float)max_inlier/s,8));
        // std::cerr<<"denominator"<< denominator<<std::endl;
        // std::cerr<<"calculated iter"<< cvRound(numerator/denominator)<<std::endl;
        int iter = denominator>=0 || -numerator>= MAX_ITER*(-denominator) ? MAX_ITER : cvRound(numerator/denominator);
        // std::cerr<<"mim iter num"<<iter<<std::endl;
        repeatance++;
        if (repeatance*INITIAL_ITER>iter) break;
    }

    cudaFree(cur_pts_d);
    cudaFree(forw_pts_d);
    cudaFree(null_space_d);
    cudaFree(inlier_count_d);
    free(inlier_count_h);
    free(bestParameter);
    free(null_space_h);
    // startt = std::chrono::high_resolution_clock::now();
    uchar *status_h  = (uchar*)malloc(sizeof(uchar)*s);
    cudaMemcpy(status_h,statuses+max_idx*s,sizeof(uchar)*s,cudaMemcpyDeviceToHost);
        cudaFree(statuses);
    // std::cout<<"result"<<std::endl;
    status.resize(s);
    for (int i=0;i<s;i++){
        // std::cout<<"point:"<<i<<":"<<(int)status_h[i]<<std::endl;
        status[i] = status_h[i];
        // float u1 = i1[i].x;
        // float v1 = i1[i].y;
        // float u2 = i2[i].x;
        // float v2 = i2[i].y;
        // float my_vec[9] = {u1*u2,u1*v2,u1,v1*u2,v1*v2,v1,u2,v2,1};
        // float result=0;
        // for (int j=0;j<9;j++){
        //     result += my_vec[j]*bestParameter[j];
        // }
        // // std::cout<<"(pt"<<i<<":"<<fabs(result)<<"),";
        // if (fabs(result) < threashold){
        //     status[i] = 1;
        // }
        // else{
        //     status[i] = 0;
        //  }    
    }
    //  std::cout<<"result"<<std::endl;
    #if DEBUGGING
    // endt = std::chrono::high_resolution_clock::now();
    startt = std::chrono::high_resolution_clock::now();
    #endif
    // cudaFree(statuses);
    // cudaFree(cur_pts_d);
    // cudaFree(forw_pts_d);
    // cudaFree(null_space_d);
    // cudaFree(inlier_count_d);
    // free(inlier_count_h);
    // free(bestParameter);
    // free(null_space_h);
    free(status_h);
    #if DEBUGGING
        endt = std::chrono::high_resolution_clock::now();
        elapsed_seconds = endt-startt;
        std::cout<<"Free memory costs:"<<std::endl<<std::fixed<<std::setprecision(6)<<elapsed_seconds.count() <<" seconds"<<std::endl;
    #endif
}
}