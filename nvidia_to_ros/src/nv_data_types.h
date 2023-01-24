#ifndef _NV_DATA_TYPES_H_
#define _NV_DATA_TYPES_H_

#include <nvbuf_utils.h>

struct Params {
  NvBufferParams params;
  NvBufferParamsEx params_ex;
};

struct ParamsResponse {
  bool success;
};

#endif
