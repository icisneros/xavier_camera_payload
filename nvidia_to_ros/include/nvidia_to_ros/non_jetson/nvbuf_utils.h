#ifndef _NVBUF_UTILS_H_
#define _NVBUF_UTILS_H_

#define MAX_NUM_PLANES 4
  
#define MAX_COMPOSITE_FRAME 16
  
#define NVBUF_CHROMA_SUBSAMPLING_HORIZ_DEFAULT 0
#define NVBUF_CHROMA_SUBSAMPLING_VERT_DEFAULT 1
  
#define NVBUF_MAX_SYNCOBJ_PARAMS 5
  
#define NVBUFFER_SYNCPOINT_WAIT_INFINITE 0xFFFFFFFF
  
typedef enum
  {
    NvBufferPayload_SurfArray,
    NvBufferPayload_MemHandle,
  } NvBufferPayloadType;
  
typedef enum
  {
    NvBufferDisplayScanFormat_Progressive = 0,
    NvBufferDisplayScanFormat_Interlaced,
  } NvBufferDisplayScanFormat;
  
typedef enum
  {
    NvBufferLayout_Pitch,
    NvBufferLayout_BlockLinear,
  } NvBufferLayout;
  
typedef enum
  {
    NvBufferMem_Read,
    NvBufferMem_Write,
    NvBufferMem_Read_Write,
  } NvBufferMemFlags;
  
typedef enum
  {
    NvBufferTag_NONE            = 0x0,
    NvBufferTag_CAMERA          = 0x200,
    NvBufferTag_JPEG            = 0x1500,
    NvBufferTag_PROTECTED       = 0x1504,
    NvBufferTag_VIDEO_ENC       = 0x1200,
    NvBufferTag_VIDEO_DEC       = 0x1400,
    NvBufferTag_VIDEO_CONVERT   = 0xf01,
  } NvBufferTag;
  
typedef enum
  {
    NvBufferColorFormat_YUV420,
    NvBufferColorFormat_YVU420,
    NvBufferColorFormat_YUV422,
    NvBufferColorFormat_YUV420_ER,
    NvBufferColorFormat_YVU420_ER,
    NvBufferColorFormat_NV12,
    NvBufferColorFormat_NV12_ER,
    NvBufferColorFormat_NV21,
    NvBufferColorFormat_NV21_ER,
    NvBufferColorFormat_UYVY,
    NvBufferColorFormat_UYVY_ER,
    NvBufferColorFormat_VYUY,
    NvBufferColorFormat_VYUY_ER,
    NvBufferColorFormat_YUYV,
    NvBufferColorFormat_YUYV_ER,
    NvBufferColorFormat_YVYU,
    NvBufferColorFormat_YVYU_ER,
    NvBufferColorFormat_ABGR32,
    NvBufferColorFormat_XRGB32,
    NvBufferColorFormat_ARGB32,
    NvBufferColorFormat_NV12_10LE,
    NvBufferColorFormat_NV12_10LE_709,
    NvBufferColorFormat_NV12_10LE_709_ER,
    NvBufferColorFormat_NV12_10LE_2020,
    NvBufferColorFormat_NV21_10LE,
    NvBufferColorFormat_NV12_12LE,
    NvBufferColorFormat_NV12_12LE_2020,
    NvBufferColorFormat_NV21_12LE,
    NvBufferColorFormat_YUV420_709,
    NvBufferColorFormat_YUV420_709_ER,
    NvBufferColorFormat_NV12_709,
    NvBufferColorFormat_NV12_709_ER,
    NvBufferColorFormat_YUV420_2020,
    NvBufferColorFormat_NV12_2020,
    NvBufferColorFormat_SignedR16G16,
    NvBufferColorFormat_A32,
    NvBufferColorFormat_YUV444,
    NvBufferColorFormat_GRAY8,
    NvBufferColorFormat_NV16,
    NvBufferColorFormat_NV16_10LE,
    NvBufferColorFormat_NV24,
    NvBufferColorFormat_NV24_10LE,
    NvBufferColorFormat_NV16_ER,
    NvBufferColorFormat_NV24_ER,
    NvBufferColorFormat_NV16_709,
    NvBufferColorFormat_NV24_709,
    NvBufferColorFormat_NV16_709_ER,
    NvBufferColorFormat_NV24_709_ER,
    NvBufferColorFormat_NV24_10LE_709,
    NvBufferColorFormat_NV24_10LE_709_ER,
    NvBufferColorFormat_NV24_10LE_2020,
    NvBufferColorFormat_NV24_12LE_2020,
    NvBufferColorFormat_RGBA_10_10_10_2_709,
    NvBufferColorFormat_RGBA_10_10_10_2_2020,
    NvBufferColorFormat_BGRA_10_10_10_2_709,
    NvBufferColorFormat_BGRA_10_10_10_2_2020,
    NvBufferColorFormat_Invalid,
  } NvBufferColorFormat;
  
typedef enum
  {
    NvBufferTransform_None,
    NvBufferTransform_Rotate90,
    NvBufferTransform_Rotate180,
    NvBufferTransform_Rotate270,
    NvBufferTransform_FlipX,
    NvBufferTransform_FlipY,
    NvBufferTransform_Transpose,
    NvBufferTransform_InvTranspose,
  } NvBufferTransform_Flip;
  
typedef enum
  {
    NvBufferTransform_Filter_Nearest,
    NvBufferTransform_Filter_Bilinear,
    NvBufferTransform_Filter_5_Tap,
    NvBufferTransform_Filter_10_Tap,
    NvBufferTransform_Filter_Smart,
    NvBufferTransform_Filter_Nicest,
  } NvBufferTransform_Filter;
  
typedef enum {
  NVBUFFER_TRANSFORM_CROP_SRC   = 1,
  NVBUFFER_TRANSFORM_CROP_DST   = 1 << 1,
  NVBUFFER_TRANSFORM_FILTER     = 1 << 2,
  NVBUFFER_TRANSFORM_FLIP       = 1 << 3,
} NvBufferTransform_Flag;
  
typedef enum {
  NVBUFFER_COMPOSITE  = 1,
  NVBUFFER_BLEND      = 1 << 1,
  NVBUFFER_COMPOSITE_FILTER  = 1 << 2,
} NvBufferComposite_Flag;
  
typedef struct _NvBufferSyncObjParams
{
  uint32_t syncpointID;
  uint32_t value;
}NvBufferSyncObjParams;
  
typedef struct _NvBufferSyncObjRec
{
  NvBufferSyncObjParams insyncobj[NVBUF_MAX_SYNCOBJ_PARAMS];
  uint32_t num_insyncobj;
  NvBufferSyncObjParams outsyncobj;
  uint32_t use_outsyncobj;
}NvBufferSyncObj;
  
typedef struct
{
  float r;
  float g;
  float b;
}NvBufferCompositeBackground;
  
typedef struct
{
  uint32_t top;
  uint32_t left;
  uint32_t width;
  uint32_t height;
}NvBufferRect;
  
typedef struct _NvBufferSession * NvBufferSession;
  
typedef struct _NvBufferChromaSubSamplingParams
{
  uint8_t chromaLocHoriz;
  uint8_t chromaLocVert;
}NvBufferChromaSubsamplingParams;
  
#define NVBUF_CHROMA_SUBSAMPLING_PARAMS_DEFAULT \
  {						\
    NVBUF_CHROMA_SUBSAMPLING_HORIZ_DEFAULT,	\
      NVBUF_CHROMA_SUBSAMPLING_VERT_DEFAULT	\
      }
  
typedef struct _NvBufferCreateParams
{
  int32_t width;
  int32_t height;
  NvBufferPayloadType payloadType;
  int32_t memsize;
  NvBufferLayout layout;
  NvBufferColorFormat colorFormat;
  NvBufferTag nvbuf_tag;
}NvBufferCreateParams;
  
typedef struct _NvBufferParams
{
  uint32_t dmabuf_fd;
  void *nv_buffer;
  NvBufferPayloadType payloadType;
  int32_t memsize;
  uint32_t nv_buffer_size;
  NvBufferColorFormat pixel_format;
  uint32_t num_planes;
  uint32_t width[MAX_NUM_PLANES];
  uint32_t height[MAX_NUM_PLANES];
  uint32_t pitch[MAX_NUM_PLANES];
  uint32_t offset[MAX_NUM_PLANES];
  uint32_t psize[MAX_NUM_PLANES];
  uint32_t layout[MAX_NUM_PLANES];
}NvBufferParams;
  
typedef struct _NvBufferParamsEx
{
  NvBufferParams params;
  int32_t startofvaliddata;
  int32_t sizeofvaliddatainbytes;
  NvBufferDisplayScanFormat scanformat[MAX_NUM_PLANES];
  uint32_t secondfieldoffset[MAX_NUM_PLANES];
  uint32_t blockheightlog2[MAX_NUM_PLANES];
  uint32_t physicaladdress[MAX_NUM_PLANES];
  uint64_t flags[MAX_NUM_PLANES];
  void *payloadmetaInfo;
  NvBufferChromaSubsamplingParams chromaSubsampling;
  bool is_protected;
  NvBufferSyncObj syncobj;
  void *reserved;
}NvBufferParamsEx;
  
typedef struct _NvBufferCompositeParams
{
  uint32_t composite_flag;
  uint32_t input_buf_count;
  NvBufferTransform_Filter composite_filter[MAX_COMPOSITE_FRAME];
  float dst_comp_rect_alpha[MAX_COMPOSITE_FRAME];
  NvBufferRect src_comp_rect[MAX_COMPOSITE_FRAME];
  NvBufferRect dst_comp_rect[MAX_COMPOSITE_FRAME];
  NvBufferCompositeBackground composite_bgcolor;
  NvBufferSession session;
}NvBufferCompositeParams;
  
typedef struct _NvBufferTransformParams
{
  uint32_t transform_flag;
  NvBufferTransform_Flip transform_flip;
  NvBufferTransform_Filter transform_filter;
  NvBufferRect src_rect;
  NvBufferRect dst_rect;
  NvBufferSession session;
}NvBufferTransformParams;

#endif
