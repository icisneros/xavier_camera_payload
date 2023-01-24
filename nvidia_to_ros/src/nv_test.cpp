#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "nvbuf_utils.h"

#define INPUT_BUFFER_LAYOUT (NvBufferLayout_BlockLinear)
#define OUTPUT_BUFFER_LAYOUT (NvBufferLayout_Pitch)

#define RED_VAL (0x10)
#define GREEN_VAL (0x40)
#define BLUE_VAL (0xA0)
#define ALPHA_VAL (0x80)

void create_buffers(int width, int height, int *input_buf, int *output_buf)
{
    int ret;

    NvBufferCreateParams input_params;
    memset(&input_params,0,sizeof(input_params));
    input_params.payloadType = NvBufferPayload_SurfArray;
    input_params.width = width;
    input_params.height = height;
    input_params.layout = INPUT_BUFFER_LAYOUT;
    input_params.colorFormat = NvBufferColorFormat_ARGB32;
    input_params.nvbuf_tag = NvBufferTag_VIDEO_CONVERT;

    ret = NvBufferCreateEx (input_buf, &input_params);
    if (ret)
    {
        printf("%s: NvBufferCreateEx (1) Failed \n", __func__);
        exit(-1);
    }

    NvBufferCreateParams output_params;
    memset(&output_params,0,sizeof(output_params));
    output_params.payloadType = NvBufferPayload_SurfArray;
    output_params.width = width;
    output_params.height = height;
    output_params.layout = OUTPUT_BUFFER_LAYOUT;
    output_params.colorFormat = NvBufferColorFormat_ARGB32;
    output_params.nvbuf_tag = NvBufferTag_VIDEO_CONVERT;

    ret = NvBufferCreateEx (output_buf, &output_params);
    if (ret)
    {
        printf("%s: NvBufferCreateEx (2) Failed \n", __func__);
        exit(-1);
    }
}

void fill_input_buffer(int input_buf)
{
    int ret;
    void *map;
    NvBufferParams params = {0};

    ret = NvBufferGetParams (input_buf, &params);
    if (ret != 0)
    {
        printf("%s: NvBufferGetParams Failed \n", __func__);
        exit(-1);
    }

    ret = NvBufferMemMap (input_buf, 0, NvBufferMem_Read_Write, &map);
    if (ret != 0)
    {
        printf("%s: NvBufferMemMap Failed \n", __func__);
        exit(-1);
    }

    printf("Filling buffer %d w=%d h=%d...\n", input_buf, params.width[0], params.height[0]);

    for (uint32_t h = 0; h < params.height[0]; h++) {
        for (uint32_t w = 0; w < params.width[0]; w++) {
            uint8_t *pixel = &((uint8_t*)map)[h * params.pitch[0] + w * 4];
            pixel[3] = ALPHA_VAL;
            pixel[2] = RED_VAL;
            pixel[1] = GREEN_VAL;
            pixel[0] = BLUE_VAL;
        }
    }

    ret = NvBufferMemSyncForDevice (input_buf, 0, &map);
    if (ret != 0)
    {
        printf("%s: NvBufferMemSyncForDevice Failed \n", __func__);
        exit(-1);
    }

    ret = NvBufferMemUnMap (input_buf, 0, &map);
    if (ret != 0)
    {
        printf("%s: NvBufferMemUnMap Failed \n", __func__);
        exit(-1);
    }
}

void convert_buffer(int input_buf, int output_buf)
{
    int ret;

    NvBufferParams input_params = {0};
    ret = NvBufferGetParams (input_buf, &input_params);
    if (ret != 0)
    {
        printf("%s: NvBufferGetParams Failed \n", __func__);
        exit(-1);
    }

    NvBufferParams output_params = {0};
    ret = NvBufferGetParams (output_buf, &output_params);
    if (ret != 0)
    {
        printf("%s: NvBufferGetParams Failed \n", __func__);
        exit(-1);
    }

    NvBufferRect src_rect, dest_rect;
    src_rect.top = 0;
    src_rect.left = 0;
    src_rect.width = input_params.width[0];
    src_rect.height = input_params.height[0];
    dest_rect.top = 0;
    dest_rect.left = 0;
    dest_rect.width = output_params.width[0];
    dest_rect.height = output_params.height[0];

    NvBufferTransformParams transform_params;
    memset(&transform_params,0,sizeof(transform_params));
    transform_params.transform_flag = NVBUFFER_TRANSFORM_FILTER;
    transform_params.transform_flip = NvBufferTransform_None;
    transform_params.transform_filter = NvBufferTransform_Filter_Smart;
    transform_params.src_rect = src_rect;
    transform_params.dst_rect = dest_rect;

    // Convert Blocklinear to PitchLinear RGBA
    ret = NvBufferTransform(input_buf, output_buf, &transform_params);
    if (ret == -1)
    {
        printf("Transform failed\n");
        exit(-1);
    }
}

bool check_alpha_valid(int buffer)
{
    int ret;
    void *map;
    bool ok = true;
    NvBufferParams params = {0};

    ret = NvBufferGetParams (buffer, &params);
    if (ret != 0)
    {
        printf("%s: NvBufferGetParams Failed \n", __func__);
        exit(-1);
    }

    ret = NvBufferMemMap (buffer, 0, NvBufferMem_Read_Write, &map);
    if (ret != 0)
    {
        printf("%s: NvBufferMemMap Failed \n", __func__);
        exit(-1);
    }

    ret = NvBufferMemSyncForCpu (buffer, 0, &map);
    if (ret != 0)
    {
        printf("%s: NvBufferMemSyncForCpu Failed \n", __func__);
        exit(-1);
    }

    printf("Checking buffer %d w=%d h=%d...\n", buffer, params.width[0], params.height[0]);

    for (uint32_t h = 0; h < params.height[0]; h++) {
        for (uint32_t w = 0; w < params.width[0]; w++) {
            uint8_t *pixel = &((uint8_t*)map)[h * params.pitch[0] + w * 4];

            if (pixel[3] != ALPHA_VAL) {
                ok = false;
                printf("%s: pixel conversion (alpha) failed (%#02x -> %#02x) \n", __func__, ALPHA_VAL, pixel[3]);
            }

            if (pixel[2] != RED_VAL) {
                ok = false;
                printf("%s: pixel conversion (red) failed (%#02x -> %#02x) \n", __func__, RED_VAL, pixel[2]);
            }

            if (pixel[1] != GREEN_VAL) {
                ok = false;
                printf("%s: pixel conversion (green) failed (%#02x -> %#02x) \n", __func__, GREEN_VAL, pixel[1]);
            }

            if (pixel[0] != BLUE_VAL) {
                ok = false;
                printf("%s: pixel conversion (blue) failed (%#02x -> %#02x) \n", __func__, BLUE_VAL, pixel[0]);
            }

            if (!ok)
                break;
        }

        if (!ok)
            break;
    }

    ret = NvBufferMemUnMap (buffer, 0, &map);
    if (ret != 0)
    {
        printf("%s: NvBufferMemUnMap Failed \n", __func__);
        exit(-1);
    }

    return ok;
}

void free_buffers(int input_buf, int output_buf)
{
    NvBufferDestroy(input_buf);
    NvBufferDestroy(output_buf);
}

int main(int argc, char *argv[])
{
    int ret = 0;
    int32_t width = 3840, height = 2160;
    int input_buf, output_buf;
    bool input_alpha_valid, output_alpha_valid;

    printf("Creating buffers...\n");
    create_buffers(width, height, &input_buf, &output_buf);

    printf("Filling input buffer...\n");
    fill_input_buffer(input_buf);

    input_alpha_valid = check_alpha_valid(input_buf);
    if (!input_alpha_valid) {
        printf("Input buffer alpha NOT valid\n");
        ret = -1;
        goto finish;
    }

    printf("Converting input to output buffer...\n");
    convert_buffer(input_buf, output_buf);

    output_alpha_valid = check_alpha_valid(output_buf);
    if (!output_alpha_valid) {
        printf("Output buffer alpha NOT valid\n");
        ret = -1;
        goto finish;
    }

finish:
    free_buffers(input_buf, output_buf);

    if (ret != 0) {
        printf("Conversion failed.\n");
    } else {
        printf("Conversion OK.\n");
    }

    return ret;
}
