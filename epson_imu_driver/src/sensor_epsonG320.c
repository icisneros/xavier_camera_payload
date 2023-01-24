//==============================================================================
//
//  sensor_epsonG320.c - Epson IMU sensor protocol specific code for 
//                                 G320 
//
//
//  THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT, 
//  SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT
//  SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE OR CLAIM, ARISING FROM OR IN CONNECTION
//  WITH THE SOFTWARE OR THE USE OF THE SOFTWARE.
//
//==============================================================================
#include "sensor_epsonCommon.h"
#include "hcl.h"
#include "hcl_gpio.h"

/*****************************************************************************
** Function name:       registerDump
** Description:         Read all registers for debug purpose
** Parameters:          None
** Return value:        None
*****************************************************************************/
void registerDump(void)
{
  unsigned int debug = TRUE;
  printf("\r\nRegister Dump:\r\n");
  registerRead16(0x00, 0x02, debug);
  registerRead16(0x00, 0x04, debug);
  registerRead16(0x00, 0x06, debug);
  printf("\r\n");
  registerRead16(0x00, 0x08, debug);
  registerRead16(0x00, 0x0A, debug);
  registerRead16(0x00, 0x0E, debug);
  printf("\r\n");
  registerRead16(0x00, 0x10, debug);
  registerRead16(0x00, 0x12, debug);
  registerRead16(0x00, 0x14, debug);
  printf("\r\n");
  registerRead16(0x00, 0x16, debug);
  registerRead16(0x00, 0x18, debug);
  registerRead16(0x00, 0x1A, debug);
  printf("\r\n");
  registerRead16(0x00, 0x1C, debug);
  registerRead16(0x00, 0x1E, debug);
  registerRead16(0x00, 0x20, debug);
  printf("\r\n");
  registerRead16(0x00, 0x22, debug);
  registerRead16(0x00, 0x24, debug);
  registerRead16(0x00, 0x26, debug);
  printf("\r\n");
  registerRead16(0x00, 0x28, debug);
  registerRead16(0x01, 0x00, debug);
  registerRead16(0x01, 0x02, debug);
  printf("\r\n");
  registerRead16(0x01, 0x04, debug);
  registerRead16(0x01, 0x06, debug);
  registerRead16(0x01, 0x08, debug);
  printf("\r\n");
  registerRead16(0x01, 0x0A, debug);
  registerRead16(0x01, 0x0C, debug);
  registerRead16(0x01, 0x0E, debug);
  printf("\r\n");
  registerRead16(0x01, 0x10, debug);
  registerRead16(0x01, 0x6A, debug);
  registerRead16(0x01, 0x6C, debug);
  printf("\r\n");
  registerRead16(0x01, 0x6E, debug);
  registerRead16(0x01, 0x70, debug);
  registerRead16(0x01, 0x72, debug);
  printf("\r\n");
  registerRead16(0x01, 0x74, debug);
  registerRead16(0x01, 0x76, debug);
  registerRead16(0x01, 0x78, debug);
  printf("\r\n");
  registerRead16(0x01, 0x7A, debug);
  registerRead16(0x01, 0x7E, debug);
  printf("\r\n");
}
