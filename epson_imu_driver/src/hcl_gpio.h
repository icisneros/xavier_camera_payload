//==============================================================================
//
//	hcl_gpio.h - Seiko Epson Hardware Control Library
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

#ifndef HCL_GPIO_H_
#define HCL_GPIO_H_

// Prototypes for generic GPIO functions
int gpioInit(void);
int gpioRelease(void);

void gpioSet(uint8_t pin);
void gpioClr(uint8_t pin);
uint8_t gpioGetPinLevel(uint8_t pin);
  
// Dummy assignments if below signal are not connected to anything
#define EPSON_RESET                 0
#define EPSON_DRDY                  0
#define EPSON_CS                    0

#endif
