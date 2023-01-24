//==============================================================================
//
// 	sensor_epsonG325PDF0.h - Epson G325PDF0 sensor specific definitions
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
#ifndef EPSONG325PDF0_H_
#define EPSONG325PDF0_H_


#define EPSON_ACCL_SF         (.400)
#define EPSON_GYRO_SF         (.05)
#define EPSON_TEMP_SF         (0.0078125)
#define EPSON_ATTI_SF         (0.00699411)
#define EPSON_COUNT_SF        (16000)

#define EPSON_DA_SF0          (EPSON_GYRO_SF * 500E-06)
#define EPSON_DA_SF1          (EPSON_DA_SF0 * 2)
#define EPSON_DA_SF2          (EPSON_DA_SF1 * 2)
#define EPSON_DA_SF3          (EPSON_DA_SF2 * 2)
#define EPSON_DA_SF4          (EPSON_DA_SF3 * 2)
#define EPSON_DA_SF5          (EPSON_DA_SF4 * 2)
#define EPSON_DA_SF6          (EPSON_DA_SF5 * 2)
#define EPSON_DA_SF7          (EPSON_DA_SF6 * 2)
#define EPSON_DA_SF8          (EPSON_DA_SF7 * 2)
#define EPSON_DA_SF9          (EPSON_DA_SF8 * 2)
#define EPSON_DA_SF10         (EPSON_DA_SF9 * 2)
#define EPSON_DA_SF11         (EPSON_DA_SF10 * 2)
#define EPSON_DA_SF12         (EPSON_DA_SF11 * 2)
#define EPSON_DA_SF13         (EPSON_DA_SF12 * 2)
#define EPSON_DA_SF14         (EPSON_DA_SF13 * 2)
#define EPSON_DA_SF15         (EPSON_DA_SF14 * 2)

#define EPSON_DV_SF0          (EPSON_ACCL_SF * 1E-03 * 9.80665 * 500E-06)
#define EPSON_DV_SF1          (EPSON_DV_SF0 * 2)
#define EPSON_DV_SF2          (EPSON_DV_SF1 * 2)
#define EPSON_DV_SF3          (EPSON_DV_SF2 * 2)
#define EPSON_DV_SF4          (EPSON_DV_SF3 * 2)
#define EPSON_DV_SF5          (EPSON_DV_SF4 * 2)
#define EPSON_DV_SF6          (EPSON_DV_SF5 * 2)
#define EPSON_DV_SF7          (EPSON_DV_SF6 * 2)
#define EPSON_DV_SF8          (EPSON_DV_SF7 * 2)
#define EPSON_DV_SF9          (EPSON_DV_SF8 * 2)
#define EPSON_DV_SF10         (EPSON_DV_SF9 * 2)
#define EPSON_DV_SF11         (EPSON_DV_SF10 * 2)
#define EPSON_DV_SF12         (EPSON_DV_SF11 * 2)
#define EPSON_DV_SF13         (EPSON_DV_SF12 * 2)
#define EPSON_DV_SF14         (EPSON_DV_SF13 * 2)
#define EPSON_DV_SF15         (EPSON_DV_SF14 * 2)


/*                                      -- Commands --
    - ADDR_ address byte of transfer to select the register to access
    - CMD_  data byte of transfer to write to the register selected 
     
    - All accesses are 16 bit transfers
    - For SPI IF:
        - For SPI write accesses - 8-bit address with msb=1b (can be even or odd) + 8-bit write data
                                 - No response
        - For SPI read accesses - 8-bit address with msb=0b(even only) + 8-bit dummy data
                                - Response is transferred on MOSI on next SPI access
                                - Return value is 16-bit read data (high byte + low byte)    
    - For UART IF:
        - For UART write accesses - 8-bit address with msb=1b(can be even or odd) + 8-bit write data + Delimiter Byte
                                  - No response
        - For UART read accesses - 8-bit address with msb=0b(even only) + 8-bit dummy data + Delimiter Byte
                                 - Response is transferred immediately
                                 - Return value consists of Register Read Address + 16-bit read data (high byte + low byte) + Delimiter Byte
    
    - NOTE: Register Address Maps that depend on the WINDOW_ID (page) */


// WINDOW_ID 0
#define ADDR_MODE_CTRL_LO          0x02     // MODE_CTRL Byte0 (W0)
#define ADDR_MODE_CTRL_HI          0x03     // MODE_CTRL Byte1 (W0)
#define ADDR_DIAG_STAT             0x04     // DIAG_STAT Byte0 (W0)
#define ADDR_FLAG                  0x06     // FLAG(ND/EA) (W0)
#define ADDR_GPIO                  0x08     // GPIO  (W0)
#define ADDR_COUNT                 0x0A     // COUNT (W0)
#define ADDR_TEMP_HIGH             0x0E     // TEMPC HIGH (W0)
#define ADDR_TEMP_LOW              0x10     // TEMPC LOW  (W0)
#define ADDR_XGYRO_HIGH            0x12     // XGYRO HIGH (W0)
#define ADDR_XGYRO_LOW             0x14     // XGYRO LOW  (W0)
#define ADDR_YGYRO_HIGH            0x16     // YGYRO HIGH (W0)
#define ADDR_YGYRO_LOW             0x18     // YGYRO LOW  (W0)
#define ADDR_ZGYRO_HIGH            0x1A     // ZGYRO HIGH (W0)
#define ADDR_ZGYRO_LOW             0x1C     // ZGYRO LOW  (W0)
#define ADDR_XACCL_HIGH            0x1E     // XACCL HIGH (W0)
#define ADDR_XACCL_LOW             0x20     // XACCL LOW  (W0)
#define ADDR_YACCL_HIGH            0x22     // YACCL HIGH (W0)
#define ADDR_YACCL_LOW             0x24     // YACCL LOW  (W0)
#define ADDR_ZACCL_HIGH            0x26     // ZACCL HIGH (W0)
#define ADDR_ZACCL_LOW             0x28     // ZACCL LOW  (W0)

#define ADDR_ROLL_HIGH             0x64     // ROLL HIGH (W0)
#define ADDR_ROLL_LOW              0x66     // ROLL LOW  (W0)
#define ADDR_PITCH_HIGH            0x68     // PITCH HIGH (W0)
#define ADDR_PITCH_LOW             0x6A     // PITCH LOW  (W0)
#define ADDR_YAW_HIGH              0x6C     // YAW HIGH (W0)
#define ADDR_YAW_LOW               0x6E     // YAW LOW  (W0)

#define ADDR_XDLTA_HIGH            0x64     // XDLTA HIGH (W0)
#define ADDR_XDLTA_LOW             0x66     // XDLTA LOW  (W0)
#define ADDR_YDLTA_HIGH            0x68     // YDLTA HIGH (W0)
#define ADDR_YDLTA_LOW             0x6A     // YDLTA LOW  (W0)
#define ADDR_ZDLTA_HIGH            0x6C     // ZDLTA HIGH (W0)
#define ADDR_ZDLTA_LOW             0x6E     // ZDLTA LOW  (W0)
#define ADDR_XDLTV_HIGH            0x70     // XDLTV HIGH (W0)
#define ADDR_XDLTV_LOW             0x72     // XDLTV LOW  (W0)
#define ADDR_YDLTV_HIGH            0x74     // YDLTV HIGH (W0)
#define ADDR_YDLTV_LOW             0x76     // YDLTV LOW  (W0)
#define ADDR_ZDLTV_HIGH            0x78     // ZDLTV HIGH (W0)
#define ADDR_ZDLTV_LOW             0x7A     // ZDLTV LOW  (W0)


// WINDOW_ID 1
#define ADDR_SIG_CTRL_LO           0x00     // SIG_CTRL Byte0 (W1)
#define ADDR_SIG_CTRL_HI           0x01     // SIG_CTRL Byte1 (W1)
#define ADDR_MSC_CTRL_LO           0x02     // MSC_CTRL Byte0 (W1)
#define ADDR_MSC_CTRL_HI           0x03     // MSC_CTRL Byte1 (W1)
#define ADDR_SMPL_CTRL_LO          0x04     // SMPL_CTRL Byte0 (W1)
#define ADDR_SMPL_CTRL_HI          0x05     // SMPL_CTRL Byte1 (W1)
#define ADDR_FILTER_CTRL_LO        0x06     // FILTER_CTRL Byte0 (W1)
#define ADDR_FILTER_CTRL_HI        0x07     // FILTER_CTRL Byte1 (W1)
#define ADDR_UART_CTRL_LO          0x08     // UART_CTRL Byte0 (W1)
#define ADDR_UART_CTRL_HI          0x09     // UART_CTRL Byte1 (W1)
#define ADDR_GLOB_CMD_LO           0x0A     // GLOB_CMD Byte0 (W1)
#define ADDR_GLOB_CMD_HI           0x0B     // GLOB_CMD Byte1 (W1)
#define ADDR_BURST_CTRL1_LO        0x0C     // BURST_CTRL1 Byte0 (W1)
#define ADDR_BURST_CTRL1_HI        0x0D     // BURST_CTRL1 Byte1 (W1)
#define ADDR_BURST_CTRL2_LO        0x0E     // BURST_CTRL2 Byte0 (W1)
#define ADDR_BURST_CTRL2_HI        0x0F     // BURST_CTRL2 Byte1 (W1)
#define ADDR_POL_CTRL_LO           0x10     // POL_CTRL Byte0 (W1)
#define ADDR_POL_CTRL_HI           0x11     // POL_CTRL Byte1 (W1)
#define ADDR_DLT_CTRL_LO           0x12     // DLT_CTRL Byte0 (W1)
#define ADDR_DLT_CTRL_HI           0x13     // DLT_CTRL Byte1 (W1)
#define ADDR_ATTI_CTRL_LO          0x14     // ATTI_CTRL Byte0 (W1)
#define ADDR_ATTI_CTRL_HI          0x15     // ATTI_CTRL Byte1 (W1)

#define ADDR_PROD_ID1              0x6A     // PROD_ID1(W1)
#define ADDR_PROD_ID2              0x6C     // PROD_ID2(W1)
#define ADDR_PROD_ID3              0x6E     // PROD_ID3(W1)
#define ADDR_PROD_ID4              0x70     // PROD_ID4(W1)
#define ADDR_VERSION               0x72     // VERSION(W1)
#define ADDR_SERIAL_NUM1           0x74     // SERIAL_NUM1(W1)
#define ADDR_SERIAL_NUM2           0x76     // SERIAL_NUM2(W1)
#define ADDR_SERIAL_NUM3           0x78     // SERIAL_NUM3(W1)
#define ADDR_SERIAL_NUM4           0x7A     // SERIAL_NUM4(W1)
#define ADDR_WIN_CTRL              0x7E     // WIN_CTRL(W0 or W1)

#define CMD_BURST                  0x80     // Write value to Issue Burst Read
#define CMD_WINDOW0                0x00     // Write value for WIN_CTRL to change to Window 0
#define CMD_WINDOW1                0x01     // Write value for WIN_CTRL to change to Window 1
#define CMD_BEGIN_SAMPLING         0x01     // Write value for MODE_CMD_HI to begin sampling
#define CMD_END_SAMPLING           0x02     // Write value for MODE_CMD_HI to stop sampling
#define CMD_SOFTRESET              0x80     // Write value for GLOB_CMD_LO to issue Software Reset
#define CMD_INITIAL_BACKUP         0x10     // Write value for GLOB_CMD_LO to issue Initial_Backup
#define CMD_FLASHTEST              0x08     // Write value for MSC_CTRL_HI to issue Flashtest
#define CMD_SELFTEST               0x04     // Write value for MSC_CTRL_HI to issue Selftest

// Write values for ADDR_SMPL_CTRL_HI to set Output Rate
#define CMD_RATE2000               0x00     // TAP>=0
#define CMD_RATE1000               0x01     // TAP>=2
#define CMD_RATE500                0x02     // TAP>=4
#define CMD_RATE250                0x03     // TAP>=8
#define CMD_RATE125                0x04     // TAP>=16
#define CMD_RATE62_5               0x05     // TAP>=32
#define CMD_RATE31_25              0x06     // TAP>=64
#define CMD_RATE15_625             0x07     // TAP=128
#define CMD_RATE400                0x08     // TAP>=8
#define CMD_RATE200                0x09     // TAP>=16
#define CMD_RATE100                0x0A     // TAP>=32
#define CMD_RATE80                 0x0B     // TAP>=32
#define CMD_RATE50                 0x0C     // TAP>=64
#define CMD_RATE40                 0x0D     // TAP>=64
#define CMD_RATE25                 0x0E     // TAP=128
#define CMD_RATE20                 0x0F     // TAP=128

// Write values for FILTER_CTRL_LO to set Filter
#define CMD_FLTAP0                 0x00
#define CMD_FLTAP2                 0x01
#define CMD_FLTAP4                 0x02
#define CMD_FLTAP8                 0x03
#define CMD_FLTAP16                0x04
#define CMD_FLTAP32                0x05
#define CMD_FLTAP64                0x06
#define CMD_FLTAP128               0x07
#define CMD_FIRTAP32FC50           0x08
#define CMD_FIRTAP32FC100          0x09
#define CMD_FIRTAP32FC200          0x0A
#define CMD_FIRTAP32FC400          0x0B
#define CMD_FIRTAP64FC50           0x0C
#define CMD_FIRTAP64FC100          0x0D
#define CMD_FIRTAP64FC200          0x0E
#define CMD_FIRTAP64FC400          0x0F
#define CMD_FIRTAP128FC50          0x10
#define CMD_FIRTAP128FC100         0x11
#define CMD_FIRTAP128FC200         0x12
#define CMD_FIRTAP128FC400         0x13

// MODE STAT
#define VAL_SAMPLING_MODE          0x00
#define VAL_CONFIG_MODE            0x04

#endif /* EPSONG325PDF0_H_ */
