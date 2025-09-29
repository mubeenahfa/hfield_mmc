#pragma once
#include "freertos/FreeRTOS.h"
#include "SPI_driver.h"
#include <stdbool.h>

//will change this pins later according to our schematic
#define SPI_SLAVE_mmc_PIN	34
typedef struct{
	float mX;			
	float mY;			
	float mZ;			
} mmc_meas_t;


typedef struct {
	int32_t Xraw;
	int32_t Yraw;
	int32_t Zraw;

	float gainX;			
	float gainY;			
	float gainZ;			

	float offsetX;			
	float offsetY;			
	float offsetZ;			

	mmc_meas_t meas;
} mmc_t;

typedef struct
  {
    uint8_t internalControl0;
    uint8_t internalControl1;
    uint8_t internalControl2;
    uint8_t internalControl3;
  } controlBitMemory_t;


esp_err_t mmc_init();
esp_err_t mmc_read_meas();
esp_err_t mmc_get_meas(mmc_meas_t * meas);


#define XOUT0     0x00
#define XOUT1     0x01
#define YOUT0     0x02
#define YOUT1     0x03
#define ZOUT0     0x04
#define ZOUT1     0x05
#define XYZOUT2   0x06
#define TOUT       0x07
#define STATUS      0x08
#define INTCTRL0  0x09
#define INTCTRL1  0x0a
#define INTCTRL2  0x0b
#define INTCTRL3  0x0C
#define PRODID     0x2F



#define MMC_PROD_ID         0x30

// Bits definitions
#define MEAS_M_DONE			(1 << 0)
#define MEAS_T_DONE			(1 << 1)
#define OTP_RD_DONE		(1 << 4)
#define TM_M				(1 << 0)
#define TM_T				(1 << 1)
#define INT_MEAS_DONE_EN	(1 << 2)
#define SET		(1 << 3)
#define RESET		(1 << 4)
#define AUTO_SR_EN			(1 << 5)
#define OTP_READ			(1 << 6)
#define BW0					(1 << 0)
#define BW1					(1 << 1)
#define X_INHIBIT			(1 << 2)
#define YZ_INHIBIT			(3 << 3)
#define SW_RST				(1 << 7)
#define CM_FREQ_0			(1 << 0)
#define CM_FREQ_1			(1 << 1)
#define CM_FREQ_2			(1 << 2)
#define CMM_EN				(1 << 3)
#define PRD_SET_0			(1 << 4)
#define PRD_SET_1			(1 << 5)
#define PRD_SET_2			(1 << 6)
#define EN_PRD_SET			(1 << 7)
#define SPI_3W				(1 << 6)
#define X2_MASK				(3 << 6)
#define Y2_MASK				(3 << 4)
#define Z2_MASK				(3 << 2)
#define XYZ_0_SHIFT			10
#define XYZ_1_SHIFT			2


  typedef enum mmc_cm_freq
{
	mmc_FREQ_1HZ = 1,
	mmc_FREQ_25HZ = 10,
	mmc_FREQ_75HZ = 20,
	mmc_FREQ_100HZ = 50,
	mmc_FREQ_250HZ = 100,
	mmc_FREQ_500HZ = 200,
	mmc_FREQ_1000HZ = 1000,
	mmc_FREQ_2000HZ = 0

} mmc_cm_freq_t;

typedef enum mmc_band
{
	mmc_BAND_100 = 100,
	mmc_BAND_200 = 200,
	mmc_BAND_400 = 400,
	mmc_BAND_800 = 800

} mmc_band_t;

