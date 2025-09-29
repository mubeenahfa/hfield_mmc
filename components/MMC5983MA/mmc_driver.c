#include <stdio.h>
#include "mmc_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"

static const char* TAG = "mmc sensor";

mmc_t mmc_d;
#define READ_REG(x) (0x80 | x)


static esp_err_t mmc_read(uint8_t addr, uint8_t * data_in, uint16_t length) __attribute__((unused));
static uint8_t mmc_read_singleb(uint8_t  address) __attribute__((unused));
static esp_err_t mmc_write_singleb(uint8_t adress, uint8_t value) __attribute__((unused));
static esp_err_t mmc_setRegisterBit(uint8_t registerAddress, uint8_t bitMask) __attribute__((unused));
static esp_err_t mmc_clearRegisterBit(const uint8_t registerAddress, const uint8_t bitMask) __attribute__((unused));
static bool mmc_isRegisterSet(const uint8_t registerAddress, const uint8_t bitMask) __attribute__((unused));
static void mmc_setControlBit(uint8_t registerAddress, const uint8_t bitMask) __attribute__((unused));
static void mmc_clearControlBit(uint8_t registerAddress, const uint8_t bitMask) __attribute__((unused));
static bool mmc_isControlBitSet(uint8_t registerAddress, const uint8_t bitMask) __attribute__((unused));
static int mmc_getTemperature() __attribute__((unused));
static void mmc_softReset() __attribute__((unused));
static void mmc_enableInterrupt() __attribute__((unused));
static void mmc_disableInterrupt() __attribute__((unused));
static bool mmc_isInterruptEnabled() __attribute__((unused));
static void mmc_disable3WireSPI() __attribute__((unused));
static bool mmc_is3WireSPIEnabled() __attribute__((unused));
static void mmc_performSetOperation() __attribute__((unused));
static void mmc_performResetOperation() __attribute__((unused));
static void mmc_enableAutomaticSetReset() __attribute__((unused));
static void mmc_disableAutomaticSetReset() __attribute__((unused));
static bool mmc_isAutomaticSetResetEnabled() __attribute__((unused));
static void mmc_enableXChannel() __attribute__((unused));
static void mmc_disableXChannel() __attribute__((unused));
static bool mmc_isXChannelEnabled() __attribute__((unused));
static void mmc_enableYZChannels() __attribute__((unused));
static void mmc_disableYZChannels() __attribute__((unused));
static bool mmc_areYZChannelsEnabled() __attribute__((unused));
static void mmc_setFilterBandwidth(mmc_band_t bandwidth) __attribute__((unused));
static void mmc_enableContinuousMode() __attribute__((unused));
static void mmc_disableContinuousMode() __attribute__((unused));
static bool mmc_isContinuousModeEnabled() __attribute__((unused));
static void mmc_setContinuousModeFrequency(mmc_cm_freq_t frequency) __attribute__((unused));
static void mmc_enablePeriodicSet() __attribute__((unused));
static void mmc_disablePeriodicSet() __attribute__((unused));
static bool mmc_isPeriodicSetEnabled() __attribute__((unused));
static void mmc_setPeriodicSetSamples(const uint16_t numberOfSamples) __attribute__((unused));
static uint32_t mmc_getMeasurementX() __attribute__((unused));
static uint32_t mmc_getMeasurementY() __attribute__((unused));
static uint32_t mmc_getMeasurementZ() __attribute__((unused));

static controlBitMemory_t controlBitMemory;
static spi_dev_handle_t spi_dev_handle_mmc;



esp_err_t mmc_spi_init(void)
{
	if(SPI_checkInit() != ESP_OK){
		ESP_LOGE(TAG, "SPI controller has error");
		return ESP_ERR_INVALID_STATE;
	}
	ESP_RETURN_ON_ERROR(SPI_registerDevice(&spi_dev_handle_mmc, SPI_SLAVE_mmc_PIN,
											SPI_SCK_10MHZ, 1, 2, 6), TAG, "SPI register failed");

	return ESP_OK;
}
esp_err_t mmc_init()
{
	mmc_spi_init();
	// Reset !!!
	mmc_enableAutomaticSetReset();
	mmc_enablePeriodicSet();
	mmc_setPeriodicSetSamples(100);
	mmc_setFilterBandwidth(mmc_BAND_100);
	mmc_setContinuousModeFrequency(mmc_FREQ_100HZ);
	mmc_enableContinuousMode();

	mmc_d.offsetX =   1741.0f;
	mmc_d.offsetY =  11276.5f;
	mmc_d.offsetZ =  -9830.4f;

	mmc_d.gainX = 6839.0f;
	mmc_d.gainY = 6660.5f;
	mmc_d.gainZ = 6553.6f;

	uint8_t ID = 0;
	mmc_read(PRODID, &ID, 1);

	if(ID == MMC_PROD_ID){
		ESP_LOGI(TAG, "mmc initialization gave: 0x%X", ID);
		return ESP_OK;
	}
	ESP_LOGW(TAG, "WARNING: mmc initialization gave: 0x%X", ID);
	return ESP_FAIL;
}

esp_err_t mmc_read_meas()
{
	if(mmc_isRegisterSet(STATUS, MEAS_M_DONE)){
		uint8_t buffer[7] = {0};
		mmc_read(XOUT0, buffer, 7);

		int32_t Xraw = (((uint32_t) buffer[0]) << 10) | (((uint32_t) buffer[1]) << 2) | ((buffer[6] & 0xC0) >> 6);
		int32_t Yraw = (((uint32_t) buffer[2]) << 10) | (((uint32_t) buffer[3]) << 2) | ((buffer[6] & 0x30) >> 4);
		int32_t Zraw = (((uint32_t) buffer[4]) << 10) | (((uint32_t) buffer[5]) << 2) | ((buffer[6] & 0x0C) >> 2);



		mmc_d.Xraw = Xraw;
		mmc_d.Yraw = Yraw;
		mmc_d.Zraw = Zraw;

		mmc_d.meas.mX = ((float)((Xraw - 131072)*2)) / 16384;
		mmc_d.meas.mY = ((float)((Yraw - 131072)*2)) / 16384;
		mmc_d.meas.mZ = ((float)((Zraw - 131072)*2)) / 16384;




		return ESP_OK;
	}
	return ESP_ERR_NOT_FOUND;
}



esp_err_t mmc_get_meas(mmc_meas_t * meas){
	*meas = mmc_d.meas;
	return ESP_OK;
}
static esp_err_t mmc_read(uint8_t addr, uint8_t * data_in, uint16_t length) {
	return SPI_transfer(spi_dev_handle_mmc, 0x02, addr, NULL, data_in, length);
}

static uint8_t mmc_read_singleb(uint8_t  addr) {
	uint8_t buf = 0;
	SPI_transfer(spi_dev_handle_mmc, 0x02, addr, NULL, &buf, 1);
	return buf;
}

static esp_err_t mmc_write_singleb(uint8_t addr, uint8_t data_out) {
	return SPI_transfer(spi_dev_handle_mmc, 0, addr, &data_out, NULL, 1);
}

static esp_err_t mmc_setRegisterBit(const uint8_t registerAddress, const uint8_t bitMask)
{
    uint8_t value = mmc_read_singleb(registerAddress);
    value |= bitMask;
    mmc_write_singleb(registerAddress, value);
    ESP_LOGD(TAG, "MMC5883MA %X Register set %X", registerAddress, value);
    return ESP_OK;
}

static esp_err_t mmc_clearRegisterBit(const uint8_t registerAddress, const uint8_t bitMask)
{
    uint8_t value = mmc_read_singleb(registerAddress);
    value &= ~bitMask;
    mmc_write_singleb(registerAddress, value);
    ESP_LOGD(TAG, "MMC5883MA %X Register set %X", registerAddress, value);
    return ESP_OK;
}

static bool mmc_isRegisterSet(const uint8_t registerAddress, const uint8_t bitMask)
{
    uint8_t value = mmc_read_singleb(registerAddress);
    return (value & bitMask);
}


static void mmc_setControlBit(uint8_t registerAddress, const uint8_t bitMask)
{
    uint8_t *shadowRegister = NULL;
    switch (registerAddress)
    {
    case INTCTRL0:
    {
        shadowRegister = &controlBitMemory.internalControl0;
    }
    break;

    case INTCTRL1:
    {
        shadowRegister = &controlBitMemory.internalControl1;
    }
    break;

    case INTCTRL2:
    {
        shadowRegister = &controlBitMemory.internalControl2;
    }
    break;

    case INTCTRL3:
    {
        shadowRegister = &controlBitMemory.internalControl3;
    }
    break;

    default:
        break;
    }

    if (shadowRegister)
    {
        *shadowRegister |= bitMask;
        mmc_write_singleb(registerAddress, *shadowRegister);
    }
}

static void mmc_clearControlBit(uint8_t registerAddress, const uint8_t bitMask)
{
    uint8_t *shadowRegister = NULL;


    switch (registerAddress)
    {
    case INTCTRL0:
    {
        shadowRegister = &controlBitMemory.internalControl0;
    }
    break;

    case INTCTRL1:
    {
        shadowRegister = &controlBitMemory.internalControl1;
    }
    break;

    case INTCTRL2:
    {
        shadowRegister = &controlBitMemory.internalControl2;
    }
    break;

    case INTCTRL3:
    {
        shadowRegister = &controlBitMemory.internalControl3;
    }
    break;

    default:
        break;
    }

    if (shadowRegister)
    {
        *shadowRegister &= ~bitMask;
        mmc_write_singleb(registerAddress, *shadowRegister);
    }
}

static bool mmc_isControlBitSet(uint8_t registerAddress, const uint8_t bitMask)
{

    switch (registerAddress)
    {
    case INTCTRL0:
    {
        return (controlBitMemory.internalControl0 & bitMask);
    }
    break;

    case INTCTRL1:
    {
        return (controlBitMemory.internalControl1 & bitMask);
    }
    break;

    case INTCTRL2:
    {
        return (controlBitMemory.internalControl2 & bitMask);
    }
    break;

    case INTCTRL3:
    {
        return (controlBitMemory.internalControl3 & bitMask);
    }
    break;

    default:
        break;
    }

    return false;
}

static int mmc_getTemperature()
{
    mmc_setRegisterBit(INTCTRL0, TM_T);
    do
    {
    	vTaskDelay(5 / portTICK_PERIOD_MS);
    } while (!mmc_isRegisterSet(STATUS, MEAS_T_DONE));

    uint8_t result = mmc_read_singleb(TOUT);

    return result; //static_cast<int>(temperature);
}

static void mmc_softReset()
{
    mmc_setRegisterBit(INTCTRL1, SW_RST);
    vTaskDelay(15 / portTICK_PERIOD_MS);
}

static void mmc_enableInterrupt()
{
	mmc_setControlBit(INTCTRL0, INT_MEAS_DONE_EN);
}
static void mmc_disableInterrupt()
{
	mmc_clearControlBit(INTCTRL0, INT_MEAS_DONE_EN);
}

static bool mmc_isInterruptEnabled()
{
    return mmc_isControlBitSet(INTCTRL0, INT_MEAS_DONE_EN);
}

static void mmc_disable3WireSPI()
{

	mmc_clearControlBit(INTCTRL3, SPI_3W);
}

static bool mmc_is3WireSPIEnabled()
{
    return mmc_isControlBitSet(INTCTRL3, SPI_3W);
}
static void mmc_performSetOperation()
{
    mmc_setRegisterBit(INTCTRL0, SET);
    vTaskDelay(1 / portTICK_PERIOD_MS);
}
static void mmc_performResetOperation()
{
    mmc_setRegisterBit(INTCTRL0, RESET);
    vTaskDelay(1 / portTICK_PERIOD_MS);
}
//HERE ARE AUTOMATIC SET RESET CODE FUNCTIONS 

static void mmc_enableAutomaticSetReset()
{
	mmc_setControlBit(INTCTRL0, AUTO_SR_EN);
}

static void mmc_disableAutomaticSetReset()
{
	mmc_clearControlBit(INTCTRL0, AUTO_SR_EN);
}

static bool mmc_isAutomaticSetResetEnabled()
{
    return mmc_isControlBitSet(INTCTRL0, AUTO_SR_EN);
}

//BELOW ARE CHANNEL ENABLES AND DISABLES

static void mmc_enableXChannel()
{
	mmc_clearControlBit(INTCTRL1, X_INHIBIT);
}

static void mmc_disableXChannel()
{
	mmc_setControlBit(INTCTRL1, X_INHIBIT);
}

static bool mmc_isXChannelEnabled()
{
    return mmc_isControlBitSet(INTCTRL1, X_INHIBIT);
}

static void mmc_enableYZChannels()
{
	mmc_clearControlBit(INTCTRL1, YZ_INHIBIT);
}

static void mmc_disableYZChannels()
{
	mmc_setControlBit(INTCTRL1, YZ_INHIBIT);
}

static bool mmc_areYZChannelsEnabled()
{
    return mmc_isControlBitSet(INTCTRL1, YZ_INHIBIT);
}

//BANDWITH SETTING
static void mmc_setFilterBandwidth(mmc_band_t bandwidth)
{
    switch (bandwidth)
    {
    case 800:
    {
    	mmc_setControlBit(INTCTRL1, BW0);
    	mmc_setControlBit(INTCTRL1, BW1);
    }
    break;

    case 400:
    {
    	mmc_clearControlBit(INTCTRL1, BW0);
    	mmc_setControlBit  (INTCTRL1, BW1);
    }
    break;

    case 200:
    {
    	mmc_setControlBit  (INTCTRL1, BW0);
    	mmc_clearControlBit(INTCTRL1, BW1);
    }
    break;

    case 100:
    default:
    {
    	mmc_clearControlBit(INTCTRL1, BW0);
    	mmc_clearControlBit(INTCTRL1, BW1);
    }
    break;
    }
}

static void mmc_enableContinuousMode()
{
	mmc_setControlBit(INTCTRL2, CMM_EN);
}

static void mmc_disableContinuousMode()
{
	mmc_clearControlBit(INTCTRL2, CMM_EN);
}

static bool mmc_isContinuousModeEnabled()
{
    return mmc_isControlBitSet(INTCTRL2, CMM_EN);
}


static void mmc_setContinuousModeFrequency(mmc_cm_freq_t frequency)
{
    switch (frequency)
    {
    case 1:
    {
        // CM_FREQ[2:0] = 001
    	mmc_clearControlBit(INTCTRL2, CM_FREQ_2);
        mmc_clearControlBit(INTCTRL2, CM_FREQ_1);
        mmc_setControlBit  (INTCTRL2, CM_FREQ_0);
    }
    break;

    case 10:
    {
        // CM_FREQ[2:0] = 010
    	mmc_clearControlBit(INTCTRL2, CM_FREQ_2);
        mmc_setControlBit  (INTCTRL2, CM_FREQ_1);
        mmc_clearControlBit(INTCTRL2, CM_FREQ_0);
    }
    break;

    case 20:
    {
        // CM_FREQ[2:0] = 011
    	mmc_clearControlBit(INTCTRL2, CM_FREQ_2);
    	mmc_setControlBit  (INTCTRL2, CM_FREQ_1);
    	mmc_setControlBit  (INTCTRL2, CM_FREQ_0);
    }
    break;

    case 50:
    {
        // CM_FREQ[2:0] = 100
    	mmc_setControlBit  (INTCTRL2, CM_FREQ_2);
    	mmc_clearControlBit(INTCTRL2, CM_FREQ_1);
    	mmc_clearControlBit(INTCTRL2, CM_FREQ_0);
    }
    break;

    case 100:
    {
        // CM_FREQ[2:0] = 101
    	mmc_setControlBit  (INTCTRL2, CM_FREQ_2);
    	mmc_clearControlBit(INTCTRL2, CM_FREQ_1);
    	mmc_setControlBit  (INTCTRL2, CM_FREQ_0);
    }
    break;

    case 200:
    {
        // CM_FREQ[2:0] = 110
    	mmc_setControlBit  (INTCTRL2, CM_FREQ_2);
    	mmc_setControlBit  (INTCTRL2, CM_FREQ_1);
    	mmc_clearControlBit(INTCTRL2, CM_FREQ_0);
    }
    break;

    case 1000:
    {
        // CM_FREQ[2:0] = 111
    	mmc_setControlBit(INTCTRL2, CM_FREQ_2);
    	mmc_setControlBit(INTCTRL2, CM_FREQ_1);
    	mmc_setControlBit(INTCTRL2, CM_FREQ_0);
    }
    break;

    case 0:
    default:
    {
        // CM_FREQ[2:0] = 000
    	mmc_clearControlBit(INTCTRL2, CM_FREQ_2);
    	mmc_clearControlBit(INTCTRL2, CM_FREQ_1);
    	mmc_clearControlBit(INTCTRL2, CM_FREQ_0);
    }
    break;
    }
}
//PEIODIC SET
static void mmc_enablePeriodicSet()
{
	mmc_setControlBit(INTCTRL2, EN_PRD_SET);
}

static void mmc_disablePeriodicSet()
{
	mmc_clearControlBit(INTCTRL2, EN_PRD_SET);
}

static bool mmc_isPeriodicSetEnabled()
{
    return mmc_isControlBitSet(INTCTRL2, EN_PRD_SET);
}


static void mmc_setPeriodicSetSamples(const uint16_t numberOfSamples)
{
    switch (numberOfSamples)
    {
    case 25:
    {
        // PRD_SET[2:0] = 001
    	mmc_clearControlBit(INTCTRL2, PRD_SET_2);
    	mmc_clearControlBit(INTCTRL2, PRD_SET_1);
    	mmc_setControlBit  (INTCTRL2, PRD_SET_0);
    }
    break;

    case 75:
    {
        // PRD_SET[2:0] = 010
    	mmc_clearControlBit(INTCTRL2, PRD_SET_2);
    	mmc_setControlBit  (INTCTRL2, PRD_SET_1);
    	mmc_clearControlBit(INTCTRL2, PRD_SET_0);
    }
    break;

    case 100:
    {
        // PRD_SET[2:0] = 011
    	mmc_clearControlBit(INTCTRL2, PRD_SET_2);
    	mmc_setControlBit  (INTCTRL2, PRD_SET_1);
    	mmc_setControlBit  (INTCTRL2, PRD_SET_0);
    }
    break;

    case 250:
    {
        // PRD_SET[2:0] = 100
    	mmc_setControlBit  (INTCTRL2, PRD_SET_2);
    	mmc_clearControlBit(INTCTRL2, PRD_SET_1);
    	mmc_clearControlBit(INTCTRL2, PRD_SET_0);
    }
    break;

    case 500:
    {
        // PRD_SET[2:0] = 101
    	mmc_setControlBit  (INTCTRL2, PRD_SET_2);
    	mmc_clearControlBit(INTCTRL2, PRD_SET_1);
    	mmc_setControlBit  (INTCTRL2, PRD_SET_0);
    }
    break;

    case 1000:
    {
        // PRD_SET[2:0] = 110
    	mmc_setControlBit  (INTCTRL2, PRD_SET_2);
    	mmc_setControlBit  (INTCTRL2, PRD_SET_1);
    	mmc_clearControlBit(INTCTRL2, PRD_SET_0);
    }
    break;

    case 2000:
    {
        // PRD_SET[2:0] = 111
    	mmc_setControlBit(INTCTRL2, PRD_SET_2);
    	mmc_setControlBit(INTCTRL2, PRD_SET_1);
    	mmc_setControlBit(INTCTRL2, PRD_SET_0);
    }
    break;

    case 1:
    default:
    {
        // PRD_SET[2:0] = 000
    	mmc_clearControlBit(INTCTRL2, PRD_SET_2);
    	mmc_clearControlBit(INTCTRL2, PRD_SET_1);
    	mmc_clearControlBit(INTCTRL2, PRD_SET_0);
    }
    break;
    }
}

static uint32_t mmc_getMeasurementX()
{

    mmc_setRegisterBit(INTCTRL0, TM_M);

   
    do
    {
        
    	vTaskDelay(5 / portTICK_PERIOD_MS);
    } while (!mmc_isRegisterSet(STATUS, MEAS_M_DONE));

    uint32_t temp = 0;
    uint32_t result = 0;
    uint8_t buffer[7] = {0};

    mmc_read(XOUT0, buffer, 7);

    temp = (uint32_t) buffer[XOUT0];
    temp = temp << XYZ_0_SHIFT;
    result |= temp;

    temp = (uint32_t) buffer[XOUT1];
    temp = temp << XYZ_1_SHIFT;
    result |= temp;

    temp = (uint32_t) buffer[XYZOUT2];
    temp &= X2_MASK;
    temp = temp >> 6;
    result |= temp;
    return result;
}


static uint32_t mmc_getMeasurementY()
{
    mmc_setRegisterBit(INTCTRL0, TM_M);
    do
    {
    	vTaskDelay(5 / portTICK_PERIOD_MS);
    } while (!mmc_isRegisterSet(STATUS, MEAS_M_DONE));

    uint32_t temp = 0;
    uint32_t result = 0;
    uint8_t registerValue = 0;

     registerValue = (mmc_read_singleb(YOUT0));

    temp = (uint32_t) registerValue;
    temp = temp << XYZ_0_SHIFT;
    result |= temp;

    registerValue = (mmc_read_singleb(YOUT1));

    temp = (uint32_t) registerValue;
    temp = temp << XYZ_1_SHIFT;
    result |= temp;

    registerValue = (mmc_read_singleb(XYZOUT2));
    temp = (uint32_t) registerValue;
    temp &= Y2_MASK;
    temp = temp >> 4;
    result |= temp;
    return result;
}

static uint32_t mmc_getMeasurementZ()
{
    mmc_setRegisterBit(INTCTRL0, TM_M);

    do
    {
    	vTaskDelay(5 / portTICK_PERIOD_MS);
    } while (!mmc_isRegisterSet(STATUS, MEAS_M_DONE));

    uint32_t temp = 0;
    uint32_t result = 0;
    uint8_t registerValue = 0;

    registerValue = (mmc_read_singleb(ZOUT0));

    temp = (uint32_t) registerValue;
    temp = temp << XYZ_0_SHIFT;
    result |= temp;

    registerValue = (mmc_read_singleb(ZOUT1));

    temp = (uint32_t) registerValue;
    temp = temp << XYZ_1_SHIFT;
    result |= temp;

    registerValue = (mmc_read_singleb(XYZOUT2));

    temp = (uint32_t) registerValue;
    temp &= Z2_MASK;
    temp = temp >> 2;
    result |= temp;
    return result;
}









