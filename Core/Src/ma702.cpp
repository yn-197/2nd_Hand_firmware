#include "ma702.h"

/**
 * Constructor
 */
MA702::MA702(SPI_HandleTypeDef* hspi, GPIO_TypeDef* arg_ps, uint16_t arg_cs){
    _cs = arg_cs;
    _ps = arg_ps;
    _spi = hspi;
    errorFlag = 0;
    position = 0;
}

#define MA702_EN_SPI HAL_GPIO_WritePin(_ps, _cs, GPIO_PIN_RESET)
#define MA702_DIS_SPI HAL_GPIO_WritePin(_ps, _cs, GPIO_PIN_SET)

/**
 * Initialiser
 * Sets up the SPI interface
 */
void MA702::init(){
    MA702::close();
    MA702::open();
    MA702_DIS_SPI;
}

/**
 * Closes the SPI connection
 */
void MA702::close(){
    if (HAL_SPI_DeInit(_spi) != HAL_OK)
    {
        //User error function
    }
}

/**
 * Opens the SPI connection
 */
void MA702::open(){
    if (HAL_SPI_Init(_spi) != HAL_OK)
    {
        //User error function
    }
}

/*
 * Read a register from the sensor
 */
uint16_t MA702::read(uint8_t registerAddress){
    uint8_t command1[2] = { static_cast<uint8_t>(0x40 | (registerAddress & 0x1F)), 0x00 };
    uint8_t data1[2] = {0};

    MA702_EN_SPI;
    HAL_SPI_TransmitReceive(_spi, command1, data1, 2, 100);
    MA702_DIS_SPI;
    HAL_Delay(1);

    uint8_t command2[2] = {0x00, 0x00};
    uint8_t data2[2] = {0};

    MA702_EN_SPI;
    HAL_SPI_TransmitReceive(_spi, command2, data2, 2, 100);
    MA702_DIS_SPI;

    return data2[0];
}

/**
 * Returns the raw angle directly from the sensor
 */
uint16_t MA702::getRawRotation(){
    uint8_t command[2] = {0x00, 0x00};
    uint8_t data[2] = {0x00, 0x00};

    MA702_EN_SPI;
    HAL_SPI_TransmitReceive(_spi, command, data, 2, 100);
    MA702_DIS_SPI;

    uint16_t angle = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    return angle;
}

/*
 * Check if an error has been encountered.
 */
uint8_t MA702::error(){
    return errorFlag;
}

/*
 * Get and clear the error register by reading it
 */
uint16_t MA702::getErrors(){
    //return MA702::read(MA702_MAGNET_FLAG);
}

/*
 * Returns the current zero position
 */
uint16_t MA702::getZeroPosition(){
    return position;
}

/*
 * Returns normalized angle value
 */
float MA702::normalize(float angle) {
    angle = fmod(angle, 360);
    if (angle < 0) {
        angle += 360;
    }
    return angle;
}

/*
 * Returns calculated angle value
 * EncoderBaseインターフェースに合わせて引数を追加
 */
float MA702::read2angle(uint16_t angle) {
    return (360.0f * angle) / 65536.0f;
}