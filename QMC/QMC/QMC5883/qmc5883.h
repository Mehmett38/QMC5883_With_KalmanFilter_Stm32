/*
 * qmc5883.h
 *
 *  Created on: Feb 8, 2024
 *      Author: Mehmet Dincer
 */

#ifndef QMC5883_QMC5883_H_
#define QMC5883_QMC5883_H_

#include "stm32f3xx_hal.h"
#include "math.h"
#include "../../QMC/Kalman/kalman_filter.h"

//<<<<<<<<<<<<<<<<<<<<<<<-MACROS->>>>>>>>>>>>>>>>>>>>>
#define QMC_DEFAULT_ADDR			(0x0D)
#define QMC_SLAVE_ADDR				(QMC_DEFAULT_ADDR << 1)
#define PI							(M_PI)

#define QMC_XOUT_LSB_ADDR			(0x00)
#define QMC_XOUT_MSB_ADDR			(0x01)
#define QMC_YOUT_LSB_ADDR			(0x02)
#define QMC_YOUT_MSB_ADDR			(0x03)
#define QMC_ZOUT_LSB_ADDR			(0x04)
#define QMC_ZOUT_MSB_ADDR			(0x05)
#define QMC_STATUS_REG1				(0x06)
#define QMC_TOUT_LSB_ADDR			(0x07)
#define QMC_TOUT_MSB_ADDR			(0x08)
#define QMC_CONTROL_REG1_ADDR		(0x09)
#define QMC_CONTROL_REG2_ADDR		(0x0A)
#define QMC_SET_RESET_ADDR			(0x0B)
#define QMC_STATUS_REG2_ADDR		(0x0C)
#define QMC_CHIP_ID_ADDR			(0x0D)

#define QMC_ROL_READ_DATA_LEN		(0x07)

//@refgroup MODE
#define MODE_STANDBY				(0x00)
#define MODE_CONTINIOUS				(0x01)

//@refgroup ODR
#define ODR_10HZ					(0x00)
#define ODR_50HZ					(0x01)
#define ODR_100HZ					(0x02)
#define ODR_200HZ					(0x03)

// @refgroup RNG
#define RNG_2G						(0x00)
#define RNG_8G						(0x01)

// @refgroup OSR
#define OSR_512						(0x00)
#define OSR_256						(0x01)
#define OSR_128						(0x02)
#define OSR_64						(0x03)

//@refgroup INT
#define INT_ENABLE					(0x01)
#define INT_DISABLE					(0x00)

//@refgroup ROL_PNT
#define ROL_PNT_ENABLE				(0x01)
#define ROL_PNT_DISABLE				(0x00)

//<<<<<<<<<<<<<<<<<<<<<<<-ENUMS->>>>>>>>>>>>>>>>>>>>>>


//<<<<<<<<<<<<<<<<<<<<<<<-STRUCTURES->>>>>>>>>>>>>>>>>>>>>>
typedef struct{
	uint8_t xLsb;					//!< XOUT[7:0] Read only
	uint8_t xMsb;					//!< XOUT[15:8] Read only
	uint8_t yLsb;					//!< YOUT[7:0] Read only
	uint8_t yMsb;					//!< YOUT[15:8] Read only
	uint8_t zLsb;					//!< ZOUT[7:0] Read only
	uint8_t zMsb;					//!< ZOUT[15:8] Read only

	union{							//!< status register Read only
		uint8_t reg;
		struct{
			uint8_t DRDY : 1;		//!< 0:1 -> no new data : new data ready
			uint8_t OVL  : 1;		//!< 0:1 -> if 1 any data out of range
			uint8_t DOR  : 1;		//!< 0:1 -> normal : data skipped for reading
			uint8_t      : 5;
		};
	}StatusReg;

	uint8_t tempLsb;
	uint8_t tempMsb;

	union{
		uint8_t reg;
		struct{
			uint8_t MODE : 2;		//!< operation mode
			uint8_t ODR  : 2;		//!< output data rate
			uint8_t RNG  : 2;		//!< range or sensitivity of the sensors
			uint8_t OSR  : 2;		//!< over sampling rate
		};
	}ControlReg1;

	union{
		uint8_t reg;
		struct{
			uint8_t INT_ENB : 1;	//!< Interrupt Pin enabling
			uint8_t         : 5;	//!< RESERVED
			uint8_t ROL_PNT : 1;	//!< Point roll over function enabling
			uint8_t SOFT_RST: 1;	//!< Soft reset
		};
	}ControlReg2;

	uint8_t setResetPeriod;			//!< recommended that the written by 0x01
	uint8_t chipId;					//!< It returns 0xFQmcRegF
}QmcReg;

typedef struct{
	uint8_t MODE;					// Mode Control @refgroup MODE
	uint8_t ODR;					// Output Data Rate @refgroup ODR
	uint8_t RNG;					// Full Scale @refgroup RNG
	uint8_t OSR;					// Over Sample Ratio @refgroup OSR
	uint8_t INT;					// Interrupt @refgroup INT
	uint8_t ROL_PNT;				// roll-over @refgroup ROL_PNT
}QmcConfig;

typedef struct{
	QmcReg qmcReg;
	QmcConfig qmcConfig;
	int16_t xPosition;
	int16_t yPosition;
	int16_t zPosition;
	int16_t temperature;
	float angle;
}Qmc;

//<<<<<<<<<<<<<<<<<<<<<<<<<<<-FUNCTIONS PROTOTYPES->>>>>>>>>>>>>>>>>>>>>>
HAL_StatusTypeDef QMC_qmcInit(I2C_HandleTypeDef * hi2c1, Qmc * qmc);
void QMC_setMagneticDelination(uint16_t degree, uint8_t minute);
uint8_t QMC_isDataReady(Qmc * qmc);
HAL_StatusTypeDef QMC_readData(Qmc * qmc);
float QMC_getAzimuth(Qmc * qmc);
HAL_StatusTypeDef QMC_interruptEnable(Qmc * qmc);
HAL_StatusTypeDef QMC_interruptDisable(Qmc * qmc);
HAL_StatusTypeDef QMC_softReset();



#endif /* QMC5883_QMC5883_H_ */

























