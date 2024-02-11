/*
 * qmc5883.c
 *
 *  Created on: Feb 8, 2024
 *      Author: Mehmet Dincer
 */

#include "../../QMC/QMC5883/qmc5883.h"



//<<<<<<<<<<<<<<<<-GLOBAL VARIABLES->>>>>>>>>>>>>>>>>>>
I2C_HandleTypeDef * i2c;
static float magneticDeclinationDegrees;
kalman_state hKalmanX;
kalman_state hKalmanY;
kalman_state hKalmanZ;

//<<<<<<<<<<<<<<<<-STATIC FUNCTIONS->>>>>>>>>>>>>>>>>>>
static HAL_StatusTypeDef QMC_getPositionAdc(Qmc * qmc);


/**
 * @brief Initialize the QMC5883's i2c register
 * @param[in] i2c global variable
 * @return if master connection is valid return HAL_OK
 */
HAL_StatusTypeDef QMC_qmcInit(I2C_HandleTypeDef * hi2c1, Qmc * qmc)
{
	HAL_StatusTypeDef status;

	i2c = hi2c1;

	HAL_I2C_Init(i2c);
	kalman1_init(&hKalmanX, 0, 5e2);
	kalman1_init(&hKalmanY, 0, 5e2);
	kalman1_init(&hKalmanZ, 0, 5e2);

	// configuration register-1 setting
	qmc->qmcReg.ControlReg1.reg  = 0x00;
	qmc->qmcReg.ControlReg1.MODE = qmc->qmcConfig.MODE;
	qmc->qmcReg.ControlReg1.ODR  = qmc->qmcConfig.ODR;
	qmc->qmcReg.ControlReg1.RNG  = qmc->qmcConfig.RNG;
	qmc->qmcReg.ControlReg1.OSR  = qmc->qmcConfig.OSR;

	// configuration register-2 setting
	qmc->qmcReg.ControlReg2.reg  = 0x00;
	qmc->qmcReg.ControlReg2.INT_ENB = qmc->qmcConfig.INT;
	qmc->qmcReg.ControlReg2.ROL_PNT = qmc->qmcConfig.ROL_PNT;

	// It is recommended that the register 0BH is written by 0x01
	qmc->qmcReg.setResetPeriod = 0x01;

	//send the initial values
	status = HAL_I2C_Mem_Write(i2c, QMC_SLAVE_ADDR, QMC_SET_RESET_ADDR,
							   1, &qmc->qmcReg.setResetPeriod, 1, 100);
	if(status != HAL_OK) return status;
	while(HAL_I2C_GetState(i2c) == HAL_I2C_STATE_BUSY_TX);


	status = HAL_I2C_Mem_Write(i2c, QMC_SLAVE_ADDR, QMC_CONTROL_REG1_ADDR,
							   1, &qmc->qmcReg.ControlReg1.reg, 1, 100);
	if(status != HAL_OK) return status;
	while(HAL_I2C_GetState(i2c) == HAL_I2C_STATE_BUSY_TX);

	status = HAL_I2C_Mem_Write(i2c, QMC_SLAVE_ADDR, QMC_CONTROL_REG2_ADDR,
							   1, &qmc->qmcReg.ControlReg2.reg, 1, 100);
	if(status != HAL_OK) return status;
	while(HAL_I2C_GetState(i2c) == HAL_I2C_STATE_BUSY_TX);

	return HAL_OK;
}

/**
 * @brief check the RDYB bit in polling mode
 * @param[in] qmc global variable
 * @return if data is ready return 1 else 0
 */
uint8_t QMC_isDataReady(Qmc * qmc)
{
	HAL_I2C_Mem_Read(i2c, QMC_SLAVE_ADDR, QMC_STATUS_REG1,
					 1, &qmc->qmcReg.StatusReg.reg, 1, 100);

	return qmc->qmcReg.StatusReg.DRDY;
}

/**
 * @brief read the X-Y-ZOUT
 * @param[in] qmc global variable
 * @return if can read succesfully return HAL_OK
 */
HAL_StatusTypeDef QMC_readData(Qmc * qmc)
{
	HAL_StatusTypeDef status;

	status = QMC_getPositionAdc(qmc);
	if(status != HAL_OK) return status;

	return HAL_OK;
}

/**
 * @brief read the register adc values and assign
 * @param[in] bmg global variable
 * @return HAL_I2c read return status
 */
HAL_StatusTypeDef QMC_getPositionAdc(Qmc * qmc)
{
	HAL_StatusTypeDef status;

	//read the 0H-6H register in one packet
	if(qmc->qmcConfig.ROL_PNT == ROL_PNT_ENABLE)
	{
		status = HAL_I2C_Mem_Read(i2c, QMC_SLAVE_ADDR, QMC_XOUT_LSB_ADDR,
								  1, &qmc->qmcReg.xLsb, QMC_ROL_READ_DATA_LEN, 100);
		if(status != HAL_OK) return status;
	}
	else
	{
		uint8_t * offsetPtr = &qmc->qmcReg.xLsb;
		for(int i = 0; i < 6; i++)
		{
			status = HAL_I2C_Mem_Read(i2c, QMC_SLAVE_ADDR, i,
									  1, &offsetPtr[i], 1, 100);
			if(status != HAL_OK) return status;
		}
	}

	//read temperature registers
	status = HAL_I2C_Mem_Read(i2c, QMC_SLAVE_ADDR, QMC_TOUT_LSB_ADDR, 1, &qmc->qmcReg.tempLsb, 1, 100);
	if(status != HAL_OK) return status;

	status = HAL_I2C_Mem_Read(i2c, QMC_SLAVE_ADDR, QMC_TOUT_MSB_ADDR, 1, &qmc->qmcReg.tempMsb, 1, 100);
	if(status != HAL_OK) return status;

	//casting
	qmc->xPosition   = qmc->qmcReg.xLsb | (qmc->qmcReg.xMsb << 8);
	qmc->xPosition = kalman1_filter(&hKalmanX, qmc->xPosition);

	qmc->yPosition   = qmc->qmcReg.yLsb | (qmc->qmcReg.yMsb << 8);
	qmc->yPosition = kalman1_filter(&hKalmanY, qmc->yPosition);

	qmc->zPosition   = qmc->qmcReg.zLsb | (qmc->qmcReg.zMsb << 8);
	qmc->zPosition = kalman1_filter(&hKalmanZ, qmc->zPosition);

	qmc->temperature = qmc->qmcReg.tempLsb | (qmc->qmcReg.tempMsb << 8);

	return HAL_OK;
}

/**
 * @brief Magnetic declination (also called magnetic variation) is the
 * angle between magnetic north and true north at a particular
 * location on the Earth's surface. The angle can change
 * over time due to polar wandering.
 * @param[in] declination degree
 * @param[in] declination minute
 * @return none
 */
void QMC_setMagneticDelination(uint16_t degree, uint8_t minute)
{
	magneticDeclinationDegrees = degree + minute / 60.0f;
}

/**
 * @brief The azimuth angle is an angular measurement in the horizontal plane,
 * typically measured clockwise from true north.
 * @param[in] qmc global variable
 * @return azimuth angle
 */
float QMC_getAzimuth(Qmc * qmc)
{
	QMC_getPositionAdc(qmc);

	float azimuth = atan2(qmc->xPosition, qmc->yPosition) * 180.0f / PI;
	azimuth += magneticDeclinationDegrees;

	azimuth = (azimuth < 0) ? (azimuth + 360) : azimuth;

	return azimuth;
}

/**
 * @brief enable data ready interrupt
 * @param[in] qmc global variable
 * @return none
 */
HAL_StatusTypeDef QMC_interruptEnable(Qmc * qmc)
{
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(i2c, QMC_SLAVE_ADDR, 1, QMC_CONTROL_REG2_ADDR,
							  &qmc->qmcReg.ControlReg2.reg, 1, 100);
	if(status != HAL_OK) return status;

	qmc->qmcReg.ControlReg2.INT_ENB = 1;

	return HAL_I2C_Mem_Write(i2c, QMC_SLAVE_ADDR, 1, QMC_CONTROL_REG2_ADDR,
			  	  	  	  	 &qmc->qmcReg.ControlReg2.reg, 1, 100);
}

/**
 * @brief disable data ready interrupt
 * @param[in] qmc global variable
 * @return none
 */
HAL_StatusTypeDef QMC_interruptDisable(Qmc * qmc)
{
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(i2c, QMC_SLAVE_ADDR, 1, QMC_CONTROL_REG2_ADDR,
							  &qmc->qmcReg.ControlReg2.reg, 1, 100);
	if(status != HAL_OK) return status;

	qmc->qmcReg.ControlReg2.INT_ENB = 0;

	return HAL_I2C_Mem_Write(i2c, QMC_SLAVE_ADDR, 1, QMC_CONTROL_REG2_ADDR,
			  	  	  	  	 &qmc->qmcReg.ControlReg2.reg, 1, 100);
}

/**
 * @brief Soft reset, restore default value of all registers.
 * @return none
 */
HAL_StatusTypeDef QMC_softReset()
{
	uint8_t reg = 1 << 7;

	return HAL_I2C_Mem_Write(i2c, QMC_SLAVE_ADDR, 1, QMC_CONTROL_REG2_ADDR,
			  	  	  	  	 &reg, 1, 100);
}
















