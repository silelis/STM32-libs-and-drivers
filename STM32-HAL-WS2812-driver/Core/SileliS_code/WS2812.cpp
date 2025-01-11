/*
 * WS2812.cpp
 *
 *  Created on: Apr 22, 2021
 *      Author: dbank
 */

#include "WS2812.h"
#include "string.h"

WS2812::WS2812 (uint8_t _ledsQuantity, TIM_HandleTypeDef* _htim, uint32_t _TIM_Channel) {
	ws2812_htim = _htim;
	ledsQuantity = _ledsQuantity;
	//minimal leds quantity is 1 in other case program may crash
	if (!ledsQuantity)
		ledsQuantity =1;

	ws2812_TIM_Channel = _TIM_Channel;

	_setPwmValue_bit0(((ws2812_htim->Init.Period+1)*1/3));	//1 bit duration is 1.25us and PWM value for Ws2812 bit 0 it is 1/3 of PWM period
	_setPwmValue_bit1(((ws2812_htim->Init.Period+1)*2/3));	//1 bit duration is 1.25us and PWM value for Ws2812 bit 0 it is 2/3 of PWM period

	//TODO: This option requires testing

//	if (resetBitsQuantity>(ledsQuantity*ws2812WordLenght))
//		pwmDataLen = resetBitsQuantity;
//	else
//		pwmDataLen = (ledsQuantity*ws2812WordLenght)+1;						//+1 for 0 PWM value which is reset condition

	//TODO: This option requires testing
	pwmDataLen = ledsQuantity*ws2812WordLenght+resetBitsQuantity;			//+40 - last 40 are reset bits (50us)
	//TODO: This option requires testing
	pwmData = new uint16_t [pwmDataLen];


	//memset
	if (!pwmData)
	{
		while(1);		//TODO: obsluga bledow assert lub uart lub Error_Handler
	}

	memset(&pwmData[0], 0, pwmDataLen*sizeof(uint16_t));		//set last 50 "bits" to 0 as a reset command
	//memset(&pwmData[(pwmDataLen-resetBitsQuantity)], 0, resetBitsQuantity*sizeof(uint16_t));		//set last 50 "bits" to 0 as a reset command
	for (int i =1; i<=ledsQuantity; i++){
		setLedColor(i, 0, 0, 0);
	}
}

WS2812::~WS2812() {
	ws2812_htim = nullptr;
	ledsQuantity = 0;
	delete [] pwmData;
	pwmData = nullptr;
}

void WS2812::_setPwmValue_bit0(uint16_t value){
	ws2812PwmBit0 = value;
}

void WS2812::_setPwmValue_bit1(uint16_t value){
	ws2812PwmBit1 =value;
}


void WS2812::setLedColor(uint8_t led, uint8_t Green, uint8_t Red, uint8_t Blue){

	uint32_t indx = 0;

	//given to function led value should be <1 to ledsQuantity>
	if (!led)
		led = 1;
	if (led>ledsQuantity)
		led = ledsQuantity;

	//but in pwmData led position is <1-1 to ledsQuantity-1>
	led = led -1;

	uint32_t data = (Green<<16)|(Red<<8)|Blue;

	for (int i = 23; i>=0; i--){
		if (data&(1<<i))
			pwmData[(led*ws2812WordLenght/*24*/)+indx/*i*/] = ws2812PwmBit1;
		else
			pwmData[(led*ws2812WordLenght/*24*/)+indx/*i*/] = ws2812PwmBit0;
		indx++;
	}

}

void WS2812::setLedColorWithBrightness(uint8_t led, uint8_t Green, uint8_t Red, uint8_t Blue, uint8_t Brightness){
	if (Brightness>100)
		Brightness = 100;

	float fBrightness = (float) Brightness/100;

	Green	= (uint8_t) Green*fBrightness;
	Red		= (uint8_t) Red*fBrightness;
	Blue	= (uint8_t) Blue*fBrightness;

	setLedColor(led, Green, Red, Blue);

}

void WS2812::_waitForPwmReady(void){
	HAL_TIM_StateTypeDef PWM_state=HAL_TIM_STATE_ERROR;
	do{
		PWM_state = HAL_TIM_PWM_GetState(ws2812_htim);
	}while(PWM_state !=HAL_TIM_STATE_READY);
}

void WS2812::sendData(void){
//	HAL_TIM_StateTypeDef PWM_state=HAL_TIM_STATE_ERROR;

//	do{
//		PWM_state = HAL_TIM_PWM_GetState(ws2812_htim);
//	}while(PWM_state !=HAL_TIM_STATE_READY);
	_waitForPwmReady();
	HAL_TIM_PWM_Start_DMA(ws2812_htim, ws2812_TIM_Channel, (uint32_t*) pwmData, pwmDataLen);
}
