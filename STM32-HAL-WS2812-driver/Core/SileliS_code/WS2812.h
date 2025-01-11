/*
 * WS2812.h
 *
 *  Created on: Apr 22, 2021
 *      Author: dbank
 */

#ifndef _WS2812_H_
#define _WS2812_H_


#include "stm32f4xx.h"

class WS2812 {
public:
	WS2812 (uint8_t ledsQuantity, TIM_HandleTypeDef* _htim, uint32_t _TIM_Channel);
	~WS2812();
	void setLedColor(uint8_t led, uint8_t Green, uint8_t Red, uint8_t Blue);
	void setLedColorWithBrightness(uint8_t led, uint8_t Green, uint8_t Red, uint8_t Blue, uint8_t Brightness);
	void sendData(void);

protected:
	void _setPwmValue_bit0 (uint16_t value);
	void _setPwmValue_bit1 (uint16_t value);
	void _waitForPwmReady(void);

private:
	const uint8_t	ws2812WordLenght = 24; 		//24 bits per one led command
	const uint8_t	resetBitsQuantity = 40;		//+40 - last 540 are reset bits (50us)
	TIM_HandleTypeDef* ws2812_htim = nullptr;
	uint32_t ws2812_TIM_Channel;
	uint8_t 	ledsQuantity = 0;
	uint16_t	pwmDataLen =0;
	uint16_t*	pwmData = nullptr;
	uint16_t	ws2812PwmBit0;
	uint16_t	ws2812PwmBit1;


};

#endif /* _WS2812_H_ */
