/**
  ******************************************************************************
  * @file    OVC3860PSKey.cpp
  * @author  Dawid "SileliS" Bańkowski	d.bankowski(at)gmail.com
  * @author  Dawid "SileliS" Bańkowski	d.bankowski(at)gmail.com
  * @brief   OVC3860 PSKey manage class
  *          This file provides code to manage OVC3860 PSKeys.
  *          It is written in C++ and SMT32 HAL.
  *          To write this module author uses helpful informations from:
  *          		- OVC3860_RevE_PSKeys_Setting_v1.2.pdf
  *          		- https://github.com/tomaskovacik/OVC3860
					- https://github.com/rigelinorion/OVC3860-MemoryMap
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 Dawid Bańkowski.
  * All rights reserved.</center></h2>
  *
  *For education and private projects:
  * This software component is licensed by GNU Public Licence, * the "License" *;
  * You to copy this scope of file to give informations about copyright to future
  * commercial / open source projects.
  *
  *To use this library for commercial and open source projects please contact with
  *To  author to agree the terms of use.
  ******************************************************************************
  */

#include "OVC3860PSKey.h"

/**
  * @brief Object constructor.
  * @note  Constructs object, set hardware properties, clead buffers.
  *
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  *               the configuration information for the specified UART module.
  * @param  ResetGPIOx	Pointer to a port where reset pin is soldered
  * @param  GPIO_Pin	pin number of a port where reset pin is soldered
  * @retval n/a
  */
OVC3860PSKey::OVC3860PSKey(UART_HandleTypeDef* huart  /*block mode*/, GPIO_TypeDef* ResetGPIOx, uint16_t GPIO_Pin)
			 :OVC3860HardWare(huart  /*in DMA mode*/, ResetGPIOx, GPIO_Pin)
{
	_cleanReceiveDataArray();
	_cleanCommandDataArray();
}

/**
  * @brief Object destructor.
  * @note  n/a
  *
  * @param  n/a
  * @retval n/a
  */
OVC3860PSKey::~OVC3860PSKey(){

}

/**
  * @brief send command (read / write / quit) to OVC
  * @note  Command is build with appropriate method buildReadCommand/
  * 		buildWriteCommand.
  * 		Command is sent in blocking mode.
  *
  * @param  pAddress pointer to command array.
  * @param  Size amount of data to be send (mostly OVC header length for reading
  * 		 + additional data if writing
  * @retval true is UART gives 'HAL_OK'.
  */
bool OVC3860PSKey::sendRawData(uint8_t * pAddress, uint16_t Size){
	_cleanReceiveDataArray();
	if (HAL_UART_Transmit(OVC_huart, pAddress, Size, HAL_MAX_DELAY)==HAL_OK)
		return true;
	else
		return false;
}


/**
  * @brief receive data from OVC which are answer to (read / write
  * 		/ quit)
  * @note  Command is build with appropriate method buildReadCommand/
  * 		buildWriteCommand.
  * 		Command is sent in blocking mode.
  *
  * @param  pAddress pointer to command array.
  * @param  Size amount of data to be send (mostly OVC header length for reading
  * 		 + additional data if writing
  * @retval true is UART gives 'HAL_OK'.
  */
void OVC3860PSKey::receiveRawData(uint16_t Size, uint32_t Timeout){
	HAL_UART_Receive(OVC_huart, receiveDataArry, Size, Timeout);
}

/**
  * @brief build PSkey read mode command header and send
  * 		it to OVC3860
  * @note  sending 4 bytes because this is the length of
  * 		read command header
  *
  * @param  type - PSKey mode command type
  * @param  address - PSkey address it was counted based
  * 		 on PSkeys numbers and its data Range(byte) -
  * 		 pls take a look at OVC3860_RevE_PSKeys_Setting_v1.2.pdf
  *
  * 		 also You can give this number from OVC3860_memmap
  * 		 file uploaded on Github
  * @param  data2readLenght - amount of data to read, pls
  * 		 take a look at OVC3860_RevE_PSKeys_Setting_v1.2.pdf
  * 		 for Range(byte)
  * @retval	true if data had been send correctly
  */
bool OVC3860PSKey::buildReadCommand(uint8_t type, uint16_t address, uint16_t data2readLenght){
	_cleanCommandDataArray();
	//build command header
	commandDataArry[0]=type<<4;
	commandDataArry[0]= commandDataArry[0] | (uint8_t) (address >>8);
	commandDataArry[1]= (uint8_t) (address & 0b11111111);
	commandDataArry[2]= (uint8_t) (data2readLenght>>8);
	commandDataArry[3]= (uint8_t) (data2readLenght & 0b11111111);
	//send command
	return sendRawData(commandDataArry, 4);			//lenght of 4 because it is only read mode
}


/**
  * @brief	build PSkey write mode command header and send
  * 		it to OVC3860
  * @note	sending 4 bytes +  data2writeLenght because whole
  * 		 command is header + data
  *
  * @param	type - PSKey mode command type
  * @param	address - PSkey address it was counted based
  * 		 on PSkeys numbers and its data Range(byte) -
  * 		 pls take a look at OVC3860_RevE_PSKeys_Setting_v1.2.pdf
  *
  * 		 also You can give this number from OVC3860_memmap
  * 		 file uploaded on Github
  * @param	data2write - array with data that should be written in
  * 		 module
  * @param	data2writeLenght - "data2write" length. The max value
  * 		 of written data is given in OVC3860_RevE_PSKeys_Setting_v1.2.pdf
  * 		  in Range(byte) for each PSKey.
  * @retval	true if data had been send correctly
  */
bool OVC3860PSKey::buildWriteCommand(uint8_t type, uint16_t address, const void* data2write, uint16_t data2writeLenght){
	_cleanCommandDataArray();
	//build command header
	commandDataArry[0]=type<<4;
	commandDataArry[0]= commandDataArry[0] | (uint8_t) (address >>8);
	commandDataArry[1]= (uint8_t) (address & 0b11111111);
	commandDataArry[2]= (uint8_t) (data2writeLenght>>8);
	commandDataArry[3]= (uint8_t) (data2writeLenght & 0b11111111);
	//copy data which have to be written to OVC3860
	memcpy(&commandDataArry[4], data2write, data2writeLenght);
	return sendRawData(commandDataArry, data2writeLenght+4);	//lenght of 4 + data2writeLenght because it is sends header + data to write
}


/**
  * @brief	Read data from OVC3860
  * @note	Send read command to OVC3860 and recives answer.
  * 		 Check if answer is correct for particular type
  * 		  of command
  *
  * @param 	 address - PSkey address it was counted based
  * 		 on PSkeys numbers and its data Range(byte) -
  * 		 pls take a look at OVC3860_RevE_PSKeys_Setting_v1.2.pdf
  *
  * 		 also You can give this number from OVC3860_memmap
  * 		 file uploaded on Github
  * @param  data2readLenght - amount of data to read, pls
  * 		 take a look at OVC3860_RevE_PSKeys_Setting_v1.2.pdf
  * 		 for Range(byte).
  * @retval	true if data had been send correctly and answer is correct.
  */
bool OVC3860PSKey::readDataFromOVC(uint16_t address, uint16_t data2readLenght){
	bool retVAL = buildReadCommand(CommandType.read, address, data2readLenght);
	receiveRawData(data2readLenght+4, 200);					//4 bytes is message header + data2readLenght
	if ((receiveDataArry[0]>>4) ==CommandType.readACK)
		return retVAL;
	return false;
}


/**
  * @brief	write data from OVC3860
  * @note	Send write command to OVC3860 and recives answer.
  * 		 Check if answer is correct for particular type
  * 		  of command
  *
  * @param 	 address - PSkey address it was counted based
  * 		 on PSkeys numbers and its data Range(byte) -
  * 		 pls take a look at OVC3860_RevE_PSKeys_Setting_v1.2.pdf
  *
  * 		 also You can give this number from OVC3860_memmap
  * 		 file uploaded on Github
  * @param 	 data2write - information that should be written.
  * @param  data2writeLenght - amount of data to write, pls
  * 		 take a look at OVC3860_RevE_PSKeys_Setting_v1.2.pdf
  * 		 for Range(byte).
  * @retval	true if data to write had been send correctly and
  * 		 answer is correct.
  */
bool OVC3860PSKey::writeData2OVC(uint16_t address, const char* data2write, uint16_t data2writeLenght){
		bool retVAL = buildWriteCommand(CommandType.wrtite, address, (void*) data2write, data2writeLenght);
		receiveRawData(4, 200);
		if ((receiveDataArry[0]>>4) ==CommandType.wrtiteACK)
				return retVAL;
			return false;
}

/**
  * @brief	Clean receive data array.
  * @note	Set value at '\0' which in some cases it
  * 		 character that point OVC3860 value
  * 		  termination mark
  *
  * @param	n/a
  * @retval n/a
  */
void OVC3860PSKey::_cleanReceiveDataArray(){
	memset(&receiveDataArry, '\0', receiveDataArry_lenght);
}

/**
  * @brief	Clean command data array.
  * @note	Set value at '\0' which in some cases it
  * 		 character that point OVC3860 value
  * 		  termination mark
  *
  * @param	n/a
  * @retval n/a
  */
void OVC3860PSKey::_cleanCommandDataArray(){
	memset(&commandDataArry, '\0', receiveDataArry_lenght);
}

/**
  * @brief	Enter OVC3860 into PSKey configuration
  * 		 mode by sending appropriate command.
  * @note	It is importand to send enterConfig
  * 		 before module send '\r\n' sequence of
  * 		  welcome message.
  * 		Normally welcome message is {0x04, 0x0F,
  * 		 0x04, 0x00, 0x01, 0x00, 0x00, '\r',
  * 		 '\n'} but to enter configuration mode
  * 		 You have to do this before '\r\n'.
  *
  * @param	n/a
  * @retval	true - if config mode had been entered
  */
bool OVC3860PSKey::enterConfigMode(){
	struct {
		uint8_t const welcome[7] 			= {0x04, 0x0F, 0x04, 0x00, 0x01, 0x00, 0x00};
		uint8_t const enterConfig[9] 		= {0xC5, 0xC7, 0xC7, 0xC9, 0xD0, 0xD7, 0xC9, 0xD1, 0xCD};
		uint8_t const enterConfigACK[7]		= {0x04, 0x0F, 0x04, 0x01, 0x01, 0x00, 0x00};
	} const Message;

	resetModule();

	receiveRawData(7); 		//this should be executed ASAP after reset line goes high
	if (memcmp(&receiveDataArry,&Message.welcome, 7)==0)
	{
		sendRawData((uint8_t*) &Message.enterConfig, 9);		//to enter config mode appropriate message should be sent
		receiveRawData(7);

		if (memcmp(&receiveDataArry,&Message.enterConfigACK, 7)==0){		//check for module answer
			return true;													//config message enabled
		}
	}
		return false;														//problem with config message enabling
}

/**
  * @brief	quit configuration mode.
  * @note	After quitting it's better to reset
  * 		 module.
  *
  * @param	n/a
  * @retval	true - if quited config mode.
  */
bool OVC3860PSKey::quitConfigMode(){
	if (buildReadCommand(CommandType.quitConfigMode, 0x0, 0x0))				//quit config mode command if 0x5 0x0 0x0 0x0
	{
		receiveRawData(4);
		if ((receiveDataArry[0]>>4) == CommandType.quitConfigModeACK)
			return true;
	}
	return false;
}

/**
  * @brief	read Bluetooth name with which other devices
  * 		 detects OVC3860.
  * @note	Name is stored in receiveDataArry at position [0].
  * 		returned value is length of name.
  * 		data2readLength is fixed at 16, because acc. to
  * 		OVC3860_RevE_PSKeys_Setting_v1.2.pdf for localname
  * 		it is max length.
  *
  * 		OVC3860 detects the end of name as '\0' mark so answer:
  * 		'N','A','M','E','_','1','\0','g,'a','r','b','a','g','e'
  * 		means that OVC mane is "NAME_1".
  *
  * 		I do not check what will happen if name is full 16 bytes without '\0'.
  *
  * @param	n/a
  * @retval length - of name. The read NAME itself is stored in receiveDataArry starting with position 0
  */
uint8_t OVC3860PSKey::readBtName(){
	if (readDataFromOVC(OVC3860_PSKEY_ADDR_NAME, 16))
		{
		memmove(receiveDataArry, receiveDataArry+4, 16);

		uint8_t i =0;
		do
		{
			i++;
		}while(receiveDataArry[i]!= '\0' && i < 16);

		return i;			//return lenght of name, name is stored in receiveDataArry
		}
}


/**
  * @brief	Write / change name of OVC3860 which will be
  * 		 broadcasted to other devices.
  * @note	Commans have length of 16 (max length of this type of command).
  * 		The name array should be ended with '\0' which means the end of name.
  *
  * 		I do not check what will happen if name is full 16 bytes without '\0'.
  *
  * @param	name - pointer to char array fith name i.e. xxx.writeBtName("Name_1");
  * @retval	true - if write command was executed correctly.
  */
bool 	OVC3860PSKey::writeBtName(const char* name){
		return writeData2OVC(OVC3860_PSKEY_ADDR_NAME, name,  16);
}
