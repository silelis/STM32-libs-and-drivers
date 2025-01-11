/**
  ******************************************************************************
  * @file    OVC3860_device.cpp
  * @author  Dawid "SileliS" Bańkowski	d.bankowski(at)gmail.com
  * @brief   OVC3860 hardware and audio/ phone classes.
  *          This file provides code to manage OVC3860 audio/ phone.
  *          It is written in C++ and SMT32 HAL.
  *          To write this module author uses helpful informations from:
  *          		- OVC3860 AT Command Application Notes Revision:1.1
  *          		- BLK-MD-SPK-B AT 命令应用指导 pdf
  *          		- https://github.com/tomaskovacik/OVC3860
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


#include "OVC3860_CMDdefinition.h"
#include <assert.h>
#include "OVC3860_device.h"


/**
  * @brief
  * @note
  *
  * @param
  * @param
  * @param
  * @retval
  */







/* ---------------------------------------------------------
 *
 * 							OVC3860
 * 						HADRWARE depend methods
 *
 *
 ---------------------------------------------------------  */

/**
  * @brief	Hardware constructor
  * @note  	You should also remember to set values of:
  * 		- OVC3860_ReceiveBufferType
  * 		- OVC3860_ReceiveBufferSize
  * 		- OVC3860_resetLineHigh
  * 		- OVC3860_resetLineLow
  * 		in OVC3860_device.h file.
  *
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  *               the configuration information for the specified UART module.
  * @param  ResetGPIOx	Pointer to a port where reset pin is soldered
  * @param  GPIO_Pin	pin number of a port where reset pin is soldered
  * @retval n/a
  */
OVC3860HardWare::OVC3860HardWare(UART_HandleTypeDef* huart  /*in DMA mode*/, GPIO_TypeDef* ResetGPIOx, uint16_t GPIO_Pin){
	OVC_huart = huart;
	OVC_ResetGPIOx=ResetGPIOx;
	OVC_Reset_Pin=GPIO_Pin;
	resetLow();
}

/**
  * @brief Object destructor.
  * @note  n/a
  *
  * @param  n/a
  * @retval n/a
  */
OVC3860HardWare::~OVC3860HardWare(){
	resetLow();
	OVC_huart = NULL;
	OVC_Reset_Pin= NULL;
}

/**
  * @brief	Set OVC3860 reset pin high.
  * @note  	You should also remember to set values of:
  * 		- OVC3860_resetLineHigh
  * 		- OVC3860_resetLineLow
  * 		in OVC3860_device.h file.
  *
  * @param  n/a
  * @retval n/a
  */
void OVC3860HardWare::resetHigh(void){
	HAL_GPIO_WritePin(OVC_ResetGPIOx, OVC_Reset_Pin, OVC3860_resetLineHigh);
}

/**
  * @brief	Set OVC3860 reset pin low.
  * @note  	You should also remember to set values of:
  * 		- OVC3860_resetLineHigh
  * 		- OVC3860_resetLineLow
  * 		in OVC3860_device.h file.
  *
  * @param  n/a
  * @retval n/a
  */
void OVC3860HardWare::resetLow(void){
	HAL_GPIO_WritePin(OVC_ResetGPIOx, OVC_Reset_Pin, OVC3860_resetLineLow);
}

/**
  * @brief	Do module reset procedure.
  * @note  	You should also remember to set values of:
  * 		- OVC3860_resetLineHigh
  * 		- OVC3860_resetLineLow
  * 		in OVC3860_device.h file.
  *
  * 		Remember that entering config mode (OVC3860PSKey
  * 		 class) rquires to send appropriate command ASAP
  * 		  after resetHigh();
  *
  * @param  n/a
  * @retval n/a
  */
void OVC3860HardWare::resetModule(void){
	resetLow();
#ifdef INC_FREERTOS_H
	#error  TODO: FreeRTOS OS_delay				//TODO: Delay zależny od tego czy jest HAL czy FreeRTOS
#else
	HAL_Delay(500);
#endif
	resetHigh();
}

/* ---------------------------------------------------------
 *
 * 							OVC3860
 * 						HADRWARE depend methods
 *
 *
 ---------------------------------------------------------  */



/* ---------------------------------------------------------
 *
 * 							OVC3860
 * 						COMMINUCATION module methods
 *
 *
 ---------------------------------------------------------  */

/**
  * @brief	Object constructor
  * @note  	You should also remember to set values of:
  * 		- OVC3860_ReceiveBufferType
  * 		- OVC3860_ReceiveBufferSize
  * 		- OVC3860_resetLineHigh
  * 		- OVC3860_resetLineLow
  * 		in OVC3860_device.h file.
  *
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  *               the configuration information for the specified UART module.
  * @param  ResetGPIOx	Pointer to a port where reset pin is soldered
  * @param  GPIO_Pin	pin number of a port where reset pin is soldered
  * @retval n/a
  */
OVC3860::OVC3860(UART_HandleTypeDef* huart  /*in DMA mode*/, GPIO_TypeDef* ResetGPIOx, uint16_t GPIO_Pin)		//TODO: dodać obsługę pinu reset
		: OVC3860HardWare(huart  /*in DMA mode*/, ResetGPIOx, GPIO_Pin)
{

}

/**
  * @brief Object destructor.
  * @note  Also clean dynamicly created memory.
  *
  * @param  n/a
  * @retval n/a
  */
OVC3860::~OVC3860(void)
{
	destroySendDataArray();
}


/**
  * @brief	search '\r','\n' string in circular buffer.
  * @note	Search for \r\n sequence do not change circular
  * 		 buffer tail head values.
  * 		This function could be also used to detect end
  * 		 of OVC3860 communicate and set the flag to
  * 		 start decodeReceivedString(void);
  *
  * @param
  * @retval	circularBufferSearchResult struct with:
  * 			- bool isFound;
  * 			- size_t tail2virtualTail_;
  * 		for more details take a look at CircularBuffer
  * 		class.
  */
circularBufferSearchResult OVC3860::detectRN(void){
		return SearchItem("\r\n", 2, false);
}

/**
  * @brief	Get data from OVC3860.
  * @note 	Get data from OVC3860 uart and store it in circular
  * 		 buffer.
  * 		 Because OVC3860 uses asynchronous communication, both:
  * 		 - as a answer to command,
  * 		 - and in some cases periodic,
  * 		 this method should be executed ASAP after
  * 		 uart reception i.e. in DMA / interrupt handling
  * 		 function.
  * 		To detect if there are any new data in buffer use
  * 		 detectRN(void) method;
  *
  * @param	n/a
  * @retval	n/a - data is storeg in circular buffer.
  */
void OVC3860::getData(uint8_t RxBuff){
	  if (!isOverflowed())
	  {
		  put(RxBuff);
	  }
}

/**
  * @brief	Decode OVC uart communicates.
  * @note	Beacause OVC3860 uses asynchronous communication, both:
  * 		 - as a answer to command,
  * 		 - and in some cases periodic,
  * 		 this method should be executed as frequent as it is
  * 		  required.
  *
  * 		After command parsing method sets tail positnion after
  * 		 the oldest \r\n sequence.
  *
  * @param	n/a
  * @retval	uint8_t retVal - to be honest I do not know what is
  * 		 it fore but was in source code that I studied.
  * 		 	Maybe it is better to rewrite it to return parsed
  * 		 	 command type (enum STATES).
  */
uint8_t OVC3860::decodeReceivedString(void){

	circularBufferSearchResult bufferState = detectRN();
	OVC3860_reponse parsedCommand = NO_MESSAGE;
	uint8_t retVal = 1;

	if (bufferState.isFound== true)		//OVC3860 receive buf_ (receive buffer) contains "\r\n" sequence which means that OVC386 send compleet message
	{

		uint8_t i=0;

		size_t searchRange = bufferState.tail2virtualTail_ + 2; /* +2 because "\r\n"*/
		OVC3860_ReceiveBufferType	item2search_begginingAddres =0;
		OVC3860_ReceiveBufferType	item2search_lenght;

		const typeOVC3860DecodeStruct* pOVC3860DecodeStruct =&OVC3860DecodeStruct;

		//received command parsing
		do
		{

			item2search_lenght = OVC3860DecodeStruct.decodeStringsLenght[i];
			if (SearchItemTail2Range(pOVC3860DecodeStruct->decodeStrings+item2search_begginingAddres, item2search_lenght,  searchRange , true).isFound)
				{
					parsedCommand = OVC3860DecodeStruct.decodedAnswer[i];
					break;
				};
			i++;
			item2search_begginingAddres =item2search_begginingAddres + item2search_lenght;
		}
		while(i< decodedAnswer_len); //ecodedAnswer item long
		//received command parsing

		switch (parsedCommand)
		{
		case AX_PA: //
			break;
		case AA1: 	//The audio sample rating is set 48000
		    PowerState = On;
		    Audio = ASR_44800;
			break;
		case AA2:	//The audio sample rating is set 44100
			PowerState = On;
			Audio = ASR_44100;
			break;
		case AA4:	//The audio sample rating is set 32000
		    PowerState = On;
		    Audio = ASR_32000;
			break;
		case AA8:	//The audio sample rating is set 16000
		    PowerState = On;
		    Audio = ASR_16000;
			break;
		case AE:	//Audio config error
		    PowerState = On;
		    Audio = ConfigError;
			break;
		case AF:	//Audio codec is closed
		    PowerState = On;
		    Audio = CodecClosed;
			break;
		case AS:	//Audio codec is in phone call mode
		    PowerState = On;
		    Audio = PhoneCall;
			break;
		case EPER:	//Error eeprom parameter
		    PowerState = On;
		    retVal = 0;
			break;
		case ERR:	//The command is error
		    PowerState = On;
		    retVal = 0;
			break;
		case II:	//HSHF enters pairing state indication
		    //DBG(F("II\n"));
		    PowerState = On;
		    BTState = Discoverable;
		    OVC3860::queryA2DPStatus();
			break;
		case IJ2:	//HSHF exits pairing mode and enters listening
		    PowerState = On;
		    BTState = Listening;
			break;
		case IA:	//Disconnected,HSHF state is listening
		    PowerState = On;
		    HFPState = Disconnected;
			break;
		case IC:	//Call-setup status is outgoing
		    PowerState = On;
		    CallState = OngoingCall;
			break;
		case IF:	//Phone hand up,Call-setup status is idle
		    PowerState = On;
		    CallState = PhoneHangUp;
			break;
		case IG:
			PowerState = On;
			break;
		case IL:	//Hold Active Call       Accept Other Call
			PowerState = On;
			CallState = OngoingCall;
			break;
		case IM:	//Make Conference Call
		    PowerState = On;
		    CallState = OngoingCall;
			break;
		case IN:	//Release Held Call       Reject Waiting Call
		    PowerState = On;
		    CallState = PhoneHangUp;
			break;
		case IP:	//IPX Outgoing call number length(X) indication
		    PowerState = On;
		    CallState = OngoingCall;
		    //todo: parsowanie parametrów, bo pełna odpowidź to IP<len>
			break;
		case IR:	//Outgoing call number indication
		    PowerState = On;
		    CallState = OutgoingCall;
		    //todo: parsowanie parametrów, bo pełna odpowidź to IR<phonenum>
			break;
		case IS:	//IS<version>        Power ON Init Complete
		    PowerState = On;
		    //todo: parsowanie parametrów, bo pełna odpowidź to IS<ver>
			break;
		case IT:	//Release Active Call       Accept Other Call
		    PowerState = On;
		    CallState = OngoingCall;
			break;
		case IV:	//Connected
		    PowerState = On;
		    HFPState = Connected;
		    OVC3860::queryA2DPStatus();
			break;
		case MA: //AV pause/stop Indication
		    PowerState = On;
		    MusicState = Idle;
		    CallState = PhoneHangUp;
		    OVC3860::queryA2DPStatus();
			break;
		case MB:	//AV play Indication
		    PowerState = On;
		    MusicState = Playing;
		    CallState = PhoneHangUp;
		    OVC3860::queryA2DPStatus();
			break;
		case MC:	//Indication the voice is on Bluetooth
			PowerState = On;
			break;
		case MD:	//Indication the voice is on phone
			PowerState = On;
			break;
		case ME:	//
			PowerState = On;
			break;
		case MEM_:	//
			PowerState = On;
			//todo: parsowanie parametrów, bo pełna odpowidź to MEM:<val> trzeba to przekazać na zewnątrz w postaci jakiejśc zmiennej/ srtuktury/ etc
			break;
		case MF:	//MFXY: X and Y are auto answer and auto connect configuration
			PowerState = On;
			skipBufferItem(2);
		    //DBG("MF");
		    switch (OVC3860::get()) {
		      case '0':
		        AutoAnswer = Off;
		        //  DBG("AA OFF");
		        break;
		      case '1':
		        AutoAnswer = On;
		        //DBG("AA ON");
		        break;
		    }
		    switch (OVC3860::get()) {
		      case '0':
		        AutoConnect = Off;
		        //DBG("AC OFF");
		        break;
		      case '1':
		        AutoConnect = On;
		        //DBG("AA ON");
		        break;
		    }
			break;
		case MG:	//MGX: The HSHF applications state is X indication       Report Current HFP Status
		    /*
		      HFP Status Value Description:(MG)
		      1. Ready (to be connected)
		      2. Connecting
		      3. Connected
		      4. Outgoing Call
		      5. Incoming Call
		      6. Ongoing Call
		    */
		    PowerState = On;

			skipBufferItem(2);
		    switch (OVC3860::get()) {
		      case '1':
		        HFPState = Ready;
		        break;
		      case '2':
		        HFPState = Connecting;
		        break;
		      case '3':
		        HFPState = Connected;
		        break;
		      case '4':
		        HFPState = OutgoingCall;
		        break;
		      case '5':
		        HFPState = IncomingCall;
		        break;
		      case '6':
		        HFPState = OngoingCall;
		        break;
		    }
			break;
		case ML:	//Report Current AVRCP Status
		    /*
		      AVRCP Status Value Description:(ML)
		      1. Ready (to be connected)
		      2. Connecting
		      3. Connected
		    */
		    PowerState = On;

			skipBufferItem(2);
		    switch (OVC3860::get()) {
		      case '1':
		        AVRCPState = Ready;
		        break;
		      case '2':
		        AVRCPState = Connecting;
		        break;
		      case '3':
		        AVRCPState = Connected;
		        break;
		    }
			break;
		case MM:	//name
			PowerState = On;
			//BT_NAME //todo: parsowanie parametrów, bo pełna odpowidź to MM<name>
			break;
		case MN:	//pin
			PowerState = On;
			//string BT_PIN;			//todo: parsowanie parametrów, bo pełna odpowidź to MN<pin>
			break;
		case MW:	//version
			//todo: parsowanie parametrów, bo pełna odpowidź to MW<version>
			break;
		case MP:	//Music Pause
		    //DBG(F("MP"));
		    PowerState = On;
		    MusicState = Idle;
		    OVC3860::queryA2DPStatus();
			break;
		case MR:	//Music Resume;
		    PowerState = On;
		    MusicState = Playing;
		    OVC3860::queryA2DPStatus();
			break;
		case MS:	//Backward song
		    PowerState = On;
		    MusicState = Rewinding;
			break;
		case MU:	//Report Current A2DP Status
		    /*
		      A2DP Status Value Description:(MU)
		      1. Ready (to be connected)
		      2. Initializing
		      3. Signalling Active
		      4. Connected
		      5. Streaming
		    */
		    PowerState = On;

			skipBufferItem(2);
		    switch (OVC3860::get()) {
		      case '1':
		        A2DPState = Ready;
		        break;
		      case '2':
		        A2DPState = Initializing;
		        break;
		      case '3':
		        A2DPState = SignallingActive;
		        break;
		      case '4':
		        A2DPState = Connected;
		        break;
		      case '5':
		        A2DPState = Streaming;
		        break;
		    }
			break;
		case MX:	//Forward song
		    PowerState = On;
		    MusicState = FastForwarding;
			break;
		case MY:	//AV Disconnect Indication
		    PowerState = On;
		    OVC3860::queryA2DPStatus();
			break;
		case M0:	//
		    PowerState = On;
		    BTState = Disconnected;
			break;
		case M1:	//AV Disconnect Indication
		    PowerState = On;
		    BTState = Connected;
			break;
		case M2:	//AV Disconnect Indication
		    PowerState = On;
		    CallState = IncomingCall;
			break;
		case M3:	//AV Disconnect Indication
		    PowerState = On;
		    CallState = OutgoingCall;
			break;
		case M4:	//AV Disconnect Indication
		    PowerState = On;
		    CallState = OngoingCall;
			break;
		case NOEP:	//No eeprom	break;
		    PowerState = On;
		    retVal = 0;
		case NUM:
			CallState = IncomingCall;
			PowerState = On;
			//CallerID = receivedString.substring(4);		//todo:
			break;
		case OK:
			PowerState = On;
			break;
		case PA:
			PowerState = On;
			//todo: parsowanie parametrów, bo pełna odpowidź to PA1/PA0
			break;
		case PB:
			PowerState = On;
			//TODO: code implementation of this feature in decodeReceivedString
			break;
		case PC:	//?
			PowerState = On;
			break;
		case PE:	//The voice dial start indication
			PowerState = On;
			break;
		case PF:	//The voice dial is not supported/stopped indication
			PowerState = On;
			break;
		case SC:	//SPP opened
		    PowerState = On;
		    BTState = SPPopened;
		   // DBG(F("SPP opened\n"));
			break;
		case SD:	//SPP closed
		    PowerState = On;
		    BTState = SPPclosed;
		    //DBG(F("SPP closed\n"));
			break;
		case SW:  //Command Accepted
			PowerState = On;
			break;
		case VOL: //Command Accepted
		    PowerState = On;
		    //volume = receivedString.substring(3).toInt();		//TODO: PARSOWANIE TEGO bo prawidłowa odpowiedź to VOLXX
			break;
		case WELCOME:	//Welcome message aka power on and reset high
			PowerState = On;
			break;
//		case PSkeyEntry:	//Enter ConfigMode
//			BTState = ConfigMode;
//
//			//todo: wyczyszczenie i kompletne zresetowanie bufora kolowego, dane mają być na samym początku???
//			break;
//		case PSkeyQuit:	//Quit ConfigMode
//			BTState = Disconnected;
//			break;
		case NO_MESSAGE:
		    //DBG(F("Received unknown string:"));		//TODO:
		    //DBG(receivedString);					//TODO:
			break;
		};

		//move tail_ to the end of the parsed command
		 SearchItem("\r\n", 2);
		 skipBufferItem(2);
		 //move tail_ to the end of command
	}
	return retVal;
}

/**
  * @brief	Move tail in circular buffer.
  * @note	Additional method to move tail x position ahead.
  *
  * @param	howMany - number of items to move forward.
  * @retval	n/a
  */
void OVC3860::skipBufferItem(uint8_t howMany){
	for(uint8_t i=0; i< howMany; i++)
	{
		OVC3860::get();
	}
}

/**
  * @brief	Creates array to store data required to store
  * 		 command that will be sent to OVC3860.
  * @note	If new command is bigger that last one it destroys
  * 		 array and create new larger.
  *
  * 		Do not forgot to destroy this array at destructor
  * 		 method.
  *
  * @param	sizeOfArray	- amound of data to stror command
  * @retval	n/s
  */
void OVC3860::createSendDataArray(size_t sizeOfArray) {

	//if data array exists but it is shorter than required destroy if
	if ((sizeOfArray > OVC3860SendDataArrayLenght)
			&& (pOVC3860SendDataArray != 0)) {
		destroySendDataArray();
	}

	//if data array do not exsists create it
	if (pOVC3860SendDataArray == 0) {
		assert(pOVC3860SendDataArray = new uint8_t[sizeOfArray]);
		OVC3860SendDataArrayLenght = sizeOfArray;
	}

}

/**
  * @brief	Destroy array to store data required to store
  * 		 command that will be sent to OVC3860.
  * @note	Do not forgot to destroy this array at destructor
  * 		 method.
  *
  * @param	n/a
  * @retval	n/a
  */
void OVC3860::destroySendDataArray (void){
	delete [] pOVC3860SendDataArray;
	pOVC3860SendDataArray = NULL;
	OVC3860SendDataArrayLenght = 0;
}


/**
  * @brief	Send command to OVC3860
  * @note	Command is stored in dynamic array located with
  * 		 pointer pOVC3860SendDataArray.
  * 		BECAUSE method send data in nonblocking mode (DMA)
  * 		 pointer with data pOVC3860SendDataArray have to
  * 		 exist as long as HAL_UART_TRANSMIT_DMA do not
  * 		 finish its work. That is why this pointer is a part
  * 		 of class / object not a part of this method.
  *
  * @param	pCMD - pointer to command that have to executed, take a look at:
  *   *          		- OVC3860 AT Command Application Notes Revision:1.1
  *          			- BLK-MD-SPK-B AT 命令应用指导 pdf
  * @param	pExtraData - pointer to extra data if pCMD requires this, take a look at:
  *   	        			- OVC3860 AT Command Application Notes Revision:1.1
  *          				- BLK-MD-SPK-B AT 命令应用指导 pdf
  *          			It is overloaded parameter "=0" and can be omitted if
  *          			 pCMD do not reqiures extra data.
  * @param	ExtraDataSize - size of pExtraData data. It is overloaded parameter "=0" and can be omitted if
  *          			 pCMD do not reqiure extra data.
  * @retval	n/a
  */
void OVC3860::sendData(const char* pCMD, const char* pExtraData, size_t ExtraDataSize){

	//create dataArray
	uint16_t arraySize = 3				/*AT#*/
						 +2				/*COMMAND*/
						 +ExtraDataSize	/*pExtraData*/
						 +2;			/*\r\n*/
	createSendDataArray (arraySize);

	memcpy(pOVC3860SendDataArray, "AT#", 3);								//Cpoy "AT#"
	memcpy(pOVC3860SendDataArray+3, pCMD, 2);							//Copy command 'XY'
	if (pExtraData!=0)
		memcpy(pOVC3860SendDataArray+3+2, pExtraData, ExtraDataSize);	//Copy ExtraData
	memcpy(pOVC3860SendDataArray+3+2+ExtraDataSize, "\r\n", 2);			//Copy '\r\n'


	//TODO: po sposobie zainicjowania UART ma dojść do tego którą metodę najlepiej wykorzystać zwykłe wysyłanie, DMA czy IT
	//HAL_UART_Transmit(OVC_huart, pOVC3860SendDataArray, OVC3860SendDataArrayLenght, 200);
	HAL_UART_Transmit_DMA(OVC_huart,pOVC3860SendDataArray, OVC3860SendDataArrayLenght);
	//HAL_UART_Transmit_IT(OVC_huart, pOVC3860SendDataArray, OVC3860SendDataArrayLenght);	//choose manualy which methot You want to use
}


/*
  Pairing
  Set pairing, waiting for the remote device to connect, the command format is:
  AT#CA // discoverable for 2 minutes  - DISCOVERY TIME CAN BE MODIFIED WITH PSKey: app_to_pairing
  ovc3860 returns the indication:
  II // state indication, HSHF enters pairing state indication
  if 2 minutes' timeout is arrived(no peer connect to ovc3860 device), returns the indication:
  IJ2 // state indication, HSHF exits pairing mode and enters listening
  The device can't be found, if need to be search, repeat Pairing operation.

  Enter Pairing Mode #CA

  Command
  #CA
  Current Status(s)
  Any
  Possible Indication(s)
  II
  Indication Description
  Enter Pairing Mode Indication

  Description
  This command puts the module in the pairing mode. The information response and causes will
  indicate the command success or failure. Enter pairing mode indication or failure indication
  will be sent to the host.

  Note:
  1. This command will cause a disconnection if module has already connected with some device.
  2. Module will exit pairing mode if connection not happen in 2 minutes.

  Syntax: AT#CA

  FOR MORE DETAILS:
  Take a look at OVC3860 AT Command Application Notes.pdf
  chapter 4 Command Explanations
*/

void OVC3860::enterPairingMode(void){
	sendData(OVC3860_PAIRING_INIT);
}


/*
  Exit pairing
  Exit pairing, can not be found by peers, the command format is:
  AT#CB // exit pairing mode, non-discoverable
  return:
  IJ2 // state indication, HSHF exits pairing mode and enters listening

  Cancel Pairing Mode #CB

  Command
  #CB
  Current Status(s)
  Pairing
  Possible   Indication(s)
  IJ2
  Indication Description
  Exit Pairing Mode Indication

  Description
  If the module is in pairing mode, this command causes the module to exit the pairing mode and
  enter the idle mode. The information response and causes will indicate the command success or
  failure.

  Syntax: AT#CB

    FOR MORE DETAILS:
  Take a look at OVC3860 AT Command Application Notes.pdf
  chapter 4 Command Explanations
*/
void OVC3860::CancelPairingMode(void){
	sendData(OVC3860_PAIRING_EXIT);
}


/*
  Active connection
  Actively connect to the last successful connected device
  AT#CC // Connect to remote Device
  then input the local PIN code, and ovc3860 returns HSHF's state indication:
  IV
  HSHF state is connected indication
  if Bluetooth mobile phone is turned off or not in Bluetooth signal range, will return a stat
  us indication:
  IJ2 // HSHF exits pairing mode and enters listening state indication

  Connect HFP to Handset #CC
  Command
  #CC
  Current Status(s)
  HFP Status = 1
  Possible Indication(s)
  IV
  Description Indication
  Connecting Indication

  Note: You can get current HFP status by #CY.

  Description
  This command causes the module to connect to a paired handset. The information response and
  causes will indicate the command success or failure. Connect Indication will be sent to the host
  after the connection is established. Otherwise Disconnect Indication will be sent to the host.

  Syntax: AT#CC
*/
void OVC3860::connectHSHF(void){
	sendData(OVC3860_CONNECT_HSHF);
}


/*
  taken from BLK module datasheet
  BLK-MD-SPK-B_AT_Command_set_original( BLK-MD-SPK-B AT Command Application Guide)

  no mention of this command in OVC3860_AT_Command_Application_Notes.pdf

  module responce:

  sending AT#CV
  ERR2
  ERR2
  OK
  OK
*/
void OVC3860::pairingDeleteThenInit() {
  OVC3860::sendData(OVC3860_PAIRING_DELETE_THEN_INIT);
}

/*
  Exit connecting
  Send the command:
  AT#CD // ACL disconnect from remote
  ovc3860 returns the indication:
  IA // HSHF state is listening
  if the remote device disconnects connection actively, ovc3860 also returns the same indication

  Disconnect HFP from Handset #CD
  Command
  #CD
  Current Status(s)
  HFP Status ≥ 3
  Possible Indication(s)
  IA
  Description Indication
  Disconnected Indication

  Description
  This command causes the module to disconnect from the connected handset. The information
  response and causes will indicate the command success or failure. Disconnect Indication will be
  sent to the host after the connection is dropped.

  Syntax: AT#CD
*/
void OVC3860::disconnectHSHF() {
	sendData(OVC3860_DISCONNECT_HSHF);;
}





/*
  user may answer the phone:
  AT#CE
  return the indication:
  IF // hang-up indication
  if the other party hangs up the phone, also return:
  IF // hang-up indication

  Answer Call #CE
  Command
  #CE
  Current Status(s)
  HFP Status = 4
  Possible
  Indication(s)
  IG
  Indication Description
  Pick up Indication

  Description
  This command causes the module to answer an incoming call. The information response and
  causes will indicate the command success or failure.

  Syntax: AT#CE
*/
void OVC3860::callAnswer(void) {
	sendData(OVC3860_CALL_ANSWARE);
}

/*
  Answering the phone
  The phone receives a call, ovc3860 returns the indication, such as:   02167850001
  Receive a incoming call 02167850001
  at this time, user may refuse to answer the phone:
  AT#CF // Refuse to answer the phone
  ovc3860 returns:
  IF // Call-setup status is idle

  Reject Call #CF

  Command
  #CF
  Current Status(s)
  HFP status = 4
  Possible Indication(s)
  IF
  Indication Description
  Hang up Indication

  Description
  This command causes the module to reject an incoming call. The information response and causes
  will indicate the command success or failure.

  Syntax: AT#CF
*/
void OVC3860::callReject() {
	sendData(OVC3860_CALL_REJECT);
}

/*
  End Call
  Command
  #CG
  Current Status(s)
  HFP Status = 5 | 6
  Posible Indication(s)
  IF
  Indication Description
  Hang up Indication

  Description
  This command causes the module to end an active call. The information response and causes will
  indicate the command success or failure.
  Syntax: AT#CG
*/
void OVC3860::callEnd() {
  sendData(OVC3860_CALL_HANGUP);
}


/*
  Redialing
  Redial the last outgoing call:
  AT#CH
  return:
  ICv//Call-setup status is outgoing

  Redial #CH
  Command
  #CH
  Current Status(s)
  HFP Status = 3 | 6
  Possible Indication(s)
  IC
  Indication Description
  Outgoing Call Indication

  Description
  This command causes the module to redial the last number called in the phone. The information
  response and causes will indicate the command success or failure.

  Syntax: AT#CH
*/
void OVC3860::callRedial() {
  sendData(OVC3860_CALL_REDIAL);
}


/*
  Voice dialing
  The command format is:
  AT#CI
  return:
  PE // The voice dial start indication
  or,
  PF // The voice dial is not supported indication

  Voice Dial #CI
  Command
  #CI
  Current Status(s)
  HFP Status = 3
  Possible Indication(s)
  PE
  PF
  Indication Description
  Voice Dial Start Indication
  Handset Not Support Void Dial

  Description
  This command causes the module to active voice dial functionary in the phone. The information
  response and causes will indicate the command success or failure.
  Note: Voice dialing not works in some handset while .

  Syntax: AT#CI
*/
void OVC3860::voiceDialStart() {
	sendData(OVC3860_VOICE_CALL_START);
}


/*
  cancel the voice dialing:
  AT#CJ
  return:
  PF //The voice dial is stopped indication

  Cancel Voice Dial #CJ
  Command
  #CJ
  Current Status(s)
  HFP Status = 3
  Possible Indication(s)
  PF
  Indication Description
  Voice Dial Stop Indication

  Description
  This command causes the module to cancel on going voice dial in the phone. The information
  response and causes will indicate the command success or failure.

  Syntax: AT#CJ
*/
void OVC3860::voiceDialEnd() {
	sendData(OVC3860_VOICE_CALL_CANCEL);
}

/*
  Mute/Unmute MIC #CM

  Command
  #CM
  Current Status(s)
  HFP Status = 6
  Possible Indication(s)
  OK
  Indication Description
  Command Accepted by Module

  Description
  This command causes the module to mute or unmute the MIC. The information response and
  causes will indicate the command success or failure.

  Syntax: AT#CM
*/
void OVC3860::micToggle() {
  sendData(OVC3860_MIC_TOGGLE);
}


/*
  Audio transfer
  Transfer audio between HSHF and phone speaker:
  AT#CO
  when transfered to HSHF, ovc3860 returns:
  MC //The voice is on Bluetooth indication
  when transfered to HSHF:
  MD //The voice is on phone indication

  Transfer Call to/from Handset #CO

  Command
  #CO
  Current Status(s)
  HFP Status = 6(without audio)
  HFP Status = 6(without audio)
  Possible Indication(s)
  MC
  MD
  Indication Description
  HFP Audio Connected
  MD HFP Audio Disconnect

  Description
  This command causes the module to transfer the active call from the module to the handset ( MD
  will received ) or from the handset to the module ( MC will received ). The information response
  and causes will indicate the command success or failure.

  Syntax: AT#CO
*/
void OVC3860::transferAudio() {
  sendData(OVC3860_TRANSFER_AUDIO_TO_SPEAKER);
}



/*
  Release&Reject Call #CQ
  Command
  #CQ
  Current Status(s)
  HFP Status = 6
  Possible Indication(s)
  IN
  Indication Description
  Release Held Call, Reject Waiting Call
  by
  Description
  This command causes the module to release held call, and reject waiting call. The information
  response and causes will indicate the command success or failure.

  Syntax: AT#CQ
*/
void OVC3860::callReleaseReject() {
  sendData(OVC3860_RELEASE_REJECT_CALL);
}


/*
  Release&Accept Call #CR

  Command
  #CR
  Current Status(s)
  HFP Status = 6
  Possible Indication(s)
  IT
  Indication Description
  Release Active Call, Accept Other Call

  Description
  This command causes the module to release active call, accept other call.
  The information response and causes will indicate the command success or failure.

  Syntax: AT#CR
*/
void OVC3860::callReleaseAccept() {
  sendData(OVC3860_RELEASE_ACCEPT_CALL);
}


/*
  Hold&Accept Call #CS

  Command
  #CS
  Current Status(s)
  HFP Status = 6
  Possible Indication(s)
  IM
  Indication Description
  Release Active Call, Accept Other Call

  Description
  This command causes the module to hold active call, accept other call. The information response
  and causes will indicate the command success or failure.

  Syntax: AT#CS
*/
void OVC3860::callHoldAccept() {
  sendData(OVC3860_HOLD_ACCEPT_CALL);
}


/*
  Conference Call #CT

  Command
  #CT
  Current Status(s)
  HFP Status = 6
  Possible Indication(s)
  IM
  Indication Description
  Make Conference Call

  Description
  This command causes the module to make a conference call. The information response and
  causes will indicate the command success or failure.

  Syntax: AT#CT
*/
void OVC3860::callConference() {
  sendData(OVC3860_CONFERENCE_CALL);
}

/*
  Dialing
  For example, dialing 10086, the command format is:
  AT#CW10086
  return
  IC // Call-setup status is outgoing
  IP5 // Outgoing call number length indication
  IR10086 // Outgoing call number indication

  Dial One Call #CW

  Command
  #CW
  Current Status(s)
  HFP Status = 3
  Possible Indication(s)
  IC
  IP<lehgth>
  IR<phone number>
  Outgoing Call Indication
  Length of Phone Number
  Current Call Indication

  Description
  This command causes the module to dial one call. The information response and causes will
  indicate the command success or failure.
  Note: IP, IR indications only supported by HFP1.5 version.


  Syntax: AT#CW13800138000
*/
void OVC3860::callDialNumber(const char* pExtraData, size_t ExtraDataSize) {

  sendData(OVC3860_CALL_DIAL_NUMBER,  pExtraData, ExtraDataSize);
}


/*
  Sending DTMF
  For example, sending number “1”
  AT#CX1
  return:
  OK // send DTMF successfully indication
  NOTE: sopported sending characters (0-9, #, *, A-D).

  Send DTMF #CX

  Command
  #CX
  Current Status(s)
  HFP Status = 6
  Possible Indication(s)
  OK
  Indication Description
  Command Accepted

  Description
  This command causes the module to send one DTMF. The information response and causes will
  indicate the command success or failure.
  Syntax:
  AT#CX1
  AT#CX5

*/
void OVC3860::sendDTMF(const char* pExtraData, size_t ExtraDataSize) {
  OVC3860::sendData(OVC3860_SEND_DTMF,  pExtraData, ExtraDataSize);
}


/*
  Query the HSHF applications state, the command is:
  AT#CY
  ovc3860 returns:
  MGX // The HSHF applications state is X indication
  NOTE: X is the return parameter:
          1 – “Ready”
          2 – “Connecting”
          3 – “Connected”
          4 –“Outgoing call”
          5 –“Incoming call”
          6 – “Ongoing call”.

  Query HFP Status #CY

  Command
  #CY
  Current Status(s)
  Any
  Possible Indication(s)
  MG<code>
  Report Current HFP Status

  Description
  This command queries the module’s HFP current status. The information response and causes will
  indicate the command success or failure.

  Syntax: AT#CY
*/
void OVC3860::queryHFPStatus() {
  sendData(OVC3860_QUERY_HFP_STATUS);
}


/*
  Reset #CZ

  Command
  #CZ
  Current Status(s)
  Any
  Possible
  IS<version>
  MF<a><b>
  Indication Description
  Power ON Init Complete
  Report Auto Answer and PowerOn Auto
  Connection Configuration

  Description
  This command causes the module to reset. The information response and causes will indicate the
  command success or failure.

  Syntax: AT#CZ
*/
void OVC3860::resetSoftware() {
  sendData(OVC3860_RESET);
}


/*
  Play/Pause Music #MA

  Command
  #MA
  Current Status(s)
  A2DP State = 5
  Possible Indication(s)
  MA
  MB
  Indication Description
  AV pause/stop Indication
  AV play Indication

  Description
  If the module is connected with a AV Source, this command causes the AV source to play/pause
  music. If module isn’t connected AV source, this command will cause module try to connected
  current connected mobile’s AV source. The information response and causes will indicate the
  command success or failure.

  Syntax: AT#MA
*/
void OVC3860::musicTogglePlayPause() {
  sendData(OVC3860_MUSIC_TOGGLE_PLAY_PAUSE);
}


/*
  Stop Music #MC

  Command
  #MC
  Current Status(s)
  A2DP State = 5
  Possible Indication(s)
  MA
  Indication Description
  AV pause/stop Indication

  Description
  If the module is connected with a AV Source, this command causes the AV Source to Stop Music.
  The information response and causes will indicate the command success or failure.

  Syntax: AT#MC
*/
void OVC3860::musicStop() {
  sendData(OVC3860_MUSIC_STOP);
}


/*
  Forward Music #MD

  Command
  #MD
  Current Status(s)
  A2DP State = 5
  Possible Indication(s)
  OK
  Indication Description
  Command Accepted by Module

  Description
  If the module is connected with a AV Source, this command causes the AV Source to Play next
  song. The information response and causes will indicate the command success or failure.

  Syntax: AT#MD
*/
void OVC3860::musicNextTrack() {
  sendData(OVC3860_MUSIC_NEXT_TRACK);
}



/*
  Backward Music #ME

  Command
  #ME
  Current Status(s)
  A2DP State = 5
  Possible Indication(s)
  OK
  Indication Description
  Command Accepted by Module

  Description
  If the module is connected with a AV Source, this command causes the AV Source to play last
  song. The information response and causes will indicate the command success or failure.

  Syntax: AT#ME
*/
void OVC3860::musicPreviousTrack() {
  sendData(OVC3860_MUSIC_PREVIOUS_TRACK);
}


/*
  Query Auto Answer and PowerOn Auto Connection Configuration #MF

  Command Current Status(s)
  #MF
  Current Status(s)
  Any
  Possible Indication(s)
  MF<a><b>
  Indication Description
  Report Auto Answer and PowerOn Auto
  Connection Configuration

  Description
  This command queries the module’s auto answer configuration and poweron auto connect
  configuration. The information response and causes will indicate the command success or failure

  Syntax: MF<a><b>
  Value:
  < a >: auto answer configuration, where 0: disable, 1: enabled
  < b >: poweron auto configuration, where 0: disable, 1: enabled
*/
void OVC3860::queryConfiguration() {
 sendData(OVC3860_QUERY_CONFIGURATION);
}


/*`
  Enable PowerOn Auto Connection #MG

  Command
  #MG
  Current Status(s)
  Any
  Possible Indication(s)
  OK
  Indication Description
  Command Accepted

  Description
  This command enables the module to connect to the last used AG after PowerOn. The information
  response and causes will indicate the command success or failure.

  Syntax: AT#MG
*/
void OVC3860::autoconnEnable() {
  sendData(OVC3860_AUTOCONN_ENABLE);
}



/*
  Disable PowerOn Auto Connection #MH

  Command
  #MH
  Current Status(s)
  Any
  Possible Indication(s)
  OK
  Indication Description
  Command Accepted

  Description
  This command disables the module to connect to the Last used AG after PowerOn. The
  information response and causes will indicate the command success or failure.

  Syntax: AT#MH
*/
void OVC3860::autoconnDisable() {
  sendData(OVC3860_AUTOCONN_DISABLE);
}


/*
  Connect to AV Source #MI

  Command
  #MI
  Current Status(s)
  A2DP State = 1
  Possible Indication(s)
  MA
  MB
  Indication Description
  AV pause/stop Indication
  AV play Indication

  Description
  If the module is connected with a HFP phone, this command causes the module try to connect to
  the phone’s AV Source. The information response and causes will indicate the command success
  or failure.
  Note: Music will be played automatic after A2DP connected in some handset.

  Syntax: AT#MI
*/
void OVC3860::connectA2DP() {
  sendData(OVC3860_AV_SOURCE_CONNECT);
}


/*
  Disconnect from AV Source #MJ

  Command
  #MJ
  Current Status(s)
  A2DP Status ≥ 3
  Possible Indication(s)
  MY
  Indication Description
  AV Disconnect Indication

  Description
  This module causes the module to disconnect from the connected phone’s AV source. The
  information response and causes will indicate the command success or failure.

  Syntax: AT#MJ
*/
void OVC3860::disconnectA2DP() {
  OVC3860::sendData(OVC3860_AV_SOURCE_DISCONNECT);
}



/*
  Query AVRCP Status #MO

  Command
  #MO
  Current Status(s)
  Any
  Possible Indication(s)
  ML<code>
  <code> Status
  1 Ready (to be connected)
  2 Connecting
  3 Connected
  Indication Description
  Report Current AVRCP Status

  Description
  This command queries the module’s AVRCP current status. The information response and causes
  will indicate the command success or failure.

  Syntax: AT#MO

  Responce ML<code>:
  <code>
  1 Ready (to be connected)
  2 Connecting
  3 Connected

*/
void OVC3860::queryAvrcpStatus() {
  OVC3860::sendData(OVC3860_QUERY_AVRCP_STATUS);
}



/*
  Enable Auto Answer #MP

  Command
  #MP
  Current Status(s)
  Any
  Possible Indication(s)
  OK
  Indication Description
  Command Accepted

  Description
  This command enables the module auto answer an incoming call. The information response and
  causes will indicate the command success or failure.

  Syntax: AT#MP
*/
void OVC3860::autoAnswerEnable() {
  OVC3860::sendData(OVC3860_AUTO_ANSWER_ENABLE);
}



/*
  Disable Auto Answer #MQ

  Command
  #MQ
  Current Status(s)
  Any
  Possible Indication(s)
  OK
  Indication Description
  Command Accepted

  Description
  This command disables the module auto answer an incoming call. The information response and
  causes will indicate the command success or failure.

  Syntax: AT#MQ
*/
void OVC3860::autoAnswerDisable() {
  OVC3860::sendData(OVC3860_AUTO_ANSWER_DISABLE);
}


/*
  Start Fast Forward #MR

  Command
  #MR
  Current Status(s)
  A2DP Status = 5 OK
  Possible Indication(s)
  OK
  Indication Description
  Command Accepted by Module

  Description
  If the module is connected with a AV Source, this command causes the AV Source to start fast
  forward. The information response and causes will indicate the command success or failure.

  Syntax: AT#MR
*/
void OVC3860::musicStartFastForward() {
  OVC3860::sendData(OVC3860_MUSIC_START_FF);
}



/*
  Start Rewind #MS

  Command
  #MS
  Current Status(s)
  A2DP Status = 5
  Possible Indication(s)
  OK
  Indication Description
  Command Accepted by Module

  Description
  If the module is connected with a AV Source, this command causes the AV Source to start rewind.
  The information response and causes will indicate the command success or failure.

  Syntax: AT#MS
*/
void OVC3860::musicStartRewind() {
  OVC3860::sendData(OVC3860_MUSIC_START_RWD);
}


/*
  Stop Fast Forward / Rewind #MT

  Command
  #MT
  Current Status(s)
  A2DP Status = 5 (after started Fast Forward or Rewind)
  Possible Indication(s)
  OK
  Indication Description
  Command Accepted by Module

  Description
  If the module is connected with a AV Source, this command causes the AV Source to stop fast
  forward or rewind. The information response and causes will indicate the command success or failure.

  Syntax: AT#MT
*/
void OVC3860::musicStopFFRWD() {
  OVC3860::sendData(OVC3860_MUSIC_STOP_FF_RWD);
}


/*
  Query A2DP Status #MV

  Command
  #MV
  Current Status(s)
  Any
  Possible Indication(s)
  MU<code>
  Indication Description
  Report Current A2DP Status

  Description
  This command queries the module’s A2DP current status. The information response and causes
  will indicate the command success or failure.

  Syntax: AT#MV

  Responce:
  MU<code>:1-5, status of A2DP
  1 Ready
  2 Initiating
  3 SignallingActive
  4 Connected
  5 Streaming
*/
void OVC3860::queryA2DPStatus(void){
	sendData(OVC3860_QUERY_A2DP_STATUS);
}



/*
  Write to Memory #MW

  Command
  #MW
  Current Status(s)
  Any
  Possible Indication(s)
  OK
  Indication Description
  Command Accepted

  Description
  This command causes the module to write a byte into a given memory address. The information
  response and causes will indicate the command success or failure.

  Syntax: AT#MWADDR_VAL
  ADDR: a given 32-bit, hexadecimal address
  VAL: a written hexadecimal byte value
*/
void OVC3860::writeToMemory(const char* pExtraData, size_t ExtraDataSize) {
  OVC3860::sendData(OVC3860_WRITE_TO_MEMORY, pExtraData, ExtraDataSize);
}


/*
  Read from Memory #MX

  Command
  #MX
  Current Status(s)
  Any
  Possible Indication(s)
  MEM:<val>
  Indication Description
  The Returned Value


  Description
  This command causes the module to read a byte from a given memory address. The information
  response and causes will indicate the returned value from module reading.

  Syntax: AT#MXADDR
  ADDR: a given 32-bit, hexadecimal address
  <val>: a read hexadecimal byte value
*/
void OVC3860::readFromMemory(const char* pExtraData, size_t ExtraDataSize) {
  OVC3860::sendData(OVC3860_READ_FROM_MEMORY, pExtraData, ExtraDataSize);
}


/*
  Switch Two Remote Devices #MZ

  Command
  #MZ
  Current Status(s)
  Any
  Possible
  Indication(s)
  SW
  Indication Description
  Command Accepted

  Description
  This command causes the module to switch two remote devices. The information response and
  causes will indicate the command success or failure.

  Syntax: AT#MZ
*/
void OVC3860::switchDevices() {
  sendData(OVC3860_SWITCH_DEVICES);
}



/*
  SPP data transmit #ST

  Command
  #ST
  Current Status(s)
  When SPP is connected
  Possible Indication(s)
  OK
  IndicationDescription
  Command Accepted


  Description
  This command will send SPP data to the remote device.

  Syntax: AT#STdata
  data: the string you need to send. The max len is 20.
*/
void OVC3860::sppDataTransmit(const char* pExtraData, size_t ExtraDataSize) {
  OVC3860::sendData(OVC3860_SPP_DATA_TRANSMIT, pExtraData, ExtraDataSize);
}


/*
  Set Clock Debug Mode #VC
  Command
  #VC
  Current Status(s)
  Any
  Possible Indication(s)
  OK
  IndicationDescription
  Command Accepted

  Description
  This command causes the module to enter clock debug mode. The information response and
  causes will indicate the command success or failure.

  Syntax: AT#VC
*/
void OVC3860::setClockdebugMode() {
  sendData(OVC3860_SET_CLOCKDEBUG_MODE);
}


/*
  Speaker Volume Down #VD

  Command
  #VD
  Current Status(s)
  Any
  Possible Indication(s)
  OK
  IndicationDescription
  Command Accepted

  Description
  This command causes the module to decrease the speaker volume. The information response and
  causes will indicate the command success or failure.

  Syntax: AT#VD
*/
void OVC3860::volumeDown() {
  OVC3860::sendData(OVC3860_VOLUME_DOWN);
}


/*
  Enter BQB Test Mode #VE

  Command
  #VE
  Current Status(s)
  Any
  Possible Indication(s)
  OK
  IndicationDescription
  Command Accepted

  Description
  This command causes the module to enter test mode. The information response and causes will
  indicate the command success or failure.

  Syntax: AT#VE
*/
void OVC3860::enterBQBTestMode() {
  sendData(OVC3860_ENTER_TEST_MODE);
}


/*
  Set to Fixed Frequency #VF

  Command
  #VF
  Current Status(s)
  Any
  Possible Indication(s)
  OK
  Indication Description
  Command Accepted

  Description
  This command causes the module to work at 2404MHz. The information response and causes will
  indicate the command success or failure.

  Syntax: AT#VF
*/
void OVC3860::setFixedFrequency() {
  OVC3860::sendData(OVC3860_SET_FIXED_FREQUENCY);
}


/*
  Enter EMC Test Mode #VG

  Command
  #VG
  Current Status(s)
  Any
  Possible Indication(s)
  OK
  Indication Description
  Command Accepted

  Description
  This command causes the module to work at 2404MHz. The information response and causes will
  indicate the command success or failure.

  Syntax: AT#VGxx_yy

  xx: if set to be 0~78, the frequency is fixed at (2402+xx)MHz, If set to 88, the frequency is in
  hopping mode.

  yy: set the tx packet type according to the following table.

  Packet Type Value
  DH1          '04
  DH3          '08
  DH5           10
  2DH1         '05
  2DH3          11
  2DH5          13
  3DH1         '06
  3DH3          12
  3DH5          14
*/
void OVC3860::enterEMCTestMode(const char* pExtraData, size_t ExtraDataSize){
  OVC3860::sendData(OVC3860_EMC_TEST_MODE, pExtraData, ExtraDataSize);
}


/*
  Set RF Register #VH

  Command
  #VH
  Current Status(s)
  Any
  Possible Indication(s)
  OK
  Indication Description
  Command Accepted

  Description
  This command causes the module to set a RF register with a given value. The information
  response and causes will indicate the command success or failure.

  Syntax: AT#VHxx_yy
  xx: a register address
  yy: a byte value
  Example: AT#VH54_88(set RF reg 0x54 to be 0x88)
*/
void OVC3860::setRFRegister(const char* pExtraData, size_t ExtraDataSize){
  OVC3860::sendData(OVC3860_SET_RF_REGISTER, pExtraData, ExtraDataSize);
}



/*
  Start Inquiry #VI

  Command
  #VI
  Current Status(s)
  Any
  Possible Indication(s)
  OK
  Indication Description
  Command Accepted

  Description
  This command causes the module to inquiry Bluetooth devices. The information response and
  causes will indicate the command success or failure.

  Syntax: AT#VI
*/
void OVC3860::inquiryStart() {
  OVC3860::sendData(OVC3860_INQUIRY_START);
}


/*
  Cancel Inquiry #VJ

  Command
  #VJ
  Current Status(s)
  Any
  Possible Indication(s)
  OK
  Indication Description
  Command Accepted

  Description
  This command causes the module to cancel inquiry Bluetooth devices. The information response
  and causes will indicate the command success or failure.

  Syntax: AT#VJ
*/
void OVC3860::inquiryStop() {
  OVC3860::sendData(OVC3860_INQUIRY_STOP);
}


/*
  Speaker Volume Up #VU

  Command
  #VU
  Current Status(s)
  Any
  Possible Indication(s)
  OK
  Indication Description
  Command Accepted

  Description
  This command causes the module to increase the speaker volume. The information response and
  causes will indicate the command success or failure.

  Syntax: AT#VU
*/
void OVC3860::volumeUp() {
  OVC3860::sendData(OVC3860_VOLUME_UP);
}


/*
  Power Off OOL #VX

  Command
  #VX
  Current Status(s)
  Any
  Possible Indication(s)
  OK
  Indication Description
  Command Accepted

  Description
  This command causes the module to power off OOL. The information response and causes will
  indicate the command success or failure.

  Syntax: AT#VX
*/
void OVC3860::shutdown() {
  OVC3860::sendData(OVC3860_SHUTDOWN_MODULE);
}



/*
  Change Local Device Name Casually #MM

  Command
  #MM
  Current Status(s)
  Any
  Parameters
  [new name]
  Possible Indication(s)
  [MM<current name>]
  Indication Description
  Report Current Local Device Name

  Description
  This command causes the module to change the device name. The information response and causes will indicate the command
  success or failure.

  Syntax: AT#MM<new name>

  Value:< new name >: local device name

  ※ If new name is empty, the module will report current local device name.

  example:
  AT#MMMy Car Kit\r\n :the new name is “My Car Kit”
  AT#MM\r\n :indication will be MM<current name>
*/
void OVC3860::changeLocalName(const char* pExtraData, size_t ExtraDataSize) {
  OVC3860::sendData(OVC3860_CHANGE_LOCAL_NAME, pExtraData, ExtraDataSize);
}

/*
  Change Local Device Pin Code #MN

  Command
  #MN
  Current Status(s)
  Any
  Parameters
  [new pin]
  Possible Indication(s)
  [MN<current pin>]
  Indication Description
  Report current local device Pin code

  Description
  This command causes the module to change the device pin code. The information response and causes will indicate the
  command success or failure.

  Syntax: AT#MN<new pin>\r\n

  Value:<new pin >: local device pin (4 digital),

  ※ If new pin is empty, the module will report current local pin code.

  example:
  AT#MN1234 :the new pin is :1234
  AT#MN :indication will be MP<current pin>
*/
void OVC3860::changePin(const char* pExtraData, size_t ExtraDataSize) {
  OVC3860::sendData(OVC3860_CHANGE_PIN, pExtraData, ExtraDataSize);;
}


/* taken from BLK-MD-SPK-B AT Command Application Guide, did not work on module for me (kovo)
    pdf: https://github.com/tomaskovacik/kicad-library/blob/master/library/datasheet/OVC3860_based_modules/BLK-MD-SPK-B_AT_Command_set_original.pdf
  Query Module Software Version #MY

  Command
  #MY
  Current Status(s)
  Any
  Parameters
  MW<version>
  Indication Description
  Report Module Software Version

  Description
  This command queries the module’s software version. The information response and causes will indicate the command
  success or failure.

  Syntax: AT#MY
*/
void OVC3860::queryVersion() {
  OVC3860::sendData(OVC3860_QUERY_VERSION);
}


/*
  Synchronize Phonebook Stored by SIM(via AT Command) #PA

  Command
  #PA
  Current Status(s)
  HFP Status = 3
  Possible Indication(s)
  PA1
  PA0
  Indication Description
  Phonebook Synchronize Indication
  Command Not Supported

  Description
  This command causes the module to synchronize the phonebook which is stored by SIM. The information response and
  causes will indicate the command success or failure.

  Syntax: AT#PA
*/
void OVC3860::pbSyncBySim() {
  OVC3860::sendData(OVC3860_PB_SYNC_BY_SIM);
}


/*
  Read Next One Phonebook Item #PC

  Command
  #PC
  Current Status(s)
  HFP Status = 3
  Possible Indication(s)
  IP<lehgth>
  PB<phonebook>
  PC
  Indication Description
  Length of Phone Number
  One Phonebook Indication
  End of Phonebook/Call History

  Description
  This command causes the module to read next one phonebook item from phone or local. The information response and causes
  will indicate the command success or failure.

  Syntax: AT#PC
*/
void OVC3860::pbReadNextItem() {
  OVC3860::sendData(OVC3860_PB_READ_NEXT_ITEM);
}



/*
  Read Previous One Phonebook Item #PD

  Command
  #PD
  Current Status(s)
  HFP Status = 3
  Possible Indication(s)
  IP<lehgth>
  PB<phonebook>
  PC
  Indication Description
  Length of Phone Number
  One Phonebook Indication
  End of Phonebook/Call History

  Description
  This command causes the module to read previous one phonebook item from phone or local. The information response and
  causes will indicate the command success or failure.

  Syntax: AT#PD
*/
void OVC3860::pbReadPreviousItem() {
  OVC3860::sendData(OVC3860_PB_READ_PREVIOUS_ITEM);
}


/*
  Synchronize Dialed Calls List (via AT Command) #PH

  Command
  #PH
  Current Status(s)
  HFP Status = 3
  Possible Indication(s)
  PA1
  PA0
  Indication Description
  Phonebook Synchronize Indication
  Command Not Supported

  Description
  This command causes the module to synchronize the dialed calls list. The information response and causes will indicate the
  command success or failure.

  Syntax: AT#PH
  */
void OVC3860::pbSyncByDialer() {
  OVC3860::sendData(OVC3860_PB_SYNC_BY_DIALED);
}


/*
  Synchronize Received Calls List (via AT Command) #PI

  Command
  #PI
  Current Status(s)
  HFP Status = 3
  Possible Indication(s)
  PA1
  PA0
  Indication Description
  Phonebook Synchronize Indication
  Command Not Supported

  Description
  This command causes the module to synchronize the received calls list. The information response and causes will indicate the
  command success or failure.

  Syntax: AT#PI
*/
void OVC3860::pbSyncByReceiver() {
  OVC3860::sendData(OVC3860_PB_SYNC_BY_RECEIVED);
}



/*
  Synchronize Missed Calls List (via AT Command) #PJ

  Command
  #PJ
  Current Status(s)
  HFP Status = 3
  Possible Indication(s)
  PA1
  PA0
  Indication Description
  Phonebook Synchronize Indication
  Command Not Supported

  Description
  This command causes the module to synchronize the missed calls list. The information response and causes
  will indicate the command success or failure.

  Syntax: AT#PJ
*/
void OVC3860::pbSyncByMissed() {
  OVC3860::sendData(OVC3860_PB_SYNC_BY_MISSED);
}

/*
  Synchronize Last Call List (via AT Command) #PK

  Command
  #PK
  Current Status(s)
  HFP Status = 3
  Possible Indication(s)
  PA1
  PA0
  Indication Description
  Phonebook Synchronize Indication
  Command Not Supported

  Description
  This command causes the module to synchronize the last call list. The information response and causes will indicate the
  command success or failure.

  Syntax: AT#PK
*/
void OVC3860::pbSyncByLastCall() {
	OVC3860::sendData(OVC3860_PB_SYNC_BY_LAST_CALL);
}

/*
  Get Recent Dialed Call History (record by module) #PL

  Command
  #PL
  Current Status(s)
  Any
  Possible Indication(s)
  IP<length>
  MN<phone number>
  PC
  Indication Description
  Length of Phone Number Indication
  One Phonebook Indication
  End of Phonbook/Call History Download Ind

  Description
  This command causes the module to read one of recently dialed call number(record by module). The information response and
  causes will indicate the command success or failure.

  Syntax: AT#PL
*/
void OVC3860::getLocalLastDialedList() {
  OVC3860::sendData(OVC3860_GET_LOCAL_LAST_DIALED_LIST);
}



/*
  Get Recent Received Call History (record by module) #PM

  Command
  #PM
  Current Status(s)
  Any
  Possible Indication(s)
  IP<length>
  MN<phone number>
  PC
  Indication Description
  Length of Phone Number Indication
  One Phonebook Indication
  End of Phonbook/Call History Download Ind

  Description
  This command causes the module to read one of recently received call number(record by module). The information response
  and causes will indicate the command success or failure.

  Syntax: AT#PM
*/
void OVC3860::getLocalLastReceivedList() {
  OVC3860::sendData(OVC3860_GET_LOCAL_LAST_RECEIVED_LIST);
}


/*
  Get Recent Missed Call History (record by module) #PN

  Command
  #PN
  Current Status(s)
  Any
  Possible Indication(s)
  IP<length>
  MN<phone number>
  PC
  Indication Description
  Length of Phone Numbervoid print Indication
  One Phonebook Indication
  End of Phonbook/Call History Download Ind

  Description
  This command causes the module to read one of recently missed call number(record by module). The information response
  and causes will indicate the command success or failure.

  Syntax: AT#PN
*/
void OVC3860::getLocalLastMissedList() {
  OVC3860::sendData(OVC3860_GET_LOCAL_LAST_MISSED_LIST);
}



/*
  Dial Last Received Phone Number #PO

  Command
  #PO
  Current Status(s)
  HFP Status = 3 | 6
  Possible Indication(s)
  IC
  IP<length>
  IR<phone number>
  Indication Description
  Outgoing Call Indication
  Length of Phone Number Indication
  Current Call Indication

  Description
  This command causes the module to dial last received phone number. The information response and causes will indicate the
  command success or failure.

  Note: IP,IR indication only supported by HFP1.5 version.

  Syntax: AT#PO
*/
void OVC3860::dialLastReceivedCall() {
  OVC3860::sendData(OVC3860_DIAL_LAST_RECEIVED_CALL);
}



/*
  Clear Call History (record by module) #PR

  Command
  Current Status(s)
  Possible Indication(s) Indication Description
  #PO
  Any
  OK
  Command Accepted by Module

  Description
  This command causes the module to clear call history(record by module). The information response and causes will indicate
  the command success or failure.

  Syntax: AT#PR
*/
void OVC3860::clearLocalCallHistory() {
  OVC3860::sendData(OVC3860_CLEAR_LOCAL_CALL_HISTORY);
}


/* ---------------------------------------------------------
 *
 * 							OVC3860
 * 						COMMINUCATION module methods
 *
 *
 ---------------------------------------------------------  */
