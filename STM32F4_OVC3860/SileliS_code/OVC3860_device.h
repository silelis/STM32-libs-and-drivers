/**
  ******************************************************************************
  * @file    OVC3860_device.h
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

#ifndef OVC3860_DEVICE_H_
#define OVC3860_DEVICE_H_


#include "CircularBuffer.h"			//You can find this code on my Github
#include "stm32f4xx_hal.h"
//#include <string>

#define	OVC3860_ReceiveBufferType	uint8_t				//defines type of data that are received from OVC6860 chip
#define	OVC3860_ReceiveBufferSize	65					//defines length of circular buffer to capture data from OVC, min. length is determined by datasheet max. val is mcu depend
#define	OVC3860_resetLineHigh		GPIO_PIN_RESET		//dfines if OVC reser high is high/ low state of mcu pin. Depend on Your hardware design
#define	OVC3860_resetLineLow		GPIO_PIN_SET		//dfines if OVC reser high is high/ low state of mcu pin. Depend on Your hardware design


/*
 * OVC3860HardWare  is class to manage hardware.
 * This is parrent class for  OVC3860 and OVC3860PSKeys.
 *
 */
class OVC3860HardWare{
public:
	OVC3860HardWare(UART_HandleTypeDef* huart  /*in DMA mode*/, GPIO_TypeDef* ResetGPIOx, uint16_t GPIO_Pin);
	~OVC3860HardWare();

	void 		resetModule(void);							//Hardware module reset
	void 		resetHigh();								//Hardware module start
	void 		resetLow();									//Hardware module stop


protected:
	UART_HandleTypeDef* OVC_huart;
	GPIO_TypeDef* 		OVC_ResetGPIOx;
	uint16_t 			OVC_Reset_Pin;
};

/*
 * OVC3860 is class to manage audio / phone for OVC3860 chip
 * The parrent class is  OVC3860HardWare which sets hardware
 *  issues of OVC3860 i.e. uart, reset line port and pin.
 *
 *  Please take under consideration that this class uses DMA
 *  mechanism to contact with chip.
 */
class OVC3860: protected CircularBuffer<OVC3860_ReceiveBufferType, OVC3860_ReceiveBufferSize>,
			   public OVC3860HardWare
{

public:
	OVC3860(UART_HandleTypeDef* huart, GPIO_TypeDef* ResetGPIOx, uint16_t GPIO_Pin);
	~OVC3860(void);

	//enumerate for all avaliable on the chip states
	enum STATES
	{
		/*
		from datasheet:
		HFP Status Value Description:(MG) => uint8_t HFPState
		1. Ready (to be connected)
		2. Connecting
		3. Connected
		4. Outgoing Call
		5. Incoming Call
		6. Ongoing Call
		A2DP Status Value Description:(MU) => uint8_t A2DPState
		1. Ready (to be connected)
		2. Initializing
		3. Signalling Active
		4. Connected
		5. Streaming
		AVRCP Status Value Description:(ML) => uint8_t AVRCPState
		1. Ready (to be connected)
		2. Connecting
		3. Connected
		 */

	  Ready, //HFP,A2DP.AVRCP
	  Connecting, //HFP.AVRCP
	  Connected, //HFP,A2DP.AVRCP
	  OutgoingCall,//HFP
	  IncomingCall,//HFP
	  OngoingCall,//HFP
	  Initializing,//A2DP
	  SignallingActive,//A2DP
	  Streaming,//A2DP
	  Playing, //Music
	  Idle, //Music = paused/stoped, call =idle;
	  On, //PowerState
	  Off, //PowerState
	  Pairing, //BT
	  ShutdownInProgress,//PowerState
	  FastForwarding, //Music
	  Rewinding, //Music
	  ConfigError, //Audio
	  CodecClosed,//Audio
	  ASR_44800,//Audio
	  ASR_44100,//Audio
	  ASR_32000,//Audio
	  ASR_16000,//Audio
	  PhoneCall,
	  Listening,
	  PhoneHangUp,
	  SPPopened,
	  SPPclosed,
	  Discoverable,
	  Disconnected,
	  ConfigMode
	};

	STATES BTState = Disconnected;
	STATES HFPState = Disconnected;
	STATES A2DPState = Disconnected;
	STATES AVRCPState = Disconnected;
	STATES CallState = Disconnected;
	STATES MusicState = Idle;
	STATES PowerState = Off;
	STATES Audio;
	STATES AutoAnswer=Off;
	STATES AutoConnect=Off;

	//uint8_t volume;			//TODO: code implementation of this feature in decodeReceivedString VOL:
	//string CallerID;			//TODO: code implementation of this feature in decodeReceivedString NUM:
	//uint8_t BT_ADDR[6];		//TODO: read thic in PSkey mode
	//string BT_NAME;			//TODO: code implementation of this feature in decodeReceivedString MM:
	//string BT_PIN;			//TODO: code implementation of this feature in decodeReceivedString MN:

	//AT COMMANDS
	//OVC3860 AT Command Application Notes.pfd (version 1.1 2012-10-18 - English)
	void enterPairingMode(void);
	void CancelPairingMode(void);
	void connectHSHF(void);
	void disconnectHSHF(void);
	void callAnswer(void);
	void callReject(void);
	void callEnd(void);
	void callRedial(void);
	void voiceDialStart();
	void voiceDialEnd();
	void micToggle();
	void transferAudio();
	void callReleaseReject();
	void callReleaseAccept();
	void callHoldAccept();
	void callConference();
	void callDialNumber(const char* pExtraData, size_t ExtraDataSize);		//Extradata is 13800138000 (number)
	void sendDTMF(const char* pExtraData, size_t ExtraDataSize=1);			//ExtraData is 1 or 5
	void queryHFPStatus();
	void resetSoftware();
	void musicTogglePlayPause();
	void musicStop();
	void musicNextTrack();
	void musicPreviousTrack();
	void queryConfiguration();
	void autoconnEnable();
	void autoconnDisable();
	void connectA2DP();
	void disconnectA2DP();
	void queryAvrcpStatus();
	void autoAnswerEnable();
	void autoAnswerDisable();
	void musicStartFastForward();
	void musicStartRewind();
	void musicStopFFRWD();
	void queryA2DPStatus();
	void writeToMemory(const char* pExtraData, size_t ExtraDataSize);	//ExtraData should be givea as: ADDR_VAL
	void readFromMemory(const char* pExtraData, size_t ExtraDataSize);	//ExtraData is ADDR: a given 32-bit, hexadecimal address so: "00000179" <--len 8
	void switchDevices();
	void sppDataTransmit(const char* pExtraData, size_t ExtraDataSize);	//ExtraData is the string you need to send. The max len is 20.
	void setClockdebugMode();
	void volumeDown();
	void enterBQBTestMode();
	void setFixedFrequency();
	void enterEMCTestMode(const char* pExtraData, size_t ExtraDataSize=5); //Extra data is xx_yy with "_"
																		 //xx: if set to be 0~78, the frequency is fixed at (2402+xx)MHz,
																		 //					If set to 88, the frequency is in hopping mode.
																		 //yy: set the tx packet type according to the following table.
	void setRFRegister(const char* pExtraData, size_t ExtraDataSize=5);	 //Extra data is xx_yy xx: a register address
																		 //			yy: a byte value Example: AT#VH54_88(set RF reg 0x54 to be 0x88)
	void inquiryStart();
	void inquiryStop();
	void volumeUp();
	void shutdown();
	//AT COMMANDS




	//AT COMMANDS from chiness documentation
	//BLK-MD-SPK-B AT.pfd (version ?? 2011-03 - Chiness)
	//I am not sure if those command works with all moduls in my case had not been working
	void pairingDeleteThenInit();
	void changeLocalName(const char* pExtraData, size_t ExtraDataSize);		//without parameter module should return actual name, did not work for me		//TODO: sprawdzić jak to działa
	void changePin(const char* pExtraData, size_t ExtraDataSize=4); 			//without parameter module should return actual pin, did not work for me				//TODO: sprawdzić jak to działa
	void queryVersion();
	void pbSyncBySim();
	void pbSyncByPhone();
	void pbReadNextItem();
	void pbReadPreviousItem();
	void pbSyncByDialer();
	void pbSyncByReceiver();
	void pbSyncByMissed();
	void pbSyncByLastCall();
	void getLocalLastDialedList();
	void getLocalLastReceivedList();
	void getLocalLastMissedList();
	void dialLastReceivedCall();
	void clearLocalCallHistory();
	//AT COMMANDS from chiness documentation

	void		getData(uint8_t RxBuff);							//get data from OVC and put it to circular buffer
	uint8_t 	decodeReceivedString(void);
	circularBufferSearchResult	detectRN(void);						//detect if received message has '\r','\n' sequence which means end of message.

protected:

private:
	uint8_t* 			pOVC3860SendDataArray = 0;		//HAL_UART_DMA and HAL_UART_IT data arrays have to be GLOBAL, that is the reason why this pointer is declared
														//it is used in sendData method
	size_t 				OVC3860SendDataArrayLenght = 0;
	void createSendDataArray (size_t sizeOfArray);
	void destroySendDataArray (void);

	void skipBufferItem(uint8_t howMany);
	void sendData(const char* pCMD, const char* pExtraData=0, size_t ExtraDataSize=0);		//most commands requires


	//this enum contains switch() function cases that are used in decodeReceivedString(void) method
	enum OVC3860_reponse
		{
			AX_PA,	//
			AA1,	//The audio sample rating is set 48000
			AA2,	//The audio sample rating is set 44100
			AA4,	//The audio sample rating is set 32000
			AA8,	//The audio sample rating is set 16000
			AE,		//Audio config error
			AF,		//Audio codec is closed
			AS,		//Audio codec is in phone call mode
			EPER,	//Error eeprom parameter
			ERR,	//The command is error
			II,		//HSHF enters pairing state indication
			IJ2,	//HSHF exits pairing mode and enters listening
			IA,		//Disconnected,HSHF state is listening
			IC,		//Call-setup status is outgoing
			IF,		//Phone hand up,Call-setup status is idle
			IG,		//
	  	  	IL,		//Hold Active Call       Accept Other Call
			IM,		//Make Conference Call
			IN,		//Release Held Call       Reject Waiting Call
			IP,		//IPX Outgoing call number length(X) indication
			IR,		//Outgoing call number indication
			IS,		//IS<version>        Power ON Init Complete
			IT,		//Release Active Call       Accept Other Call
			IV,		//Connected
			MA, 	//AV pause/stop Indication
			MB,		//AV play Indication
			MC,		//Indication the voice is on Bluetooth
			MD,		//Indication the voice is on phone
			ME,		//
	  	    MEM_,	//
	  	  	MF,		//MFXY: X and Y are auto answer and auto connect configuration
			MG,		//MGX: The HSHF applications state is X indication       Report Current HFP Status
			ML,		//Report Current AVRCP Status
			MM,		//name
			MN,		//pin
			MW,		//version
			MP,		//Music Pause
			MR,		//Music Resume;
			MS,		//Backward song
			MU,		//Report Current A2DP Status
			MX,		//Forward song
			MY,		//AV Disconnect Indication
			M0,		//
			M1,		//AV Disconnect Indication
			M2,		//AV Disconnect Indication
			M3,		//AV Disconnect Indication
			M4,		//AV Disconnect Indication
			NOEP,	//No eeprom
	  	  	NUM,
	  	  	OK,
	  	  	PA,
	  	  	PB,
	  	  	PC,		//?
			PE,		//The voice dial start indication
			PF,		//The voice dial is not supported/stopped indication
			SC,		//SPP opened
			SD,		//SPP closed
			SW,  	//Command Accepted
			VOL, 	//Command Accepted
			WELCOME,
//			PSkeyEntry,
//			PSkeyQuit,
			NO_MESSAGE
		};

	#define decodedAnswer_len			61	//63

	//this is parsing data table which is used by decodeReceivedString(void)
	struct typeOVC3860DecodeStruct{

		//this array contains all searched words used by decodeReceivedString(void)
		//DO NOT CHANDE ORDER of the lines
		OVC3860_ReceiveBufferType decodeStrings [143  /*153*/] = {
			0x04, 0x0F, 0x04, 0x00, 0x01, 0x00, 0x00, 	//7	//WELCOME
			//0x04, 0x0F, 0x04, 0x01, 0x01, 0x00, 0x00, 	//7	//PSkeyEntry
			'A','X','_','P','A',	//5
			//0x60, 0x00, 0x00, 0x00,						//4	//PSkeyQuit
			'E','P','E','R',		//4		//Error eeprom parameter
			'M','E','M',':',		//4
			'N','O','E','P',		//4		//No eeprom
			'V','O','L',			//3		//Command Accepted
			'A','A','1',			//3     //The audio sample rating is set 48000
			'A','A','2',			//3		//The audio sample rating is set 44100
			'A','A','4',			//3		//The audio sample rating is set 32000
			'A','A','8',			//3		//The audio sample rating is set 16000
			'E','R','R',			//3		//The command is error
			'I','J','2',			//3		//HSHF exits pairing mode and enters listening
			'N','U','M',			//3
			'A','E',				//2		//Audio config error
			'A','F',				//2		//Audio codec is closed
			'A','S',				//2		//Audio codec is in phone call mode
			'I','I',				//2		//HSHF enters pairing state indication
			'I','A',				//2		//Disconnected,HSHF state is listening
			'I','C',				//2		//Call-setup status is outgoing
			'I','F',				//2		//Phone hand up,Call-setup status is idle
			'I','G',				//2
			'I','L',				//2    	//Hold Active Call       Accept Other Call
			'I','M',				//2		//Make Conference Call
			'I','N',				//2		//Release Held Call       Reject Waiting Call
			'I','P',				//2		//IPX Outgoing call number length(X) indication
			'I','R',				//2		//Outgoing call number indication
			'I','S',				//2		//IS<version>        Power ON Init Complete
			'I','T',				//2		//Release Active Call       Accept Other Call
			'I','V',				//2		//Connected
			'M','A',				//2		//AV pause/stop Indication
			'M','B',				//2		//AV play Indication
			'M','C',				//2		//Indication the voice is on Bluetooth
			'M','D',				//2		//Indication the voice is on phone
			'M','E',				//2
			'M','F',				//2		//MFXY: X and Y are auto answer and auto connect configuration
			'M','G',				//2		//MGX: The HSHF applications state is X indication       Report Current HFP Status
			'M','L',				//2		//Report Current AVRCP Status
			'M','M',				//2		//name
			'M','N',				//2		//pin
			'M','W',				//2		//version
			'M','P',				//2		//Music Pause
			'M','R',				//2		//Music Resume;
			'M','S',				//2		//Backward song
			'M','U',				//2		//Report Current A2DP Status
			'M','X',				//2		//Forward song
			'M','Y',				//2		//AV Disconnect Indication
			'M','0',				//2		//
			'M','1',				//2		//AV Disconnect Indication
			'M','2',				//2		//AV Disconnect Indication
			'M','3',				//2		//AV Disconnect Indication
			'M','4',				//2		//AV Disconnect Indication
			'O','K',				//2
			'P','A',				//2
			'P','B',				//2
			'P','C',				//2
			'P','E',				//2		//The voice dial start indication
			'P','F',				//2		//The voice dial is not supported/stopped indication
			'S','C',				//2		//SPP opened
			'S','D',				//2		//SPP closed
			'S','W'				//2		//Command Accepted

		};

		//this array contains all searched words length used by decodeReceivedString(void)
		//DO NOT CHANDE ORDER of the lines
		uint8_t decodeStringsLenght [decodedAnswer_len-1] ={
				7,		//0x04, 0x0F, 0x04, 0x00, 0x01, 0x00, 0x00,	//WELCOME
				//7,		//0x04, 0x0F, 0x04, 0x01, 0x01, 0x00, 0x00, 	//PSkeyEntry
				5,		//AX_PA
				//4,		//0x60, 0x00, 0x00, 0x00			//PSkeyQuit
				4,		//EPER	//Error eeprom parameter
				4,		//MEM:
				4,		//NOEP	//No eeprom
				3,		//VOL	//Command Accepted
				3,		//AA1	//The audio sample rating is set 48000
				3,		//AA2	//The audio sample rating is set 44100
				3,		//AA4	//The audio sample rating is set 32000
				3,		//AA8	//The audio sample rating is set 16000
				3,		//ERR	//The command is error
				3,		//IJ2	//HSHF exits pairing mode and enters listening
				3,		//NUM
				2,		//AE	//Audio config error
				2,		//AF	//Audio codec is closed
				2,		//AS	//Audio codec is in phone call mode
				2,		//II	//HSHF enters pairing state indication
				2,		//IA	//Disconnected,HSHF state is listening
				2,		//IC	//Call-setup status is outgoing
				2,		//IF	//Phone hand up,Call-setup status is idle
				2,		//IG
				2,		//IL	//Hold Active Call       Accept Other Call
				2,		//IM	//Make Conference Call
				2,		//IN	//Release Held Call       Reject Waiting Call
				2,		//IP	//IPX Outgoing call number length(X) indication
				2,		//IR	//Outgoing call number indication
				2,		//IS	//IS<version>        Power ON Init Complete
				2,		//IT	//Release Active Call       Accept Other Call
				2,		//IV	//Connected
				2,		//MA	//AV pause/stop Indication
				2,		//MB	//AV play Indication
				2,		//MC	//Indication the voice is on Bluetooth
				2,		//MD	//Indication the voice is on phone
				2,		//ME
				2,		//MF	//MFXY: X and Y are auto answer and auto connect configuration
				2,		//MG	//MGX: The HSHF applications state is X indication       Report Current HFP Status
				2,		//ML	//Report Current AVRCP Status
				2,		//MM	//name
				2,		//MN	//pin
				2,		//MW	//version
				2,		//MP	//Music Pause
				2,		//MR	//Music Resume;
				2,		//MS	//Backward song
				2,		//MU	//Report Current A2DP Status
				2,		//MX	//Forward song
				2,		//MY	//AV Disconnect Indication
				2,		//M0	//
				2,		//M1	//AV Disconnect Indication
				2,		//M2	//AV Disconnect Indication
				2,		//M3	//AV Disconnect Indication
				2,		//M4	//AV Disconnect Indication
				2,		//OK
				2,		//PA
				2,		//PB
				2,		//PC
				2,		//PE	//The voice dial start indication
				2,		//PF	//The voice dial is not supported/stopped indication
				2,		//SC	//SPP opened
				2,		//SD	//SPP closed
				2		//SW	//Command Accepted
				//0		//NONE
		};

		//this array contains all searched words enum values (enum OVC3860_reponse) used by decodeReceivedString(void)
		//DO NOT CHANDE ORDER of the lines
		OVC3860_reponse decodedAnswer [decodedAnswer_len] = {		//is +1 than "decodeStringsLenght" ponieważ posiada jeszcze odpowiedź NONE
				WELCOME,		//7,		//0x04, 0x0F, 0x04, 0x00, 0x01, 0x00, 0x00,	//WELCOME
				//PSkeyEntry,	//7,		//0x04, 0x0F, 0x04, 0x01, 0x01, 0x00, 0x00, 	//PSkeyEntry
				AX_PA,			//5,		//AX_PA
				//PSkeyQuit,	//4,		//0x60, 0x00, 0x00, 0x00			//PSkeyQuit
				EPER,		//4,		//EPER	//Error eeprom parameter
				MEM_,		//4,		//MEM:
				NOEP,		//4,		//NOEP	//No eeprom
				VOL,		//3,		//VOL	//Command Accepted
				AA1,		//3,		//AA1	//The audio sample rating is set 48000
				AA2,		//3,		//AA2	//The audio sample rating is set 44100
				AA4,		//3,		//AA4	//The audio sample rating is set 32000
				AA8,		//3,		//AA8	//The audio sample rating is set 16000
				ERR,		//3,		//ERR	//The command is error
				IJ2,		//3,		//IJ2	//HSHF exits pairing mode and enters listening
				NUM,		//3,		//NUM
				AE,			//2,		//AE	//Audio config error
				AF,			//2,		//AF	//Audio codec is closed
				AS,			//2,		//AS	//Audio codec is in phone call mode
				II,			//2,		//II	//HSHF enters pairing state indication
				IA,			//2,		//IA	//Disconnected,HSHF state is listening
				IC,			//2,		//IC	//Call-setup status is outgoing
				IF,			//2,		//IF	//Phone hand up,Call-setup status is idle
				IG,			//2,		//IG
				IL,			//2,		//IL	//Hold Active Call       Accept Other Call
				IM,			//2,		//IM	//Make Conference Call
				IN,			//2,		//IN	//Release Held Call       Reject Waiting Call
				IP,			//2,		//IP	//IPX Outgoing call number length(X) indication
				IR,			//2,		//IR	//Outgoing call number indication
				IS,			//2,		//IS	//IS<version>        Power ON Init Complete
				IT,			//2,		//IT	//Release Active Call       Accept Other Call
				IV,			//2,		//IV	//Connected
				MA,			//2,		//MA	//AV pause/stop Indication
				MB,			//2,		//MB	//AV play Indication
				MC,			//2,		//MC	//Indication the voice is on Bluetooth
				MD,			//2,		//MD	//Indication the voice is on phone
				ME,			//2,		//ME
				MF,			//2,		//MF	//MFXY: X and Y are auto answer and auto connect configuration
				MG,			//2,		//MG	//MGX: The HSHF applications state is X indication       Report Current HFP Status
				ML,			//2,		//ML	//Report Current AVRCP Status
				MM,			//2,		//MM	//name
				MN,			//2,		//MN	//pin
				MW,			//2,		//MW	//version
				MP,			//2,		//MP	//Music Pause
				MR,			//2,		//MR	//Music Resume;
				MS,			//2,		//MS	//Backward song
				MU,			//2,		//MU	//Report Current A2DP Status
				MX,			//2,		//MX	//Forward song
				MY,			//2,		//MY	//AV Disconnect Indication
				M0,			//2,		//M0	//
				M1,			//2,		//M1	//AV Disconnect Indication
				M2,			//2,		//M2	//AV Disconnect Indication
				M3,			//2,		//M3	//AV Disconnect Indication
				M4,			//2,		//M4	//AV Disconnect Indication
				OK,			//2,		//OK
				PA,			//2,		//PA
				PB,			//2,		//PB
				PC,			//2,		//PC
				PE,			//2,		//PE	//The voice dial start indication
				PF,			//2,		//PF	//The voice dial is not supported/stopped indication
				SC,			//2,		//SC	//SPP opened
				SD,			//2,			//SPP closed
				SW,			//2		//Command Accepted
				NO_MESSAGE	//0		//NONE
		};
	} const OVC3860DecodeStruct;
};

#endif /* OVC3860_DEVICE_H_ */
