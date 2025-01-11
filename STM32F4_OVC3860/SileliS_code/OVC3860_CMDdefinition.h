/**
  ******************************************************************************
  * @file    OVC3860_CMDdefinition.h
  * @author  Dawid "SileliS" Bańkowski	d.bankowski(at)gmail.com
  * @brief   OVC3860 AT command definitions.
  *          This file provides code to manage OVC3860 PSKeys.
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

#ifndef OVC3860_CMDDEFINITION_H_
#define OVC3860_CMDDEFINITION_H_


//cmds definitions:
#define OVC3860_PAIRING_INIT "CA" /* discoverable for 2 minutes */
#define OVC3860_PAIRING_EXIT "CB"
#define OVC3860_CONNECT_HSHF "CC"
#define OVC3860_DISCONNECT_HSHF "CD"
#define OVC3860_CALL_ANSWARE "CE"
#define OVC3860_CALL_REJECT "CF"
#define OVC3860_CALL_HANGUP "CG"
#define OVC3860_CALL_REDIAL "CH"
#define OVC3860_VOICE_CALL_START "CI"
#define OVC3860_VOICE_CALL_CANCEL "CJ"
#define OVC3860_MIC_TOGGLE "CM"
#define OVC3860_TRANSFER_AUDIO_TO_SPEAKER "CO" // to secod BT device???
#define OVC3860_RELEASE_REJECT_CALL "CQ"
#define OVC3860_RELEASE_ACCEPT_CALL "CR"
#define OVC3860_HOLD_ACCEPT_CALL "CS"
#define OVC3860_CONFERENCE_CALL "CT"
#define OVC3860_PAIRING_DELETE_THEN_INIT "CV"
#define OVC3860_CALL_DIAL_NUMBER "CW"
#define OVC3860_SEND_DTMF "CX"
#define OVC3860_QUERY_HFP_STATUS "CY"
#define OVC3860_RESET "CZ"
#define OVC3860_MUSIC_TOGGLE_PLAY_PAUSE "MA"
#define OVC3860_MUSIC_STOP "MC"
#define OVC3860_MUSIC_NEXT_TRACK "MD"
#define OVC3860_MUSIC_PREVIOUS_TRACK "ME"
#define OVC3860_QUERY_CONFIGURATION "MF"
#define OVC3860_AUTOCONN_ENABLE "MG"
#define OVC3860_AUTOCONN_DISABLE "MH"
#define OVC3860_AV_SOURCE_CONNECT "MI"
#define OVC3860_AV_SOURCE_DISCONNECT "MJ"
#define OVC3860_CHANGE_LOCAL_NAME "MM"
#define OVC3860_CHANGE_PIN "MN"
#define OVC3860_QUERY_AVRCP_STATUS "MO"
#define OVC3860_AUTO_ANSWER_ENABLE "MP"
#define OVC3860_AUTO_ANSWER_DISABLE "MQ"
#define OVC3860_MUSIC_START_FF "MR"
#define OVC3860_MUSIC_START_RWD "MS"
#define OVC3860_MUSIC_STOP_FF_RWD "MT"
#define OVC3860_QUERY_A2DP_STATUS "MV"
#define OVC3860_WRITE_TO_MEMORY "MW"
#define OVC3860_READ_FROM_MEMORY "MX"
#define OVC3860_QUERY_VERSION "MY"
#define OVC3860_SWITCH_DEVICES "MZ"
#define OVC3860_PB_SYNC_BY_SIM "PA"
#define OVC3860_PB_SYNC_BY_PHONE "PB"
#define OVC3860_PB_READ_NEXT_ITEM "PC"
#define OVC3860_PB_READ_PREVIOUS_ITEM "PD"
#define OVC3860_PB_SYNC_BY_DIALED "PH"
#define OVC3860_PB_SYNC_BY_RECEIVED "PI"
#define OVC3860_PB_SYNC_BY_MISSED "PJ"
#define OVC3860_PB_SYNC_BY_LAST_CALL "PK"
#define OVC3860_GET_LOCAL_LAST_DIALED_LIST "PL"
#define OVC3860_GET_LOCAL_LAST_RECEIVED_LIST "PM"
#define OVC3860_GET_LOCAL_LAST_MISSED_LIST "PN"
#define OVC3860_DIAL_LAST_RECEIVED_CALL "PO"
#define OVC3860_CLEAR_LOCAL_CALL_HISTORY "PR"
#define OVC3860_SPP_DATA_TRANSMIT "ST"
#define OVC3860_SET_CLOCKDEBUG_MODE "VC"
#define OVC3860_VOLUME_DOWN "VD"
#define OVC3860_ENTER_TEST_MODE "VE"
#define OVC3860_SET_FIXED_FREQUENCY "VF"
#define OVC3860_EMC_TEST_MODE "VG"
#define OVC3860_SET_RF_REGISTER "VH"
#define OVC3860_INQUIRY_START "VI"
#define OVC3860_INQUIRY_STOP "VJ"
#define OVC3860_VOLUME_UP "VU"
#define OVC3860_SHUTDOWN_MODULE "VX"

#endif /* OVC3860_CMDDEFINITION_H_ */
