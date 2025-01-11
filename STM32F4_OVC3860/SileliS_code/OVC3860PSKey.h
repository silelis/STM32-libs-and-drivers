/**
  ******************************************************************************
  * @file    OVC3860PSKey.h
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

#ifndef OVC3860PSKEY_H_
#define OVC3860PSKEY_H_
#include "OVC3860_device.h"

/* PARAMETRY DO ZMIANY NA INDYWIDUALNE */
//app_to_autooff
//app_to_pairing
//localname		-- bool 	writeBtName(const char* name);
//boundrate to 115200
//pin to 0000
//inne jakie sa potrzebne


#define OVC3860_PSKEY_ADDR_NAME 			PSkeys_localname
#define OVC3860_PSKEY_ADDR_PIN 				PSkeys_pincode
#define OVC3860_PSKEY_ADDR_LOCAL_BTADDR 	PSkeys_local_bdaddr
#define OVC3860_PSKEY_ADDR_BTSYS_MODE 		PSkeys_btsys_mode
#define OVC3860_PSKEY_ADDR_UART_BAUDRATE 	PSkeys_uart_baudrate
#define OVC3860_PSKEY_ADDR_CLASSOFDEVICE 	PSkeys_classofdevice

#define OVC3860_BAUDRATE_1200 				0x00
#define OVC3860_BAUDRATE_2400 				0x01
#define OVC3860_BAUDRATE_4800 				0x02
#define OVC3860_BAUDRATE_9600 				0x03
#define OVC3860_BAUDRATE_14400 				0x04
#define OVC3860_BAUDRATE_19200 				0x05
#define OVC3860_BAUDRATE_38400 				0x06
#define OVC3860_BAUDRATE_57600 				0x07
#define OVC3860_BAUDRATE_115200 			0x08
#define OVC3860_BAUDRATE_230400 			0x09
#define OVC3860_BAUDRATE_460800 			0x0A
#define OVC3860_BAUDRATE_921600 			0x0B

//------------------------------------------------------------------------------//
//		PSkey_name				Key_address		Key_number		Default_value	//
//------------------------------------------------------------------------------//
#define PSkeys_ool_reg04 		0x00		//	0
//------------------------------------------------------------------------------
#define PSkeys_ool_reg05 		0x01		//	1
//------------------------------------------------------------------------------
#define PSkeys_ool_reg07 		0x02		//	2
//------------------------------------------------------------------------------
#define PSkeys_ool_reg08 		0x03		//	3
//------------------------------------------------------------------------------
#define PSkeys_ool_reg0b 		0x04		//	4
//------------------------------------------------------------------------------
#define PSkeys_ool_reg0c 		0x05		//	5
//------------------------------------------------------------------------------
#define PSkeys_ool_reg0d 		0x06		//	6
//------------------------------------------------------------------------------
#define PSkeys_btsys_mode 		0x07		//	7				//HCI UART:00; HEADSET:03; CARKIT:04; AVSNK:06; MOBILE:09;
//------------------------------------------------------------------------------
#define PSkeys_sysclk 			0x08		//	8				//48M:00; 24M:01; 12M:02; 36M:03;
//------------------------------------------------------------------------------
#define PSkeys_lowpow_clk 		0x09		//	9
//------------------------------------------------------------------------------
#define PSkeys_idle_clk 		0x0A		//	10				//48M:00; 24M:01; 12M:02; 36M:03;
//------------------------------------------------------------------------------
#define PSkeys_con_clk 			0x0B		//	11				//48M:00; 24M:01; 12M:02; 36M:03;
//------------------------------------------------------------------------------
#define PSkeys_waketime 		0x0C		//	12
//------------------------------------------------------------------------------
#define PSkeys_radio_skew 		0x0D		//	13
//------------------------------------------------------------------------------
#define PSkeys_clk_recov 		0x0E		//	14
//------------------------------------------------------------------------------
#define PSkeys_diag_port 		0x0F		//	15
//------------------------------------------------------------------------------
#define PSkeys_sys_debug_ctrl 		0x10		//	16
//------------------------------------------------------------------------------
#define PSkeys_uart_baudrate 		0x11		//	17			//Dec:1200,2400,4800,9600,14400,19200,38400,57600,115200,230400,460800,921600
//------------------------------------------------------------------------------
#define PSkeys_gpioA_sec_func 		0x12		//	18
//------------------------------------------------------------------------------
#define PSkeys_gpioB_sec_func 		0x13		//	19
//------------------------------------------------------------------------------
#define PSkeys_auto_conn_time 		0x14		//	20
//------------------------------------------------------------------------------
#define PSkeys_sbc_max_mute_num 	0x15		//	21
//------------------------------------------------------------------------------
#define PSkeys_gpioA_inout_sel 		0x16		//	22
//------------------------------------------------------------------------------
#define PSkeys_gpioB_inout_sel 		0x17		//	23
//------------------------------------------------------------------------------
#define PSkeys_local_bdaddr 		0x18		//	24
//------------------------------------------------------------------------------
#define PSkeys_tester_addr 			0x1E		//	25
//------------------------------------------------------------------------------
#define PSkeys_classofdevice 		0x24		//	26
//------------------------------------------------------------------------------
#define PSkeys_clk_accuracy 		0x27		//	27
//------------------------------------------------------------------------------
#define PSkeys_clk_dis_reg 			0x29		//	28
//------------------------------------------------------------------------------
#define PSkeys_opccr_reg 			0x2B		//	29
//------------------------------------------------------------------------------
#define PSkeys_localfeatures0 		0x2C		//	30
//------------------------------------------------------------------------------
#define PSkeys_localfeatures1 		0x2D		//	31
//------------------------------------------------------------------------------
#define PSkeys_localfeatures2 		0x2E		//	32
//------------------------------------------------------------------------------
#define PSkeys_localfeatures3 		0x2F		//	33
//------------------------------------------------------------------------------
#define PSkeys_localfeatures4 		0x30		//	34
//------------------------------------------------------------------------------
#define PSkeys_localfeatures5 		0x31		//	35
//------------------------------------------------------------------------------
#define PSkeys_option_flag 			0x32		//	36
//------------------------------------------------------------------------------
#define PSkeys_reg_enbpreset 		0x33		//	37
//------------------------------------------------------------------------------
#define PSkeys_sys_pllclk 			0x37		//	38
//------------------------------------------------------------------------------
#define PSkeys_ool_spi_clkdiv 		0x39		//	39
//------------------------------------------------------------------------------
#define PSkeys_mute_opt_flag 		0x3B		//	40
//------------------------------------------------------------------------------
#define PSkeys_sleep_delay_count 	0x3C		//	41
//------------------------------------------------------------------------------
#define PSkeys_idle_ledoff_cnt 		0x3D		//	42
//------------------------------------------------------------------------------
#define PSkeys_idle_waketime 		0x3E		//	43
//------------------------------------------------------------------------------
#define PSkeys_sniff_ledcnt 		0x3F		//	44
//------------------------------------------------------------------------------
#define PSkeys_ledmode 				0x40		//	45
//------------------------------------------------------------------------------
#define PSkeys_idle_wakecount 		0x41		//	46
//------------------------------------------------------------------------------
#define PSkeys_master_sched_count 	0x42		//	47
//------------------------------------------------------------------------------
#define PSkeys_option_select 		0x43		//	48
//------------------------------------------------------------------------------
#define PSkeys_sleep_intcount 		0x44		//	49
//------------------------------------------------------------------------------
#define PSkeys_wd_loadvalue 		0x48		//	50
//------------------------------------------------------------------------------
#define PSkeys_rf_regs00 		0x4A		//	51
#define PSkeys_rf_regs01 		0x4B		//	52
#define PSkeys_rf_regs02 		0x4C		//	53
#define PSkeys_rf_regs03 		0x4D		//	54
#define PSkeys_rf_regs04 		0x4E		//	55
#define PSkeys_rf_regs05 		0x4F		//	56
#define PSkeys_rf_regs06 		0x50		//	57
#define PSkeys_rf_regs07 		0x51		//	58
#define PSkeys_rf_regs08 		0x52		//	59
#define PSkeys_rf_regs09 		0x53		//	60
#define PSkeys_rf_regs0a 		0x54		//	61
#define PSkeys_rf_regs0b 		0x55		//	62
#define PSkeys_rf_regs0c 		0x56		//	63
#define PSkeys_rf_regs0d 		0x57		//	64
#define PSkeys_rf_regs0e 		0x58		//	65
#define PSkeys_rf_regs0f 		0x59		//	66
#define PSkeys_rf_regs11 		0x5A		//	67
#define PSkeys_rf_regs12 		0x5B		//	68
#define PSkeys_rf_regs13 		0x5C		//	69
#define PSkeys_rf_regs14 		0x5D		//	70
#define PSkeys_rf_regs15 		0x5E		//	71
#define PSkeys_rf_regs16 		0x5F		//	72
#define PSkeys_rf_regs17 		0x60		//	73
#define PSkeys_rf_regs18 		0x61		//	74
#define PSkeys_rf_regs19 		0x62		//	75
#define PSkeys_rf_regs1a 		0x63		//	76
#define PSkeys_rf_regs1b 		0x64		//	77
#define PSkeys_rf_regs1c 		0x65		//	78
#define PSkeys_rf_regs1d 		0x66		//	79
#define PSkeys_rf_regs1e 		0x67		//	80
#define PSkeys_rf_regs1f 		0x68		//	81
#define PSkeys_rf_regs22 		0x69		//	82
#define PSkeys_rf_regs23 		0x6A		//	83
#define PSkeys_rf_regs24 		0x6B		//	84
#define PSkeys_rf_regs25 		0x6C		//	85
#define PSkeys_rf_regs26 		0x6D		//	86
#define PSkeys_rf_regs27 		0x6E		//	87
#define PSkeys_rf_regs28 		0x6F		//	88
#define PSkeys_rf_regs29 		0x70		//	89
#define PSkeys_rf_regs2a 		0x71		//	90
#define PSkeys_rf_regs2b 		0x72		//	91
#define PSkeys_rf_regs2c 		0x73		//	92
#define PSkeys_rf_regs2d 		0x74		//	93
#define PSkeys_rf_regs2e 		0x75		//	94
#define PSkeys_rf_regs2f 		0x76		//	95
#define PSkeys_rf_regs30 		0x77		//	96
#define PSkeys_rf_regs31 		0x78		//	97
#define PSkeys_rf_regs32 		0x79		//	98
#define PSkeys_rf_regs33 		0x7A		//	99
#define PSkeys_rf_regs34 		0x7B		//	100
#define PSkeys_rf_regs37 		0x7C		//	101
#define PSkeys_rf_regs38 		0x7D		//	102
#define PSkeys_rf_regs39 		0x7E		//	103
#define PSkeys_rf_regs3a 		0x7F		//	104
#define PSkeys_rf_regs3b 		0x80		//	105
#define PSkeys_rf_regs3c 		0x81		//	106
#define PSkeys_rf_regs3d 		0x82		//	107
#define PSkeys_rf_regs3e 		0x83		//	108
#define PSkeys_rf_regs3f 		0x84		//	109
#define PSkeys_rf_regs41 		0x85		//	110
#define PSkeys_rf_regs42 		0x86		//	111
#define PSkeys_rf_regs43 		0x87		//	112
#define PSkeys_rf_regs44 		0x88		//	113
#define PSkeys_rf_regs45 		0x89		//	114
#define PSkeys_rf_regs46 		0x8A		//	115
#define PSkeys_rf_regs47 		0x8B		//	116
#define PSkeys_rf_regs48 		0x8C		//	117
#define PSkeys_rf_regs49 		0x8D		//	118
#define PSkeys_rf_regs4d 		0x8E		//	119
#define PSkeys_rf_regs4e 		0x8F		//	120
#define PSkeys_rf_regs4f 		0x90		//	121
#define PSkeys_rf_regs50 		0x91		//	122
#define PSkeys_rf_regs51 		0x92		//	123
#define PSkeys_rf_regs52 		0x93		//	124
#define PSkeys_rf_regs53 		0x94		//	125
#define PSkeys_rf_regs54 		0x95		//	126
#define PSkeys_rf_regs55 		0x96		//	127
#define PSkeys_rf_regs56 		0x97		//	128
#define PSkeys_rf_regs57 		0x98		//	129
#define PSkeys_rf_regs58 		0x99		//	130
#define PSkeys_rf_regs59 		0x9A		//	131
#define PSkeys_rf_regs5a 		0x9B		//	132
#define PSkeys_rf_regs5b 		0x9C		//	133
#define PSkeys_rf_regs5c 		0x9D		//	134
#define PSkeys_rf_regs5d 		0x9E		//	135
#define PSkeys_rf_regs5e 		0x9F		//	136
#define PSkeys_rf_regs5f 		0xA0		//	137
#define PSkeys_rf_regs60 		0xA1		//	138
#define PSkeys_rf_regs61 		0xA2		//	139
#define PSkeys_rf_regs62 		0xA3		//	140
#define PSkeys_rf_regs64 		0xA4		//	141
#define PSkeys_rf_regs65 		0xA5		//	142
#define PSkeys_rf_regs67 		0xA6		//	143
#define PSkeys_rf_regs68 		0xA7		//	144
#define PSkeys_rf_regs6b 		0xA8		//	145
#define PSkeys_rf_regs6c 		0xA9		//	146
#define PSkeys_rf_regs6d 		0xAA		//	147
#define PSkeys_rf_regs6e 		0xAB		//	148
#define PSkeys_rf_regs6f 		0xAC		//	149
#define PSkeys_rf_regs70 		0xAD		//	150
#define PSkeys_rf_regs71 		0xAE		//	151
#define PSkeys_rf_regs72 		0xAF		//	152
#define PSkeys_rf_regs73 		0xB0		//	153
#define PSkeys_rf_regs74 		0xB1		//	154
#define PSkeys_rf_regs75 		0xB2		//	155
#define PSkeys_rf_regs76 		0xB3		//	156
#define PSkeys_rf_regs77 		0xB4		//	157
#define PSkeys_rf_regs78 		0xB5		//	158
#define PSkeys_rf_regs79 		0xB6		//	159
#define PSkeys_rf_regs7a 		0xB7		//	160
#define PSkeys_rf_regs7b 		0xB8		//	161
#define PSkeys_rf_regs7c 		0xB9		//	162
#define PSkeys_rf_regs7d 		0xBA		//	163
#define PSkeys_rf_regs7e 		0xBB		//	164
#define PSkeys_rf_regs7f 		0xBC		//	165
#define PSkeys_rf_regs80 		0xBD		//	166
#define PSkeys_rf_regs81 		0xBE		//	167
#define PSkeys_rf_regs82 		0xBF		//	168
#define PSkeys_rf_regs83 		0xC0		//	169
#define PSkeys_rf_regs84 		0xC1		//	170
#define PSkeys_rf_regs85 		0xC2		//	171
#define PSkeys_rf_regs86 		0xC3		//	172
#define PSkeys_rf_regs87 		0xC4		//	173
#define PSkeys_rf_regs88 		0xC5		//	174
#define PSkeys_rf_regs89 		0xC6		//	175
#define PSkeys_rf_regs8a 		0xC7		//	176
#define PSkeys_rf_regs8b 		0xC8		//	177
#define PSkeys_rf_regs8c 		0xC9		//	178
#define PSkeys_rf_regs8d 		0xCA		//	179
#define PSkeys_rf_regs8e 		0xCB		//	180
#define PSkeys_rf_regs8f 		0xCC		//	181
#define PSkeys_rf_regs90 		0xCD		//	182
#define PSkeys_rf_regs91 		0xCE		//	183
#define PSkeys_rf_regs92 		0xCF		//	184
#define PSkeys_rf_regs93 		0xD0		//	185
#define PSkeys_rf_regs9c 		0xD1		//	186
#define PSkeys_rf_regs9d 		0xD2		//	187
#define PSkeys_rf_regsa2 		0xD3		//	188
#define PSkeys_rf_regsa3 		0xD4		//	189
#define PSkeys_rf_regsa4 		0xD5		//	190
#define PSkeys_rf_regsa5 		0xD6		//	191
#define PSkeys_rf_regsa6 		0xD7		//	192
#define PSkeys_rf_regsa7 		0xD8		//	193
#define PSkeys_rf_regsa8 		0xD9		//	194
//------------------------------------------------------------------------------
#define PSkeys_lvd_reserved 	0xDA		//	195
//------------------------------------------------------------------------------
#define PSkeys_app_btn_num 		0xDB		//	196
//------------------------------------------------------------------------------
#define PSkeys_dure_short 		0xDC		//	197
//------------------------------------------------------------------------------
#define PSkeys_dure_long 		0xDD		//	198
//------------------------------------------------------------------------------
#define PSkeys_dure_vlong 		0xDE		//	199
//------------------------------------------------------------------------------
#define PSkeys_dure_repeat 		0xDF		//	200
//------------------------------------------------------------------------------
#define PSkeys_dure_dshort_interval 		0xE0		//	201
//------------------------------------------------------------------------------
#define PSkeys_dure_hold 			0xE1		//	202
//------------------------------------------------------------------------------
#define PSkeys_func_vol1_short 		0xE2		//	203
//------------------------------------------------------------------------------
#define PSkeys_func_vol1_hold 		0xE3		//	204
//------------------------------------------------------------------------------
#define PSkeys_func_vol2_short 		0xE4		//	205
//------------------------------------------------------------------------------
#define PSkeys_func_vol2_hold 		0xE5		//	206
//------------------------------------------------------------------------------
#define PSkeys_on_acok 				0xE6		//	207
//------------------------------------------------------------------------------
#define PSkeys_app_status_num 		0xE7		//	208
//------------------------------------------------------------------------------
#define PSkeys_led_para_00__app_status 			0xE8		//	209
#define PSkeys_led_para_00__led_on_time 		0xE9		//	210
#define PSkeys_led_para_00__led_off_time 		0xEA		//	211
#define PSkeys_led_para_00__led_repeat_time 	0xEB		//	212
#define PSkeys_led_para_00__led_flash_num 		0xEC		//	213
#define PSkeys_led_para_00__led_color 			0xED		//	214
#define PSkeys_led_para_01__app_status 			0xEE		//	215
#define PSkeys_led_para_01__led_on_time 		0xEF		//	216
#define PSkeys_led_para_01__led_off_time 		0xF0		//	217
#define PSkeys_led_para_01__led_repeat_time 	0xF1		//	218
#define PSkeys_led_para_01__led_flash_num 		0xF2		//	219
#define PSkeys_led_para_01__led_color  			0xF3		//	220
#define PSkeys_led_para_02__app_status 			0xF4		//	221
#define PSkeys_led_para_02__led_on_time 		0xF5		//	222
#define PSkeys_led_para_02__led_off_time 		0xF6		//	223
#define PSkeys_led_para_02__led_repeat_time 	0xF7		//	224
#define PSkeys_led_para_02__led_flash_num 		0xF8		//	225
#define PSkeys_led_para_02__led_color 			0xF9		//	226
#define PSkeys_led_para_03__app_status 			0xFA		//	227
#define PSkeys_led_para_03__led_on_time 		0xFB		//	228
#define PSkeys_led_para_03__led_off_time 		0xFC		//	229
#define PSkeys_led_para_03__led_repeat_time 	0xFD		//	230
#define PSkeys_led_para_03__led_flash_num 		0xFE		//	231
#define PSkeys_led_para_03__led_color 			0xFF		//	232
#define PSkeys_led_para_04__app_status 			0x100		//	233
#define PSkeys_led_para_04__led_on_time 		0x101		//	234
#define PSkeys_led_para_04__led_off_time 		0x102		//	235
#define PSkeys_led_para_04__led_repeat_time 	0x103		//	236
#define PSkeys_led_para_04__led_flash_num 		0x104		//	237
#define PSkeys_led_para_04__led_color 			0x105		//	238
#define PSkeys_led_para_05__app_status 			0x106		//	239
#define PSkeys_led_para_05__led_on_time 		0x107		//	240
#define PSkeys_led_para_05__led_off_time 		0x108		//	241
#define PSkeys_led_para_05__led_repeat_time 	0x109		//	242
#define PSkeys_led_para_05__led_flash_num 		0x10A		//	243
#define PSkeys_led_para_05__led_color 			0x10B		//	244
#define PSkeys_led_para_06__app_status 			0x10C		//	245
#define PSkeys_led_para_06__led_on_time 		0x10D		//	246
#define PSkeys_led_para_06__led_off_time 		0x10E		//	247
#define PSkeys_led_para_06__led_repeat_time 	0x10F		//	248
#define PSkeys_led_para_06__led_flash_num: 		0x110		//	249
#define PSkeys_led_para_06__led_color 			0x111		//	250
#define PSkeys_led_para_07__app_status 			0x112		//	251
#define PSkeys_led_para_07__led_on_time 		0x113		//	252
#define PSkeys_led_para_07__led_off_time 		0x114		//	253
#define PSkeys_led_para_07__led_repeat_time 	0x115		//	254
#define PSkeys_led_para_07__led_flash_num 		0x116		//	255
#define PSkeys_led_para_07__led_color 			0x117		//	256
#define PSkeys_led_para_08__app_status 			0x118		//	257
#define PSkeys_led_para_08__led_on_time 		0x119		//	258
#define PSkeys_led_para_08__led_off_time 		0x11A		//	259
#define PSkeys_led_para_08__led_repeat_time 	0x11B		//	260
#define PSkeys_led_para_08__led_flash_num 		0x11C		//	261
#define PSkeys_led_para_08__led_color 			0x11D		//	262
#define PSkeys_led_para_09__app_status 			0x11E		//	263
#define PSkeys_led_para_09__led_on_time 		0x11F		//	264
#define PSkeys_led_para_09__led_off_time 		0x120		//	265
#define PSkeys_led_para_09__led_repeat_time 	0x121		//	266
#define PSkeys_led_para_09__led_flash_num 		0x122		//	267
#define PSkeys_led_para_09__led_color 			0x123		//	268
#define PSkeys_led_para_10__app_status 			0x124		//	269
#define PSkeys_led_para_10__led_on_time 		0x125		//	270
#define PSkeys_led_para_10__led_off_time 		0x126		//	271
#define PSkeys_led_para_10__led_repeat_time 	0x127		//	272
#define PSkeys_led_para_10__led_flash_num 		0x128		//	273
#define PSkeys_led_para_10__led_color 			0x129		//	274
#define PSkeys_led_para_11__app_status 			0x12A		//	275
#define PSkeys_led_para_11__led_on_time 		0x12B		//	276
#define PSkeys_led_para_11__led_off_time 		0x12C		//	277
#define PSkeys_led_para_11__led_repeat_time 	0x12D		//	278
#define PSkeys_led_para_11__led_flash_num 		0x12E		//	279
#define PSkeys_led_para_11__led_color 			0x12F		//	280
//------------------------------------------------------------------------------
#define PSkeys_tone_conf_00__size_array 		0x130		//	281
#define PSkeys_tone_conf_00__tone_array_0_ 		0x131		//	282
#define PSkeys_tone_conf_00__tone_array_1_ 		0x132		//	283
#define PSkeys_tone_conf_00__tone_array_2_ 		0x133		//	284
#define PSkeys_tone_conf_00__cnt 				0x134		//	285
#define PSkeys_tone_conf_01__size_array 		0x135		//	286
#define PSkeys_tone_conf_01__tone_array_0_ 		0x136		//	287
#define PSkeys_tone_conf_01__tone_array_1_ 		0x137		//	288
#define PSkeys_tone_conf_01__tone_array_2_ 		0x138		//	289
#define PSkeys_tone_conf_01__cnt 				0x139		//	290
#define PSkeys_tone_conf_02__size_array 		0x13A		//	291
#define PSkeys_tone_conf_02__tone_array_0_ 		0x13B		//	292
#define PSkeys_tone_conf_02__tone_array_1_ 		0x13C		//	293
#define PSkeys_tone_conf_02__tone_array_2_ 		0x13D		//	294
#define PSkeys_tone_conf_02__cnt 				0x13E		//	295
#define PSkeys_tone_conf_03__size_array 		0x13F		//	296
#define PSkeys_tone_conf_03__tone_array_0_ 		0x140		//	297
#define PSkeys_tone_conf_03__tone_array_1_ 		0x141		//	298
#define PSkeys_tone_conf_03__tone_array_2_ 		0x142		//	299
#define PSkeys_tone_conf_03__cnt 				0x143		//	300
#define PSkeys_tone_conf_04__size_array 		0x144		//	301
#define PSkeys_tone_conf_04__tone_array_0_ 		0x145		//	302
#define PSkeys_tone_conf_04__tone_array_1_ 		0x146		//	303
#define PSkeys_tone_conf_04__tone_array_2_ 		0x147		//	304
#define PSkeys_tone_conf_04__cnt 				0x148		//	305
#define PSkeys_tone_conf_05__size_array 		0x149		//	306
#define PSkeys_tone_conf_05__tone_array_0_ 		0x14A		//	307
#define PSkeys_tone_conf_05__tone_array_1_ 		0x14B		//	308
#define PSkeys_tone_conf_05__tone_array_2_ 		0x14C		//	309
#define PSkeys_tone_conf_05__cnt 				0x14D		//	310
#define PSkeys_tone_conf_06__size_array 		0x14E		//	311
#define PSkeys_tone_conf_06__tone_array_0_ 		0x14F		//	312
#define PSkeys_tone_conf_06__tone_array_1_ 		0x150		//	313
#define PSkeys_tone_conf_06__tone_array_2_ 		0x151		//	314
#define PSkeys_tone_conf_06__cnt 				0x152		//	315
#define PSkeys_tone_conf_07__size_array 		0x153		//	316
#define PSkeys_tone_conf_07__tone_array_0_ 		0x154		//	317
#define PSkeys_tone_conf_07__tone_array_1_ 		0x155		//	318
#define PSkeys_tone_conf_07__tone_array_2_ 		0x156		//	319
#define PSkeys_tone_conf_07__cnt 				0x157		//	320
#define PSkeys_tone_conf_08__size_array 		0x158		//	321
#define PSkeys_tone_conf_08__tone_array_0_ 		0x159		//	322
#define PSkeys_tone_conf_08__tone_array_1_ 		0x15A		//	323
#define PSkeys_tone_conf_08__tone_array_2_ 		0x15B		//	324
#define PSkeys_tone_conf_08__cnt 				0x15C		//	325
#define PSkeys_tone_conf_09__size_array 		0x15D		//	326
#define PSkeys_tone_conf_09__tone_array_0_ 		0x15E		//	327
#define PSkeys_tone_conf_09__tone_array_1_ 		0x15F		//	328
#define PSkeys_tone_conf_09__tone_array_2_ 		0x160		//	329
#define PSkeys_tone_conf_09__cnt 				0x161		//	330
#define PSkeys_tone_conf_10__size_array 		0x162		//	331
#define PSkeys_tone_conf_10__tone_array_0_ 		0x163		//	332
#define PSkeys_tone_conf_10__tone_array_1_ 		0x164		//	333
#define PSkeys_tone_conf_10__tone_array_2_ 		0x165		//	334
#define PSkeys_tone_conf_10__cnt 				0x166		//	335
#define PSkeys_tone_conf_11__size_array 		0x167		//	336
#define PSkeys_tone_conf_11__tone_array_0_ 		0x168		//	337
#define PSkeys_tone_conf_11__tone_array_1_ 		0x169		//	338
#define PSkeys_tone_conf_11__tone_array_2_ 		0x16A		//	339
#define PSkeys_tone_conf_11__cnt 				0x16B		//	340
//------------------------------------------------------------------------------
#define PSkeys_ringtone_tone_array_00_ 		0x16C		//	341
#define PSkeys_ringtone_tone_array_01_ 		0x16D		//	342
#define PSkeys_ringtone_tone_array_02_ 		0x16E		//	343
#define PSkeys_ringtone_tone_array_03_ 		0x16F		//	344
#define PSkeys_ringtone_tone_array_04_ 		0x170		//	345
#define PSkeys_ringtone_tone_array_05_ 		0x171		//	346
#define PSkeys_ringtone_tone_array_06_ 		0x172		//	347
#define PSkeys_ringtone_tone_array_07_ 		0x173		//	348
#define PSkeys_ringtone_tone_array_08_ 		0x174		//	349
#define PSkeys_ringtone_tone_array_09_ 		0x175		//	350
#define PSkeys_ringtone_tone_array_10_ 		0x176		//	351
#define PSkeys_ringtone_tone_array_11_ 		0x177		//	352
#define PSkeys_ringtone_tone_array_12_ 		0x178		//	353
#define PSkeys_ringtone_tone_array_13_ 		0x179		//	354
#define PSkeys_ringtone_size_array 			0x17A		//	355
#define PSkeys_ringtone_cnt 				0x17B		//	356
#define PSkeys_ringtone_useLocal 			0x17C		//	357
//------------------------------------------------------------------------------
#define PSkeys_lvdtone_tone_array_00_ 		0x17D		//	358
#define PSkeys_lvdtone_tone_array_01_ 		0x17E		//	359
#define PSkeys_lvdtone_tone_array_02_  		0x17F		//	360
#define PSkeys_lvdtone_tone_array_03_    	0x180		//	361
#define PSkeys_lvdtone_size_array       	0x181		//	362
#define PSkeys_lvdtone_cnt    				0x182		//	363
#define PSkeys_lvdtone_interval    			0x183		//	364
//------------------------------------------------------------------------------
#define PSkeys_mutetone_tone_array_00_ 		0x185		//	365
#define PSkeys_mutetone_tone_array_01_    	0x186		//	366
#define PSkeys_mutetone_tone_array_02_    	0x187		//	367
#define PSkeys_mutetone_tone_array_03_    	0x188		//	368
#define PSkeys_mutetone_size_array       	0x189		//	369
#define PSkeys_mutetone_cnt    				0x18A		//	370
#define PSkeys_mutetone_interval    		0x18B		//	371
//------------------------------------------------------------------------------
#define PSkeys_app_btn_mask_bit 	0x18D		//	372
//------------------------------------------------------------------------------
#define PSkeys_app_to_pairing 		0x18E		//	373
//------------------------------------------------------------------------------
#define PSkeys_app_to_autooff 		0x18F		//	374
//------------------------------------------------------------------------------
#define PSkeys_app_to_autoconn 		0x190		//	375
//------------------------------------------------------------------------------
#define PSkeys_app_to_sniff 		0x191		//	376
//------------------------------------------------------------------------------
#define PSkeys_app_vol_spk 			0x192		//	377
//------------------------------------------------------------------------------
#define PSkeys_app_vol_mic 			0x193		//	378
//------------------------------------------------------------------------------
#define PSkeys_isAutoConnect 		0x194		//	379
//------------------------------------------------------------------------------
#define PSkeys_isAutoPair 			0x195		//	380
//------------------------------------------------------------------------------
#define PSkeys_isAutoAnswer 		0x196		//	381
//------------------------------------------------------------------------------
#define PSkeys_isP2MPSupport 		0x197		//	382
//------------------------------------------------------------------------------
#define PSkeys_app_hf_feature 		0x198		//	383
//------------------------------------------------------------------------------
#define PSkeys_app_av_feature 		0x199		//	384
//------------------------------------------------------------------------------
#define PSkeys_app_avc_feature 		0x19A		//	385
//------------------------------------------------------------------------------
#define PSkeys_sdp_max_attr_byte_cnt 	0x19B		//	386
//------------------------------------------------------------------------------
#define PSkeys_codec_para_reg_r00 		0x19D		//	387
//------------------------------------------------------------------------------
#define PSkeys_codec_para_reg_r04   		0x19E		//	388
#define PSkeys_codec_para_reg_r05   		0x19F		//	389
#define PSkeys_codec_para_reg_r07    		0x1A0		//	390
#define PSkeys_codec_para_reg_r09_iis 		0x1A1		//	391
#define PSkeys_codec_para_reg_r0a    		0x1A2		//	392
#define PSkeys_codec_para_reg_r0b    		0x1A3		//	393
#define PSkeys_codec_para_reg_r0c    		0x1A4		//	394
#define PSkeys_codec_para_reg_r12    		0x1A5		//	395
#define PSkeys_codec_para_reg_r15    		0x1A6		//	396
#define PSkeys_codec_para_reg_r16    		0x1A7		//	397
#define PSkeys_codec_para_reg_r17    		0x1A8		//	398
#define PSkeys_codec_para_reg_r18    		0x1A9		//	399
#define PSkeys_codec_para_reg_r1a    		0x1AA		//	400
#define PSkeys_codec_para_reg_r1b    		0x1AB		//	401
#define PSkeys_codec_para_reg_r1d    		0x1AC		//	402
#define PSkeys_codec_para_reg_r1e    		0x1AD		//	403
#define PSkeys_codec_para_reg_r1f    		0x1AE		//	404
#define PSkeys_codec_para_reg_r20    		0x1AF		//	405
#define PSkeys_codec_para_reg_r21    		0x1B0		//	406
#define PSkeys_codec_para_reg_r22    		0x1B1		//	407
#define PSkeys_codec_para_reg_r23    		0x1B2		//	408
#define PSkeys_codec_para_reg_r12_iis 		0x1B3		//	409
#define PSkeys_codec_para_reg_r12_pcm 		0x1B4		//	410
#define PSkeys_codec_para_reg_r09_pcm 		0x1B5		//	411
//------------------------------------------------------------------------------
#define PSkeys_pcm_en 			0x1B6		//	412
//------------------------------------------------------------------------------
#define PSkeys_pcm_reg 			0x1B7		//	413
//------------------------------------------------------------------------------
#define PSkeys_extint_ctrl 		0x1B9		//	414
//------------------------------------------------------------------------------
#define PSkeys_uart_flowctl 	0x1BE		//	415
//------------------------------------------------------------------------------
#define PSkeys_pincode 			0x1BF		//	416
//------------------------------------------------------------------------------
#define PSkeys_localname 		0x1C7		//	417
//------------------------------------------------------------------------------
#define PSkeys_dma_line_reg 	0x1D7		//	418
//------------------------------------------------------------------------------
#define PSkeys_opera_mem 		0x1D8		//	419
//------------------------------------------------------------------------------
#define PSkeys_content0 		0x1D9		//	420
//------------------------------------------------------------------------------
#define PSkeys_content1 		0x1DB		//	421
//------------------------------------------------------------------------------
#define PSkeys_content2 		0x1DD		//	422
//------------------------------------------------------------------------------
#define PSkeys_content3 		0x1DF		//	423
//------------------------------------------------------------------------------
#define PSkeys_mem_addr0 		0x1E1		//	424
//------------------------------------------------------------------------------
#define PSkeys_mem_addr1 		0x1E5		//	425
//------------------------------------------------------------------------------
#define PSkeys_mem_addr2 		0x1E9		//	426
//------------------------------------------------------------------------------
#define PSkeys_mem_addr3 		0x1ED		//	427
//------------------------------------------------------------------------------
#define PSkeys_sbc_eq_reg 		0x1F1		//	428
//------------------------------------------------------------------------------
#define PSkeys_pskeys_enable 	0x1F3		//	429
//------------------------------------------------------------------------------
#define PSkeys_reserved0_00_ 		0x1F4		//	430
#define PSkeys_reserved0_01_ 		0x1F5		//	431
#define PSkeys_reserved0_02_ 		0x1F6		//	432
#define PSkeys_reserved0_03_ 		0x1F7		//	433
#define PSkeys_STMO 				0x1F8		//	434
#define PSkeys_COUNT 				0x1F9		//	435
#define PSkeys_reserved0_06_ 		0x1FA		//	436
#define PSkeys_reserved0_07_ 		0x1FB		//	437
#define PSkeys_reserved0_08_ 		0x1FC		//	438
#define PSkeys_reserved0_09_ 		0x1FD		//	439
#define PSkeys_reserved0_10_ 		0x1FE		//	440
#define PSkeys_reserved0_11_ 		0x1FF		//	441
//------------------------------------------------------------------------------
#define PSkeys_nvram_app_ready 		0x200		//	442
//------------------------------------------------------------------------------
#define PSkeys_keyTable_00__valid 		0x201		//	443
#define PSkeys_keyTable_00__bdaddr 		0x202		//	444
#define PSkeys_keyTable_00__key 		0x208		//	445
#define PSkeys_keyTable_01__valid 		0x218		//	446
#define PSkeys_keyTable_01__bdaddr 		0x219		//	447
#define PSkeys_keyTable_01__key 		0x21F		//	448
#define PSkeys_keyTable_02__valid 		0x22F		//	449
#define PSkeys_keyTable_02__bdaddr 		0x230		//	450
#define PSkeys_keyTable_02__key 		0x236		//	451
#define PSkeys_keyTable_03__valid 		0x246		//	452
#define PSkeys_keyTable_03__bdaddr 		0x247		//	453
#define PSkeys_keyTable_03__key 		0x24D		//	454
#define PSkeys_keyTable_04__valid 		0x25D		//	455
#define PSkeys_keyTable_04__bdaddr 		0x25E		//	456
#define PSkeys_keyTable_04__key 		0x264		//	457
#define PSkeys_keyTable_05__valid 		0x274		//	458
#define PSkeys_keyTable_05__bdaddr 		0x275		//	459
#define PSkeys_keyTable_05__key 		0x27B		//	460
#define PSkeys_keyTable_06__valid 		0x28B		//	461
#define PSkeys_keyTable_06__bdaddr 		0x28C		//	462
#define PSkeys_keyTable_06__key 		0x292		//	463
#define PSkeys_keyTable_07__valid 		0x2A2		//	464
#define PSkeys_keyTable_07__bdaddr 		0x2A3		//	465
#define PSkeys_keyTable_07__key 		0x2A9		//	466
//------------------------------------------------------------------------------
#define PSkeys_last_device 				0x2B9		//	467
//------------------------------------------------------------------------------
#define PSkeys_last_device_profile 		0x2BF		//	468
//------------------------------------------------------------------------------
#define PSkeys_app_vol_spk__			0x2C0		//	469
//------------------------------------------------------------------------------
#define PSkeys_app_vol_mic__			0x2C1		//	470
//------------------------------------------------------------------------------
#define PSkeys_Code_Patch_enable 		0x2C2		//	471
//------------------------------------------------------------------------------
#define PSkeys_Code_Patch_code_size 	0x2C6		//	472
//------------------------------------------------------------------------------
#define PSkeys_Code_Patch_code_src 		0x2C8		//	473
//------------------------------------------------------------------------------
#define PSkeys_Code_Patch_code_des 		0x2CA		//	474
//------------------------------------------------------------------------------
#define PSkeys_Code_Patch_matrix_00__address 		0x2CE		//	475
#define PSkeys_Code_Patch_Code_Patch_matrix_00__ret_opt 		0x2D2		//	476
#define PSkeys_Code_Patch_matrix_00__enable 		0x2D4		//	477
#define PSkeys_Code_Patch_matrix_01__address 		0x2D6		//	478
#define PSkeys_Code_Patch_matrix_01__ret_opt 		0x2DA		//	479
#define PSkeys_Code_Patch_matrix_01__enable 		0x2DC		//	480
#define PSkeys_Code_Patch_matrix_02__address 		0x2De		//	481
#define PSkeys_Code_Patch_matrix_02__ret_opt 		0x2E2		//	482
#define PSkeys_Code_Patch_matrix_02__enable 		0x2E4		//	483
#define PSkeys_Code_Patch_matrix_03__address 		0x2E6		//	484
#define PSkeys_Code_Patch_matrix_03__ret_opt 		0x2EA		//	485
#define PSkeys_Code_Patch_matrix_03__enable 		0x2EC		//	486
#define PSkeys_Code_Patch_matrix_04__address 		0x2EE		//	487
#define PSkeys_Code_Patch_matrix_04__ret_opt 		0x2F2		//	488
#define PSkeys_Code_Patch_matrix_04__enable 		0x2F4		//	489
#define PSkeys_Code_Patch_matrix_05__address 		0x2F6		//	490
#define PSkeys_Code_Patch_matrix_05__ret_opt 		0x2FA		//	491
#define PSkeys_Code_Patch_matrix_05__enable 		0x2FC		//	492
#define PSkeys_Code_Patch_matrix_06__address 		0x2FE		//	493
#define PSkeys_Code_Patch_matrix_06__ret_opt 		0x302		//	494
#define PSkeys_Code_Patch_matrix_06__enable 		0x304		//	495
#define PSkeys_Code_Patch_matrix_07__address 		0x306		//	496
#define PSkeys_Code_Patch_matrix_07__ret_opt 		0x30A		//	497
#define PSkeys_Code_Patch_matrix_07__enable 		0x30C		//	498
#define PSkeys_Code_Patch_matrix_08__address 		0x30E		//	499
#define PSkeys_Code_Patch_matrix_08__ret_opt 		0x312		//	500
#define PSkeys_Code_Patch_matrix_08__enable 		0x314		//	501
#define PSkeys_Code_Patch_matrix_09__address 		0x316		//	502
#define PSkeys_Code_Patch_matrix_09__ret_opt 		0x31A		//	503
#define PSkeys_Code_Patch_matrix_09__enable 		0x31C		//	504
#define PSkeys_Code_Patch_matrix_10__address 		0x31E		//	505
#define PSkeys_Code_Patch_matrix_10__ret_opt 		0x322		//	506
#define PSkeys_Code_Patch_matrix_10__enable 		0x324		//	507
#define PSkeys_Code_Patch_matrix_11__address 		0x326		//	508
#define PSkeys_Code_Patch_matrix_11__ret_opt 		0x32A		//	509
#define PSkeys_Code_Patch_matrix_11__enable 		0x32C		//	510
#define PSkeys_Code_Patch_matrix_12__address 		0x32E		//	511
#define PSkeys_Code_Patch_matrix_12__ret_opt 		0x332		//	512
#define PSkeys_Code_Patch_matrix_12__enable 		0x334		//	513
#define PSkeys_Code_Patch_matrix_13__address 		0x336		//	514
#define PSkeys_Code_Patch_matrix_13__ret_opt 		0x33A		//	515
#define PSkeys_Code_Patch_matrix_13__enable 		0x33C		//	516
#define PSkeys_Code_Patch_matrix_14__address 		0x33E		//	517
#define PSkeys_Code_Patch_matrix_14__ret_opt 		0x342		//	518
#define PSkeys_Code_Patch_matrix_14__enable 		0x344		//	519
#define PSkeys_Code_Patch_matrix_15__address 		0x346		//	520
#define PSkeys_Code_Patch_matrix_15__ret_opt 		0x34A		//	521
#define PSkeys_Code_Patch_matrix_15__enable 		0x34C		//	522
#define PSkeys_Code_Patch_matrix_16__address 		0x34E		//	523
#define PSkeys_Code_Patch_matrix_16__ret_opt 		0x352		//	524
#define PSkeys_Code_Patch_matrix_16__enable 		0x354		//	525
#define PSkeys_Code_Patch_matrix_17__address 		0x356		//	526
#define PSkeys_Code_Patch_matrix_17__ret_opt 		0x35A		//	527
#define PSkeys_Code_Patch_matrix_17__enable 		0x35C		//	528
#define PSkeys_Code_Patch_matrix_18__address 		0x35E		//	529
#define PSkeys_Code_Patch_matrix_18__ret_opt 		0x362		//	530
#define PSkeys_Code_Patch_matrix_18__enable 		0x364		//	531
#define PSkeys_Code_Patch_matrix_19__address 		0x366		//	532
#define PSkeys_Code_Patch_matrix_19__ret_opt 		0x36A		//	533
#define PSkeys_Code_Patch_matrix_19__enable 		0x36C		//	534
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#define PSkeys_user_defined_tone_sbc_size_00_ 		0x36E		//	535
#define PSkeys_user_defined_tone_sbc_des_00_ 		0x370		//	536
#define PSkeys_user_defined_tone_sbc_size_01_ 		0x374		//	537
#define PSkeys_user_defined_tone_sbc_des_01_ 		0x376		//	538
#define PSkeys_user_defined_tone_sbc_size_02_ 		0x37A		//	539
#define PSkeys_user_defined_tone_sbc_des_02_ 		0x37C		//	540
#define PSkeys_user_defined_tone_sbc_size_03_ 		0x380		//	541
#define PSkeys_user_defined_tone_sbc_des_03_ 		0x382		//	542
#define PSkeys_user_defined_tone_sbc_size_04_ 		0x386		//	543
#define PSkeys_user_defined_tone_sbc_des_04_ 		0x388		//	544
#define PSkeys_user_defined_tone_sbc_size_05_ 		0x38C		//	545
#define PSkeys_user_defined_tone_sbc_des_05_ 		0x38E		//	546
//------------------------------------------------------------------------------


/*
 * OVC3860PSKey is class to manage with OVC3860 PSKeys
 * The parrent class is  OVC3860HardWare which sets hardware
 *  issues of OVC3860 i.e. uart, reset line port and pin
 *
 *  Please take under consideration that this class uses blocking
 *   mode (NOT dma or interrupt) to contact with chip.
 */
class OVC3860PSKey: public OVC3860HardWare
{
public:
	OVC3860PSKey(UART_HandleTypeDef* huart, GPIO_TypeDef* ResetGPIOx, uint16_t GPIO_Pin);
	~OVC3860PSKey();
	bool enterConfigMode();
	bool quitConfigMode();
	bool readDataFromOVC(uint16_t address, uint16_t data2readLenght);
	bool writeData2OVC(uint16_t address, const char* data2write, uint16_t data2writeLenght);

	//read / write function examples:
	uint8_t readBtName();
	bool 	writeBtName(const char* name);


protected:
	#define receiveDataArry_lenght 	25							//max command lenght acc. to OVC3860_RevE_PSKeys_Setting_v1.2.pdf
	uint8_t receiveDataArry [receiveDataArry_lenght];			//array to store receiveed from OVC informations




private:
	void _cleanReceiveDataArray();
	void _cleanCommandDataArray();
	bool buildReadCommand(uint8_t type, uint16_t address, uint16_t data2readLenght);
	bool buildWriteCommand(uint8_t type, uint16_t address, const void* data2write, uint16_t data2writeLenght);
	void receiveRawData(uint16_t Size, uint32_t Timeout= HAL_MAX_DELAY);
	bool sendRawData(uint8_t * pAddress, uint16_t Size);

	uint8_t commandDataArry [receiveDataArry_lenght];			//array to store command to be send to OVC
	struct {
		uint8_t	const read 				= 0x1;
		uint8_t const readACK 			= 0x2;
		uint8_t const wrtite			= 0x3;
		uint8_t const wrtiteACK			= 0x4;
		uint8_t	const quitConfigMode	= 0x5;	//rest is 0x0 so command is 0x50, 0x00, 0x00, 0x00
		uint8_t	const quitConfigModeACK	= 0x6;	//rest is 0x0 so responde is 0x60, 0x00, 0x00, 0x00
	} const CommandType;						//struct with command header informations which determines command type
};

#endif /* OVC3860PSKEY_H_ */
