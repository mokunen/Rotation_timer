/**
 * @example CompText.ino
 *
 * @par How to Use
 * Show how to use API of class NexText.
 *
 * @author  Wu Pengfei (email:<pengfei.wu@itead.cc>)
 * @date    2015/7/10
 * @copyright
 * Copyright (C) 2014-2015 ITEAD Intelligent Systems Co., Ltd. \n
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 * update log
 * 2021/01/05 update slider kill  ogihara
 * 2021/01/11 Change timer interrupt processing
 * 2021/01/13 Fixed a problem of charging time

 */

#include <TimeLib.h>
#include <Time.h>
#include <DS3232RTC.h>
 //#include "Nextion.h"
#include <MsTimer2.h>
#include <SoftwareSerial.h>
#include "PCF8574.h"
#include <Wire.h>
#include <AT24CX.h>

#include <avr/pgmspace.h>


AT24C32 mem(0x57);


SoftwareSerial nexSerial(3, 4);

#define I2CADDR0 0x20
#define I2CADDR1 0x21

PCF8574 PCF_SW(I2CADDR0);
PCF8574 PCF_SOL(I2CADDR1);

#define MEM_CHECK 0x00					//Memory Check
#define MEM_C1_START_HOUR 0x01			//回路１の開始時間　ｎ時
#define MEM_C1_START_MINUTE 0x02		//回路１の開始時間　ｎ分
#define MEM_C1_CHARGE_TIME 0x03			//回路１の充電時間　ｎ分
#define MEM_C2_START_HOUR 0x04			//回路２の開始時間　ｎ時
#define MEM_C2_START_MINUTE 0x05		//回路２の開始時間　ｎ分
#define MEM_C2_CHARGE_TIME 0x06			//回路２の充電時間　ｎ分			
#define MEM_C3_START_HOUR 0x07			//回路３の開始時間　ｎ時
#define MEM_C3_START_MINUTE 0x08		//回路３の開始時間　ｎ分
#define MEM_C3_CHARGE_TIME 0x09			//回路３の充電時間　ｎ分
#define MEM_C4_START_HOUR 0x0a			//回路４の開始時間　ｎ時
#define MEM_C4_START_MINUTE 0x0b		//回路４の開始時間　ｎ分
#define MEM_C4_CHARGE_TIME 0x0c			//回路４の充電時間　ｎ分
#define MEM_WEEK_MON 0x0d				//月曜日の充電許可
#define MEM_WEEK_TUE 0x0e				//火曜日の充電許可
#define MEM_WEEK_WED 0x0f				//水曜日の充電許可
#define MEM_WEEK_THU 0x10				//木曜日の充電許可
#define MEM_WEEK_FRI 0x11				//金曜日の充電許可
#define MEM_WEEK_STA 0x12				//土曜日の充電許可
#define MEM_WEEK_SUN 0x13				//日曜日の充電許可

#define INIT_CHECK 0x5a
#define INIT_C1_START_HOUR 0
#define INIT_C1_START_MINUTE 0
#define INIT_C1_CHARGE_TIME 120
#define INIT_C2_START_HOUR 2
#define INIT_C2_START_MINUTE 15
#define INIT_C2_CHARGE_TIME 120
#define INIT_C3_START_HOUR 4
#define INIT_C3_START_MINUTE 30
#define INIT_C3_CHARGE_TIME 120
#define INIT_C4_START_HOUR 6
#define INIT_C4_START_MINUTE 45
#define INIT_C4_CHARGE_TIME 120
#define INIT_WEEK_MON 1
#define INIT_WEEK_TUE 1
#define INIT_WEEK_WED 1
#define INIT_WEEK_THU 1
#define INIT_WEEK_FRI 1
#define INIT_WEEK_STA 1
#define INIT_WEEK_SUN 1

#define WEEK_MON 2
#define WEEK_TUE 3
#define WEEK_WED 4
#define WEEK_THU 5
#define WEEK_FRI 6
#define WEEK_STA 7
#define WEEK_SUN 1

#define NEX_RET_CMD_FINISHED            (0x01)
#define NEX_RET_EVENT_LAUNCHED          (0x88)
#define NEX_RET_EVENT_UPGRADED          (0x89)
#define NEX_RET_EVENT_TOUCH_HEAD            (0x65)     
#define NEX_RET_EVENT_POSITION_HEAD         (0x67)
#define NEX_RET_EVENT_SLEEP_POSITION_HEAD   (0x68)
#define NEX_RET_CURRENT_PAGE_ID_HEAD        (0x66)
#define NEX_RET_STRING_HEAD                 (0x70)
#define NEX_RET_NUMBER_HEAD                 (0x71)
#define NEX_RET_INVALID_CMD             (0x00)
#define NEX_RET_INVALID_COMPONENT_ID    (0x02)
#define NEX_RET_INVALID_PAGE_ID         (0x03)
#define NEX_RET_INVALID_PICTURE_ID      (0x04)
#define NEX_RET_INVALID_FONT_ID         (0x05)
#define NEX_RET_INVALID_BAUD            (0x11)
#define NEX_RET_INVALID_VARIABLE        (0x1A)
#define NEX_RET_INVALID_OPERATION       (0x1B)

#define START_PAGE				0x00
#define PAGE_MAIN				0x01
#define PAGE_SETUP				0x02
#define PAGE_TIMER_SETUP		0x03
#define PAGE_WEEK_SETUP			0x04
#define PAGE_DATE_SET			0x05
#define PAGE_TIME_SET			0x06
#define PAGE_MANUAL				0x07
#define PAGE_AUTO_1				0x08
#define PAGE_AUTO_2				0x09
#define PAGE_AUTO_3				0x0a
#define PAGE_AUTO_4				0x0b
#define PAGE_CHARGE_SETUP_1		0x0c
#define PAGE_CHARGE_SETUP_2		0x0d
#define PAGE_CHARGE_SETUP_3		0x0e
#define PAGE_CHARGE_SETUP_4		0x0f
#define PAGE_AUTO				0x10
#define PAGE_WARNING			0x11

#define PAGE9 9

#define MTR0_PIN        3           //MOTOR制御
#define MTR1_PIN        4           //MOTOR制御
#define LED1_PIN        6           //LED1の制御。
#define LED2_PIN        7           //LED2の制御。
#define ASW_PIN         2           //EXT SWITCH
#define POW_CONT1_PIN   5
#define POW_CONT2_PIN   8
#define MOTOR_OFF 0
#define MOTOR_ON_OPEN 1
#define MOTOR_ON_OPEN_STOP 2
#define MOTOR_ON_CLOSE 3
#define MOTOR_ON_CLOSE_STOP 4
#define POW_CONTROL_ON 1
#define POW_CONTROL_OFF 0                                                                         
#define MOTOR_SW_ON HIGH
#define MOTOR_SW_OFF LOW

#define RELAY_1         0x01
#define RELAY_2         0x02
#define RELAY_3         0x04
#define RELAY_4         0x08
#define LED_1           0x10
#define LED_2           0x20

#define MAX_CHAGE_TIME 240
#define TIME_BOUNDARY 24 * 60



uint8_t autoManualMode = 0;			//0はオートモード、1はマニュアルモード


int16_t gYear;
int8_t gMonth;
int8_t gDay;
int8_t gHour;
int8_t gMinute;
int8_t gWeek;
int16_t setYear;                //設定用年レジスタ
int8_t setMonth;               //設定用月レジスタ
int8_t setDay;                 //設定用日レジスタ
int8_t setHour;                //設定用時レジスタ
int8_t setMinute;              //設定用分レジスタ
//int8_t timerFlag = 0;          //timer処理フラグ

int8_t nextSecond = 0;
int8_t checkTimerFlag = 0;

int8_t c1StartTimeHour = 0;
int8_t c1StartTimeMinute = 0;
int16_t c1ChargeTime = 0;
int16_t c1TimerValue = 0;
int8_t c1TimerOnFalg = 0;
int8_t c1TimerStartFalg = 0;
int8_t c1StartTimeHourTemporary = 0;
int8_t c1StartTimeMinuteTemporary = 0;
uint8_t c1TimerOverCounter = 0;

int8_t c2StartTimeHour = 0;
int8_t c2StartTimeMinute = 0;
int16_t c2ChargeTime = 0;
int16_t c2TimerValue = 0;
int8_t c2TimerOnFalg = 0;
int8_t c2TimerStartFalg = 0;
int8_t c2StartTimeHourTemporary = 0;
int8_t c2StartTimeMinuteTemporary = 0;
uint8_t c2TimerOverCounter = 0;

int8_t c3StartTimeHour = 0;
int8_t c3StartTimeMinute = 0;
int16_t c3ChargeTime = 0;
int16_t c3TimerValue = 0;
int8_t c3TimerOnFalg = 0;
int8_t c3TimerStartFalg = 0;
int8_t c3StartTimeHourTemporary = 0;
int8_t c3StartTimeMinuteTemporary = 0;
uint8_t c3TimerOverCounter = 0;

int8_t c4StartTimeHour = 0;
int8_t c4StartTimeMinute = 0;
int16_t c4ChargeTime = 0;
int16_t c4TimerValue = 0;
int8_t c4TimerOnFalg = 0;
int8_t c4TimerStartFalg = 0;
int8_t c4StartTimeHourTemporary = 0;
int8_t c4StartTimeMinuteTemporary = 0;
uint8_t c4TimerOverCounter = 0;

int8_t m_relay1 = 0;
int8_t m_relay2 = 0;
int8_t m_relay3 = 0;
int8_t m_relay4 = 0;

int8_t slider1ChgFlag = 0;
int8_t slider2ChgFlag = 0;
int8_t slider3ChgFlag = 0;
int8_t slider4ChgFlag = 0;





char gbuffer[8] = { 0 };

uint16_t ptoptReg[8] = { 0x0000 };
uint32_t ptopTotal;
uint32_t ainSum;
uint8_t moveCount = 0;
uint32_t calCurrent = 0;

int pageNum = 0;

const uint8_t timer_table[30] PROGMEM = { 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14 };
const int8_t timerMinutes[4] PROGMEM = { 0,15,30,45 };
//const int16_t timer_correction[25] PROGMEM = { 0, 15,30,45,60,75,90,105,120,135,150.165,180,195,210,225,240,255,270,285,300,315,330,345,360 };

uint8_t mem_buffer[20] = { 0x5a, 0x00, 0x00, 0x78, 0x02, 0x00, 0x78, 0x04, 0x00, 0x78, 0x06, 0x00, 0x78, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 };					//設定用メモリバッファ

//int8_t timer_table[30] = { 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14 };

//int8_t timerMinutes[4] = { 0,15,30,45 };

int8_t week_flag[8];	//SUN->SAT 0:non

//int8_t timer_correction[25] = { 0, 15,30,45,60,75,90,105,120,135,150.165,180,195,210,225,240,255,270,285,300,315,330,345,360 };


//--------------------------
// LED 1 control
//--------------------------
void led_1_cont(int on_off)
{
    uint8_t led_data;
    led_data = PCF_SOL.read8();


    if (on_off == 1)
    {
        led_data |= LED_1;
        PCF_SOL.write8(led_data);      //LED1 ON
    }
    else if (on_off == 0)
    {
        led_data &= ~LED_1;
        PCF_SOL.write8(led_data);      //LED1 OFF
    }

}
//--------------------------
// LED 2 control
//--------------------------
void led_2_cont(int on_off)
{
    uint8_t led_data;
    led_data = PCF_SOL.read8();


    if (on_off == 1)
    {
        led_data |= LED_2;
        PCF_SOL.write8(led_data);      //LED2 ON
    }
    else if (on_off == 0)
    {
        led_data &= ~LED_2;
        PCF_SOL.write8(led_data);      //LED2 OFF
    }

}



//--------------------------
// Relay 1 control
//--------------------------
void relay_1_cont(int on_off)
{
    uint8_t sol_data;
    sol_data = PCF_SOL.read8();

    
    if (on_off == 1)
    {
        sol_data |= RELAY_1;
        PCF_SOL.write8(sol_data);      //リレー1 ON
        //Serial.println("RELAY1ON");
    }
    else if (on_off == 0)
    {
        sol_data &= ~RELAY_1;
        PCF_SOL.write8(sol_data);      //リレー1 OFF
        //Serial.println("RELAY1OFF");
    }

}

//--------------------------
// Relay 2 control
//--------------------------
void relay_2_cont(int on_off)
{
    uint8_t sol_data;
    sol_data = PCF_SOL.read8();


    if (on_off == 1)
    {
        sol_data |= RELAY_2;
        PCF_SOL.write8(sol_data);      //リレー2 ON
        //Serial.println("RELAY2ON");
    }
    else if (on_off == 0)
    {
        sol_data &= ~RELAY_2;
        PCF_SOL.write8(sol_data);      //リレー2 OFF
        //Serial.println("RELAY2OFF");
    }

}
//--------------------------
// Relay 3 control
//--------------------------
void relay_3_cont(int on_off)
{
    uint8_t sol_data;
    sol_data = PCF_SOL.read8();


    if (on_off == 1)
    {
        sol_data |= RELAY_3;
        PCF_SOL.write8(sol_data);      //リレー3 ON
		//Serial.println("RELAY3ON");

    }
    else if (on_off == 0)
    {
        sol_data &= ~RELAY_3;
        PCF_SOL.write8(sol_data);      //リレー3 OFF
		//Serial.println("RELAY3OFF");
    }

}
//--------------------------
// Relay 4 control
//--------------------------
void relay_4_cont(int on_off)
{
    uint8_t sol_data;
    sol_data = PCF_SOL.read8();


    if (on_off == 1)
    {
        sol_data |= RELAY_4;
        PCF_SOL.write8(sol_data);      //リレー4 ON
		//Serial.println("RELAY4ON");
    }
    else if (on_off == 0)
    {
        sol_data &= ~RELAY_4;
        PCF_SOL.write8(sol_data);      //リレー4 OFF
		//Serial.println("RELAY4OFF");
    }

}


/*
 * Receive string data.
 *
 * @param buffer - save string data.
 * @param len - string buffer length.
 * @param timeout - set timeout time.
 *
 * @return the length of string buffer.
 *
 */
uint16_t recvRetString(char* buffer, uint16_t len, uint32_t timeout)
{
    uint16_t ret = 0;
    bool str_start_flag = false;
    uint8_t cnt_0xff = 0;
    String temp = String("");
    uint8_t c = 0;
    long start;

    if (!buffer || len == 0)
    {
        goto __return;
    }

    start = millis();
    while (millis() - start <= timeout)
    {
        while (nexSerial.available())
        {
            c = nexSerial.read();
            if (str_start_flag)
            {
                if (0xFF == c)
                {
                    cnt_0xff++;
                    if (cnt_0xff >= 3)
                    {
                        break;
                    }
                }
                else
                {
                    temp += (char)c;
                }
            }
            else if (NEX_RET_STRING_HEAD == c)
            {
                str_start_flag = true;
            }
        }

        if (cnt_0xff >= 3)
        {
            break;
        }
    }

    ret = temp.length();
    ret = ret > len ? len : ret;
    strncpy(buffer, temp.c_str(), ret);

__return:

    //Serial.print("recvRetString[");
    //Serial.print(temp.length());
    //Serial.print(",");
    //Serial.print(temp);
    //Serial.println("]");

    return ret;
}


/*
 * Command is executed successfully.
 *
 * @param timeout - set timeout time.
 *
 * @retval true - success.
 * @retval false - failed.
 *
 */
bool recvRetCommandFinished(uint32_t timeout)
{
    bool ret = false;
    uint8_t temp[4] = { 0 };

    nexSerial.setTimeout(timeout);
    if (sizeof(temp) != nexSerial.readBytes((char*)temp, sizeof(temp)))
    {
        ret = false;
    }

    if (temp[0] == NEX_RET_CMD_FINISHED
        && temp[1] == 0xFF
        && temp[2] == 0xFF
        && temp[3] == 0xFF
        )
    {
        ret = true;
    }

    if (ret)
    {
        //Serial.println("recvRetCommandFinished ok");
    }
    else
    {
        //Serial.println("recvRetCommandFinished err");
    }

    return ret;
}
//---------------------------------------------------------------------
//コマンド送信
//---------------------------------------------------------------------
void sendCommand(const char* cmd)
{
    while (nexSerial.available())
    {
        nexSerial.read();
    }

    nexSerial.print(cmd);
    nexSerial.write(0xFF);
    nexSerial.write(0xFF);
    nexSerial.write(0xFF);
}
//--------------------------
//Page制御
//--------------------------
bool goto_page(int8_t pageNumber)
{
	char buf[10] = { 0 };
	String cmd;

	pageNum = (int)pageNumber;
	utoa(pageNumber, buf, 10);
	cmd += "page ";
	cmd += buf;

	sendCommand(cmd.c_str());
	return recvRetCommandFinished(100);

}

//---------------------------------------------------------------------
//テキスト送信関数 "t0","55"
//---------------------------------------------------------------------
bool setText(const char* getObjName, const char* buffer)
{
    String cmd;
    cmd += getObjName;
    cmd += ".txt=\"";
    cmd += buffer;
    cmd += "\"";
    sendCommand(cmd.c_str());
    return recvRetCommandFinished(100);
}
bool setPic(const char* getObjName, uint32_t number)
{
	char buf[10] = { 0 };
	String cmd;

	utoa(number, buf, 10);
	cmd += getObjName;
	cmd += ".pic=";
	cmd += buf;

	sendCommand(cmd.c_str());
	return recvRetCommandFinished(100);
}

//---------------------------------------------------------------------
//スライダー先頭位置を送信
//---------------------------------------------------------------------
bool setValue(const char* getObjName, uint32_t number)
{
    char buf[10] = { 0 };
    String cmd;

    utoa(number, buf, 10);
    cmd += getObjName;
    cmd += ".val=";
    cmd += buf;

    sendCommand(cmd.c_str());
    return recvRetCommandFinished(100);
}
bool Set_pointer_thickness_wid(const char* getObjName, uint32_t number)
{
    char buf[10] = { 0 };
    String cmd;

    utoa(number, buf, 10);
    cmd += getObjName;
    cmd += ".wid=";
    cmd += buf;
    sendCommand(cmd.c_str());

    cmd = "";
    cmd += "ref ";
    cmd += getObjName;
    sendCommand(cmd.c_str());
    return recvRetCommandFinished(100);
}
bool setMaxval(const char* getObjName, uint32_t number)
{
    char buf[10] = { 0 };
    String cmd;

    utoa(number, buf, 10);
    cmd += getObjName;
    cmd += ".maxval=";
    cmd += buf;
    sendCommand(cmd.c_str());

    cmd = "";
    cmd += "ref ";
    cmd += getObjName;
    sendCommand(cmd.c_str());
    return recvRetCommandFinished(100);
}

bool Set_font_color_pco(const char* getObjName, uint32_t number)
{
	char buf[10] = { 0 };
	String cmd;

	utoa(number, buf, 10);
	cmd += getObjName;
	cmd += ".pco=";
	cmd += buf;
	sendCommand(cmd.c_str());

	cmd = "";
	cmd += "ref ";
	cmd += getObjName;
	sendCommand(cmd.c_str());
	return recvRetCommandFinished(100);
}


bool recvRetNumber(uint32_t* number, uint32_t timeout)
{
    bool ret = false;
    uint8_t temp[8] = { 0 };

    if (!number)
    {
        goto __return;
    }

    nexSerial.setTimeout(timeout);
    if (sizeof(temp) != nexSerial.readBytes((char*)temp, sizeof(temp)))
    {
        goto __return;
    }

    if (temp[0] == NEX_RET_NUMBER_HEAD
        && temp[5] == 0xFF
        && temp[6] == 0xFF
        && temp[7] == 0xFF
        )
    {
        *number = ((uint32_t)temp[4] << 24) | ((uint32_t)temp[3] << 16) | (temp[2] << 8) | (temp[1]);
        ret = true;
    }

__return:

    if (ret)
    {
        //Serial.print("recvRetNumber :");
        //Serial.println(*number);
    }
    else
    {
        //Serial.println("recvRetNumber err");
    }

    return ret;
}

uint16_t getText(const char* getObjName , char* buffer , uint16_t len)
{
    String cmd;
    cmd += "get ";
    cmd += getObjName;
    cmd += ".txt";
    //Serial.print("cmd=");
    //Serial.println(cmd);
    sendCommand(cmd.c_str());
    return recvRetString(buffer, len,100);
}

bool getValue(const char* getObjName, uint32_t* number)
{
    String cmd = String("get ");
    cmd += getObjName;
    cmd += ".val";
    sendCommand(cmd.c_str());
    return recvRetNumber(number, 100);
}

//---------------------------------------------------------------------
//時計データの取り出し
//---------------------------------------------------------------------
void getClock()
{
    gYear = (int16_t)year();
    gMonth = (int8_t)month();
    gDay = (int8_t)day();
    gHour = (int8_t)hour();
    gMinute = (int8_t)minute();
	gWeek = (int8_t)weekday();

}
//---------------------------------------------------------------------
// page 0 時計表示処理
//---------------------------------------------------------------------
void page0DigitalClockDisplay()
{
    // digital clock display of the time
    //dbSerial.print(hour());
    //printDigits(minute());
    //printDigits(second());
	//week "Sun"=1,"Mon"=2,"Tue"=3,"Wed"=4,"Thu"=5,"Fri"=6,"Sat"=7

    //getClock();

	//Serial.print(gYear);
	//Serial.print(' ');
	//Serial.print(gMonth);
	//Serial.print(' ');
	//Serial.print(gDay);
	//Serial.print(' ');
    //Serial.print(gHour);
    //Serial.print(':');
    //Serial.print(gMinute);
    //Serial.print(' ');
    //Serial.print(gWeek);
    //Serial.println();


    sprintf(gbuffer, "%02d", gMonth);
    setText("t0", gbuffer);

    sprintf(gbuffer, "%02d", gDay);
    setText("t2", gbuffer);

    sprintf(gbuffer, "%02d", gHour);
    setText("t3", gbuffer);

    sprintf(gbuffer, "%02d", gMinute);
    setText("t5", gbuffer);

	setValue("n0", (int)calCurrent);

	//回路１
	if (m_relay1 == 1)
	{
		setPic("p3", 38);			//オン表示
	}
	else
	{
		setPic("p3", 57);			//オフ表示
	}
	//回路２
	if (m_relay2 == 1)
	{
		setPic("p4", 38);			//オン表示
	}
	else
	{
		setPic("p4", 57);			//オフ表示
	}
	//回路３
	if (m_relay3 == 1)
	{
		setPic("p7", 38);			//オン表示
	}
	else
	{
		setPic("p7", 57);			//オフ表示
	}
	//回路４
	if (m_relay4 == 1)
	{
		setPic("p8", 38);			//オン表示
	}
	else
	{
		setPic("p8", 57);			//オフ表示
	}



}
//---------------------------------------------------------------------
// page 9 時計表示処理
//---------------------------------------------------------------------
void page9DigitalClockDisplay()
{
    // digital clock display of the time
    //dbSerial.print(hour());
    //printDigits(minute());
    //printDigits(second());

    //getClock();


    //Serial.print(gHour);
    //Serial.print(':');
    //Serial.print(gMinute);
    //Serial.print(' ');
    //Serial.print(gDay);
    //Serial.print(' ');
    //Serial.print(gMonth);
    //Serial.print(' ');
    //Serial.print(gYear);
    //Serial.print(" ");
	//Serial.print("A[");
	//Serial.print(calCurrent);
	//Serial.println("]");


    sprintf(gbuffer, "%02d", gMonth);
    setText("t0", gbuffer);

    sprintf(gbuffer, "%02d", gDay);
    setText("t2", gbuffer);

    sprintf(gbuffer, "%02d", gHour);
    setText("t3", gbuffer);

	sprintf(gbuffer, "%02d", gMinute);
	setText("t5", gbuffer);
	
	
}

//----------------------------------------------------------------------
//日付設定の為の日付データの取り出し
//----------------------------------------------------------------------
void SetDateTime()
{
    //getClock();
    setYear = (int16_t)gYear;
    setMonth = (int8_t)gMonth;
    setDay = (int8_t)gDay;
	setHour = (int8_t)gHour;
	setMinute = (int8_t)gMinute;

    Serial.println("Set Date !!");
}
//----------------------------------------------------------------------
//日付設定の為の日付データの取り出し
//----------------------------------------------------------------------
void SetTimeMain()
{
    //getClock();

    setHour = (int8_t)gHour;
    setMinute = (int8_t)gMinute;
    Serial.println("Set Time !!");
}
//----------------------------------------------------------------------
//Year button
//----------------------------------------------------------------------
void upYearCallback(void* ptr)
{
    Serial.println("Up Year !!");
    setYear++;
    sprintf(gbuffer, "%d", setYear);
    //setText(buffer);
}
void downYearCallback(void* ptr)
{
    Serial.println("Down Year !!");
    setYear--;
    sprintf(gbuffer, "%d", setYear);
    //setText(buffer);
}
//---------------------------------------------
//Month Button
//---------------------------------------------
void upMonthCallback(void* ptr)
{
    Serial.println("Up Month !!");
    setMonth++;
    if (setMonth >= 12) setMonth = 12;
    //memset(gbuffer, 0, sizeof(gbuffer));
    //itoa(setMonth, gbuffer, 10);
    //setText(buffer);
}
void downMonthCallback(void* ptr)
{
    Serial.println("Down Month !!");
    setMonth--;
    if (setMonth <= 1) setMonth = 1;
    //memset(buffer, 0, sizeof(buffer));
    //itoa(setMonth, buffer, 10);
    //setText(buffer);
}
//---------------------------------------------
//Day Button
//---------------------------------------------
void upDayCallback(void* ptr)
{
    Serial.println("Up Day !!");
    setDay++;
    if (setDay >= 31) setDay = 31;
    //memset(buffer, 0, sizeof(buffer));
    //itoa(setDay, buffer, 10);
    //setText(buffer);
}
void downDayCallback(void* ptr)
{
    Serial.println("Down Day !!");

    setDay--;
    if (setDay <= 1) setDay = 1;
    //memset(buffer, 0, sizeof(buffer));
    //itoa(setDay, buffer, 10);
    //setText(buffer);
}


void printDigits(int digits)
{
    // utility function for digital clock display: prints preceding colon and leading 0
    Serial.print(':');
    if (digits < 10)
        Serial.print('0');
    Serial.print(digits);
}


void timerCheck()
{
	checkTimerFlag = 1;
    
}
//----------------------------------------------------------------------------------------------------
//タイマー１設定での現在のタイマー値の表示
//----------------------------------------------------------------------------------------------------
void dispCurrentTimer1()
{
	if ((c1ChargeTime % 15) != 0)
	{
		c1ChargeTime = 60;
	}
	if (c1ChargeTime > MAX_CHAGE_TIME)
	{
		c1ChargeTime = MAX_CHAGE_TIME;
	}
    sprintf(gbuffer, "%d", c1StartTimeHour);
    setText("t0", gbuffer);
    sprintf(gbuffer, "%d", c1StartTimeMinute);
    setText("t1", gbuffer);
    sprintf(gbuffer, "%d", c1ChargeTime);
    setText("t2", gbuffer);

}
//----------------------------------------------------------------------------------------------------
//タイマー2設定での現在のタイマー値の表示
//----------------------------------------------------------------------------------------------------
void dispCurrentTimer2()
{
	if ((c2ChargeTime % 15) != 0)
	{
		c2ChargeTime = 60;
	}
	if (c2ChargeTime > MAX_CHAGE_TIME)
	{
		c2ChargeTime = MAX_CHAGE_TIME;
	}
    sprintf(gbuffer, "%d", c2StartTimeHour);
    setText("t0", gbuffer);
    sprintf(gbuffer, "%d", c2StartTimeMinute);
    setText("t1", gbuffer);
    sprintf(gbuffer, "%d", c2ChargeTime);
    setText("t2", gbuffer);

}
//----------------------------------------------------------------------------------------------------
//タイマー3設定での現在のタイマー値の表示
//----------------------------------------------------------------------------------------------------
void dispCurrentTimer3()
{
	if ((c3ChargeTime % 15) != 0)
	{
		c3ChargeTime = 60;
	}
	if (c3ChargeTime > MAX_CHAGE_TIME)
	{
		c3ChargeTime = MAX_CHAGE_TIME;
	}
	sprintf(gbuffer, "%d", c3StartTimeHour);
	setText("t0", gbuffer);
	sprintf(gbuffer, "%d", c3StartTimeMinute);
	setText("t1", gbuffer);
	sprintf(gbuffer, "%d", c3ChargeTime);
	setText("t2", gbuffer);

}
//----------------------------------------------------------------------------------------------------
//タイマー4設定での現在のタイマー値の表示
//----------------------------------------------------------------------------------------------------
void dispCurrentTimer4()
{
	if ((c4ChargeTime % 15) != 0)
	{
		c4ChargeTime = 60;
	}
	if (c4ChargeTime > MAX_CHAGE_TIME)
	{
		c4ChargeTime = MAX_CHAGE_TIME;
	}
	sprintf(gbuffer, "%d", c4StartTimeHour);
	setText("t0", gbuffer);
	sprintf(gbuffer, "%d", c4StartTimeMinute);
	setText("t1", gbuffer);
	sprintf(gbuffer, "%d", c4ChargeTime);
	setText("t2", gbuffer);

}
//----------------------------------------------------------------------------------------------------
//タイマー1設定処理
//----------------------------------------------------------------------------------------------------
void timer1Set(uint8_t cid)
{
    //---------回路１開始時刻設定
    if (cid == 0x10)                 //回路１の時間をインクリメント +
    {
        c1StartTimeHour++;
        if (c1StartTimeHour >= 25) c1StartTimeHour = 24;
        sprintf(gbuffer, "%d", c1StartTimeHour);
        setText("t0", gbuffer);
    }
    else if (cid == 0x03)                  //回路１の時間をデクリメント
    {
        c1StartTimeHour--;
        if (c1StartTimeHour < 0) c1StartTimeHour = 0;
        sprintf(gbuffer, "%d", c1StartTimeHour);
        setText("t0", gbuffer);
    }
    if (cid == 0x04)                 //回路１の分を１５分単位で増やす
    {
        c1StartTimeMinute +=15;
        if (c1StartTimeMinute >= 60) c1StartTimeMinute = 0;
        sprintf(gbuffer, "%d", c1StartTimeMinute);
        setText("t1", gbuffer);
    }
    else if (cid == 0x05)                  //回路１の分を１５分単位で減らす
    {
        c1StartTimeMinute -=15;
        if (c1StartTimeMinute < 0) c1StartTimeMinute = 0;
        sprintf(gbuffer, "%d", c1StartTimeMinute);
        setText("t1", gbuffer);
    }
    //---------回路１充電時間設定（分）
    if (cid == 0x0a)                 //充電時間 +
    {
		c1ChargeTime +=15;
        if (c1ChargeTime >= MAX_CHAGE_TIME) c1ChargeTime = MAX_CHAGE_TIME;
        sprintf(gbuffer, "%d", c1ChargeTime);
        setText("t2", gbuffer);
    }
    else if (cid == 0x0b)                  //充電時間 -
    {
		c1ChargeTime -= 15;
        if (c1ChargeTime < 0) c1ChargeTime = 0;
        sprintf(gbuffer, "%d", c1ChargeTime);
        setText("t2", gbuffer);
    }



}
//----------------------------------------------------------------------------------------------------
//タイマー2設定処理
//----------------------------------------------------------------------------------------------------
void timer2Set(uint8_t cid)
{
    //---------回路２開始時刻設定
    if (cid == 0x10)                 //回路２の時間をインクリメント +
    {
        c2StartTimeHour++;
        if (c2StartTimeHour >= 25) c2StartTimeHour = 24;
        sprintf(gbuffer, "%d", c2StartTimeHour);
        setText("t0", gbuffer);
    }
    else if (cid == 0x03)                  //回路２の時間をデクリメント -
    {
        c2StartTimeHour--;
        if (c2StartTimeHour < 0) c2StartTimeHour = 0;
        sprintf(gbuffer, "%d", c2StartTimeHour);
        setText("t0", gbuffer);
    }
    if (cid == 0x04)                 //回路２の分を１５分単位で増やす
    {
        c2StartTimeMinute += 15;
        if (c2StartTimeMinute >= 60) c2StartTimeMinute = 0;
        sprintf(gbuffer, "%d", c2StartTimeMinute);
        setText("t1", gbuffer);
    }
    else if (cid == 0x05)                  //回路２の分を１５分単位で減らす
    {
        c2StartTimeMinute -=15;
        if (c2StartTimeMinute < 0) c2StartTimeMinute = 0;
        sprintf(gbuffer, "%d", c2StartTimeMinute);
        setText("t1", gbuffer);
    }
    //---------回路２充電時間設定（分）
    if (cid == 0x0a)                 //c1EndTimeHour +
    {
		c2ChargeTime +=15;
        if (c2ChargeTime >= MAX_CHAGE_TIME) c2ChargeTime = MAX_CHAGE_TIME;
        sprintf(gbuffer, "%d", c2ChargeTime);
        setText("t2", gbuffer);
    }
    else if (cid == 0x0b)                  //c1StartTimeHour -
    {
		c2ChargeTime -=15;
        if (c2ChargeTime < 0) c2ChargeTime = 0;
        sprintf(gbuffer, "%d", c2ChargeTime);
        setText("t2", gbuffer);
    }
 

}

void timer3Set(uint8_t cid)
{
	//---------回路３開始時刻設定
	if (cid == 0x10)                 //回路３の時間をインクリメント +
	{
		c3StartTimeHour++;
		if (c3StartTimeHour >= 25) c3StartTimeHour = 24;
		sprintf(gbuffer, "%d", c3StartTimeHour);
		setText("t0", gbuffer);
	}
	else if (cid == 0x03)                  //回路３の時間をデクリメント
	{
		c3StartTimeHour--;
		if (c3StartTimeHour < 0) c3StartTimeHour = 0;
		sprintf(gbuffer, "%d", c3StartTimeHour);
		setText("t0", gbuffer);
	}
	if (cid == 0x04)                 //回路３の分を１５分単位で増やす
	{
		c3StartTimeMinute += 15;
		if (c3StartTimeMinute >= 60) c3StartTimeMinute = 0;
		sprintf(gbuffer, "%d", c3StartTimeMinute);
		setText("t1", gbuffer);
	}
	else if (cid == 0x05)                  //回路３の分を１５分単位で減らす
	{
		c3StartTimeMinute -= 15;
		if (c3StartTimeMinute < 0) c3StartTimeMinute = 0;
		sprintf(gbuffer, "%d", c3StartTimeMinute);
		setText("t1", gbuffer);
	}
	//---------回路３充電時間設定（分）
	if (cid == 0x0a)                 //回路３の充電時間 +
	{
		c3ChargeTime += 15;
		if (c3ChargeTime >= MAX_CHAGE_TIME) c3ChargeTime = MAX_CHAGE_TIME;
		sprintf(gbuffer, "%d", c3ChargeTime);
		setText("t2", gbuffer);
	}
	else if (cid == 0x0b)                  //回路３の充電時間 -
	{
		c3ChargeTime -= 15;
		if (c3ChargeTime < 0) c3ChargeTime = 0;
		sprintf(gbuffer, "%d", c3ChargeTime);
		setText("t2", gbuffer);
	}
}

	void timer4Set(uint8_t cid)
	{
		//---------回路４開始時刻設定
		if (cid == 0x10)                 //回路４の時間をインクリメント +
		{
			c4StartTimeHour++;
			if (c4StartTimeHour >= 25) c4StartTimeHour = 24;
			sprintf(gbuffer, "%d", c4StartTimeHour);
			setText("t0", gbuffer);
		}
		else if (cid == 0x03)                  //回路４の時間をデクリメント
		{
			c4StartTimeHour--;
			if (c4StartTimeHour < 0) c4StartTimeHour = 0;
			sprintf(gbuffer, "%d", c4StartTimeHour);
			setText("t0", gbuffer);
		}
		if (cid == 0x04)                 //回路４の分を１５分単位で増やす
		{
			c4StartTimeMinute += 15;
			if (c4StartTimeMinute >= 60) c4StartTimeMinute = 0;
			sprintf(gbuffer, "%d", c4StartTimeMinute);
			setText("t1", gbuffer);
		}
		else if (cid == 0x05)                  //回路４の分を１５分単位で減らす
		{
			c4StartTimeMinute -= 15;
			if (c4StartTimeMinute < 0) c4StartTimeMinute = 0;
			sprintf(gbuffer, "%d", c4StartTimeMinute);
			setText("t1", gbuffer);
		}
		//---------回路４充電時間設定（分）
		if (cid == 0x0a)                 //回路４充電時間 +
		{
			c4ChargeTime += 15;
			if (c4ChargeTime >= MAX_CHAGE_TIME) c4ChargeTime = MAX_CHAGE_TIME;
			sprintf(gbuffer, "%d", c4ChargeTime);
			setText("t2", gbuffer);
		}
		else if (cid == 0x0b)                  //回路４充電時間 -
		{
			c4ChargeTime -= 15;
			if (c4ChargeTime < 0) c4ChargeTime = 0;
			sprintf(gbuffer, "%d", c4ChargeTime);
			setText("t2", gbuffer);
		}



	}




//----------------------------------------------------------------------------------------------------
//日付設定での現在の日付の表示 PAGE_MAIN
//----------------------------------------------------------------------------------------------------
void dispCurrentDateTime()
{
	//sprintf(gbuffer, "%d", setYear);
	//setText("t0", gbuffer);
	sprintf(gbuffer, "%d", setMonth);
	setText("t0", gbuffer);
	sprintf(gbuffer, "%d", setDay);
	setText("t2", gbuffer);
	sprintf(gbuffer, "%d", setHour);
	setText("t3", gbuffer);
	sprintf(gbuffer, "%d", setMinute);
	setText("t5", gbuffer);

}
//----------------------------------------------------------------------------------------------------
//日付設定での現在の日付の表示
//----------------------------------------------------------------------------------------------------
void dispCurrentDate()
{
    sprintf(gbuffer, "%d", setYear);
    setText("t0", gbuffer);
    sprintf(gbuffer, "%d", setMonth);
    setText("t1", gbuffer);
    sprintf(gbuffer, "%d", setDay);
    setText("t2", gbuffer);

}
//----------------------------------------------------------------------------------------------------
//時刻設定での現在の日付の表示
//----------------------------------------------------------------------------------------------------
void dispCurrentTime()
{
    sprintf(gbuffer, "%d", setHour);
    setText("t0", gbuffer);
    sprintf(gbuffer, "%d", setMinute);
    setText("t1", gbuffer);

}
//----------------------------------------------------------------------------------------------------
//日付設定
//----------------------------------------------------------------------------------------------------
void page6Set(uint8_t cid)
{
    if (cid == 0x0f)                      //Yaer +
    {
        setYear++;
        sprintf(gbuffer, "%d", setYear);
        setText("t0", gbuffer);

    }
    else if (cid == 0x04)                 //Yaer -
    {
        setYear--;
        if (setYear <= 1970) setYear = 1970;
        sprintf(gbuffer, "%d", setYear);
        setText("t0", gbuffer);

    }
    else if (cid == 0x05)                 //Month +
    {
        setMonth++;
        if (setMonth >= 13) setMonth = 12;
        sprintf(gbuffer, "%d", setMonth);
        setText("t1", gbuffer);
    }
    else if (cid == 0x06)                  //Month -
    {
        setMonth--;
        if (setMonth <= 0) setMonth = 1;
        sprintf(gbuffer, "%d", setMonth);
        setText("t1", gbuffer);
    }
    else if (cid == 0x0b)                  //Day +
    {
        setDay++;
        if (setDay >= 32) setDay = 31;
        sprintf(gbuffer, "%d", setDay);
        setText("t2", gbuffer);

    }
    else if (cid == 0x0c)                  //Day -
    {
        setDay--;
        if (setDay <= 0) setDay = 1;
        sprintf(gbuffer, "%d", setDay);
        setText("t2", gbuffer);

    }

}
//----------------------------------------------------------------------------------------------------
//時刻設定
//----------------------------------------------------------------------------------------------------
void page7Set(uint8_t cid)
{
    if (cid == 0x03)                      //Yaer +
    {
        setHour++;
        if (setHour >= 25) setMonth = 24;
        sprintf(gbuffer, "%d", setHour);
        setText("t0", gbuffer);

    }
    else if (cid == 0x04)                 //Yaer -
    {
        setHour--;
        if (setHour < 0) setHour = 0;
        sprintf(gbuffer, "%d", setHour);
        setText("t0", gbuffer);

    }
    else if (cid == 0x05)                 //Month +
    {
        setMinute++;
        if (setMinute >= 60) setMinute = 59;
        sprintf(gbuffer, "%d", setMinute);
        setText("t1", gbuffer);
    }
    else if (cid == 0x06)                  //Month -
    {
        setMinute--;
        if (setMinute < 0) setMinute = 0;
        sprintf(gbuffer, "%d", setMinute);
        setText("t1", gbuffer);
    }

}


//----------------------------------------------------------------------------------------------------
//自動画面表示1
//----------------------------------------------------------------------------------------------------
void setDispAuto1()
{
    int StartTimeSlider;
	int StartTimeSlider_a;
    int ValueTimeSlider;
    int EndTimeSlider;

	int xxx = 0;
	int yyy = 0;
	int barStart;

    setMaxval("h0", 300);
	if (c1StartTimeHour < 12)
	{
		barStart = c1StartTimeHour + 15;
	}
	else
	{
		barStart = c1StartTimeHour - 9;
	}
	if (barStart < 15)
	{
		Serial.println("XXXS");
		xxx = (int)(((150.0 - (float)barStart*10.0) / 150.0)*10);
	}
	else
	{
		Serial.println("YYYS");
		yyy = (int)((((float)barStart*10.0 - 150) / 150) * 10);

	}
	StartTimeSlider = (barStart * 10) + ((c1StartTimeMinute / 60) * 10);
    ValueTimeSlider = (c1ChargeTime / 60) * 10;
	if (ValueTimeSlider < 4)
	{
		ValueTimeSlider = 4;
	}
	StartTimeSlider_a =( StartTimeSlider +(ValueTimeSlider / 2)) - xxx + yyy;
    Set_pointer_thickness_wid("h0", ValueTimeSlider);
    setValue("h0", StartTimeSlider_a);

	//if (time_chaeck(1) == 1)
	//{
	//	Set_font_color_pco("h0", 63488);
	//	Serial.println("SET RED");
	//}
	//else
	//{
	//	Set_font_color_pco("h0", 1055);
	//	Serial.println("SET BLUE");
	//}

	//Serial.print("SL0[");
	//Serial.print(c1StartTimeHour);
	//Serial.print("]");
	//Serial.print("CHG[");
	//Serial.print(c1ChargeTime);
	//Serial.print("]");
	//Serial.print("BAR[");
	//Serial.print(barStart);
	//Serial.print("]");
	//Serial.print("SLS[");
	//Serial.print(StartTimeSlider);
	//Serial.print("]");
	//Serial.print("SLV[");
	//Serial.print(ValueTimeSlider);
	//Serial.print("]");	
	//Serial.print("SLS2[");
	//Serial.print(StartTimeSlider_a);
	//Serial.print("]");
	//Serial.print("XXX[");
	//Serial.print(xxx);
	//Serial.println("]");
}
//----------------------------------------------------------------------------------------------------
//自動画面表示2
//----------------------------------------------------------------------------------------------------
void setDispAuto2()
{
	int StartTimeSlider;
	int StartTimeSlider_a;
	int ValueTimeSlider;
	int EndTimeSlider;
	int xxx = 0;
	int yyy = 0;
	int barStart;

	setMaxval("h1", 300);
	if (c2StartTimeHour < 12)
	{
		barStart = c2StartTimeHour + 15;
	}
	else
	{
		barStart = c2StartTimeHour - 9;
	}
	if (barStart < 15)
	{
		Serial.println("XXXS");
		xxx = (int)(((150.0 - (float)barStart*10.0) / 150.0) * 10);
	}
	else
	{
		Serial.println("YYYS");
		yyy = (int)((((float)barStart*10.0 - 150) / 150) * 10);

	}
	StartTimeSlider = (barStart * 10) + ((c2StartTimeMinute / 60) * 10);
	ValueTimeSlider = (c2ChargeTime / 60) * 10;
	if (ValueTimeSlider < 4)
	{
		ValueTimeSlider = 4;
	}
	StartTimeSlider_a = (StartTimeSlider + (ValueTimeSlider / 2)) - xxx + yyy;
	Set_pointer_thickness_wid("h1", ValueTimeSlider);
	setValue("h1", StartTimeSlider_a);
	//if (time_chaeck(2) == 1)
	//{
	//	Set_font_color_pco("h1", 63488);
		//Serial.println("SET RED");
	//}
	//else
	//{
	//	Set_font_color_pco("h1", 1055);
		//Serial.println("SET BLUE");
	//}
	//Serial.print("SL1[");
	//Serial.print(c2StartTimeHour);
	//Serial.print("]");
	//Serial.print("[");
	//Serial.print(barStart);
	//Serial.print("]");
	//Serial.print("[");
	//Serial.print(StartTimeSlider);
	//Serial.print("]");
	//Serial.print("[");
	//Serial.print(ValueTimeSlider);
	//Serial.println("]");
}
//----------------------------------------------------------------------------------------------------
//自動画面表示3
//----------------------------------------------------------------------------------------------------
void setDispAuto3()
{
	int StartTimeSlider;
	int StartTimeSlider_a;
	int ValueTimeSlider;
	int EndTimeSlider;
	int xxx = 0;
	int yyy = 0;
	int barStart;

	setMaxval("h2", 300);
	if (c3StartTimeHour < 12)
	{
		barStart = c3StartTimeHour + 15;
	}
	else
	{
		barStart = c3StartTimeHour - 9;
	}
	if (barStart < 15)
	{
		Serial.println("XXXS");
		xxx = (int)(((150.0 - (float)barStart*10.0) / 150.0) * 10);
	}
	else
	{
		Serial.println("YYYS");
		yyy = (int)((((float)barStart*10.0 - 150) / 150) * 10);

	}
	StartTimeSlider = (barStart * 10) + ((c3StartTimeMinute / 60) * 10);
	ValueTimeSlider = (c3ChargeTime / 60) * 10;
	if (ValueTimeSlider < 4)
	{
		ValueTimeSlider = 4;
	}
	StartTimeSlider_a = (StartTimeSlider + (ValueTimeSlider / 2)) - xxx + yyy;
	Set_pointer_thickness_wid("h2", ValueTimeSlider);
	setValue("h2", StartTimeSlider_a);
	//if (time_chaeck(3) == 1)
	//{
	//	Set_font_color_pco("h2", 63488);
	//	//Serial.println("SET RED");
	//}
	//else
	//{
	//	Set_font_color_pco("h2", 1055);
	//	//Serial.println("SET BLUE");
	//}
	//Serial.print("SL2[");
	//Serial.print(c3StartTimeHour);
	//Serial.print("]");
	//Serial.print("[");
	//Serial.print(barStart);
	//Serial.print("]");
	//Serial.print("[");
	//Serial.print(StartTimeSlider);
	//Serial.print("]");
	//Serial.print("[");
	//Serial.print(ValueTimeSlider);
	//Serial.println("]");
}
//----------------------------------------------------------------------------------------------------
//自動画面表示4
//----------------------------------------------------------------------------------------------------
void setDispAuto4()
{
	int StartTimeSlider;
	int StartTimeSlider_a;
	int ValueTimeSlider;
	int EndTimeSlider;
	int xxx = 0;
	int yyy = 0;
	int barStart;

	setMaxval("h3", 300);
	if (c4StartTimeHour < 12)
	{
		barStart = c4StartTimeHour + 15;
	}
	else
	{
		barStart = c4StartTimeHour - 9;
	}
	if (barStart < 15)
	{
		Serial.println("XXXS");
		xxx = (int)(((150.0 - (float)barStart*10.0) / 150.0) * 10);
	}
	else
	{
		Serial.println("YYYS");
		yyy = (int)((((float)barStart*10.0 - 150) / 150) * 10);

	}
	StartTimeSlider = (barStart * 10) + ((c4StartTimeMinute / 60) * 10);
	ValueTimeSlider = (c4ChargeTime / 60) * 10;
	if (ValueTimeSlider < 4)
	{
		ValueTimeSlider = 4;
	}
	StartTimeSlider_a =( StartTimeSlider + (ValueTimeSlider / 2)) - xxx + yyy;
	//StartTimeSlider_a = (StartTimeSlider + (ValueTimeSlider / 2)) - xxx;
	Set_pointer_thickness_wid("h3", ValueTimeSlider);
	setValue("h3", StartTimeSlider_a);
	//if (time_chaeck(4) == 1)
	//{
	//	Set_font_color_pco("h3", 63488);
	//	//Serial.println("SET RED");
	//}
	//else
	//{
	//	Set_font_color_pco("h3", 1055);
	//	//Serial.println("SET BLUE");
	//}
	//Serial.print("SL3[");
	//Serial.print(c4StartTimeHour);
	//Serial.print("]");
	//Serial.print("CHG[");
	//Serial.print(c1ChargeTime);
	//Serial.print("]");
	//Serial.print("BAR[");
	//Serial.print(barStart);
	//Serial.print("]");
	//Serial.print("SLS[");
	//Serial.print(StartTimeSlider);
	//Serial.print("]");
	//Serial.print("SLV[");
	//Serial.print(ValueTimeSlider);
	//Serial.print("]");
	//Serial.print("SLS2[");
	//Serial.print(StartTimeSlider_a);
	//Serial.print("]");
	//Serial.print("YYY[");
	//Serial.print(yyy);
	//Serial.println("]");
}


//----------------------------------------------------------------------------------------------------
//設定された日付と時刻のデータを書き込む
//----------------------------------------------------------------------------------------------------
void setDateAndTime()
{

    setTime(setHour, setMinute, 00, setDay, setMonth, setYear);
    RTC.set(now());
    if (timeStatus() != timeSet)
        Serial.println("Unable to sync with the RTC");
    else
        Serial.println("RTC has set the system time");
}
//----------------------------------------------------------------------------------------------------
//手動外部リレー制御
//----------------------------------------------------------------------------------------------------
void relayControl(uint8_t cid)
{
    uint32_t dualState1 = 0;
    uint32_t dualState2 = 0;
    uint32_t dualState3 = 0;
    uint32_t dualState4 = 0;


    if ((cid == 0x01) | (cid == 0x02) | (cid == 0x08) | (cid == 0x09))
    {
		getValue("bt0", &dualState1);
		getValue("bt1", &dualState2);
		getValue("bt2", &dualState3);
		getValue("bt3", &dualState4);
		Serial.print("bt0[");
		Serial.print(dualState1);
		Serial.print("]");
		Serial.print("bt1[");
		Serial.print(dualState2);
		Serial.print("]");
		Serial.print("bt2[");
		Serial.print(dualState3);
		Serial.print("]");
		Serial.print("bt3[");
        Serial.print(dualState4);
        Serial.println("]");
        m_relay1 = dualState1;
        m_relay2 = dualState2;
        m_relay3 = dualState3;
        m_relay4 = dualState4;
        if (m_relay1 == 1)
        {
            //digitalWrite(POW_CONT1_PIN, POW_CONTROL_ON);
            relay_1_cont(1);        //RELAY1 ON
        }
        else
        {
            //digitalWrite(POW_CONT1_PIN, POW_CONTROL_OFF);
            relay_1_cont(0);        //RELAY1 OFF
        }

        if (m_relay2 == 1)
        {
            //digitalWrite(POW_CONT2_PIN, POW_CONTROL_ON);
            relay_2_cont(1);        //RELAY2 OFF
        }
        else
        {
            //digitalWrite(POW_CONT2_PIN, POW_CONTROL_OFF);
            relay_2_cont(0);        //RELAY2 OFF
        }

        if (m_relay3 == 1)
        {
            //digitalWrite(POW_CONT2_PIN, POW_CONTROL_ON);
            relay_3_cont(1);        //RELAY3 OFF
        }
        else
        {
            //digitalWrite(POW_CONT2_PIN, POW_CONTROL_OFF);
            relay_3_cont(0);        //RELAY3 OFF
        }

        if (m_relay4 == 1)
        {
            //digitalWrite(POW_CONT2_PIN, POW_CONTROL_ON);
            relay_4_cont(1);        //RELAY4 OFF
        }
        else
        {
            //digitalWrite(POW_CONT2_PIN, POW_CONTROL_OFF);
            relay_4_cont(0);        //RELAY4 OFF
        }
    }
}
//----------------------------------------------------------------------------------------------------
//自動外部リレー制御
//----------------------------------------------------------------------------------------------------
void autoRelayControl()
{

		if (m_relay1 == 1)
		{
			//digitalWrite(POW_CONT1_PIN, POW_CONTROL_ON);
			relay_1_cont(1);        //RELAY1 ON
		}
		else
		{
			//digitalWrite(POW_CONT1_PIN, POW_CONTROL_OFF);
			relay_1_cont(0);        //RELAY1 OFF
		}

		if (m_relay2 == 1)
		{
			//digitalWrite(POW_CONT2_PIN, POW_CONTROL_ON);
			relay_2_cont(1);        //RELAY2 OFF
		}
		else
		{
			//digitalWrite(POW_CONT2_PIN, POW_CONTROL_OFF);
			relay_2_cont(0);        //RELAY2 OFF
		}

		if (m_relay3 == 1)
		{
			//digitalWrite(POW_CONT2_PIN, POW_CONTROL_ON);
			relay_3_cont(1);        //RELAY3 OFF
		}
		else
		{
			//digitalWrite(POW_CONT2_PIN, POW_CONTROL_OFF);
			relay_3_cont(0);        //RELAY3 OFF
		}

		if (m_relay4 == 1)
		{
			//digitalWrite(POW_CONT2_PIN, POW_CONTROL_ON);
			relay_4_cont(1);        //RELAY4 OFF
		}
		else
		{
			//digitalWrite(POW_CONT2_PIN, POW_CONTROL_OFF);
			relay_4_cont(0);        //RELAY4 OFF
		}
	
}
//----------------------------------------------------------------------------------------------------
//LCDから日付データを取り出す
//----------------------------------------------------------------------------------------------------
void getDateLcd()
{
    char dualState1[5] = { 0x00,0x00,0x00,0x00,0x00 };
    char dualState2[3] = { 0x00, 0x00, 0x00 };
    char dualState3[3] = { 0x00,0x00,0x00 };

    getText("t0", &dualState1[0], 4);
    getText("t1", &dualState2[0], 2);
    getText("t2", &dualState3[0], 2);
    Serial.print("Y(t0)[");
    Serial.print(dualState1);
    Serial.print("]");
    Serial.print(" M(t1)[");
    Serial.print(dualState2);
    Serial.print("]");
    Serial.print(" D(t2)[");
    Serial.print(dualState3);
    Serial.println("]");
    setYear = atoi(dualState1);
    setMonth = atoi(dualState2);
    setDay = atoi(dualState3);

}
//----------------------------------------------------------------------------------------------------
//LCDから時間データを取り出す
//----------------------------------------------------------------------------------------------------
void getTimeLcd()
{
    char dualState1[3] = { 0x00,0x00,0x00 };
    char dualState2[3] = { 0x00, 0x00, 0x00 };


    getText("t0", &dualState1[0], 2);
    getText("t1", &dualState2[0], 2);
    Serial.print("H(t0)[");
    Serial.print(dualState1);
    Serial.print("]");
    Serial.print(" M(t1)[");
    Serial.print(dualState2);
    Serial.println("]");
    setHour = atoi(dualState1);
    setMinute = atoi(dualState2);

}
//----------------------------------------------------------------------------------------------------
//LCD画面にリレーの状態を表示する
//----------------------------------------------------------------------------------------------------
void set_relay_state()
{
	Serial.print("SET bt ");
	Serial.print("bt0[");
	Serial.print(m_relay1);
	Serial.print("]");
	Serial.print("bt1[");
	Serial.print(m_relay2);
	Serial.print("]");
	Serial.print("bt2[");
	Serial.print(m_relay3);
	Serial.print("]");
	Serial.print("bt3[");
	Serial.print(m_relay4);
	Serial.println("]");

	//回路１
	if (m_relay1 == 1)
	{
		setValue("bt0", 1);
	}
	else
	{
		setValue("bt0", 0);
	}
	//回路２
	if (m_relay2 == 1)
	{
		setValue("bt1", 1);
	}
	else
	{
		setValue("bt1", 0);
	}
	//回路３
	if (m_relay3 == 1)
	{
		setValue("bt2", 1);
	}
	else
	{
		setValue("bt2", 0);
	}
	//回路４
	if (m_relay4 == 1)
	{
		setValue("bt3", 1);
	}
	else
	{
		setValue("bt3", 0);
	}



}

void check_slider_1()
{
	uint32_t sliderNum = 0;
	float  timef;
	int   timei;
	float xxx = 0;
	float yyy = 0;
	float vvv;
	int zzz = 0;

	slider1ChgFlag = 1;

	getValue("h0", &sliderNum);

	timef = (float)sliderNum / 10.0;
	//timei = (int)timef;

	xxx = ((float)sliderNum / 300) * ((c1ChargeTime /60) *10);		//補正値を求める
	yyy = (sliderNum - (int)xxx) * 10;								//補正後の数値 * 10
	timei = (int)yyy / 100;
	//floor((1580 - floor(1580 / 100)* 100) / 25)
	vvv = floor((yyy - floor(yyy / 100.0) * 100.0) / 25.0);
	zzz = timerMinutes[(int)vvv];
	//c1StartTimeHourTemporary = timer_table[timei];
	c1StartTimeHourTemporary = pgm_read_word(timer_table + timei);
	//c1StartTimeMinuteTemporary = timerMinutes[(int)vvv];
	c1StartTimeMinuteTemporary = pgm_read_word(timerMinutes + (int)vvv);



	//Serial.print("h00[");
	//Serial.print(sliderNum);
	//Serial.print("]");
	//timer_table[timei];
	//Serial.print("TH[");
	//Serial.print(c1StartTimeHourTemporary);
	//Serial.print("]");
	//Serial.print("TM[");
	//Serial.print(c1StartTimeMinuteTemporary);
	//Serial.print("]");
	//Serial.print("XXX[");
	//Serial.print(xxx);
	//Serial.print("]");
	//Serial.print("YYY[");
	//Serial.print(yyy);
	//Serial.print("]");
	//Serial.print("VVV[");
	//Serial.print(vvv);
	//Serial.print("]");
	//Serial.print("ZZZ[");
	//Serial.print(zzz);
	//Serial.println("]");

}

void check_slider_2()
{
	uint32_t sliderNum = 0;
	float  timef;
	int   timei;
	float xxx = 0;
	float yyy = 0;
	float vvv;
	int zzz = 0;

	slider2ChgFlag = 1;

	getValue("h1", &sliderNum);

	timef = (float)sliderNum / 10.0;
	//timei = (int)timef;

	xxx = ((float)sliderNum / 300) * ((c2ChargeTime / 60) * 10);		//補正値を求める
	yyy = (sliderNum - (int)xxx) * 10;								//補正後の数値 * 10
	timei = (int)yyy / 100;
	//floor((1580 - floor(1580 / 100)* 100) / 25)
	vvv = floor((yyy - floor(yyy / 100.0) * 100.0) / 25.0);
	zzz = timerMinutes[(int)vvv];
	c2StartTimeHourTemporary = pgm_read_word(timer_table + timei);
	c2StartTimeMinuteTemporary = pgm_read_word(timerMinutes + (int)vvv);
	//Serial.print("h2[");
	//Serial.print(sliderNum);
	//Serial.print("]");
	//timer_table[timei];
	//Serial.print("TH[");
	//Serial.print(c2StartTimeHourTemporary);
	//Serial.print("]");
	//Serial.print("TM[");
	//Serial.print(c2StartTimeMinuteTemporary);
	//Serial.print("]");
	//Serial.print("XXX[");
	//Serial.print(xxx);
	//Serial.print("]");
	//Serial.print("YYY[");
	//Serial.print(yyy);
	//Serial.print("]");
	//Serial.print("VVV[");
	//Serial.print(vvv);
	//Serial.print("]");
	//Serial.print("ZZZ[");
	//Serial.print(zzz);
	//Serial.println("]");

}
void check_slider_3()
{
	uint32_t sliderNum = 0;
	float  timef;
	int   timei;
	float xxx = 0;
	float yyy = 0;
	float vvv;
	int zzz = 0;

	slider3ChgFlag = 1;

	getValue("h2", &sliderNum);

	timef = (float)sliderNum / 10.0;
	//timei = (int)timef;

	xxx = ((float)sliderNum / 300) * ((c3ChargeTime / 60) * 10);		//補正値を求める
	yyy = (sliderNum - (int)xxx) * 10;								//補正後の数値 * 10
	timei = (int)yyy / 100;
	//floor((1580 - floor(1580 / 100)* 100) / 25)
	vvv = floor((yyy - floor(yyy / 100.0) * 100.0) / 25.0);
	zzz = timerMinutes[(int)vvv];
	c3StartTimeHourTemporary = pgm_read_word(timer_table + timei);
	c3StartTimeMinuteTemporary = pgm_read_word(timerMinutes + (int)vvv);
	//Serial.print("h2[");
	//Serial.print(sliderNum);
	//Serial.print("]");
	//timer_table[timei];
	//Serial.print("TH[");
	//Serial.print(c3StartTimeHourTemporary);
	//Serial.print("]");
	//Serial.print("TM[");
	//Serial.print(c3StartTimeMinuteTemporary);
	//Serial.print("]");
	//Serial.print("XXX[");
	//Serial.print(xxx);
	//Serial.print("]");
	//Serial.print("YYY[");
	//Serial.print(yyy);
	//Serial.print("]");
	//Serial.print("VVV[");
	//Serial.print(vvv);
	//Serial.print("]");
	//Serial.print("ZZZ[");
	//Serial.print(zzz);
	//Serial.println("]");
}
void check_slider_4()
{
	uint32_t sliderNum = 0;
	float  timef;
	int   timei;
	float xxx = 0;
	float yyy = 0;
	float vvv;
	int zzz = 0;

	slider4ChgFlag = 1;

	getValue("h3", &sliderNum);

	timef = (float)sliderNum / 10.0;
	//timei = (int)timef;

	xxx = ((float)sliderNum / 300) * ((c4ChargeTime / 60) * 10);		//補正値を求める
	yyy = (sliderNum - (int)xxx) * 10;								//補正後の数値 * 10
	timei = (int)yyy / 100;
	//floor((1580 - floor(1580 / 100)* 100) / 25)
	vvv = floor((yyy - floor(yyy / 100.0) * 100.0) / 25.0);
	zzz = timerMinutes[(int)vvv];
	c4StartTimeHourTemporary = pgm_read_word(timer_table + timei);
	c4StartTimeMinuteTemporary = pgm_read_word(timerMinutes + (int)vvv);
	//Serial.print("h2[");
	//Serial.print(sliderNum);
	//Serial.print("]");
	//timer_table[timei];
	//Serial.print("TH[");
	//Serial.print(c4StartTimeHourTemporary);
	//Serial.print("]");
	//Serial.print("TM[");
	//Serial.print(c4StartTimeMinuteTemporary);
	//Serial.print("]");
	//Serial.print("XXX[");
	//Serial.print(xxx);
	//Serial.print("]");
	//Serial.print("YYY[");
	//Serial.print(yyy);
	//Serial.print("]");
	//Serial.print("VVV[");
	//Serial.print(vvv);
	//Serial.print("]");
	//Serial.print("ZZZ[");
	//Serial.print(zzz);
	//Serial.println("]");
}

void set_week_data()
{
	uint32_t eeprom_data;
	eeprom_data = (uint32_t)read_eerpom(MEM_WEEK_MON);
	setValue("c0", eeprom_data);
	eeprom_data = (uint32_t)read_eerpom(MEM_WEEK_TUE);
	setValue("c1", eeprom_data);
	eeprom_data = (uint32_t)read_eerpom(MEM_WEEK_WED);
	setValue("c2", eeprom_data);
	eeprom_data = (uint32_t)read_eerpom(MEM_WEEK_THU);
	setValue("c3", eeprom_data);
	eeprom_data = (uint32_t)read_eerpom(MEM_WEEK_FRI);
	setValue("c4", eeprom_data);
	eeprom_data = (uint32_t)read_eerpom(MEM_WEEK_STA);
	setValue("c5", eeprom_data);
	eeprom_data = (uint32_t)read_eerpom(MEM_WEEK_SUN);
	setValue("c6", eeprom_data);

}

void get_week_data()
{

uint32_t weekNum = 0;

getValue("c0", &weekNum);
write_eeprom(MEM_WEEK_MON, (uint8_t)weekNum);
week_flag[WEEK_MON] = (int8_t)weekNum;
Serial.print("MON[");
Serial.print(weekNum);
Serial.print("]");
getValue("c1", &weekNum);
write_eeprom(MEM_WEEK_TUE, (uint8_t)weekNum);
week_flag[WEEK_TUE] = (int8_t)weekNum;
Serial.print("TUE[");
Serial.print(weekNum);
Serial.print("]");
getValue("c2", &weekNum);
write_eeprom(MEM_WEEK_WED, (uint8_t)weekNum);
week_flag[WEEK_WED] = (int8_t)weekNum;
Serial.print("WED[");
Serial.print(weekNum);
Serial.print("]");
getValue("c3", &weekNum);
write_eeprom(MEM_WEEK_THU, (uint8_t)weekNum);
week_flag[WEEK_THU] = (int8_t)weekNum;
Serial.print("THU[");
Serial.print(weekNum);
Serial.print("]");
getValue("c4", &weekNum);
write_eeprom(MEM_WEEK_FRI, (uint8_t)weekNum);
week_flag[WEEK_FRI] = (int8_t)weekNum;
Serial.print("FRI[");
Serial.print(weekNum);
Serial.print("]");
getValue("c5", &weekNum);
write_eeprom(MEM_WEEK_STA, (uint8_t)weekNum);
week_flag[WEEK_STA] = (int8_t)weekNum;
Serial.print("STA[");
Serial.print(weekNum);
Serial.print("]");
getValue("c6", &weekNum);
write_eeprom(MEM_WEEK_SUN, (uint8_t)weekNum);
week_flag[WEEK_SUN] = (int8_t)weekNum;
Serial.print("SUN[");
Serial.print(weekNum);
Serial.println("]");

}


void write_eeprom_timer1()
{
	write_eeprom(MEM_C1_START_HOUR, (uint8_t)c1StartTimeHour);
	write_eeprom(MEM_C1_START_MINUTE, (uint8_t)c1StartTimeMinute);
	write_eeprom(MEM_C1_CHARGE_TIME, (uint8_t)c1ChargeTime);

}
void write_eeprom_timer2()
{
	write_eeprom(MEM_C2_START_HOUR, (uint8_t)c2StartTimeHour);
	write_eeprom(MEM_C2_START_MINUTE, (uint8_t)c2StartTimeMinute);
	write_eeprom(MEM_C2_CHARGE_TIME, (uint8_t)c2ChargeTime);

}
void write_eeprom_timer3()
{
	write_eeprom(MEM_C3_START_HOUR, (uint8_t)c3StartTimeHour);
	write_eeprom(MEM_C3_START_MINUTE, (uint8_t)c3StartTimeMinute);
	write_eeprom(MEM_C3_CHARGE_TIME, (uint8_t)c3ChargeTime);

}
void write_eeprom_timer4()
{
	write_eeprom(MEM_C4_START_HOUR, (uint8_t)c4StartTimeHour);
	write_eeprom(MEM_C4_START_MINUTE, (uint8_t)c4StartTimeMinute);
	write_eeprom(MEM_C4_CHARGE_TIME, (uint8_t)c4ChargeTime);

}

//---------------------------------------------------------------------------------------------------
//タイマーの自動処理
//---------------------------------------------------------------------------------------------------
int8_t timer_auto_n(int8_t cStartTimeHour, int8_t cStartTimeMinute, int8_t week_flag_x, int16_t cTimerValue)
{
	int8_t timerFlag = 0;
	int16_t watchPoint = 0;
	int16_t chargeStartPoint = 0;
	int16_t chargeEndPoint = 0;
	int16_t timeBoundaryAns = 0;
	int16_t offsetTime = 0;

	timerFlag = 0;
	if (week_flag_x == 1)
	{
		watchPoint = (gHour * 60) + gMinute;
		chargeStartPoint = (cStartTimeHour * 60) + cStartTimeMinute;
		chargeEndPoint = chargeStartPoint + cTimerValue;
		timeBoundaryAns = chargeEndPoint - TIME_BOUNDARY;
		if (timeBoundaryAns < 0)
		{
			if ((watchPoint >= chargeStartPoint)  & (watchPoint < (chargeStartPoint + cTimerValue)))
			{
				timerFlag = 1;
			}
		}
		else
		{
			if (gHour < (MAX_CHAGE_TIME / 60))
			{
				offsetTime = 1440 + gHour * 60;
				if ((offsetTime >= chargeStartPoint) & (offsetTime < (chargeStartPoint + cTimerValue)))
				{
					timerFlag = 1;
				}
			}
			else
			{
				if ((watchPoint >= chargeStartPoint) & (watchPoint < (chargeStartPoint + cTimerValue)))
				{
					timerFlag = 1;
				}
			}
		}
	}
	return timerFlag;

}

//----------------------------------------------------------------------------------------------------
//タイマーの自動処理
//----------------------------------------------------------------------------------------------------
void timer_auto()
{
	//timer 1
	m_relay1 = timer_auto_n(c1StartTimeHour, c1StartTimeMinute, week_flag[gWeek], c1ChargeTime);

	//timer 2
	m_relay2 = timer_auto_n(c2StartTimeHour, c2StartTimeMinute, week_flag[gWeek], c2ChargeTime);

	//timer 3
	m_relay3 = timer_auto_n(c3StartTimeHour, c3StartTimeMinute, week_flag[gWeek], c3ChargeTime);

	//timer 4
	m_relay4 = timer_auto_n(c4StartTimeHour, c4StartTimeMinute, week_flag[gWeek], c4ChargeTime);

	//Serial.print("MR1[");
	//Serial.print(m_relay1);
	//Serial.println("]");
	//Serial.print("ST1[");
	//Serial.print(c1StartTimeHour);
	//Serial.println("]");
	//Serial.print("ET1[");
	//Serial.print(c1StartTimeMinute);
	//Serial.println("]");
	//Serial.print("CV1[");
	//Serial.print(c1ChargeTime);
	//Serial.println("]");
	//Serial.print("GWEK[");
	//Serial.print(gWeek);
	//Serial.println("]");
	//Serial.print("WEK[");
	//Serial.print(week_flag[gWeek]);
	//Serial.println("]");

}

void checkLedData(uint8_t hedder, uint8_t pid, uint8_t cid, int32_t event)
{
    Serial.print("H[");
    Serial.print(hedder, HEX);
    Serial.print(":PID ");
    Serial.print(pid, HEX);
    Serial.print(":CID ");
    Serial.print(cid, HEX);
    Serial.print(":");
    Serial.print(event);
    Serial.println("]");
    if (hedder == 0x65)
    {
        //page管理
        //page 0 メイン画面
        if (pid == PAGE_MAIN)
        {
            if (cid == 0x09)
            {
                //pageNum = PAGE_SETUP;
				goto_page(PAGE_SETUP);

            }
            else if (cid == 0x0a)
            {
                //pageNum = PAGE_AUTO_1;            //自動画面へ
				goto_page(PAGE_AUTO);
				m_relay1 = 0;
				m_relay2 = 0;
				m_relay3 = 0;
				m_relay4 = 0;

                setDispAuto1();
				setDispAuto2();
				setDispAuto3();
				setDispAuto4();
            }
            else if (cid == 0x0b)
            {
                //pageNum = PAGE_MANUAL;
				autoManualMode = 1;			//手動モード
				goto_page(PAGE_MANUAL);
				set_relay_state();
            }



        }
        //page 1 設定画面
        else if (pid == PAGE_SETUP)
        {
            if (cid == 0x01)
            {
                //pageNum = PAGE_DATE_SET;        //page 6 時計設定へ
				goto_page(PAGE_DATE_SET);
                SetDateTime();              //日付設定画面に遷移する前に設定データを設定レジスタにコピーする。
                dispCurrentDate();      //画面に現在の最新の日付を表示する。
            }
            else if (cid == 0x02)
            {
                //pageNum = PAGE_TIMER_SETUP;        //page 2 タイマー設定へ
				goto_page(PAGE_TIMER_SETUP);
            }
        }
        //タイマー設定メイン画面
        else if (pid == PAGE_TIMER_SETUP)
        {
            if (cid == 0x03)
            {
                //pageNum = PAGE_CHARGE_SETUP_1;        //page 4 回路1タイマー設定へ
				goto_page(PAGE_CHARGE_SETUP_1);
                dispCurrentTimer1();    //回路１タイマー設定へ遷移する時に表示タイマー値の現在値を表示する。
            }
			else if (cid == 0x04)
			{
				//pageNum = PAGE_CHARGE_SETUP_2;        //page 5 回路2タイマー設定へ
				goto_page(PAGE_CHARGE_SETUP_2);
				dispCurrentTimer2();    //回路２タイマー設定へ遷移する時に表示タイマー値の現在値を表示する。
			}
			else if (cid == 0x07)
			{
				//pageNum = PAGE_CHARGE_SETUP_3;        //page 5 回路2タイマー設定へ
				goto_page(PAGE_CHARGE_SETUP_3);
				dispCurrentTimer3();    //回路２タイマー設定へ遷移する時に表示タイマー値の現在値を表示する。
			}
			else if (cid == 0x08)
			{
				//pageNum = PAGE_CHARGE_SETUP_4;        //page 5 回路2タイマー設定へ
				goto_page(PAGE_CHARGE_SETUP_4);
				dispCurrentTimer4();    //回路２タイマー設定へ遷移する時に表示タイマー値の現在値を表示する。
			}
			else if (cid == 0x05)
            {
				//pageNum = PAGE_WEEK_SETUP;
				
				goto_page(PAGE_WEEK_SETUP);
				set_week_data();
;				//page 3 週間スケジュール設定へ
            }
            else if (cid == 0x06)
            {
                //pageNum = PAGE_MAIN;        //page 0 メインへ
				goto_page(PAGE_MAIN);
            }

        }
        //週間予定設定画面
        else if (pid == PAGE_WEEK_SETUP)
        {
            if (cid == 0x0b)
            {
                //pageNum = PAGE_MAIN;        //page 0 メインへ
				get_week_data();
				goto_page(PAGE_MAIN);
            }
        }
        //回路1タイマー設定画面
        else if (pid == PAGE_CHARGE_SETUP_1)
        {
            if (cid == 0x0e)
            {
                //pageNum = PAGE_MAIN;        //page 0 メインへ
				write_eeprom_timer1();
				goto_page(PAGE_MAIN);
            }
			else
			{
				timer1Set(cid);
			}
            
        }
		//回路2タイマー設定画面
		else if (pid == PAGE_CHARGE_SETUP_2)
		{
			if (cid == 0x0e)
			{
				//pageNum = PAGE_MAIN;        //page 0 メインへ
				write_eeprom_timer2();
				goto_page(PAGE_MAIN);
			}
			else
			{
				timer2Set(cid);
			}
			
		}
		//回路3タイマー設定画面
		else if (pid == PAGE_CHARGE_SETUP_3)
		{
			if (cid == 0x0e)
			{
				//pageNum = PAGE_MAIN;        //page 0 メインへ
				write_eeprom_timer3();
				goto_page(PAGE_MAIN);
			}
			else
			{
				timer3Set(cid);
			}
			
		}
		//回路4タイマー設定画面
		else if (pid == PAGE_CHARGE_SETUP_4)
		{
			if (cid == 0x0e)
			{
				//pageNum = PAGE_MAIN;        //page 0 メインへ
				write_eeprom_timer4();
				goto_page(PAGE_MAIN);
			}
			else
			{
				timer4Set(cid);
			}
			
		}
		//日付設定画面
        else if (pid == PAGE_DATE_SET)
        {
            if (cid == 0x03)
            {
                //pageNum = PAGE_TIME_SET;        //page 7 時刻設定へ
				goto_page(PAGE_TIME_SET);
                //getDateLcd();           ///時刻設定画面に遷移する前に設定された日付データをLCDから取り出す。
                //SetTimeMain();              //時刻設定画面に遷移する前に時刻データを設定レジスタにコピーする。
                dispCurrentTime();
            }
			else
			{
				page6Set(cid);              //日付の設定
				getDateLcd();

			}
            
        }
        //時刻設定画面
        else if (pid == PAGE_TIME_SET)
        {
            if (cid == 0x0b)
            {
                //pageNum = PAGE_MAIN;        //page 0 メインへ
				goto_page(PAGE_MAIN);
                
                setDateAndTime();       //この画面を抜ける時に日付時刻の設定を時計ICに書き込む
				dispCurrentDateTime();
            }
			else
			{
				page7Set(cid);              //時刻の設定
				getTimeLcd();

			}
        }
        //手動操作画面
        else if (pid == PAGE_MANUAL)
        {
            if (cid == 0x07)
            {
                //pageNum = PAGE_MAIN;        //page 0 メインへ
				goto_page(PAGE_MAIN);
				//autoManualMode = 1;			//手動モード
				setPic("p11", 59);			//手動モード表示
            }
			else
			{
				relayControl(cid);

			}
            

        }
		/*
		//回路１タイマー設定状態画面
		else if (pid == PAGE_AUTO_1)
		{
			if (cid == 0x05)
			{
				///pageNum = PAGE_AUTO_2;        //page 0 メインへ
				goto_page(PAGE_AUTO_2);
				setDispAuto2();
			}
		}
		//回路２タイマー設定状態画面
		else if (pid == PAGE_AUTO_2)
		{
			if (cid == 0x05)
			{
				//pageNum = PAGE_AUTO_3;        //page 0 メインへ
				goto_page(PAGE_AUTO_3);
				setDispAuto3();
			}
		}
		//回路３タイマー設定状態画面
		else if (pid == PAGE_AUTO_3)
		{
			if (cid == 0x05)
			{
				//pageNum = PAGE_AUTO_4;        //page 0 メインへ
				goto_page(PAGE_AUTO_4);
				setDispAuto4();
			}
		}
		//回路４タイマー設定状態画面
		else if (pid == PAGE_AUTO_4)
		{
			if (cid == 0x05)
			{
				//pageNum = PAGE_MAIN;        //page 0 メインへ
				goto_page(PAGE_MAIN);
				autoManualMode = 0;			//手動モード
				setPic("p11", 58);			//手動モード表示
			}
		}
		*/
		//タイマー設定状態画面
		else if (pid == PAGE_AUTO)
		{
			if (cid == 0x02)
			{
				if (slider1ChgFlag == 1)
				{
					//c1StartTimeHour = c1StartTimeHourTemporary;
					//c1StartTimeMinute = c1StartTimeMinuteTemporary;
					//Serial.print("slider1");
					//setDispAuto1();
				}
				if (slider2ChgFlag == 1)
				{
					//c2StartTimeHour = c2StartTimeHourTemporary;
					//c2StartTimeMinute = c2StartTimeMinuteTemporary;
					//setDispAuto2();
				}
				if (slider3ChgFlag == 1)
				{
					//c3StartTimeHour = c3StartTimeHourTemporary;
					//c3StartTimeMinute = c3StartTimeMinuteTemporary;
					//setDispAuto3();
				}
				if (slider4ChgFlag == 1)
				{
					//c4StartTimeHour = c4StartTimeHourTemporary;
					//c4StartTimeMinute = c4StartTimeMinuteTemporary;
					//setDispAuto4();
				}
				//pageNum = PAGE_MAIN;        //page 0 メインへ
				goto_page(PAGE_MAIN);
				autoManualMode = 0;			//autoモード
				setPic("p11", 58);			//手動モード表示
			}
			else if (cid == 0x01)
			{
				check_slider_1();
				setDispAuto1();
			}
			else if (cid == 0x0a)
			{
				check_slider_2();
				setDispAuto2();
			}
			else if (cid == 0x0b)
			{
				check_slider_3();
				setDispAuto3();
			}
			else if (cid == 0x0c)
			{
				check_slider_4();
				setDispAuto4();
			}


		}
		else if (pid == PAGE_WARNING)
		{
			if (cid == 0x03)
			{
				goto_page(PAGE_MAIN);
			}
			

		}

}

    Serial.print("PAGE[");
    Serial.print(pageNum);
    Serial.println("]");

}

int8_t check_eeprom()
{
	byte rdata = mem.read(0);
	if (rdata == 0x5a)
	{
		return 1;
	}
	else
	{
		return 0;
	}

}

uint8_t read_eerpom(unsigned int address)
{
	uint8_t rdata;

	mem.read(address, &rdata, 1);
	return rdata;

}
void init_eeprom()
{
	mem.write(0, mem_buffer, 20);

}
void write_eeprom(unsigned int address, uint8_t wdata)
{
	mem.write(address, wdata);

}
void read_buffer_eeprom()
{
	mem.read(0, mem_buffer, 20);
}

void mem_buffer_copy()
{

	c1StartTimeHour =(int8_t) mem_buffer[MEM_C1_START_HOUR];
	c1StartTimeMinute = (int8_t)mem_buffer[MEM_C1_START_MINUTE];
	c1ChargeTime = (int16_t)mem_buffer[MEM_C1_CHARGE_TIME];

	c2StartTimeHour = (int8_t)mem_buffer[MEM_C2_START_HOUR];
	c2StartTimeMinute = (int8_t)mem_buffer[MEM_C2_START_MINUTE];
	c2ChargeTime = (int16_t)mem_buffer[MEM_C2_CHARGE_TIME];

	c3StartTimeHour = (int8_t)mem_buffer[MEM_C3_START_HOUR];
	c3StartTimeMinute = (int8_t)mem_buffer[MEM_C3_START_MINUTE];
	c3ChargeTime = (int16_t)mem_buffer[MEM_C3_CHARGE_TIME];

	c4StartTimeHour = (int8_t)mem_buffer[MEM_C4_START_HOUR];
	c4StartTimeMinute = (int8_t)mem_buffer[MEM_C4_START_MINUTE];
	c4ChargeTime = (int16_t)mem_buffer[MEM_C4_CHARGE_TIME];

	week_flag[WEEK_MON] = (int8_t)mem_buffer[MEM_WEEK_MON];
	week_flag[WEEK_TUE] = (int8_t)mem_buffer[MEM_WEEK_TUE];
	week_flag[WEEK_WED] = (int8_t)mem_buffer[MEM_WEEK_WED];
	week_flag[WEEK_THU] = (int8_t)mem_buffer[MEM_WEEK_THU];
	week_flag[WEEK_FRI] = (int8_t)mem_buffer[MEM_WEEK_FRI];
	week_flag[WEEK_STA] = (int8_t)mem_buffer[MEM_WEEK_STA];
	week_flag[WEEK_SUN] = (int8_t)mem_buffer[MEM_WEEK_SUN];


}

bool initLcd()
{
    bool ret1 = false;
    bool ret2 = false;

    sendCommand("");
    sendCommand("bkcmd=1");
    ret1 = recvRetCommandFinished(500);
    sendCommand("page 0");
    ret2 = recvRetCommandFinished(500);
    return ret1 && ret2;
}

void setup(void)
{

	pinMode(9, INPUT);
	pinMode(10, INPUT);
	pinMode(11, INPUT);
	pinMode(14, INPUT);    //SQW
	pinMode(15, INPUT);    //PAD1
	pinMode(16, INPUT);    //NC

	pinMode(POW_CONT1_PIN, OUTPUT);
	pinMode(POW_CONT2_PIN, OUTPUT);
	digitalWrite(POW_CONT1_PIN, POW_CONTROL_OFF);
	digitalWrite(POW_CONT2_PIN, POW_CONTROL_OFF);

	//pinMode(MTR0_PIN,OUTPUT);
	//pinMode(MTR1_PIN,OUTPUT);
	//digitalWrite(MTR0_PIN,MOTOR_SW_OFF);
	//digitalWrite(MTR1_PIN,MOTOR_SW_OFF);
	pinMode(LED1_PIN, OUTPUT);
	digitalWrite(LED1_PIN, LOW);
	pinMode(LED2_PIN, OUTPUT);
	digitalWrite(LED2_PIN, LOW);

	Serial.begin(115200);
	while (!Serial);
	nexSerial.begin(9600);
	while (!nexSerial);

	PCF_SOL.write8(0x00);      //リレーALLオフ
	uint8_t sol_data = PCF_SOL.read8();     //確認
	Serial.print("SOL=");
	Serial.println(sol_data, HEX);


	uint8_t sw = ~PCF_SW.read8();   //スイッチの読み込み
	Serial.print("SW=");
	Serial.println(sw, HEX);

	m_relay1 = 0;
	m_relay2 = 0;
	m_relay3 = 0;
	m_relay4 = 0;
	autoRelayControl();

	setSyncProvider(RTC.get);   // the function to get the time from the RTC
	if (timeStatus() != timeSet)
	{
		Serial.println("Unable to sync with the RTC");
	}
	else
	{

		Serial.println("RTC has set the system time");
	}
	Serial.println("GET CLOCK");
    getClock();
    if (gYear < 2020)
    {
        gYear = 2020;
        gMonth = 1;
        gDay = 1;
        gHour = 0;
        gMinute = 0;

    }
	Serial.println("CHECK EEPROM");
	int8_t resultEeprom = check_eeprom();
	Serial.println(resultEeprom);


	if (resultEeprom == 0)
	{
		Serial.println("INIT EEPROM!!");
		init_eeprom();
	}
	else
	{
		Serial.println("READ EEPROM!!");
		read_buffer_eeprom();
	}
	Serial.println("BUFFER COPY");
	mem_buffer_copy();




    initLcd();				//START画面を2秒間表示
    pageNum = 0;
	delay(5000);
	goto_page(PAGE_MAIN);
    //timerFlag = 0;
    MsTimer2::set(200, timerCheck);       //Timer Config
    MsTimer2::start();                    //Start Timer
}

void loop(void)
{

    static uint8_t __buffer[10];
	int8_t checkSecond;
    uint16_t i;
    uint8_t c;
	uint16_t upper = 0;
	uint16_t lower = 0;
	uint16_t ain = 0;

    while (nexSerial.available() > 0)
    {
        delay(10);
        c = nexSerial.read();

        if (NEX_RET_EVENT_TOUCH_HEAD == c)
        {
            if (nexSerial.available() >= 6)
            {
                __buffer[0] = c;
                for (i = 1; i < 7; i++)
                {
                    __buffer[i] = nexSerial.read();
                }
                __buffer[i] = 0x00;

                if (0xFF == __buffer[4] && 0xFF == __buffer[5] && 0xFF == __buffer[6])
                {
                    checkLedData(__buffer[0], __buffer[1], __buffer[2], (int32_t)__buffer[3]);
                }

            }
        }
    }    //if ( pageNum == 0)
    //{
    //    timerCheck();
    //}
	if (checkTimerFlag == 1)
	{
		checkTimerFlag = 0;
		//Serial.print("TH[");
		checkSecond = second();
		//Serial.print("TH[");
		//Serial.print(checkSecond);
		//Serial.println("]");
		if (nextSecond != checkSecond)
		{
			nextSecond = checkSecond;
			getClock();

			if (pageNum == PAGE_MAIN)
			{
				page0DigitalClockDisplay();
				if (autoManualMode == 0)
				{
					timer_auto();
					autoRelayControl();
				}
				if (calCurrent > 10)
				{
					c1TimerStartFalg = 3;
					c2TimerStartFalg = 3;
					c3TimerStartFalg = 3;
					c4TimerStartFalg = 3;
					m_relay1 = 0;
					m_relay2 = 0;
					m_relay3 = 0;
					m_relay4 = 0;
					autoRelayControl();
					goto_page(PAGE_WARNING);
				}
			}
			else if ((pageNum == PAGE_AUTO_1) | (pageNum == PAGE_AUTO_2) | (pageNum == PAGE_AUTO_3) | (pageNum == PAGE_AUTO_4) | (pageNum == PAGE_AUTO))
			{
				page9DigitalClockDisplay();

			}
			upper = 0;
			lower = 2000;
			ain = 0;
			for (int xxx = 0; xxx < 200; xxx++)
			{

				ain = (uint32_t)analogRead(A2);

				if (ain > upper)
				{
					upper = ain;
				}
				if (ain < lower)
				{
					lower = ain;
				}

			}
			ptoptReg[moveCount] = upper - lower;
			ptopTotal = (uint32_t)(ptoptReg[0] + ptoptReg[1] + ptoptReg[2] + ptoptReg[3] + ptoptReg[4] + ptoptReg[5] + ptoptReg[6] + ptoptReg[7]);
			ptopTotal = ptopTotal >> 3;
			calCurrent = ptopTotal * 49 * 3;
			calCurrent = calCurrent / 10000;
			//Serial.print("UP[");
			//Serial.print(upper);
			//Serial.print("] ");
			//Serial.print("LOWE[");
			//Serial.print(lower);
			//Serial.print("]");
			//Serial.print("P-P[");
			//Serial.print(ptoptReg[moveCount]);
			//Serial.print("]");
			//Serial.print("A[");
			//Serial.print(calCurrent);
			//Serial.println("]");
			moveCount++;
			if (moveCount >= 8)
			{
				moveCount = 0;
			}
		}
	}

    //delay(2000);
}
