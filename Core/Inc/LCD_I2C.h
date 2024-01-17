/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCD_I2C_H
#define __LCD_I2C_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

#define LCD_Not_Use_Port_RW 0xFF
#define LCD_Instruction LOW
#define LCD_Data HIGH
#define LCD_Write LOW
#define LCD_Read HIGH

/* Private defines -----------------------------------------------------------*/

#define addr_pcf8574 0x4E // dia chi cua slave i2c 

/* Exported constants --------------------------------------------------------*/
#define LCD_Clear 0x01

#define LCD_CursorReturn 0x02

#define LCD_InputSet 0x04
#define LCD_Input_Increment 0x02
#define LCD_Input_Decrement 0x00
#define LCD_Input_Shift 0x01
#define LCD_Input_Noshift 0x00

#define LCD_DisplaySwitch 0x08
#define LCD_Display_DisplayOn 0x04
#define LCD_Display_CursorOn 0x02
#define LCD_Display_BlinkOn 0x01
#define LCD_Display_DisplayOff 0x00
#define LCD_Display_CursorOff 0x0C
#define LCD_Display_BlinkOff 0x00

#define LCD_Shift 0x10
#define LCD_Shift_DisplayShift 0x08
#define LCD_Shift_CursorShift 0x00
#define LCD_Shift_Right 0x04
#define LCD_Shift_Left 0x00
#define LCD_LEFT false
#define LCD_RIGHT true
#define LCD_CURSOR false
#define LCD_DISPLAY true

#define LCD_FunctionSet 0x20
#define LCD_Function_8Port 0x10
#define LCD_Function_4Port 0x28
#define LCD_Function_2Line 0x08
#define LCD_Funciton_1Line 0x00
#define LCD_Function_char_510 0x04  
#define LCD_Function_char_57 0x00

#define LCD_CGRAMAddSet 0x40

#define LCD_DDRAMAddSet 0x80
#define LCD_Line1Start 0x00
#define LCD_Line2Start 0x40


/* Exported functions prototypes ---------------------------------------------*/


void LCD_I2C_Write_CMD(uint8_t); // Ham gui lenh

void LCD_I2C_WRITE_DATA(uint8_t); // Ham gui data

void LCD_I2C_Init(void); // Ham khoi dong LCD

void LCD_I2C_Clear(void);// Ham xoa man hinh LCD

void LCD_I2C_Set_Cursor(uint8_t,uint8_t); // Ham dat vi tri con tro LCD 

void LCD_I2C_Write_String(char *); // Ham gui mot chuoi len LCD

void LCD_I2C_WRITE_NUMBER(int);// Ham gui mot so nguyen len LCD

void LCD_I2C_WRITE_Num_f(double);// Ham gui mot so thuc len LCD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#endif