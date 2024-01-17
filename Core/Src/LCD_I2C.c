/*
 * LCD_I2C.c
 *
 *  Created on: Dec 14, 2023
 *      Author: Tran Luong, Xuan Mai
 */

/* Private includes ----------------------------------------------------------*/
#include "LCD_I2C.h"
#include "main.h"
#include "string.h"
#include "stdio.h"

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/

extern I2C_HandleTypeDef hi2c1;  // handle xu ly i2c


/* Private function prototypes -----------------------------------------------*/


/* Private user code ---------------------------------------------------------*/

void LCD_I2C_Write_CMD(uint8_t data)
{
	uint8_t buf[4] = { (data & 0xF0) | 0x0C ,(data & 0xF0) |0x08 , (data << 4) | 0x0C ,( data << 4) | 0x08 }; // truyen 4 byte du lieu trong do 
//truyen 4 byte cao truoc , 0x0C v� 0x08 l� de chon chan RW tu 0->1 truyen cmd
	HAL_I2C_Master_Transmit(&hi2c1, addr_pcf8574,buf,4, 100); // truyen i2c trong STM32
}
void LCD_I2C_WRITE_DATA(uint8_t data)
{
	uint8_t buf[4] = { (data& 0xF0) | 0x0D ,(data & 0xF0) |0x09 , (data << 4) |0x0D ,(data << 4)| 0x09 };// truyen 4 byte du lieu trong do 
//truyen 4 byte cao truoc , 0x0D v� 0x09 l� de chon chan RS tu 0->1 truyen data
	HAL_I2C_Master_Transmit(&hi2c1, addr_pcf8574, buf,4, 100);// truyen 4 byte du lieu di 
}	
void LCD_I2C_Init()
{ 
	LCD_I2C_Write_CMD(0x33);
	HAL_Delay(50);
	LCD_I2C_Write_CMD(0x32);
	HAL_Delay(50);
    LCD_I2C_Write_CMD(LCD_Function_4Port); // Che do 4 bit 
	HAL_Delay(50);
	LCD_I2C_Write_CMD(LCD_Clear); // Xoa man hinh LCD 
	HAL_Delay(50);
	LCD_I2C_Write_CMD(0x06); // Di chuyen tro xuat 1 ky tu 
	HAL_Delay(50);
	LCD_I2C_Write_CMD(LCD_Display_CursorOff);// Tat con tro 
	HAL_Delay(50);
	LCD_I2C_Write_CMD(LCD_CursorReturn);// Con tro ve dau man hinh 
	HAL_Delay(50);
	LCD_I2C_Write_CMD(LCD_DDRAMAddSet); // Con tro ve dong 1 
	HAL_Delay(2);
}

void LCD_I2C_Clear()// xoa man hinh 
{
	LCD_I2C_Write_CMD(LCD_Clear); 
	HAL_Delay(2);
}

void LCD_I2C_Set_Cursor(uint8_t x,uint8_t y) // dat vi tri cho con tro LCD 
{
	if( x == 0)
		LCD_I2C_Write_CMD(LCD_DDRAMAddSet + y); 
	else if(x == 1)
		LCD_I2C_Write_CMD(0xC0+y);	
}

void  LCD_I2C_Write_String(char *string)  // in ra chuoi 
{
	for(uint8_t i=0; i < strlen(string); i++ ){
    LCD_I2C_WRITE_DATA( string[i] );
	}
}	
void LCD_I2C_WRITE_NUMBER(int number) // in ra so nguyen
{ 
   char buf[8];
	 sprintf(buf,"%d",number);
	 LCD_I2C_Write_String(buf);
}
void LCD_I2C_WRITE_Num_f(double var)  // in ra so dang thuc 
{ 
   char a[5];
	 sprintf(a, "%3.1f", var);
	 LCD_I2C_Write_String(a);
}
