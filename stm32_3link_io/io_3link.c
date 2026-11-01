/**
 * @file io_3link.c
 * @brief mainから移したprintf・DAC出力の実体。Core/Srcに1個だけ置く。
 * 3関数の本体は添付mainから変更せず移動している。
 * GPIO/SPI/UART/TIM/クロックの初期化や設定値はここでは変更しない。
 */
#include "io_3link.h"
#include "main.h"
#include "dma_printf.h"
#include "param_3link.h"

/* SPIハンドルの実体は既存main.cに残す。ここでは参照宣言だけを置く。 */
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

/* cmd1/cmd2の実体は既存param_3link.cに残し、ヘッダーのextern宣言で参照する。 */

int __io_putchar(int ch)
{
      dma_printf_putc(ch&0xFF);
        return ch;
}

void MCP4922_Write_DAC1(uint8_t channel, uint16_t val){
	channel = channel & 0x01;
	val = val & 0x0fff;


	cmd1[0] = (channel << 7) | (val >> 8) | 0x30;
	cmd1[1] = val & 0xff;

	HAL_GPIO_WritePin(CS_DAC1_GPIO_Port, CS_DAC1_Pin, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit(&hspi1, cmd1, 2, 100) != HAL_OK) {
		Error_Handler();
	} else{}

	HAL_GPIO_WritePin(CS_DAC1_GPIO_Port, CS_DAC1_Pin, GPIO_PIN_SET);

}

void MCP4922_Write_DAC2(uint8_t channel, uint16_t val)
{
	channel = channel & 0x01;
	val = val & 0x0fff;


	cmd2[0] = (channel << 7) | (val >> 8) | 0x30;
	cmd2[1] = val & 0xff;

	HAL_GPIO_WritePin(CS_DAC2_GPIO_Port, CS_DAC2_Pin, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit(&hspi2, cmd2, 2, 100) != HAL_OK) {
		Error_Handler();
	} else{}
	HAL_GPIO_WritePin(CS_DAC2_GPIO_Port, CS_DAC2_Pin, GPIO_PIN_SET);
}
