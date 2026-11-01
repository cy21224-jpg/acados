/**
 * @file io_3link.h
 * @brief 3リンク装置のprintf出力とMCP4922 DAC送信の公開宣言。
 * Core/Incに置き、呼出し元のCファイルからincludeする。
 * 関数の実体はio_3link.cに1個だけ置く。
 */
#ifndef IO_3LINK_H
#define IO_3LINK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* printfの1文字出力先。先にmain側でdma_printf_init(&huart3)を実行する。 */
int __io_putchar(int ch);

/* SPI1接続のMCP4922へ送信。channelは下位1bit、valは下位12bitを使用する。
 * SPI/GPIO初期化後に呼ぶ。元の送信データ、CS操作、タイムアウト100 msを維持。
 */
void MCP4922_Write_DAC1(uint8_t channel, uint16_t val);

/* SPI2接続のMCP4922へ、DAC1と同じ形式で送信する。 */
void MCP4922_Write_DAC2(uint8_t channel, uint16_t val);

#ifdef __cplusplus
}
#endif

#endif /* IO_3LINK_H */
