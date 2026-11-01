# printf・DAC関数のファイル分離

添付の最新版mainから、`__io_putchar`、`MCP4922_Write_DAC1`、`MCP4922_Write_DAC2`の3関数を移しました。
関数本体は元のバイト列をそのまま使用しています。SPIの送信データ、CS操作、100 msのタイムアウト、
エラー処理、printf出力経路、周辺機器・タイマ・クロック設定は変更していません。

## 配置

- `io_3link.h` → `Core/Inc`
- `io_3link.c` → `Core/Src`
- 添付`main.c` → 直前に送ってもらったmainから3関数の実体だけを取り除き、includeを追加した版です。

手元のmainをその後変更している場合は、添付mainで丸ごと上書きせず、次の変更だけを反映してください。

1. `USER CODE BEGIN Includes`内へ`#include "io_3link.h"`を追加。
2. mainにある3関数の実体を削除。`io_3link.c`側に実体が1個ずつあります。

既存のmainにある`hspi1`/`hspi2`の定義、SPI/GPIO/UART初期化は残します。
`cmd1`/`cmd2`は`param_3link.c`の定義をそのまま使い、`param_3link.h`の宣言で参照します。
`dma_printf_init(&huart3)`と`HAL_UART_TxCpltCallback()`も元のmainに残しています。
SPIやprintfを使う前に、それぞれ既存の初期化を完了してください。

## 別のCファイルから使う

```c
#include "io_3link.h"

void ExampleDacOutput(void)
{
    MCP4922_Write_DAC1(0, 2048);
    MCP4922_Write_DAC2(1, 2048);
}
```

`printf()`を使うファイルは通常どおり`#include <stdio.h>`を使います。
`__io_putchar()`を直接呼ぶ必要はなく、既存の`syscalls.c`がこの関数へ文字を渡します。

mainとio_3linkのARMコンパイル、および両者の部分リンクで関数・SPIハンドルの定義が各1個であることを確認しています。
プロジェクト全体のビルドと実機の入出力動作は手元で確認してください。
既存の共有送信バッファとSPIを使う構造を維持しているので、同じDAC関数を割込みとmainから同時に実行する用途には追加の排他設計が必要です。
