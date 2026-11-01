# STM32F767ZI acados HardFault 修正版

## 手元で試す手順
1. 元の STM32CubeIDE プロジェクトをフォルダごとバックアップしてください。
2. この ZIP の Core/Src にある main.c、stm32f7xx_it.c、sysmem.c の3ファイルを、手元の acadosdayo/Core/Src の同名ファイルにコピーしてください。
3. STM32CubeIDE でプロジェクトを右クリックして Refresh、その後 Project → Clean、Build Project を実行してください。
4. Debug で基板に書き込み、main() から実行してください。
5. interaction_model_acados_solve() の呼び出し直後にブレークポイントを置き、下記 Expressions を確認してください。

既存の Windows 用ライブラリパス・リンカスクリプト・IDE 設定は変更していません。
提供 ELF は ARM GCC 14.2.1 で作成。元の IDE の GCC 13.3.rel1 と異なるため、手元では上記ソースを再ビルドする方法を推奨します。
firmware/acadosdayo.elf と .bin は同じソースから作成した参考成果物です。.bin の Flash 開始アドレスは 0x08000000 です。
実際の基板での動作は未確認です。クラウドから手元のパソコンのファイル変更・基板への書き込みはしていません。

## 修正内容
- main.c: 2回あったソルバ生成を1回にし、最初のソルバを失うメモリリークを除去。
- stm32f7xx_it.c: 例外フレームの誤った +18 ワードの補正を削除。Cortex-M7 は FP 拡張フレームでも先頭に R0..xPSR を配置する。ステータスを先に保存し、スタック異常・範囲を検査してからフレームを読む。
- sysmem.c: ヒープの最大使用量・失敗回数・失敗した増分を記録。境界のチェックに範囲外ポインタ演算を使わず、縮小時の下限も検査。

## Expressions
- mpc_create_status: 作成成功時 0
- mpc_solve_status: 計算成功時 0。-999 は solve が戻っていないことを示す。
- g_heap_peak_bytes: ヒープの最大使用量（bytes。malloc の管理領域を含む）
- g_heap_failed_requests: 通常は 0
- g_heap_last_failed_incr: 失敗した _sbrk の要求量
- HardFault が再発した場合: g_hf_frame_valid、g_hf_stacked_pc、g_hf_stacked_lr、g_hf_stacked_xpsr、g_hf_cfsr、g_hf_hfsr、g_hf_abfsr、g_hf_bfar、g_hf_mmfar、g_heap_peak_bytes、g_heap_failed_requests を控える。

BFAR は CFSR の bit15、MMFAR は bit7 がセットされている場合のみ有効です。
CFSR=0x400 は IMPRECISERR、HFSR=0x40000000 は FORCED。非同期バスエラーでは保存 PC が原因の命令とは限りません。

## 検証と限界
- 33ソースのクリーンコンパイル・リンク成功。
- 元のアップロード済み Debug ELF と元ソースの再ビルド版の双方で、2回目の作成中にヒープ不足→NULL への書き込みを ARM エミュレーションで再現。
- 修正版: 作成1回、solve1回、両戻り値0、ヒープ失敗0。最大ヒープ使用量290348 bytes、観測スタック使用量1808 bytes。
- 基本例外フレーム・FP拡張例外フレームの保存 PC/LR/xPSR、およびスタック異常時の読み取り回避を検証。
- _sbrk の増減・上限・下限・ピーク・失敗カウンタを検証。
- Unicorn の ARM MAX CPU を使用。M7 モードでは double の命令実行に制限があったため、この検証は実機・M7 の完全な再現ではありません。HAL/周辺機器の初期化はスキップしています。DTCM/バス/割り込み/実機タイミングは未検証です。
- 既存の UNUSED 再定義・未使用変数の警告あり。ライブラリは POSIX gettimeofday を呼び、_gettimeofday は nosys の未実装スタブのままです。ソルバの計時値は信頼できず、別途STM32向けタイマ実装が必要です。今回は計時処理を変更していません。
- 提供された実機レジスタとエミュレーションの失敗位置は完全一致とは未確認。二重生成は確認済み不具合ですが、実機 HardFault の唯一の原因と断定していません。
