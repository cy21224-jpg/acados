# 制御入力の数値・制約チェック追加版

送っていただいた、実機で solve=0 を確認済みの main.c を基にチェック処理を追加しました。
HardFault 修正を適用済みの場合、今回差し替えるのは `Core/Src/main.c` の1ファイルだけです。

## 使い方
1. 手元の main.c をバックアップし、このフォルダの Core/Src/main.c に差し替える。
2. STM32CubeIDE で Refresh → Project / Clean → Build → Debug。
3. USART3 の出力を 115200 bps・8 bit・パリティなし・ストップ1 bit で確認する。既存の USART3 接続を使用する。
4. Expressions でも `mpc_solve_status`、`mpc_check_done`、`mpc_check_status`、`mpc_u0` を確認できる。

## 追加したチェック
- solve が失敗したら結果を有効な入力として扱わず、mpc_u0 は NaN のままにする。
- 10ステップ分の入力と、終端を含む11点の予測状態が NaN / Inf でないか確認する。
- ソルバから lbu / ubu / lbx / ubx と制約対象のインデックスを読み取り、入力・状態の上下限違反をチェックする。現在の入力範囲は 0～10。定数で判定していないため、同じ次元のモデルで上下限を変更すれば、その設定で判定する。
- 許容誤差は各変数のモデル単位で絶対値 1e-6。これは丸め誤差用であり、実機の安全余裕ではない。
- UART に最初の入力 u0、10個の入力、11点の状態、違反の最大量、最初の違反ステップを一度だけ表示する。元の無限 OK 表示は停止し、ループ内は HAL_Delay(1000) とした。
- アクチュエータへの出力や入力のクリッピングは行わない。

## 判定の読み方
`mpc_check_done=1` の後に `mpc_check_status` を確認する。

| 値 / ビット | 意味 |
| --- | --- |
| -999 | 未チェック |
| 0 | 数値・上下限制約のチェックに合格 |
| 1 | ソルバ失敗 |
| 2 | NaN または Inf |
| 4 | 入力の上下限違反 |
| 8 | 予測状態の上下限違反 |
| 16 | モデルの次元・制約設定の不整合 |

非ゼロはビットの組み合わせになる。例えば 6 = 非有限値 + 入力制約違反。
`mpc_first_bad_stage` は最初の不良ステップ、-1 は該当なし。
`mpc_u_pred[0..9]`、`mpc_x_pred[0..10][0..3]`、`mpc_u_lower`、`mpc_u_upper` も Expressions で確認できる。

## このチェックで分かることの限界
`NUMERIC_BOUNDS_PASS` は、実機で望ましい動作をする保証や、最適性・閉ループ安定性の検証ではない。現在の初期状態 x0 は上下限の範囲だけで、実測状態に固定されていない。UART はその場合 `WARNING: x0 is a range, not fixed to a measured state.` と表示する。`mpc_initial_state_fixed=0` でも数値・制約の判定とは別なので check=0 になり得る。
実測状態を使った制御にするには、状態の意味・単位・センサ値を確認し、solve 前に stage 0 の lbx / ubx を同じ実測値に更新する必要がある。今回はモデルや制御条件は変更していない。
現在の SQP_RTI は1回の近似ステップであり、solve=0 だけで目標追従を保証しない。モデルの入力参照値が3であっても、u0=3になることを合格条件にはしていない。

## 今回の検証
33ソースのビルド・リンク成功。ARM エミュレーションで通常出力を取得し、solve=0、check=0、u0≈0 を確認した。実機の数値が同じになると断定するものではない。
NaN / Inf、入力の上下限超過（終端直前のステップを含む）、状態の終端制約違反、許容誤差の境界、ソルバ失敗、再実行時のフラグリセットを検証した。printf の出力も捕捉し、全ステップの値と初期状態の警告が表示されることを確認した。UART の電気的な出力を実機で検証したわけではない。
`validation/test_control_output.py` が追加テスト。元の HardFault / ヒープのテストも残している。

---

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
