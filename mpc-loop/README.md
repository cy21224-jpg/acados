# while内で制御入力を繰り返し計算する版

最後に送っていただいた main.c を基に、初期化と計算ループを分離した動作確認用コードです。各処理の直前に、日本語で何を行うかのコメントを付けています。

## 差し替え方
1. 手元の main.c をバックアップする。
2. このフォルダの `Core/Src/main.c` を手元の `acadosdayo/Core/Src/main.c` にコピーする。
3. STM32CubeIDEで Refresh → Project / Clean → Build → Debug を実行する。
4. USART3（115200 bps、8 bit、パリティなし、ストップ1 bit）または Expressions で結果を確認する。

以前適用した HardFault 修正の stm32f7xx_it.c と sysmem.c、および既存のライブラリ・リンカ・IDE設定はそのまま使用する。
同梱firmwareはARM GCC 14.2.1で作成した参考成果物。手元のIDEではソースの再ビルドを推奨する。

## コードの構成
- `MPC_Init()`：whileに入る前に1回呼ぶ。カプセル確保、モデル・ソルバの作成、config / dims / out の取得を行う。
- `MPC_RunOnce()`：while内で毎回呼ぶ。作成済みのソルバで計算し、入力と状態を取得して結果変数を更新する。ループ内でカプセル作成や解放は行わない。前回のsolver出力を保持したまま次のsolveへ渡す。予測系列の明示的なシフトは行わない。
- `MPC_PrintResult()`：初回と、その後は約1秒ごとに最新結果を表示する。
- `HAL_Delay(1)`：計算・必要な表示を終えてから1 ms待つ。実際の周期には計算・UARTの時間が加算されるため、1 kHzのリアルタイム制御ではない。表示中は計算が停止する。

待ち時間は `MPC_LOOP_DELAY_MS`、表示間隔は `MPC_REPORT_INTERVAL_MS` で調整する。

## Expressionsで確認する変数
| 変数 | 意味 |
| --- | --- |
| mpc_create_status | 起動時の初期化結果。0が成功 |
| mpc_solve_status | 直近のsolveの戻り値。0が成功 |
| mpc_iteration_count | whileで実行したsolveの回数 |
| mpc_success_count | solve成功かつ取得値が有限値だった回数 |
| mpc_failure_count | solve失敗またはNaN/Infを検出した回数 |
| mpc_result_valid | 1なら直近の結果を取得できた。計算前・失敗時は0 |
| mpc_u0 | stage 0の制御入力 |
| mpc_u1 | stage 1の予測制御入力。別の入力軸ではない |
| mpc_x0[0..3] | ソルバが求めたstage 0の状態。実測状態ではない |
| mpc_solve_time_ms | HAL_GetTickによる粗い計算時間。1 ms未満は0になり得る |
| g_heap_peak_bytes | 既存のsysmem.cが記録する最大ヒープ使用量 |

計算前に前回の出力をNaNに戻し、失敗時に古い値を今回の結果として使わない。失敗時はループを停止せず、次のループで再試行する。solve=0でも取得結果が非有限値なら result_valid=0 となる。アクチュエータの異常時処理は実装していない。

## 状態と実機入出力について
現在のコードにはセンサ読み取りがなく、生成済みモデルの初期状態範囲をそのまま使う。新しい実測値はソルバへ渡していないため、この版だけで閉ループ制御になるわけではない。
センサを追加する場合は、while内の「1. 実測状態」のコメント位置で状態を取得し、stage 0の lbx / ubx を同じ測定値に更新する処理を実装する。状態の意味・単位を確認してから行うこと。ソルバのmpc_x0を測定値に見立てて戻す処理は追加していない。
実機への入力処理を追加する位置もコメントで示した。今回は計算と表示までで、アクチュエータへは出力しない。
result_valid=1はsolve成功と有限値の確認であり、制約適合・最適性・物理的妥当性の判定ではない。以前の詳細な上下限制約チェック版は `hardfault-fix` に残しており、この版と用途を分けている。

## 検証
- 33ソースのコンパイル・リンク成功。
- ARMエミュレーションで300回連続計算し、カプセル作成・ソルバ作成が各1回、カプセルのアドレスが一定、ヒープ増加なし、失敗0を確認。
- 5回目のsolve失敗を模擬し、出力がNaN・result_valid=0になり、次の回から復帰することを確認。
- 5回目の予測入力にNaNを模擬し、古い出力を公開せず、次の回から復帰することを確認。
- UART表示が毎回ではなく間引かれることと、入力値・状態値の結果変数への保存を確認。
- エミュレータはARM MAX CPUでHAL初期化・時刻・UARTを模擬する。実機の周期、周辺機器、バス、制御性能は未検証。
- 既存ライブラリの_gettimeofday未実装と既存のコンパイル警告は今回変更していない。acados内部の計時値は信頼できない。

検証の再実行（このフォルダで実行）：
```sh
python -m pip install -r validation/requirements.txt
python validation/test_loop.py firmware/acadosdayo.elf
python validation/test_loop.py firmware/acadosdayo.elf solve_failure
python validation/test_loop.py firmware/acadosdayo.elf nan_output
```
