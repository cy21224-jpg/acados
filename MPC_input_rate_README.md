# MPFC入力変化率コストの追加

最新に添付されたMPC Pythonを基準に、初段に入力変化率の評価を追加した。既存の状態方程式、負位置で反力0の接触モデル、力・釣合い・入力・接近コストとその設定は維持している。

```python
self.R_delta = 1.0e-3
```

追加項は `R_delta * ((u0 - u_applied) / T)**2`。acadosのNONLINEAR_LSとEULER積分の係数を含む実際の初段コストは `0.5*T*R_delta*((u0-u_applied)/T)**2` となる。T=0.010 sでは、R_delta=0.001は差分二乗の重み10に相当する（さらに0.5*Tが掛かる）。0にすると追加項を無効化でき、大きくすると前回の適用指令から動きにくくなる。目標力への応答も遅くなるため、最適値を確定した値ではない。

「適用済み」はTIM7が加速度参照へ採用したMPC力指令を意味する。実測接触力や、DOB補償・DAC飽和後の実アクチュエータ力ではない。計算成功だけでこの履歴を更新しない。計算開始時のスナップショットを参照とし、そのsolve中は一定に保つ。

状態は3、入力は1、実行時パラメータは質量1個を維持した。初段のコスト出力だけが4→5へ増え、NY0=5、NY=4、NYN=2となる。初段以降の予測入力同士の差分は追加していない。予測区間全体の厳密な隣接入力差分には別の設計が必要であり、この版の範囲には含めない。

## 初回の導入

Pythonだけの差し替えでは、前回の適用入力をSTM32から渡せない。初回だけ同梱のインストーラでMPCアダプタと適用通知を導入する。対象は添付された最新ZIPのMPCファイル構成。

ZIPをWSLの「コード」フォルダへ展開し、その直下に `mpfc_input_rate` フォルダができる配置にする。CubeIDEのビルド・実行とコード自動生成を止めた状態で、次を実行する。

```bash
(
    set -euo pipefail

    cd "/mnt/c/Users/Yngsw0407/OneDrive - 芝浦工業大学 教研テナント (SIC)/ドキュメント/3リンクプロ/コード"
    dst="/mnt/c/Users/Yngsw0407/STM32CubeIDE/workspace_1.18.0/3linc_DRL_EKF_MPFC"

    python3 mpfc_input_rate/install_rate_cost.py "$dst" --check
    python3 mpfc_input_rate/install_rate_cost.py "$dst"

    # 生成先は今の作業フォルダのc_generated_code
    python mpfc_input_rate/DRL_EKF_MPFC_for_3link_3.py
    python3 mpfc_input_rate/update_acados.py "$PWD/c_generated_code" "$dst" --check
    python3 mpfc_input_rate/update_acados.py "$PWD/c_generated_code" "$dst"
)
```

その後、CubeIDEでRefresh → Clean → Build → 実機へ書き込む。インストーラ直後は古い生成Cと新しいMPCアダプタのコスト次元が異なるため、上記の生成・同期まで完了してからビルドする。

インストーラは `MPC_function.c/.h` と `MPC_param.c/.h` の4ファイルを更新し、main.cのMPC指令適用箇所2か所に通知だけを追加する。main.c全体を置き換えない。.ioc、TIM、NVIC、クロック、EKF、DOB/DAC処理、位置制御ゲイン、ライブラリ、IDE設定は更新しない。既存MPCファイルが添付版から編集されていたら上書きせず止まる。main.cの対象外編集は保持する。バックアップはプロジェクト外の隣接フォルダに作成される。異なる版で停止した場合は強制的にチェックを外さず、差分を合わせる。

## 2回目以降

`mpfc_input_rate/DRL_EKF_MPFC_for_3link_3.py` の重みやNを変更したら、次の生成・同期を繰り返す。初回インストーラの再実行は不要。過去の6ファイル固定型や1 N固定型の更新スクリプトは使わず、このZIPの `update_acados.py` を使う。

```bash
(
    set -euo pipefail

    cd "/mnt/c/Users/Yngsw0407/OneDrive - 芝浦工業大学 教研テナント (SIC)/ドキュメント/3リンクプロ/コード"
    dst="/mnt/c/Users/Yngsw0407/STM32CubeIDE/workspace_1.18.0/3linc_DRL_EKF_MPFC"

    python mpfc_input_rate/DRL_EKF_MPFC_for_3link_3.py
    python3 mpfc_input_rate/update_acados.py "$PWD/c_generated_code" "$dst" --check
    python3 mpfc_input_rate/update_acados.py "$PWD/c_generated_code" "$dst"
)
```

更新スクリプトの `--check` は確認のみ。通常実行でも書き換えるのはacadosの生成C/Hだけで、コスト用のC/Hも含める。今回の追加項では初段コストの3つの生成Cも変わるため、ソルバーC/Hだけの転送では不十分。古いアダプタ・適用通知なしのmain.c・旧コスト次元との混在を検出する。

Pythonを生成だけでなくシミュレーションにも使う場合は、実際に採用した値を `controller.notify_applied_input(applied_u)` で通知するか、`compute_optimal_control_acados(..., previous_applied_input=applied_u)` として渡す。`u_seq` は初期guess専用であり、適用履歴には使わない。保持中は通知不要、適用を0へ切り替えた場合は0を通知する。STM32では同等の処理を `MPC_NotifyAppliedInput` で行う。

デバッガでは `mpc_applied_input` がTIM7の採用値、`mpc_rate_reference_u` が直近計算開始時に使った値、`mpc_rate_sample_time` が生成T。非同期実行なので計算中に採用値が更新されると、前2者が一時的に違うことは正常。

## 検証結果と範囲

実際のacados v0.5.1で生成・計算し、固定状態による180回の比較で全てstatus=0、入力有限、生成入力範囲[-0.5,1]内だった。N=1でも生成とsolveを確認した。

非接触 `[v,x,k]=[0,-0.002,0]`、M=1 kg、前回適用入力0 Nの比較では、先頭入力は次のように変わった。各条件で固定状態に対するRTIを15回実行した結果であり、実機の時系列ではない。

| R_delta | u0 [N] |
|---:|---:|
| 0 | 0.386837 |
| 0.001 | 0.107761 |
| 0.01 | 0.014382 |

前回指令0.8 Nでも、重みを増やすと0.8 Nへ近づくことを確認した。Cアダプタの5テストで、参照の既存4成分の保存、適用値の反映、非有限通知の拒否、計算成功/失敗時に履歴を勝手に進めないことを確認した。更新スクリプト・初回インストーラの検証結果も同梱している。

STM32F767ZI向けに最新プロジェクトの全53ソースを-O2でコンパイルし、添付のacados/HPIPM/BLASFEOライブラリとリンク成功。ツールチェーンはarm-none-eabi-gcc14.2.1を使用した。CubeIDE実環境での最終ビルドと実機動作は別途必要。

この項はソフトな重みで、入力変化率の上限制約ではない。異常時の0出力への切替は抑制しない。分母Tは公称予測刻みで、非同期処理の実適用間隔を測定した値ではない。実機の10 ms以内の完了・接触力追従・安定性は未検証。前の解析で見つかったDAC飽和/DOBとの指令不一致は別問題として残り、この追加で解消しない。
