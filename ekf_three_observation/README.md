# function_kalmanfilter.c / EKF2 に合わせたPython EKF

最後に添付されたPython全体を修正元に、添付 `function_kalmanfilter.c` の `EKF2` に対応させた。Pythonの公開関数名は従来どおり `EKF`。ファイル名・クラス名も `DRL_EKF_func_series.py` / `DRL_EKF_func_series` のまま。

## 変更点

- 状態は `[速度, 位置, 環境剛性]` の3成分を維持する。
- 観測を `[速度, 位置, 環境反力]` の3成分へ変更し、`y_k` は3要素、`Rkf` は3×3とする。
- 状態予測は `Bd=diag(T/M,0,0)` を使い、有限の入力のうち `u_k_1[0]` だけを決定論的な状態予測へ入れる。
- 雑音共分散の予測は別の `Bd_KF=diag(T/M,T,T)` を使い、`Ad*P*Ad.T + Bd_KF*Q*Bd_KF.T` とする。
- 観測ヤコビアンは予測後の状態から `H=[[1,0,0],[0,1,0],[0,k_minus,x_minus]]` とする。
- 観測予測は `h(x_minus)=[v_minus,x_minus,k_minus*x_minus]` を直接計算する。`H*x_minus` は使わない。
- 事後共分散はCどおり `(I-K*H)*P_minus` とする。
- 各入力、作業配列、行列積の累積をfloat32に揃え、Cと同じループ順で計算する。逆行列補助関数の閾値比較は、Cのfloat→double昇格と同じ判定にする。

添付Cは負位置でも符号付き `k*x` を使うため、このEKFにも接触判定や0クリップを追加していない。以前のMPC用Pythonの片側接触モデルは今回の更新対象ではない。

Cは `inv_mat` の失敗時にも途中まで計算された逆行列を使って処理を続ける。この挙動も揃え、旧Pythonにあった逆行列失敗時の例外や質量の正値・有限性検査は外した。NaN/InfはCと同じように伝播する。配列形状が不正な場合だけPythonの `ValueError` とする。これらの挙動はCへの一致を目的としたもので、特異行列や非有限値への対処を追加した版ではない。

## 呼び出し方

```python
from DRL_EKF_func_series import DRL_EKF_func_series
import numpy as np

kf = DRL_EKF_func_series()
kf.T = 0.010  # 実際のC側 sampling_time と同じ値にする

# y_k[2]は、C側へ渡すのと同じ環境反力の観測値を使う。
y_k = np.array([velocity, position, observed_reaction_force])
u_k_1 = np.array([previous_force_input, 0.0, 0.0])

x_hat_k, P_k = kf.EKF(
    y_k, u_k_1, previous_equivalent_mass,
    x_hat_k_1, P_k_1, Rkf, Qkf,
)
```

`u_k_1` と `x_hat_k_1` は3要素、`P_k_1`・`Rkf`・`Qkf` は3×3。ベクトルは列ベクトルも受け取る。戻り値は従来と同じ状態リスト・共分散リストで、入力配列は変更しない。次回は返された値を前回値として渡す。

コンストラクタの `Rkf` は既存の `1e-6` スケールを引き継ぎ、3×3へ拡張した。添付CはR/Qの初期数値を定めていないため、これが実機の初期値と一致するとの主張はしない。実機と比較するときは、実際にCへ渡している `Rkf`・`Qkf`・初期状態・初期共分散・質量・周期・観測値をそのままPythonへ渡す。3番目の観測反力にも、Cと同じ符号・単位・位置原点を使う。

## 変更範囲

PythonのEKF、コンストラクタのRkf次元、逆行列補助関数の閾値比較だけを変更した。その他のLPF、ノイズ生成、行列補助関数、R2R設定は保持した。STM32のCファイル、main.c、.ioc、タイマー、MPCの評価関数や生成コードは今回変更していない。

## 検証

添付Cファイル自体を変更せず、直近に共有されたSTM32プロジェクトの `RIM_functions.c/.h` とネイティブコンパイルしてPythonと比較した。ソースのSHA-256と詳細結果は `comparison_results.json` に記録している。

ランダムな相関共分散・非ゼロ観測残差・符号付き位置/剛性・周期/質量の変更を含む500ケースと、300ステップの独立した状態/共分散の繰り返し更新を検証した。`-O0` / `-O2` と `-ffp-contract=off` の条件で、状態と共分散がビット単位で一致した。非線形反力観測、予測後ヤコビアン、入力と雑音の行列分離、特異行列、NaN/Inf、質量0/負値も個別に比較した。

これはホスト上の同一演算条件での一致確認。STM32のFMA融合、fast-math、異なる浮動小数点設定によっては末尾ビットが異なるため、全コンパイラ・全実機条件でのビット一致を保証するものではない。

検証スクリプトは同フォルダの `test_c_equivalence.py`。実行用のパス指定はそのファイル冒頭を参照する。PythonにはNumPy、SciPy、Matplotlib、CasADiが必要で、C比較にはCコンパイラと元のC/行列補助ソースが必要。

再実行例（リポジトリのルートから）:

```bash
EKF_PYTHON_FILE="$PWD/DRL_EKF_func_series.py" \
EKF_C_FILE="/path/to/function_kalmanfilter.c" \
EKF_STM32_PROJECT="/path/to/3linc_DRL_EKF_MPFC" \
python3 ekf_three_observation/test_c_equivalence.py
```

テスト用の共有ライブラリ、ヘッダー、ログはスクリプトと同じフォルダへ作成する。元のC/STM32プロジェクトを変更しない。
