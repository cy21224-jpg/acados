# MPCの等価質量更新・N=5・R=10

## 今回の設定

| 項目 | 設定 |
|---|---|
| 状態 | 速度、位置、環境粘性、環境剛性の4状態 |
| 制御入力 | 力F [N]、1入力 |
| 実行時パラメータ | `p[0]` = 正の等価質量M [kg] |
| 予測刻み | 1 ms |
| 予測段数 | 5（以前は10） |
| 予測時間 | 5 ms |
| 入力重みR | 10（以前は1） |
| 入力参照値 | 3 Nを維持 |
| 入力制約 | 0～10 Nを維持 |
| 状態重み・状態制約・ソルバ方式 | 元の設定を維持 |

Rが罰するのは`(F - 3 N)^2`です。Rを上げると3 Nからのずれを強く抑えます。
力の絶対値を0へ寄せる設定には変更していません。

```text
v_dot = (F - c*v - k*x) / M
x_dot = v
c_dot = 0
k_dot = 0
```

元のPythonの`self.M_wy = 1.0`は生成時の数値定数でした。
修正版では`model.p`のシンボルにし、生成時の初期値を`ocp.parameter_values`で1 kgに設定します。
以後はCの`MPC_UpdateModelMass()`、Pythonの`update_mass()`でソルバを作り直さず変更できます。
Pythonの`self.M_wy`という属性名・生成モデル名は互換性のため維持しています。
X方向に使う際はX方向の状態・質量を接続します。

## STM32への導入

`MPC_function.h/.c`、`MPC_param.h/.c`、生成ソルバ・モデルのC/ヘッダーをセットで更新します。
`INTERACTION_MODEL_NP`は1、`INTERACTION_MODEL_N`は5になります。
マクロだけを書き換えて古いODE/VDEと混在させてはいけません。

ZIPには、既存ライブラリに対応する生成C一式も含めています。
OCPラッパーは以前動作した生成CのAPIと構成を基に、NP・初期質量・N・Rを更新しています。
ODE・順方向VDE・随伴VDEは修正したモデルからCasADi 3.7.2で再生成しています。
このクラウドではacados_template全体による生成は行っていません。
修正版Pythonを元と同じacados環境で実行して、一式を再生成する方法も使えます。

旧来の`.a`をそのまま使う構成でARMリンクを確認しています。
main、EKF、3リンクのパラメータファイル、.ioc、.cproject、リンカ、タイマー、クロックは自動更新しません。
最新の添付mainを基に接続した参考版`main.c`と`main.diff`も同梱しています。
main側の変更は、X方向への観測統一、EKFの4状態の使用、質量更新APIの呼出し、
DWT計測、今回設定した質量での出力変換です。周辺機器・クロック・タイマ初期化関数は元と同一です。
手元のmainを添付後に変更している場合は丸ごと上書きせず、差分だけ反映してください。
手元の最新の3リンク/EKF/RIMヘッダーは今回未添付のため、参考mainとの全体ビルドは実機側で確認が必要です。

### バックアップ付き導入スクリプト

ZIPをWindowsのDownloadsへ保存し、CubeIDEを終了してからWSLで実行します。

```bash
mpc_project="/mnt/c/Users/Yngsw0407/STM32CubeIDE/workspace_1.18.0/3linc_DRL_EKF_MPFC"
mpc_package=$(mktemp -d /tmp/mpc-runtime-mass.XXXXXX)
python3 -m zipfile -e "/mnt/c/Users/Yngsw0407/Downloads/MPC_runtime_mass.zip" "$mpc_package"
python3 "$mpc_package/install_runtime_mass.py" "$mpc_project" --check
python3 "$mpc_package/install_runtime_mass.py" "$mpc_project"
```

確認済みの旧ファイル、または同梱と同一の更新済みファイルであることを全件確認してから更新します。
個別に変更したファイルがある場合は、その変更を上書きせず停止します。古い版へ戻して強制適用せず、
現在の変更を保った差分適用が必要です。
バックアップはプロジェクト直下`.mpc-backups/runtime_mass_<日時>/`に保存します。
`MPC_timing.h`だけは新規追加です。mainへは`main_integration.txt`に従って接続するか、
同梱`main.c`との差分を反映します。参考mainは、既存のEKF呼出しの引数順を維持しています。
その後、CubeIDEで対象プロジェクトを更新し、ビルドしてください。

## mainからの呼出し順序

1. 正しく計算した正の等価質量と、成功したEKFの4状態を用意します。
2. `MPC_UpdateModelMass(equivalent_mass)`で全6ステージに質量を設定します。
3. 成功したら`MPC_UpdateInitialState(estimated_state)`を呼びます。
4. 成功したら`MPC_Execute()`を呼びます。
5. 今回の更新・計算が成功しているときだけ`mpc_u0`を使用します。

```c
const double equivalent_mass = (double)M_w_eq[0][0];
const double estimated_state[4] = {
    (double)x_hat_k[0], (double)x_hat_k[1],
    (double)x_hat_k[2], (double)x_hat_k[3]
};

mpc_mass_update_status = MPC_UpdateModelMass(equivalent_mass);
if (mpc_mass_update_status == 0) {
    mpc_update_status = MPC_UpdateInitialState(estimated_state);
    if (mpc_update_status == 0)
        mpc_step_status = MPC_Execute();
}
```

質量更新は前の結果と未実行の状態を無効化するので、この順序を維持します。
成功時の力を加速度指令へ変換する場合は、今回設定できた質量を使います。

```c
ddx_w_ref[0] = (float)(mpc_u0 / mpc_model_mass);
```

ソルバは毎回作り直さず、前回入力列のウォームスタートも維持します。
無効な質量では-2を返し、古い結果を無効化します。API同士はmain側で直列に呼び、
同じソルバを割込みとmainから同時に操作しないでください。

元の状態制約（特に粘性の-2～2など）は維持しています。
EKF推定値が予測段の状態制約を満たせない場合、最適化は失敗し得ます。
推定値をこのパッケージが自動で丸めたり、制約を勝手に緩めたりはしません。

## Pythonからの更新

```python
controller.update_mass(current_equivalent_mass)
# または既存のcompute呼出しに任意引数を追加する。
controller.compute_optimal_control_acados(
    solver, x_init, x_ref, u_seq,
    equivalent_mass=current_equivalent_mass,
)
```

既存の引数順は維持し、新しい任意引数だけを末尾に追加しています。
N=5なので`u_seq`は5段分です。質量を変更するたびのコード再生成は不要です。

## 制御周期について

添付mainの設定は、8 MHz HSE・PLL後96 MHz、TIM7プリスケーラ96、周期1000で、
設定上のTIM7周期はすでに1 msです。その設定は変更していません。
Python側も`T=0.001`で、使われていなかった`dt=0.04`を`dt=T`に統一しました。
予測時間5 msと、制御結果を更新したい周期1 msは別の値です。

**MPCの計算結果が実際に毎1 ms得られることは、まだ確認していません。**
Nを5へ減らすことで負荷を下げる方針ですが、1 ms達成を保証する変更ではありません。
既存制御、逆行列、EKF、MPC、DACを含むTIM7処理全体の最悪時間が1 ms以内である必要があります。

`MPC_timing.h`と`main_integration.txt`にDWT計測例を付けています。
既存TIMやCPUクロックを変更せず、MPC更新から結果取得までをus単位で観測します。
`mpc_total_time_us`、`mpc_max_time_us`、`mpc_overrun_count`をデバッガで確認してください。
MPC以外の処理時間は含まないため、TIM7全体も同じ方法で計測する必要があります。
HAL_GetTickによるms計測は高優先度割込み内で進まない場合があるので、1 ms判定には使いません。

直近ビルドの`-O0`は維持しています。高速化の比較をするときは既に合意したOptimizationだけを
`-O3`へ変更して、同じ入力で結果・失敗率・最大実行時間を比較してください。
このパッケージが`.cproject`を変更することはありません。

## 検証範囲

- 実CasADiで、質量に応じて加速度が変わること、入力が力のままであることを確認。
- Python設定でN=5、刻み1 ms、R=10、入力参照3 N、その他の制約・重み・方式の維持を確認。
  Pythonの設定検証ではacados_template部分を代用しており、ホストOCP solveの試験ではありません。
- Cortex-M7/FPv5-D16/hard-floatでC一式をコンパイル・既存の.aとリンク。
- ARMエミュレーションで、6種類の質量へ切り替えながら18回solve。
  全6ステージへの更新、RK4予測状態、入力制約、無効質量の拒否、ヒープ取得量の安定を確認。
- DWT補助ヘッダーのARMコンパイルを確認。DWTの実機動作・最大実行時間は未確認。

物理装置での制御性能、最大スタック使用量、1 ms以内の実行は、実機で確認してください。
