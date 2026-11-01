# WSLからMPCの依存ファイルを追加する

今回は移植準備とビルド確認までです。既存の `main.c`、制御式、モータ出力、`.cproject`、`.ioc`、リンカスクリプト、クロック・タイマ・DMAの設定を書き換えません。MPC関数をどこから呼ぶか、4状態の対応、出力の接続は使用者が後で実装します。

## 実行手順

WSL（Ubuntu）で必要なツールを入れます。既にある場合は再インストール不要です。

```bash
sudo apt update
sudo apt install -y curl unzip python3 make gcc-arm-none-eabi libnewlib-arm-none-eabi libnewlib-dev
```

空いている作業フォルダでセットアップZIPを取得します。

```bash
mkdir -p ~/stm32-mpc-import
cd ~/stm32-mpc-import
curl -fL https://github.com/cy21224-jpg/acados/raw/refs/heads/main/MPC_WSL_setup.zip -o MPC_WSL_setup.zip
unzip -q MPC_WSL_setup.zip
```

**次のパスだけ、実際のSTM32プロジェクトの場所に置き換えてください。** プロジェクトは `Core/Src/main.c` と `.cproject` があるフォルダです。CubeIDEでビルドやコード生成をしていない状態で実行します。

```bash
mpc_project='/mnt/c/Users/ユーザー名/STM32CubeIDE/workspace_1.18.0/3linc_DRL_EKF_MPFC'
python3 MPC_WSL_setup/setup_mpc_wsl.py "$mpc_project" --build
```

Gitリポジトリを既に取得している場合は、その中から直接実行できます。

```bash
python3 tools/setup_mpc_wsl.py "$mpc_project" --build
```

再度ビルドするときは次のコマンドです。

```bash
make -C "$mpc_project" -f MPC_build.mk -j4 all mpc-link-check
```

## 追加されるもの

- `Core/Inc/MPC_param.h`、`MPC_function.h`：MPCの公開ヘッダ
- `Core/Src/MPC_param.c`、`MPC_function.c`：MPCの実装
- `Core/Inc` 内のacados/HPIPM/BLASFEOヘッダと生成モデルヘッダ
- `Core/Src/acados_generated`：生成されたソルバ・モデル関数
- `MPC_lib`：libacados.a、libhpipm.a、libblasfeo.a
- `makefile.defs`：CubeIDE生成Makefileが既に用意している追加フックで、MPCのリンク入力だけを追加
- `MPC_build.mk`：WSL用の追加Makefile。元のDebug構成から定義・インクルード・最適化・デバッグ・リンカスクリプトを読み取る
- `.mpc_import_manifest.json`：追加ファイルを識別するハッシュ情報
- `MPC_build`：WSLのビルド出力。元のDebug/Releaseの成果物は上書きしない

既存の `Core/Inc` インクルード設定を使うため、`.cproject` のInclude path編集は不要です。一部のベンダーヘッダの短い名前に対しては、Core/Incに転送ヘッダを追加します。元のヘッダは変更しません。

CubeIDEから使う場合は追加後にRefreshとClean Buildを行ってください。Core/Driversの既存ソース探索設定で新しいCファイルが見つかり、生成Makefileの `makefile.defs` フックからライブラリが追加されます。`.ioc` のコード再生成は不要です。今回クラウドで検証したのはWSL用GNU Makeビルドで、CubeIDE自体でのGUIビルドは未実行です。

同名の既存ファイルに異なる内容があれば、何も追加する前に停止します。再実行時は同じ内容を再利用し、ユーザーが編集した追加ファイルを勝手に上書きしません。今回は `3linc_DRL_EKF_MPFC.zip` に入っていた元の255ファイルすべてについて変更0件を確認しました。

## ビルド結果の意味

- `MPC_build/3linc_DRL_EKF_MPFC.elf`：元のアプリケーション。mainを書き換えていないので、MPCはまだ呼ばれない
- `MPC_build/MPC_link_check.elf`：MPC APIを強制的にリンクし、未使用コード除去で問題が隠れないようにしたリンク確認用ELF。mainの動作をMPCへ置き換えるものではない

成功時には `MPC_Init`、`MPC_Execute`、制約更新APIのシンボル一覧が出ます。GCC/newlibの版によって未使用変数や `_gettimeofday` 未実装の警告が出る場合があります。acados内部のPOSIX計時はこの追加作業で変更していません。

添付されたプロジェクトのDebug最適化は未指定（-O0）だったため、WSLビルドもそれを既定にしています。必要なら新しいWSLビルドだけに `MPC_OPT=-O2` を指定できます。元のIDE設定は変わりません。

**最適化の指定を変更するときは、別の出力先を使ってください。** Makeは通常、コマンドラインのフラグ変更だけでは既存オブジェクトを再コンパイルしません。

```bash
make -C "$mpc_project" -f MPC_build.mk -j4 MPC_BUILD_DIR=MPC_build_O2 MPC_OPT=-O2 all mpc-link-check
```

MPC自体のAPIはARMエミュレーションで20回の計算、状態固定、ヒープ増加なしを確認済みです。実機・既存モータ制御・割込みの重なりは未検証です。元のスタック予約0x400もそのまま保持しています。MPCを実際に呼び出す段階では、元の予約値を超えるスタック使用があり得るため、ヒープ・スタック余裕と割込み時の使用量を確認してください。今回の独立APIテストでは約1864 bytesのスタック使用と約227 kBのヒープからの余裕を観測しました。

## 次に行う作業

1. 状態4値を確定する。
2. 元の設定を維持したまま、使用者が `MPC_Init` / `MPC_UpdateInitialState` / `MPC_Execute` を接続する。
3. `MPC_SetTickProvider(HAL_GetTick)` を登録すれば、従来と同じmsの計算時間表示を使える。
4. 実機で周期・計算時間・制約・出力単位を確認する。

このセットアップにはsin波入力やMPCをタイマ割込みで実行する処理、DACへの出力は追加していません。
