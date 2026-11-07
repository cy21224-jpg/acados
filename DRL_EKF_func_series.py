import numpy as np
import math
import matplotlib.pyplot as plt
import casadi
from scipy.linalg import solve_discrete_are
from scipy import signal
from scipy.signal import lfilter, lfilter_zi

class DRL_EKF_func_series:
    def __init__(self):
        #LPFパラメータ
        self.pred_sample=10
        self.samplerate=500
        self.fp = 30     #通過域端周波数[Hz]
        self.fs = 20     #阻止域端周波数[Hz]
        self.gpass = 3       #通過域端最大損失[dB]
        self.gstop = 40      #阻止域端最小損失[dB]
        self.fn = self.samplerate / 2                           #ナイキスト周波数
        self.wp = self.fp / self.fn                                  #ナイキスト周波数で通過域端周波数を正規化
        self.ws = self.fs / self.fn                                  #ナイキスト周波数で阻止域端周波数を正規化
        self.N, self.Wn = signal.buttord(self.wp, self.ws, self.gpass, self.gstop)  #オーダーとバターワースの正規化周波数を計算
        self.b, self.a = signal.butter(self.N, self.Wn, "low")            #フィルタ伝達関数の分子と分母を計算
        self.zi = None

        #LPF2
        self.fc=20
        self.Ts=1/self.samplerate
        self.omega_n=2*np.pi*self.fc
        self.alpha=(self.Ts*self.omega_n)/(1+self.Ts*self.omega_n)
        self.y_prev=None

        self.x0_guess = None
        # R2Rのパラメータ2*0.01
        self.web_wide = 200*1e-3  # ウェブの幅:m
        self.web_density=1800 #ウェブの密度：kg/m^3 ”””今回は1.3g/cm^3~1.45g/cm^3のため中央値の1.4g/cm^3を代表値として使用”””
        self.Winding_core_weight=0.534 # ウェブの巻き枠の重さ:kg
        self.Winding_core_radius = 41*0.001  # ウェブの巻き枠の半径:m
        # webの仮想質量：ウェブの円柱(pi*ウェブ半径^2*ウェブの幅)×ウェブの密度
        self.web_weight_virtual=(self.Winding_core_radius*self.Winding_core_radius*np.pi*self.web_wide*self.web_density)

        self.k_web1=1e-4
        self.d_web1=0.0005
        self.k_web2=1e-9
        self.d_web2=0.002
        self.M_1, self.M_2, self.J_1, self.J_2, self.m1, self.m2 = 130.0, 130.0,  2.36, 3.42, 0.001, 0.001
        self.r1 = 0.271776917
        self.r2 = 0.327042236

        # EKF周期[s]: 共有済みSTM32の sampling_time=10.0e-3f に対応。
        self.T = 0.010


        # 既存の雑音スケールを維持し、EKF2の観測次元3へ拡張。
        # 状態 [速度, 位置, 剛性]、観測 [速度, 位置, 反力]。
        self.Qkf = 1.0e-3 * np.eye(3)
        self.Rkf = 1.0e-6 * np.eye(3)
        self.x_init = casadi.DM([0.0, 0.0, 0.0])

        # 既存のシミュレーション用設定。
        self.f_inp = 1.5 * 1e3
        # Ad/Bd/CdはEKF内で、その回の状態・等価質量から作成する。

        # 得られた角周波数 omega (rad/s)
        self.omega = np.array([2 * np.pi * 1, 2 * np.pi * 5, 2 * np.pi * 10])
        # 2. 振幅 A = 20^(-3) = 0.000125
        self.amplitude = 20**(-2)  # 20**(-3) と書くだけで 0.000125 に計算されます
        # 3. 各sin波の初期位相をランダム設定
        self.phases = np.random.uniform(0, 2 * np.pi, size=len(self.omega))




    def LPF2(self,x):
        if self.y_prev is None:
            self.y_prev = np.zeros_like(x)

        y=self.alpha*x+(1-self.alpha)*self.y_prev

        self.y_prev=y.copy()

        return y


    def LPF(self, x_now):
        if self.zi is None:
            self.zi = np.array([
                lfilter_zi(self.b, self.a) * x_now[i]
                for i in range(len(x_now))
            ])

        y = np.zeros_like(x_now)
        for i in range(len(x_now)):
            y[i], self.zi[i] = lfilter(
                self.b, self.a,
                [x_now[i]],
                zi=self.zi[i]
            )
        return y

    def noise(self, t):
        # A * sin(w * t + phi) の重ね合わせ
        sin_waves = sum(
            self.amplitude * np.sin(w * t + phi) 
            for w, phi in zip(self.omega, self.phases)
        )

        # 信号スケール (0.000125) に合わせて背景ノイズも調整
        # (※正弦波成分のみで十分な場合は 0 にしてください)
        noise_level = self.amplitude * 0.1  # 約 0.0000125
        
        noise_shape = np.shape(t)
        gaussian_noise = np.random.randn(*noise_shape) if noise_shape else np.random.randn()

        return sin_waves + noise_level * gaussian_noise


    @np.errstate(all="ignore")
    def EKF(self, y_k, u_k_1, M_wy_k_1, x_hat_k_1, P_k_1, Rkf, Qkf):
        """function_kalmanfilter.c の EKF2 をfloat32で同じ順序で実行する。

        状態: [速度, 位置, 環境剛性]、観測: [速度, 位置, 環境反力]。
        y_k/u_k_1/x_hat_k_1: (3,)、P_k_1/Rkf/Qkf: (3,3)。
        ベクトルは(3,1)も可。self.TはCのsampling_timeに合わせる。
        Bdは力u[0]だけを予測へ入れ、Bd_KFは3成分の雑音を写像する。
        観測予測はh(x_minus)=[v_minus,x_minus,k_minus*x_minus]。
        添付Cと同じく負位置でも符号付きk*xを使い、クリップしない。
        行列補助関数へfloat32配列を渡し、各演算・累積でfloatへ丸める。
        戻り値は従来と同じ(x_hat_kのリスト, P_kのリスト)。入力は不変。

        Cは逆行列の失敗を検査せず続行するため、この関数も同じ扱い。
        数値の有限性・質量の正値検査は追加しない。NaN/Infも伝播する。
        Python側の形状検査だけを行う。FMA等の実機コンパイラ差は別途。
        """
        y_k = np.asarray(y_k, dtype=np.float32).reshape(-1)
        u_k_1 = np.asarray(u_k_1, dtype=np.float32).reshape(-1)
        x_hat_k_1 = np.asarray(x_hat_k_1, dtype=np.float32).reshape(-1)
        P_k_1 = np.asarray(P_k_1, dtype=np.float32)
        Rkf = np.asarray(Rkf, dtype=np.float32)
        Qkf = np.asarray(Qkf, dtype=np.float32)
        for name, value, shape in (
            ("y_k", y_k, (3,)),
            ("u_k_1", u_k_1, (3,)),
            ("x_hat_k_1", x_hat_k_1, (3,)),
            ("P_k_1", P_k_1, (3, 3)),
            ("Rkf", Rkf, (3, 3)),
            ("Qkf", Qkf, (3, 3)),
        ):
            if value.shape != shape:
                raise ValueError(f"{name} は形状{shape}で指定してください")
        M_wy_k_1 = np.float32(M_wy_k_1)
        T = np.float32(self.T)
        dot_y_c_k = x_hat_k_1[0]
        y_c_k = x_hat_k_1[1]
        k_env_k = x_hat_k_1[2]

        Ad_KF = np.array([
            [1.0, -(T * k_env_k) / M_wy_k_1, -(T * y_c_k) / M_wy_k_1],
            [T, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float32)
        # 共分散の予測に使う雑音行列。
        Bd_KF = np.array([
            [T / M_wy_k_1, 0.0, 0.0],
            [0.0, T, 0.0],
            [0.0, 0.0, T],
        ], dtype=np.float32)
        # 状態の予測に使う入力行列（u[1], u[2]の係数は0）。
        Bd = np.array([
            [T / M_wy_k_1, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ], dtype=np.float32)

        x_hat_k = np.zeros(3, dtype=np.float32)
        P_k = np.zeros((3, 3), dtype=np.float32)
        Ad_x = np.zeros(3, dtype=np.float32)
        Bd_u = np.zeros(3, dtype=np.float32)
        x_hat_minus_k_KF = np.zeros(3, dtype=np.float32)
        Ad_P = np.zeros((3, 3), dtype=np.float32)
        Ad_KF_tp = np.zeros((3, 3), dtype=np.float32)
        P_minus_k_KF1 = np.zeros((3, 3), dtype=np.float32)
        Bd_KF_tp = np.zeros((3, 3), dtype=np.float32)
        Bd_KF_Qkf = np.zeros((3, 3), dtype=np.float32)
        P_minus_k_KF2 = np.zeros((3, 3), dtype=np.float32)
        P_minus_k_KF = np.zeros((3, 3), dtype=np.float32)
        P_Cd = np.zeros((3, 3), dtype=np.float32)
        Cd_KF_tp = np.zeros((3, 3), dtype=np.float32)
        Cd_P = np.zeros((3, 3), dtype=np.float32)
        Cd_P_Cd_tp = np.zeros((3, 3), dtype=np.float32)
        den = np.zeros((3, 3), dtype=np.float32)
        den_inv = np.zeros((3, 3), dtype=np.float32)
        g_k_KF = np.zeros((3, 3), dtype=np.float32)
        Cd_KF_x = np.zeros(3, dtype=np.float32)
        y_Cd_x = np.zeros(3, dtype=np.float32)
        g_k_y_Cd_x = np.zeros(3, dtype=np.float32)
        I = np.zeros((3, 3), dtype=np.float32)
        g_k_Cd = np.zeros((3, 3), dtype=np.float32)
        I_g_k_Cd = np.zeros((3, 3), dtype=np.float32)

        # --- 推定ステップ: Cと同じ非線形Euler予測 ---
        Ad_x[0] = dot_y_c_k - (T * k_env_k * y_c_k) / M_wy_k_1
        Ad_x[1] = T * dot_y_c_k + y_c_k
        Ad_x[2] = k_env_k
        self.mat_vec_mul(3, 3, Bd, u_k_1, Bd_u)
        self.add_vec(3, Ad_x, Bd_u, x_hat_minus_k_KF)

        # 観測ヤコビアンは予測後の位置・剛性から作る。
        Cd_KF = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, x_hat_minus_k_KF[2], x_hat_minus_k_KF[1]],
        ], dtype=np.float32)

        # --- 事前共分散: Ad*P*Ad.T + Bd_KF*Q*Bd_KF.T ---
        self.mul_mat(3, 3, 3, 3, Ad_KF, P_k_1, Ad_P)
        self.transpose_mat(3, 3, Ad_KF, Ad_KF_tp)
        self.mul_mat(3, 3, 3, 3, Ad_P, Ad_KF_tp, P_minus_k_KF1)
        self.transpose_mat(3, 3, Bd_KF, Bd_KF_tp)
        self.mul_mat(3, 3, 3, 3, Bd_KF, Qkf, Bd_KF_Qkf)
        self.mul_mat(3, 3, 3, 3, Bd_KF_Qkf, Bd_KF_tp, P_minus_k_KF2)
        self.add_mat(3, 3, P_minus_k_KF1, P_minus_k_KF2, P_minus_k_KF)

        # --- 更新ステップ: カルマンゲイン ---
        self.transpose_mat(3, 3, Cd_KF, Cd_KF_tp)
        self.mul_mat(3, 3, 3, 3, P_minus_k_KF, Cd_KF_tp, P_Cd)
        self.mul_mat(3, 3, 3, 3, Cd_KF, P_minus_k_KF, Cd_P)
        self.mul_mat(3, 3, 3, 3, Cd_P, Cd_KF_tp, Cd_P_Cd_tp)
        self.add_mat(3, 3, Cd_P_Cd_tp, Rkf, den)
        # 添付CのEKF2と同じく戻り値で分岐しない。
        self.inv_mat(3, den, den_inv)
        self.mul_mat(3, 3, 3, 3, P_Cd, den_inv, g_k_KF)

        # H*xではなく非線形観測h(x)を使う。
        Cd_KF_x[0] = x_hat_minus_k_KF[0]
        Cd_KF_x[1] = x_hat_minus_k_KF[1]
        Cd_KF_x[2] = x_hat_minus_k_KF[2] * x_hat_minus_k_KF[1]
        self.sub_vec(3, y_k, Cd_KF_x, y_Cd_x)
        self.mat_vec_mul(3, 3, g_k_KF, y_Cd_x, g_k_y_Cd_x)
        self.add_vec(3, x_hat_minus_k_KF, g_k_y_Cd_x, x_hat_k)

        # --- 事後共分散: Cと同じ(I-K*H)*P_minus ---
        self.eye_mat(3, I)
        self.mul_mat(3, 3, 3, 3, g_k_KF, Cd_KF, g_k_Cd)
        self.sub_mat(3, 3, I, g_k_Cd, I_g_k_Cd)
        self.mul_mat(3, 3, 3, 3, I_g_k_Cd, P_minus_k_KF, P_k)

        return x_hat_k.tolist(), P_k.tolist()


    def custom_lpf_step(self, x, y_prev, cutoff_freq, dt):
        """NumPyのみで実装する1次IIRローパスフィルタ（1ステップ分）

        y[k] = (1 - alpha) * y[k-1] + alpha * x[k]
        """
        tau = 1.0 / (2.0 * np.pi * cutoff_freq)
        alpha = dt / (tau + dt)

        # 1ステップ分の計算
        y = y_prev + alpha * (x - y_prev)

        return y

    def add_vec(self,N, A, B, C):
        for i in range(N):
            C[i] = A[i] + B[i]


    def sub_vec(self,N, A, B, C):
        for i in range(N):
            C[i] = A[i] - B[i]


    def scale_vec(self,N, A, C, k):
        for i in range(N):
            C[i] = k * A[i]


    def norm_vec(self,N, A):
        result = 0.0
        for i in range(N):
            result += A[i] * A[i]
        return math.sqrt(result)


    def normalize_vec(self,N, A, B):
        norm = norm_vec(N, A)

        if norm < 1e-9:
            print("ゼロベクトルは正規化できません")
            return -1

        for i in range(N):
            B[i] = A[i] / norm

        return 0


    def dot_vec(self,N, A, B):
        result = 0.0
        for i in range(N):
            result += A[i] * B[i]
        return result


    def cross_vec(self,A, B, C):
        C[0] = A[1] * B[2] - A[2] * B[1]
        C[1] = A[2] * B[0] - A[0] * B[2]
        C[2] = A[0] * B[1] - A[1] * B[0]


    def copy_vec(self,N, src, dest):
        for i in range(N):
            dest[i] = src[i]


    def eye_mat(self,N, A):
        for i in range(N):
            for j in range(N):
                A[i][j] = 1.0 if i == j else 0.0


    def add_mat(self,row, col, A, B, C):
        for i in range(row):
            for j in range(col):
                C[i][j] = A[i][j] + B[i][j]


    def sub_mat(self,row, col, A, B, C):
        for i in range(row):
            for j in range(col):
                C[i][j] = A[i][j] - B[i][j]


    def mat_vec_mul(self,row, col, A, x, y):
        for i in range(row):
            y[i] = 0.0
            for j in range(col):
                y[i] += A[i][j] * x[j]


    def mul_mat(self,rowA, colA, rowB, colB, A, B, C):
        if colA != rowB:
            print("行列のサイズが合いません")
            return -1

        for i in range(rowA):
            for j in range(colB):
                C[i][j] = 0.0
                for k in range(colA):
                    C[i][j] += A[i][k] * B[k][j]

        return 0


    def copy_mat(self,row, col, src, dest):
        for i in range(row):
            for j in range(col):
                dest[i][j] = src[i][j]


    def transpose_mat(self,row, col, A, B):
        for i in range(row):
            for j in range(col):
                B[j][i] = A[i][j]


    # 参考URL: https://imagingsolution.net/math/gauss-jordan-with-pivot/
    def inv_mat(self,N, A, inv):
        # temp と inv の初期化（temp = A, inv = 単位行列）
        temp = [[0.0] * N for _ in range(N)]

        for i in range(N):
            for j in range(N):
                temp[i][j] = A[i][j]
                inv[i][j] = 1.0 if i == j else 0.0

        for i in range(N):
            # ピボット選択
            max_row = i
            for k in range(i + 1, N):
                if abs(temp[k][i]) > abs(temp[max_row][i]):
                    max_row = k

            # 行交換
            if max_row != i:
                for j in range(N):
                    temp[i][j], temp[max_row][j] = temp[max_row][j], temp[i][j]
                    inv[i][j], inv[max_row][j] = inv[max_row][j], inv[i][j]

            # 逆行列が存在しないチェック
            if float(abs(temp[i][i])) < 1e-14:
                print("逆行列が存在しません")
                return -1

            # ピボットを1にする
            pivot = temp[i][i]
            for j in range(N):
                temp[i][j] /= pivot
                inv[i][j] /= pivot

            # 他の行を0にする
            for k in range(N):
                if k == i:
                    continue

                factor = temp[k][i]
                for j in range(N):
                    temp[k][j] -= factor * temp[i][j]
                    inv[k][j] -= factor * inv[i][j]

        return 0