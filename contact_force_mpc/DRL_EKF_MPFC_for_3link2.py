
import numpy as np
import casadi

from acados_template import (
    AcadosOcp,
    AcadosModel,
    AcadosOcpSolver
)


class DRL_EKF_func_series:

    def __init__(self):

        self.x0_guess = None

        # =================================
        # モデルのパラメータ
        # =================================

        self.M_wy = 1.0       # [kg]

        self.nx = 4           # 状態変数の次元
        self.nu = 1           # 制御入力の次元

        # =================================
        # 評価関数の重み
        # =================================

        # 接触力・釣り合いの重み。位置・速度の目標追従は評価しない。
        self.force_ref = 1.0
        self.q_force = 1.0
        self.q_balance = 0.1
        self.q_force_terminal = 1.0
        self.R = casadi.DM.eye(self.nu) * 1.0

        # =================================
        # MPCパラメータ
        # =================================

        self.T = 0.020            # サンプリング時間 [s]
        self.K = 5              # ホライゾン
        self.T_all = 1.0
        self.max_iter = 2

        # =================================
        # 状態制約
        # x = [dot_y_c, y_c, c_env, k_env]
        # =================================

        self.x_lb = [
            -3.0,
            -3.0,
            -200.0,
            -1.0e6
        ]

        self.x_ub = [
            3.0,
            3.0,
            200.0,
            1.0e6
        ]

        # =================================
        # 入力制約
        # =================================

        self.F_MIN = -10.0
        self.F_MAX = 10.0

        self.u_lb = [self.F_MIN]
        self.u_ub = [self.F_MAX]

        # =================================
        # 目標値
        # =================================

        self.x_ref = casadi.DM([
            0.0,
            0.0,
            0.0,
            0.0
        ])

        self.u_ref = casadi.DM([
            1.0
        ])

        # =================================
        # OCP生成
        # =================================

        self.ocp = self.make_ocp()

        self.solver = AcadosOcpSolver(
            self.ocp,
            json_file="acados_ocp.json"
        )


    # =====================================
    # モデル定義
    # =====================================

    def make_model(self):

        # 状態・入力
        x = casadi.SX.sym("x", self.nx)
        xdot = casadi.SX.sym("xdot", self.nx)
        u = casadi.SX.sym("u", self.nu)

        # 実行時パラメータp[0]: 正の等価質量[kg]。制御入力u[0]は力[N]のまま。
        p = casadi.SX.sym("p", 1)
        equivalent_mass = p[0]

        # 状態
        dot_y_c_k = x[0]
        y_c_k = x[1]
        c_env_k = x[2]
        k_env_k = x[3]

        # 入力
        f_c = u[0]

        # 運動方程式
        dot_dot_y_c = (
            f_c
            - c_env_k * dot_y_c_k
            - k_env_k * y_c_k
        ) / equivalent_mass

        dot_y_c = dot_y_c_k

        # 予測中は環境インピーダンス一定
        dot_c_env = 0.0
        dot_k_env = 0.0

        # 非線形状態方程式
        f_expl = casadi.vertcat(
            dot_dot_y_c,
            dot_y_c,
            dot_c_env,
            dot_k_env
        )

        # Acados model
        model = AcadosModel()

        model.x = x
        model.xdot = xdot
        model.u = u
        model.p = p

        model.f_expl_expr = f_expl
        model.f_impl_expr = xdot - f_expl

        model.name = "interaction_model"

        return model


    # =====================================
    # OCP定義
    # =====================================

    def make_ocp(self):

        ocp = AcadosOcp()

        # ---------------------------------
        # モデル
        # ---------------------------------

        ocp.model = self.make_model()

        # ソルバ作成時の初期質量。作成後はupdate_mass()で全予測段へ更新する。
        if not np.isfinite(self.M_wy) or self.M_wy <= 0.0:
            raise ValueError("M_wy must be finite and positive")
        ocp.parameter_values = np.array([self.M_wy], dtype=float)

        # ---------------------------------
        # ホライゾン
        # ---------------------------------

        ocp.dims.N = self.K

        ocp.solver_options.tf = self.K * self.T

        # ---------------------------------
        # コスト関数
        # ---------------------------------

        ocp.cost.cost_type_0 = "NONLINEAR_LS"
        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"

        x = ocp.model.x
        u = ocp.model.u
        f_env = x[2] * x[0] + x[3] * x[1]
        outputs = casadi.vertcat(f_env, u[0] - f_env, u[0])

        ocp.model.cost_y_expr_0 = outputs
        ocp.model.cost_y_expr = outputs
        ocp.model.cost_y_expr_e = casadi.vertcat(f_env)

        W = np.diag([
            self.q_force, self.q_balance, float(self.R[0, 0])
        ])
        yref = np.array([
            self.force_ref, 0.0, float(self.u_ref[0])
        ])
        ocp.cost.W_0 = W.copy()
        ocp.cost.W = W.copy()
        ocp.cost.W_e = np.array([[self.q_force_terminal]])
        ocp.cost.yref_0 = yref.copy()
        ocp.cost.yref = yref.copy()
        ocp.cost.yref_e = np.array([self.force_ref])

        # ---------------------------------
        # ステージコストの離散化
        # ---------------------------------

        ocp.solver_options.cost_discretization = "EULER"

        # ---------------------------------
        # 状態制約
        # ---------------------------------

        ocp.constraints.idxbx = np.arange(
            self.nx,
            dtype=int
        )

        ocp.constraints.lbx = np.array(
            self.x_lb,
            dtype=float
        )

        ocp.constraints.ubx = np.array(
            self.x_ub,
            dtype=float
        )

        # ---------------------------------
        # 終端状態制約
        # ---------------------------------

        ocp.constraints.idxbx_e = np.arange(
            self.nx,
            dtype=int
        )

        ocp.constraints.lbx_e = np.array(
            self.x_lb,
            dtype=float
        )

        ocp.constraints.ubx_e = np.array(
            self.x_ub,
            dtype=float
        )

        # ---------------------------------
        # 初期状態制約
        # ---------------------------------

        ocp.constraints.idxbx_0 = np.arange(
            self.nx,
            dtype=int
        )

        ocp.constraints.lbx_0 = np.array(
            self.x_lb,
            dtype=float
        )

        ocp.constraints.ubx_0 = np.array(
            self.x_ub,
            dtype=float
        )

        # ---------------------------------
        # 入力制約
        # ---------------------------------

        ocp.constraints.idxbu = np.arange(
            self.nu,
            dtype=int
        )

        ocp.constraints.lbu = np.array(
            self.u_lb,
            dtype=float
        )

        ocp.constraints.ubu = np.array(
            self.u_ub,
            dtype=float
        )

        # ---------------------------------
        # ソルバー設定
        # ---------------------------------

        ocp.solver_options.qp_solver = (
            "PARTIAL_CONDENSING_HPIPM"
        )

        ocp.solver_options.hessian_approx = (
            "GAUSS_NEWTON"
        )

        ocp.solver_options.integrator_type = "ERK"

        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.sim_method_num_steps = 10

        ocp.solver_options.nlp_solver_type = "SQP_RTI"

        ocp.solver_options.levenberg_marquardt = 1e-2

        ocp.solver_options.qp_solver_iter_max = 50
        

        return ocp


    # =====================================
    # MPC最適化計算
    # =====================================

    def update_mass(self, equivalent_mass, solver=None):
        """全予測段のp[0]へ等価質量を設定する。モデルの再生成は不要。"""
        mass = float(equivalent_mass)
        if not np.isfinite(mass) or mass <= 0.0:
            raise ValueError("equivalent_mass must be finite and positive")
        active_solver = self.solver if solver is None else solver
        values = np.array([mass], dtype=float)
        for stage in range(self.K + 1):
            active_solver.set(stage, "p", values)
        self.M_wy = mass

    def compute_optimal_control_acados(
        self,
        solver,
        x_init,
        x_ref,
        u_seq,
        equivalent_mass=None
    ):

        # 1回のsolve中では同じ質量を使う。未指定時は最後に設定した質量を使う。
        mass = self.M_wy if equivalent_mass is None else equivalent_mass
        self.update_mass(mass, solver=solver)

        x0 = x_init.full().flatten()

        # 初期状態の次元確認
        if x0.size != self.nx:
            raise ValueError(
                f"x0 dimension error: {x0.size} != {self.nx}"
            )

        if len(u_seq) != self.K:
            raise ValueError(
                f"u_seq length error: {len(u_seq)} != {self.K}"
            )

        # 初期状態固定
        solver.set(0, "lbx", x0)
        solver.set(0, "ubx", x0)

        # 状態予測の初期guess
        for k in range(self.K + 1):
            solver.set(k, "x", x0)

        # 入力系列の初期guess
        for k in range(self.K):
            solver.set(
                k,
                "u",
                np.array(u_seq[k]).flatten()
            )

        # ---------------------------------
        # 目標値
        # ---------------------------------

        yref = np.array([
            self.force_ref, 0.0, float(self.u_ref[0])
        ])
        for k in range(self.K):
            solver.set(k, "yref", yref)
        solver.set(self.K, "yref", np.array([self.force_ref]))

        # ---------------------------------
        # 最適化計算
        # ---------------------------------

        status = solver.solve()

        print("MPC status:", status)

        if status != 0:
            raise RuntimeError(
                f"acados solver failed. status = {status}"
            )

        # ---------------------------------
        # 最適状態系列
        # ---------------------------------

        X_opt = np.zeros(
            (self.nx, self.K + 1)
        )

        for k in range(self.K + 1):
            X_opt[:, k] = solver.get(k, "x")

        # ---------------------------------
        # 最適入力系列
        # ---------------------------------

        U_opt = np.zeros(
            (self.nu, self.K)
        )

        for k in range(self.K):
            U_opt[:, k] = solver.get(k, "u")

        print("X0:", X_opt[:, 0])
        print("X1:", X_opt[:, 1])
        print("U0:", U_opt[:, 0])

        return U_opt[:, 0], X_opt, U_opt


# =========================================
# Main
# =========================================

if __name__ == "__main__":

    print("Pythonプログラム開始")

    controller = DRL_EKF_func_series()

    print("MPCソルバー生成完了")

    print("nx:", controller.nx)
    print("nu:", controller.nu)
    print("N:", controller.K)
    print("T:", controller.T)
    print("tf:", controller.K * controller.T)

    print("M_wy:", controller.M_wy)
    print("force_ref [N]:", controller.force_ref)
    print("q_force:", controller.q_force)
    print("q_balance:", controller.q_balance)
    print("q_force_terminal:", controller.q_force_terminal)
    print("u_ref [N]:", controller.u_ref)
    print("u_lb:", controller.u_lb)
    print("u_ub:", controller.u_ub)