"""
完整版：基于有限控制集的 PMSM 模型预测直接转速控制 (MP-DSC)
包含：延时补偿、开关状态图、负载转矩观测器、MTPA 吸引区
参考文献: Preindl & Bolognani (2013), IEEE Trans. on Power Electronics
"""

import math

import matplotlib.pyplot as plt
import numpy as np


def main() -> None:
    # =========================================================================
    # 1. 系统与电机参数 (摘录自论文 Table I)
    # =========================================================================
    Ts = 100e-6          # 采样时间 100 us
    Udc = 200.0          # 直流母线电压 200 V
    Ir_max = 10.0        # 额定电流限制 10 A

    # IPMSM 参数
    Ld = 12e-3           # d轴电感 12 mH
    Lq = 20e-3           # q轴电感 20 mH
    Rs = 0.636           # 定子电阻 0.636 Ohm
    psi_pm = 0.088       # 永磁体磁链 88 mWb
    p = 5                # 极对数
    J = 1.0e-3           # 转动惯量 kg*m^2
    B_fric = 1.7e-3      # 摩擦系数

    # =========================================================================
    # 2. 控制器参数与权重
    # =========================================================================
    lambda_T = 1.0       # 跟踪代价权重 (Tracking)
    lambda_A = 1e-3      # 吸引区代价权重 (Attraction - MTPA)
    lambda_L = 1e4       # 限制惩罚权重 (Limitations)

    # 负载观测器 (Luenberger Observer) 增益设计
    Lp = 100.0           # 比例增益
    Li = 5000.0          # 积分增益

    # =========================================================================
    # 3. 开关状态图定义 (Switch State Graph - 论文 Fig. 3)
    # =========================================================================
    # 8个基本电压矢量的开关组合 [Sa, Sb, Sc]，索引为 0~7
    states = np.array([
        [0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
        [0, 1, 1], [0, 0, 1], [1, 0, 1], [1, 1, 1]
    ])

    # 允许的跳转字典 (仅允许一个桥臂跳变或保持不变，Python索引从0开始)
    allowed_transitions = {
        0: [0, 1, 3, 5],
        1: [1, 0, 2, 6],
        2: [2, 1, 3, 7],
        3: [3, 2, 0, 4],
        4: [4, 3, 5, 7],
        5: [5, 4, 0, 6],
        6: [6, 5, 1, 7],
        7: [7, 6, 2, 4]
    }

    # =========================================================================
    # 4. 初始化仿真环境变量
    # =========================================================================
    T_sim = 0.4          # 仿真总时间 0.4s
    N_steps = int(T_sim / Ts)
    time = np.arange(N_steps) * Ts

    # 数据记录数组
    id_log = np.zeros(N_steps)
    iq_log = np.zeros(N_steps)
    w_log = np.zeros(N_steps)
    Te_log = np.zeros(N_steps)
    w_ref_log = np.zeros(N_steps)
    Tl_hat_log = np.zeros(N_steps)

    # 电机真实物理状态
    id, iq, omega_m, theta_e, Tl = 0.0, 0.0, 0.0, 0.0, 0.0
    # 观测器与控制器状态记忆
    omega_hat, Tl_hat = 0.0, 0.0
    last_state_idx = 0   # 初始开关状态设为 000 (索引0)
    u_d_prev, u_q_prev = 0.0, 0.0

    # =========================================================================
    # 5. 主控制与仿真循环
    # =========================================================================
    for k in range(N_steps):

        # --- A. 设定参考值与外部扰动 ---
        # 转速阶跃 0 -> 500 rpm
        w_ref = 0.0 if time[k] < 0.05 else 500.0 * (2 * math.pi / 60)
        # 在 t=0.2s 突加 6 Nm 负载
        Tl = 0.0 if time[k] < 0.2 else 6.0
        w_ref_log[k] = w_ref

        # --- B. 测量与观测器 (Disturbance Rejection - Eq. 5) ---
        v_p = omega_m - omega_hat
        Te_est = 1.5 * p * (psi_pm * iq + (Ld - Lq) * id * iq)
        # 更新预测模型
        omega_hat += (Ts / J) * (Te_est - B_fric * omega_hat - Tl_hat) + Lp * v_p
        Tl_hat -= Li * v_p * Ts  # 积分校正负载转矩
        Tl_hat_log[k] = Tl_hat

        # --- C. 延时补偿 (Delay Compensation) ---
        omega_e = p * omega_m
        id_k = id + (Ts / Ld) * (u_d_prev - Rs * id + omega_e * Lq * iq)
        iq_k = iq + (Ts / Lq) * (u_q_prev - Rs * iq - omega_e * Ld * id - omega_e * psi_pm)
        wm_k = omega_hat  # 使用观测器的平滑速度作为起点

        # --- D. 模型预测控制 (MP-DSC) 核心寻优 ---
        cost_min = float('inf')
        best_state_idx = last_state_idx
        best_ud, best_uq = 0.0, 0.0

        # 仅遍历开关图中允许的 4 种下一步状态
        valid_next_states = allowed_transitions[last_state_idx]

        for idx in valid_next_states:
            Sa, Sb, Sc = states[idx]

            # Clarke & Park 变换
            Valpha = (2/3) * Udc * (Sa - 0.5 * Sb - 0.5 * Sc)
            Vbeta = (2/3) * Udc * (math.sqrt(3)/2 * Sb - math.sqrt(3)/2 * Sc)
            ud_pred = Valpha * math.cos(theta_e) + Vbeta * math.sin(theta_e)
            uq_pred = -Valpha * math.sin(theta_e) + Vbeta * math.cos(theta_e)

            # 预测 k+1 时刻电气状态
            id_next = id_k + (Ts / Ld) * (ud_pred - Rs * id_k + omega_e * Lq * iq_k)
            iq_next = iq_k + (Ts / Lq) * (uq_pred - Rs * iq_k - omega_e * Ld * id_k - omega_e * psi_pm)

            # 预测 k+1 时刻电磁转矩
            Te_next = 1.5 * p * (psi_pm * iq_next + (Ld - Lq) * id_next * iq_next)

            # 预测 k+2 时刻机械状态 (转速响应滞后一拍)
            wm_next = wm_k + (Ts / J) * (Te_next - B_fric * wm_k - Tl_hat)

            # --- 代价函数计算 ---
            # 1. 跟踪代价 c_T
            c_T = (wm_next - w_ref)**2

            # 2. MTPA 吸引区代价 c_A1
            c_A1 = (id_next + ((Ld - Lq) / psi_pm) * (id_next**2 - iq_next**2))**2

            # 3. 限制惩罚代价 c_L1 & c_L2
            I_mag = math.sqrt(id_next**2 + iq_next**2)
            c_L1 = (I_mag - Ir_max)**2 if I_mag > Ir_max else 0.0

            limit_side = 2 * ((Ld - Lq) / psi_pm) * id_next + 1
            c_L2 = (limit_side)**2 if limit_side < 0 else 0.0

            # 总代价
            cost = lambda_T * c_T + lambda_A * c_A1 + lambda_L * (c_L1 + c_L2)

            # 寻找全局最优
            if cost < cost_min:
                cost_min = cost
                best_state_idx = idx
                best_ud = ud_pred
                best_uq = uq_pred

        # --- E. 更新控制器状态记忆 ---
        last_state_idx = best_state_idx
        u_d_prev = best_ud
        u_q_prev = best_uq

        # --- F. 电机物理本体仿真 (连续域的离散化模拟) ---
        did_dt = (best_ud - Rs * id + omega_e * Lq * iq) / Ld
        diq_dt = (best_uq - Rs * iq - omega_e * Ld * id - omega_e * psi_pm) / Lq
        id += Ts * did_dt
        iq += Ts * diq_dt

        Te = 1.5 * p * (psi_pm * iq + (Ld - Lq) * id * iq)
        domega_dt = (Te - B_fric * omega_m - Tl) / J
        omega_m += Ts * domega_dt

        # 角度更新并归一化到 0~2pi
        theta_e = (theta_e + Ts * omega_e) % (2 * math.pi)

        # --- G. 记录数据 ---
        id_log[k] = id
        iq_log[k] = iq
        w_log[k] = omega_m * (60 / (2 * math.pi))  # 转换为 rpm
        Te_log[k] = Te

    # =========================================================================
    # 6. 绘图展示 (对标论文 Fig. 8)
    # =========================================================================
    plt.figure(figsize=(10, 12), facecolor='white')

    # 1. 速度响应与参考值
    plt.subplot(4, 1, 1)
    plt.plot(time, w_ref_log * (60 / (2 * math.pi)), 'r--', linewidth=1.5, label='Reference')
    plt.plot(time, w_log, 'b', linewidth=1.5, label='Actual')
    plt.ylabel('Speed (rpm)')
    plt.title('Speed with Load Torque Rejection (t=0.2s added 6Nm)')
    plt.legend(loc='lower right')
    plt.grid(True)

    # 2. 电磁转矩与观测负载
    plt.subplot(4, 1, 2)
    plt.plot(time, Te_log, 'k', linewidth=1.0, label='Electromagnetic Te')
    plt.plot(time, Tl_hat_log, 'm--', linewidth=1.5, label='Observed Load Tl_hat')
    plt.ylabel('Torque (Nm)')
    plt.legend(loc='lower right')
    plt.grid(True)

    # 3. dq 轴电流
    plt.subplot(4, 1, 3)
    plt.plot(time, id_log, 'b', linewidth=1.0, label='i_d (MTPA track)')
    plt.plot(time, iq_log, 'g', linewidth=1.0, label='i_q')
    plt.ylabel('Current (A)')
    plt.legend(loc='lower right')
    plt.grid(True)

    # 4. id-iq 平面轨迹 (观察 MTPA 效果)
    plt.subplot(4, 1, 4)
    plt.plot(id_log, iq_log, 'b', linewidth=1.0, label='Actual Trajectory')

    # 绘制理论 MTPA 曲线
    id_mtpa = np.linspace(-10, 0, 100)
    iq_mtpa = np.sqrt((id_mtpa) / ((Lq - Ld) / psi_pm) + id_mtpa**2)
    plt.plot(id_mtpa, iq_mtpa, 'r--', linewidth=2, label='Theoretical MTPA')

    plt.xlabel('i_d (A)')
    plt.ylabel('i_q (A)')
    plt.title('Current Trajectory & MTPA Curve')
    plt.legend(loc='upper right')
    plt.grid(True)
    plt.xlim([-10, 2])
    plt.ylim([0, 12])

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
