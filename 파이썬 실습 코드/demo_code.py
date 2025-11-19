#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TSID + QP Single-File Demo (Explained & Animated)
- 2-DoF planar arm (fixed base)
- TSID-style objective on desired joint accelerations
- Equality: M(q) ddq + h(q,dq) - tau = 0
- Inequality: tau bounds
- QP vars: x = [ddq(2), tau(2)]
- In-file tiny Active-Set QP (educational)
- Extra: errors/EE path plots + saved animation (GIF/MP4)
"""

import os
import math
import numpy as np
import matplotlib.pyplot as plt

from matplotlib import animation

# ---------- Output directory ----------
OUTDIR = os.path.join(os.path.dirname(__file__), "outputs")
os.makedirs(OUTDIR, exist_ok=True)

# ---------- Tiny Active-Set QP (equality + inequality) ----------
class ActiveSetQP:
    def __init__(self, H, g, Aeq=None, beq=None, Ain=None, bin=None, tol=1e-8, max_iter=200):
        self.H = np.array(H, dtype=float)
        self.g = np.array(g, dtype=float)
        self.Aeq = np.zeros((0, self.H.shape[0])) if Aeq is None else np.array(Aeq, dtype=float)
        self.beq = np.zeros((0,)) if beq is None else np.array(beq, dtype=float)
        self.Ain = np.zeros((0, self.H.shape[0])) if Ain is None else np.array(Ain, dtype=float)
        self.bin = np.zeros((0,)) if bin is None else np.array(bin, dtype=float)
        self.tol = tol
        self.max_iter = max_iter
        self.Astack = np.vstack([self.Aeq, self.Ain]) if self.Ain.size else self.Aeq.copy()
        self.bstack = np.concatenate([self.beq, self.bin]) if self.Ain.size else self.beq.copy()
        self.is_ineq = np.array([False]*len(self.beq) + [True]*len(self.bin))

    def solve(self, x0=None):
        n = self.H.shape[0]
        W = list(range(len(self.beq)))     # start with all equalities active
        x = np.zeros(n) if x0 is None else x0.copy()

        for it in range(self.max_iter):
            if len(W) > 0:
                Aw = self.Astack[W, :]
                bw = self.bstack[W]
                KKT = np.block([[self.H, Aw.T],
                                [Aw, np.zeros((Aw.shape[0], Aw.shape[0]))]])
                rhs = np.concatenate([-self.g, bw])
                try:
                    sol = np.linalg.solve(KKT, rhs)
                except np.linalg.LinAlgError:
                    Hreg = self.H + 1e-8*np.eye(n)
                    KKT = np.block([[Hreg, Aw.T],
                                    [Aw, np.zeros((Aw.shape[0], Aw.shape[0]))]])
                    sol = np.linalg.lstsq(KKT, rhs, rcond=None)[0]
                x = sol[:n]
                lam = sol[n:]
            else:
                try:
                    x = -np.linalg.solve(self.H, self.g)
                except np.linalg.LinAlgError:
                    x = -np.linalg.lstsq(self.H, self.g, rcond=None)[0]
                lam = np.zeros(0)

            # primal violation (Ain x <= bin)
            if self.Ain.shape[0] > 0:
                s = self.Ain.dot(x) - self.bin
                max_viol = np.max(s)
            else:
                max_viol = -np.inf

            # dual violation: any active inequality with negative lambda?
            neg_lambda_idx = -1
            neg_lambda_val = 0.0
            if len(W) > 0:
                for k, idx in enumerate(W):
                    if self.is_ineq[idx] and lam[k] < neg_lambda_val - self.tol:
                        neg_lambda_val = lam[k]
                        neg_lambda_idx = k

            if max_viol > self.tol:
                i_in = int(np.argmax(self.Ain.dot(x) - self.bin))
                global_idx = len(self.beq) + i_in
                if global_idx not in W:
                    W.append(global_idx)
                continue

            if neg_lambda_idx != -1:
                W.pop(neg_lambda_idx)
                continue

            return x, {'status': 0, 'iterations': it+1, 'active_set': W, 'lambdas': lam}

        return x, {'status': 1, 'iterations': self.max_iter, 'active_set': W, 'lambdas': lam}

# ---------- 2-DoF planar arm ----------
class Planar2Link:
    def __init__(self):
        self.l1 = 0.5; self.l2 = 0.5
        self.m1 = 2.0; self.m2 = 1.5
        self.c1 = self.l1*0.5; self.c2 = self.l2*0.5
        self.I1 = 0.06; self.I2 = 0.04
        self.g = 9.81

    def M(self, q):
        c2 = math.cos(q[1])
        m11 = self.I1 + self.I2 + self.m1*self.c1**2 + self.m2*(self.l1**2 + self.c2**2 + 2*self.l1*self.c2*c2)
        m12 = self.I2 + self.m2*(self.c2**2 + self.l1*self.c2*c2)
        m22 = self.I2 + self.m2*self.c2**2
        return np.array([[m11, m12],[m12, m22]], dtype=float)

    def h(self, q, dq):
        s2 = math.sin(q[1])
        h1 = - self.m2*self.l1*self.c2*s2*(2*dq[0]*dq[1] + dq[1]**2) \
             + (self.m1*self.c1 + self.m2*self.l1)*self.g*math.cos(q[0]) \
             + self.m2*self.c2*self.g*math.cos(q[0]+q[1])
        h2 =  self.m2*self.l1*self.c2*s2*(dq[0]**2) \
             + self.m2*self.c2*self.g*math.cos(q[0]+q[1])
        return np.array([h1, h2], dtype=float)

    def fk(self, q):
        x1 = self.l1*np.cos(q[0]); y1 = self.l1*np.sin(q[0])
        x2 = x1 + self.l2*np.cos(q[0]+q[1]); y2 = y1 + self.l2*np.sin(q[0]+q[1])
        return (x1, y1), (x2, y2)

# ---------- QP builder ----------
def build_qp(robot, q, dq, ddq_des, tau_min, tau_max, w_ddq=1.0):
    M = robot.M(q); h = robot.h(q, dq)
    n_ddq = 2; n_tau = 2; n = n_ddq + n_tau

    H = np.zeros((n, n)); g = np.zeros(n)
    H[0:n_ddq, 0:n_ddq] = w_ddq * np.eye(n_ddq)
    g[0:n_ddq] = -w_ddq * ddq_des

    Aeq = np.zeros((2, n))
    Aeq[:, 0:n_ddq] = M
    Aeq[:, n_ddq:] = -np.eye(n_tau)
    beq = -h

    Ain = np.zeros((2*n_tau, n)); binv = np.zeros((2*n_tau,))
    Ain[0:n_tau, n_ddq:] =  np.eye(n_tau); binv[0:n_tau] = tau_max
    Ain[n_tau:, n_ddq:] = -np.eye(n_tau); binv[n_tau:] = -tau_min
    return H, g, Aeq, beq, Ain, binv

# ---------- reference & TSID desired ddq ----------
def posture_reference(t, q0):
    amp = np.array([0.4, 0.3]); omega = np.array([1.0, 1.2]); phase = np.array([0.0, 0.5])
    qref = q0 + amp*np.sin(omega*t + phase)
    dqref = (omega*amp)*np.cos(omega*t + phase)
    ddqref = - (omega**2)*amp*np.sin(omega*t + phase)
    return qref, dqref, ddqref

def tsid_ddq_des(q, dq, qref, dqref, ddqref, kp=60.0, kd=12.0):
    e  = qref - q
    ed = dqref - dq
    return kp*e + kd*ed + ddqref

# ---------- animation helper ----------
def save_animation(robot, Q, QREF, tgrid, fname_base="tsid_qp_control", step_stride=2):
    """Save control process as GIF (and MP4 if ffmpeg is available)."""
    L1, L2 = robot.l1, robot.l2
    idx = np.arange(0, Q.shape[1], step_stride)
    fig = plt.figure(figsize=(5,5))
    ax = fig.add_subplot(111); ax.set_aspect('equal')
    ax.set_xlim(-L1-L2-0.1, L1+L2+0.1); ax.set_ylim(-L1-L2-0.1, L1+L2+0.1)
    line1, = ax.plot([], [], marker='o', lw=2)   # robot
    line2, = ax.plot([], [], 'k--', lw=1, alpha=0.4)  # reference linkage
    time_txt = ax.text(0.02, 0.95, '', transform=ax.transAxes)

    def init():
        line1.set_data([], []); line2.set_data([], [])
        time_txt.set_text('')
        return line1, line2, time_txt

    def frame(i):
        k = idx[i]
        q = Q[:, k]; qr = QREF[:, k]
        (x1,y1),(x2,y2) = robot.fk(q)
        (xr1,yr1),(xr2,yr2) = robot.fk(qr)
        line1.set_data([0,x1,x2],[0,y1,y2])
        line2.set_data([0,xr1,xr2],[0,yr1,yr2])
        time_txt.set_text(f"t = {tgrid[k]:.2f} s")
        return line1, line2, time_txt

    ani = animation.FuncAnimation(fig, frame, init_func=init,
                                  frames=len(idx), interval=20, blit=True)
    # GIF
    try:
        gif_path = os.path.join(OUTDIR, f"{fname_base}.gif")
        ani.save(gif_path, writer=animation.PillowWriter(fps=30))
        print("Saved GIF:", gif_path)
    except Exception as e:
        print("GIF save failed:", e)

    # MP4 (requires ffmpeg in PATH)
    try:
        mp4_path = os.path.join(OUTDIR, f"{fname_base}.mp4")
        ani.save(mp4_path, writer=animation.FFMpegWriter(fps=30, bitrate=1800))
        print("Saved MP4:", mp4_path)
    except Exception as e:
        print("MP4 save failed (need ffmpeg?):", e)

    plt.close(fig)

# ---------- main sim ----------
def run_sim(T=6.0, dt=0.002, seed=0, plot=True, animate=True):
    np.random.seed(seed)
    robot = Planar2Link()
    q = np.array([0.0, 0.0]); dq = np.array([0.0, 0.0])
    tau_min = np.array([-25.0, -20.0]); tau_max = np.array([25.0, 20.0])

    steps = int(T/dt)
    tgrid = np.arange(steps)*dt
    Q = np.zeros((2, steps)); DQ = np.zeros((2, steps)); DDQ = np.zeros((2, steps)); TAU = np.zeros((2, steps))
    QREF = np.zeros((2, steps)); DQREF = np.zeros((2, steps)); DDQREF = np.zeros((2, steps))
    iters = np.zeros(steps, dtype=int); n_active_ineq = np.zeros(steps, dtype=int)
    EE = np.zeros((2, steps)); EE_REF = np.zeros((2, steps))

    for k in range(steps):
        t = tgrid[k]
        qref, dqref, ddqref = posture_reference(t, q0=np.array([0.3, -0.4]))
        ddq_des = tsid_ddq_des(q, dq, qref, dqref, ddqref)

        H,g,Aeq,beq,Ain,binv = build_qp(robot, q, dq, ddq_des, tau_min, tau_max, w_ddq=1.0)
        solver = ActiveSetQP(H,g,Aeq,beq,Ain,binv, tol=1e-8, max_iter=100)
        x, info = solver.solve()
        ddq = x[0:2]; tau = x[2:4]

        # integrate
        dq = dq + dt*ddq
        q  = q  + dt*dq

        # logs
        Q[:,k]=q; DQ[:,k]=dq; DDQ[:,k]=ddq; TAU[:,k]=tau
        QREF[:,k]=qref; DQREF[:,k]=dqref; DDQREF[:,k]=ddqref
        iters[k]=info['iterations']
        n_eq = Aeq.shape[0]
        n_active_ineq[k] = max(0, sum(1 for idx in info['active_set'] if idx >= n_eq))
        (_, _), (xEE, yEE) = robot.fk(q); EE[:,k] = [xEE, yEE]
        (_, _), (xEER, yEER) = robot.fk(qref); EE_REF[:,k] = [xEER, yEER]

    if plot:
        # Figure 1: raw signals
        fig, axs = plt.subplots(5, 1, figsize=(10, 12), sharex=True)
        axs[0].plot(tgrid, Q[0], label='q1'); axs[0].plot(tgrid, QREF[0], '--', label='q1_ref')
        axs[0].plot(tgrid, Q[1], label='q2'); axs[0].plot(tgrid, QREF[1], '--', label='q2_ref')
        axs[0].set_ylabel('pos [rad]'); axs[0].legend(loc='upper right')

        axs[1].plot(tgrid, DQ[0], label='dq1'); axs[1].plot(tgrid, DQREF[0], '--', label='dq1_ref')
        axs[1].plot(tgrid, DQ[1], label='dq2'); axs[1].plot(tgrid, DQREF[1], '--', label='dq2_ref')
        axs[1].set_ylabel('vel [rad/s]'); axs[1].legend(loc='upper right')

        axs[2].plot(tgrid, DDQ[0], label='ddq1'); axs[2].plot(tgrid, DDQREF[0], '--', label='ddq1_ref')
        axs[2].plot(tgrid, DDQ[1], label='ddq2'); axs[2].plot(tgrid, DDQREF[1], '--', label='ddq2_ref')
        axs[2].set_ylabel('acc [rad/s^2]'); axs[2].legend(loc='upper right')

        axs[3].plot(tgrid, TAU[0], label='tau1'); axs[3].plot(tgrid, TAU[1], label='tau2')
        axs[3].axhline(tau_min[0], ls='--', alpha=0.6); axs[3].axhline(tau_max[0], ls='--', alpha=0.6)
        axs[3].axhline(tau_min[1], ls='--', alpha=0.6); axs[3].axhline(tau_max[1], ls='--', alpha=0.6)
        axs[3].set_ylabel('torque [Nm]'); axs[3].legend(loc='upper right')

        axs[4].plot(tgrid, iters, label='QP iterations')
        axs[4].plot(tgrid, n_active_ineq, label='#active ineq')
        axs[4].set_ylabel('solver'); axs[4].set_xlabel('time [s]'); axs[4].legend(loc='upper right')
        plt.tight_layout()
        plt.savefig(os.path.join(OUTDIR, 'tsid_qp_demo_plots.png'), dpi=150)

        # Figure 2: errors + EE path + snapshot
        fig2 = plt.figure(figsize=(12, 5))
        ax1 = fig2.add_subplot(1,3,1)
        e  = Q - QREF; ed = DQ - DQREF; ea = DDQ - DDQREF
        ax1.plot(tgrid, np.linalg.norm(e,  axis=0),  label='|q-q_ref|')
        ax1.plot(tgrid, np.linalg.norm(ed, axis=0),  label='|dq-dq_ref|')
        ax1.plot(tgrid, np.linalg.norm(ea, axis=0),  label='|ddq-ddq_ref|')
        ax1.set_title('Tracking error (L2)'); ax1.set_xlabel('time [s]'); ax1.set_ylabel('norm'); ax1.legend(loc='upper right')

        ax2 = fig2.add_subplot(1,3,2); ax2.set_aspect('equal')
        ax2.plot(EE[0], EE[1], label='EE path')
        ax2.plot(EE_REF[0], EE_REF[1], '--', label='EE ref')
        ax2.plot(EE[0,0], EE[1,0], 'o', label='start'); ax2.plot(EE[0,-1], EE[1,-1], 's', label='end')
        ax2.set_title('End-Effector trajectory'); ax2.set_xlabel('x [m]'); ax2.set_ylabel('y [m]'); ax2.legend(loc='upper right')

        ax3 = fig2.add_subplot(1,3,3); ax3.set_aspect('equal')
        L1, L2 = robot.l1, robot.l2
        q1, q2 = Q[:, -1]
        (x1,y1),(x2,y2) = robot.fk(np.array([q1,q2]))
        ax3.plot([0, x1],[0, y1], marker='o'); ax3.plot([x1, x2],[y1, y2], marker='o')
        ax3.set_xlim(-L1-L2-0.1, L1+L2+0.1); ax3.set_ylim(-L1-L2-0.1, L1+L2+0.1)
        ax3.set_title('Robot snapshot (last pose)')
        plt.tight_layout()
        plt.savefig(os.path.join(OUTDIR, 'tsid_qp_demo_plots_explain.png'), dpi=150)

    if animate:
        # Save animation of the control process
        save_animation(robot, Q, QREF, tgrid, fname_base="tsid_qp_control", step_stride=2)

    return {
        'Q': Q, 'DQ': DQ, 'DDQ': DDQ, 'TAU': TAU,
        'QREF': QREF, 'DQREF': DQREF, 'DDQREF': DDQREF,
        'iters': iters, 'n_active_ineq': n_active_ineq,
        't': tgrid
    }

if __name__ == "__main__":
    run_sim(T=6.0, dt=0.002, seed=0, plot=True, animate=True)
    print("Saved:", os.path.join(OUTDIR, 'tsid_qp_demo_plots.png'))
    print("Saved:", os.path.join(OUTDIR, 'tsid_qp_demo_plots_explain.png'))
    print("Saved: outputs/tsid_qp_control.gif (and .mp4 if ffmpeg is available)")
