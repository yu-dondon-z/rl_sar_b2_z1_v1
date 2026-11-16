#!/usr/bin/env python3
"""Render the VD10 training-method diagram, without importing any simulator.

The user's VD1 diagram is a style/layout reference, not the VD10 algorithm.
Sources are the VD10/VD9/VD5Startup task configuration and command classes in
/home/dong/b2_z1_VD2. No VD11 reward or optimizer settings are depicted.
"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
from matplotlib.path import Path as MplPath


OUT = Path(__file__).resolve().parent
WIDTH, HEIGHT, DPI = 3600, 2160, 200
REGULAR = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
BOLD = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
NAVY, MUTED = "#1B3149", "#60758D"
BLUE, GREEN, PURPLE = "#286DB7", "#138975", "#7954B4"
plt.rcParams.update({
    "svg.fonttype": "none",
    "pdf.fonttype": 42,
    "mathtext.fontset": "dejavusans",
    "axes.unicode_minus": False,
    "savefig.facecolor": "white",
})


def main():
    fig = plt.figure(figsize=(WIDTH / DPI, HEIGHT / DPI), dpi=DPI, facecolor="white")
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set(xlim=(0, WIDTH), ylim=(HEIGHT, 0))
    ax.axis("off")
    labels = []

    def text(x, y, value, size=43, color=NAVY, bold=False, ha="left", width=None):
        font = FontProperties(fname=BOLD if bold else REGULAR)
        artist = ax.text(x, y, value, fontsize=size * 72 / DPI,
                         fontproperties=font, color=color, ha=ha, va="center",
                         clip_on=False, zorder=4)
        labels.append((artist, width, value))
        return artist

    def panel(x, y, w, h, fill, edge):
        ax.add_patch(FancyBboxPatch((x, y), w, h,
            boxstyle="round,pad=0,rounding_size=30", linewidth=1.25,
            edgecolor=edge, facecolor=fill, zorder=1))

    def arrow(points, color=NAVY, dashed=False):
        path = MplPath(points, [MplPath.MOVETO] + [MplPath.LINETO] * (len(points) - 1))
        ax.add_patch(FancyArrowPatch(path=path, arrowstyle="-|>", mutation_scale=20,
            linewidth=1.8, color=color, linestyle="--" if dashed else "-", zorder=2))

    def chip(x, y, w, h, value, color, size=40):
        panel(x, y, w, h, color, color)
        text(x + 25, y + h / 2, value, size=size, color="white", width=w - 50)

    # Header: match the supplied reference's restrained white/blue visual style.
    ax.plot([94, 94], [80, 152], color=BLUE, linewidth=5, solid_capstyle="round")
    text(132, 117, "VD10 训练方法", size=80, bold=True)
    text(90, 211, "同一世界焊缝命令 → 70维全身策略 → 关节位置控制", size=43, color=MUTED)
    text(3505, 118, "B2 + Z1  ·  2.93 kg 新枪", size=35, color=MUTED, ha="right")

    # Left / blue: calibrated ideal-geometric base-command generator.
    panel(90, 292, 2050, 450, "#F0F5FD", "#D4E2F7")
    text(148, 359, "底盘命令生成", size=55, color=BLUE, bold=True)
    text(148, 438, "真值世界路径 → 标定相机理想几何 → 横向、航向与曲率", size=42, width=1910)
    text(148, 515, r"$e_h=\operatorname{wrap}\!\left(\psi_t-\operatorname{atan2}(e_y,\,0.6)\right)$",
         size=51, color=BLUE, width=1875)
    text(148, 595, r"$\omega=\operatorname{clip}(1.5\,e_h+v\kappa,\,-0.8,\,0.8)$",
         size=51, color=BLUE, width=1875)
    text(148, 682, "v = 0.2 m/s；20%停车工况；2秒平滑渐入", size=40, color=BLUE, width=1240)
    chip(1490, 653, 580, 70, r"$\mathbf{c}_v=\alpha(t)\,[v,\,0,\,\omega]$", BLUE, 41)

    # Left / green: world seam, not the independent random Bezier target of VD1.
    panel(90, 789, 2050, 641, "#EFF8F5", "#D0E9E1")
    text(148, 851, "末端命令生成 · 与底盘共用同一条世界焊缝", size=51, color=GREEN,
         bold=True, width=1900)
    text(148, 922, "沿焊缝弧长向后取0.80 m工作点；下式为完整地面阶段（β = 1）", size=39, width=1900)
    text(148, 999, r"$\mathbf{p}_{s}^{W}=\boldsymbol{\gamma}\!\left(s_{\rm proj}-0.80\,{\rm m}\right)$",
         size=51, color=GREEN, width=1900)
    text(148, 1078, r"$\mathbf{p}_{d}^{W}=(1-\alpha)\,\mathbf{p}_{0}^{W}+\alpha\left(\mathbf{p}_{s}^{W}+h\,\mathbf{e}_{z}\right)$",
         size=51, color=GREEN, width=1900)
    text(148, 1157, r"$\mathbf{p}_{d}^{B}=R_{BW}\!\left(\mathbf{p}_{d}^{W}-\mathbf{p}_{B}^{W}\right)$",
         size=51, color=GREEN, width=1900)
    text(148, 1234, r"$\mathbf{q}_{d}^{B}=\operatorname{nlerp}\!\left(\mathbf{q}_{0}^{B}(t),\,\mathbf{q}_{\rm seam}^{B},\,\alpha\right)$",
         size=49, color=GREEN, width=1900)
    text(148, 1312, "从复位实测TCP渐入；跟踪枪轴方向；管口外缘目标离地5 mm", size=38, width=1900)
    text(148, 1383, "不是随机独立末端轨迹，也不是TCP中心固定离地5 mm", size=35, color=MUTED, width=1300)
    chip(1520, 1351, 550, 62, r"$\mathbf{c}_{ee}=[\mathbf{p}_{d}^{B},\,\mathbf{q}_{d}^{B}]$", GREEN, 39)

    # Input fan-in, maintaining the reference's orthogonal colored arrows.
    arrow([(2070, 688), (2210, 688), (2210, 434), (2310, 434)], BLUE)
    arrow([(2070, 1382), (2255, 1382), (2255, 510), (2310, 510)], GREEN)

    panel(2320, 337, 1190, 412, "#F7F9FC", "#DCE6F0")
    text(2375, 398, "策略输入 · 70维", size=53, bold=True)
    rows = [
        (478, "底盘速度命令", "3", BLUE),
        (552, "TCP位姿命令（位置 + 四元数）", "7", GREEN),
        (626, "本体状态 + 上一动作", "60", MUTED),
    ]
    for y, label, number, color in rows:
        text(2375, y, label, size=42, color=color, width=980)
        text(3450, y, number, size=46, bold=True, color=color, ha="right")
    text(2375, 704, "60 = 角速度3 + 重力3 + 关节状态36 + 上一动作18", size=28,
         color=MUTED, width=1070)

    arrow([(2580, 749), (2580, 810)])
    panel(2320, 821, 520, 260, NAVY, NAVY)
    text(2580, 878, "全身策略", size=52, color="white", bold=True, ha="center")
    text(2580, 960, r"$\pi_{\theta}(\mathbf{o}_t)$", size=68, color="white", ha="center")
    text(2580, 1040, "MLP  512 → 256 → 128", size=30, color="#C8D7E6", ha="center")
    arrow([(2852, 951), (2927, 951)])
    panel(2940, 821, 570, 260, "#FFFFFF", "#CDDBEB")
    text(3225, 878, "策略输出 · 18维", size=47, bold=True, ha="center")
    text(3225, 947, "12腿部 + 6机械臂", size=39, color=MUTED, ha="center")
    text(3225, 1023, r"$\mathbf{a}_t$", size=65, ha="center")
    arrow([(3225, 1082), (3225, 1129)])

    panel(2320, 1140, 1190, 290, "#EDF2F7", "#D5E0ED")
    text(2375, 1196, "关节位置目标 → 驱动 → Isaac Lab", size=45, bold=True, width=1080)
    text(2375, 1269, r"$\mathbf{q}_{\rm ref}=\mathbf{q}_{\rm default}+0.25\,\mathbf{a}_t$",
         size=56, width=1080)
    text(2375, 1338, "腿：显式PD；臂：隐式位置驱动（非直接力矩策略）", size=34, width=1080)
    text(2375, 1395, "策略50 Hz  /  驱动与物理200 Hz  /  Kp = 160，Kd = 5", size=32,
         color=MUTED, width=1080)

    # The hardware and initialization/course strip are VD10-specific.
    panel(90, 1485, 3420, 300, "#FAFBFD", "#E0E6EF")
    ax.plot([1480, 1480], [1530, 1741], color="#DCE4EE", linewidth=1)
    text(148, 1545, "新枪模型与工具坐标", size=46, bold=True)
    text(148, 1613, "2.93 kg  |  外径56 / 内径52 mm  |  长1100 mm", size=35, width=1265)
    text(148, 1675, "枪轴平行法兰+X；夹环上偏100 mm；前伸900 mm", size=35, width=1265)
    text(148, 1738, "TCP（link06局部）= (0.975, 0, 0.100) m", size=36, color=GREEN, width=1265)

    text(1540, 1545, "安全起步与地面课程", size=46, bold=True)
    text(1540, 1606, "80% AMP参考复位 + 20%标称站立；初始TCP世界高度约0.85 m", size=36, width=1910)
    text(1540, 1661, r"$\alpha=10u^3-15u^4+6u^5,\quad u=\operatorname{clip}(t/2,\,0,\,1)$",
         size=42, color=MUTED, width=1910)
    text(1540, 1714, "高位→地面：20,000个50 Hz控制步；俯角0.30 → π/2 − 0.45", size=35, width=1910)
    text(1540, 1760, "保留真实枪体承重、非法触地与倾倒终止；复位forward后再采命令", size=32,
         color=MUTED, width=1910)

    # Training feedback: VD9 terms, not the different VD11 rewards/hyperparameters.
    panel(90, 1840, 3420, 192, "#F5F1FB", "#E4DCF3")
    text(1800, 1891, "任务奖励 + AMP步态奖励 + 腿部镜像约束 → PPO更新策略", size=48,
         color=PURPLE, ha="center")
    text(1800, 1950, "200 Hz机身系TCP跟踪误差3 Hz高通 + 近地脚下行速度约束", size=37,
         color=PURPLE, ha="center")
    text(1800, 2000, r"任务：底盘速度 / TCP位置 / 枪轴方向 / 动作平滑与稳定    |    学习率$5\times10^{-5}$固定，熵系数0.01", size=31,
         color=PURPLE, ha="center")

    text(90, 2082, r"W：世界系；B：底盘系；γ：世界焊缝；h：管口外缘间隙对应的TCP高度；$p_0$：复位实测TCP；α：每回合2秒渐入；β：地面课程；末端姿态q按wxyz排列。", size=31,
         color=MUTED, width=3420)
    text(90, 2132, "世界真值用于训练命令与奖励；actor不直接输入图像、世界位姿或接触真值。本图不含VD11奖励修改。", size=30,
         color=MUTED, width=3420)

    # Check text sizes against their assigned regions before writing deliverables.
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    for artist, allowed_width, value in labels:
        bounds = artist.get_window_extent(renderer)
        if allowed_width and bounds.width > allowed_width:
            scale = allowed_width / bounds.width
            artist.set_fontsize(artist.get_fontsize() * scale * .99)
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    for artist, allowed_width, value in labels:
        bounds = artist.get_window_extent(renderer)
        if bounds.x0 < 25 or bounds.x1 > WIDTH - 25 or bounds.y0 < 3 or bounds.y1 > HEIGHT - 3:
            raise ValueError("Text outside canvas: " + value)
        if allowed_width and bounds.width > allowed_width + 2:
            raise ValueError("Text exceeds region: " + value)

    OUT.mkdir(parents=True, exist_ok=True)
    for suffix in ("png", "svg", "pdf"):
        path = OUT / ("VD10_TRAINING_METHOD_20261113." + suffix)
        fig.savefig(path, dpi=DPI, facecolor="white", transparent=False)
        print(path)
    plt.close(fig)


if __name__ == "__main__":
    main()
