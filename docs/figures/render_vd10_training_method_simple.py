#!/usr/bin/env python3
"""Render a compact VD10 overview in the style of the user's VD1 reference."""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
from matplotlib.path import Path as MplPath
from fontTools.ttLib import TTFont


OUT = Path(__file__).resolve().parent
WIDTH, HEIGHT, DPI = 3600, 1908, 200
NAVY, MUTED = "#1B3149", "#60758D"
BLUE, GREEN, PURPLE = "#286DB7", "#138975", "#7954B4"
plt.rcParams.update({"svg.fonttype": "path", "pdf.fonttype": 3,
                     "mathtext.fontset": "dejavusans", "axes.unicode_minus": False})


def simplified_chinese_font(bold=False):
    """Select the SC face explicitly; matplotlib opens TTC face 0 (Japanese)."""
    weight = "Bold" if bold else "Regular"
    cache = Path.home() / ".cache" / "b2_z1_diagrams" / "fonts"
    cache.mkdir(parents=True, exist_ok=True)
    path = cache / ("NotoSansCJKsc-" + weight + ".otf")
    if not path.is_file():
        source = "/usr/share/fonts/opentype/noto/NotoSansCJK-" + weight + ".ttc"
        with TTFont(source, fontNumber=2) as font:
            if font["name"].getDebugName(1) != "Noto Sans CJK SC":
                raise ValueError("Expected the Simplified Chinese font face")
            font.save(path)
    font = FontProperties(fname=str(path))
    if font.get_name() != "Noto Sans CJK SC":
        raise ValueError("Incorrect cached font face")
    return font


def main():
    fig = plt.figure(figsize=(WIDTH / DPI, HEIGHT / DPI), dpi=DPI, facecolor="white")
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set(xlim=(0, WIDTH), ylim=(HEIGHT, 0))
    ax.axis("off")
    labels = []
    fonts = {False: simplified_chinese_font(), True: simplified_chinese_font(True)}

    def text(x, y, value, size=45, color=NAVY, bold=False, ha="left", width=None):
        font = fonts[bold]
        item = ax.text(x, y, value, fontsize=size * 72 / DPI, fontproperties=font,
                       color=color, ha=ha, va="center", zorder=4)
        labels.append((item, width, value))

    def panel(x, y, w, h, fill, edge):
        ax.add_patch(FancyBboxPatch((x, y), w, h,
                     boxstyle="round,pad=0,rounding_size=30", linewidth=1.2,
                     edgecolor=edge, facecolor=fill, zorder=1))

    def arrow(points, color=NAVY):
        path = MplPath(points, [MplPath.MOVETO] + [MplPath.LINETO] * (len(points) - 1))
        ax.add_patch(FancyArrowPatch(path=path, arrowstyle="-|>", mutation_scale=22,
                                    linewidth=1.8, color=color, zorder=2))

    def chip(x, y, w, value, color):
        panel(x, y, w, 88, color, color)
        text(x + 30, y + 44, value, size=44, color="white", width=w - 60)

    ax.plot([96, 96], [82, 149], color=BLUE, linewidth=5, solid_capstyle="round")
    text(138, 116, "VD10 训练方法", size=80, bold=True)
    text(90, 212, "命令生成 → 策略输入 → 全身控制", size=44, color=MUTED)

    panel(90, 315, 2020, 486, "#F0F5FD", "#D4E2F7")
    text(152, 381, "底盘命令生成", size=57, color=BLUE, bold=True)
    text(152, 460, r"真值路径点 → 理想相机几何 → $e_y$、$\psi_t$、$\kappa$", size=46)
    text(152, 550, r"$e_h=\operatorname{wrap}(\psi_t-\operatorname{atan2}(e_y,\,0.6))$",
         size=53, color=BLUE, width=1870)
    text(152, 640, r"$\omega=\operatorname{clip}(1.5\,e_h+v\kappa,\,-0.8,\,0.8)$",
         size=53, color=BLUE, width=1870)
    text(152, 730, "v = 0.2 m/s", size=49, color=BLUE)
    chip(1488, 686, 560, r"$\mathbf{c}_v=\alpha\,[v,\,0,\,\omega]$", BLUE)

    panel(90, 855, 2020, 665, "#EFF8F5", "#D0E9E1")
    text(152, 923, "末端命令生成", size=57, color=GREEN, bold=True)
    text(152, 1009, "同一世界焊缝 → 沿弧长后移0.80 m的工作点", size=46, width=1870)
    text(152, 1107,
         r"$\mathbf{p}_g^W=\boldsymbol{\gamma}(s_{\rm proj}-0.80\,{\rm m})+h\,\mathbf{e}_z$",
         size=53, color=GREEN, width=1870)
    text(152, 1204,
         r"$\mathbf{p}_g^B=R_{BW}(\mathbf{p}_g^W-\mathbf{p}_B^W)$",
         size=53, color=GREEN, width=1870)
    text(152, 1300, "姿态：跟踪枪轴方向；管口外缘离地5 mm", size=46, color=GREEN, width=1870)
    text(152, 1388, "复位位姿 → 2秒平滑过渡 → 目标位姿", size=45, color=MUTED, width=1870)
    chip(1488, 1420, 560, r"$\mathbf{c}_{ee}=[\mathbf{p}_d^B,\,\mathbf{q}_d^B]$", GREEN)

    arrow([(2048, 730), (2165, 730), (2165, 527), (2274, 527)], BLUE)
    arrow([(2048, 1464), (2220, 1464), (2220, 618), (2274, 618)], GREEN)

    panel(2285, 370, 1170, 420, "#F7F9FC", "#DCE6F0")
    text(2340, 438, "策略输入 · 70维", size=56, bold=True)
    for y, label, number, color in [
        (535, r"底盘速度命令 $\mathbf{c}_v$", "3", BLUE),
        (626, r"末端位姿命令 $\mathbf{c}_{ee}$", "7", GREEN),
        (717, "本体状态 + 上一动作", "60", MUTED),
    ]:
        text(2340, y, label, size=47, color=color, width=960)
        text(3390, y, number, size=50, color=color, bold=True, ha="right")

    arrow([(2528, 790), (2528, 858)])
    panel(2285, 870, 487, 305, NAVY, NAVY)
    text(2528, 935, "全身策略", size=55, color="white", bold=True, ha="center")
    text(2528, 1025, r"$\pi_\theta(\mathbf{o}_t)$", size=67, color="white", ha="center")
    text(2528, 1117, "MLP", size=43, color="#C8D7E6", ha="center")

    arrow([(2783, 1022), (2868, 1022)])
    panel(2880, 870, 575, 305, "#FFFFFF", "#CDDBEB")
    text(3167, 935, "策略输出 · 18维", size=52, bold=True, ha="center")
    text(3167, 1025, "12腿部 + 6机械臂", size=44, color=MUTED, ha="center")
    text(3167, 1110, r"$\mathbf{a}_t$", size=68, ha="center")

    arrow([(3167, 1175), (3167, 1277)])
    panel(2285, 1290, 1170, 230, "#EDF2F7", "#D5E0ED")
    text(2340, 1358, "关节位置目标 → PD → 仿真", size=54, bold=True, width=1060)
    text(2340, 1451, r"$\mathbf{q}_{\rm ref}=\mathbf{q}_{\rm default}+0.25\,\mathbf{a}_t$",
         size=58, width=1060)

    panel(90, 1610, 3365, 148, "#F5F1FB", "#E4DCF3")
    text(1772, 1684, "训练：任务奖励 + AMP步态奖励 + 腿部镜像约束 → PPO更新策略",
         size=52, color=PURPLE, ha="center", width=3210)
    text(90, 1832,
         "W：世界系；B：底盘系；γ：焊缝路径；h：管口间隙对应的TCP高度；α：启动渐入系数。",
         size=38, color=MUTED, width=3365)

    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    for item, width, value in labels:
        bounds = item.get_window_extent(renderer)
        if width and bounds.width > width:
            item.set_fontsize(item.get_fontsize() * width / bounds.width * .99)
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    for item, width, value in labels:
        bounds = item.get_window_extent(renderer)
        if bounds.x0 < 25 or bounds.x1 > WIDTH - 25 or bounds.y0 < 3 or bounds.y1 > HEIGHT - 3:
            raise ValueError("Text outside canvas: " + value)

    OUT.mkdir(parents=True, exist_ok=True)
    for suffix in ("png", "svg", "pdf"):
        path = OUT / ("VD10_TRAINING_METHOD_SIMPLE_20261113_SC." + suffix)
        fig.savefig(path, dpi=DPI, facecolor="white", transparent=False)
        print(path)
    plt.close(fig)


if __name__ == "__main__":
    main()
