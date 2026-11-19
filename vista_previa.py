"""
vista_previa.py — Animación del ciclo SIN CoppeliaSim (matplotlib), a partir del mismo modelo DH.
Sirve para revisar el movimiento rápido; la simulación oficial es la de CoppeliaSim.

    python vista_previa.py            -> resultados/vista_previa.gif
"""
import io
import contextlib
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import modelo_dh as m
import escenario as esc
import ciclo_repostaje as cr


def caja(ax, centro, tam, color, alpha=1.0):
    c, t = np.array(centro), np.array(tam) / 2
    v = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * t + c
    caras = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]
    pc = Poly3DCollection([[v[i] for i in f] for f in caras], facecolor=color, edgecolor="k",
                          linewidths=0.3, alpha=alpha)
    ax.add_collection3d(pc)
    return pc


def main():
    sys.argv = ["x", "--sin-coppelia", "--ciclos", "1", "--litros", "3"]
    with contextlib.redirect_stdout(io.StringIO()):
        escena, res, ctl = cr.main()
    # solo los cuadros con el carro en su sitio 
    sel = [i for i, (q, c) in enumerate(escena.cuadros) if abs(c[0]) < 0.6][::3]
    cuadros = [escena.cuadros[i] for i in sel]
    estados = [ctl.filas[i][2] for i in sel]

    fig = plt.figure(figsize=(7, 5.5), dpi=90)
    ax = fig.add_subplot(projection="3d")

    def dibujar(k):
        ax.cla()
        q, carro = cuadros[k]
        dx, dy = carro
        ax.set_xlim(-1.3, 1.3); ax.set_ylim(-1.9, 0.7); ax.set_zlim(0, 1.6)
        ax.set_box_aspect((2.6, 2.6, 1.6))
        ax.view_init(elev=20, azim=62)
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
        # isla, pedestal, surtidor
        caja(ax, (0, 0.25, 0.075), (2.4, 1.0, 0.15), "0.7")
        caja(ax, (0, 0, 0.42), (0.3, 0.3, 0.55), "0.35")
        sx, sy, _ = esc.SURTIDOR_POS
        caja(ax, (sx, sy, 1.0), (0.5, 0.35, 1.7), "0.85")
        # carro (solo la parte trasera que entra en el cuadro)
        yc = -(esc.DIST_LATERAL_NOMINAL + dy)
        x0 = dx + esc.CARRO_X_TRASERA
        caja(ax, (x0 + 1.2, yc - esc.CARRO_ANCHO / 2, 0.675), (2.4, esc.CARRO_ANCHO, 0.75), "#b0161e", 0.28)
        pb = esc.posicion_boca(dx, dy)
        ax.scatter(*pb, color="k", s=25)
        # robot
        F = [esc.a_mundo(T) for T in m.fk_frames(q)]
        pts = [F[0][:3, 3]]
        for i in range(6):
            d = m.DH_TABLA[i][2]
            pts.append(F[i][:3, 3] + d * F[i][:3, 2])
            pts.append(F[i + 1][:3, 3])
        pts = np.array(pts)
        ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], "-o", color="#3466c2", lw=5, ms=5, zorder=10)
        boq = np.array([F[6][:3, 3], F[7][:3, 3]])
        ax.plot(boq[:, 0], boq[:, 1], boq[:, 2], color="#1a9a40", lw=3)
        ax.set_title(f"Estado: {estados[k]}   (litros: {ctl.filas[sel[k]][-1]:.1f})", fontsize=11)
        return []

    anim = FuncAnimation(fig, dibujar, frames=len(cuadros), interval=1000 * ctl.dt * 3)
    anim.save("resultados/vista_previa.gif", writer=PillowWriter(fps=int(round(1 / (ctl.dt * 3)))))
    print("resultados/vista_previa.gif", len(cuadros), "cuadros")


if __name__ == "__main__":
    main()
