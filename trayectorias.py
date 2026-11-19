"""
trayectorias.py — Generación de trayectorias (Tema 9).

  - escala_quintica():   s(tau) = 10 tau^3 - 15 tau^4 + 6 tau^5
                         velocidad y aceleración nulas al inicio y al final (sin sacudidas)
  - ptp_articular():     movimiento punto a punto (MoveJ) en el espacio articular
  - lineal_cartesiana(): movimiento en línea recta de la punta (MoveL), con IK en cada muestra
"""
import numpy as np
from modelo_dh import ik, fk


def escala_quintica(tau):
    tau = np.clip(tau, 0.0, 1.0)
    return 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5


def ptp_articular(q0, qf, duracion, dt):
    """Lista de configuraciones de q0 a qf con perfil quíntico en cada articulación."""
    q0, qf = np.asarray(q0, float), np.asarray(qf, float)
    n = max(2, int(round(duracion / dt)) + 1)
    return [q0 + escala_quintica(k / (n - 1)) * (qf - q0) for k in range(n)]


def lineal_cartesiana(q_inicio, p_final, eje, duracion, dt):
    """La punta de la boquilla va en línea recta desde su posición actual hasta p_final,
    manteniendo el eje de la boquilla alineado con 'eje'. Devuelve (lista_q, lista_p_deseada)."""
    p0 = fk(q_inicio)[:3, 3]
    p_final = np.asarray(p_final, float)
    n = max(2, int(round(duracion / dt)) + 1)
    qs, ps = [], []
    q = np.asarray(q_inicio, float)
    for k in range(n):
        p = p0 + escala_quintica(k / (n - 1)) * (p_final - p0)
        q, ok, _ = ik(p, eje, q)          
        if not ok:
            raise RuntimeError(f"IK no convergió en la muestra {k} del tramo lineal")
        qs.append(q.copy())
        ps.append(p.copy())
    return qs, ps
