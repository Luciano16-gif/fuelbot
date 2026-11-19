"""
modelo_dh.py: Modelo cinemático del brazo de repostaje (6R, tipo UR5).

Convención: Denavit-Hartenberg ESTÁNDAR :
    theta_i y d_i se miden sobre z_{i-1};  a_i y alpha_i se miden sobre x_i.
    A_i = Rot_z(theta_i) · Trans_z(d_i) · Trans_x(a_i) · Rot_x(alpha_i)

Parámetros: los publicados por Universal Robots para el UR5
(https://www.universal-robots.com/articles/ur/application-installation/
 dh-parameters-for-calculations-of-kinematics-and-dynamics/)

Contenido:
    dh_matrix()     matriz homogénea de una fila DH
    fk_frames()     todas las matrices 0T1 ... 0T6 y la de la boquilla
    fk()            cinemática directa: pose de la punta de la boquilla
    jacobiano()     jacobiano geométrico 6x6
    ik()            cinemática inversa numérica (mínimos cuadrados amortiguados)
"""
import numpy as np

# ---------------------------------------------------------------------------
# Tabla DH estándar del UR5:  (a_i [m], alpha_i [rad], d_i [m], offset theta_i [rad])
#   theta_i = q_i + offset_i   (offset = 0 en el UR5)
# ---------------------------------------------------------------------------
DH_TABLA = [
    # a         alpha        d          offset
    (0.0,       np.pi / 2,   0.089159,  0.0),   # 1  base -> hombro
    (-0.425,    0.0,         0.0,       0.0),   # 2  brazo
    (-0.39225,  0.0,         0.0,       0.0),   # 3  antebrazo
    (0.0,       np.pi / 2,   0.10915,   0.0),   # 4  muñeca 1
    (0.0,      -np.pi / 2,   0.09465,   0.0),   # 5  muñeca 2
    (0.0,       0.0,         0.0823,    0.0),   # 6  muñeca 3 
]
N_GDL = len(DH_TABLA)

# Herramienta: boquilla recta de 20 cm a lo largo de z6 
LONG_BOQUILLA = 0.20
T_TOOL = np.array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, LONG_BOQUILLA],
                   [0, 0, 0, 1]], dtype=float)

# Límites articulares del UR5: ±360° en todas las articulaciones
Q_MIN = np.full(N_GDL, -2 * np.pi)
Q_MAX = np.full(N_GDL, 2 * np.pi)


def dh_matrix(theta, d, a, alpha):
    """Matriz {i-1}A_i de la convención DH estándar."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([[ct, -st * ca,  st * sa, a * ct],
                     [st,  ct * ca, -ct * sa, a * st],
                     [0.0,     sa,       ca,      d],
                     [0.0,    0.0,      0.0,    1.0]])


def fk_frames(q):
    """Devuelve [0T0, 0T1, ..., 0T6, 0T_tool] (8 matrices 4x4)."""
    T = np.eye(4)
    marcos = [T.copy()]
    for (a, alpha, d, off), qi in zip(DH_TABLA, q):
        T = T @ dh_matrix(qi + off, d, a, alpha)
        marcos.append(T.copy())
    marcos.append(T @ T_TOOL)
    return marcos


def fk(q):
    """Cinemática directa: 0T_tool (pose de la punta de la boquilla respecto a la base)."""
    return fk_frames(q)[-1]


def jacobiano(q):
    """Jacobiano geométrico 6xN de la punta de la boquilla, expresado en la base.
    Columna i (articulación rotacional):  Jv = z_{i-1} x (p_tool - o_{i-1}),  Jw = z_{i-1}
    """
    marcos = fk_frames(q)
    p = marcos[-1][:3, 3]
    J = np.zeros((6, N_GDL))
    for i in range(N_GDL):
        z = marcos[i][:3, 2]
        o = marcos[i][:3, 3]
        J[:3, i] = np.cross(z, p - o)
        J[3:, i] = z
    return J


def error_pose(T_actual, p_obj, eje_obj):
    """Error de 'pose 5D': posición completa + alineación del eje z de la boquilla.
    El giro de la boquilla sobre su propio eje queda libre (la boquilla es simétrica).
    e = [p_obj - p ;  z x eje_obj]
    """
    p = T_actual[:3, 3]
    z = T_actual[:3, 2]
    e_pos = p_obj - p
    e_rot = np.cross(z, eje_obj)
    return np.concatenate([e_pos, e_rot])


def angulo_entre(u, v):
    """Ángulo [rad] entre dos vectores."""
    c = np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v))
    return float(np.arccos(np.clip(c, -1.0, 1.0)))


def ik(p_obj, eje_obj, q0, tol_pos=1e-5, tol_ang=1e-4, max_iter=300, lam=0.05):
    """Cinemática inversa numérica por mínimos cuadrados amortiguados (DLS).

        dq = J^T (J J^T + lambda^2 I)^-1 e

    p_obj   : posición objetivo de la punta (en la base) [m]
    eje_obj : dirección deseada del eje de la boquilla (vector unitario, en la base)
    q0      : semilla (normalmente la configuración actual -> soluciones continuas)
    Devuelve (q, convergio, iteraciones).
    """
    q = np.array(q0, dtype=float)
    eje_obj = np.asarray(eje_obj, dtype=float) / np.linalg.norm(eje_obj)
    for it in range(max_iter):
        T = fk(q)
        e = error_pose(T, p_obj, eje_obj)
        ang = angulo_entre(T[:3, 2], eje_obj)
        if np.linalg.norm(e[:3]) < tol_pos and ang < tol_ang:
            return q, True, it
        J = jacobiano(q)
        dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), e)
        # limitar el paso para no saltar a otra rama de la solución
        paso = np.max(np.abs(dq))
        if paso > 0.2:
            dq *= 0.2 / paso
        q = np.clip(q + dq, Q_MIN, Q_MAX)
    T = fk(q)
    ok = (np.linalg.norm(p_obj - T[:3, 3]) < tol_pos * 10
          and angulo_entre(T[:3, 2], eje_obj) < tol_ang * 10)
    return q, ok, max_iter


def tabla_dh_texto():
    """Tabla DH para imprimir en consola o pegar en el informe."""
    filas = ["i | theta_i      | d_i [m]   | a_i [m]   | alpha_i",
             "--+--------------+-----------+-----------+--------"]
    for i, (a, alpha, d, off) in enumerate(DH_TABLA, 1):
        filas.append(f"{i} | q{i}{'+%.2f' % off if off else '':<10} | {d:<9.5f} | {a:<9.5f} | {np.degrees(alpha):+.0f}°")
    return "\n".join(filas)


if __name__ == "__main__":
    print(tabla_dh_texto())
    q = np.radians([0, -90, 90, -90, -90, 0])
    np.set_printoptions(precision=4, suppress=True)
    print("\nq =", np.degrees(q), "°")
    print("0T_tool =\n", fk(q))
