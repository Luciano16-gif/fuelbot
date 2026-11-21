"""
verificar_modelo.py: Pruebas del modelo sin necesidad de CoppeliaSim.

1. Cinemática directa: nuestro modelo DH vs. Robotics Toolbox for Python, construida con
   la misma tabla DH, en 20 configuraciones aleatorias.  (pip install roboticstoolbox-python)
2. Jacobiano: el analítico (geométrico) vs. uno numérico por diferencias finitas.
3. Cinemática inversa: 200 objetivos aleatorios alrededor de la boca de llenado (50 por tipo de vehículo).
"""
import numpy as np
import modelo_dh as m
import escenario as esc

rng = np.random.default_rng(0)
np.set_printoptions(precision=4, suppress=True)
print("Tabla DH (estándar):\n" + m.tabla_dh_texto() + "\n")

# FK contra una librería independiente ---------------------------------------
try:
    import roboticstoolbox as rtb
    from spatialmath import SE3
    eslabones = [rtb.RevoluteDH(d=d, a=a, alpha=al, offset=off) for (a, al, d, off) in m.DH_TABLA]
    robot = rtb.DHRobot(eslabones, name="UR5_FPTEN27", tool=SE3(0, 0, m.LONG_BOQUILLA))
    errores = []
    for _ in range(20):
        q = rng.uniform(-np.pi, np.pi, 6)
        errores.append(np.linalg.norm(m.fk(q)[:3, 3] - robot.fkine(q).t) * 1000)
    print(f"1) FK vs Robotics Toolbox (20 config.): error máx = {max(errores):.2e} mm")
except ImportError:
    print("1) Robotics Toolbox no instalada: se omite la comparación (pip install roboticstoolbox-python)")

# Jacobiano analítico vs numérico ---------------------------------------------
q = rng.uniform(-np.pi, np.pi, 6)
J = m.jacobiano(q)
Jn = np.zeros((3, 6))
h = 1e-7
for i in range(6):
    dq = np.zeros(6)
    dq[i] = h
    Jn[:, i] = (m.fk(q + dq)[:3, 3] - m.fk(q - dq)[:3, 3]) / (2 * h)
print(f"2) Jacobiano (parte lineal) analítico vs numérico: diferencia máx = {np.max(np.abs(J[:3] - Jn)):.2e}")

# IK en el entorno de trabajo ----------------------------------------------------
fallos, its, err = 0, [], []
for k in range(200):
    tipo = esc.TIPOS[k % len(esc.TIPOS)]
    lado = esc.lado_de(tipo)
    eje = esc.eje_boca(lado)
    dx, dy = rng.uniform(-esc.RANGO_DX, esc.RANGO_DX), rng.uniform(-esc.RANGO_DY, esc.RANGO_DY)
    p = esc.a_base(esc.posicion_boca(dx, dy, tipo) - esc.DIST_APROXIMACION * eje)
    q, ok, it = m.ik(p, eje, esc.q_semilla(lado))
    fallos += not ok
    its.append(it)
    err.append(np.linalg.norm(m.fk(q)[:3, 3] - p) * 1000)
print(f"3) IK en 200 posiciones del carro (4 tipos, 2 carriles): {200 - fallos}/200 convergen, "
      f"iteraciones medias {np.mean(its):.1f}, error máx {max(err):.4f} mm")
