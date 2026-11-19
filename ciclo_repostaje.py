"""
ciclo_repostaje.py: Programa principal.

Uso (con CoppeliaSim abierto, en una escena nueva y vacía):
    python ciclo_repostaje.py                 # 3 ciclos, 5 litros cada uno
    python ciclo_repostaje.py --ciclos 10     # corrida de verificación (objetivo general)
    python ciclo_repostaje.py --sin-coppelia  # corre todo sin simulador (prueba rápida del código)

Máquina de estados de cada ciclo:
    LLEGADA -> DETECCION -> APROXIMACION (PTP quíntico) -> INSERCION (lineal)
            -> SURTIDO (caudal x tiempo) -> EXTRACCION (lineal) -> REPOSO (PTP quíntico) -> SALIDA

Salidas:
    resultados/ciclos.csv  una fila por ciclo: errores de sensores, de inserción y de volumen
    resultados/pasos.csv   una fila por paso de simulación: q, punta según modelo y según simulador
"""
import argparse
import csv
import os
import time
import numpy as np

import modelo_dh as m
import escenario as esc
from trayectorias import ptp_articular, lineal_cartesiana


# ----------------------------------------------------------------------------
# prueba para correr el ciclo sin CoppeliaSim 
# ----------------------------------------------------------------------------
class EscenaSinSimulador:
    def __init__(self):
        self.q = np.array(esc.Q_REPOSO)
        self.carro = np.zeros(2)
        self.cuadros = []          

    def construir(self):
        pass

    def colocar_vehiculo(self, x, dy):
        self.carro = np.array([x, dy])

    def poner_q(self, q):
        self.q = np.array(q)

    def leer_q(self):
        return self.q.copy()

    def pose_punta(self):
        return esc.a_mundo(m.fk(self.q))

    def medir_sensores(self, ruido_mm=0.0, rng=None, respaldo=None):
        rng = rng or np.random.default_rng()
        return [v + rng.normal(0, ruido_mm / 1000) for v in respaldo]

    def marcar_surtido(self, activo):
        pass

    def log(self, texto):
        print(texto)


# ----------------------------------------------------------------------------
class Controlador:
    def __init__(self, escena, paso_sim, dt, tiempo_real=False):
        self.e = escena
        # función que avanza un paso del simulador
        self.paso_sim = paso_sim       
        self.dt = dt
        self.tiempo_real = tiempo_real
        self.t = 0.0
        self.filas = []
        self.estado = ""
        self.ciclo = 0
        self.litros = 0.0
        self.err_max_modelo_sim = 0.0

    def _paso(self, q=None):
        if q is not None:
            self.e.poner_q(q)
        self.paso_sim()
        self.t += self.dt
        if self.tiempo_real:
            time.sleep(self.dt)
        # verificación continua: punta según el modelo DH vs. según el simulador
        q_act = self.e.leer_q()
        p_modelo = esc.a_mundo(m.fk(q_act))[:3, 3]
        p_sim = self.e.pose_punta()[:3, 3]
        err = float(np.linalg.norm(p_modelo - p_sim)) * 1000
        self.err_max_modelo_sim = max(self.err_max_modelo_sim, err)
        self.filas.append([self.ciclo, round(self.t, 3), self.estado,
                           *np.round(np.degrees(q_act), 4),
                           *np.round(p_modelo, 6), *np.round(p_sim, 6),
                           round(err, 4), round(self.litros, 3)])

    def _ejecutar(self, lista_q):
        for q in lista_q:
            self._paso(q)

    def _esperar(self, segundos):
        for _ in range(int(round(segundos / self.dt))):
            self._paso()

    def ciclo_completo(self, dx, dy, litros_obj, caudal, ruido_mm, rng):
        e = self.e
        self.ciclo += 1
        self.litros = 0.0
        res = {"ciclo": self.ciclo, "dx_mm": round(dx * 1000, 1), "dy_mm": round(dy * 1000, 1)}

        # el carro entra desde adelante y se detiene
        self.estado = "LLEGADA"
        e.log(f"--- Ciclo {self.ciclo}: vehículo llegando (dx={dx*1000:+.0f} mm, dy={dy*1000:+.0f} mm)")
        n = int(round(2.0 / self.dt))
        for k in range(n + 1):
            s = 1 - (1 - k / n) ** 2                      
            e.colocar_vehiculo(dx + 6.0 * (1 - s), dy)
            self._paso()

        # sensores para la posición estimada de la boca de llenado
        self.estado = "DETECCION"
        ideal = esc.lecturas_ideales(dx, dy)
        d_lat, d_tras = e.medir_sensores(ruido_mm, rng, respaldo=ideal)
        p_boca_est = esc.estimar_boca(d_lat, d_tras)
        p_boca_real = esc.posicion_boca(dx, dy)
        res["err_sensor_lat_mm"] = round((d_lat - ideal[0]) * 1000, 2)
        res["err_sensor_tras_mm"] = round((d_tras - ideal[1]) * 1000, 2)
        res["err_estim_boca_mm"] = round(float(np.linalg.norm(p_boca_est - p_boca_real)) * 1000, 2)
        e.log(f"Sensores: lateral {d_lat*1000:.1f} mm, trasero {d_tras*1000:.1f} mm "
              f"-> boca estimada con error {res['err_estim_boca_mm']:.2f} mm")
        self._esperar(0.5)

        # objetivos en el marco de la BASE del robot
        eje = esc.EJE_BOCA
        p_aprox = esc.a_base(p_boca_est - esc.DIST_APROXIMACION * eje)
        p_inser = esc.a_base(p_boca_est + esc.PROFUNDIDAD_INSERCION * eje)

        # PTP articular con perfil quíntico hasta el vía point
        self.estado = "APROXIMACION"
        q_aprox, ok, it = m.ik(p_aprox, eje, e.leer_q())
        if not ok:
            raise RuntimeError("IK sin solución para el punto de aproximación")
        e.log(f"Aproximación PTP (IK en {it} iteraciones)")
        self._ejecutar(ptp_articular(e.leer_q(), q_aprox, 3.0, self.dt))

        # línea recta sobre el eje de la boca
        self.estado = "INSERCION"
        e.log("Inserción lineal")
        qs, ps = lineal_cartesiana(e.leer_q(), p_inser, eje, 2.0, self.dt)
        desv = []
        for q, p_des in zip(qs, ps):
            self._paso(q)
            p_real = esc.a_base(e.pose_punta()[:3, 3])
            # desviación respecto a la recta ideal 
            d0 = p_real - p_aprox
            u = (p_inser - p_aprox) / np.linalg.norm(p_inser - p_aprox)
            desv.append(np.linalg.norm(d0 - np.dot(d0, u) * u))
        res["desv_recta_max_mm"] = round(float(max(desv)) * 1000, 3)

        # error de inserción contra la boca 
        T_sim = e.pose_punta()
        p_obj_real = p_boca_real + esc.PROFUNDIDAD_INSERCION * eje
        res["err_insercion_mm"] = round(float(np.linalg.norm(T_sim[:3, 3] - p_obj_real)) * 1000, 2)
        res["err_orientacion_grados"] = round(np.degrees(m.angulo_entre(T_sim[:3, 2], eje)), 3)
        e.log(f"Insertado: error de posición {res['err_insercion_mm']:.2f} mm, "
              f"orientación {res['err_orientacion_grados']:.3f}°")

        # el caudalímetro integra hasta el volumen pedido
        self.estado = "SURTIDO"
        e.marcar_surtido(True)
        e.log(f"Surtiendo {litros_obj:.1f} L a {caudal:.2f} L/s")
        while self.litros < litros_obj:
            self.litros += caudal * self.dt
            self._paso()
        e.marcar_surtido(False)
        res["litros_pedidos"] = litros_obj
        res["litros_despachados"] = round(self.litros, 3)
        res["err_volumen_pct"] = round(100 * (self.litros - litros_obj) / litros_obj, 3)

        # misma recta, de vuelta al vía point para la extraccion
        self.estado = "EXTRACCION"
        qs, _ = lineal_cartesiana(e.leer_q(), p_aprox, eje, 2.0, self.dt)
        self._ejecutar(qs)

        # PTP de vuelta a la configuración de reposo
        self.estado = "REPOSO"
        self._ejecutar(ptp_articular(e.leer_q(), esc.Q_REPOSO, 3.0, self.dt))
        res["err_reposo_grados"] = round(float(np.max(np.abs(np.degrees(e.leer_q() - esc.Q_REPOSO)))), 4)

        # el carro se va
        self.estado = "SALIDA"
        n = int(round(1.5 / self.dt))
        for k in range(n + 1):
            e.colocar_vehiculo(dx + 7.0 * (k / n) ** 2, dy)
            self._paso()

        res["cumple"] = res["err_insercion_mm"] < 5 and res["err_orientacion_grados"] < 2
        e.log(f"Ciclo {self.ciclo} {'CUMPLE' if res['cumple'] else 'NO CUMPLE'} el objetivo general")
        return res


def conectar_coppelia():
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    from escena_coppelia import EscenaCoppelia
    client = RemoteAPIClient()
    sim = client.require("sim")
    # detener cualquier simulación previa antes de construir
    if sim.getSimulationState() != sim.simulation_stopped:
        sim.stopSimulation()
        while sim.getSimulationState() != sim.simulation_stopped:
            time.sleep(0.05)
    escena = EscenaCoppelia(sim)
    escena.construir()
    sim.setStepping(True)
    sim.startSimulation()
    return sim, escena


def guardar_csv(ruta, encabezado, filas):
    os.makedirs(os.path.dirname(ruta), exist_ok=True)
    with open(ruta, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(encabezado)
        w.writerows(filas)


def main():
    ap = argparse.ArgumentParser(description="Ciclo de repostaje automático (FPTEN27)")
    ap.add_argument("--ciclos", type=int, default=3)
    ap.add_argument("--litros", type=float, default=5.0)
    ap.add_argument("--caudal", type=float, default=esc.CAUDAL_L_S, help="L/s")
    ap.add_argument("--dt", type=float, default=0.05, help="paso de control [s]")
    ap.add_argument("--ruido-mm", type=float, default=1.0, help="ruido del sensor (desv. estándar)")
    ap.add_argument("--semilla", type=int, default=7)
    ap.add_argument("--tiempo-real", action="store_true", help="ritmo de reloj real (para grabar video)")
    ap.add_argument("--sin-coppelia", action="store_true")
    args = ap.parse_args()

    rng = np.random.default_rng(args.semilla)
    sim = None
    if args.sin_coppelia:
        escena = EscenaSinSimulador()
        paso = lambda: escena.cuadros.append((escena.q.copy(), escena.carro.copy()))
    else:
        sim, escena = conectar_coppelia()
        paso = sim.step

    ctl = Controlador(escena, paso, args.dt, args.tiempo_real)
    resultados = []
    try:
        for _ in range(args.ciclos):
            dx = rng.uniform(-esc.RANGO_DX, esc.RANGO_DX)
            dy = rng.uniform(-esc.RANGO_DY, esc.RANGO_DY)
            resultados.append(ctl.ciclo_completo(dx, dy, args.litros, args.caudal, args.ruido_mm, rng))
    finally:
        if sim is not None:
            sim.stopSimulation()

    # ---- guardar 
    cab_pasos = (["ciclo", "t_s", "estado"] + [f"q{i}_grados" for i in range(1, 7)]
                 + ["x_modelo", "y_modelo", "z_modelo", "x_sim", "y_sim", "z_sim",
                    "err_modelo_vs_sim_mm", "litros"])
    guardar_csv("resultados/pasos.csv", cab_pasos, ctl.filas)
    claves = list(resultados[0].keys())
    guardar_csv("resultados/ciclos.csv", claves, [[r[k] for k in claves] for r in resultados])

    print("\n================ RESUMEN ================")
    print(f"{'ciclo':>5} {'err_ins[mm]':>11} {'err_ori[°]':>10} {'desv_recta[mm]':>14} {'err_vol[%]':>10}  cumple")
    for r in resultados:
        print(f"{r['ciclo']:>5} {r['err_insercion_mm']:>11.2f} {r['err_orientacion_grados']:>10.3f} "
              f"{r['desv_recta_max_mm']:>14.3f} {r['err_volumen_pct']:>10.3f}  {'sí' if r['cumple'] else 'NO'}")
    n_ok = sum(r["cumple"] for r in resultados)
    print(f"Ciclos que cumplen: {n_ok}/{len(resultados)}")
    print(f"Error máximo modelo DH vs. simulador en toda la corrida: {ctl.err_max_modelo_sim:.4f} mm")
    print("CSV en la carpeta resultados/")
    return escena, resultados, ctl


if __name__ == "__main__":
    main()
