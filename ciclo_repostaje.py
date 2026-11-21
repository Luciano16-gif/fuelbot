"""
ciclo_repostaje.py: Programa principal.

Uso (con CoppeliaSim abierto, en una escena nueva y vacía):
    python ciclo_repostaje.py                 # 4 ciclos, uno por tipo de vehículo, 5 litros cada uno
    python ciclo_repostaje.py --ciclos 20     # corrida de verificación (objetivo general)
    python ciclo_repostaje.py --tipo suv      # solo un tipo de vehículo
    python ciclo_repostaje.py --sin-coppelia  # corre todo sin simulador (prueba rápida del código)

Máquina de estados de cada ciclo:
    LLEGADA -> DETECCION (placa, carril, boca y decisión de aproximación)
            -> APROXIMACION (PTP quíntico) -> INSERCION (lineal)
            -> SURTIDO (caudal x tiempo) -> EXTRACCION (lineal) -> REPOSO (PTP quíntico) -> SALIDA
Si en DETECCION el robot decide no repostar (carro en el carril equivocado o boca fuera de
alcance), salta directo a SALIDA sin mover el brazo.

Salidas:
    resultados/ciclos.csv  una fila por ciclo: tipo, decisión, errores de sensores, de inserción y de volumen
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

NOMBRE_LADO = {-1: "izquierdo (-y)", +1: "derecho (+y)"}


# ----------------------------------------------------------------------------
# prueba para correr el ciclo sin CoppeliaSim
# ----------------------------------------------------------------------------
class EscenaSinSimulador:
    def __init__(self):
        self.q = np.array(esc.Q_REPOSO)
        self.carro = np.zeros(2)
        self.tipo = None
        self.lado = None
        self.cuadros = []

    def construir(self):
        pass

    def preparar_vehiculo(self, tipo, lado):
        self.tipo, self.lado = tipo, lado

    def colocar_vehiculo(self, x, dy):
        self.carro = np.array([x, dy])

    def leer_placa(self):
        return self.tipo

    def poner_q(self, q):
        self.q = np.array(q)

    def leer_q(self):
        return self.q.copy()

    def pose_punta(self):
        return esc.a_mundo(m.fk(self.q))

    def medir_sensores(self, ruido_mm=0.0, rng=None, respaldo=None):
        rng = rng or np.random.default_rng()
        return {k: v + rng.normal(0, ruido_mm / 1000) for k, v in respaldo.items()}

    def marcar_surtido(self, activo):
        pass

    def log(self, texto):
        print(texto)


# ----------------------------------------------------------------------------
CLAVES_CICLO = ["ciclo", "tipo", "carril", "altura_boca_m", "dx_mm", "dy_mm", "decision",
                "err_sensor_lat_mm", "err_sensor_tras_mm", "err_estim_boca_mm",
                "manip_insercion", "manip_min_lineal", "desv_recta_max_mm",
                "err_insercion_mm", "err_orientacion_grados", "litros_pedidos", "litros_despachados",
                "err_volumen_pct", "err_reposo_grados", "cumple"]


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
                           round(err, 4), round(m.manipulabilidad(q_act), 5), round(self.litros, 3)])

    def _ejecutar(self, lista_q):
        for q in lista_q:
            self._paso(q)

    def _esperar(self, segundos):
        for _ in range(int(round(segundos / self.dt))):
            self._paso()

    def decidir_aproximacion(self, lecturas, tipo):
        """Decide si se puede repostar y cómo. Devuelve (plan, motivo_rechazo).
        1. El carril ocupado sale de los sensores laterales.
        2. La boca tiene que quedar hacia la isla: el carril tiene que coincidir con el lado de la
           boca que dice la base de datos para ese tipo.
        3. La semilla de la IK se elige según el carril (q1 girado hacia ese lado).
        4. El vía point y el punto de inserción tienen que ser alcanzables con la boquilla
           alineada con el eje del cuello."""
        lado = esc.detectar_carril(lecturas)
        if lado is None:
            return None, "no se detectó un carro en un solo carril"
        if lado != esc.lado_de(tipo):
            return None, (f"boca {esc.TIPOS_VEHICULO[tipo]['boca']} con el carro en el carril "
                          f"{NOMBRE_LADO[lado]}: la boca queda del lado contrario a la isla")
        p_boca = esc.estimar_boca(lecturas, tipo, lado)
        eje = esc.eje_boca(lado)
        p_aprox = esc.a_base(p_boca - esc.DIST_APROXIMACION * eje)
        p_inser = esc.a_base(p_boca + esc.PROFUNDIDAD_INSERCION * eje)
        q_aprox, ok1, it = m.ik(p_aprox, eje, esc.q_semilla(lado))
        q_inser, ok2, _ = m.ik(p_inser, eje, q_aprox)
        if not (ok1 and ok2):
            return None, "boca fuera del espacio de trabajo con la orientación requerida"
        plan = dict(lado=lado, p_boca=p_boca, eje=eje, p_aprox=p_aprox, p_inser=p_inser,
                    q_aprox=q_aprox, iteraciones=it, manip_insercion=m.manipulabilidad(q_inser))
        return plan, ""

    def ciclo_completo(self, tipo, dx, dy, litros_obj, caudal, ruido_mm, rng, lado=None):
        """lado: carril donde se estaciona el carro. Por defecto, el que deja la boca hacia la isla."""
        e = self.e
        self.ciclo += 1
        self.litros = 0.0
        lado = esc.lado_de(tipo) if lado is None else lado
        res = dict.fromkeys(CLAVES_CICLO, "")
        res.update(ciclo=self.ciclo, tipo=tipo, carril=lado,
                   altura_boca_m=esc.TIPOS_VEHICULO[tipo]["altura_boca"],
                   dx_mm=round(dx * 1000, 1), dy_mm=round(dy * 1000, 1), cumple=False)
        e.preparar_vehiculo(tipo, lado)

        # el carro entra desde adelante y se detiene
        self.estado = "LLEGADA"
        e.log(f"--- Ciclo {self.ciclo}: {tipo} llegando al carril {NOMBRE_LADO[lado]} "
              f"(dx={dx*1000:+.0f} mm, dy={dy*1000:+.0f} mm)")
        n = int(round(2.0 / self.dt))
        for k in range(n + 1):
            s = 1 - (1 - k / n) ** 2
            e.colocar_vehiculo(dx + 6.0 * (1 - s), dy)
            self._paso()

        # placa -> modelo; sensores -> carril y posición estimada de la boca; decisión
        self.estado = "DETECCION"
        tipo_leido = e.leer_placa()
        ideal = esc.lecturas_ideales(dx, dy, tipo, lado)
        lecturas = e.medir_sensores(ruido_mm, rng, respaldo=ideal)
        plan, motivo = self.decidir_aproximacion(lecturas, tipo_leido)
        self._esperar(0.5)

        if plan is None:
            res["decision"] = "rechazar: " + motivo
            e.log(f"Decisión: NO repostar ({motivo})")
        else:
            res["decision"] = "repostar"
            p_boca_real = esc.posicion_boca(dx, dy, tipo, lado)
            res["err_sensor_lat_mm"] = round((lecturas[("lat", lado)] - ideal[("lat", lado)]) * 1000, 2)
            res["err_sensor_tras_mm"] = round((lecturas[("tras", lado)] - ideal[("tras", lado)]) * 1000, 2)
            res["err_estim_boca_mm"] = round(float(np.linalg.norm(plan["p_boca"] - p_boca_real)) * 1000, 2)
            res["manip_insercion"] = round(plan["manip_insercion"], 4)
            e.log(f"Placa: {tipo_leido}. Carril {NOMBRE_LADO[lado]}, boca a {plan['p_boca'][2]:.2f} m, "
                  f"estimada con error {res['err_estim_boca_mm']:.2f} mm")
            e.log(f"Decisión: repostar; semilla q1 = {np.degrees(esc.q_semilla(lado)[0]):+.0f}°, "
                  f"manipulabilidad en la inserción {plan['manip_insercion']:.4f}")
            self._repostar(plan, p_boca_real, litros_obj, caudal, res)

        # el carro se va
        self.estado = "SALIDA"
        n = int(round(1.5 / self.dt))
        for k in range(n + 1):
            e.colocar_vehiculo(dx + 7.0 * (k / n) ** 2, dy)
            self._paso()

        if plan is not None:
            res["cumple"] = res["err_insercion_mm"] < 5 and res["err_orientacion_grados"] < 2
        e.log(f"Ciclo {self.ciclo} {'CUMPLE' if res['cumple'] else 'NO CUMPLE'} el objetivo general")
        return res

    def _repostar(self, plan, p_boca_real, litros_obj, caudal, res):
        e = self.e
        eje, p_aprox, p_inser = plan["eje"], plan["p_aprox"], plan["p_inser"]

        # PTP articular con perfil quíntico hasta el vía point
        self.estado = "APROXIMACION"
        e.log(f"Aproximación PTP (IK en {plan['iteraciones']} iteraciones)")
        self._ejecutar(ptp_articular(e.leer_q(), plan["q_aprox"], 3.0, self.dt))

        # línea recta sobre el eje de la boca
        self.estado = "INSERCION"
        e.log("Inserción lineal")
        qs, ps = lineal_cartesiana(e.leer_q(), p_inser, eje, 2.0, self.dt)
        desv = []
        u = (p_inser - p_aprox) / np.linalg.norm(p_inser - p_aprox)
        for q, p_des in zip(qs, ps):
            self._paso(q)
            p_real = esc.a_base(e.pose_punta()[:3, 3])
            # desviación respecto a la recta ideal
            d0 = p_real - p_aprox
            desv.append(np.linalg.norm(d0 - np.dot(d0, u) * u))
        res["desv_recta_max_mm"] = round(float(max(desv)) * 1000, 3)
        res["manip_min_lineal"] = round(min(m.manipulabilidad(q) for q in qs), 4)

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


def sortear_ciclos(args, rng):
    """Tipo, posición y carril de cada ciclo. Los tipos se turnan (compacto, sedán, SUV, pickup)
    para que la corrida los cubra a todos; dx y dy son aleatorios. verificar_colisiones.py usa
    esta misma función para repetir exactamente los mismos ciclos."""
    ciclos = []
    for k in range(args.ciclos):
        tipo = args.tipo or esc.TIPOS[k % len(esc.TIPOS)]
        dx = rng.uniform(-esc.RANGO_DX, esc.RANGO_DX)
        dy = rng.uniform(-esc.RANGO_DY, esc.RANGO_DY)
        lado = -esc.lado_de(tipo) if k + 1 == args.carril_equivocado else None
        ciclos.append((tipo, dx, dy, lado))
    return ciclos


def argumentos_comunes(ap):
    ap.add_argument("--ciclos", type=int, default=4)
    ap.add_argument("--tipo", choices=esc.TIPOS, help="usar siempre este tipo de vehículo")
    ap.add_argument("--carril-equivocado", type=int, default=0, metavar="N",
                    help="en el ciclo N el carro se estaciona con la boca del lado contrario a la isla")
    ap.add_argument("--litros", type=float, default=5.0)
    ap.add_argument("--caudal", type=float, default=esc.CAUDAL_L_S, help="L/s")
    ap.add_argument("--dt", type=float, default=0.05, help="paso de control [s]")
    ap.add_argument("--ruido-mm", type=float, default=1.0, help="ruido del sensor (desv. estándar)")
    ap.add_argument("--semilla", type=int, default=7)


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
    argumentos_comunes(ap)
    ap.add_argument("--tiempo-real", action="store_true", help="ritmo de reloj real (para grabar video)")
    ap.add_argument("--sin-coppelia", action="store_true")
    args = ap.parse_args()

    rng = np.random.default_rng(args.semilla)
    ciclos = sortear_ciclos(args, rng)
    sim = None
    if args.sin_coppelia:
        escena = EscenaSinSimulador()
        paso = lambda: escena.cuadros.append((escena.q.copy(), escena.carro.copy(), escena.tipo, escena.lado))
    else:
        sim, escena = conectar_coppelia()
        paso = sim.step

    ctl = Controlador(escena, paso, args.dt, args.tiempo_real)
    resultados = []
    try:
        for tipo, dx, dy, lado in ciclos:
            resultados.append(ctl.ciclo_completo(tipo, dx, dy, args.litros, args.caudal,
                                                 args.ruido_mm, rng, lado))
    finally:
        if sim is not None:
            sim.stopSimulation()

    # ---- guardar
    cab_pasos = (["ciclo", "t_s", "estado"] + [f"q{i}_grados" for i in range(1, 7)]
                 + ["x_modelo", "y_modelo", "z_modelo", "x_sim", "y_sim", "z_sim",
                    "err_modelo_vs_sim_mm", "manipulabilidad", "litros"])
    guardar_csv("resultados/pasos.csv", cab_pasos, ctl.filas)
    guardar_csv("resultados/ciclos.csv", CLAVES_CICLO, [[r[k] for k in CLAVES_CICLO] for r in resultados])

    print("\n================ RESUMEN ================")
    print(f"{'ciclo':>5} {'tipo':<9} {'carril':>6} {'err_ins[mm]':>11} {'err_ori[°]':>10} "
          f"{'desv_recta[mm]':>14} {'manip_ins':>9} {'err_vol[%]':>10}  cumple")
    for r in resultados:
        if r["decision"] != "repostar":
            print(f"{r['ciclo']:>5} {r['tipo']:<9} {r['carril']:>+6d}  {r['decision']}")
            continue
        print(f"{r['ciclo']:>5} {r['tipo']:<9} {r['carril']:>+6d} {r['err_insercion_mm']:>11.2f} "
              f"{r['err_orientacion_grados']:>10.3f} {r['desv_recta_max_mm']:>14.3f} "
              f"{r['manip_insercion']:>9.4f} {r['err_volumen_pct']:>10.3f}  {'sí' if r['cumple'] else 'NO'}")
    n_ok = sum(r["cumple"] for r in resultados)
    rechazados = sum(r["decision"] != "repostar" for r in resultados)
    print(f"Ciclos que cumplen: {n_ok}/{len(resultados)}")
    if rechazados:
        print(f"Ciclos en los que el robot decidió no repostar: {rechazados}")
    print(f"Error máximo modelo DH vs. simulador en toda la corrida: {ctl.err_max_modelo_sim:.4f} mm")
    print("CSV en la carpeta resultados/")
    return escena, resultados, ctl


if __name__ == "__main__":
    main()
