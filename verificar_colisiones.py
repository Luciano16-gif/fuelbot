"""
verificar_colisiones.py: Mide en CoppeliaSim la holgura del brazo con el carro y con la estación
durante el ciclo de repostaje.

Corre los mismos ciclos que ciclo_repostaje.py (misma semilla, mismos tipos y posiciones del carro)
y, en cada paso, mide con sim.checkDistance la distancia mínima entre tres pares de grupos:

    brazo_carro      todo el brazo menos la boquilla  vs. el carro (carrocería, cabina, ruedas, boca, tapa)
    boquilla_carro   el tubo de la boquilla           vs. el carro
    robot_estacion   todo el brazo con la boquilla     vs. surtidor, pantalla y postes de sensores

El pedestal y la losa de la isla no se incluyen en la estación porque el brazo está montado sobre ellos.

Ojo: la carrocería es un bloque macizo, sin cuello hueco. Durante INSERCION, SURTIDO y EXTRACCION
la boquilla atraviesa el bloque por diseño, así que en esos estados boquilla_carro vale 0 y no indica
un choque real. En los demás estados sí cuenta.

Uso (con CoppeliaSim abierto):
    python verificar_colisiones.py --ciclos 20

Salida: resultados/colisiones.csv, una fila por ciclo y estado con la holgura mínima de cada par.
No toca ciclos.csv ni pasos.csv.
"""
import argparse
import csv
import os
import numpy as np

import modelo_dh as m
import escenario as esc
from ciclo_repostaje import Controlador, conectar_coppelia, sortear_ciclos, argumentos_comunes

PARES = ("brazo_carro", "boquilla_carro", "robot_estacion")
ESTADOS_BOQUILLA_DENTRO = ("INSERCION", "SURTIDO", "EXTRACCION")


class MedidorHolgura:
    """Arma las colecciones de CoppeliaSim y mide las distancias mínimas."""

    def __init__(self, sim, escena):
        """Se crea con la escena recién construida, antes de que llegue el primer carro."""
        self.sim = sim
        self.e = escena
        forma = sim.object_shape_type
        raiz = sim.getObject("/FuelBot")
        base = sim.getObject("/FuelBot/Base")
        todas = set(sim.getObjectsInTree(raiz, forma, 0))
        robot = set(sim.getObjectsInTree(base, forma, 0))
        # el brazo está montado sobre el pedestal, que a su vez está sobre la losa de la isla:
        # ninguno de los dos es un obstáculo que el brazo pueda alcanzar en el ciclo
        piso = {sim.getObject("/FuelBot/Pedestal"), sim.getObject("/FuelBot/Isla")}
        estacion = todas - robot - piso
        tubo = self._tubo_boquilla(robot)
        boquilla = {tubo}
        brazo = robot - boquilla
        self._hacer_medibles(todas)

        self.grupos = {"brazo": brazo, "boquilla": boquilla, "robot": robot, "estacion": estacion}
        self.col = {k: self._coleccion(v) for k, v in self.grupos.items()}
        self.carro_actual = None
        self.pares = {"brazo_carro": ("brazo", "carro"),
                      "boquilla_carro": ("boquilla", "carro"),
                      "robot_estacion": ("robot", "estacion")}
        self.nombres = {tubo: "TuboBoquilla"}

    def _tubo_boquilla(self, robot):
        """El tubo de la boquilla no tiene alias: es la forma del robot cuyo centro coincide con
        el punto medio entre la brida + 10 cm y la punta (así lo arma escena_coppelia.py)."""
        marcos = [esc.a_mundo(F) for F in m.fk_frames(self.e.leer_q())]
        F6, Ft = marcos[6], marcos[7]
        centro = (F6[:3, 3] + 0.10 * F6[:3, 2] + Ft[:3, 3]) / 2
        dist = {h: np.linalg.norm(np.array(self.sim.getObjectPosition(h, -1)) - centro) for h in robot}
        h = min(dist, key=dist.get)
        if dist[h] > 1e-3:
            raise RuntimeError("No se encontró el tubo de la boquilla en la escena")
        return h

    def _hacer_medibles(self, formas):
        """checkDistance solo considera formas "measurable"."""
        props = self.sim.objectspecialproperty_collidable | self.sim.objectspecialproperty_measurable
        for h in formas:
            self.sim.setObjectSpecialProperty(h, self.sim.getObjectSpecialProperty(h) | props)

    def actualizar_carro(self):
        """El carro se arma de nuevo cuando cambia el tipo: rehace su colección."""
        if self.e.carro == self.carro_actual:
            return
        if "carro" in self.col:
            self.sim.destroyCollection(self.col["carro"])
        carro = set(self.sim.getObjectsInTree(self.e.carro, self.sim.object_shape_type, 0))
        self._hacer_medibles(carro)
        self.grupos["carro"] = carro
        self.col["carro"] = self._coleccion(carro)
        self.carro_actual = self.e.carro

    def _coleccion(self, formas):
        c = self.sim.createCollection(0)
        for h in formas:
            self.sim.addItemToCollection(c, self.sim.handle_single, h, 0)
        return c

    def nombre(self, h):
        """Alias legible: el de la forma, o el de su padre si la forma no tiene uno propio."""
        if h not in self.nombres:
            alias = self.sim.getObjectAlias(h, 0)
            padre = self.sim.getObjectParent(h)
            if alias.split("[")[0] in ("Cuboid", "Cylinder", "Spheroid", "Shape") and padre != -1:
                alias = f"{self.sim.getObjectAlias(padre, 0)}/{alias}"
            self.nombres[h] = alias
        return self.nombres[h]

    def medir(self):
        """Devuelve {par: (distancia_m, forma_a, forma_b)}."""
        self.actualizar_carro()
        res = {}
        for par, (a, b) in self.pares.items():
            r = self.sim.checkDistance(self.col[a], self.col[b], 0)
            res[par] = (float(r[1][6]), r[2][0], r[2][1])
        return res


class ControladorConHolgura(Controlador):
    def __init__(self, *args, medidor, **kwargs):
        super().__init__(*args, **kwargs)
        self.medidor = medidor
        self.tipos = {}            # ciclo -> tipo de vehículo
        self.minimos = {}          # (ciclo, estado, par) -> (distancia, forma_a, forma_b)

    def _paso(self, q=None):
        super()._paso(q)
        self.tipos[self.ciclo] = self.e.tipo
        for par, (d, ha, hb) in self.medidor.medir().items():
            clave = (self.ciclo, self.estado, par)
            if clave not in self.minimos or d < self.minimos[clave][0]:
                # el nombre se resuelve ya: el carro se borra cuando cambia el tipo
                self.minimos[clave] = (d, self.medidor.nombre(ha), self.medidor.nombre(hb))


def main():
    ap = argparse.ArgumentParser(description="Holgura del brazo durante el ciclo (FPTEN27)")
    argumentos_comunes(ap)
    args = ap.parse_args()

    rng = np.random.default_rng(args.semilla)      # mismos ciclos que ciclo_repostaje.py
    ciclos = sortear_ciclos(args, rng)
    sim, escena = conectar_coppelia()
    medidor = MedidorHolgura(sim, escena)
    ctl = ControladorConHolgura(escena, sim.step, args.dt, medidor=medidor)
    try:
        for tipo, dx, dy, lado in ciclos:
            ctl.ciclo_completo(tipo, dx, dy, args.litros, args.caudal, args.ruido_mm, rng, lado)
    finally:
        sim.stopSimulation()

    # ---- guardar
    filas = []
    for (ciclo, estado, par), (d, na, nb) in sorted(ctl.minimos.items(), key=lambda kv: kv[0][0]):
        filas.append([ciclo, ctl.tipos[ciclo], estado, par, round(d * 1000, 1), na, nb])
    os.makedirs("resultados", exist_ok=True)
    with open("resultados/colisiones.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["ciclo", "tipo", "estado", "par", "holgura_min_mm", "forma_a", "forma_b"])
        w.writerows(filas)

    # ---- resumen: peor caso de cada estado y par en todos los ciclos
    estados = list(dict.fromkeys(e for (_, e, _) in ctl.minimos))
    print("\n========== HOLGURA MÍNIMA EN TODOS LOS CICLOS [mm] ==========")
    print(f"{'estado':<13}" + "".join(f"{p:>16}" for p in PARES))
    choques = []
    for e in estados:
        celdas = []
        for p in PARES:
            casos = [(v, c) for (c, e2, p2), v in ctl.minimos.items() if e2 == e and p2 == p]
            (d, na, nb), c = min(casos, key=lambda x: x[0][0])
            marca = "*" if (p == "boquilla_carro" and e in ESTADOS_BOQUILLA_DENTRO) else ""
            celdas.append(f"{d * 1000:>15.1f}{marca or ' '}")
            if d <= 0 and not marca:
                choques.append((c, e, p, na, nb))
        print(f"{e:<13}" + "".join(celdas))
    print("* la boquilla está dentro del bloque macizo del carro por diseño; no cuenta como choque")

    # ---- peor caso por tipo de vehículo (brazo con el carro, en cualquier estado)
    print(f"\n{'tipo':<9} {'brazo_carro[mm]':>15}  estado        formas")
    for t in esc.TIPOS:
        casos = [(v, e2) for (c, e2, p2), v in ctl.minimos.items() if p2 == "brazo_carro" and ctl.tipos[c] == t]
        if casos:
            (d, na, nb), e2 = min(casos, key=lambda x: x[0][0])
            print(f"{t:<9} {d * 1000:>15.1f}  {e2:<13} {na} con {nb}")
    if choques:
        print("\nCONTACTOS DETECTADOS:")
        for c in choques:
            print(f"  ciclo {c[0]}, {c[1]}, {c[2]}: {c[3]} con {c[4]}")
    else:
        print("\nSin contactos fuera de la inserción en ningún ciclo.")
    print("Detalle por ciclo en resultados/colisiones.csv")


if __name__ == "__main__":
    main()
