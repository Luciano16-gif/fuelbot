"""
escena_coppelia.py: Construye la escena de la estación en CoppeliaSim desde Python
y ofrece las operaciones que usa el ciclo de repostaje.

El brazo se arma con primitivas y articulaciones a partir de la tabla DH de modelo_dh.py:
cada articulación i se coloca en el marco {i-1} de la cadena DH, con su eje de giro = z_{i-1}.
Así, lo que el simulador muestra y lo que el modelo predice salen de la misma definición, y la
verificación compara dos cálculos independientes de la misma pose.

Probado contra la API "moderna" de CoppeliaSim. Si algo falla por versión,
el mensaje de error dice qué función fue.
"""
import numpy as np
import modelo_dh as m
import escenario as esc

# matiz de colores para el carrito y el robot
GRIS_ROBOT = [0.75, 0.77, 0.80]
AZUL_JUNTA = [0.20, 0.45, 0.85]
VERDE_BOQ = [0.10, 0.60, 0.25]
ROJO_CARRO = [0.70, 0.08, 0.10]
NEGRO = [0.05, 0.05, 0.05]
VIDRIO = [0.55, 0.70, 0.80]
CONCRETO = [0.62, 0.62, 0.60]
AMARILLO = [0.95, 0.80, 0.10]


def _mat12(T):
    """numpy 4x4 -> lista de 12 valores (formato de matriz de CoppeliaSim, por filas)."""
    return [float(v) for v in np.asarray(T)[:3, :].flatten()]


def _rot_z_hacia(direccion):
    """Matriz de rotación cuyo eje z apunta en 'direccion'."""
    z = np.asarray(direccion, float)
    z = z / np.linalg.norm(z)
    aux = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
    x = np.cross(aux, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def _pose(R=np.eye(3), p=(0, 0, 0)):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


class EscenaCoppelia:
    def __init__(self, sim):
        self.sim = sim
        self.juntas = []
        self.carro = None
        self.manguera = None

    # ---------------------------------------------------------------- utilidades
    def _const(self, *nombres):
        """Devuelve la primera constante que exista (los nombres cambian entre versiones)."""
        for n in nombres:
            v = getattr(self.sim, n, None)
            if v is not None:
                return v
        raise AttributeError(f"Ninguna de estas constantes existe en tu versión: {nombres}")

    def _forma(self, tipo, tam, T_mundo, color, padre, nombre=None):
        sim = self.sim
        tipos = {"caja": "primitiveshape_cuboid", "cil": "primitiveshape_cylinder",
                 "esfera": "primitiveshape_spheroid"}
        h = sim.createPrimitiveShape(self._const(tipos[tipo]), [float(v) for v in tam], 0)
        # estática y no "respondable": nada se cae ni choca por física, el movimiento lo decideremos nosotros (podriamos cambiarlo luego segun lo que diga jose)
        for param, val in (("shapeintparam_static", 1), ("shapeintparam_respondable", 0)):
            try:
                sim.setObjectInt32Param(h, getattr(sim, param), val)
            except Exception:
                pass
        sim.setObjectMatrix(h, _mat12(T_mundo))
        sim.setShapeColor(h, None, sim.colorcomponent_ambient_diffuse, color)
        if nombre:
            sim.setObjectAlias(h, nombre)
        if padre is not None:
            sim.setObjectParent(h, padre, True)
        return h

    def _barra(self, p0, p1, diam, color, padre):
        """Cilindro entre dos puntos del mundo (para dibujar eslabones)."""
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        L = np.linalg.norm(p1 - p0)
        if L < 1e-3:
            return None
        T = _pose(_rot_z_hacia(p1 - p0), (p0 + p1) / 2)
        return self._forma("cil", [diam, diam, L], T, color, padre)

    def _dummy(self, T_mundo, nombre, padre, tam=0.02):
        h = self.sim.createDummy(tam)
        self.sim.setObjectMatrix(h, _mat12(T_mundo))
        self.sim.setObjectAlias(h, nombre)
        if padre is not None:
            self.sim.setObjectParent(h, padre, True)
        return h

    # ---------------------------------------------------------------- construcción
    def limpiar(self):
        sim = self.sim
        try:
            h = sim.getObject("/FuelBot", {"noError": True})
        except Exception:
            h = -1
        if h != -1:
            sim.removeObjects(sim.getObjectsInTree(h))

    def construir(self):
        self.limpiar()
        sim = self.sim
        self.raiz = self._dummy(np.eye(4), "FuelBot", None, 0.05)
        self._construir_isla()
        self._construir_robot()
        self._construir_carro()
        self._construir_sensores()
        try:   # manguera decorativa (ESTO NO HAY CHANCE QUE LO PROGRAMEMOS, de hecho jose no nos dijo nada al respecto) 
            self.manguera = sim.addDrawingObject(sim.drawing_lines, 4, 0, -1, 200, NEGRO)
        except Exception:
            self.manguera = None
        self.poner_q(esc.Q_REPOSO)

    def _construir_isla(self):
        r = self.raiz
        self._forma("caja", [3.0, 1.0, 0.15], _pose(p=(0, 0.25, 0.075)), CONCRETO, r, "Isla")
        self._forma("caja", [0.30, 0.30, esc.ALTURA_BASE - 0.15],
                    _pose(p=(0, 0, 0.15 + (esc.ALTURA_BASE - 0.15) / 2)), [0.3, 0.3, 0.32], r, "Pedestal")
        sx, sy, _ = esc.SURTIDOR_POS
        self._forma("caja", [0.50, 0.35, 1.70], _pose(p=(sx, sy, 0.15 + 0.85)), [0.85, 0.85, 0.85], r, "Surtidor")
        self._forma("caja", [0.40, 0.02, 0.25], _pose(p=(sx, sy - 0.18, 1.35)), [0.1, 0.2, 0.3], r, "Pantalla")

    def _construir_robot(self):
        sim = self.sim
        tipo = self._const("joint_revolute", "joint_revolute_subtype")
        modo = self._const("jointmode_kinematic", "jointmode_passive")
        marcos = [esc.a_mundo(F) for F in m.fk_frames(np.zeros(m.N_GDL))]   # q = 0
        # base 
        base = self._forma("cil", [0.16, 0.16, 0.06], marcos[0] @ _pose(p=(0, 0, -0.03)),
                           [0.25, 0.25, 0.28], self.raiz, "Base")
        padre = base
        self.juntas = []
        for i in range(m.N_GDL):
            F_ant, F_sig = marcos[i], marcos[i + 1]
            j = sim.createJoint(tipo, modo, 0, [0.15, 0.10])
            sim.setObjectMatrix(j, _mat12(F_ant))                  # eje de giro = z_{i-1}
            sim.setObjectAlias(j, f"Junta{i + 1}")
            try:
                sim.setJointInterval(j, False, [-2 * np.pi, 4 * np.pi])
            except Exception:
                pass
            sim.setObjectParent(j, padre, True)
            self.juntas.append(j)
            # carcasa de la junta + eslabón i: de O_{i-1} sube d_i por z_{i-1}, luego a_i por x_i
            self._forma("cil", [0.11, 0.11, 0.13], F_ant, AZUL_JUNTA, j)
            a, alpha, d, off = m.DH_TABLA[i]
            codo = F_ant[:3, 3] + d * F_ant[:3, 2]
            self._barra(F_ant[:3, 3], codo, 0.085, GRIS_ROBOT, j)
            self._barra(codo, F_sig[:3, 3], 0.085, GRIS_ROBOT, j)
            padre = j
        # herramienta: brida + mango + boquilla, y la punta de mentira
        F6, Ft = marcos[6], marcos[7]
        self._forma("cil", [0.09, 0.09, 0.03], F6 @ _pose(p=(0, 0, 0.015)), NEGRO, padre, "Brida")
        self._forma("caja", [0.05, 0.05, 0.08], F6 @ _pose(p=(0, 0, 0.07)), VERDE_BOQ, padre, "Mango")
        self._barra(F6[:3, 3] + 0.10 * F6[:3, 2], Ft[:3, 3], 0.025, [0.6, 0.6, 0.6], padre)
        self.punta = self._dummy(Ft, "PuntaBoquilla", padre)

    def _construir_carro(self):
        """El carro se arma alrededor de su marco (origen en la boca de llenado) y luego se mueve
        moviendo solo el dummy raíz 'Vehiculo'."""
        c = self._dummy(np.eye(4), "Vehiculo", self.raiz, 0.05)
        zb = esc.ALTURA_BOCA
        x0, x1 = esc.CARRO_X_TRASERA, esc.CARRO_X_DELANTERA
        L, W = x1 - x0, esc.CARRO_ANCHO
        def P(x, y, z):   # punto en el marco del carro 
            return (x, y, z)
        zc = (esc.CARRO_Z_PISO + esc.CARRO_Z_CINTURA) / 2
        self._forma("caja", [L, W, esc.CARRO_Z_CINTURA - esc.CARRO_Z_PISO],
                    _pose(p=P((x0 + x1) / 2, -W / 2, zc)), ROJO_CARRO, c, "Carroceria")
        self._forma("caja", [2.3, W - 0.2, esc.CARRO_Z_TECHO - esc.CARRO_Z_CINTURA],
                    _pose(p=P(1.25, -W / 2, (esc.CARRO_Z_CINTURA + esc.CARRO_Z_TECHO) / 2)), VIDRIO, c, "Cabina")
        Ry = _rot_z_hacia([0, 1, 0])
        for xr in (0.0, 2.70):
            for yr in (-0.12, -W + 0.12):
                self._forma("cil", [0.64, 0.64, 0.22], _pose(Ry, P(xr, yr, 0.32)), NEGRO, c)
        # boca de llenado que se representa con un disco oscuro en el costado + tapa abierta
        self._forma("cil", [0.07, 0.07, 0.006], _pose(Ry, P(0, 0.003, zb)), NEGRO, c, "BocaLlenado")
        self._forma("caja", [0.003, 0.12, 0.12], _pose(p=P(-0.07, 0.06, zb)), ROJO_CARRO, c, "TapaAbierta")
        self.boca_dummy = self._dummy(_pose(_rot_z_hacia(esc.EJE_BOCA), (0, 0, zb)), "EjeBoca", c)
        self.carro = c
        self.carroceria = self.sim.getObject("/FuelBot/Vehiculo/Carroceria")
        self._hacer_medible(self.carroceria)

    def _construir_sensores(self):
        r = self.raiz
        for nombre, p in (("SensorLateral", esc.SENSOR_LATERAL), ("SensorTrasero", esc.SENSOR_TRASERO)):
            self._forma("caja", [0.06, 0.06, p[2]], _pose(p=(p[0], p[1], p[2] / 2)), [0.3, 0.3, 0.3], r)
            h = self._forma("esfera", [2 * esc.RADIO_SENSOR] * 3, _pose(p=p), AMARILLO, r, nombre)
            self._hacer_medible(h)
            setattr(self, nombre, h)

    def _hacer_medible(self, h):
        sim = self.sim
        try:
            props = 0
            for n in ("objectspecialproperty_collidable", "objectspecialproperty_measurable",
                      "objectspecialproperty_detectable"):
                props |= getattr(sim, n, 0)
            sim.setObjectSpecialProperty(h, props)
        except Exception:
            pass

    # ---------------------------------------------------------------- operación
    def colocar_vehiculo(self, x, dy):
        """Mueve el carro: x = posición de la boca a lo largo, dy = desplazamiento lateral."""
        self.sim.setObjectPosition(self.carro, [float(x), float(-(esc.DIST_LATERAL_NOMINAL + dy)), 0.0])

    def poner_q(self, q):
        for j, qi in zip(self.juntas, q):
            self.sim.setJointPosition(j, float(qi))
        self._actualizar_manguera()

    def leer_q(self):
        return np.array([self.sim.getJointPosition(j) for j in self.juntas])

    def pose_punta(self):
        """Pose 4x4 (mundo) de la punta de la boquilla SEGÚN EL SIMULADOR."""
        M = np.array(self.sim.getObjectMatrix(self.punta)).reshape(3, 4)
        return np.vstack([M, [0, 0, 0, 1]])

    def medir_sensores(self, ruido_mm=0.0, rng=None, respaldo=None):
        """Distancias sensor -> carrocería calculadas por el simulador (sim.checkDistance),
        con ruido gaussiano opcional para emular un ultrasonido real.
        Si checkDistance no está disponible, usa 'respaldo' (lecturas ideales) y lo avisa."""
        rng = rng or np.random.default_rng()
        lecturas = []
        for s, ideal in zip((self.SensorLateral, self.SensorTrasero), respaldo or (None, None)):
            try:
                res = self.sim.checkDistance(s, self.carroceria, 0)
                dist = float(res[1][6])
            except Exception as e:
                if ideal is None:
                    raise
                if not getattr(self, "_aviso_sensor", False):
                    self.log(f"AVISO: checkDistance falló ({e}); se usan lecturas emuladas")
                    self._aviso_sensor = True
                dist = ideal
            lecturas.append(dist + rng.normal(0, ruido_mm / 1000))
        return lecturas

    def marcar_surtido(self, activo):
        try:
            h = self.sim.getObject("/FuelBot/Vehiculo/BocaLlenado")
            self.sim.setShapeColor(h, None, self.sim.colorcomponent_ambient_diffuse,
                                   [0.1, 0.8, 0.2] if activo else NEGRO)
        except Exception:
            pass

    def log(self, texto):
        print(texto)
        try:
            self.sim.addLog(self.sim.verbosity_scriptinfos, "[FuelBot] " + texto)
        except Exception:
            pass

    def _actualizar_manguera(self):
        if self.manguera is None or not self.juntas:
            return
        try:
            a = np.array(esc.SURTIDOR_POS) + [0.0, -0.18, 1.10]
            b = self.pose_punta() @ np.array([0, 0, -0.17, 1.0])     
            b = b[:3]
            pts = []
            for k in range(13):          
                t = k / 12
                p = (1 - t) * a + t * b
                p[2] -= 0.35 * 4 * t * (1 - t)
                pts.append(p)
            self.sim.addDrawingObjectItem(self.manguera, None)      
            for p, q in zip(pts[:-1], pts[1:]):
                self.sim.addDrawingObjectItem(self.manguera, [float(v) for v in (*p, *q)])
        except Exception:
            self.manguera = None
