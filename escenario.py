"""
escenario.py: Geometría de la estación, de los vehículos y de los sensores.

Marco MUNDO: origen en el piso bajo la base del robot; ejes paralelos a la base del robot.
El robot está en el centro de la isla. A cada lado de la isla hay un carril:
    lado = -1   carril del lado -y del robot
    lado = +1   carril del lado +y del robot
Los vehículos miran hacia +x (la trompa en +x). Un carro con la boca a la izquierda la tiene en su
costado +y, así que se estaciona en el carril -1 para dejarla hacia la isla; uno con la boca a la
derecha se estaciona en el carril +1.

Todas las medidas en metros.
"""
import numpy as np

# --- Robot -------------------------------------------------------------------
ALTURA_BASE = 0.70                     # la base del brazo está sobre un pedestal
T_MUNDO_BASE = np.array([[1, 0, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, ALTURA_BASE],
                         [0, 0, 0, 1]], dtype=float)
Q_REPOSO = np.radians([90, -130, 120, -80, -90, 0])   # brazo recogido, boquilla hacia abajo


def q_semilla(lado):
    """Semilla de la IK para cada carril. Es la configuración de reposo girada en q1 hacia el
    carril, así la IK converge a la rama de codo arriba y muñeca hacia afuera de ese lado."""
    q = np.array(Q_REPOSO)
    if lado == +1:
        q[0] -= np.pi
    return q


# --- Isla ----------------------------------------------------------------------
ISLA_ANCHO = 0.90                      # en y, centrada en el robot
ISLA_LARGO = 3.00

# --- Vehículos -----------------------------------------------------------------
DIST_LATERAL_NOMINAL = 0.70            # distancia y del costado del carro a la base [m]
INCLINACION_BOCA = np.radians(30)      # el cuello de llenado baja 30° hacia el interior

# Base de datos de modelos. Las medidas x van respecto a la boca de llenado (x = 0):
#   trasera / delantera  posición de las defensas
#   cabina               tramo x que ocupa la cabina (vidrios)
#   ejes                 posición x de los ejes de las ruedas
# Las alturas (z) van respecto al piso.
TIPOS_VEHICULO = {
    "compacto": dict(boca="izquierda", altura_boca=0.85, trasera=-0.65, delantera=3.15,
                     ancho=1.70, z_piso=0.28, z_cintura=0.95, z_techo=1.45,
                     cabina=(0.10, 2.10), ejes=(-0.05, 2.40), radio_rueda=0.30,
                     color=[0.10, 0.35, 0.75]),
    "sedan":    dict(boca="izquierda", altura_boca=0.95, trasera=-0.80, delantera=3.50,
                     ancho=1.80, z_piso=0.30, z_cintura=1.05, z_techo=1.48,
                     cabina=(0.10, 2.40), ejes=(0.00, 2.70), radio_rueda=0.32,
                     color=[0.70, 0.08, 0.10]),
    "suv":      dict(boca="derecha", altura_boca=1.05, trasera=-0.85, delantera=3.85,
                     ancho=1.90, z_piso=0.40, z_cintura=1.15, z_techo=1.75,
                     cabina=(-0.70, 2.70), ejes=(0.05, 2.85), radio_rueda=0.36,
                     color=[0.20, 0.20, 0.22]),
    "pickup":   dict(boca="derecha", altura_boca=1.15, trasera=-1.30, delantera=4.00,
                     ancho=1.95, z_piso=0.50, z_cintura=1.25, z_techo=1.85,
                     cabina=(0.40, 2.40), ejes=(-0.40, 2.90), radio_rueda=0.38,
                     color=[0.85, 0.85, 0.80]),
}
TIPOS = tuple(TIPOS_VEHICULO)


def lado_de(tipo):
    """Carril en el que se estaciona el tipo: la boca tiene que quedar hacia la isla."""
    return -1 if TIPOS_VEHICULO[tipo]["boca"] == "izquierda" else +1


def eje_boca(lado):
    """Eje del cuello de llenado (unitario): entra al carro (alejándose de la isla) y baja."""
    return np.array([0.0, lado * np.cos(INCLINACION_BOCA), -np.sin(INCLINACION_BOCA)])


# Posición aleatoria del vehículo alrededor de la nominal
RANGO_DX = 0.10                        # ±10 cm a lo largo
RANGO_DY = 0.05                        # ±5 cm a lo ancho

# --- Movimiento ---------------------------------------------------------------
DIST_APROXIMACION = 0.12               # vía point: 12 cm afuera, sobre el eje de la boca
PROFUNDIDAD_INSERCION = 0.06           # la punta entra 6 cm en el cuello de llenado

# --- Sensores  ------------------------------------------
# En cada carril hay dos sensores de distancia:
#   lateral: en un poste en el borde de la isla, mide la distancia al costado del carro
#   trasero: en un poste detrás del carro, mide la distancia a la defensa trasera
# El sensor lateral del carril vacío no ve nada cerca; así se sabe en qué carril está el carro.
RADIO_SENSOR = 0.03
ALTURA_SENSOR = 0.65                   # dentro de la carrocería de todos los tipos


def sensor_lateral(lado):
    return np.array([0.60, lado * 0.40, ALTURA_SENSOR])


def sensor_trasero(lado):
    return np.array([-2.00, lado * (DIST_LATERAL_NOMINAL + 0.95), ALTURA_SENSOR])


UMBRAL_CARRIL = 0.60                   # lectura lateral menor a esto: hay carro en ese carril

# --- Surtidor -----------------------------------------------------------------
CAUDAL_L_S = 0.6                       # ≈ 36 L/min (el proemdio segun lo que consegui en google, eso podemos verlo luego)
SURTIDOR_POS = np.array([-0.95, 0.0, 0.0])


def posicion_boca(dx, dy, tipo, lado=None):
    """Posición REAL de la boca de llenado (mundo) para un carro desplazado (dx, dy).
    lado: carril donde está el carro; por defecto, el que deja la boca hacia la isla."""
    lado = lado_de(tipo) if lado is None else lado
    return np.array([dx, lado * (DIST_LATERAL_NOMINAL + dy), TIPOS_VEHICULO[tipo]["altura_boca"]])


def caja_carroceria(dx, dy, tipo, lado=None):
    """Esquinas (mínima, máxima) de la carrocería en el mundo."""
    v = TIPOS_VEHICULO[tipo]
    lado = lado_de(tipo) if lado is None else lado
    y_costado = lado * (DIST_LATERAL_NOMINAL + dy)
    y_lejos = y_costado + lado * v["ancho"]
    lo = np.array([dx + v["trasera"], min(y_costado, y_lejos), v["z_piso"]])
    hi = np.array([dx + v["delantera"], max(y_costado, y_lejos), v["z_cintura"]])
    return lo, hi


def _dist_esfera_caja(centro, lo, hi):
    return float(np.linalg.norm(centro - np.clip(centro, lo, hi))) - RADIO_SENSOR


def lecturas_ideales(dx, dy, tipo, lado=None):
    """Lo que medirían sensores perfectos: distancia de cada sensor a la carrocería
    (sirve para validar la lectura del simulador). Claves: ('lat', lado) y ('tras', lado)."""
    lo, hi = caja_carroceria(dx, dy, tipo, lado)
    return {(s, lado): _dist_esfera_caja(f(lado), lo, hi)
            for lado in (-1, +1) for s, f in (("lat", sensor_lateral), ("tras", sensor_trasero))}


def detectar_carril(lecturas):
    """Carril ocupado según los sensores laterales, o None si no hay carro o hay dos."""
    ocupados = [lado for lado in (-1, +1) if lecturas[("lat", lado)] < UMBRAL_CARRIL]
    return ocupados[0] if len(ocupados) == 1 else None


def estimar_boca(lecturas, tipo, lado):
    """Reconstruye la posición de la boca a partir de las dos distancias del carril ocupado.
    La altura de la boca y su posición respecto a la defensa vienen de la base de datos del modelo."""
    v = TIPOS_VEHICULO[tipo]
    y_costado = sensor_lateral(lado)[1] + lado * (lecturas[("lat", lado)] + RADIO_SENSOR)
    x_trasera = sensor_trasero(lado)[0] + (lecturas[("tras", lado)] + RADIO_SENSOR)
    return np.array([x_trasera - v["trasera"], y_costado, v["altura_boca"]])


def a_base(p_mundo):
    """Convierte un punto del mundo al marco de la base del robot."""
    return (np.linalg.inv(T_MUNDO_BASE) @ np.append(p_mundo, 1.0))[:3]


def a_mundo(T_base):
    """Convierte una pose 4x4 de la base al mundo."""
    return T_MUNDO_BASE @ T_base
