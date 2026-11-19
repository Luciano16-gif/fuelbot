"""
escenario.py: Geometría de la estación, del vehículo y de los sensores.

Marco MUNDO: origen en el piso bajo la base del robot; ejes paralelos a la base del robot.
El vehículo se estaciona del lado -y del robot, alineado con el eje x.

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

# --- Carrito
DIST_LATERAL_NOMINAL = 0.70            # distancia y del costado del carro a la base [m]
ALTURA_BOCA = 0.95                     # altura de la boca de llenado sobre el piso [m]
INCLINACION_BOCA = np.radians(30)      # el cuello de llenado baja 30° hacia el interior
# eje de la boca: entra al carro (-y) y baja
EJE_BOCA = np.array([0.0, -np.cos(INCLINACION_BOCA), -np.sin(INCLINACION_BOCA)])

# Dimensiones del carro respecto a la boca de llenado, los datos del llenado (esto seguramente es lo que jose quiere que podamos hacer relativamente variable en la simulacion)
CARRO_X_TRASERA = -0.80                # la boca está 0,80 m por delante de la defensa trasera
CARRO_X_DELANTERA = 3.50
CARRO_ANCHO = 1.80
CARRO_Z_PISO = 0.30                    # altura del fondo de la carrocería
CARRO_Z_CINTURA = 1.05                 # altura de la línea de cintura
CARRO_Z_TECHO = 1.48

# Posición aleatoria del vehículo alrededor de la nominal, aqui es donde tenemos que variar las cosas
RANGO_DX = 0.10                        # ±10 cm a lo largo
RANGO_DY = 0.05                        # ±5 cm a lo ancho

# --- Movimiento ---------------------------------------------------------------
DIST_APROXIMACION = 0.12               # vía point: 12 cm afuera, sobre el eje de la boca
PROFUNDIDAD_INSERCION = 0.06           # la punta entra 6 cm en el cuello de llenado

# --- Sensores  ------------------------------------------
# Sensor lateral en un poste de la isla: mide la distancia al costado del carro.
SENSOR_LATERAL = np.array([0.60, -0.25, 0.55])
# Sensor longitudinal en un poste detrás del carro: mide la distancia a la defensa trasera.
SENSOR_TRASERO = np.array([-1.60, -(DIST_LATERAL_NOMINAL + CARRO_ANCHO / 2), 0.55])
RADIO_SENSOR = 0.03

# --- Surtidor -----------------------------------------------------------------
CAUDAL_L_S = 0.6                       # ≈ 36 L/min (el proemdio segun lo que consegui en google, eso podemos verlo luego)
SURTIDOR_POS = np.array([-0.95, 0.30, 0.0])


def posicion_boca(dx, dy):
    """Posición REAL de la boca de llenado (mundo) para un carro desplazado (dx, dy)."""
    return np.array([dx, -(DIST_LATERAL_NOMINAL + dy), ALTURA_BOCA])


def lecturas_ideales(dx, dy):
    """Lo que medirían sensores perfectos (sirve para validar la lectura del simulador)."""
    y_costado = -(DIST_LATERAL_NOMINAL + dy)
    d_lat = (SENSOR_LATERAL[1] - y_costado) - RADIO_SENSOR
    x_trasera = dx + CARRO_X_TRASERA
    d_tras = (x_trasera - SENSOR_TRASERO[0]) - RADIO_SENSOR
    return d_lat, d_tras


def estimar_boca(d_lat, d_tras):
    """Reconstruye la posición de la boca a partir de las dos distancias medidas.
    La altura de la boca y su posición respecto a la defensa vienen de la base de datos del modelo."""
    y_costado = SENSOR_LATERAL[1] - (d_lat + RADIO_SENSOR)
    x_trasera = SENSOR_TRASERO[0] + (d_tras + RADIO_SENSOR)
    return np.array([x_trasera - CARRO_X_TRASERA, y_costado, ALTURA_BOCA])


def a_base(p_mundo):
    """Convierte un punto del mundo al marco de la base del robot."""
    return (np.linalg.inv(T_MUNDO_BASE) @ np.append(p_mundo, 1.0))[:3]


def a_mundo(T_base):
    """Convierte una pose 4x4 de la base al mundo."""
    return T_MUNDO_BASE @ T_base
