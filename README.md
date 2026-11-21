# FuelBot: Brazo robótico de repostaje automático (Robótica Industrial FPTEN27, UNIMET)

Simulación en **CoppeliaSim** controlada desde **Python** (API remota ZeroMQ).
El robot es un brazo 6R con la tabla DH estándar del UR5; la escena (isla, surtidor,
carro, sensores y el propio brazo) se **construye desde código**, no desde una escena de demostración.

El robot está en el centro de la isla y atiende dos carriles. Hay cuatro tipos de vehículo (compacto,
sedán, SUV y pickup) con la boca de llenado a distinta altura (0,85 a 1,15 m) y del lado izquierdo o
derecho. En cada ciclo el robot lee el tipo (placa simulada), detecta con los sensores en qué carril
está el carro, estima la boca y decide la aproximación: semilla de la cinemática inversa del lado
que corresponde, o no repostar si la boca queda del lado contrario a la isla o fuera de alcance.

## Archivos

| Archivo | Descripción |
| --- | --- |
| `modelo_dh.py` | Tabla DH, cinemática directa, jacobiano y cinemática inversa (DLS) |
| `trayectorias.py` | Perfil quíntico, movimiento PTP articular y lineal cartesiano |
| `escenario.py` | Medidas de la estación, base de datos de los tipos de vehículo y sensores |
| `escena_coppelia.py` | Construye la escena en CoppeliaSim y lee/escribe articulaciones y sensores |
| `ciclo_repostaje.py` | **Programa principal**: máquina de estados del ciclo y registro de resultados |
| `verificar_modelo.py` | Pruebas del modelo sin simulador (FK, jacobiano, IK) |
| `espacio_trabajo.py` | Alcanzabilidad de la boca con la orientación requerida y manipulabilidad (cercanía a singularidades), sin simulador |
| `verificar_colisiones.py` | Holgura mínima del brazo con el carro y la estación en cada estado del ciclo (requiere CoppeliaSim) |
| `vista_previa.py` | Animación rápida en matplotlib, sin simulador |

## Instalación (Windows 11)

1. **CoppeliaSim Edu** (coppeliaedu gratis): https://www.coppeliarobotics.com/ → Downloads →
   *Edu* → Windows. Instalen la versión más reciente (4.6 o superior).
2. En una terminal (PowerShell), dentro de esta carpeta:

   ```
   python -m pip install -r requirements.txt
   ```

## Prueba rápida sin simulador

```
python verificar_modelo.py
python ciclo_repostaje.py --sin-coppelia --ciclos 10
```

## Correr la simulación

1. Abran CoppeliaSim. Dejen que la escena nueva por defecto (*File → New scene*). No le den a Play.
2. En la terminal:

   ```
   python ciclo_repostaje.py --ciclos 4 --tiempo-real
   ```

   El script construye la escena, arranquen la simulación en modo *stepping* y corran los ciclos.
   Al terminar imprime un resumen y guarda `resultados/ciclos.csv` y `resultados/pasos.csv`.
3. Para la corrida de verificación del objetivo general: `python ciclo_repostaje.py --ciclos 20`
   (5 ciclos por tipo de vehículo). Para medir la holgura del brazo con el carro y la estación en esos
   mismos ciclos: `python verificar_colisiones.py --ciclos 20` (guarda `resultados/colisiones.csv`).
4. Para guardar la escena construida: *File → Save scene as…* (útil para entregar el `.ttt`).

Opciones: `--tipo suv` (un solo tipo), `--carril-equivocado 3` (en el ciclo 3 el carro se estaciona con
la boca del lado contrario, para ver que el robot decide no repostar), `--litros 5`, `--caudal 0.6`,
`--ruido-mm 1.0` (ruido del sensor), `--semilla 7`, `--dt 0.05`.

## Análisis del espacio de trabajo

```
python espacio_trabajo.py
```

Tarda unos 3 minutos. Recorre una malla de posiciones de la boca en los dos carriles, resuelve la
cinemática inversa del vía point y del tramo de inserción con la boquilla alineada con el cuello, y
calcula el índice de manipulabilidad w = |det J| (normalizado con su máximo para el UR5). Guarda
`resultados/espacio_trabajo.csv`, `resultados/espacio_tipos.csv` y el mapa `resultados/espacio_trabajo.png`.

## Si algo falla

- **"Connection refused" / se queda colgado:** CoppeliaSim no está abierto o el servidor ZMQ remote API no
  arrancó. Normalmente arranca solo con CoppeliaSim; si no, seguro esta en el menú de módulos/add-ons como
  *ZMQ remote API server* y actívalo. Si Windows pregunta por el firewall, den el acceso.
- **`AttributeError` con una constante o función:** la versión de CoppeliaSim es vieja. Actualiza a 4.6+.
- **"AVISO: checkDistance falló":** el sensor está usando lecturas emuladas en lugar de las del simulador.
  El ciclo sigue funcionando; anótalo y lo arreglamos.

## Qué no es propio (declaración, requisito 5)

- Parámetros DH del UR5: publicados por Universal Robots.
- Cliente `coppeliasim-zmqremoteapi-client`: Coppelia Robotics.
- `verificar_modelo.py` usa Robotics Toolbox for Python (P. Corke) solo como referencia de comparación.
- Todo lo demás (escena, construcción del brazo, IK, trayectorias, lógica del ciclo, verificación) lo verificaremos nosotros.
