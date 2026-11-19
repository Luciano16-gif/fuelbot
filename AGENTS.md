# AGENTS.md

Guía para agentes de código (Claude, Codex, etc.) que trabajen en este repo. El README explica el proyecto para personas; esto es lo que un agente necesita saber además.

## Contexto

- Proyecto final de Robótica Industrial (FPTEN27, UNIMET), hecho en equipo por tres estudiantes.
- Brazo 6R con la tabla DH del UR5 que reposta un carro en una estación. Simulación en CoppeliaSim controlada desde Python por la API remota ZeroMQ.
- La escena completa (isla, surtidor, carro, sensores y el brazo) se construye desde código en `escena_coppelia.py`. No se usa ninguna escena `.ttt` de demostración, y eso es un requisito del proyecto: no lo cambies.
- Las directrices oficiales del curso están en `docs/directrices.md` (transcripción completa del PDF, que también está en `docs/`). `docs/requisitos.md` las resume y dice qué archivo del repo cumple cada requisito. Léelos antes de cualquier tarea que toque el alcance, la verificación o la entrega. Si algo no aparece ahí, pregunta antes de suponer.

## Estructura

- `modelo_dh.py`: tabla DH, cinemática directa, jacobiano e IK por mínimos cuadrados amortiguados (DLS). Es el núcleo; cualquier cambio aquí se verifica con `verificar_modelo.py`.
- `trayectorias.py`: perfil quíntico, PTP articular y lineal cartesiano.
- `escenario.py`: medidas de la estación, el carro y los sensores (constantes).
- `escena_coppelia.py`: todo lo que habla con CoppeliaSim.
- `ciclo_repostaje.py`: programa principal y máquina de estados `LLEGADA -> DETECCION -> APROXIMACION -> INSERCION -> SURTIDO -> EXTRACCION -> REPOSO -> SALIDA`. Incluye `EscenaSinSimulador`, que imita la interfaz de la escena real para correr sin CoppeliaSim.
- `vista_previa.py`: animación en matplotlib, sin simulador.
- `resultados/`: CSV que genera `ciclo_repostaje.py`.

## Cómo verificar cambios

No hay CoppeliaSim disponible para los agentes, así que verifica con estas dos pruebas, que no lo necesitan:

```
python -m pip install -r requirements.txt
python verificar_modelo.py
python ciclo_repostaje.py --sin-coppelia --ciclos 10
```

Resultado esperado: IK 200/200 convergen, diferencia del jacobiano del orden de 1e-9 y "Ciclos que cumplen: 10/10". Si un cambio toca `escena_coppelia.py`, dilo en el resumen: solo se puede probar de verdad con CoppeliaSim abierto, y eso lo hace una persona del equipo.

`ciclo_repostaje.py` sobrescribe `resultados/ciclos.csv` y `resultados/pasos.csv`. No hagas commit de esos CSV cuando vengan de una corrida `--sin-coppelia`; restáuralos con `git checkout -- resultados` después de probar. Solo se actualizan cuando el equipo hace una corrida real en el simulador.

## Convenciones

- Todo en español: código, comentarios, nombres de variables, mensajes de commit y documentación.
- Nada de em dashes (el guion largo, U+2014) en ningún texto. Usa dos puntos, comas o puntos.
- Python 3.12, dependencias en `requirements.txt`. No agregues dependencias nuevas sin que el equipo lo pida.
- Si usas código o parámetros de otra fuente, agrégalo a la sección "Qué no es propio" del README (es un requisito de entrega).

## Git

- Somos tres trabajando en el mismo repo: haz `git pull` antes de empezar y trabaja en una rama por tarea (`git switch -c nombre-de-la-tarea`).
- No hagas push directo a `main` ni force push salvo que te lo pidan.
