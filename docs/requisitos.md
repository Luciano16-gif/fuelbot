# Requisitos del proyecto final (resumen de las directrices)

Resumen propio de las directrices oficiales del Proyecto Final de Robótica Industrial (FPTEN27, UNIMET, trimestre septiembre a diciembre 2026), cruzado con lo que hay en el repo. El texto completo está en `directrices.md` (transcripción) y `directrices-proyecto-final-fpten27.pdf` (original).

## Ruta elegida: A (simulación en CoppeliaSim)

- La escena debe estar construida o modificada sustancialmente por el equipo. Abrir una escena de demostración no cuenta.
- La forma de controlar la escena la elige el equipo y la justifica. Nosotros usamos la API remota ZeroMQ desde Python.
- Aprender CoppeliaSim es parte de lo que se evalúa ("manejo palpable de la herramienta": demostrar que se domina, no que se copió un ejemplo).

## Los cinco requisitos obligatorios (piso común)

| # | Requisito | Dónde está en el repo |
| --- | --- | --- |
| 1 | Modelo del robot: configuración, grados de libertad, tabla DH y cinemática directa (Tema 8) | `modelo_dh.py`, `verificar_modelo.py` |
| 2 | Un tema del segundo corte aplicado al funcionamiento del robot (Tema 9 trayectorias, 10 comunicación, 11 visión o 12 IA y planificación) | Tema 9: `trayectorias.py` (quíntico, PTP articular, lineal cartesiano) |
| 3 | Verificación: contrastar con números lo que predice el modelo frente a lo que reporta el simulador | Columnas `*_modelo` vs `*_sim` y `err_modelo_vs_sim_mm` de `resultados/pasos.csv` |
| 4 | Aplicación industrial con fuente fiable | Va en el documento, no en el código |
| 5 | Declaración de lo que no es propio, mostrando qué se modificó o extendió | Sección "Qué no es propio" del README |

Los objetivos (general y 3 o 4 específicos) tienen que ser medibles con una evidencia trazable. El objetivo general que verifica el código (`ciclo_repostaje.py`) es: error de inserción menor a 5 mm y error de orientación menor a 2 grados en cada ciclo, en una corrida de 10 ciclos.

## Cronograma

| Fecha | Corte | Qué se entrega |
| --- | --- | --- |
| Viernes 6 de noviembre | Entrega 0: anteproyecto (sin nota) | PDF de 2 a 3 páginas |
| Domingo 22 de noviembre | Entrega 1: avance (5%) | Video, archivos fuente y documento v1 |
| Semana 11 | Coordinación | Acordar con el profesor el horario de la defensa |
| 30 de noviembre al 4 de diciembre | Entrega final y defensa (15%) | Documento final, archivos fuente y defensa presencial |

### Entrega 1 (5%)

1. Video de 1 a 2 minutos con la simulación funcionando (resultados, no diapositivas).
2. Archivos fuente en un .zip, incluida la escena de CoppeliaSim (`.ttt`, ver el paso 4 de "Correr la simulación" en el README).
3. Documento v1 en PDF con la estructura completa del documento final. Introducción, marco teórico y modelo del robot ya redactados; implementación y resultados pueden ser preliminares.
4. Apartado de respuesta a la devolución del anteproyecto: qué cambió, qué falta y en qué fechas.

Evaluación: avance funcional 40%, documento v1 40%, respuesta a la devolución y plan 20%.

### Entrega final (15%)

Estructura del documento: portada, resumen, introducción y aplicación industrial, marco teórico, modelo del robot, implementación (con la declaración de lo que no es propio), resultados y verificación (contraste numérico y grado de cumplimiento de cada objetivo), conclusiones y limitaciones, contribución por integrante (obligatoria), referencias y anexos con el código.

Evaluación sobre 15 puntos:

| Criterio | Puntos | Alcance |
| --- | --- | --- |
| Preguntas individuales y pregunta cruzada | 5 | Individual |
| Funcionamiento y verificación numérica | 4 | Equipo |
| Documento final | 3 | Equipo |
| Manejo de la herramienta | 2 | Equipo |
| Aplicación industrial con fuente | 1 | Equipo |

En la defensa cada integrante responde sobre lo que hizo y también sobre el trabajo de otro (pregunta cruzada). Por eso conviene que todos entiendan todo el código, no solo su parte.
