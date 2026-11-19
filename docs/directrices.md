# Directrices del Proyecto Final

Transcripción completa de `docs/directrices-proyecto-final-fpten27.pdf` (7 páginas). El contenido es el del PDF original; solo se cambió el formato a Markdown y los guiones largos por dos puntos.

**Robótica Industrial · Trimestre Septiembre a Diciembre 2026**

| | |
| --- | --- |
| **Asignatura** | Robótica Industrial (FPTEN27) |
| **Departamento** | Energía y Automatización · Escuela de Ingeniería |
| **Profesores** | José Montilva · Rafael Matienzo |
| **Período** | Trimestre Septiembre a Diciembre 2026 |
| **Peso en la nota** | 20% de la calificación final |

## 1. Propósito y peso

El Proyecto Final vale **20% de la calificación final** y se desarrolla en equipo, en tres cortes: un anteproyecto sin nota, un avance (5%) y una entrega final con defensa (15%).

El proyecto es abierto. Cada equipo elige una de tres rutas y define qué construir dentro de ella. En cualquier ruta se evalúan los mismos aspectos:

- **Rigurosidad académica:** cada afirmación del proyecto está respaldada por un modelo, una medición o una fuente.
- **Manejo palpable de la herramienta:** el equipo demuestra que domina el software o el hardware que usa, no que copió un ejemplo.
- **Comprensión del trasfondo teórico:** cada integrante puede explicar por qué su robot se comporta como se comporta.
- **Aplicación industrial:** el equipo identifica dónde se usa algo equivalente en la industria y qué problema resuelve.

| Corte | Peso sobre la calificación final |
| --- | --- |
| Entrega 0: Anteproyecto | 0% |
| Entrega 1: Avance | 5% |
| Entrega final: Documento y defensa | 15% |
| **Total** | **20%** |

## 2. Las tres rutas

Cada equipo elige **una** ruta y la declara en el anteproyecto. Dentro de ella, qué construir y cómo es decisión del equipo; lo único fijo es lo que indica cada ruta y el piso común de la sección 3.

### Ruta A: Simulación en CoppeliaSim

No hay una tarea asignada. El equipo investiga la herramienta, descubre qué permite hacer y propone su propia aplicación.

- La escena debe ser construida o modificada sustancialmente por el equipo. Abrir una escena de demostración y ejecutarla no cuenta como proyecto.
- La forma de controlar la escena (scripts internos, API remota desde Python u otra) la elige el equipo y la justifica en el anteproyecto.
- CoppeliaSim no se enseña en clase ni en los laboratorios. Aprender la herramienta es parte de lo que el proyecto evalúa.

### Ruta B: MATLAB / Simulink

El proyecto debe usar **al menos un toolbox especializado de robótica** de MathWorks, en MATLAB, en Simulink o en ambos. Se pueden combinar varios toolboxes.

- La elección del toolbox y de qué hacer con él es libre, pero se justifica en el anteproyecto.
- La licencia de la Universidad cubre todos los toolboxes, por lo que no hay restricción por disponibilidad.
- La sección 8 indica por dónde empezar a explorarlos.

### Ruta C: Aplicación robótica física

El equipo construye un robot real: por ejemplo, un siguelíneas, un brazo robótico o cualquier otra aplicación que proponga.

- **Todos los materiales corren por cuenta del equipo.** El laboratorio no presta componentes para el proyecto.
- El robot se muestra **en vivo y en persona** en la defensa final. En la Entrega 1 el avance se muestra en video, igual que en las otras rutas.

## 3. Piso común: requisitos de todo proyecto

Sea cual sea la ruta, todo proyecto cumple los cinco requisitos siguientes. Son el patrón con el que se corrige el anteproyecto y se evalúan los cortes con nota.

1. **Modelo del robot.** Identificar la configuración y los grados de libertad, y presentar un modelo cinemático acorde al tipo de robot. En un manipulador, la tabla de Denavit-Hartenberg y la cinemática directa (Tema 8). En un robot móvil, el modelo cinemático que le corresponda, citando su fuente.
2. **Un tema adicional del segundo corte, aplicado.** Al menos uno entre generación de trayectorias (Tema 9), lenguajes y arquitectura de comunicación (Tema 10), visión artificial (Tema 11) o inteligencia artificial y planificación (Tema 12). Debe formar parte del funcionamiento del robot, no solo mencionarse en el documento.
3. **Verificación.** Contrastar con números lo que predice la teoría con lo que muestra la simulación o la medición. Por ejemplo, comparar la posición del efector calculada con el modelo frente a la que reporta el software o la que se mide con una regla.
4. **Aplicación industrial con fuente.** Explicar qué problema real representa el proyecto y dónde se usa algo equivalente en la industria, citando una fuente fiable.
5. **Declaración de lo que no es propio.** Si el proyecto parte de un ejemplo oficial, una escena de demostración, un tutorial o código de terceros, el equipo lo declara y muestra con precisión qué modificó o extendió.

### Los objetivos deben ser medibles

El objetivo general y los específicos deben poder verificarse con una medición o una evidencia trazable. Un objetivo que suena bien pero no se puede comprobar no se acepta.

| No se acepta | Se acepta |
| --- | --- |
| Diseñar un brazo robótico eficiente para aplicaciones industriales. | Que el brazo traslade 10 piezas a su posición destino con un error de posición menor a 5 mm en al menos 9 de los 10 intentos. |
| Desarrollar un robot móvil capaz de seguir una línea. | Que el robot recorra un circuito cerrado de 3 m con dos curvas de 90° sin salirse de la línea, en menos de 20 s, en 5 intentos consecutivos. |

## 4. Cronograma

Las entregas escritas se suben a Aulas Virtuales. La defensa es presencial, fuera del horario de clase, en un horario acordado con el profesor.

| Semana | Fecha | Corte | Qué ocurre |
| --- | --- | --- | --- |
| 8 | Viernes 6 de noviembre | Entrega 0: Anteproyecto | El equipo entrega su propuesta. |
| 8 | Fin de semana 7 y 8 de noviembre | Devolución del profesor | Cada equipo recibe su veredicto con recomendaciones y arranca la semana 9 con el alcance definido. |
| 10 | Domingo 22 de noviembre | Entrega 1: Avance (5%) | Video, archivos fuente y primera versión del documento. |
| 11 | Durante la semana | Coordinación de la defensa | Cada equipo acuerda con el profesor su horario de defensa. |
| 12 | Del 30 de noviembre al 4 de diciembre | Entrega final y defensa (15%) | Documento final, archivos fuente y defensa presencial. |

## 5. Entrega 0: Anteproyecto (sin nota)

El anteproyecto no tiene nota. Sirve para que el profesor revise la propuesta y ajuste su dificultad, de modo que todos los proyectos queden en un nivel equilibrado.

**Formato:** un PDF de 2 a 3 páginas, uno por equipo, subido a Aulas Virtuales.

**Contenido:**

1. Integrantes del equipo y ruta elegida (A, B o C).
2. Título y descripción del proyecto en un párrafo.
3. Aplicación industrial que lo motiva, con al menos una fuente.
4. Objetivo general y 3 o 4 objetivos específicos, todos medibles (ver sección 3).
5. Cómo cumplirá cada uno de los cinco requisitos del piso común: qué modelo usará, qué tema adicional aplicará y cómo verificará los resultados.
6. Herramientas: software y versión, toolbox o toolboxes elegidos, o lista de materiales si es la ruta física.
7. Alcance, separado en dos partes:
   1. **Mínimo funcional:** lo que el equipo se compromete a entregar.
   2. **Extensiones:** lo que agregaría si le sobra tiempo.
8. Riesgos principales: qué puede fallar y qué hará el equipo si falla.
9. Plan de trabajo semana a semana hasta la semana 12.
10. Referencias iniciales.

### Devolución del profesor

Cada equipo recibe uno de tres veredictos, acompañado de recomendaciones concretas sobre qué hacer:

| Veredicto | Significado |
| --- | --- |
| **Aprobado** | El alcance es adecuado; el equipo continúa con su plan. |
| **Subir alcance** | El proyecto queda por debajo de lo esperado; se indica qué agregar al mínimo funcional. |
| **Bajar alcance** | El proyecto excede lo que se puede lograr en el tiempo disponible; se indica qué pasar a extensiones o eliminar. |

> **Si un equipo no entrega:** puede continuar con el proyecto, pero su propuesta no se revisa. El equipo asume el riesgo de descubrir en la Entrega 1 que su proyecto está muy por encima o muy por debajo de lo pedido.

## 6. Entrega 1: Avance (5%)

La Entrega 1 demuestra un avance tangible y una primera versión del documento final. Vale 5% de la calificación y es la misma para las tres rutas.

**Qué se entrega en Aulas Virtuales:**

1. **Video de 1 a 2 minutos** mostrando lo que ya funciona. En la ruta física se graba el robot real; en las rutas A y B, la pantalla con la simulación o el modelo en ejecución. El video muestra resultados, no diapositivas.
2. **Archivos fuente** en un .zip: la escena de CoppeliaSim, los archivos .m o .slx, o el código del robot físico, según la ruta.
3. **Documento v1** en PDF, con la estructura completa del documento final (sección 7). La introducción, el marco teórico y el modelo del robot deben estar redactados; la implementación y los resultados pueden ser preliminares.
4. **Respuesta a la devolución:** un apartado breve dentro del documento con qué cambió desde el anteproyecto, qué falta y en qué fechas se hará.

### Criterios de evaluación

| Criterio | Peso | Qué se espera |
| --- | --- | --- |
| Avance funcional | 40% | El video muestra una parte del mínimo funcional operando, y los archivos fuente corresponden a lo que se ve. |
| Documento v1 | 40% | El marco teórico y el modelo del robot están completos, son correctos y citan fuentes fiables. |
| Respuesta a la devolución y plan | 20% | El equipo atendió las recomendaciones del anteproyecto y su plan hasta la semana 12 es realista. |

## 7. Entrega final y defensa (15%)

La entrega final consiste en el documento final, los archivos fuente y una defensa presencial. Vale 15% de la calificación: **10 puntos son comunes al equipo y 5 puntos son individuales**, según las preguntas individuales y la pregunta cruzada.

### Documento final

PDF subido a Aulas Virtuales antes de la defensa, con la siguiente estructura:

1. Portada.
2. Resumen.
3. Introducción y aplicación industrial.
4. Marco teórico.
5. Modelo del robot.
6. Implementación: qué se construyó y cómo, incluida la declaración de lo que no es propio.
7. Resultados y verificación: contraste numérico entre teoría y simulación o medición, y grado de cumplimiento de cada objetivo.
8. Conclusiones y limitaciones.
9. **Contribución por integrante:** qué hizo concretamente cada miembro del equipo.
10. Referencias.
11. Anexos con el código.

> **Importante:** la sección de contribución por integrante es obligatoria. Orienta las preguntas individuales de la defensa.

### Defensa

- Presencial, fuera del horario de clase, en el horario acordado con el profesor durante la semana 11.
- El equipo expone el proyecto y lo demuestra funcionando. **En la ruta física, el robot se muestra en vivo.**
- Luego vienen las preguntas, en tres momentos:
  1. **Preguntas generales al equipo** sobre el proyecto en conjunto.
  2. **Preguntas individuales** a cada integrante, orientadas por la sección de contribución.
  3. **Pregunta cruzada:** cada integrante responde sobre el trabajo de otro. Mide la cohesión del equipo y qué tanto conoce cada uno lo que hicieron los demás.
- Las preguntas individuales y la cruzada determinan la parte individual de la nota. Por eso la nota final del proyecto puede variar de un integrante a otro dentro del mismo equipo.

### Criterios de evaluación

| Criterio | Puntos (sobre 15) | Alcance | Qué se espera |
| --- | --- | --- | --- |
| Preguntas individuales y pregunta cruzada | 5 | Individual | El integrante explica con precisión el modelo, las decisiones de diseño y los resultados de lo que declaró haber hecho, y demuestra conocer el trabajo de sus compañeros. |
| Funcionamiento y verificación | 4 | Equipo | Se cumple el mínimo funcional y se verifica con números que el robot hace lo que el modelo predice. |
| Documento final | 3 | Equipo | Completo, riguroso, con objetivos medibles evaluados y fuentes fiables. |
| Manejo de la herramienta | 2 | Equipo | Uso del software o hardware que va más allá de ejecutar un ejemplo, con las modificaciones propias bien identificadas. |
| Aplicación industrial | 1 | Equipo | Conexión justificada y con fuente entre el proyecto y su equivalente industrial. |
| **Total** | **15** | | |

## 8. Recursos de partida

Estos enlaces son un punto de entrada, no una lista cerrada. Cada equipo amplía su investigación según su ruta.

### Ruta A: CoppeliaSim (manual oficial de Coppelia Robotics)

- [Tutorial de controladores externos](https://manual.coppeliarobotics.com/en/externalControllerTutorial.htm): resume las distintas formas de controlar un robot en CoppeliaSim, desde scripts internos hasta aplicaciones externas.
- [Visión general de la API remota](https://manual.coppeliarobotics.com/en/remoteApiOverview.htm): cómo comunicar CoppeliaSim con una aplicación externa.
- [API remota ZeroMQ](https://manual.coppeliarobotics.com/en/zmqRemoteApiOverview.htm): incluye ejemplos mínimos de cliente en Python y en MATLAB/Octave.

### Ruta B: MATLAB / Simulink

- [Robotics and Autonomous Systems (MathWorks)](https://www.mathworks.com/help/overview/robotics-and-autonomous-systems.html): reúne los productos de robótica, entre ellos Robotics System Toolbox y ROS Toolbox, con ejemplos que van desde la planificación de trayectorias de manipuladores en Simulink hasta el seguimiento de rutas con evasión de obstáculos (Navigation Toolbox).

### Todas las rutas: bibliografía del curso

- Lynch, K. M. y Park, F. C. *Modern Robotics: Mechanics, Planning, and Control*. Cambridge University Press. Para robots móviles con ruedas, la sección 13.3 desarrolla los modelos cinemáticos de tracción diferencial y de tipo carro.
- Correll, N. et al. *Introduction to Autonomous Robots*. La sección 3.2 presenta la cinemática directa de brazos simples, robots de ruedas diferenciales y vehículos con dirección tipo carro.
- St-Onge, D. y Herath, D. *Foundations of Robotics: A Multidisciplinary Approach with Python and ROS*. Springer.
- Angeles, J. *Fundamentals of Robotic Mechanical Systems*. Springer.
