# SdS TP5 — Dinámica peatonal

Problema 2: evacuación con esquemas por pasos, por eventos e híbrido.

Modelo elegido: CPM. Se asumen parámetros de entrada válidos, sin validaciones defensivas adicionales.

## Base actual

- `Particle`: identificador, posición, velocidad, target, radio y esquema de actualización.
- `Board`: habitación cuadrada y cinco segmentos de pared que dejan una puerta centrada en el borde inferior (`y = 0`). Provee el target de la puerta y uno exterior a 1 m.
- `Vector2D`: operaciones vectoriales básicas, reutilizadas de TP4.
- `StateWriter`: escritura de estados en texto, adaptada de TP4. El motor decide cuándo escribir; el archivo se cierra automáticamente al destruir el writer.
- `Simulation`: contiene el tablero, las partículas y el tiempo. Inicializa 500 agentes por defecto, con posiciones aleatorias sin superposiciones, radio de 0,25 m y velocidad de 1,5 m/s hacia la puerta. El tiempo inicial es cero y el esquema inicial es time driven.
- `Grid`: Cell Index Method para seleccionar pares candidatos a contacto durante cada paso.

Se pueden indicar cantidad de partículas, semilla y tablero en el constructor, por ejemplo `simulation::Simulation simulation(500, 42, types::Board(50.0, 0.8));`. La semilla por defecto es 42; usar la misma semilla y parámetros permite repetir el estado inicial para comparar esquemas. Las posiciones se sortean entre `rmax` y `L - rmax` en ambos ejes, rechazando las que se superponen con una partícula existente. No hay límite de intentos: se asume una cantidad que cabe holgadamente en la habitación, como en los casos del TP.

La elección de radio máximo y rapidez deseada inicial representa movimiento libre ya recuperado; el enunciado no fija un radio inicial para CPM.

## Paso de CPM

`Simulation::step()` avanza TSDMD puro con `dt = 0,10 / (2 * 1,5) = 1/30 s`. No recibe un dt arbitrario.

1. Detecta todos los contactos usando las posiciones y radios anteriores. Suma las direcciones unitarias de separación con cada agente y con el punto más cercano de cada pared, incluidos los extremos de la puerta.
2. Ante contacto, fija el radio en 0,10 m y la velocidad de escape en la dirección de esa suma, con módulo de 1,5 m/s.
3. Sin contacto, aumenta el radio en `rmax * dt / tau`, hasta 0,25 m. La rapidez hacia el target es `1,5 * ((r - rmin) / (rmax - rmin))^beta`.
4. Una vez calculadas todas las velocidades, mueve las partículas y avanza el tiempo.

Se usan `tau = 0,5 s`, `beta = 0,9` y velocidad de escape igual a la rapidez deseada, según [Baglietto y Parisi (2011), sección II y parámetros de sección III](https://ri.conicet.gov.ar/bitstream/handle/11336/192293/CONICET_Digital_Nro.72ac9ef0-c141-498b-b436-d97f55dfd157_B.pdf?sequence=2). La detección de contacto se realiza antes de contraer los radios; no se reevalúan contactos a mitad del paso. Radios y velocidades pueden actualizarse en una misma pasada porque dependen del contacto ya calculado y del estado de cada partícula.

Cuando una dirección tiene norma cero, `normalized()` devuelve el vector cero. En particular, si las direcciones de escape se cancelan exactamente, el agente se contrae y queda detenido ese paso. Es una convención para el caso en que la dirección normalizada del paper no está definida.

Al cruzar `y = 0` por la abertura de la puerta, se cambia el target al punto exterior. La coordenada horizontal de cruce se calcula sobre el segmento recorrido en el paso. Los agentes permanecen activos fuera de la habitación hasta alcanzar `y <= -1 m`, cuando se retiran del vector.

`getPopulation()` cuenta agentes con centro en `y > 0`. `getEvacuationTime()` guarda el tiempo del último cruce de salida, con resolución `dt`; es distinto del tiempo final, que incluye el recorrido hasta el retiro. Si un agente vuelve a entrar, vuelve a contar en la población interior.

## Búsqueda de vecinos (CIM)

Antes de detectar contactos se reconstruye una grilla con los índices actuales de las partículas. El lado de celda es al menos `2 * rmax` (0,5 m); se ajusta para dividir el lado del tablero en un número entero de celdas. Así, dos agentes en contacto deben estar en la misma celda o en celdas contiguas.

`Grid::getCandidates()` reúne las partículas de esas nueve celdas. `findContacts()` revisa solo los índices `j > i` para comprobar cada par una vez y confirma el contacto mediante la distancia y los radios actuales. Las cinco paredes se siguen evaluando directamente.

Los agentes que están fuera de la habitación se asignan a las celdas del borde hasta su retiro. Esto puede agregar candidatos, pero conserva los contactos. La grilla no es periódica. La inicialización aleatoria mantiene su búsqueda simple de superposiciones; CIM se usa en los pasos de evolución.

El costo depende de la cantidad de celdas y candidatos; si todas las partículas se concentran en una misma celda puede seguir siendo cuadrático. En una distribución inicial de 500 agentes con semilla 0 se verificaron 83 pares candidatos frente a 124.750 pares posibles; esto no es una medición de aceleración total.

Las partículas retiradas dejan de formar parte del vector que se escribe. Cada registro conserva su identificador. La cantidad del encabezado es la cantidad de partículas presentes en ese vector, no necesariamente la población dentro de la habitación: los agentes que ya cruzaron la puerta pueden seguir activos hasta alcanzar la distancia de retiro.

Formato de cada estado (precedido por dos líneas de comentarios al abrir el archivo):

```text
time particle_count
id x y vx vy radius scheme
...

```

`scheme`: `0` para time driven y `1` para event driven. El constructor de `Particle` deja velocidad y target en cero; el motor debe asignarlos al inicializar. El esquema inicial es time driven.

## Compilación

Se requiere C++17 y CMake:

```sh
cmake -S . -B build
cmake --build build --config Release
```

Se construyen la biblioteca `tp5_core` y el ejecutable `tp5`. En Windows con un generador de varias configuraciones, el ejecutable queda en `build/Release/tp5.exe`; con un generador de una sola configuración, queda directamente en `build` (usar `-DCMAKE_BUILD_TYPE=Release` al configurar).

## Ejecución

Ejecutar desde la raíz del proyecto para que `output/` quede fuera de `build`. Todas las salidas, incluidas las de pruebas, deben guardarse fuera de `build`. Si se ejecuta desde otra carpeta, indicar una ruta de salida que respete esa separación.

```sh
./build/tp5 500 42 50 output
```

Los argumentos opcionales son cantidad de agentes, semilla, lado de habitación, carpeta de salida y esquema (`tsdmd` o `edmd`), en ese orden. Sus valores por defecto son `500 42 50 output tsdmd`. La puerta mide 0,8 m. En Windows, ajustar la ruta del ejecutable según el generador usado.

Con `edmd` se aceptan dos argumentos más: la búsqueda de vecinos (`grid`, por defecto, o `allpairs`) y el intervalo de reconstrucción de la grilla en segundos (por defecto 0,05). Con `hybrid` se aceptan dos argumentos más: la anticipación k, en pasos (por defecto 1), y la distancia de cierre d* en metros (por defecto y mínimo 0,6 m: con menos, un agente ED podría tocar a uno TS sin detectarse). Entra a TS todo agente a d* o menos de un agente TS.

```sh
./build/tp5 500 42 50 output_hybrid hybrid 1
```

```sh
./build/tp5 500 42 50 output_edmd edmd
```

El programa ejecuta el esquema elegido hasta retirar a todos los agentes y genera:

- `states.txt`: estado inicial, estados cada tres pasos (`dt2 = 0,1 s`) y estado final vacío.
- `population.txt`: una fila por paso, incluido el inicial, con columnas `time inside active timeDriven` (la última es la cantidad de agentes que avanzaron en TS en ese paso).

La carpeta se crea si no existe. Cada ejecución reemplaza esos archivos; usar carpetas diferentes para conservar realizaciones. En consola se imprimen el tiempo de evacuación y el tiempo del último retiro.

Con `edmd` también se imprimen la cantidad de eventos procesados y de eventos obsoletos descartados. Con `hybrid` se imprimen las entradas a TS por predicción y por cierre de la bolsa, y la cantidad de predicciones de pares.

## Esquema por eventos (EDMD)

`SimulationEDMD` tiene la misma interfaz pública que `Simulation`. Los parámetros CPM de los dos motores están en `simulation/CpmParameters.h`. Las trayectorias y la predicción de eventos están en `simulation/EventPrediction`.

Interpretación del CPM en tiempo continuo (alternativa B):

- Cada agente está en una de tres fases, todas con fórmula cerrada: libre (radio `rmax`, rapidez `vd` hacia el target), escape (radio `rmin`, rapidez `ve`) y recuperación (`r(t) = rmin + (rmax / tau) t`, rapidez `vd ((r - rmin) / (rmax - rmin))^beta`, cuya integral es exacta). La recuperación completa dura 0,3 s.
- El contacto se resuelve en el instante en que dos discos (o un disco y una pared) se tocan. Ambos se contraen a `rmin` y escapan durante `dt = 1/30 s` en la dirección de la suma de direcciones de separación, como en el TSDMD. **La duración del escape es una decisión propia**, porque en tiempo continuo el contacto desaparece al contraerse; se eligió `dt` porque es lo que dura el escape en el TSDMD.
- Al terminar el escape se reevalúan los contactos: si sigue en contacto, escapa de nuevo; si no, empieza a recuperarse.
- Los contactos simultáneos se resuelven a partir de una foto común: primero se calculan todos los conjuntos de contacto y recién después se contrae a los agentes, así el resultado no depende del orden de almacenamiento.
- Eventos: contacto agente-agente, contacto con pared (cápsula: banda más extremos del segmento), fin de escape, fin de recuperación, cruce de la puerta (`y = 0` dentro de la abertura) y retiro (`y = -1`).

Predicción: con radio fijo y movimiento rectilíneo, el contacto sale de una cuadrática con raíz estable. Durante la recuperación el movimiento no es rectilíneo y se usa avance conservador con una cota de velocidad de cierre, más bisección. La penetración máxima que puede pasar inadvertida es 1 µm.

Agenda: heap de eventos con versiones. Cada evento guarda la versión de sus agentes; si cambió antes de procesarlo, se descarta. Después de cada grupo solo se reprograman los agentes cuyo tramo cambió, y ningún paso del procesamiento de un grupo recorre a los N agentes. Los agentes no se borran del vector interno; `getParticles()` devuelve los activos calculados en el instante de muestreo.

Vecinos (`NeighbourSearch::CellGrid`, por defecto): la grilla del CIM se reconstruye cada Δ = 0,05 s con las posiciones de ese instante, y ningún choque entre agentes se predice más allá de la próxima reconstrucción. Como el hueco entre dos agentes se cierra a lo sumo a 2 ve + 2 rmax / tau = 4 m/s, quien pueda tocar a un agente antes de la próxima reconstrucción está a menos de `2 rmax + 4 Δ` en la reconstrucción. Esos son sus candidatos: celdas de ese lado y descarte de los que están más lejos. Al reconstruir no hay que invalidar nada, porque ningún choque de la ventana anterior pasa del instante de reconstrucción. Las paredes no usan la grilla. `NeighbourSearch::AllPairs` compara contra todos y queda como referencia.

Con N = 500 y L = 50 m, la grilla es unas 7 veces más rápida que `AllPairs`, y el resultado es el mismo salvo por la divergencia caótica. Valores de Δ entre 0,02 y 0,05 s son los más rápidos; con Δ mayor hay más candidatos. Aun así, el EDMD es entre 13 y 18 veces más lento que el TSDMD: casi todos los eventos ocurren en la aglomeración de la puerta, donde cada agente tiene decenas de candidatos.

Un detector aborta la corrida con un error si se procesan 100 000 grupos sin que el tiempo avance 1 ms (cascada de eventos), en lugar de agotar la memoria.

`step()` no es un paso de CPM: procesa todos los eventos hasta `getTime() + dt` y calcula las posiciones en ese instante, para que la salida tenga los mismos instantes que el TSDMD.

### Verificación

Los programas de `tests/` no forman parte de la entrega. Se compilan con `-DTP5_BUILD_VERIFICATION=ON`:

- `test_edmd_prototype`: casos analíticos, comparación entre predictores, escenarios pequeños y comparación de la agenda contra una versión de fuerza bruta.
- `verify_edmd`: estado inicial idéntico al del TSDMD, coincidencia con el prototipo y comparación estadística TSDMD vs EDMD.
- `overlap_diagnostic`: solapamientos y caudal en la puerta para cada esquema.
- `benchmark_edmd check|stats|interval|scaling`: la grilla no pierde choques (sin solapamientos en ninguna muestra), grilla contra `AllPairs` en 10 semillas, barrido de Δ y escalamiento con N a densidad constante.

La evacuación es caótica: una diferencia de 10⁻¹² s en un instante de contacto crece hasta cambiar la secuencia de eventos. Las comparaciones entre esquemas deben ser estadísticas.

## Esquema híbrido (ED + TS)

`SimulationHybrid` tiene la misma interfaz pública que los otros motores. Hay una sola bolsa de agentes TS y un reloj común, el `dt` del CPM. Cambiar de esquema no contrae, mueve ni reorienta a nadie: solo cambia cómo se calcula. Por eso la física del híbrido es la del TSDMD para cualquier anticipación k.

En cada paso la bolsa se reconstruye desde el estado actual, sin arrastrar la del paso anterior:

1. Entran a TS los agentes que no están recuperados (`r < rmax`) o están en contacto.
2. Entran los agentes ED con un choque previsto (con otro ED o con una pared) dentro de `k dt`. Esos choques se predicen con los mismos predictores del EDMD y se guardan en un heap con versiones.
3. Cierre de la bolsa: entra todo agente a 0,60 m o menos de un agente TS (`2 rmax + 2 v dt`). Más lejos no pueden tocarse durante el próximo paso. Se repite hasta que no entra nadie.
4. Los TS que podrían salir (recuperados y sin contacto) predicen su vuelo. Si chocarían dentro de `k dt`, se quedan; si no, pasan a ED y sus predicciones se agendan.

Los agentes TS siguen las reglas del TSDMD con el mismo código (`simulation/CpmRules.h`). Los contactos se buscan solo entre agentes TS. Los agentes ED vuelan en línea recta a `vd` hacia su target. La puerta y el retiro se comprueban igual para los dos, sobre el segmento recorrido en el paso, y el cambio de target ocurre al final de ese paso, como en el TSDMD.

Con todos los agentes forzados a TS, el híbrido es idéntico byte a byte al TSDMD. Con N = 500 y L = 50 m, cuesta unas 1,35 veces lo que el TSDMD y es unas 12 veces más rápido que el EDMD. La fracción de agentes TS sube de 0 % al inicio a casi 100 % desde los 30 s, cuando se forma la cola en la puerta; la mayor parte de la evacuación es esa cola. La anticipación k casi no cambia el costo (de 1,14 s con k = 0 a 1,34 s con k = 32) ni la evacuación.

La rama de salto largo con la bolsa vacía no está implementada: la salida se escribe en cada `dt` y la bolsa está vacía solo en los primeros instantes.

`verify_hybrid check|stats|cost|scaling` (en `tests/`) verifica la equivalencia con todos en TS, la ausencia de contactos con agentes ED para k entre 0 y 8, la coincidencia tick a tick con el TSDMD hasta la divergencia caótica, la comparación estadística en 10 semillas y el costo según k y según N.

## Estudios, gráficos y animaciones

Tres scripts en `scripts/` (requieren `numpy`, `matplotlib` y, para MP4, FFmpeg). Se corren desde la raíz del proyecto, con `build/tp5` compilado en Release.

**`correr.py`** corre los estudios con los tres esquemas y guarda cada corrida en `resultados/<estudio>/<configuración>/s<semilla>/` (`population.txt`, `salida.txt`, `resumen.json`). Las corridas ya hechas no se repiten, así que se puede cortar y retomar.

```sh
python3 scripts/correr.py base          # N=500, L=50, 10 semillas de cada esquema (híbrido con k=1)
python3 scripts/correr.py barrido       # híbrido con k = 0 1 2 4 8 16 32
python3 scripts/correr.py umbral        # híbrido con d* = 0,6 1 2 4 8 m, para N = 10 100 500 a L = 50 m
python3 scripts/correr.py densidad      # N = 100 ... 2000 a densidad 0,2 /m²
python3 scripts/correr.py lado          # N = 5 ... 1500 a L = 50 m
python3 scripts/correr.py animaciones   # una corrida por esquema con estados, video y fotogramas
python3 scripts/correr.py todo
```

Opciones útiles: `--realizaciones`, `--esquemas`, `--valores` (k), `--n`, `--n-lado`, `--velocidad`, `--fotogramas`, `--jobs` y `--rehacer`. Las corridas en lote no escriben `states.txt`. El tiempo de cómputo es el que mide `tp5` sobre `step()` (`Engine time`), sin escritura. Para los tiempos finales conviene `--jobs 1`.

**`graficos.py`** lee `resultados/` y escribe en `figuras/` las figuras (PNG) y las tablas (CSV) de cada estudio: población en el tiempo con cada realización identificada, comparación de esquemas con la misma semilla, fracción TS del híbrido, tiempos de evacuación y de cómputo (media y desvío estándar), barrido de k con TSDMD y EDMD de referencia, y escalamientos con su costo relativo al TSDMD.

```sh
python3 scripts/graficos.py               # todos los estudios con resultados
python3 scripts/graficos.py base barrido  # solo esos
```

**`animar.py`** anima un `states.txt`: a la izquierda la habitación completa y a la derecha un zoom a la puerta. En el híbrido colorea por esquema (rojo TS, azul ED); en TSDMD y EDMD, por radio. También guarda fotogramas en PNG para el informe.

```sh
TP5_STATES_EVERY=1 ./build/tp5 500 42 50 output hybrid
python3 scripts/animar.py output/states.txt --esquema hybrid --velocidad 5 --guardar output/hibrido.mp4
python3 scripts/animar.py output/states.txt --esquema hybrid --fotogramas 10 30 100 --carpeta-fotogramas output/fotos
```

`TP5_STATES_EVERY` fija cada cuántos pasos de `dt` se escribe `states.txt` (por defecto 1; 3 equivale a `dt2 = 0,1 s`; 0 no lo escribe). `scripts/visualizar.py` sigue disponible.

## Animación

Instalar las dependencias de Python:

```sh
python -m pip install -r scripts/requirements.txt
```

El visor lee `states.txt` sin ejecutar la simulación. Muestra la habitación completa y un zoom en la puerta, con radios reales en metros. Time driven se muestra en rojo y event driven en azul. El título indica tiempo, población interior y agentes activos.

```sh
python scripts/visualizar.py output/states.txt
python scripts/visualizar.py output/states.txt --every 3 --fps 30
python scripts/visualizar.py output/states.txt --every 30 --save output/animacion.gif
python scripts/visualizar.py output/states.txt --save output/animacion.mp4 --fps 30
python scripts/visualizar.py output/states.txt --png output/cuadro.png --frame 1000
```

`--every` selecciona uno de cada N estados; el estado final vacío se incluye siempre. `--fps` determina la reproducción. Con los estados actuales separados por 0,1 s, `--every 1 --fps 10` corresponde a tiempo real; `--every 30 --fps 10` acelera 30 veces. El índice de `--frame` se aplica después de seleccionar los estados (`-1` es el último).

Para otro tamaño de habitación o puerta, indicar `--side` y `--door`; los valores por defecto son 50 m y 0,8 m. Guardar las imágenes y videos en `output/`, siempre fuera de `build`. GIF y PNG funcionan con las dependencias indicadas; MP4 requiere además FFmpeg instalado y disponible en PATH. Para GIF largos conviene usar `--every`, porque el escritor conserva los cuadros en memoria.

En Ubuntu/WSL, instalar FFmpeg dentro de WSL (aunque ya esté instalado en Windows):

```sh
sudo apt update
sudo apt install ffmpeg
```

Luego repetir la exportación MP4. Si no se desea instalarlo, exportar a `.gif`. El script comprueba la disponibilidad de FFmpeg antes de cargar los estados para evitar el traceback y la advertencia de animación sin renderizar.
