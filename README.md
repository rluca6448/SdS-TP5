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

Los argumentos opcionales son cantidad de agentes, semilla, lado de habitación y carpeta de salida, en ese orden. Sus valores por defecto son `500 42 50 output`. La puerta mide 0,8 m. En Windows, ajustar la ruta del ejecutable según el generador usado.

El programa ejecuta TSDMD puro hasta retirar a todos los agentes y genera:

- `states.txt`: estado inicial, estados cada tres pasos (`dt2 = 0,1 s`) y estado final vacío.
- `population.txt`: una fila por paso, incluido el inicial, con columnas `time inside active`.

La carpeta se crea si no existe. Cada ejecución reemplaza esos archivos; usar carpetas diferentes para conservar realizaciones. En consola se imprimen el tiempo de evacuación y el tiempo del último retiro.

Todavía faltan los esquemas EDMD e híbrido.

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
