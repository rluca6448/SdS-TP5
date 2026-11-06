"""Corre los estudios del TP5 con los tres esquemas y guarda los resultados.

Estudios (cada uno se puede correr por separado):
  base         N=500, L=50 m, M realizaciones de cada esquema (híbrido con k=1)
  barrido      híbrido con N=500, L=50 m para varios k (TSDMD y EDMD de referencia: estudio base)
  densidad     escalamiento con N a densidad constante 0,2 /m2 (L = sqrt(N / 0,2))
  lado         escalamiento con N a L = 50 m constante
  animaciones  una corrida de cada esquema con estados, más su video y fotogramas
  todo         todo lo anterior

Cada corrida queda en resultados/<estudio>/<configuracion>/s<semilla>/ con
population.txt, la salida del programa (salida.txt) y resumen.json. Las corridas
que ya existen no se repiten (salvo con --rehacer), así que se puede cortar y retomar.

Las corridas en lote no escriben states.txt. El tiempo de cómputo que se reporta
es el del motor (solo step()), medido por el programa: no incluye escritura.

Ejemplos:
  python3 scripts/correr.py base
  python3 scripts/correr.py barrido --valores 0 1 2 4 8 16 32
  python3 scripts/correr.py densidad --n 100 200 500 1000 2000 --realizaciones 5
  python3 scripts/correr.py animaciones --velocidad 5
"""

import argparse
import json
import os
import re
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import comun


def configuracion(esquema, n, lado, k):
    nombre = f"{esquema}_N{n}_L{lado:g}"
    if esquema == "hybrid":
        nombre += f"_k{k}"
    return nombre


def leer_salida(texto):
    """Extrae los números que imprime tp5 ('Evacuation time: 231.5 s', etc.)."""
    valores = {}
    for linea in texto.splitlines():
        coincidencia = re.match(r"\s*([A-Za-z ]+[A-Za-z]):\s*([-+0-9.eE]+)", linea)
        if coincidencia:
            clave = coincidencia.group(1).strip().lower().replace(" ", "_")
            valores[clave] = float(coincidencia.group(2))
    return valores


def correr_una(ejecutable, carpeta, esquema, n, lado, semilla, k, cada_estados, rehacer):
    """Corre una simulación. Devuelve (resumen, si se ejecutó ahora)."""
    resumen_path = carpeta / "resumen.json"
    if resumen_path.exists() and not rehacer:
        with resumen_path.open() as archivo:
            return json.load(archivo), False

    carpeta.mkdir(parents=True, exist_ok=True)
    comando = [str(ejecutable), str(n), str(semilla), f"{lado:g}", str(carpeta), esquema]
    if esquema == "hybrid":
        comando.append(str(k))
    entorno = dict(os.environ, TP5_STATES_EVERY=str(cada_estados))
    inicio = time.perf_counter()
    proceso = subprocess.run(comando, capture_output=True, text=True, env=entorno)
    total = time.perf_counter() - inicio
    (carpeta / "salida.txt").write_text(proceso.stdout + proceso.stderr)
    if proceso.returncode != 0:
        raise RuntimeError(f"Falló {' '.join(comando)}:\n{proceso.stderr}")

    valores = leer_salida(proceso.stdout)
    tiempos, dentro, activos, ts = comun.leer_poblacion(carpeta / "population.txt")
    resumen = {
        "esquema": esquema,
        "N": n,
        "L": lado,
        "k": k if esquema == "hybrid" else None,
        "semilla": semilla,
        "t_evacuacion": valores.get("evacuation_time"),
        "t_ultimo_retiro": valores.get("last_removal_time"),
        "t_motor": valores.get("engine_time"),
        "t_proceso": total,
        "fraccion_ts_media": comun.fraccion_ts_media(activos, ts),
        "evacuaron_todos": bool(dentro[-1] == 0 and activos[-1] == 0),
        "salida": valores,
    }
    with resumen_path.open("w") as archivo:
        json.dump(resumen, archivo, indent=2)
    return resumen, True


def correr_lote(args, estudio, corridas):
    """corridas: lista de (esquema, n, lado, k). Se corren M semillas de cada una."""
    tareas = []
    for esquema, n, lado, k in corridas:
        for r in range(args.realizaciones):
            semilla = args.semilla_inicial + r
            carpeta = args.resultados / estudio / configuracion(esquema, n, lado, k) / f"s{semilla}"
            tareas.append((esquema, n, lado, k, semilla, carpeta))

    print(f"[{estudio}] {len(tareas)} corridas ({args.jobs} en paralelo)")
    if args.jobs > 1:
        print("  Aviso: con varias corridas en paralelo, el tiempo de cómputo puede ser mayor"
              " que corriendo de a una. Para los tiempos finales usar --jobs 1.")
    hechas = 0
    with ThreadPoolExecutor(max_workers=args.jobs) as grupo:
        futuros = {
            grupo.submit(correr_una, args.ejecutable, carpeta, esquema, n, lado, semilla, k, 0, args.rehacer):
                (esquema, n, lado, k, semilla)
            for esquema, n, lado, k, semilla, carpeta in tareas
        }
        for futuro in as_completed(futuros):
            esquema, n, lado, k, semilla = futuros[futuro]
            resumen, ejecutada = futuro.result()
            hechas += 1
            estado = "" if ejecutada else " (ya estaba)"
            etiqueta = comun.NOMBRES[esquema] + (f" k={k}" if esquema == "hybrid" else "")
            print(f"  [{hechas}/{len(tareas)}] {etiqueta:12s} N={n:<5d} L={lado:<6g} semilla {semilla}: "
                  f"evacuación {resumen['t_evacuacion']:8.2f} s, motor {resumen['t_motor']:8.2f} s{estado}",
                  flush=True)
            if not resumen["evacuaron_todos"]:
                print("    Aviso: quedaron agentes sin evacuar.")


def estudio_base(args):
    corridas = [(esquema, 500, 50.0, args.k) for esquema in args.esquemas]
    correr_lote(args, "base", corridas)


def estudio_barrido(args):
    corridas = [("hybrid", 500, 50.0, k) for k in args.valores]
    correr_lote(args, "barrido", corridas)


def estudio_densidad(args):
    corridas = [(esquema, n, (n / 0.2) ** 0.5, args.k) for n in args.n for esquema in args.esquemas]
    correr_lote(args, "densidad", corridas)


def estudio_lado(args):
    corridas = [(esquema, n, 50.0, args.k) for n in args.n_lado for esquema in args.esquemas]
    correr_lote(args, "lado", corridas)


def estudio_animaciones(args):
    animar = Path(__file__).resolve().parent / "animar.py"
    destino = args.figuras / "animaciones"
    destino.mkdir(parents=True, exist_ok=True)
    for esquema in args.esquemas:
        carpeta = args.resultados / "animaciones" / configuracion(esquema, args.n_animacion, 50.0, args.k)
        print(f"[animaciones] {comun.NOMBRES[esquema]}: simulando (N={args.n_animacion}, semilla {args.semilla_animacion})...",
              flush=True)
        correr_una(args.ejecutable, carpeta, esquema, args.n_animacion, 50.0, args.semilla_animacion, args.k,
                   1, args.rehacer)
        video = destino / f"{esquema}.mp4"
        comando = [sys.executable, str(animar), str(carpeta / "states.txt"), "--esquema", esquema,
                   "--lado", "50", "--velocidad", str(args.velocidad), "--guardar", str(video),
                   "--fotogramas", *[str(t) for t in args.fotogramas],
                   "--carpeta-fotogramas", str(destino / f"fotogramas_{esquema}")]
        print(f"[animaciones] {comun.NOMBRES[esquema]}: generando {video}...", flush=True)
        subprocess.run(comando, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("estudio", choices=["base", "barrido", "densidad", "lado", "animaciones", "todo"])
    parser.add_argument("--ejecutable", type=Path, default=comun.EJECUTABLE)
    parser.add_argument("--resultados", type=Path, default=comun.RESULTADOS)
    parser.add_argument("--figuras", type=Path, default=comun.FIGURAS, help="Destino de videos y fotogramas")
    parser.add_argument("--esquemas", nargs="+", choices=comun.ESQUEMAS, default=list(comun.ESQUEMAS))
    parser.add_argument("--realizaciones", type=int, default=10, help="Realizaciones por configuración (M)")
    parser.add_argument("--semilla-inicial", type=int, default=100)
    parser.add_argument("--k", type=int, default=1, help="Anticipación del híbrido fuera del barrido")
    parser.add_argument("--valores", type=int, nargs="+", default=[0, 1, 2, 4, 8, 16, 32],
                        help="Valores de k del barrido")
    parser.add_argument("--n", type=int, nargs="+", default=[100, 200, 500, 1000, 2000],
                        help="N del escalamiento a densidad constante")
    parser.add_argument("--n-lado", type=int, nargs="+", default=[100, 200, 500, 1000, 1500],
                        help="N del escalamiento a L = 50 m")
    parser.add_argument("--n-animacion", type=int, default=500)
    parser.add_argument("--semilla-animacion", type=int, default=42)
    parser.add_argument("--velocidad", type=float, default=5.0, help="Veces más rápido que el tiempo real")
    parser.add_argument("--fotogramas", type=float, nargs="+", default=[0, 10, 30, 100, 200],
                        help="Instantes [s] de los fotogramas en PNG")
    parser.add_argument("--jobs", type=int, default=1, help="Corridas en paralelo")
    parser.add_argument("--rehacer", action="store_true", help="Repetir corridas ya hechas")
    args = parser.parse_args()

    if not args.ejecutable.exists():
        parser.error(f"No existe {args.ejecutable}. Compilar antes: cmake --build build --config Release")

    estudios = {
        "base": estudio_base,
        "barrido": estudio_barrido,
        "densidad": estudio_densidad,
        "lado": estudio_lado,
        "animaciones": estudio_animaciones,
    }
    seleccion = list(estudios) if args.estudio == "todo" else [args.estudio]
    for nombre in seleccion:
        estudios[nombre](args)


if __name__ == "__main__":
    main()
