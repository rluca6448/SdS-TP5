"""Genera los gráficos del TP5 a partir de los resultados de correr.py.

  python3 scripts/graficos.py               todos los estudios que tengan resultados
  python3 scripts/graficos.py base barrido  solo esos
  python3 scripts/graficos.py umbral        híbrido según la distancia de cierre d*, una figura por N

Las figuras van a figuras/ (PNG) y las tablas a figuras/*.csv.

Convenciones:
  - Mismo color y marcador para cada esquema en todas las figuras.
  - Las curvas temporales muestran realizaciones identificadas por su semilla, sin promediar.
  - En los escalares, el punto es la media de las M realizaciones y la barra es
    el DESVÍO ESTÁNDAR entre realizaciones.
  - Guía de presentaciones: sin títulos ni notas dentro de las figuras (los
    parámetros fijos van al costado, en la diapositiva), ejes en palabras con
    unidades MKS entre paréntesis, letra >= 20, potencias de 10 con supraíndice
    y tablas con las cifras significativas del desvío.
  - El tiempo de cómputo es el del motor (sin escritura de archivos).
"""

import argparse
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib
import matplotlib.ticker

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import comun  # noqa: E402



def guardar(figura, nombre, carpeta):
    carpeta.mkdir(parents=True, exist_ok=True)
    destino = carpeta / f"{nombre}.png"
    figura.savefig(destino, bbox_inches="tight")
    plt.close(figura)
    print(f"  {destino.relative_to(comun.RAIZ) if destino.is_relative_to(comun.RAIZ) else destino}")


def agrupar(resumenes, *claves):
    grupos = defaultdict(list)
    for resumen in resumenes:
        grupos[tuple(resumen[clave] for clave in claves)].append(resumen)
    return grupos


def ticks_n(eje, ns):
    """En los ejes logarítmicos de N, marcar los N usados con números enteros."""
    from matplotlib.ticker import NullFormatter

    eje.set_xticks(ns, [str(n) for n in ns])
    eje.xaxis.set_minor_formatter(NullFormatter())


def escribir_tabla(path, filas, encabezado):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as archivo:
        escritor = csv.writer(archivo)
        escritor.writerow(encabezado)
        escritor.writerows(filas)
    print(f"  {path.relative_to(comun.RAIZ) if path.is_relative_to(comun.RAIZ) else path}")


# ---------------------------------------------------------------------------
# Caso base
# ---------------------------------------------------------------------------

def graficos_base(resumenes, carpeta):
    print("[base]")
    por_esquema = agrupar(resumenes, "esquema")
    esquemas = [e for e in comun.ESQUEMAS if (e,) in por_esquema]

    # Población dentro vs t: una figura por esquema, cada realización identificada.
    for esquema in esquemas:
        corridas = sorted(por_esquema[(esquema,)], key=lambda r: r["semilla"])
        figura, eje = plt.subplots(figsize=(10, 6))
        for corrida in corridas:
            t, dentro, _, _ = comun.leer_poblacion(Path(corrida["carpeta"]) / "population.txt")
            eje.plot(t, dentro, color=comun.COLORES[esquema], linewidth=1.2, alpha=0.6)
        eje.set_xlabel("Tiempo (s)")
        eje.set_ylabel("Agentes dentro de la habitación")
        eje.set_xlim(left=0)
        eje.set_ylim(bottom=0)
        guardar(figura, f"base_poblacion_{esquema}", carpeta)

    # Comparación de los tres esquemas con la misma semilla.
    semillas_comunes = set.intersection(*[{r["semilla"] for r in por_esquema[(e,)]} for e in esquemas])
    if semillas_comunes:
        semilla = min(semillas_comunes)
        figura, eje = plt.subplots(figsize=(10, 6))
        for esquema in esquemas:
            corrida = next(r for r in por_esquema[(esquema,)] if r["semilla"] == semilla)
            t, dentro, _, _ = comun.leer_poblacion(Path(corrida["carpeta"]) / "population.txt")
            eje.plot(t, dentro, color=comun.COLORES[esquema], label=comun.NOMBRES[esquema])
        eje.set_xlabel("Tiempo (s)")
        eje.set_ylabel("Agentes dentro de la habitación")
        eje.set_xlim(left=0)
        eje.set_ylim(bottom=0)
        eje.legend()
        guardar(figura, "base_poblacion_comparacion", carpeta)

    # Fracción de agentes TS en el híbrido.
    if ("hybrid",) in por_esquema:
        corridas = sorted(por_esquema[("hybrid",)], key=lambda r: r["semilla"])
        figura, eje = plt.subplots(figsize=(10, 6))
        for corrida in corridas:
            t, _, activos, ts = comun.leer_poblacion(Path(corrida["carpeta"]) / "population.txt")
            # Se omite t = 0: el estado inicial marca a todos como TS antes de clasificar.
            validos = (activos > 0) & (t > 0)
            eje.plot(t[validos], ts[validos] / activos[validos], color=comun.COLORES["hybrid"],
                     linewidth=1.2, alpha=0.6)
        eje.set_xlabel("Tiempo (s)")
        eje.set_ylabel("Fracción de agentes por pasos")
        eje.set_xlim(left=0)
        eje.set_ylim(0, 1.03)
        guardar(figura, "base_fraccion_ts", carpeta)

    # Escalares: tiempo de evacuación y tiempo de cómputo.
    filas = []
    medidas = {}
    for esquema in esquemas:
        corridas = por_esquema[(esquema,)]
        ev = comun.media_y_desvio([r["t_evacuacion"] for r in corridas])
        mo = comun.media_y_desvio([r["t_motor"] for r in corridas])
        fr = comun.media_y_desvio([r["fraccion_ts_media"] for r in corridas])
        medidas[esquema] = (ev, mo)
        filas.append([comun.NOMBRES[esquema], len(corridas), *comun.con_error(*ev),
                      *comun.con_error(*mo), *comun.con_error(*fr)])
    escribir_tabla(carpeta / "base_resumen.csv", filas,
                   ["esquema", "M", "t_evac_media_s", "t_evac_desvio_s", "t_computo_media_s",
                    "t_computo_desvio_s", "fraccion_ts_media", "fraccion_ts_desvio"])

    for indice, (nombre, etiqueta, logaritmica) in enumerate([
            ("base_evacuacion", "Tiempo de evacuación (s)", False),
            ("base_tiempo_computo", "Tiempo de cómputo (s)", True)]):
        figura, eje = plt.subplots(figsize=(8, 6))
        for x, esquema in enumerate(esquemas):
            media, desvio = medidas[esquema][indice]
            eje.errorbar(x, media, yerr=desvio, fmt=comun.MARCADORES[esquema], color=comun.COLORES[esquema],
                         capsize=8, markersize=11, elinewidth=2)
        eje.set_xticks(range(len(esquemas)), [comun.NOMBRES[e] for e in esquemas])
        eje.set_xlim(-0.5, len(esquemas) - 0.5)
        eje.set_ylabel(etiqueta)
        if logaritmica:
            eje.set_yscale("log")
        guardar(figura, nombre, carpeta)


# ---------------------------------------------------------------------------
# Barrido del parámetro del híbrido
# ---------------------------------------------------------------------------

def referencias(base):
    """Media y desvío de TSDMD y EDMD del caso base, para dibujar como referencia."""
    salida = {}
    for esquema in ("tsdmd", "edmd"):
        corridas = [r for r in base if r["esquema"] == esquema]
        if corridas:
            salida[esquema] = (comun.media_y_desvio([r["t_evacuacion"] for r in corridas]),
                               comun.media_y_desvio([r["t_motor"] for r in corridas]))
    return salida


def linea_referencia(eje, esquema, media):
    eje.axhline(media, color=comun.COLORES[esquema], linestyle="--", linewidth=1.8,
                label=f"{comun.NOMBRES[esquema]} (media)")


def barrido_parametro(resumenes, refs, carpeta, prefijo, clave, etiqueta_x, escala_x):
    """Tabla y figuras del híbrido en función de un parámetro (k o d*), con TSDMD y EDMD de referencia."""
    por_valor = agrupar(resumenes, clave)
    xs = sorted(x for (x,) in por_valor)
    evac = [comun.media_y_desvio([r["t_evacuacion"] for r in por_valor[(x,)]]) for x in xs]
    motor = [comun.media_y_desvio([r["t_motor"] for r in por_valor[(x,)]]) for x in xs]
    fraccion = [comun.media_y_desvio([r["fraccion_ts_media"] for r in por_valor[(x,)]]) for x in xs]

    escribir_tabla(carpeta / f"{prefijo}_resumen.csv",
                   [[f"{x:g}", len(por_valor[(x,)]), *comun.con_error(*e), *comun.con_error(*c),
                     *comun.con_error(*f)] for x, e, c, f in zip(xs, evac, motor, fraccion)],
                   [clave, "M", "t_evac_media_s", "t_evac_desvio_s", "t_computo_media_s", "t_computo_desvio_s",
                    "fraccion_ts_media", "fraccion_ts_desvio"])

    def figura(valores, etiqueta, nombre, indice_ref=None, logaritmica=False):
        figura, eje = plt.subplots(figsize=(10, 6))
        eje.errorbar(xs, [v[0] for v in valores], yerr=[v[1] for v in valores], fmt="-^",
                     color=comun.COLORES["hybrid"], capsize=6, label="Híbrido")
        if indice_ref is not None:
            for esquema, medidas in refs.items():
                linea_referencia(eje, esquema, medidas[indice_ref][0])
        if escala_x == "symlog":
            eje.set_xscale("symlog", linthresh=1)
        else:
            eje.set_xscale(escala_x)
        eje.set_xticks(xs, [f"{x:g}" for x in xs])
        eje.xaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
        eje.set_xlabel(etiqueta_x)
        eje.set_ylabel(etiqueta)
        if logaritmica:
            eje.set_yscale("log")
        eje.legend()
        guardar(figura, nombre, carpeta)

    figura(evac, "Tiempo de evacuación (s)", f"{prefijo}_evacuacion", indice_ref=0)
    figura(motor, "Tiempo de cómputo (s)", f"{prefijo}_tiempo_computo", indice_ref=1, logaritmica=True)
    figura(fraccion, "Fracción media de agentes por pasos", f"{prefijo}_fraccion_ts")


def graficos_barrido(resumenes, base, carpeta):
    print("[barrido]")
    barrido_parametro(resumenes, referencias(base), carpeta, "barrido", "k",
                      "Anticipación (pasos temporales)", "symlog")


def graficos_umbral(resumenes, carpeta):
    """Un juego de figuras por N: híbrido según d*, con TSDMD y EDMD del mismo N."""
    print("[umbral]")
    for n in sorted({r["N"] for r in resumenes}):
        del_n = [r for r in resumenes if r["N"] == n]
        hibrido = [r for r in del_n if r["esquema"] == "hybrid"]
        if hibrido:
            barrido_parametro(hibrido, referencias(del_n), carpeta, f"umbral_N{n}", "d",
                              "Distancia de cierre (m)", "log")


# ---------------------------------------------------------------------------
# Escalamientos
# ---------------------------------------------------------------------------

def graficos_escalamiento(resumenes, carpeta, estudio):
    print(f"[{estudio}]")
    por_config = agrupar(resumenes, "esquema", "N")
    esquemas = [e for e in comun.ESQUEMAS if any(c[0] == e for c in por_config)]
    datos = {}
    filas = []
    for esquema in esquemas:
        ns = sorted(n for (e, n) in por_config if e == esquema)
        evac = [comun.media_y_desvio([r["t_evacuacion"] for r in por_config[(esquema, n)]]) for n in ns]
        motor = [comun.media_y_desvio([r["t_motor"] for r in por_config[(esquema, n)]]) for n in ns]
        datos[esquema] = (ns, evac, motor)
        for n, e, c in zip(ns, evac, motor):
            lado = por_config[(esquema, n)][0]["L"]
            filas.append([comun.NOMBRES[esquema], n, f"{lado:.2f}", f"{n / lado ** 2:.3f}",
                          len(por_config[(esquema, n)]), *comun.con_error(*e), *comun.con_error(*c)])
    escribir_tabla(carpeta / f"{estudio}_resumen.csv", filas,
                   ["esquema", "N", "L_m", "densidad_m2", "M", "t_evac_media_s", "t_evac_desvio_s",
                    "t_computo_media_s", "t_computo_desvio_s"])

    for indice, (nombre, etiqueta, logaritmica) in enumerate([
            (f"{estudio}_evacuacion", "Tiempo de evacuación (s)", False),
            (f"{estudio}_tiempo_computo", "Tiempo de cómputo (s)", True)]):
        figura, eje = plt.subplots(figsize=(10, 6))
        for esquema in esquemas:
            ns, evac, motor = datos[esquema]
            valores = (evac, motor)[indice]
            eje.errorbar(ns, [v[0] for v in valores], yerr=[v[1] for v in valores],
                         fmt="-" + comun.MARCADORES[esquema], color=comun.COLORES[esquema], capsize=5,
                         label=comun.NOMBRES[esquema])
        eje.set_xlabel("Número de agentes")
        eje.set_ylabel(etiqueta)
        if logaritmica:
            eje.set_xscale("log")
            eje.set_yscale("log")
            ticks_n(eje, sorted({n for e in esquemas for n in datos[e][0]}))
        eje.legend()
        guardar(figura, nombre, carpeta)

    # Costo relativo al TSDMD, con la misma N.
    if "tsdmd" in datos:
        ns_ts, _, motor_ts = datos["tsdmd"]
        referencia = dict(zip(ns_ts, [m[0] for m in motor_ts]))
        figura, eje = plt.subplots(figsize=(10, 6))
        for esquema in esquemas:
            if esquema == "tsdmd":
                continue
            ns, _, motor = datos[esquema]
            comunes = [(n, m[0] / referencia[n]) for n, m in zip(ns, motor) if n in referencia]
            if comunes:
                eje.plot([c[0] for c in comunes], [c[1] for c in comunes], "-" + comun.MARCADORES[esquema],
                         color=comun.COLORES[esquema], label=comun.NOMBRES[esquema])
        eje.axhline(1.0, color=comun.COLORES["tsdmd"], linestyle="--", label="TSDMD")
        eje.set_xscale("log")
        eje.set_yscale("log")
        ticks_n(eje, ns_ts)
        eje.set_xlabel("Número de agentes")
        eje.set_ylabel("Costo relativo al TSDMD")
        eje.legend()
        guardar(figura, f"{estudio}_costo_relativo", carpeta)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    todos = ["base", "barrido", "umbral", "densidad", "lado"]
    # Sin `choices`: argparse rechaza la lista vacía de nargs="*" contra choices.
    parser.add_argument("estudios", nargs="*", help=f"Por defecto, todos: {' '.join(todos)}")
    parser.add_argument("--resultados", type=Path, default=comun.RESULTADOS)
    parser.add_argument("--figuras", type=Path, default=comun.FIGURAS)
    args = parser.parse_args()

    desconocidos = [e for e in args.estudios if e not in todos]
    if desconocidos:
        parser.error(f"estudios desconocidos: {' '.join(desconocidos)} (opciones: {' '.join(todos)})")
    if not args.estudios:
        args.estudios = todos
    comun.estilo_figuras()
    base = comun.leer_resumenes(args.resultados / "base")
    for estudio in args.estudios:
        resumenes = comun.leer_resumenes(args.resultados / estudio)
        if not resumenes:
            print(f"[{estudio}] sin resultados en {args.resultados / estudio}; correr: python3 scripts/correr.py {estudio}")
            continue
        incompletas = [r for r in resumenes if not r["evacuaron_todos"]]
        if incompletas:
            print(f"  Aviso: {len(incompletas)} corridas de '{estudio}' no evacuaron a todos.")
        if estudio == "base":
            graficos_base(resumenes, args.figuras)
        elif estudio == "barrido":
            graficos_barrido(resumenes, base, args.figuras)
        elif estudio == "umbral":
            graficos_umbral(resumenes, args.figuras)
        else:
            graficos_escalamiento(resumenes, args.figuras, estudio)


if __name__ == "__main__":
    main()
