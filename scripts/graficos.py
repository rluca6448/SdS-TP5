"""Genera los gráficos del TP5 a partir de los resultados de correr.py.

  python3 scripts/graficos.py               todos los estudios que tengan resultados
  python3 scripts/graficos.py base barrido  solo esos

Las figuras van a figuras/ (PNG) y las tablas a figuras/*.csv.

Convenciones:
  - Mismo color y marcador para cada esquema en todas las figuras.
  - Las curvas temporales muestran realizaciones identificadas por su semilla, sin promediar.
  - En los escalares, el punto es la media de las M realizaciones y la barra es
    el DESVÍO ESTÁNDAR entre realizaciones.
  - El tiempo de cómputo es el del motor (sin escritura de archivos).
"""

import argparse
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import comun  # noqa: E402

LEYENDA_BARRAS = "barras: desvío estándar entre realizaciones"


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


def nota(eje, texto):
    """Aclaración debajo del eje, para no tapar datos."""
    eje.annotate(texto, xy=(1, 0), xycoords="axes fraction", xytext=(0, -58), textcoords="offset points",
                 ha="right", va="top", fontsize=11, color="dimgray")


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
        mapa = matplotlib.colormaps["tab10"]
        for i, corrida in enumerate(corridas):
            t, dentro, _, _ = comun.leer_poblacion(Path(corrida["carpeta"]) / "population.txt")
            eje.plot(t, dentro, color=mapa(i % 10), linewidth=1.4, label=f"semilla {corrida['semilla']}")
        eje.set_xlabel("tiempo [s]")
        eje.set_ylabel("agentes dentro de la habitación")
        eje.set_title(f"{comun.NOMBRES[esquema]}: población (N = 500, L = 50 m)")
        eje.set_xlim(left=0)
        eje.set_ylim(bottom=0)
        eje.legend(ncol=2, fontsize=11)
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
        eje.set_xlabel("tiempo [s]")
        eje.set_ylabel("agentes dentro de la habitación")
        eje.set_title(f"Población con el mismo estado inicial (semilla {semilla})")
        eje.set_xlim(left=0)
        eje.set_ylim(bottom=0)
        eje.legend()
        guardar(figura, "base_poblacion_comparacion", carpeta)

    # Fracción de agentes TS en el híbrido.
    if ("hybrid",) in por_esquema:
        corridas = sorted(por_esquema[("hybrid",)], key=lambda r: r["semilla"])
        figura, eje = plt.subplots(figsize=(10, 6))
        mapa = matplotlib.colormaps["tab10"]
        for i, corrida in enumerate(corridas):
            t, _, activos, ts = comun.leer_poblacion(Path(corrida["carpeta"]) / "population.txt")
            validos = activos > 0
            eje.plot(t[validos], ts[validos] / activos[validos], color=mapa(i % 10), linewidth=1.2,
                     label=f"semilla {corrida['semilla']}")
        eje.set_xlabel("tiempo [s]")
        eje.set_ylabel("fracción de agentes TS")
        eje.set_title("Híbrido: fracción de agentes que avanzan por pasos")
        eje.set_xlim(left=0)
        eje.set_ylim(0, 1.03)
        eje.legend(ncol=2, fontsize=11, loc="lower right")
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
        filas.append([comun.NOMBRES[esquema], len(corridas), f"{ev[0]:.2f}", f"{ev[1]:.2f}",
                      f"{mo[0]:.3f}", f"{mo[1]:.3f}", f"{fr[0]:.3f}"])
    escribir_tabla(carpeta / "base_resumen.csv", filas,
                   ["esquema", "M", "t_evac_media_s", "t_evac_desvio_s", "t_computo_media_s",
                    "t_computo_desvio_s", "fraccion_ts_media"])

    for indice, (nombre, etiqueta, logaritmica) in enumerate([
            ("base_evacuacion", "tiempo de evacuación [s]", False),
            ("base_tiempo_computo", "tiempo de cómputo [s]", True)]):
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
        eje.set_title(f"N = 500, L = 50 m, M = {len(por_esquema[(esquemas[0],)])}")
        nota(eje, LEYENDA_BARRAS)
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


def graficos_barrido(resumenes, base, carpeta):
    print("[barrido]")
    por_k = agrupar(resumenes, "k")
    ks = sorted(k for (k,) in por_k)
    refs = referencias(base)
    evac = [comun.media_y_desvio([r["t_evacuacion"] for r in por_k[(k,)]]) for k in ks]
    motor = [comun.media_y_desvio([r["t_motor"] for r in por_k[(k,)]]) for k in ks]
    fraccion = [comun.media_y_desvio([r["fraccion_ts_media"] for r in por_k[(k,)]]) for k in ks]
    m = len(por_k[(ks[0],)])

    escribir_tabla(carpeta / "barrido_resumen.csv",
                   [[k, len(por_k[(k,)]), f"{e[0]:.2f}", f"{e[1]:.2f}", f"{c[0]:.3f}", f"{c[1]:.3f}",
                     f"{f[0]:.3f}", f"{f[1]:.3f}"] for k, e, c, f in zip(ks, evac, motor, fraccion)],
                   ["k", "M", "t_evac_media_s", "t_evac_desvio_s", "t_computo_media_s", "t_computo_desvio_s",
                    "fraccion_ts_media", "fraccion_ts_desvio"])

    def figura_k(valores, etiqueta, nombre, indice_ref=None, logaritmica=False):
        figura, eje = plt.subplots(figsize=(10, 6))
        eje.errorbar(ks, [v[0] for v in valores], yerr=[v[1] for v in valores], fmt="-^",
                     color=comun.COLORES["hybrid"], capsize=6, label="Híbrido")
        if indice_ref is not None:
            for esquema, medidas in refs.items():
                linea_referencia(eje, esquema, medidas[indice_ref][0])
        eje.set_xscale("symlog", linthresh=1)
        eje.set_xticks(ks, [str(k) for k in ks])
        eje.set_xlabel("anticipación k [pasos de dt]")
        eje.set_ylabel(etiqueta)
        if logaritmica:
            eje.set_yscale("log")
        eje.set_title(f"Híbrido según k (N = 500, L = 50 m, M = {m})")
        eje.legend()
        nota(eje, LEYENDA_BARRAS)
        guardar(figura, nombre, carpeta)

    figura_k(evac, "tiempo de evacuación [s]", "barrido_evacuacion", indice_ref=0)
    figura_k(motor, "tiempo de cómputo [s]", "barrido_tiempo_computo", indice_ref=1, logaritmica=True)
    figura_k(fraccion, "fracción media de agentes TS", "barrido_fraccion_ts")


# ---------------------------------------------------------------------------
# Escalamientos
# ---------------------------------------------------------------------------

def graficos_escalamiento(resumenes, carpeta, estudio):
    print(f"[{estudio}]")
    titulo = "densidad constante 0,2 /m²" if estudio == "densidad" else "L = 50 m constante"
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
                          len(por_config[(esquema, n)]), f"{e[0]:.2f}", f"{e[1]:.2f}", f"{c[0]:.3f}", f"{c[1]:.3f}"])
    escribir_tabla(carpeta / f"{estudio}_resumen.csv", filas,
                   ["esquema", "N", "L_m", "densidad_m2", "M", "t_evac_media_s", "t_evac_desvio_s",
                    "t_computo_media_s", "t_computo_desvio_s"])

    for indice, (nombre, etiqueta, logaritmica) in enumerate([
            (f"{estudio}_evacuacion", "tiempo de evacuación [s]", False),
            (f"{estudio}_tiempo_computo", "tiempo de cómputo [s]", True)]):
        figura, eje = plt.subplots(figsize=(10, 6))
        for esquema in esquemas:
            ns, evac, motor = datos[esquema]
            valores = (evac, motor)[indice]
            eje.errorbar(ns, [v[0] for v in valores], yerr=[v[1] for v in valores],
                         fmt="-" + comun.MARCADORES[esquema], color=comun.COLORES[esquema], capsize=5,
                         label=comun.NOMBRES[esquema])
        eje.set_xlabel("N (agentes)")
        eje.set_ylabel(etiqueta)
        if logaritmica:
            eje.set_xscale("log")
            eje.set_yscale("log")
            ticks_n(eje, sorted({n for e in esquemas for n in datos[e][0]}))
        eje.set_title(f"Escalamiento con N, {titulo}")
        eje.legend()
        nota(eje, LEYENDA_BARRAS)
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
        eje.set_xlabel("N (agentes)")
        eje.set_ylabel("tiempo de cómputo / tiempo del TSDMD")
        eje.set_title(f"Costo relativo, {titulo}")
        eje.legend()
        guardar(figura, f"{estudio}_costo_relativo", carpeta)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("estudios", nargs="*", choices=["base", "barrido", "densidad", "lado"],
                        help="Por defecto, todos")
    parser.add_argument("--resultados", type=Path, default=comun.RESULTADOS)
    parser.add_argument("--figuras", type=Path, default=comun.FIGURAS)
    args = parser.parse_args()

    if not args.estudios:
        args.estudios = ["base", "barrido", "densidad", "lado"]
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
        else:
            graficos_escalamiento(resumenes, args.figuras, estudio)


if __name__ == "__main__":
    main()
