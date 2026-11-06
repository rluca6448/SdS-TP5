"""Constantes y utilidades compartidas por correr.py, graficos.py y animar.py."""

import json
from pathlib import Path

import numpy as np

RAIZ = Path(__file__).resolve().parent.parent
RESULTADOS = RAIZ / "resultados"
FIGURAS = RAIZ / "figuras"
EJECUTABLE = RAIZ / "build" / "tp5"

ESQUEMAS = ("tsdmd", "edmd", "hybrid")
NOMBRES = {"tsdmd": "TSDMD", "edmd": "EDMD", "hybrid": "Híbrido"}
# Mismo color y marcador para cada esquema en todas las figuras (el marcador
# permite distinguirlos sin depender del color).
COLORES = {"tsdmd": "tab:red", "edmd": "tab:blue", "hybrid": "tab:purple"}
MARCADORES = {"tsdmd": "o", "edmd": "s", "hybrid": "^"}
# En las animaciones: agente que avanzó por pasos (TS) o por eventos (ED).
COLOR_TS = "tab:red"
COLOR_ED = "tab:blue"

PUERTA = 0.8          # [m]
RMIN = 0.10           # [m]
RMAX = 0.25           # [m]
Y_RETIRO = -1.0       # [m]


def estilo_figuras():
    """Letras grandes y legibles en una diapositiva."""
    import matplotlib.pyplot as plt

    plt.rcParams.update({
        "font.size": 15,
        "axes.titlesize": 17,
        "axes.labelsize": 17,
        "xtick.labelsize": 14,
        "ytick.labelsize": 14,
        "legend.fontsize": 13,
        "lines.linewidth": 2.0,
        "lines.markersize": 8,
        "figure.dpi": 100,
        "savefig.dpi": 150,
        "axes.grid": True,
        "grid.alpha": 0.3,
    })


def leer_poblacion(path):
    """Columnas de population.txt: tiempo, dentro, activos, agentes TS."""
    datos = np.loadtxt(path, comments="#", ndmin=2)
    if datos.shape[1] == 3:   # salida anterior, sin la columna de TS
        datos = np.column_stack([datos, np.full(len(datos), np.nan)])
    return datos[:, 0], datos[:, 1], datos[:, 2], datos[:, 3]


def fraccion_ts_media(activos, ts):
    """Promedio temporal de la fracción de agentes TS, solo mientras hay activos."""
    con_agentes = activos > 0
    if not np.any(con_agentes):
        return float("nan")
    return float(np.mean(ts[con_agentes] / activos[con_agentes]))


def leer_resumenes(raiz=RESULTADOS):
    """Todos los resumen.json debajo de raiz."""
    resumenes = []
    for path in sorted(Path(raiz).glob("**/resumen.json")):
        with path.open() as archivo:
            resumen = json.load(archivo)
        resumen["carpeta"] = str(path.parent)
        resumenes.append(resumen)
    return resumenes


def media_y_desvio(valores):
    valores = np.asarray(valores, dtype=float)
    if len(valores) == 0:
        return float("nan"), float("nan")
    if len(valores) == 1:
        return float(valores[0]), 0.0
    return float(np.mean(valores)), float(np.std(valores, ddof=1))
