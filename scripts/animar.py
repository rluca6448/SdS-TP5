"""Anima un states.txt de cualquiera de los tres esquemas.

Panel izquierdo: la habitación completa, con un recuadro que marca la zona ampliada.
Panel derecho: zoom a la puerta, con el borde de cada disco para ver contactos.

Colores (--color):
  esquema  rojo = avanzó por pasos (TS), azul = por eventos (ED). Por defecto en el híbrido.
  radio    del radio mínimo (contraído) al máximo (recuperado). Por defecto en TSDMD y EDMD,
           donde todos los agentes tienen el mismo esquema.

Ejemplos:
  python3 scripts/animar.py output/states.txt --esquema hybrid --guardar figuras/hibrido.mp4
  python3 scripts/animar.py output/states.txt --esquema edmd --velocidad 2 --desde 20 --hasta 60 --guardar edmd.gif
  python3 scripts/animar.py output/states.txt --esquema tsdmd --fotogramas 0 30 100 --carpeta-fotogramas figuras/fotos
  python3 scripts/animar.py output/states.txt            (abre una ventana)

states.txt se escribe con ./build/tp5 (TP5_STATES_EVERY=1 escribe un estado cada dt = 1/30 s).
"""

import argparse
from itertools import islice
from pathlib import Path

import matplotlib
import numpy as np

import comun


def leer_estados(path, instantes_video, instantes_fotos):
    """Lee solo los cuadros necesarios.

    instantes_video: instantes deseados, en orden creciente; se toma el primer
    cuadro en o después de cada uno. instantes_fotos: idem para los fotogramas.
    Devuelve (tiempos, estados) del video y una lista de (tiempo, estado) de fotos.
    """
    tiempos, estados = [], []
    fotos = []
    i_video = 0
    i_fotos = 0
    eps = 1e-6
    with Path(path).open() as archivo:
        for linea in archivo:
            if not linea.strip() or linea.startswith("#"):
                continue
            tiempo_texto, cantidad_texto = linea.split()
            tiempo = float(tiempo_texto)
            cantidad = int(cantidad_texto)
            quiere_video = i_video < len(instantes_video) and tiempo >= instantes_video[i_video] - eps
            quiere_foto = i_fotos < len(instantes_fotos) and tiempo >= instantes_fotos[i_fotos] - eps
            if not (quiere_video or quiere_foto):
                for _ in islice(archivo, cantidad):
                    pass
                continue
            filas = [fila.split() for fila in islice(archivo, cantidad)]
            estado = np.array(filas, dtype=float).reshape(cantidad, 7)
            if quiere_video:
                tiempos.append(tiempo)
                estados.append(estado)
                while i_video < len(instantes_video) and instantes_video[i_video] <= tiempo + eps:
                    i_video += 1
            while i_fotos < len(instantes_fotos) and instantes_fotos[i_fotos] <= tiempo + eps:
                fotos.append((tiempo, estado))
                i_fotos += 1
            if i_video >= len(instantes_video) and i_fotos >= len(instantes_fotos):
                break
    return tiempos, estados, fotos


def ultimo_tiempo(path):
    """Instante del último estado (recorre el archivo sin convertir los datos)."""
    ultimo = 0.0
    with Path(path).open() as archivo:
        for linea in archivo:
            if not linea.strip() or linea.startswith("#"):
                continue
            tiempo_texto, cantidad_texto = linea.split()
            ultimo = float(tiempo_texto)
            for _ in islice(archivo, int(cantidad_texto)):
                pass
    return ultimo


class Dibujo:
    """Figura de dos paneles que se actualiza con un estado."""

    def __init__(self, esquema, lado, puerta, zoom, color):
        import matplotlib.pyplot as plt
        from matplotlib.patches import Patch, Rectangle

        comun.estilo_figuras()
        plt.rcParams["axes.grid"] = False
        self.esquema = esquema
        self.color = color
        self.figura, self.ejes = plt.subplots(1, 2, figsize=(15, 8), layout="constrained",
                                              gridspec_kw={"width_ratios": [1, 1]})
        centro = lado / 2
        izquierda, derecha = centro - puerta / 2, centro + puerta / 2
        paredes = [((0, izquierda), (0, 0)), ((derecha, lado), (0, 0)),
                   ((0, 0), (0, lado)), ((lado, lado), (0, lado)), ((0, lado), (lado, lado))]
        for eje in self.ejes:
            for x, y in paredes:
                eje.plot(x, y, color="black", linewidth=2.5, solid_capstyle="butt")
            eje.axhline(comun.Y_RETIRO, color="gray", linestyle="--", linewidth=1)
            eje.set_aspect("equal")
            eje.set_xlabel("x [m]")
            eje.set_ylabel("y [m]")

        margen = 0.02 * lado
        self.ejes[0].set_xlim(-margen, lado + margen)
        self.ejes[0].set_ylim(comun.Y_RETIRO - margen, lado + margen)
        self.ejes[0].set_title("Habitación completa")
        x0, x1 = centro - zoom, centro + zoom
        y0, y1 = comun.Y_RETIRO - 0.4, 1.3 * zoom
        self.ejes[1].set_xlim(x0, x1)
        self.ejes[1].set_ylim(y0, y1)
        self.ejes[1].set_title("Zoom a la puerta")
        self.ejes[1].text(x1 - 0.05 * (x1 - x0), comun.Y_RETIRO + 0.05, "retiro (y = −1 m)",
                          ha="right", va="bottom", color="gray", fontsize=12)
        self.ejes[0].add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, edgecolor="black",
                                         linewidth=1.2, linestyle=":"))

        if color == "esquema":
            self.ejes[1].legend(handles=[Patch(color=comun.COLOR_TS, label="Por pasos (TS)"),
                                         Patch(color=comun.COLOR_ED, label="Por eventos (ED)")],
                                loc="upper right", framealpha=0.9)
        else:
            import matplotlib.cm as cm
            from matplotlib.colors import Normalize

            self.normal = Normalize(comun.RMIN, comun.RMAX)
            self.mapa = matplotlib.colormaps["viridis"]
            barra = self.figura.colorbar(cm.ScalarMappable(norm=self.normal, cmap=self.mapa),
                                         ax=self.ejes, shrink=0.75, pad=0.01)
            barra.set_label("radio [m]")
        self.titulo = self.figura.suptitle("", fontsize=18)
        self.colecciones = []

    def colores(self, estado):
        if self.color == "esquema":
            return np.where(estado[:, 6] == 0, comun.COLOR_TS, comun.COLOR_ED)
        return self.mapa(self.normal(estado[:, 5]))

    def dibujar(self, tiempo, estado):
        from matplotlib.collections import EllipseCollection

        for coleccion in self.colecciones:
            coleccion.remove()
        self.colecciones.clear()
        colores = self.colores(estado)
        for eje, borde in zip(self.ejes, (0.0, 0.6)):
            discos = EllipseCollection(widths=2 * estado[:, 5], heights=2 * estado[:, 5], angles=0,
                                       units="xy", offsets=estado[:, 1:3], offset_transform=eje.transData,
                                       facecolors=colores, edgecolors="black", linewidths=borde)
            eje.add_collection(discos)
            self.colecciones.append(discos)

        dentro = int(np.count_nonzero(estado[:, 2] > 0))
        texto = f"{comun.NOMBRES.get(self.esquema, self.esquema)}   t = {tiempo:6.1f} s   " \
                f"dentro: {dentro}   activos: {len(estado)}"
        if self.esquema == "hybrid" and len(estado) > 0:
            ts = int(np.count_nonzero(estado[:, 6] == 0))
            texto += f"   TS: {ts} ({ts / len(estado):.0%})"
        self.titulo.set_text(texto)
        return self.colecciones


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("estados", type=Path, help="Archivo states.txt")
    parser.add_argument("--esquema", choices=comun.ESQUEMAS, default="hybrid", help="Solo para el título y el color")
    parser.add_argument("--lado", type=float, default=50.0, help="Lado de la habitación [m]")
    parser.add_argument("--puerta", type=float, default=comun.PUERTA, help="Ancho de la puerta [m]")
    parser.add_argument("--zoom", type=float, default=2.5, help="Semiancho de la zona ampliada [m]")
    parser.add_argument("--color", choices=["auto", "esquema", "radio"], default="auto")
    parser.add_argument("--velocidad", type=float, default=5.0, help="Veces más rápido que el tiempo real")
    parser.add_argument("--fps", type=int, default=30, help="Cuadros por segundo del video")
    parser.add_argument("--desde", type=float, default=0.0, help="Instante inicial [s]")
    parser.add_argument("--hasta", type=float, default=None, help="Instante final [s] (por defecto, el último)")
    parser.add_argument("--guardar", type=Path, help="Guardar MP4 (requiere FFmpeg) o GIF")
    parser.add_argument("--fotogramas", type=float, nargs="*", default=[], help="Instantes [s] a guardar en PNG")
    parser.add_argument("--carpeta-fotogramas", type=Path, default=Path("fotogramas"))
    parser.add_argument("--dpi", type=int, default=100, help="Resolución del video")
    args = parser.parse_args()

    if args.guardar or args.fotogramas:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FFMpegWriter, FuncAnimation, PillowWriter, writers

    es_gif = args.guardar is not None and args.guardar.suffix.lower() == ".gif"
    if args.guardar and not es_gif and not writers.is_available("ffmpeg"):
        parser.error("Falta FFmpeg para exportar MP4 (Ubuntu/WSL: sudo apt install ffmpeg). "
                     "También se puede guardar un .gif sin FFmpeg.")

    hasta = args.hasta if args.hasta is not None else ultimo_tiempo(args.estados)
    paso = args.velocidad / args.fps
    instantes_video = list(np.arange(args.desde, hasta + 1e-9, paso)) if (args.guardar or not args.fotogramas) else []
    instantes_fotos = sorted(args.fotogramas)
    tiempos, estados, fotos = leer_estados(args.estados, instantes_video, instantes_fotos)

    color = args.color
    if color == "auto":
        color = "esquema" if args.esquema == "hybrid" else "radio"
    dibujo = Dibujo(args.esquema, args.lado, args.puerta, args.zoom, color)

    if fotos:
        args.carpeta_fotogramas.mkdir(parents=True, exist_ok=True)
        for tiempo, estado in fotos:
            dibujo.dibujar(tiempo, estado)
            destino = args.carpeta_fotogramas / f"{args.esquema}_t{tiempo:07.2f}.png"
            dibujo.figura.savefig(destino, dpi=150)
            print(f"  fotograma {destino}")

    if args.guardar or not args.fotogramas:
        if not estados:
            parser.error("No hay estados en el intervalo pedido.")
        animacion = FuncAnimation(dibujo.figura, lambda i: dibujo.dibujar(tiempos[i], estados[i]),
                                  frames=len(estados), interval=1000 / args.fps, repeat=False)
        if args.guardar:
            args.guardar.parent.mkdir(parents=True, exist_ok=True)
            if es_gif:
                escritor = PillowWriter(fps=args.fps)
            else:
                escritor = FFMpegWriter(fps=args.fps, codec="libx264", bitrate=4000,
                                        extra_args=["-pix_fmt", "yuv420p"])
            animacion.save(args.guardar, writer=escritor, dpi=args.dpi)
            print(f"  video {args.guardar} ({len(estados)} cuadros, x{args.velocidad:g} del tiempo real)")
        else:
            plt.show()
    plt.close(dibujo.figura)


if __name__ == "__main__":
    main()
