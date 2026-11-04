"""Anima states.txt sin ejecutar el motor de simulacion."""

import argparse
from pathlib import Path

import matplotlib
import numpy as np


def read_states(path, every):
    times, states = [], []
    frame = 0
    with path.open() as file:
        for line in file:
            if not line.strip() or line.startswith("#"):
                continue
            time, count = line.split()
            count = int(count)
            rows = [next(file).split() for _ in range(count)]
            if frame % every == 0 or count == 0:
                times.append(float(time))
                states.append(np.array(rows, dtype=float).reshape(count, 7))
            frame += 1
    return times, states


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path, help="Archivo states.txt")
    parser.add_argument("--side", type=float, default=50.0)
    parser.add_argument("--door", type=float, default=0.8)
    parser.add_argument("--every", type=int, default=1, help="Mostrar uno de cada N estados")
    parser.add_argument("--fps", type=int, default=10)
    output = parser.add_mutually_exclusive_group()
    output.add_argument("--save", type=Path, help="Guardar GIF o MP4 sin abrir ventana")
    output.add_argument("--png", type=Path, help="Guardar un cuadro sin abrir ventana")
    parser.add_argument("--frame", type=int, default=0, help="Cuadro para --png, despues de --every")
    args = parser.parse_args()

    if args.save or args.png:
        matplotlib.use("Agg")

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter, FFMpegWriter, writers
    from matplotlib.collections import EllipseCollection
    from matplotlib.patches import Patch

    if args.save and args.save.suffix.lower() != ".gif" and not writers.is_available("ffmpeg"):
        parser.error("Falta FFmpeg para exportar MP4. En Ubuntu/WSL: sudo apt install ffmpeg. "
                     "Tambien podes usar --save output/animacion.gif sin FFmpeg.")

    times, states = read_states(args.path, args.every)
    fig, axes = plt.subplots(1, 2, figsize=(12, 6), layout="constrained")
    center = args.side / 2
    left, right = center - args.door / 2, center + args.door / 2
    walls = [((0, left), (0, 0)), ((right, args.side), (0, 0)),
             ((0, 0), (0, args.side)), ((args.side, args.side), (0, args.side)),
             ((0, args.side), (args.side, args.side))]
    for ax in axes:
        for x, y in walls:
            ax.plot(x, y, color="black", linewidth=1.5)
        ax.set_aspect("equal")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
    axes[0].set_xlim(-1, args.side + 1)
    axes[0].set_ylim(-1.3, args.side + 1)
    axes[0].set_title("Habitacion completa")
    axes[1].set_xlim(center - 2.5, center + 2.5)
    axes[1].set_ylim(-1.3, 4)
    axes[1].set_title("Detalle de la puerta")
    axes[1].legend(handles=[Patch(color="tab:red", label="Time driven"),
                            Patch(color="tab:blue", label="Event driven")], loc="upper right")
    title = fig.suptitle("")
    collections = []

    def draw(frame):
        state = states[frame]
        for collection in collections:
            collection.remove()
        collections.clear()
        for ax in axes:
            particles = EllipseCollection(
                widths=2 * state[:, 5], heights=2 * state[:, 5], angles=0,
                units="xy", offsets=state[:, 1:3], offset_transform=ax.transData,
                facecolors=np.where(state[:, 6] == 0, "tab:red", "tab:blue"),
                edgecolors="none")
            ax.add_collection(particles)
            collections.append(particles)
        inside = np.count_nonzero(state[:, 2] > 0)
        title.set_text(f"t = {times[frame]:.2f} s | Dentro: {inside} | Activos: {len(state)}")

    destination = args.png or args.save
    if destination:
        destination.parent.mkdir(parents=True, exist_ok=True)
    if args.png:
        draw(args.frame)
        fig.savefig(args.png, dpi=120)
    else:
        animation = FuncAnimation(fig, draw, frames=len(states), interval=1000 / args.fps,
                                  repeat=False)
        if args.save:
            writer = PillowWriter(fps=args.fps) if args.save.suffix.lower() == ".gif" else FFMpegWriter(fps=args.fps)
            animation.save(args.save, writer=writer, dpi=100)
        else:
            plt.show()
    plt.close(fig)


if __name__ == "__main__":
    main()
