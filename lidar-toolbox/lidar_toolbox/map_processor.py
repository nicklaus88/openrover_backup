from pathlib import Path
from dataclasses import dataclass
from functools import cached_property
from typing import Iterable
import cv2, yaml, numpy as np

def repeat(action: callable, *args, **kwargs):
    while True:
        yield action(*args, **kwargs)


def readline(stream: Iterable[bytes]) -> str:
    line: bytes = b""
    for byte in stream:
        line += byte
        if byte == b"\n":
            break
    return line.decode("utf-8")


def distance(p0: Iterable[float], p1: Iterable[float]):
    return sum((x0 - x1) ** 2 for x0, x1 in zip(p0, p1)) ** 0.5


@dataclass
class SLAM:
    image: str
    mode: str
    resolution: float
    origin: tuple[float, float, float]
    negate: int
    occupied_thresh: float
    free_thresh: float

    def __init__(self, path: Path, scale: float = 8.0):
        with path.open("r") as file:
            self.__dict__.update(yaml.safe_load(file))
        self.scale = scale
        self.dir = path.parent

    @cached_property
    def map(self):
        with (self.dir / self.image).open("rb") as pgm:
            stream = repeat(pgm.read, 1)
            assert readline(stream) == "P5\n"
            (w, h) = [int(i) for i in readline(stream).split()]
            depth = int(readline(stream))
            assert depth <= 255, f"Unsupported PGM depth ({depth})"
            count = w * h
            buffer = pgm.read(count)
            img = np.frombuffer(buffer, np.uint8, count).reshape((h, w))
            return cv2.resize(
                img,
                (0, 0),
                fx=self.scale,
                fy=self.scale,
                interpolation=cv2.INTER_NEAREST,
            )

    def __getitem__(self, pos: tuple[float, float]) -> tuple[int, int]:
        h, w = self.map.shape
        x0, y0, _ = self.origin
        x1, y1 = pos
        resolution = self.resolution / self.scale
        x = int((+x1 - x0) / resolution)
        y = int((+y1 - y0) / resolution)
        return x, h - y
    
    def __truediv__(self, seg: str | Path):
        return self.dir / seg

    @property
    def trajectory(self):
        trj_list = self.dir / "trj.list"
        if not (trj_list.exists() and trj_list.is_file()):
            raise FileNotFoundError(f"{trj_list} not found or not a file")
        with trj_list.open("rt") as file:
            for line in file:
                if line.startswith("#"):
                    continue
                t, x, y, r, l = map(float, line.split(","))
                yield x, y


if __name__ == "__main__":
    import sys
    from argparse import ArgumentParser

    parser = ArgumentParser()
    parser.add_argument("path", type=Path, help="Paths to PGM config file", nargs="+")
    args = parser.parse_args()
    for path in map(Path, args.path):
        TRJ_COMPLETE_FLAG = path.parent / "TRJ_COMPLETE.flag"
        TRJ_COMPLETE_FLAG.unlink(missing_ok=True)
        if not path.exists() or not path.is_file():
            print(f"Error: {path} not found or not a file", file=sys.stderr)
            continue
        slam = SLAM(path)
        cv2.imwrite(f"{path.parent / path.stem}.png", slam.map)
        # Now try to render the trajectory
        h, w = slam.map.shape
        slam_map = cv2.cvtColor(slam.map, cv2.COLOR_GRAY2BGR)
        p0 = None
        travel = 0.0
        for p1 in slam.trajectory:
            if p0 is None:
                p0 = p1
                continue
            d = distance(p0, p1)
            if d <= 0.1:
                continue
            else:
                travel += d
                cv2.line(slam_map, slam[p0], slam[p1], (0, 0, 255), 2)
                p0 = p1

        (slam / "trj.len").write_text(f"{travel:.4f}\n")
        print("Travel distance:", travel)
        cv2.drawMarker(slam_map, slam[0, 0], (0, 128, 0), cv2.MARKER_TRIANGLE_UP, 64, 4)
        # cv2.drawMarker(slam_map, slam[0, 0], (0, 128, 0), cv2.MARKER_TILTED_CROSS, 64, 4)
        cv2.drawMarker(slam_map, slam[p1], (255, 0, 0), cv2.MARKER_STAR, 64, 4)
        trj_img = slam / "trj.png"
        cv2.imwrite(str(slam / "trj.png"), slam_map)
        print("Trajectory image saved to", slam / "trj.png")
        TRJ_COMPLETE_FLAG.touch()
