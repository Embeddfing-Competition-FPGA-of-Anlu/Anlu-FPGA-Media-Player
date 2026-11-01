"""把仿真输出的 128x64 OLED 显存（P2 PGM）放大成 PNG：python pgm2png.py oled_3.pgm [...]"""
import sys
from PIL import Image

for path in sys.argv[1:]:
    tok = open(path).read().split()
    w, h = int(tok[1]), int(tok[2])
    px = [int(v) for v in tok[4:4 + w * h]]
    img = Image.new("RGB", (w, h))
    img.putdata([(80, 200, 255) if v else (0, 0, 0) for v in px])
    img.resize((w * 4, h * 4), Image.NEAREST).save(path.rsplit(".", 1)[0] + ".png")
