# 从正点原子 ATK-MC5640 例程（阿波罗 STM32F429，DCMI）提取 OV5640 寄存器表，生成 src/ov5640_rom.v
# 顺序同例程：init_cfg（上电配置） + rgb565_cfg（输出模式），保留例程的 RGB565 输出，只把尺寸改为 160x120、裁成 4:3
# 用法：python tools/gen_ov5640_rom.py
import pathlib, re, zipfile

BOARD = pathlib.Path(__file__).resolve().parent.parent
DOC = BOARD.parent.parent / '【正点原子】OV5640单目摄像头模块'
ZIP = next(p for p in DOC.rglob('阿波罗STM32F429开发板.zip') if 'DCMI）' in str(p.parent))

OVERRIDE = {
    0x3800: 0x00, 0x3801: 0xA8,   # 传感器 X 起点 168：1280x720(16:9) 两边各裁 168，得到 4:3 视野
    0x3804: 0x09, 0x3805: 0x97,   # 传感器 X 终点 2455
    0x3808: 0x00, 0x3809: 0xA0,   # 输出宽 160
    0x380A: 0x00, 0x380B: 0x78,   # 输出高 120
}


def table(src, name):
    body = re.search(name + r'\[\]\s*=\s*\{(.*?)\};', src, re.S).group(1)
    return [(int(r, 16), int(v, 16)) for r, v in re.findall(r'\{\s*0x([0-9A-Fa-f]+)\s*,\s*0x([0-9A-Fa-f]+)\s*\}', body)]


with zipfile.ZipFile(ZIP) as z:
    cfg = next(i for i in z.infolist() if i.filename.endswith('atk_mc5640_cfg.h'))
    src = z.read(cfg).decode('gbk', 'replace')

regs = table(src, 'atk_mc5640_init_cfg') + [(r, OVERRIDE.get(r, v)) for r, v in table(src, 'atk_mc5640_rgb565_cfg')]
assert all(any(r == k for r, _ in regs) for k in OVERRIDE)

out = ['`timescale 1ns/1ps',
       '// 由 tools/gen_ov5640_rom.py 生成（源：正点原子 atk_mc5640_cfg.h），改寄存器请改脚本后重新生成',
       '// rv = {寄存器地址[15:0], 数据[7:0]}，last = 最后一条',
       'module ov5640_rom (input wire [8:0] idx, output reg [23:0] rv, output wire last);',
       f'    assign last = idx == 9\'d{len(regs) - 1};',
       '    always @* case (idx)']
out += [f"        9'd{i}: rv = 24'h{r:04X}{v:02X};" for i, (r, v) in enumerate(regs)]
out += ["        default: rv = 24'h000000;", '    endcase', 'endmodule', '']
(BOARD / 'src' / 'ov5640_rom.v').write_text('\n'.join(out), encoding='utf-8')
print(f'{len(regs)} 条寄存器 -> src/ov5640_rom.v')
