`define   DATA_WIDTH                        32
`define   ADDR_WIDTH                        21
`define   DM_WIDTH                          4

`define   ROW_WIDTH                        11
`define   BA_WIDTH                        2

// 噪点修复：宏是文本替换，必须整体加括号。
// 原写法 64000000/`SDR_CLK_PERIOD/... 展开成 64000000/1000000000/125000000/2048 = 0，
// 刷新间隔被算成 0，SDRAM 刷新失控 -> 存储数据随机出错（屏幕随机噪点）。
// 正确值：125MHz 下 8ns/拍，64ms / 2048 行 = 31.25us = 3906 拍刷新一行。
`define	  SDR_CLK_PERIOD				(1000000000 / 125000000)
`define   SELF_REFRESH_INTERVAL			(64000000 / `SDR_CLK_PERIOD / (2 ** (`ROW_WIDTH)))
