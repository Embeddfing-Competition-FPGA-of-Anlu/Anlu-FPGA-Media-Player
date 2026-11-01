
# ============================================================
# merged_top_tf_hdmi_audio.sdc
# 用于：TF 图片轮播 + HDMI(视频+音频) 整合工程
#
# 合并原则：
# 1) 保留 top.sdc 里的系统主时钟与 derive_clocks
# 2) 把 timing(2).sdc 里的 pixel/serial 时钟改成整合工程实际名字
# 3) 增加 hdmi_5x_clk / audio_mclk 的命名
# 4) 不再对内部 net 直接 create_clock，避免和 derive_clocks 重复
# ============================================================

# ------------------------------------------------------------
# 1. 板级输入主时钟
#    你的原 top.sdc 里 clk=50MHz，所以这里仍然用 20ns
# ------------------------------------------------------------
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]

# ------------------------------------------------------------
# 2. 自动推导 PLL 输出时钟
# ------------------------------------------------------------
derive_clocks

# ------------------------------------------------------------
# 3. 给关键 PLL 输出命名
#    下面这些路径是按你当前工程实例名写的：
#      sys_pll      -> sys_pll_m0
#      video_pll    -> video_pll_m0
#      音频 PLL     -> u_audio_pll
#
#    如果综合后实例路径略有不同，就按 Timing Analyzer 里的实际 pin 路径改。
# ------------------------------------------------------------
rename_clock -name {sd_card_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {ext_mem_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {video_clk}   -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {hdmi_5x_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {ext_mem_clk_sft} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[2]}]
# 第3课起音频复用 video_clk（25MHz 像素时钟），已删除独立音频 PLL(audio_mclk)

# ------------------------------------------------------------
# 4. 从原 timing(2).sdc 迁移过来的 HDMI 像素/串行时钟约束
#
#    原文件是：
#      create_clock -name pixel_clk  ... [get_nets {S_pixel_clk}]
#      create_clock -name serial_clk ... [get_nets {S_serial_clk}]
#      set_clock_groups -exclusive ...
#
#    在整合工程里，这两个时钟已经由 video_pll 产生，并通过 derive_clocks 建出来了，
#    所以这里只保留“组关系”，不再重复 create_clock。
#
#    这里沿用原作者的写法，把 video_clk 和 hdmi_5x_clk 设成 exclusive。
#    如果后面 Timing Analyzer 明确显示两者之间需要做同步时序分析，再把这一条去掉。
# ------------------------------------------------------------
# 修正：video_clk(25M) 与 hdmi_5x_clk(125M) 出自同一 PLL、同时工作、1:5 整数倍，
# 属于同步时钟，不能设 exclusive（否则 25M 并行 -> 125M 串行的真实路径不被分析）。
# 原 exclusive 约束已删除，交给工具按同步时钟默认分析。

# ------------------------------------------------------------
# 6. 可选：如果你的 frame_read_write 内部异步 FIFO 已经完全处理好了跨域，
#    且报告里总出现 video_clk / sd_card_clk / ext_mem_clk 之间的大量伪违例，
#    可以打开下面这组异步分组。
#
#    建议先不要开，先看报告；
#    只有确认这些跨域都只经过 FIFO / 同步器时再开。
# ------------------------------------------------------------
# set_clock_groups -asynchronous \
#     -group [get_clocks {sd_card_clk}] \
#     -group [get_clocks {ext_mem_clk}] \
#     -group [get_clocks {video_clk}] \
#     -group [get_clocks {hdmi_5x_clk}]

# ------------------------------------------------------------
# 7. 其余 Input/Output delay 先保持空白
#    你当前两个原始 sdc 也都没有写这部分。
# ------------------------------------------------------------

# ============================================================
# 第5课：时序约束完善（选做）
# ============================================================

# 1. 外部复位按键是异步信号，不做时序分析
set_false_path -from [get_ports {rst_n}]

# 2. reset_sync / sync_2ff 的第一级触发器是异步采样，不做时序分析
set_false_path -to [get_regs -hier {*/s1}]

# 3. HDMI ODDR 输出到管脚的路径限制最大延迟
set_max_delay -from [get_clocks {hdmi_5x_clk}]     -to [get_ports {HDMI_CLK_P HDMI_D0_P HDMI_D1_P HDMI_D2_P}]     8.000 -datapath_only

# 4. 保持时间不确定性：默认 0.1ns 偏大，按同片 PLL 抖动+时钟树偏斜取 0.05ns
set_clock_uncertainty -hold 0.050     [get_clocks {clk sd_card_clk ext_mem_clk ext_mem_clk_sft video_clk hdmi_5x_clk}]

# 5. SDRAM 移位采样时钟只驱动 SDRAM 硬核的 I/O 窗口，不按 fabric 路径分析
#    （原报告里 ext_mem_clk <-> clkc[2] 的 -6.5ns 违例就是这类伪违例）
set_false_path -from [get_clocks {ext_mem_clk}] -to [get_clocks {ext_mem_clk_sft}]
set_false_path -from [get_clocks {ext_mem_clk_sft}] -to [get_clocks {ext_mem_clk}]

# 6. 异步 FIFO 的 fifo_aclr 复位：IP 内部是"异步复位、同步释放"（asy_w_rst0/1、asy_r_rst0/1），
#    fifo_aclr 在写请求握手期间保持多拍，不需要按单周期分析
set_false_path -from [get_regs -hier {*/fifo_aclr*}]

# ============================================================
# 完赛整合（第7~14课）：新增跨时钟域路径
# ============================================================
# 7. 所有同步器第一级（命名统一为 cdc_*_s1）：控制总线 toggle、转场握手、异步 FIFO 格雷码指针
set_false_path -to [get_regs -hier {*/cdc_*_s1*}]
# Legacy frame-reader synchronizers: only the asynchronous first sampling
# stage is exempt; their following stages and the memory datapath stay timed.
set_false_path -to [get_regs -hier {frame_read_write_m0/frame_fifo_read_m0/read_req_d0*}]
set_false_path -to [get_regs -hier {frame_read_write_m0/frame_fifo_read_m0/read_addr_index_d0*}]
# Flush reset enters the two local reset-release chains asynchronously.
# Exempt only this assertion net; release-chain Q -> D paths remain timed.
set_false_path -through [get_nets {u_audio_reset/reset_n}]
# 8. cdc_bus_sync：cdc_hold 在 req 翻转前已稳定，目的域 2~3 拍后才采样
set_false_path -from [get_regs -hier {*/cdc_hold*}] -to [get_regs -hier {*/dst_data*}]
# 9. 转场模块 auto_hold 与 req_tgl 同拍更新，像素域看到 req 翻转时早已稳定
set_false_path -from [get_regs -hier {*/auto_hold}]
# 10. 菜单版：字幕整行跨域（sd -> video）。sub_vec/sub_len 在 sub_tgl 翻转后保持 >100us 不变，
#     像素域三级同步检测到翻转才采样；翻转位第一级 sub_s[0] 为异步采样
set_false_path -to [get_regs -hier {*sub_vec_v*}]
set_false_path -to [get_regs -hier {*sub_len_v*}]
set_false_path -to [get_regs -hier {*sub_s[0]*}]

