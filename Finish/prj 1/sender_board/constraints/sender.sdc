create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]
# OV5640 PCLK：上板实测 10.8MHz（SW5 PCL 页），按 33MHz 约束，给 Sobel/帧差的组合逻辑留足余量
create_clock -name cam_pclk -period 30 -waveform {0 15} [get_ports {cam_pclk}]
set_false_path -from [get_clocks {clk}] -to [get_clocks {cam_pclk}]
set_false_path -from [get_clocks {cam_pclk}] -to [get_clocks {clk}]
set_false_path -from [get_ports {link_rx col[*] sw[*] cam_sda cam_vsync cam_href cam_d[*]}]
