#!/bin/sh
# 用法：build.sh  —— 依次跑 syn_1 与 phy_1，输出关键结果
TD=/d/Anlogic/TD_6.2.1_Engineer_6.2.168.116/bin/td_commands_prompt.exe
FLOW="D:/Anlogic/TD_6.2.1_Engineer_6.2.168.116/doc/scripts/DefaultFlow.tcl"
R="$(cd "$(dirname "$0")" && pwd)/HDMI1.4b_Transmitter_v1.0_Runs"
cd "$R/syn_1" && rm -f .*.f && $TD "$FLOW" < /dev/null > /tmp/syn.log 2>&1
if grep -q "ERROR" /tmp/syn.log; then echo "SYN FAIL"; grep -n "ERROR" /tmp/syn.log | head -20; exit 1; fi
echo "SYN OK"
# TD 打开工程时会重写 settings.cfg：每次布局前补回布局参数（LUT ~85%，缺了会出布线短路）
P="$R/phy_1/settings.cfg"; sed -i '/^set phyWildParams/d' "$P"; echo 'set phyWildParams {place effort high place max_fanout 200}' >> "$P"
cd "$R/phy_1" && rm -f .*.f && $TD "$FLOW" < /dev/null > /tmp/phy.log 2>&1
if grep -q "ERROR" /tmp/phy.log; then echo "PHY FAIL"; grep -n "ERROR" /tmp/phy.log | head -20; exit 1; fi
echo "PHY OK"
grep -A12 "Utilization Statistics" "$R/phy_1/HDMI1.4b_Transmitter_v1.0_phy.area" | grep -i "lut\|reg\|dsp\|bram\|pad"
grep -i -m4 "WNS\|TNS" "$R/phy_1/HDMI1.4b_Transmitter_v1.0_phy.ts"
ls -la "$R/phy_1/HDMI1.4b_Transmitter_v1.0.bit"
# Hold fix: this project's arr_filter=false, so the default flow doesn't run fix_hold; run it separately on the routed database
cd "$R/phy_1" && /d/Anlogic/TD_6.2.1_Engineer_6.2.168.116/bin/td_commands_prompt.exe holdfix.tcl < /dev/null > /tmp/hold.log 2>&1
echo "after holdfix:"; grep -i "WNS" "$R/phy_1/HDMI1.4b_Transmitter_v1.0_phy.ts"
