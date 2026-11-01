#!/bin/sh
# 发送板综合 + 布局布线 + 比特流：sh build.sh
TD=/d/Anlogic/TD_6.2.1_Engineer_6.2.168.116/bin/td_commands_prompt.exe
FLOW="D:/Anlogic/TD_6.2.1_Engineer_6.2.168.116/doc/scripts/DefaultFlow.tcl"
R="$(cd "$(dirname "$0")" && pwd)/sender_Runs"
cd "$R/syn_1" && rm -f .*.f && $TD "$FLOW" < /dev/null > /tmp/sender_syn.log 2>&1
if grep -q "ERROR" /tmp/sender_syn.log; then echo "SYN FAIL"; grep -n "ERROR" /tmp/sender_syn.log | head; exit 1; fi
echo "SYN OK"
cd "$R/phy_1" && rm -f .*.f && $TD "$FLOW" < /dev/null > /tmp/sender_phy.log 2>&1
if grep -q "ERROR" /tmp/sender_phy.log; then echo "PHY FAIL"; grep -n "ERROR" /tmp/sender_phy.log | head; exit 1; fi
echo "PHY OK"
grep -A6 "Utilization Statistics" "$R/phy_1/sender_phy.area" | grep -i "lut\|reg"
cat "$R/phy_1/sender_phy.ts" 2>/dev/null
ls -la "$R/phy_1/sender.bit"
