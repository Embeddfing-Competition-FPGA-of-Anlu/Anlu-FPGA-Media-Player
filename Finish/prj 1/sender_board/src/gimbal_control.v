`timescale 1ns/1ps
// Fixed camera / pointing-arrow gimbal. No visual feedback drives a servo.
// State 5 means command position + dwell completed; it is NOT measured arrival.
// States: 0 off, 1 idle, 2 point, 3 dwell, 4 patrol, 5 review, 6 confirmed, 7 return,
//         8 failed, 9 manual takeover. Errors: 1 link, 5 limits, 6 timeout, 7 uncalibrated.
// Priority: direction keys > board alarm (!G from the display board) > local intrusion > patrol.
//   Patrol (patrol_en): while idle, point at calibrated stations A -> B -> C, hold PATROL_HOLD_MS each.
//   Intrusion (intr_zone, debounced per-station motion from the camera): point at that station and wait
//     for confirm; after confirm and the zone has gone quiet, return to centre and resume patrol.
//     Local tasks do not depend on the board link, so they work with the display board unplugged.
//   Manual / failed states release back to idle after RELEASE_MS without keys (no board alarm active).
module gimbal_control #(
    parameter integer PAN_MIN=30, PAN_MAX=150, TILT_MIN=45, TILT_MAX=135,
    parameter integer STEP_MS=20, SETTLE_MS=400,
    parameter integer LINK_TIMEOUT_MS=2000, TASK_TIMEOUT_MS=15000,
    parameter integer PATROL_HOLD_MS=3000, RELEASE_MS=10000
)(
    input wire clk, rst_n, ms_tick, enable,
    input wire command_valid, alarm,
    input wire [2:0] tool,
    input wire pan_minus, pan_plus, tilt_minus, tilt_plus, center, confirm,
    input wire save_preset,
    input wire [2:0] calibration_tool,
    input wire patrol_en,
    input wire [2:0] intr_zone,                 // bit0 = A, bit1 = B, bit2 = C
    output reg [7:0] pan, tilt,
    output reg [3:0] state, error,
    output reg [2:0] task_tool,
    output reg [3:0] calibrated,
    output wire moving,                         // servo command changing (camera holds off intrusion checks)
    output reg intr_task                        // current task came from local intrusion, not the board
);
    localparam [3:0] OFF=0, IDLE=1, POINT=2, DWELL=3, PATROL=4, REVIEW=5,
                     CONFIRMED=6, RETURN=7, FAILED=8, MANUAL=9;
    reg [7:0] preset_pan[0:3], preset_tilt[0:3];
    reg [7:0] goal_pan, goal_tilt, step_ms;
    reg [15:0] link_ms, task_ms, dwell_ms, rel_ms;
    reg returning, live_alarm, phold;
    reg [1:0] pidx;
    integer i;
    wire manual_key=pan_minus || pan_plus || tilt_minus || tilt_plus || center;
    assign moving=state==POINT || state==RETURN || (state==PATROL && !phold);
    wire busy=state==POINT || state==RETURN || state==DWELL;
    wire task_active=busy || state==REVIEW || state==CONFIRMED;
    wire in_limits=pan>=PAN_MIN && pan<=PAN_MAX && tilt>=TILT_MIN && tilt<=TILT_MAX;
    // Next calibrated patrol station after pidx (A/B/C only; tool 4 = whole view is not patrolled).
    wire [2:0] cal3=calibrated[2:0];
    reg [1:0] pnext;
    always @* case(pidx)
        2'd0: pnext=cal3[1] ? 2'd1 : cal3[2] ? 2'd2 : 2'd0;
        2'd1: pnext=cal3[2] ? 2'd2 : cal3[0] ? 2'd0 : 2'd1;
        default: pnext=cal3[0] ? 2'd0 : cal3[1] ? 2'd1 : 2'd2;
    endcase
    // Intrusion only on calibrated stations; lowest station wins.
    wire [2:0] izone=intr_zone & cal3;
    wire [1:0] iz=izone[0] ? 2'd0 : izone[1] ? 2'd1 : 2'd2;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) begin
            pan<=90; tilt<=90; goal_pan<=90; goal_tilt<=90;
            state<=OFF; error<=0; task_tool<=0; calibrated<=0; intr_task<=0;
            link_ms<=0; task_ms<=0; dwell_ms<=0; rel_ms<=0; step_ms<=0; returning<=0; live_alarm<=0;
            phold<=0; pidx<=2'd2;
            for(i=0;i<4;i=i+1) begin preset_pan[i]<=90; preset_tilt[i]<=90; end
        end else begin
            if(ms_tick) begin
                if(link_ms!=16'hffff) link_ms<=link_ms+1'b1;
                if(busy && task_ms!=16'hffff) task_ms<=task_ms+1'b1;
            end
            if(command_valid) link_ms<=0;
            if(manual_key || !(state==MANUAL || state==FAILED)) rel_ms<=0;
            else if(ms_tick && rel_ms!=16'hffff) rel_ms<=rel_ms+1'b1;
            if(save_preset && (state==OFF || state==IDLE || state==MANUAL || state==FAILED) &&
                calibration_tool>=1 && calibration_tool<=4 && in_limits) begin
                preset_pan[calibration_tool-1]<=pan; preset_tilt[calibration_tool-1]<=tilt;
                calibrated[calibration_tool-1]<=1;
            end
            // Direction keys always override automation. Existing manual 0..180 is retained.
            if(manual_key) begin
                if(pan_minus) pan<=pan>=5 ? pan-8'd5 : 8'd0;
                if(pan_plus) pan<=pan<=175 ? pan+8'd5 : 8'd180;
                if(tilt_minus) tilt<=tilt>=5 ? tilt-8'd5 : 8'd0;
                if(tilt_plus) tilt<=tilt<=175 ? tilt+8'd5 : 8'd180;
                if(center) begin pan<=90; tilt<=90; end
                state<=enable ? MANUAL : OFF; error<=0; intr_task<=0;
            end else if(!enable) begin
                state<=OFF; error<=0; live_alarm<=0; task_ms<=0; returning<=0; intr_task<=0;
            end else if(state==OFF) begin
                state<=IDLE; live_alarm<=0;
            end else if(command_valid && !alarm && live_alarm) begin
                live_alarm<=0;
                if(task_active) begin
                    goal_pan<=90; goal_tilt<=90; returning<=1; state<=RETURN;
                    task_ms<=0; step_ms<=0; error<=0;
                end else if(state==FAILED || state==MANUAL) begin state<=IDLE; error<=0; end
            end else if(command_valid && !alarm && (state==FAILED || state==MANUAL)) begin
                state<=IDLE; error<=0;
            end else if(command_valid && alarm && tool>=1 && tool<=4 && state!=MANUAL &&
                        (!live_alarm || task_tool!=tool)) begin
                // Board alarm preempts patrol and local intrusion tasks.
                live_alarm<=1; intr_task<=0; returning<=0; task_tool<=tool; task_ms<=0; step_ms<=0; error<=0;
                if(!calibrated[tool-1]) begin state<=FAILED; error<=7; end
                else if(!in_limits) begin state<=FAILED; error<=5; end
                else begin
                    goal_pan<=preset_pan[tool-1]; goal_tilt<=preset_tilt[tool-1]; state<=POINT;
                end
            end else if(task_active && !intr_task && !command_valid && link_ms>=LINK_TIMEOUT_MS) begin
                state<=FAILED; error<=1;
            end else if(busy && task_ms>=TASK_TIMEOUT_MS) begin
                state<=FAILED; error<=6;
            end else if(!live_alarm && (state==MANUAL || state==FAILED) && rel_ms>=RELEASE_MS) begin
                state<=IDLE; error<=0; intr_task<=0;
            end else if(!live_alarm && (state==IDLE || state==PATROL) && izone!=0) begin
                intr_task<=1; task_tool<={1'b0, iz} + 3'd1; returning<=0; task_ms<=0; step_ms<=0; error<=0;
                goal_pan<=preset_pan[iz]; goal_tilt<=preset_tilt[iz]; state<=POINT;
            end else if(intr_task && state==CONFIRMED && !intr_zone[task_tool-1]) begin
                intr_task<=0; goal_pan<=90; goal_tilt<=90; returning<=1; state<=RETURN; task_ms<=0; step_ms<=0;
            end else if(state==PATROL && !patrol_en) begin
                state<=IDLE;
            end else if(state==IDLE && patrol_en && cal3!=0 && !live_alarm) begin
                pidx<=pnext; task_tool<={1'b0, pnext} + 3'd1;
                goal_pan<=preset_pan[pnext]; goal_tilt<=preset_tilt[pnext]; phold<=0; step_ms<=0; dwell_ms<=0;
                state<=PATROL;
            end else case(state)
                POINT, RETURN, PATROL: if(ms_tick) begin
                    if(state==PATROL && phold) begin
                        if(dwell_ms>=PATROL_HOLD_MS-1) begin
                            pidx<=pnext; task_tool<={1'b0, pnext} + 3'd1;
                            goal_pan<=preset_pan[pnext]; goal_tilt<=preset_tilt[pnext]; phold<=0; step_ms<=0;
                        end else dwell_ms<=dwell_ms+1'b1;
                    end else if(pan==goal_pan && tilt==goal_tilt) begin
                        dwell_ms<=0;
                        if(state==PATROL) phold<=1; else state<=DWELL;
                    end else if(step_ms==STEP_MS-1) begin
                        step_ms<=0;
                        if(pan<goal_pan) pan<=pan+1'b1; else if(pan>goal_pan) pan<=pan-1'b1;
                        if(tilt<goal_tilt) tilt<=tilt+1'b1; else if(tilt>goal_tilt) tilt<=tilt-1'b1;
                    end else step_ms<=step_ms+1'b1;
                end
                DWELL: if(ms_tick) begin
                    if(dwell_ms==SETTLE_MS-1) begin
                        if(returning) begin state<=IDLE; returning<=0; end
                        else state<=REVIEW;
                    end else dwell_ms<=dwell_ms+1'b1;
                end
                REVIEW: if(confirm) state<=CONFIRMED;
                default: ;
            endcase
        end
endmodule
