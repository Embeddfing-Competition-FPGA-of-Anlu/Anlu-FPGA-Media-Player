`timescale 1ns/1ps

module config_journal (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        load_start,
    input  wire        slot0_present,
    input  wire [31:0] slot0_version,
    input  wire [31:0] slot0_period,
    input  wire [7:0]  slot0_volume,
    input  wire [7:0]  slot0_mode,
    input  wire [15:0] slot0_crc,
    input  wire        slot1_present,
    input  wire [31:0] slot1_version,
    input  wire [31:0] slot1_period,
    input  wire [7:0]  slot1_volume,
    input  wire [7:0]  slot1_mode,
    input  wire [15:0] slot1_crc,
    output reg         load_done,
    output reg         config_valid,
    output reg  [31:0] active_version,
    output reg  [31:0] active_period,
    output reg  [7:0]  active_volume,
    output reg  [7:0]  active_mode,
    input  wire        save_start,
    input  wire [31:0] save_period,
    input  wire [7:0]  save_volume,
    input  wire [7:0]  save_mode,
    output reg         record_ready,
    output reg         target_slot,
    output reg  [31:0] record_version,
    output reg  [31:0] record_period,
    output reg  [7:0]  record_volume,
    output reg  [7:0]  record_mode,
    output reg  [15:0] record_crc
);
    reg load_sr;
    function [15:0] crc16_word;
        input [79:0] payload;
        integer i;
        reg [15:0] crc;
        begin
            crc = 16'hffff;
            for (i = 79; i >= 0; i = i - 1) begin
                if (crc[15] ^ payload[i])
                    crc = {crc[14:0],1'b0} ^ 16'h1021;
                else
                    crc = {crc[14:0],1'b0};
            end
            crc16_word = crc;
        end
    endfunction

    function version_newer;
        input [31:0] candidate;
        input [31:0] reference;
        reg [31:0] delta;
        begin
            delta = candidate - reference;
            version_newer = (candidate != reference) && !delta[31];
        end
    endfunction

    // Timing fix (100MHz): the reference project computed the 80-bit CRC as a combinational chain in one cycle
    // (that only closes at 50MHz). Changed to a bit-serial CRC: shift in one bit per cycle, done in 80 cycles. Load/save are both one-off events;
    // the outer state machine waits on load_done / record_ready, so the handshake is unchanged and the results are identical to the original.
    function [15:0] crc_step;
        input [15:0] c;
        input        b;
        crc_step = (c[15] ^ b) ? ({c[14:0],1'b0} ^ 16'h1021) : {c[14:0],1'b0};
    endfunction

    reg  [79:0] sh0, sh1;                 // load: two slots checked in parallel; save: sh0 reused for the new record
    reg  [15:0] c0, c1;
    reg  [6:0]  bit_cnt;
    reg         run_load, run_save, fin_load, fin_save;
    reg         slot0_good, slot1_good, newer_q;
    wire        pick0 = slot0_good && (!slot1_good || !newer_q);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            load_done<=0; config_valid<=0; active_version<=0; active_period<=0;
            active_volume<=0; active_mode<=0; record_ready<=0; target_slot<=0;
            record_version<=0; record_period<=0; record_volume<=0; record_mode<=0;
            record_crc<=0; sh0<=0; sh1<=0; c0<=16'hffff; c1<=16'hffff; bit_cnt<=0;
            run_load<=0; run_save<=0; fin_load<=0; fin_save<=0;
            slot0_good<=0; slot1_good<=0; newer_q<=0;
        end else begin
            load_done<=0;
            record_ready<=0;
            fin_load<=0;
            fin_save<=0;
            if (load_start) begin
                sh0 <= {slot0_version,slot0_period,slot0_volume,slot0_mode};
                sh1 <= {slot1_version,slot1_period,slot1_volume,slot1_mode};
                c0 <= 16'hffff;  c1 <= 16'hffff;  bit_cnt <= 7'd0;  run_load <= 1'b1;
                newer_q <= version_newer(slot1_version, slot0_version);
            end else if (save_start) begin
                record_version <= active_version + 1'b1;
                record_period  <= save_period;
                record_volume  <= save_volume;
                record_mode    <= save_mode;
                sh0 <= {active_version + 1'b1, save_period, save_volume, save_mode};
                c0 <= 16'hffff;  bit_cnt <= 7'd0;  run_save <= 1'b1;
            end else if (run_load || run_save) begin
                c0  <= crc_step(c0, sh0[79]);
                c1  <= crc_step(c1, sh1[79]);
                sh0 <= {sh0[78:0], 1'b0};
                sh1 <= {sh1[78:0], 1'b0};
                bit_cnt <= bit_cnt + 7'd1;
                if (bit_cnt == 7'd79) begin
                    fin_load <= run_load;  fin_save <= run_save;
                    run_load <= 1'b0;      run_save <= 1'b0;
                end
            end
            if (fin_load) begin
                slot0_good <= slot0_present && (slot0_crc == c0);
                slot1_good <= slot1_present && (slot1_crc == c1);
            end
            // fin_load one cycle later: good flags are registered, select the slot
            if (load_sr) begin
                load_done<=1;
                if(pick0) begin
                    config_valid<=1; active_version<=slot0_version; active_period<=slot0_period;
                    active_volume<=slot0_volume; active_mode<=slot0_mode;
                end else if(slot1_good) begin
                    config_valid<=1; active_version<=slot1_version; active_period<=slot1_period;
                    active_volume<=slot1_volume; active_mode<=slot1_mode;
                end else begin
                    config_valid<=0; active_version<=0; active_period<=0;
                    active_volume<=0; active_mode<=0;
                end
            end
            if (fin_save) begin
                // Target slot: overwrite the older one (same rule as the original; slot0_good/slot1_good come from the last load or save)
                target_slot  <= pick0 ? 1'b1 : 1'b0;
                record_crc   <= c0;
                record_ready <= 1'b1;
            end
        end
    end
    always @(posedge clk or negedge rst_n)
        if (!rst_n) load_sr <= 1'b0; else load_sr <= fin_load;
endmodule
