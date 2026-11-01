`timescale 1ns/1ps

// PICn selects MUSICn. Drain the old SD transaction before flushing both
// audio domains; natural EOF preserves queued PCM for the next loop.
module audio_play_ctrl #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer GAP_MS      = 20,
    parameter integer FIFO_AW     = 12
)(
    input              clk,
    input              rst_n,
    input wire         pause,
    input              preload_done,
    input              music_found,
    input      [1:0]   media_idx,
    input              display_valid,
    output reg [1:0]   audio_idx,
    output             audio_clear,
    input              audio_clear_ack,
    input              reader_ready,
    input              sd_ready,
    input              audio_done,
    input              audio_err,
    input              fmt_ok,
    input              fmt_error,
    input  [FIFO_AW:0] fifo_used,
    output reg         audio_start,
    output reg         audio_stop,
    output             audio_hold,
    output             wav_mode
);
    localparam [31:0] GAP_CYCLES = (CLK_FREQ_HZ / 1000) * GAP_MS;
    localparam [FIFO_AW:0] HOLD_LEVEL = (1 << FIFO_AW) - 640;
    localparam [2:0] IDLE = 3'd0, RUN = 3'd1, STOP = 3'd2,
                     CLEAR = 3'd3, RELEASE = 3'd4;
    reg [2:0] state;
    reg track_valid, reading, armed, disabled;
    reg [1:0] err_cnt;
    reg [31:0] gap;
    wire changed = display_valid && (!track_valid || media_idx != audio_idx);

    assign audio_hold = (fifo_used >= HOLD_LEVEL);
    assign audio_clear = (state == CLEAR);
    assign wav_mode = armed && fmt_ok && !disabled && !changed &&
                      state != STOP && state != CLEAR && state != RELEASE;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= CLEAR;
            audio_idx <= 2'd0;
            audio_start <= 1'b0;
            audio_stop <= 1'b0;
            track_valid <= 1'b0;
            reading <= 1'b0;
            armed <= 1'b0;
            disabled <= 1'b0;
            err_cnt <= 2'd0;
            gap <= 32'd0;
        end else begin
            audio_start <= 1'b0;
            audio_stop <= 1'b0;
            if (gap != 0) gap <= gap - 1'b1;
            // Use the latest actual display bank. Intentional stops do not
            // count toward the read-error limit.
            if (changed) begin
                audio_idx <= media_idx;
                track_valid <= 1'b1;
                armed <= 1'b0;
                disabled <= 1'b0;
                err_cnt <= 2'd0;
                gap <= 32'd0;
                if (reading && !audio_done) begin
                    audio_stop <= 1'b1;
                    state <= STOP;
                end else begin
                    reading <= 1'b0;
                    state <= CLEAR;
                end
            end else if(pause && state==RUN && !audio_done) begin
                audio_stop<=1;armed<=0;state<=STOP;
            end else begin
                case (state)
                    CLEAR: if (audio_clear_ack) state <= RELEASE;
                    RELEASE: if (!audio_clear_ack) state <= IDLE;
                    IDLE: if (track_valid && display_valid && !disabled && gap == 0 &&
                              !pause && preload_done && music_found && reader_ready && sd_ready) begin
                        audio_start <= 1'b1;
                        reading <= 1'b1;
                        armed <= 1'b1;
                        state <= RUN;
                    end
                    RUN: begin
                        if (audio_done) begin
                            reading <= 1'b0;
                            gap <= GAP_CYCLES;
                            if (audio_err || fmt_error || !fmt_ok) begin
                                armed <= 1'b0;
                                err_cnt <= (err_cnt == 2'd3) ? 2'd3 : err_cnt + 1'b1;
                                if (fmt_error || err_cnt >= 2'd2) disabled <= 1'b1;
                                state <= CLEAR;
                            end else begin
                                err_cnt <= 2'd0;
                                state <= IDLE;
                            end
                        end else if (fmt_error) begin
                            disabled <= 1'b1;
                            armed <= 1'b0;
                            audio_stop <= 1'b1;
                            state <= STOP;
                        end
                    end
                    STOP: if (audio_done) begin
                        reading <= 1'b0;
                        state <= CLEAR;
                    end
                    default: state <= CLEAR;
                endcase
            end
        end
    end
endmodule
