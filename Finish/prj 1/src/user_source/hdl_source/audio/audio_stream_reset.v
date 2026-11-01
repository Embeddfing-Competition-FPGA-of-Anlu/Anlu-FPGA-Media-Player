`timescale 1ns/1ps

// Four-phase flush handshake. Both FIFO pointers and the PCM assembler reset
// together. Assertion is asynchronous; release is synchronized in each domain.
// clear_ack stays high until the read domain has released its local reset.
module audio_stream_reset (
    input  wire wr_clk,
    input  wire wr_rst_n,
    input  wire rd_clk,
    input  wire rd_rst_n,
    input  wire clear_req,
    output wire clear_ack,
    output wire fifo_wr_rst_n,
    output wire fifo_rd_rst_n
);
    wire reset_n = wr_rst_n && rd_rst_n && !clear_req;
    reg [1:0] wr_release, rd_release;
    // Match the project's cdc_*_s1 first-stage timing convention.
    reg cdc_req_s1, req_s2, req_s3;
    reg cdc_ack_s1, ack_s2;
    wire ack_rd = req_s3 || !fifo_rd_rst_n;

    always @(posedge wr_clk or negedge reset_n)
        if (!reset_n) wr_release <= 2'b00;
        else wr_release <= {wr_release[0], 1'b1};
    always @(posedge rd_clk or negedge reset_n)
        if (!reset_n) rd_release <= 2'b00;
        else rd_release <= {rd_release[0], 1'b1};
    assign fifo_wr_rst_n = wr_release[1];
    assign fifo_rd_rst_n = rd_release[1];

    // Synchronizers use global resets, never their own flush.
    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            cdc_req_s1 <= 1'b0; req_s2 <= 1'b0; req_s3 <= 1'b0;
        end else begin
            cdc_req_s1 <= clear_req; req_s2 <= cdc_req_s1; req_s3 <= req_s2;
        end
    end
    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            cdc_ack_s1 <= 1'b0; ack_s2 <= 1'b0;
        end else begin
            cdc_ack_s1 <= ack_rd; ack_s2 <= cdc_ack_s1;
        end
    end
    assign clear_ack = ack_s2;
endmodule
