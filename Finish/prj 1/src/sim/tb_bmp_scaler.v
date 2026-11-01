`timescale 1ns/1ps
// Scaler self-check: feeds a synthesized BMP pixel-area byte stream (with row padding) and compares every 640x480 output pixel
//   Expected value is computed independently from the closed-form formula: src col = floor((2i+1)*W/(2*img_w)), row likewise; letterbox is black
//   Source pixel encoding: R = row[7:0], G = col[7:0], B = {row[11:8], col[11:8]}, uniquely identifies each pixel
module tb_bmp_scaler;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg  [11:0] W, H;  reg b32;
    reg  start = 0;
    reg  push = 0;  reg [7:0] din = 0;
    wire [11:0] cnt;  wire empty, pop;  wire [7:0] dout;
    wire pv;  wire [23:0] prgb;  wire busy;
    reg  [8:0] wr_used = 0;

    byte_fifo #(.AW(11)) u_f (.clk(clk), .rst_n(rst_n), .clr(1'b0), .push(push), .din(din),
        .pop(pop), .dout(dout), .count(cnt), .empty(empty));
    bmp_scaler dut (.clk(clk), .rst_n(rst_n), .start(start), .abort(1'b0), .src_w(W), .src_h(H),
        .target_width(10'd640), .bpp32(b32), .fifo_empty(empty), .fifo_dout(dout), .fifo_pop(pop), .wr_used(wr_used),
        .pix_valid(pv), .pix_rgb(prgb), .busy(busy));

    // Random backpressure: wr_used occasionally exceeds the threshold
    always @(posedge clk) wr_used <= ($random & 15) == 0 ? 9'd450 : 9'd100;

    // ---------- byte producer ----------
    integer r, c, k, stride, bpp;
    reg [23:0] px;
    reg prod_done;
    task produce;
        begin
            prod_done = 0;
            bpp = b32 ? 4 : 3;
            stride = ((W * bpp) + 3) / 4 * 4;
            for (r = 0; r < H; r = r + 1)
                for (k = 0; k < stride; k = k + 1) begin
                    c = k / bpp;
                    px = {r[7:0], c[7:0], r[11:8], c[11:8]};       // {R,G,B}
                    @(posedge clk);
                    while (cnt >= 12'd2000) @(posedge clk);
                    push <= 1'b1;
                    if (c >= W)            din <= 8'hEE;           // row padding
                    else case (k % bpp)
                        0: din <= px[7:0];   1: din <= px[15:8];   2: din <= px[23:16];
                        default: din <= 8'hAA;                     // 32-bit A byte
                    endcase
                    @(posedge clk); push <= 1'b0;
                    if (($random & 3) == 0) @(posedge clk);         // irregular arrival
                end
            prod_done = 1;
        end
    endtask

    // ---------- checker ----------
    integer nout, errs, img_w, img_h, off_x, off_y, orow, ocol, sr, sc;
    reg [23:0] exp_px;
    always @(posedge clk) if (pv) begin
        orow = nout / 640;  ocol = nout % 640;
        if (orow < off_y || orow >= off_y + img_h || ocol < off_x || ocol >= off_x + img_w)
            exp_px = 24'd0;
        else begin
            sr = ((2*(orow-off_y)+1) * H) / (2*img_h);
            sc = ((2*(ocol-off_x)+1) * W) / (2*img_w);
            exp_px = {sr[7:0], sc[7:0], sr[11:8], sc[11:8]};
        end
        if (prgb !== exp_px) begin
            if (errs < 8) $display("  MISMATCH out(%0d,%0d) got %h exp %h", orow, ocol, prgb, exp_px);
            errs = errs + 1;
        end
        nout = nout + 1;
    end

    task run_case;
        input [11:0] w, h;  input b;
        begin
            W = w; H = h; b32 = b;  nout = 0; errs = 0;
            if (w * 3 >= h * 4) begin img_w = 640; img_h = (h*640 + w/2) / w; end
            else begin img_h = 480; img_w = (w*480 + h/2) / h; end
            if (img_h < 1) img_h = 1;  if (img_w < 1) img_w = 1;
            off_x = (640 - img_w) / 2;  off_y = (480 - img_h) / 2;
            @(posedge clk) start <= 1;  @(posedge clk) start <= 0;
            produce;
            wait (!busy);  repeat (10) @(posedge clk);
            $display("CASE %0dx%0d %0dbpp: fit %0dx%0d off(%0d,%0d) out_pixels=%0d errors=%0d %s",
                     w, h, b ? 32 : 24, img_w, img_h, off_x, off_y, nout, errs,
                     (nout == 307200 && errs == 0) ? "PASS" : "FAIL");
        end
    endtask

    initial begin
        repeat (5) @(posedge clk); rst_n = 1;
        run_case(640, 480, 0);      // same size: identity
        run_case(1280, 720, 0);     // 16:9 downscale, letterbox top/bottom
        run_case(300, 200, 0);      // non-4-aligned width (900B row + 0 padding), upscale
        run_case(301, 199, 0);      // odd width needs 1 padding byte per row
        run_case(480, 640, 1);      // portrait 32-bit, pillarbox left/right
        run_case(37, 23, 0);        // tiny-image upscale
        $finish;
    end
endmodule
