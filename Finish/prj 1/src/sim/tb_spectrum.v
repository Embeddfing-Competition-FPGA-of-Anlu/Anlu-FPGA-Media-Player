`timescale 1ns/1ps
// Spectrum self-check: feed 48kHz sine waves at known bins and check that the tallest bar lands on the corresponding bin, with low energy on other bars
//   bin_k table: bars 0..32 = bin 1..33 (one bar per bin), bin*93.75Hz
module tb_spectrum;
    reg clk = 0, rst_n = 0;
    always #20 clk = ~clk;                       // 25MHz
    reg sv = 0;  reg signed [15:0] smp = 0;
    reg ft = 0;
    reg [5:0] rd = 0;
    wire [6:0] h, pk, bass;
    spectrum_dft64 dut (.clk(clk), .rst_n(rst_n), .sample_valid(sv), .sample(smp), .frame_tick(ft),
        .rd_bar(rd), .rd_h(h), .rd_pk(pk), .bass(bass), .overrun());

    real ph;  real f;  integer n, i, best, bh, others, errs;
    integer hv [0:63];
    task feed;                                   // nblk blocks (512 samples each), plus a frame_tick every 128 samples
        input real freq;  input real amp;  input integer nblk;
        begin
            for (n = 0; n < nblk * 512; n = n + 1) begin
                smp = $rtoi(amp * $sin(ph));  ph = ph + 6.283185307 * freq / 48000.0;
                @(posedge clk) sv <= 1;  @(posedge clk) sv <= 0;
                repeat (518) @(posedge clk);
                if (n % 128 == 127) begin @(posedge clk) ft <= 1; @(posedge clk) ft <= 0; end
            end
        end
    endtask
    task readall;
        begin
            best = 0; bh = 0; others = 0;
            for (i = 0; i < 64; i = i + 1) begin
                @(posedge clk) rd <= i;  @(posedge clk); @(posedge clk); hv[i] = h;
                if (h > bh) begin bh = h; best = i; end
            end
            for (i = 0; i < 64; i = i + 1) if (i < best - 2 || i > best + 2) if (hv[i] > others) others = hv[i];
        end
    endtask
    task check;
        input real freq;  input integer exp_bar;  input real amp;
        begin
            rst_n = 0;  repeat (3) @(posedge clk);  rst_n = 1;     // each case starts from zero
            for (i = 0; i < 128; i = i + 1) begin dut.dsp_mem[i] = 0; if (i < 64) dut.st_mem[i] = 0; end   // RAM is not affected by reset; clear it manually
            ph = 0.0;  feed(freq, amp, 3);  readall;
            $display("tone %7.1fHz amp %6.0f: tallest bar=%0d height=%0d (expected bar %0d), max off-peak bar=%0d, bass=%0d %s",
                     freq, amp, best, bh, exp_bar, others, bass,
                     (best == exp_bar && bh > others + 20) ? "PASS" : "FAIL");
            if (!(best == exp_bar && bh > others + 20)) errs = errs + 1;
        end
    endtask
    initial begin
        errs = 0;
        repeat (5) @(posedge clk); rst_n = 1;
        check(375.0,  3, 20000.0);      // bin 4  -> bar 3
        check(1875.0, 19, 20000.0);     // bin 20 -> bar 19
        check(1875.0, 19, 2000.0);      // same frequency, -20dB: bar should be lower
        check(3093.75, 32, 30000.0);    // bin 33 -> bar 32
        // silence: all bars should decay
        // decay: about 1 second of silence after the last tone (100 blocks); bars should fall to near 0
        ph = 0.0; feed(1000.0, 0.0, 100); readall;
        $display("silence 100 blocks after tone: tallest bar height=%0d %s", bh, (bh < 8) ? "PASS" : "FAIL");
        if (bh >= 8) errs = errs + 1;
        $display("SPECTRUM %s", errs == 0 ? "ALL PASS" : "HAS FAIL");
        $finish;
    end
endmodule
