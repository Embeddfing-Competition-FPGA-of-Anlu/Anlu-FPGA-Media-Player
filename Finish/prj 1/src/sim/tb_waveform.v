`timescale 1ns/1ps

// 波形仿真：440Hz 正弦，先大音量（30000）8 屏，再小音量（2000，约 -24dB）40 屏
//   检查 1：同一音量下，相邻两屏前 32 点差 <= 4 像素（过零触发后相位固定）
//   检查 2：幅度不超 ±30（不冲框）
//   检查 3：自动增益稳定后，大小音量的峰值都在 14~30 像素（波形明显）
module tb_waveform;
    reg clk = 0, rst_n = 0, vsync = 1, pv = 0;
    reg signed [15:0] pcm = 0;
    always #20 clk = ~clk;                        // 25MHz

    promax_audio_waveform dut (
        .clk(clk), .rst_n(rst_n), .vsync(vsync), .de(1'b0), .rgb_in(24'd0),
        .pixel_x(10'd0), .pixel_x_pre(10'd0), .enable(1'b1),
        .pcm_tap_valid(pv), .pcm_tap_left(pcm), .rgb_out(), .waveform_active());

    real ph = 0.0, amp = 30000.0;
    integer sc = 0;
    always @(posedge clk) begin
        pv <= 1'b0;
        sc <= sc + 1;
        if (sc == 520) begin
            sc <= 0;  pv <= 1'b1;
            pcm <= $rtoi(amp * $sin(ph));
            ph = ph + 2.0 * 3.14159265 * 440.0 / 48077.0;
        end
    end

    integer fc = 0;
    always @(posedge clk) begin
        fc <= (fc == 416666) ? 0 : fc + 1;
        vsync <= !(fc < 100);
    end

    integer i, f, maxd, maxv, errs;
    reg signed [7:0] prev [0:31];
    reg signed [7:0] v;
    reg last_bank;
    initial begin
        errs = 0;
        #100 rst_n = 1;
        last_bank = 1'b1;
        for (f = 0; f < 48; f = f + 1) begin
            if (f == 8) amp = 2000.0;
            @(posedge clk);
            wait (dut.dbank != last_bank && dut.dvalid);
            last_bank = dut.dbank;
            #100;
            maxd = 0;  maxv = 0;
            for (i = 0; i < 512; i = i + 1) begin
                v = dut.mem[{dut.dbank, i[8:0]}];
                if (v > maxv) maxv = v;
                if (-v > maxv) maxv = -v;
                if (i < 32) begin
                    if (v - prev[i] > maxd) maxd = v - prev[i];
                    if (prev[i] - v > maxd) maxd = prev[i] - v;
                    prev[i] = v;
                end
            end
            if (f < 3 || (f >= 6 && f <= 10) || f >= 44)
                $display("swap %0d amp %0d: gain shift %0d, max |v| %0d, diff vs last %0d",
                         f, $rtoi(amp), 5 + dut.gsh, maxv, maxd);
            if (maxv > 30) errs = errs + 1;
            // 相位稳定：跳过刚开机、换音量后增益调整中的几屏
            if ((f >= 2 && f < 8 || f >= 44) && maxd > 4) errs = errs + 1;
            if ((f >= 3 && f < 8 || f >= 44) && maxv < 14) begin
                errs = errs + 1;  $display("  amplitude too small at swap %0d", f);
            end
        end
        if (errs == 0) $display("WAVEFORM PASS");
        else           $display("WAVEFORM FAIL errs=%0d", errs);
        $finish;
    end
endmodule
