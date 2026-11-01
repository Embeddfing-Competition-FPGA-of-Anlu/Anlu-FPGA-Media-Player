`timescale 1ns/1ps
// OLED 仿真：整板 sender_top（时钟按 5MHz 缩放，I2C 仍 200kHz），testbench 充当 SSD1306：
//   解码 I2C（START/STOP/9 位一字节），检查地址 0x78、ACK 位主机释放 SDA、页寻址模式、每页 128 字节；
//   把显存按帧写成 oled_<n>.pgm（128x64），用 tools/pgm2png.py 看图
module tb_oled;
    reg clk = 0; always #100 clk = ~clk;                 // 5MHz
    reg [7:0] sw = 8'h00;
    wire [3:0] row; reg [3:0] col_r; wire [3:0] col = col_r;
    integer pressed = -1;                                // 硬件扫描号，-1 = 无
    always @* begin
        col_r = 4'hf;
        if (pressed >= 0 && row[pressed / 4] == 1'b0) col_r[pressed % 4] = 1'b0;
    end
    wire oled_scl, oled_sda, cam_scl, cam_sda;
    pullup (oled_scl); pullup (oled_sda); pullup (cam_scl); pullup (cam_sda);
    wire link_tx, vid_tx, sp, st; wire [7:0] led; wire [3:0] seg_sel; wire [7:0] seg_led; wire rst_c, pwdn;
    sender_top #(.CLK_HZ(5000000)) dut (
        .clk(clk), .row(row), .col(col), .seg_sel(seg_sel), .seg_led(seg_led), .sw(sw), .led(led),
        .link_tx(link_tx), .link_rx(1'b1), .cam_scl(cam_scl), .cam_sda(cam_sda), .cam_rst_n(rst_c), .cam_pwdn(pwdn),
        .cam_pclk(1'b0), .cam_vsync(1'b0), .cam_href(1'b0), .cam_d(8'h00), .vid_tx(vid_tx),
        .servo_pan(sp), .servo_tilt(st), .oled_scl(oled_scl), .oled_sda(oled_sda));

    // ---------------- SSD1306 模型 ----------------
    reg [7:0] fb [0:1023];
    integer nbit, nbyte, errs = 0, page = 0, colp = 0, frames = 0, data_bytes, last_cmd = -1, mode_ok = 0;
    reg [8:0] sh; reg in_tr = 0, is_data;
    always @(negedge oled_sda) if (oled_scl === 1'b1) begin in_tr = 1; nbit = 0; nbyte = 0; data_bytes = 0; end
    always @(posedge oled_sda) if (oled_scl === 1'b1 && in_tr) begin
        in_tr = 0;
        if (!(nbit == 0 || (nbit == 1 && sh[0] == 1'b0))) begin errs = errs + 1; $display("ERR %t STOP mid-byte (bit %0d)", $time, nbit); end
        if (is_data && data_bytes != 128) begin errs = errs + 1; $display("ERR data transaction %0d bytes", data_bytes); end
        if (is_data && page == 7) begin dump(frames); frames = frames + 1; end
    end
    always @(posedge oled_scl) if (in_tr) begin
        sh = {sh[7:0], oled_sda === 1'b0 ? 1'b0 : 1'b1};
        nbit = nbit + 1;
        if (nbit == 9) begin
            nbit = 0;
            if (sh[0] !== 1'b1) begin errs = errs + 1; $display("ERR %t master drove SDA during ACK", $time); end
            byte_in(sh[8:1]);
            nbyte = nbyte + 1;
        end
    end
    task byte_in(input [7:0] b);
        begin
            if (nbyte == 0) begin if (b != 8'h78) begin errs = errs + 1; $display("ERR address %h", b); end end
            else if (nbyte == 1) is_data = (b == 8'h40);
            else if (is_data) begin fb[page*128 + colp] = b; colp = (colp + 1) % 128; data_bytes = data_bytes + 1; end
            else begin
                if (last_cmd == 8'h20 && b == 8'h02) mode_ok = 1;
                if (nbyte >= 2 && b >= 8'hB0 && b <= 8'hB7) page = b - 8'hB0;
                else if (nbyte == 3 && b < 8'h10) colp = b;
                else if (nbyte == 4 && b >= 8'h10 && b < 8'h20) colp = colp + (b - 8'h10) * 16;
                last_cmd = b;
            end
        end
    endtask
    integer fd, i, x, y;
    reg [8*20:1] fname;
    task dump(input integer n);
        begin
            $sformat(fname, "oled_%0d.pgm", n);
            fd = $fopen(fname, "w");
            $fwrite(fd, "P2\n128 64\n1\n");
            for (y = 0; y < 64; y = y + 1) begin
                for (x = 0; x < 128; x = x + 1) $fwrite(fd, "%0d ", fb[(y/8)*128 + x][y%8]);
                $fwrite(fd, "\n");
            end
            $fclose(fd);
        end
    endtask

    // ---------------- 场景 ----------------
    task press(input integer key);                       // 键号（操作者视角）
        begin pressed = 15 - key; #(60_000_000); pressed = -1; #(40_000_000); end
    endtask
    task snap(input [8*24:1] what);                      // 等两帧再记下帧号
        begin #(110_000_000); $display("frame %0d : %0s", frames - 1, what); end
    endtask
    initial begin
        for (i = 0; i < 1024; i = i + 1) fb[i] = 8'h55;
        #(1_000_000);
        snap("DATA idle");
        press(0); press(0); snap("DATA after VOL+ x2");
        press(13); snap("DATA limit edited");
        press(7); snap("DATA fault on");
        sw = 8'h40; #(1_000_000); press(1); snap("VIS edge");
        sw = 8'h20; snap("CAM diag");
        sw = 8'h80; press(9); snap("TEST key 9");
        $display("frames=%0d errors=%0d page_mode=%0d", frames, errs, mode_ok);
        $finish;
    end
endmodule
