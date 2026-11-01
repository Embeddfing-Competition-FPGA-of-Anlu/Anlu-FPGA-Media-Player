`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: anlgoic
// Author: 	xg 
// description:  top module
//////////////////////////////////////////////////////////////////////////////////

`define DEBUG

`include "../user_source/hdl_source/include/global_def.v"

module sdram(
    `ifdef SIMULATION
	output					SDRAM_CLK,
	output					SDR_CKE,
	output					SDR_RAS,
	output					SDR_CAS,
	output					SDR_WE,
	output	[`BA_WIDTH-1:0]	SDR_BA,
	output	[`ROW_WIDTH-1:0]SDR_ADDR,
	output	[`DM_WIDTH-1:0]	SDR_DM,
	inout	[`DATA_WIDTH-1:0]	SDR_DQ,	
	`endif
    input   					Clk,
    input						Clk_sft,
    input						Rst,
    
    output 						Sdr_init_done,
    output						Sdr_init_ref_vld,
    output						Sdr_busy,
    
    input						App_wr_en,
	input [`ADDR_WIDTH-1:0]		App_wr_addr,  	
	input [`DM_WIDTH-1:0]		App_wr_dm,
	input [`DATA_WIDTH-1:0]		App_wr_din,	//synthesis keep
    
    input 						App_rd_en,
	input [`ADDR_WIDTH-1:0]		App_rd_addr,
	output 						Sdr_rd_en,
	output [`DATA_WIDTH-1:0]	Sdr_rd_dout	//synthesis keep
    
    
    
);
`ifndef SIMULATION
wire  			SDRAM_CLK;
wire 			SDR_RAS;
wire  			SDR_CAS;
wire  			SDR_WE;
wire  [`BA_WIDTH-1:0]	SDR_BA; 
wire  [`ROW_WIDTH-1:0]	SDR_ADDR;
wire  [`DATA_WIDTH-1:0]	SDR_DQ ;
wire  [`DM_WIDTH-1:0]	SDR_DM; 
`endif

// 噪点修复第二步：刷新准入窗口（参考米醋参考工程 sdram_refresh_admission 的思路，按本工程 256 突发重算）
// 修好刷新间隔宏以后，控制器每 3906 拍真的会插入一次自刷新。上游读写调度一旦开始突发就连续 256 拍，
// 若突发跨过刷新点，命令会和刷新冲突。这里用一个与厂商 init_cnt 同步的计数器，
// 在刷新点前后各留保护窗，窗内 Sdr_busy=1，上游只在 busy=0 时启动新突发，已开始的突发不拆。
wire controller_busy;
sdram_refresh_admission u_refresh_admission (
    .clk(Clk), .rst(Rst), .init_done(Sdr_init_done),
    .controller_busy(controller_busy), .app_busy(Sdr_busy));

sdr_as_ram  #( .self_refresh_open(1'b1))
	u2_ram(
		.Sdr_clk(Clk),
		.Sdr_clk_sft(Clk_sft),
		.Rst(Rst),

		.Sdr_init_done(Sdr_init_done),
		.Sdr_init_ref_vld(Sdr_init_ref_vld),
		.Sdr_busy(controller_busy),
		
		.App_ref_req(1'b0),
		
        .App_wr_en(App_wr_en), 
        .App_wr_addr(App_wr_addr),  	
		.App_wr_dm(App_wr_dm),
		.App_wr_din(App_wr_din),

		.App_rd_en(App_rd_en),
		.App_rd_addr(App_rd_addr),
		.Sdr_rd_en	(Sdr_rd_en),
		.Sdr_rd_dout(Sdr_rd_dout),
	
		.SDRAM_CLK(SDRAM_CLK),
		.SDR_RAS(SDR_RAS),
		.SDR_CAS(SDR_CAS),
		.SDR_WE(SDR_WE),
		.SDR_BA(SDR_BA),
		.SDR_ADDR(SDR_ADDR),
		.SDR_DM(SDR_DM),
		.SDR_DQ(SDR_DQ)	
	);


assign SDR_CKE=1'b1;

//`ifndef SIMULATION
	EG_PHY_SDRAM_2M_32 sdram(
		.clk(SDRAM_CLK),
		.ras_n(SDR_RAS),
		.cas_n(SDR_CAS),
		.we_n(SDR_WE),
		.addr(SDR_ADDR[10:0]),
		.ba(SDR_BA),
		.dq(SDR_DQ),
		.cs_n(1'b0),
		.dm0(SDR_DM[0]),
		.dm1(SDR_DM[1]),
		.dm2(SDR_DM[2]),
		.dm3(SDR_DM[3]),
		.cke(1'b1)
		);
//`endif

endmodule

// 厂商 sdr_init_ref：init_cnt 在 !init_done 时清零，到 SELF_REFRESH_INTERVAL 回零（周期 INTERVAL+1），
// 在 init_cnt == INTERVAL-10 时发 ref_req，刷新本身约 20 拍。本计数器复位条件与回绕点完全相同，两者同拍对齐。
// 保护窗：刷新请求前 MAX_BURST+余量 拍不再准入新突发；刷新后再留 POST 拍。
module sdram_refresh_admission (
    input  wire clk,
    input  wire rst,
    input  wire init_done,
    input  wire controller_busy,
    output wire app_busy
);
    localparam integer INTERVAL  = `SELF_REFRESH_INTERVAL;
    localparam integer REQ_AT    = INTERVAL - 10;
    localparam integer MAX_BURST = 256 + 16;          // 256 字突发 + 读侧 10 拍收尾
    localparam integer PRE_GUARD = MAX_BURST + 32;
    localparam integer POST_GUARD = 48;               // ref_req -> ack -> 20 拍 ref_vld
    reg [13:0] slot;
    reg        blocked;
    always @(posedge clk) begin
        if (rst || !init_done)
            slot <= 14'd0;
        else if (slot == INTERVAL)
            slot <= 14'd0;
        else
            slot <= slot + 1'b1;
    end
    // 窗口：[REQ_AT-PRE_GUARD, INTERVAL] ∪ [0, POST_GUARD-10)。REQ_AT 离回绕点只有 10 拍，
    // 所以刷新后的保护段落在回绕之后。寄存一拍（窗口早一拍/晚一拍都被余量覆盖）
    always @(posedge clk) begin
        if (rst || !init_done)
            blocked <= 1'b1;
        else
            blocked <= (slot >= REQ_AT - PRE_GUARD) || (slot < POST_GUARD - 10);
    end
    assign app_busy = controller_busy || blocked;
endmodule
