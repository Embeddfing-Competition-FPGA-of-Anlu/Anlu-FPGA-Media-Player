module video_timing_data
#(
	parameter DYNAMIC_MODE=0,
	parameter DATA_WIDTH = 16                       // Video data one clock data width
)
(
	input wire mode_800,
	input                       video_clk,          // Video pixel clock
	input                       rst,
	output reg                  read_req,           // Start reading a frame of data     
	input                       read_req_ack,       // Read request response
	//output                      read_en,            // Read data enable
	//input[DATA_WIDTH - 1:0]     read_data,          // Read data
	output                      hs,                 // horizontal synchronization
	output                      vs,                 // vertical synchronization
	output                      de                  // video valid
	//output[DATA_WIDTH - 1:0]    vout_data           // video data
);
wire                   video_hs;
wire                   video_vs;
wire                   video_de;
//delay video_hs video_vs  video_de 2 clock cycles
reg                    video_hs_d0;
reg                    video_vs_d0;
reg                    video_de_d0;
reg                    video_hs_d1;
reg                    video_vs_d1;
reg                    video_de_d1;

//reg[DATA_WIDTH - 1:0]  vout_data_r;
//assign read_en = video_de;
assign hs = video_hs_d1;
assign vs = video_vs_d1;
assign de = video_de_d1;
//assign vout_data = vout_data_r;
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		video_hs_d0 <= 1'b0;
		video_vs_d0 <= 1'b0;
		video_de_d0 <= 1'b0;
	end
	else
	begin
		//delay video_hs video_vs  video_de 2 clock cycles
		video_hs_d0 <= video_hs;
		video_vs_d0 <= video_vs;
		video_de_d0 <= video_de;
		video_hs_d1 <= video_hs_d0;
		video_vs_d1 <= video_vs_d0;
		video_de_d1 <= video_de_d0;		
	end
end
/*
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		vout_data_r <= {DATA_WIDTH{1'b0}};
	else if(video_de_d0)
		vout_data_r <= read_data;
	else
		vout_data_r <= {DATA_WIDTH{1'b0}};
end
*/
generate if(!DYNAMIC_MODE)begin
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		read_req <= 1'b0;
	else if(video_vs_d0 & ~video_vs) //vertical synchronization edge (the rising or falling edges are OK)
		read_req <= 1'b1;
	else if(read_req_ack)
		read_req <= 1'b0;
end

color_bar color_bar_m0(
	.clk(video_clk),
	.rst(rst),
	.hs(video_hs),
	.vs(video_vs),
	.de(video_de),
	.rgb_r(),
	.rgb_g(),
	.rgb_b()
);
end else begin
 wire request;
 video_mode_timing u_mode_timing(.clk(video_clk),.rst(rst),.mode_800(mode_800),
  .hs(video_hs),.vs(video_vs),.de(video_de),.frame_request(request),.frame_ack(read_req_ack));
 always @*read_req=request;
 end endgenerate
endmodule 
// 640x480 / 800x600 时序发生器（原在 video_output_mode.v；顶层 mode_800 接 0，只用 640x480）

module video_mode_timing(
 input wire clk,rst,mode_800,
 output wire hs,vs,de,
 output reg frame_request,
 input wire frame_ack
);
 reg [10:0] x;reg [9:0] y;
 wire [10:0] ht=mode_800?1056:800;
 wire [9:0] vt=mode_800?628:525;
 wire [10:0] w=mode_800?800:640;
 wire [9:0] h=mode_800?600:480;
 wire [10:0] hbegin=mode_800?840:656,hend=mode_800?968:752;
 wire [9:0] vbegin=mode_800?601:490,vend=mode_800?605:492;
 assign hs=((x>=hbegin)&&(x<hend))?mode_800:!mode_800;
 assign vs=((y>=vbegin)&&(y<vend))?mode_800:!mode_800;
 assign de=!rst && x<w && y<h;
 always @(posedge clk or posedge rst)
  if(rst)begin x<=0;y<=0;frame_request<=0;end
  else begin
   if(x==ht-1)begin x<=0;y<=y==vt-1?0:y+1'b1;end else x<=x+1'b1;
   if(x==0 && y==vend)frame_request<=1;
   else if(frame_ack)frame_request<=0;
  end
endmodule
