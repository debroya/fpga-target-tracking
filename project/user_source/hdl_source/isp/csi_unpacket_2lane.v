/*******************************MILIANKE*******************************
*Company : MiLianKe Electronic Technology Co., Ltd.
*WebSite:https://www.milianke.com
*TechWeb:https://www.uisrc.com
*tmall-shop:https://milianke.tmall.com
*jd-shop:https://milianke.jd.com
*taobao-shop1: https://milianke.taobao.com
*Create Date: 2023/03/23
*Module Name:
*File Name:
*Description: 
*The reference demo provided by Milianke is only used for learning. 
*We cannot ensure that the demo itself is free of bugs, so users 
*should be responsible for the technical problems and consequences
*caused by the use of their own products.
*Copyright: Copyright (c) MiLianKe
*All rights reserved.
*Revision: 1.0
*Signal description
*1) I_ input
*2) O_ output
*3) IO_ input output
*4) S_ system internal signal
*5) _n activ low
*6) _dg debug signal 
*7) _r delay or register
*8) _s state mechine
*********************************************************************/
/*********csi_unpacket_2lane MIPI CSI***********

*********************************************************************/

`timescale 1ns / 1ns

module csi_unpacket_2lane 
(
input wire          I_clk,  
input wire          I_rst_n,

input wire          I_hs_valid,//synthesis keep 
input wire[15: 0]   I_hs_data, //synthesis keep 

output reg           O_csi_frame_start, 
output reg           O_csi_frame_end,   
output reg           O_csi_valid, //synthesis keep    
output reg           O_csi_hs,
output reg  [31: 0]  O_csi_data   //synthesis keep     
);




localparam FRAME_DATA_DT  = 8'h2B;   //RAW10
//localparam FRAME_DATA_DT  = 8'h2A;     //RAW8
localparam FRAME_START_DT = 8'h00;
localparam FRAME_END_DT   = 8'h01;

reg        S_hs_valid_1d;      //synthesis keep   
reg [15:0] S_hs_data_1d;       //synthesis keep   
wire       S_head_en;          //synthesis keep   
reg        S_head_en_1d;       //synthesis keep      
reg [15:0] S_data_length;      //synthesis keep      
reg        S_data_en;          //synthesis keep        
reg [15:0] S_data_en_cnt;      //synthesis keep      

assign  S_head_en  = ~S_hs_valid_1d & I_hs_valid;


always @(posedge I_clk) begin 
    S_hs_valid_1d        <= I_hs_valid;
    S_hs_data_1d         <= I_hs_data;
    S_head_en_1d         <= S_head_en;
end


always @(posedge I_clk) 
    O_csi_frame_start <= (S_head_en_1d && S_hs_data_1d[15:8] == FRAME_START_DT);



always @(posedge I_clk) 
    O_csi_frame_end  <= (S_head_en_1d && S_hs_data_1d[15:8] == FRAME_END_DT);


always @(posedge I_clk) begin 
    if(I_hs_valid)begin
        if(S_head_en_1d && S_hs_data_1d[15:8] == FRAME_DATA_DT)
            S_data_length <= {I_hs_data[15:8],S_hs_data_1d[7:0]} >> 1;
        else    
            S_data_length <= S_data_length;
    end
    else    
        S_data_length <= 'd0;
end



always @(posedge I_clk or negedge I_rst_n) begin
    if(!I_rst_n)
        S_data_en <= 1'b0;
    else    
        if(S_head_en_1d && S_hs_data_1d[15:8] == FRAME_DATA_DT)
            S_data_en <= 1'b1;
        else if(S_data_en_cnt == S_data_length-1)
            S_data_en <= 1'b0;
        else    
            S_data_en <= S_data_en;
end


always @(posedge I_clk) begin
    if(S_data_en)
        S_data_en_cnt <= S_data_en_cnt + 'd1;
    else
        S_data_en_cnt <= 'd0;
end


reg  csi_valid_r;//synthesis keep

always @(posedge I_clk) begin
    csi_valid_r   <= S_data_en ? ~csi_valid_r : 0; 
    O_csi_valid   <= csi_valid_r;
    O_csi_hs      <= I_hs_valid; 
end

always @(posedge I_clk) begin
    O_csi_data    <= S_data_en ? {O_csi_data[15:0],I_hs_data} : 0;
end

    
endmodule