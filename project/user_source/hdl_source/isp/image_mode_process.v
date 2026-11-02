module image_mode_process #(
    parameter [7:0] BINARY_TH = 8'd128
) (
    input  wire        I_tlast,
    input  wire        I_tuser,
    input  wire [95:0] I_tdata,
    input  wire        I_tvalid,
    input  wire        I_binary,
    output wire        O_tlast,
    output wire        O_tuser,
    output wire [95:0] O_tdata,
    output wire        O_tvalid
);

wire [23:0] S_pixel0 = I_tdata[95:72];
wire [23:0] S_pixel1 = I_tdata[71:48];
wire [23:0] S_pixel2 = I_tdata[47:24];
wire [23:0] S_pixel3 = I_tdata[23:0];

wire [7:0] S_gray0 = {2'b0, S_pixel0[23:18]} + {1'b0, S_pixel0[15:9]} + {2'b0, S_pixel0[7:2]};
wire [7:0] S_gray1 = {2'b0, S_pixel1[23:18]} + {1'b0, S_pixel1[15:9]} + {2'b0, S_pixel1[7:2]};
wire [7:0] S_gray2 = {2'b0, S_pixel2[23:18]} + {1'b0, S_pixel2[15:9]} + {2'b0, S_pixel2[7:2]};
wire [7:0] S_gray3 = {2'b0, S_pixel3[23:18]} + {1'b0, S_pixel3[15:9]} + {2'b0, S_pixel3[7:2]};

wire [7:0] S_out0 = I_binary ? ((S_gray0 >= BINARY_TH) ? 8'hff : 8'h00) : S_gray0;
wire [7:0] S_out1 = I_binary ? ((S_gray1 >= BINARY_TH) ? 8'hff : 8'h00) : S_gray1;
wire [7:0] S_out2 = I_binary ? ((S_gray2 >= BINARY_TH) ? 8'hff : 8'h00) : S_gray2;
wire [7:0] S_out3 = I_binary ? ((S_gray3 >= BINARY_TH) ? 8'hff : 8'h00) : S_gray3;

assign O_tlast  = I_tlast;
assign O_tuser  = I_tuser;
assign O_tvalid = I_tvalid;
assign O_tdata  = {
    S_out0, S_out0, S_out0,
    S_out1, S_out1, S_out1,
    S_out2, S_out2, S_out2,
    S_out3, S_out3, S_out3
};

endmodule
