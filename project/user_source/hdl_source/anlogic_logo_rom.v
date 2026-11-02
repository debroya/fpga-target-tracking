// Lossless synchronous block-ROM replacement for the former 6387-entry case.
// Feed the stage-A COMBINATIONAL address; the memory itself replaces the
// former address register, so the mixer output keeps its original latency.
module anlogic_logo_rom #(
 parameter INIT_FILE="logo_pixels.mem"
)(input wire I_clk,input wire [14:0] I_addr,output wire [24:0] O_pixel);
 (* ram_style="bram" *) reg [24:0] pixels[0:16383];
 initial $readmemh(INIT_FILE,pixels);
 wire [13:0] index=I_addr-15'd4096;
 reg [24:0] raw_pixel;
 reg in_window;
 always @(posedge I_clk) begin
  raw_pixel<=pixels[index];
  in_window<=I_addr>=15'd4096 && I_addr<15'd20480;
 end
 assign O_pixel=in_window ? raw_pixel : 25'd0;
endmodule
