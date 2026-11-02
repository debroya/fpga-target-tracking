`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Create Date:    01:32:02 06/12/2023 
// Module Name:    SC500GainTbl 
//////////////////////////////////////////////////////////////////////////////////
module SC500GainTbl(
	input 			I_clk,
	input 			I_rstn,
	input 	[6:0] 	I,
	output reg	[15:0] 	O
);

	// Four 32-step analogue ranges from the SC500CS data sheet.  The
	// original demo stopped at index 73 (5.125x); this covers the complete
	// documented 1.0x to 15.75x range without a large case table.
	always @(posedge I_clk) begin
		case(I[6:5])
			2'd0: O <= {8'h03,3'b001,I[4:0]};
			2'd1: O <= {8'h07,3'b001,I[4:0]};
			2'd2: O <= {8'h0f,3'b001,I[4:0]};
			default: O <= {8'h1f,3'b001,I[4:0]};
		endcase
	end

endmodule
