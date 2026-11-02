// Streaming four-pixel Sobel detector.  Two synchronous line memories retain
// grayscale rows; memory contents need no reset because the first two rows
// of every frame are explicitly masked.  3x3 windows are causal: a detected
// edge is displayed one pixel right and one row down from its window centre.
module edge_detect_96 #(
    parameter IMG_WIDTH = 1280,
    parameter IMG_HEIGHT = 720,
    parameter EDGE_TH = 11'd80
)(
    input wire I_clk, I_rst_n, I_tlast, I_tuser,
    input wire [95:0] I_tdata,
    input wire I_tvalid, I_tready,
    output reg O_tlast, O_tuser,
    output reg [95:0] O_tdata,
    output reg O_tvalid,
    output wire O_tready
);
    // The surrounding ISP is a continuous video stream and does not provide
    // end-to-end elastic buffering. Preserve its existing ready contract.
    assign O_tready = I_tready;
    localparam integer GROUPS = IMG_WIDTH / 4;
    localparam integer XBITS = GROUPS > 1 ? $clog2(GROUPS) : 1;
    localparam integer YBITS = IMG_HEIGHT > 1 ? $clog2(IMG_HEIGHT) : 1;

    function [7:0] luma;
        input [23:0] rgb;
        begin luma = {2'b0,rgb[23:18]} + {1'b0,rgb[15:9]} + {2'b0,rgb[7:2]}; end
    endfunction
    wire [31:0] gray_in = {luma(I_tdata[95:72]),luma(I_tdata[71:48]),
                           luma(I_tdata[47:24]),luma(I_tdata[23:0])};

    (* ram_style = "block" *) reg [31:0] line1 [0:GROUPS-1];
    (* ram_style = "block" *) reg [31:0] line2 [0:GROUPS-1];
    reg [XBITS-1:0] x_group;
    reg [YBITS-1:0] y_line;
    wire [XBITS-1:0] read_x = I_tuser ? {XBITS{1'b0}} : x_group;
    wire [YBITS-1:0] read_y = I_tuser ? {YBITS{1'b0}} : y_line;
    reg [31:0] middle_word,top_word,bottom_word;
    reg [XBITS-1:0] stage_x;
    reg [YBITS-1:0] stage_y;
    reg stage_valid,stage_sof,stage_eol;
    reg [7:0] t2,t1,m2,m1,b2,b1;

    // Exact |Gx|+|Gy| with 11-bit signed gradients. Each weighted sum is at
    // most 1020, so no multipliers or widened 32-bit arithmetic are needed.
    function [11:0] magnitude;
        input [7:0] a,b,c,d,f,g,h,i;
        reg [10:0] left_sum,right_sum,top_sum,bottom_sum;
        reg [10:0] dx,dy;
        begin
            left_sum={3'b0,a}+({3'b0,d}<<1)+{3'b0,g};
            right_sum={3'b0,c}+({3'b0,f}<<1)+{3'b0,i};
            top_sum={3'b0,a}+({3'b0,b}<<1)+{3'b0,c};
            bottom_sum={3'b0,g}+({3'b0,h}<<1)+{3'b0,i};
            dx=left_sum>=right_sum ? left_sum-right_sum : right_sum-left_sum;
            dy=top_sum>=bottom_sum ? top_sum-bottom_sum : bottom_sum-top_sum;
            magnitude={1'b0,dx}+{1'b0,dy};
        end
    endfunction
    wire [7:0] t0=top_word[31:24],ta=top_word[23:16],tb=top_word[15:8],tc=top_word[7:0];
    wire [7:0] m0=middle_word[31:24],ma=middle_word[23:16],mb=middle_word[15:8],mc=middle_word[7:0];
    wire [7:0] b0=bottom_word[31:24],ba=bottom_word[23:16],bb=bottom_word[15:8],bc=bottom_word[7:0];
    wire rows_ok = stage_y >= 2;
    wire e0=rows_ok && stage_x!=0 && magnitude(t2,t1,t0,m2,m0,b2,b1,b0)>=EDGE_TH;
    wire e1=rows_ok && stage_x!=0 && magnitude(t1,t0,ta,m1,ma,b1,b0,ba)>=EDGE_TH;
    wire e2=rows_ok && magnitude(t0,ta,tb,m0,mb,b0,ba,bb)>=EDGE_TH;
    wire e3=rows_ok && magnitude(ta,tb,tc,ma,mc,ba,bb,bc)>=EDGE_TH;
    wire [23:0] p0={24{e0}},p1={24{e1}},p2={24{e2}},p3={24{e3}};

    // Synchronous read/write is important for mapping the line stores to
    // dedicated memory instead of hundreds of slice-based distributed RAMs.
    always @(posedge I_clk) begin
        if (I_tvalid) begin
            middle_word <= line1[read_x];
            top_word <= line2[read_x];
            line1[read_x] <= gray_in;
        end
        if (stage_valid) line2[stage_x] <= middle_word;
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            x_group<=0; y_line<=0; stage_valid<=0; stage_sof<=0; stage_eol<=0;
            stage_x<=0; stage_y<=0; bottom_word<=0;
            t2<=0; t1<=0; m2<=0; m1<=0; b2<=0; b1<=0;
            O_tvalid<=0; O_tuser<=0; O_tlast<=0; O_tdata<=0;
        end else begin
            stage_valid<=I_tvalid;
            // AWB forwards frame-start independently of pixel valid; it may
            // arrive in blanking dozens of clocks before the first pixel.
            stage_sof<=I_tuser;
            // AWB may assert tlast in the blanking cycle; row length supplies
            // the correctly aligned last-beat marker for this fixed stream.
            stage_eol<=I_tvalid && read_x==GROUPS-1;
            if(I_tuser && !I_tvalid) begin x_group<=0; y_line<=0; end
            if(I_tvalid) begin
                bottom_word<=gray_in; stage_x<=read_x; stage_y<=read_y;
                if(read_x==GROUPS-1) begin
                    x_group<=0;
                    y_line<=read_y==IMG_HEIGHT-1 ? 0 : read_y+1'b1;
                end else begin
                    x_group<=read_x+1'b1; y_line<=read_y;
                end
            end
            O_tvalid<=stage_valid;
            O_tuser<=stage_sof;
            O_tlast<=stage_valid && stage_eol;
            if(stage_valid) begin
                O_tdata<={p0,p1,p2,p3};
                if(stage_eol) begin t2<=0;t1<=0;m2<=0;m1<=0;b2<=0;b1<=0;end
                else begin t2<=tb;t1<=tc;m2<=mb;m1<=mc;b2<=bb;b1<=bc;end
            end else O_tdata<=0;
        end
    end
endmodule
