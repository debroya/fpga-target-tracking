// Lightweight display-only RGB effects.  The detector continues to consume
// the unmodified video stream in hdmi_mixer.
module display_mode_pixel (
    input  wire [3:0]  I_mode,
    input  wire [1:0]  I_rgb_channel,
    input  wire [23:0] I_rgb,
    output reg  [23:0] O_rgb
);
    wire [7:0] S_r = I_rgb[23:16];
    wire [7:0] S_g = I_rgb[15:8];
    wire [7:0] S_b = I_rgb[7:0];
    // One shared, multiplier-free luma approximation for all three new modes:
    // Y ~= R/4 + G/2 + B/4.  This costs only two small adders.
    wire [8:0] S_gray_sum = {1'b0, S_r[7:2]} +
                            {1'b0, S_g[7:1]} +
                            {1'b0, S_b[7:2]};
    wire [7:0] S_gray = S_gray_sum[7:0];

    function [23:0] F_pseudo_color;
        input [2:0] I_level;
        begin
            case(I_level)
                3'd0: F_pseudo_color = 24'h000020; // near black / navy
                3'd1: F_pseudo_color = 24'h0000ff; // blue
                3'd2: F_pseudo_color = 24'h00ffff; // cyan
                3'd3: F_pseudo_color = 24'h00ff00; // green
                3'd4: F_pseudo_color = 24'hffff00; // yellow
                3'd5: F_pseudo_color = 24'hff8000; // orange
                3'd6: F_pseudo_color = 24'hff0000; // red
                default: F_pseudo_color = 24'hffffff; // brightest band
            endcase
        end
    endfunction

    function [7:0] F_add32_sat;
        input [7:0] I_value;
        begin
            F_add32_sat = (I_value > 8'd223) ? 8'hff :
                          (I_value + 8'd32);
        end
    endfunction

    always @(*) begin
        case(I_mode)
            4'd4: O_rgb = ~I_rgb;
            4'd5: begin
                case(I_rgb_channel)
                    2'd0: O_rgb = {S_r, 8'h00, 8'h00};
                    2'd1: O_rgb = {8'h00, S_g, 8'h00};
                    default: O_rgb = {8'h00, 8'h00, S_b};
                endcase
            end
            4'd6: O_rgb = {{S_r[7:5], 5'b0},
                           {S_g[7:5], 5'b0},
                           {S_b[7:5], 5'b0}};
            4'd7: O_rgb = {F_add32_sat(S_r),
                           F_add32_sat(S_g),
                           F_add32_sat(S_b)};
            // Brightness distribution visualization; this is not thermal data.
            4'd8: O_rgb = F_pseudo_color(S_gray[7:5]);
            // Highlight/shadow warning: dark pixels blue, near-saturated pixels
            // red, and the useful mid-range remains the original image.
            4'd9: O_rgb = (S_gray < 8'd32)  ? 24'h0000ff :
                           (S_gray >= 8'd224) ? 24'hff0000 : I_rgb;
            // Fixed gray bit plane 5: structure remains visible without a
            // selector or frame buffer.  A later revision can make it tunable.
            4'd10: O_rgb = S_gray[5] ? 24'hffffff : 24'h000000;
            default: O_rgb = I_rgb;
        endcase
    end
endmodule
