`timescale 1ns / 1ps

module ae_set (
    input wire I_clk, input wire I_rst, input wire [3:0] I_btn,
    input wire I_ae_cfg_done, input wire I_cam_cfg_done,
    input wire [19:0] I_luma_sum, input wire I_luma_toggle,
    output wire O_ae_req, output wire [15:0] O_ae,
    output wire [15:0] O_ag
);
    localparam [15:0] AE_MIN=16'd200, AE_INIT=16'd1800, AE_MAX=16'd1900;
    // Start close to unity gain.  V11 started at index 80 (about 6x), which
    // made the image bright quickly but amplified SC500CS dark noise.
    localparam [15:0] AG_MIN=16'd0, AG_INIT=16'd16, AG_MAX=16'd127;
    // Sparse sampler count: 40*23=920. Initial target mean is 112.
    localparam [19:0] TARGET_MIN_SUM=20'd73600;
    localparam [19:0] TARGET_INIT_SUM=20'd103040;
    localparam [19:0] TARGET_MAX_SUM=20'd147200;
    localparam [19:0] TARGET_STEP_SUM=20'd3680;
    localparam [19:0] DEADBAND_SUM=20'd11040;
    localparam [19:0] FAST_BAND_SUM=20'd27600;

    reg [19:0] S_target_sum;
    reg [15:0] S_ae_set, S_ag_set, S_ae, S_ag;
    reg S_ae_req;

    // KEY2/KEY3 adjust AE compensation, while the loop stays automatic.
    reg [19:0] S_ms_count;
    reg [4:0] S_repeat_count;
    reg S_key_tick;
    reg [3:0] S_button_d1, S_button_d2;
    always @(posedge I_clk or posedge I_rst) begin
        if(I_rst) begin
            S_ms_count<=0; S_repeat_count<=0; S_key_tick<=0;
            S_button_d1<=0; S_button_d2<=0;
        end else begin
            S_button_d1 <= ~I_btn;
            S_button_d2 <= S_button_d1;
            S_key_tick <= 1'b0;
            if(S_ms_count == 20'd23999) begin
                S_ms_count <= 0;
                if(S_repeat_count == 5'd31) begin
                    S_repeat_count <= 0;
                    S_key_tick <= 1'b1;
                end else S_repeat_count <= S_repeat_count + 1'b1;
            end else S_ms_count <= S_ms_count + 1'b1;
        end
    end

    // Bundled-data CDC: source holds the bus for a full frame. Three preserved
    // toggle synchronizer stages precede the bus capture; another two clocks
    // precede consumption. Only the first toggle stage is asynchronous.
    (* syn_preserve=1, syn_keep=1, ASYNC_REG="TRUE" *) reg S_luma_toggle_meta;
    (* syn_preserve=1, syn_keep=1, ASYNC_REG="TRUE" *) reg S_luma_toggle_sync1;
    (* syn_preserve=1, syn_keep=1, ASYNC_REG="TRUE" *) reg S_luma_toggle_sync2;
    reg S_luma_toggle_seen;
    reg [1:0] S_luma_capture_valid;
    reg [19:0] S_luma_sum_d1, S_luma_sum_d2;
    wire S_capture_luma = S_luma_toggle_sync2 ^ S_luma_toggle_seen;
    wire S_new_luma = S_luma_capture_valid[1];
    always @(posedge I_clk or posedge I_rst) begin
        if(I_rst) begin
            S_luma_toggle_meta<=0;S_luma_toggle_sync1<=0;S_luma_toggle_sync2<=0;
            S_luma_toggle_seen<=0;S_luma_capture_valid<=0;
            S_luma_sum_d1<=0; S_luma_sum_d2<=0;
        end else begin
            S_luma_toggle_meta<=I_luma_toggle;
            S_luma_toggle_sync1<=S_luma_toggle_meta;
            S_luma_toggle_sync2<=S_luma_toggle_sync1;
            S_luma_toggle_seen<=S_luma_toggle_sync2;
            S_luma_capture_valid<={S_luma_capture_valid[0],S_capture_luma};
            if(S_capture_luma) S_luma_sum_d1 <= I_luma_sum;
            S_luma_sum_d2 <= S_luma_sum_d1;
        end
    end

    reg [19:0] S_luma_filtered;
    reg        S_luma_filter_valid;
    wire [19:0] S_low_limit = S_target_sum-DEADBAND_SUM;
    wire [19:0] S_high_limit = S_target_sum+DEADBAND_SUM;
    wire [19:0] S_fast_low_limit = S_target_sum-FAST_BAND_SUM;
    wire [19:0] S_fast_high_limit = S_target_sum+FAST_BAND_SUM;
    reg [2:0] S_frame_wait;

    function [15:0] F_add_sat;
        input [15:0] value, step, maximum;
        reg [16:0] sum;
        begin
            sum={1'b0,value}+{1'b0,step};
            F_add_sat=(sum>{1'b0,maximum})?maximum:sum[15:0];
        end
    endfunction
    function [15:0] F_sub_sat;
        input [15:0] value, step, minimum;
        begin F_sub_sat=(value<=minimum+step)?minimum:value-step; end
    endfunction

    // Filter scene brightness over time and update only once per eight frames.
    // The slower loop is intentional: SC500CS exposure/gain takes N+2 frames,
    // and reacting to an unfiltered single frame caused visible breathing.
    always @(posedge I_clk or posedge I_rst) begin
        if(I_rst) begin
            S_target_sum<=TARGET_INIT_SUM; S_ae_set<=AE_INIT;
            S_ag_set<=AG_INIT; S_frame_wait<=0;
            S_luma_filtered<=0; S_luma_filter_valid<=1'b0;
        end else begin
            if(S_key_tick && S_button_d2[3] && S_target_sum<TARGET_MAX_SUM)
                S_target_sum <= S_target_sum+TARGET_STEP_SUM;
            else if(S_key_tick && S_button_d2[2] && S_target_sum>TARGET_MIN_SUM)
                S_target_sum <= S_target_sum-TARGET_STEP_SUM;

            if(S_new_luma) begin
                if(!S_luma_filter_valid) begin
                    S_luma_filtered <= S_luma_sum_d2;
                    S_luma_filter_valid <= 1'b1;
                end else begin
                    S_luma_filtered <= S_luma_filtered-
                                       (S_luma_filtered>>3)+
                                       (S_luma_sum_d2>>3);
                end
                if(S_frame_wait != 3'd7)
                    S_frame_wait <= S_frame_wait+1'b1;
                else begin
                    S_frame_wait <= 0;
                    if(S_luma_filtered < S_low_limit) begin
                        if(S_ae_set < AE_MAX)
                            S_ae_set <= F_add_sat(S_ae_set,
                              (S_luma_filtered<S_fast_low_limit)?16'd32:16'd8,AE_MAX);
                        else if(S_ag_set < AG_MAX)
                            S_ag_set <= F_add_sat(S_ag_set,
                              (S_luma_filtered<S_fast_low_limit)?16'd2:16'd1,AG_MAX);
                    end else if(S_luma_filtered > S_high_limit) begin
                        // Remove noisy gain before shortening exposure.
                        if(S_ag_set > AG_MIN)
                            S_ag_set <= F_sub_sat(S_ag_set,
                              (S_luma_filtered>S_fast_high_limit)?16'd2:16'd1,AG_MIN);
                        else if(S_ae_set > AE_MIN)
                            S_ae_set <= F_sub_sat(S_ae_set,
                              (S_luma_filtered>S_fast_high_limit)?16'd32:16'd8,AE_MIN);
                    end
                end
            end
        end
    end

    assign O_ae_req=S_ae_req; assign O_ae=S_ae; assign O_ag=S_ag;
    always @(posedge I_clk or posedge I_rst) begin
        if(I_rst) begin S_ae<=AE_INIT; S_ag<=AG_INIT; S_ae_req<=0; end
        else if(S_ae_req) S_ae_req<=0;
        else if(((S_ae!=S_ae_set)||(S_ag!=S_ag_set)) &&
                I_cam_cfg_done && I_ae_cfg_done) begin
            S_ae<=S_ae_set; S_ag<=S_ag_set; S_ae_req<=1;
        end
    end
endmodule

// Sparse full-frame luminance sampler in the HDMI pixel-clock domain.
module ae_luma_sampler (
    input wire I_clk, input wire I_rst_n, input wire I_vsync,
    input wire I_de, input wire [23:0] I_rgb,
    output reg [19:0] O_luma_sum, output reg O_luma_toggle
);
    wire [7:0] S_luma={2'b0,I_rgb[23:18]}+{1'b0,I_rgb[15:9]}+
                      {2'b0,I_rgb[7:2]};
    reg S_vsync_d,S_de_d;
    reg [11:0] S_x,S_y;
    reg [19:0] S_sum;
    reg [10:0] S_count;
    wire S_sample_x=!S_de_d||(S_x[4:0]==5'd31);
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_vsync_d<=0; S_de_d<=0; S_x<=0; S_y<=0; S_sum<=0;
            S_count<=0; O_luma_sum<=0; O_luma_toggle<=0;
        end else begin
            S_vsync_d<=I_vsync; S_de_d<=I_de;
            if(I_vsync&&!S_vsync_d) begin
                if(S_count>=11'd800) begin
                    O_luma_sum<=S_sum; O_luma_toggle<=~O_luma_toggle;
                end
                S_x<=0; S_y<=0; S_sum<=0; S_count<=0;
            end else if(I_de) begin
                if(!S_de_d) S_x<=0; else S_x<=S_x+1'b1;
                if(S_sample_x&&(S_y[4:0]==0)) begin
                    S_sum<=S_sum+{12'd0,S_luma}; S_count<=S_count+1'b1;
                end
            end else if(S_de_d) S_y<=S_y+1'b1;
        end
    end
endmodule
