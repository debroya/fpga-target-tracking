module hdmi_mixer #(
    parameter H_OFFSET = 128,
    parameter V_OFFSET = 60,
    parameter IMG_WIDTH = 1280,
    parameter IMG_HEIGHT = 720,
    parameter integer DEBUG_MODE = 0
)(
    input wire        I_clk,
    input wire        I_rst_n,

    input wire        I_video_vsync,
    input wire        I_video_hsync,
    input wire        I_video_de,
    input wire        I_video_user,
    input wire        I_video_last,

    input wire[3:0]   I_debug_status,
    input wire[3:0]   I_mode,
    input wire[1:0]   I_rgb_channel,
    input wire        I_motion_mode,
    input wire        I_detection_enable,

    output wire       O_video_rd_en,
    input wire[23:0]  I_video_rd_data,

    output wire       O_target_valid,
    output wire[11:0] O_target_cx,
    output wire[11:0] O_target_cy,
    output wire[11:0] O_target_radius,
    output wire[7:0]  O_target_confidence,

    output reg        O_hdmi_vsync,
    output reg        O_hdmi_hsync,
    output reg        O_hdmi_de,
    output reg[23:0]  O_hdmi_data
);

    reg        S_video_vsync_1d;
    reg        S_video_vsync_2d;
    reg        S_video_vsync_3d;
    reg        S_video_vsync_4d;
    reg        S_video_hsync_1d;
    reg        S_video_hsync_2d;
    reg        S_video_hsync_3d;
    reg        S_video_hsync_4d;
    reg        S_video_de_1d;
    reg        S_video_de_2d;
    reg        S_video_de_3d;
    reg        S_video_de_4d;
    reg [23:0] S_video_data_1d;
    reg [23:0] S_video_data_2d;
    reg [23:0] S_video_data_3d;
    reg [23:0] S_video_data_4d;
    reg [11:0] S_x;
    reg [11:0] S_y;
    reg [11:0] S_x_1d;
    reg [11:0] S_x_2d;
    reg [11:0] S_x_3d;
    reg [11:0] S_x_4d;
    reg [11:0] S_y_1d;
    reg [11:0] S_y_2d;

    reg        S_osd_hit;
    reg [23:0] S_osd_color;
    reg        S_osd_hit_3d;
    reg [23:0] S_osd_color_3d;
    reg [14:0] S_logo_addr;
    reg [14:0] S_logo_stream_addr;
    reg        S_logo_on;
    reg        S_logo_on_a;
    wire[24:0] S_logo_pixel;
    reg [11:0] S_dyn_x;
    reg [11:0] S_dyn_y;
    reg        S_dyn_on;
    reg [11:0] S_dyn_digit_x;
    reg [3:0]  S_cnt_d0;
    reg [3:0]  S_cnt_d1;
    reg [3:0]  S_cnt_d2;
    reg [3:0]  S_cnt_d3;
    reg [3:0]  S_cnt_d4;
    reg [3:0]  S_cnt_d5;
    reg [3:0]  S_dyn_dec_digit;
    reg [7:0]  S_pf_char;
    reg [4:0]  S_pf_local_x;
    reg [4:0]  S_pf_local_y;
    reg [3:0]  S_pf_col;
    reg [3:0]  S_pf_row;
    wire [15:0] S_pf_row_bits;
    reg        S_pf_on;
    reg [7:0]  S_pf_char_a;
    reg [3:0]  S_pf_col_a;
    reg [3:0]  S_pf_row_a;
    reg        S_pf_on_a;
    reg [3:0]  S_mode_sync_1d;
    reg [3:0]  S_mode_sync_2d;
    reg [3:0]  S_mode_active;
    reg [1:0]  S_rgb_sync_1d;
    reg [1:0]  S_rgb_sync_2d;
    reg [1:0]  S_rgb_active;
    reg [4:0]  S_mode_char_idx;
    reg [7:0]  S_mode_char;
    
    reg  [3:0]  S_mode_col;
    reg  [3:0]  S_mode_row;
    wire [15:0] S_mode_row_bits;
    reg         S_mode_on;
    reg  [7:0]  S_mode_char_a;
    reg  [3:0]  S_mode_col_a;
    reg  [3:0]  S_mode_row_a;
    reg         S_mode_on_a;
    wire        S_target_valid;
    wire [11:0] S_target_cx;
    wire [11:0] S_target_cy;
    wire [11:0] S_target_radius;
    wire [7:0]  S_target_confidence;
    reg         S_motion_active,S_detection_active,S_motion_vsync_d;
    wire        S_motion_valid;
    wire [11:0] S_motion_left,S_motion_right,S_motion_top,S_motion_bottom;
    wire [1:0]  S_fence_state;
    wire [2:0]  S_fence_direction;
    wire        S_fence_inside;
    wire [11:0] S_fence_entry_bcd,S_fence_dwell_bcd;
    wire [2:0]  S_passage_event;
    wire [11:0] S_passage_in_bcd,S_passage_out_bcd;
    wire        S_latency_timing,S_latency_done;
    wire [11:0] S_latency_bcd,S_latency_running_bcd;
    wire [11:0] S_latency_seconds_bcd,S_latency_running_seconds_bcd;
    wire        S_box_valid=I_detection_enable && S_detection_active &&
                           (S_motion_active ? S_motion_valid : S_target_valid);
    reg         S_target_box_on;
    reg [23:0]  S_target_box_color;
    reg         S_target_box_on_a;
    reg [23:0]  S_target_box_color_a;
    wire[23:0]  S_display_mode_data;
    reg [11:0]  S_target_left;
    reg [11:0]  S_target_right;
    reg [11:0]  S_target_top;
    reg [11:0]  S_target_bottom;
    reg [11:0]  S_target_cross_left;
    reg [11:0]  S_target_cross_right;
    reg [11:0]  S_target_cross_top;
    reg [11:0]  S_target_cross_bottom;
    reg [11:0]  S_target_left_inner;
    reg [11:0]  S_target_right_inner;
    reg [11:0]  S_target_top_inner;
    reg [11:0]  S_target_bottom_inner;
    reg [11:0]  S_target_mid_left;
    reg [11:0]  S_target_mid_right;
    reg [11:0]  S_target_mid_top;
    reg [11:0]  S_target_mid_bottom;
    // Region comparisons are registered one stage before the OSD priority
    // mux.  They remain aligned with S_x_2d/S_y_2d but no longer sit in the
    // coordinate-compare -> priority -> colour critical path.
    reg S_center_reference_2d;
    reg S_target_border_2d;
    reg S_target_cross_2d;
    reg S_fence_border_2d;
    reg S_pass_line_a_2d;
    reg S_pass_line_b_2d;
    localparam [11:0] SCREEN_CENTER_X = IMG_WIDTH/2;
    localparam [11:0] SCREEN_CENTER_Y = IMG_HEIGHT/2;
    // Exact integer-pixel disk x^2+y^2<=3^2 without subtractors, squares or
    // multipliers: a 5x5 core plus the four radius-three cardinal pixels.
    // This is both smaller and faster than the former approximate ring path.

    function [7:0] F_detection_char;
    input motion;input [3:0] idx;
    begin case(idx)
        0:F_detection_char=motion ? 8'h4d : 8'h54;
        1:F_detection_char=motion ? 8'h4f : 8'h41;
        2:F_detection_char=motion ? 8'h54 : 8'h52;
        3:F_detection_char=motion ? 8'h49 : 8'h47;
        4:F_detection_char=motion ? 8'h4f : 8'h45;
        5:F_detection_char=motion ? 8'h4e : 8'h54;
        default:F_detection_char=8'h20;
    endcase end
    endfunction

    function [7:0] F_fence_line_char;
    input [2:0] dir; input [11:0] in_count; input [11:0] out_count; input [4:0] idx;
    input [1:0] fence_state;
    begin
        F_fence_line_char=8'h20;
        case(idx)
            0:case(fence_state)0:F_fence_line_char="S";1:F_fence_line_char="E";
                                2:F_fence_line_char="L";default:F_fence_line_char="I";endcase
            1:case(fence_state)0:F_fence_line_char="A";1:F_fence_line_char="N";
                                2:F_fence_line_char="E";default:F_fence_line_char="N";endcase
            2:case(fence_state)0:F_fence_line_char="F";1:F_fence_line_char="T";
                                2:F_fence_line_char="A";default:F_fence_line_char="T";endcase
            3:case(fence_state)0:F_fence_line_char="E";1:F_fence_line_char="E";
                                2:F_fence_line_char="V";default:F_fence_line_char="R";endcase
            4:case(fence_state)1:F_fence_line_char="R";2:F_fence_line_char="E";
                                3:F_fence_line_char="U";endcase
            5:if(fence_state==3)F_fence_line_char="S";
            6:if(fence_state==3)F_fence_line_char="I";
            7:if(fence_state==3)F_fence_line_char="O";
            8:if(fence_state==3)F_fence_line_char="N";
            10:case(dir)1:F_fence_line_char="L";2:F_fence_line_char="R";
                         3:F_fence_line_char="U";4:F_fence_line_char="D";default:F_fence_line_char="-";endcase
            12:F_fence_line_char="I";
            13:F_fence_line_char=8'h30+{4'd0,in_count[11:8]};
            14:F_fence_line_char=8'h30+{4'd0,in_count[7:4]};
            15:F_fence_line_char=8'h30+{4'd0,in_count[3:0]};
            17:F_fence_line_char="O";
            18:F_fence_line_char=8'h30+{4'd0,out_count[11:8]};
            19:F_fence_line_char=8'h30+{4'd0,out_count[7:4]};
            20:F_fence_line_char=8'h30+{4'd0,out_count[3:0]};
        endcase
    end
    endfunction

    function [7:0] F_passage_line_char;
    input [2:0] event_code;input [11:0] intrusion_bcd;input [11:0] time_bcd;input [4:0] idx;
    begin
        F_passage_line_char=8'h20;
        case(idx)
            0:case(event_code)1:F_passage_line_char="P";2:F_passage_line_char="W";
                               3:F_passage_line_char="L";4:F_passage_line_char="S";
                               default:F_passage_line_char="N";endcase
            1:case(event_code)1:F_passage_line_char="A";2:F_passage_line_char="R";
                               3:F_passage_line_char="O";4:F_passage_line_char="T";
                               default:F_passage_line_char="O";endcase
            2:case(event_code)1:F_passage_line_char="S";2:F_passage_line_char="O";
                               3:F_passage_line_char="I";4:F_passage_line_char="A";
                               default:F_passage_line_char="R";endcase
            3:case(event_code)1:F_passage_line_char="S";2:F_passage_line_char="N";
                               3:F_passage_line_char="T";4:F_passage_line_char="Y";
                               default:F_passage_line_char="M";endcase
            4:case(event_code)0:F_passage_line_char="A";2:F_passage_line_char="G";
                               3:F_passage_line_char="E";default:F_passage_line_char=8'h20;endcase
            5:case(event_code)0:F_passage_line_char="L";3:F_passage_line_char="R";
                               default:F_passage_line_char=8'h20;endcase
            12:F_passage_line_char="T";
            13:F_passage_line_char=8'h30+{4'd0,time_bcd[11:8]};
            14:F_passage_line_char=8'h30+{4'd0,time_bcd[7:4]};
            15:F_passage_line_char=8'h30+{4'd0,time_bcd[3:0]};
        endcase
    end
    endfunction

    function [7:0] F_latency_char;
    input result_valid;input [11:0] result_bcd;input [11:0] live_bcd;input [3:0] idx;
    begin
        F_latency_char=8'h20;
        case(idx)
            0:F_latency_char="L";
            1:F_latency_char="A";
            2:F_latency_char="T";
            // First field: the last completed/frozen result.
            4:F_latency_char=result_valid ? 8'h30+{4'd0,result_bcd[11:8]} : "-";
            5:F_latency_char=result_valid ? 8'h30+{4'd0,result_bcd[7:4]} : "-";
            6:F_latency_char=result_valid ? "." : "-";
            7:F_latency_char=result_valid ? 8'h30+{4'd0,result_bcd[3:0]} : "-";
            8:F_latency_char="S";
            // Second field: this acquisition's live count.  It is 000 while
            // idle or immediately after the frozen result is captured.
            10:F_latency_char=8'h30+{4'd0,live_bcd[11:8]};
            11:F_latency_char=8'h30+{4'd0,live_bcd[7:4]};
            12:F_latency_char=".";
            13:F_latency_char=8'h30+{4'd0,live_bcd[3:0]};
            14:F_latency_char="S";
        endcase
    end
    endfunction

    // Centralized mode-name lookup avoids eleven replicated nested decoders in
    // the pixel OSD block.  Ten character cells are available on screen.
    function [7:0] F_mode_char;
    input [3:0] mode; input [1:0] rgb_channel; input [3:0] idx;
    begin
        F_mode_char=8'h20;
        case(mode)
            4'd0: case(idx) 0:F_mode_char="N";1:F_mode_char="O";2:F_mode_char="R";
                              3:F_mode_char="M";4:F_mode_char="A";5:F_mode_char="L";endcase
            4'd1: case(idx) 0:F_mode_char="S";1:F_mode_char="O";2:F_mode_char="B";
                              3:F_mode_char="E";4:F_mode_char="L";endcase
            4'd2: case(idx) 0:F_mode_char="G";1:F_mode_char="R";2:F_mode_char="A";
                              3:F_mode_char="Y";endcase
            4'd3: case(idx) 0:F_mode_char="B";1:F_mode_char="I";2:F_mode_char="N";
                              3:F_mode_char="A";4:F_mode_char="R";5:F_mode_char="Y";endcase
            4'd4: case(idx) 0:F_mode_char="N";1:F_mode_char="E";2:F_mode_char="G";
                              3:F_mode_char="A";4:F_mode_char="T";5:F_mode_char="I";
                              6:F_mode_char="V";7:F_mode_char="E";endcase
            4'd5: begin
                case(idx)
                    0:F_mode_char="R";1:F_mode_char="G";2:F_mode_char="B";
                    4:F_mode_char=(rgb_channel==0)?"R":(rgb_channel==1)?"G":"B";
                    5:F_mode_char=(rgb_channel==0)?"E":(rgb_channel==1)?"R":"L";
                    6:F_mode_char=(rgb_channel==0)?"D":(rgb_channel==1)?"E":"U";
                    7:F_mode_char=(rgb_channel==1)?"E":(rgb_channel==2)?"E":" ";
                    8:F_mode_char=(rgb_channel==1)?"N":" ";
                endcase
            end
            4'd6: case(idx) 0:F_mode_char="P";1:F_mode_char="O";2:F_mode_char="S";
                              3:F_mode_char="T";4:F_mode_char="E";5:F_mode_char="R";
                              6:F_mode_char="I";7:F_mode_char="Z";8:F_mode_char="E";endcase
            4'd7: case(idx) 0:F_mode_char="B";1:F_mode_char="R";2:F_mode_char="I";
                              3:F_mode_char="G";4:F_mode_char="H";5:F_mode_char="T";
                              6:F_mode_char="N";7:F_mode_char="E";8:F_mode_char="S";
                              9:F_mode_char="S";endcase
            4'd8: case(idx) 0:F_mode_char="P";1:F_mode_char="S";2:F_mode_char="E";
                              3:F_mode_char="U";4:F_mode_char="D";5:F_mode_char="O";endcase
            4'd9: case(idx) 0:F_mode_char="E";1:F_mode_char="X";2:F_mode_char="P";
                              3:F_mode_char="O";4:F_mode_char="S";5:F_mode_char="U";
                              6:F_mode_char="R";7:F_mode_char="E";endcase
            4'd10:case(idx) 0:F_mode_char="B";1:F_mode_char="I";2:F_mode_char="T";
                              3:F_mode_char="P";4:F_mode_char="L";5:F_mode_char="A";
                              6:F_mode_char="N";7:F_mode_char="E";endcase
        endcase
    end
    endfunction

    localparam LOGO_X = 12'd0;
    localparam LOGO_Y = 12'd0;
    localparam LOGO_W = 12'd160;
    localparam LOGO_H = 12'd160;
    localparam DYN_OSD_X         = 12'd375;
    localparam DYN_OSD_Y         = 12'd120;
    localparam DYN_OSD_W         = 12'd200;
    localparam DYN_OSD_H         = 12'd24;
    localparam CN_Y_OFFSET       = 12'd2;
    localparam COLOR_DYN_TEXT    = 24'hfff200;
    localparam CHAR_AN           = 8'h80;
    localparam CHAR_LU           = 8'h81;
    localparam CHAR_C            = 8'h43;
    localparam CHAR_N            = 8'h4e;
    localparam CHAR_T            = 8'h54;
    localparam CHAR_0            = 8'h30;
    localparam CHAR_COLON        = 8'h3a;
    localparam MODE_OSD_X        = 12'd1120;
    localparam MODE_OSD_Y        = 12'd16;
    localparam MODE_OSD_W        = 12'd160;
    localparam FENCE_LEFT        = 12'd400;
    localparam FENCE_RIGHT       = 12'd880;
    localparam FENCE_TOP         = 12'd160;
    localparam FENCE_BOTTOM      = 12'd560;
    localparam PASS_LINE_A       = 12'd520;
    localparam PASS_LINE_B       = 12'd760;
    localparam FENCE_OSD_X       = 12'd16;
    localparam FENCE_INFO_Y      = 12'd672;
    
    function [3:0] F_scale_24_to_16;
    input [4:0] I_pos;

    begin
        case (I_pos)
            5'd0,  5'd1:  F_scale_24_to_16 = 4'd0;
            5'd2:         F_scale_24_to_16 = 4'd1;
            5'd3,  5'd4:  F_scale_24_to_16 = 4'd2;
            5'd5:         F_scale_24_to_16 = 4'd3;
            5'd6,  5'd7:  F_scale_24_to_16 = 4'd4;
            5'd8:         F_scale_24_to_16 = 4'd5;
            5'd9,  5'd10: F_scale_24_to_16 = 4'd6;
            5'd11:        F_scale_24_to_16 = 4'd7;
            5'd12, 5'd13: F_scale_24_to_16 = 4'd8;
            5'd14:        F_scale_24_to_16 = 4'd9;
            5'd15, 5'd16: F_scale_24_to_16 = 4'd10;
            5'd17:        F_scale_24_to_16 = 4'd11;
            5'd18, 5'd19: F_scale_24_to_16 = 4'd12;
            5'd20:        F_scale_24_to_16 = 4'd13;
            5'd21, 5'd22: F_scale_24_to_16 = 4'd14;
            5'd23:        F_scale_24_to_16 = 4'd15;

            default:      F_scale_24_to_16 = 4'd0;
        endcase
    end
endfunction
// 7-seg bit mapping: {a,b,c,d,e,f,g}
// Minimal 16x16 Chinese bitmap rows for "an" and "lu".
// Unified prefix character library (16x16 rows): an/lu/C/N/T/:
    assign O_video_rd_en = I_video_de;

    display_mode_pixel u_display_mode_pixel (
        .I_mode        (S_mode_active),
        .I_rgb_channel (S_rgb_active),
        .I_rgb         (I_video_rd_data),
        .O_rgb         (S_display_mode_data)
    );

    // Passive, video-only paper target detector.  It does not drive any
    // actuator or modify the four existing image processing modes.
    simple_black_disk_detector u_bw_ring_detector_stream (
        .I_clk       (I_clk),
        .I_rst_n     (I_rst_n),
        .I_vsync     (I_video_vsync),
        .I_de        (I_video_de && S_detection_active && !S_motion_active),
        .I_rgb       (I_video_rd_data),
        .O_valid     (S_target_valid),
        .O_center_x  (S_target_cx),
        .O_center_y  (S_target_cy),
        .O_radius    (S_target_radius),
        .O_confidence(S_target_confidence)
    );

    // Enable/mode changes take effect on a frame boundary. A disable request
    // vetoes boxes/servo data immediately, before the next boundary.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin S_motion_active<=0;S_detection_active<=0;S_motion_vsync_d<=0;end
        else begin
            S_motion_vsync_d<=I_video_vsync;
            if(I_video_vsync && !S_motion_vsync_d) begin
                S_motion_active<=I_motion_mode;
                S_detection_active<=I_detection_enable;
            end
        end
    end
    motion_object_detector u_motion_detector (
        .clk(I_clk),.rst_n(I_rst_n),
        .enable(S_detection_active && I_detection_enable && S_motion_active),
        .vsync(I_video_vsync),.de(I_video_de),.rgb(I_video_rd_data),
        .valid(S_motion_valid),.left(S_motion_left),.right(S_motion_right),
        .top(S_motion_top),.bottom(S_motion_bottom)
    );
    motion_fence_monitor #(
        .FENCE_LEFT(FENCE_LEFT),.FENCE_RIGHT(FENCE_RIGHT),
        .FENCE_TOP(FENCE_TOP),.FENCE_BOTTOM(FENCE_BOTTOM),
        .LINE_A(PASS_LINE_A),.LINE_B(PASS_LINE_B)
    ) u_motion_fence_monitor (
        .clk(I_clk),.rst_n(I_rst_n),
        .enable(S_detection_active && I_detection_enable && S_motion_active && I_motion_mode),
        .vsync(I_video_vsync),.motion_valid(S_motion_valid),
        .motion_left(S_motion_left),.motion_right(S_motion_right),
        .motion_top(S_motion_top),.motion_bottom(S_motion_bottom),
        .state(S_fence_state),.direction(S_fence_direction),.inside_fence(S_fence_inside),
        .entry_bcd(S_fence_entry_bcd),.dwell_bcd(S_fence_dwell_bcd),
        .passage_event(S_passage_event),.in_bcd(S_passage_in_bcd),
        .out_bcd(S_passage_out_bcd)
    );

    target_latency_monitor u_target_latency_monitor (
        .clk(I_clk),.rst_n(I_rst_n),
        .enable(S_detection_active && I_detection_enable && !S_motion_active && !I_motion_mode),
        .target_valid(S_target_valid),
        .target_cx(S_target_cx),.target_cy(S_target_cy),
        .cnt_bcd({S_cnt_d2,S_cnt_d1,S_cnt_d0}),
        .timing(S_latency_timing),.done(S_latency_done),.latency_bcd(S_latency_bcd),
        .running_bcd(S_latency_running_bcd)
    );

    latency_frames_to_seconds u_latency_result_seconds (
        .clk(I_clk),.rst_n(I_rst_n),.frames_bcd(S_latency_bcd),
        .seconds_bcd(S_latency_seconds_bcd));
    latency_frames_to_seconds u_latency_live_seconds (
        .clk(I_clk),.rst_n(I_rst_n),.frames_bcd(S_latency_running_bcd),
        .seconds_bcd(S_latency_running_seconds_bcd));

    assign O_target_valid      = S_target_valid && S_detection_active && I_detection_enable &&
                                !S_motion_active && !I_motion_mode;
    assign O_target_cx         = S_target_cx;
    assign O_target_cy         = S_target_cy;
    assign O_target_radius     = S_target_radius;
    assign O_target_confidence = S_target_confidence;

    anlogic_logo_rom u_anlogic_logo_rom(
        .I_clk   ( I_clk ),
        .I_addr  ( S_logo_addr ),
        .O_pixel ( S_logo_pixel )
    );

    osd_char_lib u_osd_char_lib(
        .I_char     ( S_pf_char_a     ),
        .I_row      ( S_pf_row_a      ),
        .O_row_bits ( S_pf_row_bits )
    );
    
    osd_char_lib u_mode_char_lib(
    .I_char     (S_mode_char_a),
    .I_row      (S_mode_row_a),
    .O_row_bits (S_mode_row_bits)
);

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_mode_sync_1d <= 4'd0;
            S_mode_sync_2d <= 4'd0;
            S_mode_active  <= 4'd0;
            S_rgb_sync_1d  <= 2'd0;
            S_rgb_sync_2d  <= 2'd0;
            S_rgb_active   <= 2'd0;
            S_x <= 12'd0;
            S_y <= 12'd0;
        end
        else begin
            S_mode_sync_1d <= I_mode;
            S_mode_sync_2d <= S_mode_sync_1d;
            S_rgb_sync_1d  <= I_rgb_channel;
            S_rgb_sync_2d  <= S_rgb_sync_1d;
            if(I_video_user) begin
                S_mode_active <= S_mode_sync_2d;
                S_rgb_active  <= S_rgb_sync_2d;
            end
            if(I_video_user) begin
                S_x <= 12'd0;
                S_y <= 12'd0;
            end
            else if(I_video_de) begin
                if(I_video_last) begin
                    S_x <= 12'd0;
                    S_y <= S_y + 12'd1;
                end
                else begin
                    S_x <= S_x + 12'd1;
                end
            end
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_cnt_d0 <= 4'd0;
            S_cnt_d1 <= 4'd0;
            S_cnt_d2 <= 4'd0;
            S_cnt_d3 <= 4'd0;
            S_cnt_d4 <= 4'd0;
            S_cnt_d5 <= 4'd0;
        end
        else if(I_video_user) begin
            if(S_cnt_d0 == 4'd9) begin
                S_cnt_d0 <= 4'd0;
                if(S_cnt_d1 == 4'd9) begin
                    S_cnt_d1 <= 4'd0;
                    if(S_cnt_d2 == 4'd9) begin
                        S_cnt_d2 <= 4'd0;
                        if(S_cnt_d3 == 4'd9) begin
                            S_cnt_d3 <= 4'd0;
                            if(S_cnt_d4 == 4'd9) begin
                                S_cnt_d4 <= 4'd0;
                                if(S_cnt_d5 == 4'd9)
                                    S_cnt_d5 <= 4'd0;
                                else
                                    S_cnt_d5 <= S_cnt_d5 + 4'd1;
                            end
                            else
                                S_cnt_d4 <= S_cnt_d4 + 4'd1;
                        end
                        else
                            S_cnt_d3 <= S_cnt_d3 + 4'd1;
                    end
                    else
                        S_cnt_d2 <= S_cnt_d2 + 4'd1;
                end
                else
                    S_cnt_d1 <= S_cnt_d1 + 4'd1;
            end
            else
                S_cnt_d0 <= S_cnt_d0 + 4'd1;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x_1d <= 12'd0;
            S_x_2d <= 12'd0;
            S_x_3d <= 12'd0;
            S_x_4d <= 12'd0;
            S_y_1d <= 12'd0;
            S_y_2d <= 12'd0;
            S_center_reference_2d <= 1'b0;
            S_target_border_2d    <= 1'b0;
            S_target_cross_2d     <= 1'b0;
            S_fence_border_2d     <= 1'b0;
            S_pass_line_a_2d      <= 1'b0;
            S_pass_line_b_2d      <= 1'b0;
        end
        else begin
            S_x_1d <= S_x;
            S_x_2d <= S_x_1d;
            S_x_3d <= S_x_2d;
            S_x_4d <= S_x_3d;
            S_y_1d <= S_y;
            S_y_2d <= S_y_1d;

            // Radius-three filled centre dot: 5x5 core plus four cardinal
            // endpoints.  Evaluate from the 1d coordinates and register the
            // result beside the 2d coordinates used by the OSD stage.
            S_center_reference_2d <= S_video_de_1d && I_detection_enable &&
                S_detection_active && !S_motion_active && !I_motion_mode &&
                (((S_x_1d>=SCREEN_CENTER_X-12'd2) &&
                   (S_x_1d<=SCREEN_CENTER_X+12'd2) &&
                   (S_y_1d>=SCREEN_CENTER_Y-12'd2) &&
                   (S_y_1d<=SCREEN_CENTER_Y+12'd2)) ||
                 ((S_x_1d==SCREEN_CENTER_X) &&
                   ((S_y_1d==SCREEN_CENTER_Y-12'd3) ||
                    (S_y_1d==SCREEN_CENTER_Y+12'd3))) ||
                 ((S_y_1d==SCREEN_CENTER_Y) &&
                   ((S_x_1d==SCREEN_CENTER_X-12'd3) ||
                    (S_x_1d==SCREEN_CENTER_X+12'd3))));

            S_target_border_2d <= S_video_de_1d && S_box_valid &&
                (((((S_x_1d==S_target_left) || (S_x_1d==S_target_left_inner) ||
                    (S_x_1d==S_target_right) || (S_x_1d==S_target_right_inner)) &&
                   (S_y_1d>=S_target_top) && (S_y_1d<=S_target_bottom))) ||
                 (((S_y_1d==S_target_top) || (S_y_1d==S_target_top_inner) ||
                   (S_y_1d==S_target_bottom) || (S_y_1d==S_target_bottom_inner)) &&
                  (S_x_1d>=S_target_left) && (S_x_1d<=S_target_right)));

            S_target_cross_2d <= S_video_de_1d && S_box_valid && !S_motion_active &&
                ((((S_x_1d>=S_target_cross_left) && (S_x_1d<=S_target_cross_right) &&
                   (S_y_1d>=S_target_mid_top) && (S_y_1d<=S_target_mid_bottom))) ||
                 (((S_y_1d>=S_target_cross_top) && (S_y_1d<=S_target_cross_bottom) &&
                   (S_x_1d>=S_target_mid_left) && (S_x_1d<=S_target_mid_right))));

            S_fence_border_2d <= S_video_de_1d && S_detection_active &&
                I_detection_enable && S_motion_active &&
                (((((S_x_1d==FENCE_LEFT) || (S_x_1d==FENCE_LEFT+1'b1) ||
                    (S_x_1d==FENCE_RIGHT) || (S_x_1d==FENCE_RIGHT-1'b1)) &&
                   (S_y_1d>=FENCE_TOP) && (S_y_1d<=FENCE_BOTTOM))) ||
                 (((S_y_1d==FENCE_TOP) || (S_y_1d==FENCE_TOP+1'b1) ||
                   (S_y_1d==FENCE_BOTTOM) || (S_y_1d==FENCE_BOTTOM-1'b1)) &&
                  (S_x_1d>=FENCE_LEFT) && (S_x_1d<=FENCE_RIGHT)));
            S_pass_line_a_2d <= S_video_de_1d && S_detection_active &&
                I_detection_enable && S_motion_active &&
                (S_y_1d>=FENCE_TOP) && (S_y_1d<=FENCE_BOTTOM) && !S_y_1d[4] &&
                ((S_x_1d==PASS_LINE_A) || (S_x_1d==PASS_LINE_A+1'b1));
            S_pass_line_b_2d <= S_video_de_1d && S_detection_active &&
                I_detection_enable && S_motion_active &&
                (S_y_1d>=FENCE_TOP) && (S_y_1d<=FENCE_BOTTOM) && !S_y_1d[4] &&
                ((S_x_1d==PASS_LINE_B) || (S_x_1d==PASS_LINE_B+1'b1));
        end
    end

    // The logo ROM is laid out linearly (160 pixels per row).  Since active
    // video is raster ordered, a streaming +1 address is equivalent to
    // y*160+x but removes the subtract/shift/add chain from the pixel clock.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            S_logo_stream_addr <= 15'd0;
        else if(S_video_de_2d &&
                (S_x_2d>=LOGO_X) && (S_x_2d<LOGO_X+LOGO_W) &&
                (S_y_2d>=LOGO_Y) && (S_y_2d<LOGO_Y+LOGO_H)) begin
            if((S_x_2d==LOGO_X) && (S_y_2d==LOGO_Y))
                S_logo_stream_addr <= 15'd0;
            else
                S_logo_stream_addr <= S_logo_stream_addr+15'd1;
        end
    end

    // Target outputs change only at the frame boundary.  Registering their
    // derived limits keeps add/subtract chains out of the pixel-rate OSD path.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_target_left         <= 12'd0;
            S_target_right        <= 12'd0;
            S_target_top          <= 12'd0;
            S_target_bottom       <= 12'd0;
            S_target_cross_left   <= 12'd0;
            S_target_cross_right  <= 12'd0;
            S_target_cross_top    <= 12'd0;
            S_target_cross_bottom <= 12'd0;
            S_target_left_inner   <= 12'd0;
            S_target_right_inner  <= 12'd0;
            S_target_top_inner    <= 12'd0;
            S_target_bottom_inner <= 12'd0;
            S_target_mid_left     <= 12'd0;
            S_target_mid_right    <= 12'd0;
            S_target_mid_top      <= 12'd0;
            S_target_mid_bottom   <= 12'd0;
        end else begin
            S_target_left         <= S_motion_active ? S_motion_left : S_target_cx - S_target_radius;
            S_target_right        <= S_motion_active ? S_motion_right : S_target_cx + S_target_radius;
            S_target_top          <= S_motion_active ? S_motion_top : S_target_cy - S_target_radius;
            S_target_bottom       <= S_motion_active ? S_motion_bottom : S_target_cy + S_target_radius;
            S_target_cross_left   <= S_target_cx - 12'd5;
            S_target_cross_right  <= S_target_cx + 12'd5;
            S_target_cross_top    <= S_target_cy - 12'd5;
            S_target_cross_bottom <= S_target_cy + 12'd5;
            // One more registered boundary for the inner two-pixel border;
            // avoids a mux + radius add/subtract + one-pixel add in one cycle.
            S_target_left_inner   <= S_target_left + 12'd1;
            S_target_right_inner  <= S_target_right - 12'd1;
            S_target_top_inner    <= S_target_top + 12'd1;
            S_target_bottom_inner <= S_target_bottom - 12'd1;
            S_target_mid_left     <= S_target_cx - 12'd1;
            S_target_mid_right    <= S_target_cx + 12'd1;
            S_target_mid_top      <= S_target_cy - 12'd1;
            S_target_mid_bottom   <= S_target_cy + 12'd1;
        end
    end

    // Pipeline stage A: coordinate/region decode only.  The bitmap ROMs are
    // deliberately kept out of this stage so the long coordinate-to-ROM-to-
    // colour path is cut at a register boundary.
    always @(*) begin
        S_target_box_on    = 1'b0;
        S_target_box_color = 24'd0;
        S_logo_on          = 1'b0;
        S_logo_addr        = 15'd0;
        S_dyn_x            = 12'd0;
        S_dyn_y            = 12'd0;
        S_dyn_on           = 1'b0;
        S_dyn_digit_x      = 12'd0;
        S_dyn_dec_digit    = 4'd0;
        S_pf_char          = 8'd0;
        S_pf_local_x       = 5'd0;
        S_pf_local_y       = 5'd0;
        S_pf_col           = 4'd0;
        S_pf_row           = 4'd0;
        S_pf_on            = 1'b0;
        S_mode_char_idx    = 5'd0;
        S_mode_char        = 8'h20;
        S_mode_col         = 4'd0;
        S_mode_row         = 4'd0;
        S_mode_on          = 1'b0;

        // Draw first so the measured target box and red target cross retain
        // priority wherever they overlap this fixed screen-centre dot.
        if(S_center_reference_2d) begin
            S_target_box_on    = 1'b1;
            S_target_box_color = 24'h00ffff;
        end

        if(S_target_border_2d) begin
            S_target_box_on    = 1'b1;
            S_target_box_color = S_motion_active ? 24'h00d8ff :
                                 ((S_target_confidence >= 8'd192) ? 24'h00ff40 : 24'hffd000);
        end
        if(S_target_cross_2d) begin
            S_target_box_on    = 1'b1;
            S_target_box_color = 24'hff3030;
        end

        if(S_fence_border_2d) begin
            S_target_box_on=1'b1;
            case(S_fence_state)
                2'd0:S_target_box_color=24'h00ff40;
                2'd1:S_target_box_color=24'hffd000;
                2'd2:S_target_box_color=24'h00d8ff;
                default:S_target_box_color=24'hff3030;
            endcase
        end
        if(S_pass_line_a_2d) begin
            S_target_box_on=1'b1;S_target_box_color=24'hffd000;
        end
        if(S_pass_line_b_2d) begin
            S_target_box_on=1'b1;S_target_box_color=24'h00d8ff;
        end

        if(S_video_de_2d) begin
            if((S_x_2d >= LOGO_X) && (S_x_2d < (LOGO_X + LOGO_W)) &&
               (S_y_2d >= LOGO_Y) && (S_y_2d < (LOGO_Y + LOGO_H))) begin
                S_logo_on   = 1'b1;
                S_logo_addr = ((S_x_2d==LOGO_X) && (S_y_2d==LOGO_Y)) ?
                              15'd0 : S_logo_stream_addr+15'd1;
            end

            if((S_x_2d >= DYN_OSD_X) &&
               (S_x_2d < (DYN_OSD_X + DYN_OSD_W)) &&
               (S_y_2d >= DYN_OSD_Y) &&
               (S_y_2d < (DYN_OSD_Y + DYN_OSD_H))) begin
                S_dyn_x = S_x_2d - DYN_OSD_X;
                S_dyn_y = S_y_2d - DYN_OSD_Y;

                if((S_dyn_x < 12'd108) && (S_dyn_y < 12'd24)) begin
                    if(S_dyn_x < 12'd48) begin
                        if(S_dyn_y >= CN_Y_OFFSET)
                            S_pf_local_y = S_dyn_y - CN_Y_OFFSET;
                        else
                            S_pf_local_y = 5'd31;

                        if(S_dyn_x < 12'd24) begin
                            S_pf_char    = CHAR_AN;
                            S_pf_local_x = S_dyn_x;
                        end else begin
                            S_pf_char    = CHAR_LU;
                            S_pf_local_x = S_dyn_x - 12'd24;
                        end

                        if((S_pf_local_x < 5'd24) && (S_pf_local_y < 5'd24)) begin
                            S_pf_col = F_scale_24_to_16(S_pf_local_x);
                            S_pf_row = F_scale_24_to_16(S_pf_local_y);
                            S_pf_on  = 1'b1;
                        end
                    end
                    else if((S_dyn_x >= 12'd52) && (S_dyn_x < 12'd100) &&
                            (S_dyn_y >= 12'd4) && (S_dyn_y < 12'd20)) begin
                        if(S_dyn_x < 12'd68) begin
                            S_pf_char    = CHAR_C;
                            S_pf_local_x = S_dyn_x - 12'd52;
                        end else if(S_dyn_x < 12'd84) begin
                            S_pf_char    = CHAR_N;
                            S_pf_local_x = S_dyn_x - 12'd68;
                        end else begin
                            S_pf_char    = CHAR_T;
                            S_pf_local_x = S_dyn_x - 12'd84;
                        end
                        S_pf_local_y = S_dyn_y - 12'd4;
                        S_pf_col      = S_pf_local_x[3:0];
                        S_pf_row      = S_pf_local_y[3:0];
                        S_pf_on       = 1'b1;
                    end
                    else if((S_dyn_x >= 12'd100) && (S_dyn_x < 12'd108) &&
                            (S_dyn_y >= 12'd4) && (S_dyn_y < 12'd20)) begin
                        S_pf_char    = CHAR_COLON;
                        S_pf_local_x = S_dyn_x - 12'd100;
                        S_pf_local_y = S_dyn_y - 12'd4;
                        S_pf_col      = S_pf_local_x[2:0] << 1;
                        S_pf_row      = S_pf_local_y[3:0];
                        S_pf_on       = 1'b1;
                    end
                end

                if((S_dyn_x >= 12'd110) && (S_dyn_x < 12'd200)) begin
                    if(S_dyn_x < 12'd125) begin
                        S_dyn_digit_x   = S_dyn_x - 12'd110;
                        S_dyn_dec_digit = S_cnt_d5;
                    end else if(S_dyn_x < 12'd140) begin
                        S_dyn_digit_x   = S_dyn_x - 12'd125;
                        S_dyn_dec_digit = S_cnt_d4;
                    end else if(S_dyn_x < 12'd155) begin
                        S_dyn_digit_x   = S_dyn_x - 12'd140;
                        S_dyn_dec_digit = S_cnt_d3;
                    end else if(S_dyn_x < 12'd170) begin
                        S_dyn_digit_x   = S_dyn_x - 12'd155;
                        S_dyn_dec_digit = S_cnt_d2;
                    end else if(S_dyn_x < 12'd185) begin
                        S_dyn_digit_x   = S_dyn_x - 12'd170;
                        S_dyn_dec_digit = S_cnt_d1;
                    end else begin
                        S_dyn_digit_x   = S_dyn_x - 12'd185;
                        S_dyn_dec_digit = S_cnt_d0;
                    end

                    if((S_dyn_digit_x < 12'd15) && (S_dyn_y < 12'd24)) begin
                        S_pf_char    = CHAR_0 + {4'd0, S_dyn_dec_digit};
                        S_pf_local_x = S_dyn_digit_x[4:0];
                        S_pf_local_y = S_dyn_y[4:0];
                        S_pf_col      = S_pf_local_x[3:0];
                        S_pf_row      = F_scale_24_to_16(S_pf_local_y);
                        S_pf_on       = 1'b1;
                    end
                end
                S_dyn_on = S_pf_on;
            end

            if((S_x_2d >= MODE_OSD_X) &&
               (S_x_2d < (MODE_OSD_X + MODE_OSD_W)) &&
               (S_y_2d >= MODE_OSD_Y) &&
               (S_y_2d < (MODE_OSD_Y + 12'd16))) begin
                S_mode_char_idx = (S_x_2d - MODE_OSD_X) >> 4;
                S_mode_char = F_mode_char(S_mode_active,S_rgb_active,S_mode_char_idx[3:0]);
                S_mode_col = S_x_2d[3:0];
                S_mode_row = S_y_2d - MODE_OSD_Y;
                S_mode_on  = 1'b1;
            end
            if((S_x_2d >= MODE_OSD_X) && (S_x_2d < MODE_OSD_X+12'd96) &&
               (S_y_2d >= MODE_OSD_Y+12'd24) && (S_y_2d < MODE_OSD_Y+12'd40)) begin
                S_mode_char_idx=(S_x_2d-MODE_OSD_X)>>4;
                S_mode_char=F_detection_char(S_motion_active,S_mode_char_idx);
                if(!S_detection_active || !I_detection_enable) begin
                    case(S_mode_char_idx)
                        0:S_mode_char=8'h4f;
                        1,2:S_mode_char=8'h46;
                        default:S_mode_char=8'h20;
                    endcase
                end
                S_mode_col=S_x_2d[3:0];S_mode_row=S_y_2d-(MODE_OSD_Y+12'd24);
                S_mode_on=1'b1;
            end
            if(S_detection_active && I_detection_enable && S_motion_active &&
               (S_x_2d>=FENCE_OSD_X) && (S_x_2d<FENCE_OSD_X+12'd336) &&
               (S_y_2d>=FENCE_INFO_Y) && (S_y_2d<FENCE_INFO_Y+12'd16)) begin
                S_mode_char_idx=(S_x_2d-FENCE_OSD_X)>>4;
                S_mode_char=F_fence_line_char(S_fence_direction,S_passage_in_bcd,
                                              S_passage_out_bcd,S_mode_char_idx,S_fence_state);
                S_mode_col=S_x_2d[3:0];S_mode_row=S_y_2d-FENCE_INFO_Y;
                S_mode_on=1'b1;
            end
            if(S_detection_active && I_detection_enable && S_motion_active &&
               (S_x_2d>=FENCE_OSD_X) && (S_x_2d<FENCE_OSD_X+12'd256) &&
               (S_y_2d>=FENCE_INFO_Y+12'd16) && (S_y_2d<FENCE_INFO_Y+12'd32)) begin
                S_mode_char_idx=(S_x_2d-FENCE_OSD_X)>>4;
                S_mode_char=F_passage_line_char(S_passage_event,S_fence_entry_bcd,
                                                S_fence_dwell_bcd,S_mode_char_idx);
                S_mode_col=S_x_2d[3:0];S_mode_row=S_y_2d-(FENCE_INFO_Y+12'd16);
                S_mode_on=1'b1;
            end
            if(S_detection_active && I_detection_enable && !S_motion_active && !I_motion_mode &&
               (S_x_2d>=FENCE_OSD_X) && (S_x_2d<FENCE_OSD_X+12'd240) &&
               (S_y_2d>=FENCE_INFO_Y+12'd16) && (S_y_2d<FENCE_INFO_Y+12'd32)) begin
                S_mode_char_idx=(S_x_2d-FENCE_OSD_X)>>4;
                S_mode_char=F_latency_char(S_latency_done,S_latency_seconds_bcd,
                                           S_latency_running_seconds_bcd,S_mode_char_idx[3:0]);
                S_mode_col=S_x_2d[3:0];S_mode_row=S_y_2d-(FENCE_INFO_Y+12'd16);
                S_mode_on=1'b1;
            end
        end
    end

    // Register all bitmap addresses and enables before the ROM lookup.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_target_box_on_a    <= 1'b0;
            S_target_box_color_a <= 24'd0;
            S_logo_on_a          <= 1'b0;
            S_pf_on_a            <= 1'b0;
            S_pf_char_a          <= 8'd0;
            S_pf_col_a           <= 4'd0;
            S_pf_row_a           <= 4'd0;
            S_mode_on_a          <= 1'b0;
            S_mode_char_a        <= 8'h20;
            S_mode_col_a         <= 4'd0;
            S_mode_row_a         <= 4'd0;
        end else begin
            S_target_box_on_a    <= S_target_box_on;
            S_target_box_color_a <= S_target_box_color;
            S_logo_on_a          <= S_logo_on;
            S_pf_on_a            <= S_pf_on;
            S_pf_char_a          <= S_pf_char;
            S_pf_col_a           <= S_pf_col;
            S_pf_row_a           <= S_pf_row;
            S_mode_on_a          <= S_mode_on;
            S_mode_char_a        <= S_mode_char;
            S_mode_col_a         <= S_mode_col;
            S_mode_row_a         <= S_mode_row;
        end
    end

    // Pipeline stage B: shallow ROM-bit selection and fixed-priority colour mux.
    always @(*) begin
        S_osd_hit   = S_target_box_on_a;
        S_osd_color = S_target_box_color_a;
        if(S_logo_on_a && S_logo_pixel[24]) begin
            S_osd_hit   = 1'b1;
            S_osd_color = S_logo_pixel[23:0];
        end
        if(S_pf_on_a && S_pf_row_bits[4'd15 - S_pf_col_a]) begin
            S_osd_hit   = 1'b1;
            S_osd_color = COLOR_DYN_TEXT;
        end
        if(S_mode_on_a && S_mode_row_bits[4'd15 - S_mode_col_a]) begin
            S_osd_hit   = 1'b1;
            S_osd_color = 24'hffffff;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_video_vsync_1d <= 1'b0;
            S_video_vsync_2d <= 1'b0;
            S_video_vsync_3d <= 1'b0;
            S_video_vsync_4d <= 1'b0;
            S_video_hsync_1d <= 1'b0;
            S_video_hsync_2d <= 1'b0;
            S_video_hsync_3d <= 1'b0;
            S_video_hsync_4d <= 1'b0;
            S_video_de_1d    <= 1'b0;
            S_video_de_2d    <= 1'b0;
            S_video_de_3d    <= 1'b0;
            S_video_de_4d    <= 1'b0;
            S_video_data_1d  <= 24'd0;
            S_video_data_2d  <= 24'd0;
            S_video_data_3d  <= 24'd0;
            S_video_data_4d  <= 24'd0;
            S_osd_hit_3d     <= 1'b0;
            S_osd_color_3d   <= 24'd0;
            O_hdmi_vsync     <= 1'b0;
            O_hdmi_hsync     <= 1'b0;
            O_hdmi_de        <= 1'b0;
            O_hdmi_data      <= 24'd0;
        end
        else begin
            S_video_vsync_1d <= I_video_vsync;
            S_video_vsync_2d <= S_video_vsync_1d;
            S_video_vsync_3d <= S_video_vsync_2d;
            S_video_vsync_4d <= S_video_vsync_3d;
            O_hdmi_vsync     <= S_video_vsync_4d;

            S_video_hsync_1d <= I_video_hsync;
            S_video_hsync_2d <= S_video_hsync_1d;
            S_video_hsync_3d <= S_video_hsync_2d;
            S_video_hsync_4d <= S_video_hsync_3d;
            O_hdmi_hsync     <= S_video_hsync_4d;

            S_video_de_1d    <= I_video_de;
            S_video_de_2d    <= S_video_de_1d;
            S_video_de_3d    <= S_video_de_2d;
            S_video_de_4d    <= S_video_de_3d;
            O_hdmi_de        <= S_video_de_4d;
            S_video_data_1d  <= S_display_mode_data;
            S_video_data_2d  <= S_video_data_1d;
            S_video_data_3d  <= S_video_data_2d;
            S_video_data_4d  <= S_video_data_3d;
            S_osd_hit_3d     <= S_osd_hit;
            S_osd_color_3d   <= S_osd_color;

            if(DEBUG_MODE == 1) begin
                if(S_video_de_4d) begin
                    if(S_x_4d < 12'd128)
                        O_hdmi_data <= 24'hff0000;
                    else if(S_x_4d < 12'd256)
                        O_hdmi_data <= 24'h00ff00;
                    else if(S_x_4d < 12'd384)
                        O_hdmi_data <= 24'h0000ff;
                    else if(S_x_4d < 12'd512)
                        O_hdmi_data <= 24'hffff00;
                    else if(S_x_4d < 12'd640)
                        O_hdmi_data <= 24'h00ffff;
                    else if(S_x_4d < 12'd768)
                        O_hdmi_data <= 24'hff00ff;
                    else if(S_x_4d < 12'd896)
                        O_hdmi_data <= 24'hffffff;
                    else
                        O_hdmi_data <= 24'h202020;
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 2) begin
                if(S_video_de_4d) begin
                    case (I_debug_status)
                        4'b0000: O_hdmi_data <= 24'h000000;
                        4'b0001: O_hdmi_data <= 24'hff0000;
                        4'b0011: O_hdmi_data <= 24'hffff00;
                        4'b0111: O_hdmi_data <= 24'h00ff00;
                        4'b1111: O_hdmi_data <= S_video_data_4d;
                        default: O_hdmi_data <= 24'h0000ff;
                    endcase
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 3) begin
                if(S_video_de_4d) begin
                    if(S_x_4d < 12'd256)
                        O_hdmi_data <= I_debug_status[0] ? 24'hff8000 : 24'h201000;
                    else if(S_x_4d < 12'd512)
                        O_hdmi_data <= I_debug_status[1] ? 24'hffff00 : 24'h202000;
                    else if(S_x_4d < 12'd768)
                        O_hdmi_data <= I_debug_status[2] ? 24'hff0000 : 24'h200000;
                    else
                        O_hdmi_data <= I_debug_status[3] ? 24'h00ffff : 24'h002020;
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else begin
                if(S_osd_hit_3d)
                    O_hdmi_data <= S_osd_color_3d;
                else
                    O_hdmi_data <= S_video_data_4d;
            end
        end
    end

endmodule

// Convert the existing three-digit frame count to tenths of a second.
// At 60 frames/s, each 0.1 second represents 6 frames. Truncate fractions.
module latency_frames_to_seconds (
 input wire clk,rst_n,
 input wire [11:0] frames_bcd,
 output reg [11:0] seconds_bcd
);
 reg [11:0] seen_bcd;
 reg [9:0] remaining;
 reg [11:0] tenths_bcd;
 reg busy;
 wire [9:0] hundreds={6'd0,frames_bcd[11:8]};
 wire [9:0] tens={6'd0,frames_bcd[7:4]};
 wire [9:0] units={6'd0,frames_bcd[3:0]};
 wire [9:0] frames_binary=(hundreds<<6)+(hundreds<<5)+(hundreds<<2)+
                          (tens<<3)+(tens<<1)+units;
 function [11:0] bcd_increment;
  input [11:0] value;
  begin
   if(value[3:0]<4'd9) bcd_increment=value+12'h001;
   else if(value[7:4]<4'd9) bcd_increment={value[11:4]+8'h01,4'h0};
   else bcd_increment={value[11:8]+4'h1,8'h00};
  end
 endfunction
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   seen_bcd<=12'hfff;remaining<=0;tenths_bcd<=0;seconds_bcd<=0;busy<=0;
  end else if(frames_bcd!=seen_bcd) begin
   seen_bcd<=frames_bcd;remaining<=frames_binary;tenths_bcd<=0;busy<=1;
  end else if(busy) begin
   if(remaining>=10'd6) begin
    remaining<=remaining-10'd6;tenths_bcd<=bcd_increment(tenths_bcd);
   end else begin seconds_bcd<=tenths_bcd;busy<=0;end
  end
 end
endmodule

// v4 detector: robust black/white paper-target localization for visualization.
// The data path remains one pixel per clock.  A 2-of-3 horizontal filter
// suppresses isolated noise, two candidates are retained per row, and an
// established vertical track survives short digit/divider occlusions.  The
// completed-frame result is published at VSYNC, so coordinates never tear.
module bw_ring_detector_stream_v4 #(
    parameter [7:0]  DARK_THRESHOLD = 8'd112,
    parameter [11:0] MAX_RUN        = 12'd180,
    parameter [7:0]  MIN_LINES      = 8'd8,
    parameter [11:0] MAX_ROW_GAP    = 12'd8
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_vsync,
    input  wire        I_de,
    input  wire [23:0] I_rgb,
    output reg         O_valid,
    output reg  [11:0] O_center_x,
    output reg  [11:0] O_center_y,
    output reg  [11:0] O_radius,
    output reg  [7:0]  O_confidence
);
    wire [9:0] S_luma_sum = {2'b0,I_rgb[23:16]} +
                            {2'b0,I_rgb[15:8]} +
                            {2'b0,I_rgb[7:0]};
    // A cheap 8-bit luma approximation used by the frame sampler.  The
    // detector threshold is learned from the previous frame, so dark Normal
    // video and high-contrast Sobel/Binary video share one stable classifier.
    wire [7:0] S_luma = {2'b0,I_rgb[23:18]} +
                        {1'b0,I_rgb[15:9]} +
                        {2'b0,I_rgb[7:2]};
    wire [7:0] S_rg_diff = (I_rgb[23:16] > I_rgb[15:8]) ?
                           (I_rgb[23:16]-I_rgb[15:8]) :
                           (I_rgb[15:8]-I_rgb[23:16]);
    wire [7:0] S_gb_diff = (I_rgb[15:8] > I_rgb[7:0]) ?
                           (I_rgb[15:8]-I_rgb[7:0]) :
                           (I_rgb[7:0]-I_rgb[15:8]);
    wire [7:0] S_rb_diff = (I_rgb[23:16] > I_rgb[7:0]) ?
                           (I_rgb[23:16]-I_rgb[7:0]) :
                           (I_rgb[7:0]-I_rgb[23:16]);
    wire       S_neutral = (S_rg_diff < 8'd80) &&
                           (S_gb_diff < 8'd80) &&
                           (S_rb_diff < 8'd80);
    reg  [7:0] S_dark_threshold;
    reg [19:0] S_frame_luma_sum;
    reg [10:0] S_frame_sample_count;
    wire [9:0] S_frame_mean_est = S_frame_luma_sum[19:10];
    wire [9:0] S_threshold_est = S_frame_mean_est - (S_frame_mean_est >> 2);
    wire [7:0] S_threshold_candidate =
        (S_threshold_est < 10'd20)  ? 8'd20  :
        (S_threshold_est > 10'd176) ? 8'd176 : S_threshold_est[7:0];
    // 1/8 frame-domain IIR.  Directly replacing the threshold every frame
    // caused the segmented ring width and therefore the measured box to pulse.
    wire [10:0] S_threshold_filter_acc =
        ({3'd0,S_dark_threshold} << 2) +
        ({3'd0,S_dark_threshold} << 1) +
         {3'd0,S_dark_threshold} +
         {3'd0,S_threshold_candidate} + 11'd4;
    wire [9:0] S_dark_limit = {2'b0,S_dark_threshold} +
                              ({2'b0,S_dark_threshold} << 1);
    wire       S_dark_raw = (S_luma_sum < S_dark_limit) &&
                            S_neutral;

    reg        S_dark_d1, S_dark_d2;
    wire       S_dark = (S_dark_raw && S_dark_d1) ||
                        (S_dark_raw && S_dark_d2) ||
                        (S_dark_d1 && S_dark_d2);

    reg        S_de_d, S_vsync_d;
    reg [11:0] S_x, S_y;
    reg        S_run_color;
    reg [11:0] S_run_len;
    reg        S_hist_color0, S_hist_color1, S_hist_color2;
    reg        S_hist_color3, S_hist_color4;
    reg [11:0] S_hist_len0, S_hist_len1, S_hist_len2;
    reg [11:0] S_hist_len3, S_hist_len4;

    reg        S_row_primary_valid, S_row_secondary_valid;
    reg [11:0] S_row_primary_x, S_row_primary_r, S_row_primary_center_w;
    reg [11:0] S_row_secondary_x, S_row_secondary_r, S_row_secondary_center_w;
    reg [13:0] S_row_primary_quality, S_row_secondary_quality;

    reg [11:0] S_cluster_x, S_cluster_y0, S_cluster_y1, S_cluster_r;
    reg [11:0] S_cluster_peak_x, S_cluster_peak_y, S_cluster_peak_r;
    reg [7:0]  S_cluster_lines;
    reg        S_row_commit_pending, S_work_match;
    reg [11:0] S_work_x, S_work_r, S_work_y;
    reg        S_best_commit_pending;
    reg [7:0]  S_commit_lines;
    reg [11:0] S_commit_x, S_commit_r, S_commit_y0, S_commit_y1;
    reg [11:0] S_commit_peak_y;
    reg        S_best_apply_pending;
    reg [7:0]  S_apply_lines;
    reg [11:0] S_apply_x, S_apply_r, S_apply_y0, S_apply_y1;
    reg [11:0] S_apply_peak_y;
    reg [7:0]  S_best_lines;
    reg [11:0] S_best_x, S_best_y0, S_best_y1, S_best_r;
    reg [11:0] S_best_peak_y;
    reg [2:0]  S_lost_frames;
    reg [1:0]  S_mismatch_frames;
    reg [11:0] S_pending_x, S_pending_y, S_pending_r;
    reg [11:0] S_last_meas_x, S_last_meas_y;
    reg [6:0]  S_motion_x, S_motion_y;
    reg [11:0] S_meas_x_d1, S_meas_x_d2;
    reg [11:0] S_meas_y_d1, S_meas_y_d2;
    reg [11:0] S_meas_r_d1, S_meas_r_d2;
    reg [11:0] S_last_median_x, S_last_median_y;
    reg [2:0]  S_static_frames;

    function [11:0] F_median3;
        input [11:0] a;
        input [11:0] b;
        input [11:0] c;
        begin
            if(a > b) begin
                if(b > c)
                    F_median3 = b;
                else if(a > c)
                    F_median3 = c;
                else
                    F_median3 = a;
            end else begin
                if(a > c)
                    F_median3 = a;
                else if(b > c)
                    F_median3 = c;
                else
                    F_median3 = b;
            end
        end
    endfunction

    // Position deadband follows target size.  A large nearby target naturally
    // has more edge/print quantisation than a small distant one, so a fixed
    // three-pixel threshold cannot keep both cases quiet.  Genuine motion is
    // still followed at one half of the measured displacement.
    function [11:0] F_smooth_position;
        input [11:0] old_value;
        input [11:0] new_value;
        input [11:0] scale_value;
        input [6:0]  motion_value;
        input        static_lock;
        reg   [11:0] delta;
        reg   [11:0] deadband;
        reg   [11:0] slow_limit;
        begin
            deadband = 12'd8 + (scale_value >> 4);
if(deadband > 12'd24)
    deadband = 12'd24;
            slow_limit = deadband + (deadband << 1);
            delta = (new_value >= old_value) ?
                    (new_value-old_value) : (old_value-new_value);
            if((delta <= deadband) ||
               (static_lock && (delta <= (deadband << 1))))
                F_smooth_position = old_value;
            else if(new_value >= old_value) begin
                if(motion_value <= 7'd3)
                    F_smooth_position = old_value + ((delta + 12'd15) >> 4);
                else if(delta <= slow_limit)
                    F_smooth_position = old_value + ((delta + 12'd3) >> 2);
                else
                    F_smooth_position = old_value + ((delta + 12'd1) >> 1);
            end else begin
                if(motion_value <= 7'd3)
                    F_smooth_position = old_value - ((delta + 12'd15) >> 4);
                else if(delta <= slow_limit)
                    F_smooth_position = old_value - ((delta + 12'd3) >> 2);
                else
                    F_smooth_position = old_value - ((delta + 12'd1) >> 1);
            end
        end
    endfunction

    // Box size is filtered more strongly than its centre.  This removes the
    // visible "breathing" caused by adjacent scan rows winning on alternate
    // frames without slowing horizontal/vertical following.
    function [11:0] F_smooth_radius;
        input [11:0] old_value;
        input [11:0] new_value;
        reg   [11:0] delta;
        reg   [11:0] deadband;
        begin
            deadband = 12'd8 + (old_value >> 4);
if(deadband > 12'd20)
    deadband = 12'd20;
            delta = (new_value >= old_value) ?
                    (new_value-old_value) : (old_value-new_value);
            if(delta <= deadband)
                F_smooth_radius = old_value;
            else if(new_value >= old_value)
                F_smooth_radius = old_value + ((delta + 12'd15) >> 4);
            else
                F_smooth_radius = old_value - ((delta + 12'd15) >> 4);
        end
    endfunction

    wire [11:0] S_outer_black_diff = (S_hist_len0 > S_hist_len4) ?
                                     (S_hist_len0-S_hist_len4) :
                                     (S_hist_len4-S_hist_len0);
    wire [11:0] S_outer_black_min = (S_hist_len0 < S_hist_len4) ?
                                    S_hist_len0 : S_hist_len4;
    wire [11:0] S_inner_white_diff = (S_hist_len1 > S_hist_len3) ?
                                     (S_hist_len1-S_hist_len3) :
                                     (S_hist_len3-S_hist_len1);
    // The majority filter is centred one pixel behind S_x.
    wire [11:0] S_candidate_x = S_x - 12'd1 - S_run_len - S_hist_len0 -
                                S_hist_len1 - (S_hist_len2 >> 1);
    wire [11:0] S_candidate_r = S_hist_len0 + S_hist_len1 +
                                (S_hist_len2 >> 1);
    wire [13:0] S_quality_base = {S_hist_len2,2'b00} +
                                 {2'b0,S_outer_black_min};
    wire [13:0] S_quality_penalty = {1'b0,S_outer_black_diff,1'b0} +
                                    {2'b0,S_inner_white_diff};
    wire [13:0] S_candidate_quality = S_quality_base - S_quality_penalty;

    wire S_signature = !S_run_color &&
                       S_hist_color0 && !S_hist_color1 &&
                       S_hist_color2 && !S_hist_color3 &&
                       S_hist_color4 &&
                       (S_hist_len0 >= 12'd5) &&
                       (S_hist_len4 >= 12'd5) &&
                       (S_hist_len2 >= 12'd8) &&
                       (S_hist_len0 <= MAX_RUN) &&
                       (S_hist_len4 <= MAX_RUN) &&
                       (S_hist_len2 <= (MAX_RUN << 1)) &&
                       (S_hist_len1 >= 12'd2) &&
                       (S_hist_len3 >= 12'd2) &&
                       (S_hist_len1 <= 12'd32) &&
                       (S_hist_len3 <= 12'd32) &&
                       (S_hist_len0 >= (S_hist_len1 << 1)) &&
                       (S_hist_len4 >= (S_hist_len3 << 1)) &&
                       (S_hist_len2 >= (S_hist_len1 << 1)) &&
                       (S_hist_len2 >= (S_hist_len3 << 1)) &&
                       (S_outer_black_diff <= ((S_outer_black_min >> 1) + 12'd4)) &&
                       (S_inner_white_diff <= (((S_hist_len1 < S_hist_len3 ?
                                                S_hist_len1 : S_hist_len3) >> 1) + 12'd3));

    wire S_better_primary = !S_row_primary_valid ||
                            (S_candidate_quality > S_row_primary_quality) ||
                            ((S_candidate_quality == S_row_primary_quality) &&
                             ((S_hist_len2 > S_row_primary_center_w) ||
                              ((S_hist_len2 == S_row_primary_center_w) &&
                               (S_candidate_r > S_row_primary_r))));
    wire S_better_secondary = !S_row_secondary_valid ||
                              (S_candidate_quality > S_row_secondary_quality) ||
                              ((S_candidate_quality == S_row_secondary_quality) &&
                               ((S_hist_len2 > S_row_secondary_center_w) ||
                                ((S_hist_len2 == S_row_secondary_center_w) &&
                                 (S_candidate_r > S_row_secondary_r))));

    wire [11:0] S_primary_x_diff = (S_row_primary_x > S_cluster_x) ?
                                   (S_row_primary_x-S_cluster_x) :
                                   (S_cluster_x-S_row_primary_x);
    wire [11:0] S_primary_r_diff = (S_row_primary_r > S_cluster_r) ?
                                   (S_row_primary_r-S_cluster_r) :
                                   (S_cluster_r-S_row_primary_r);
    wire [11:0] S_secondary_x_diff = (S_row_secondary_x > S_cluster_x) ?
                                     (S_row_secondary_x-S_cluster_x) :
                                     (S_cluster_x-S_row_secondary_x);
    wire [11:0] S_secondary_r_diff = (S_row_secondary_r > S_cluster_r) ?
                                     (S_row_secondary_r-S_cluster_r) :
                                     (S_cluster_r-S_row_secondary_r);
    wire S_primary_match = S_row_primary_valid && (S_cluster_lines != 0) &&
                           (S_y <= S_cluster_y1 + MAX_ROW_GAP) &&
                           (S_primary_x_diff <= (S_cluster_r >> 2) + 12'd6) &&
                           (S_primary_r_diff <= (S_cluster_r >> 2) + 12'd5);
    wire S_secondary_match = S_row_secondary_valid && (S_cluster_lines != 0) &&
                             (S_y <= S_cluster_y1 + MAX_ROW_GAP) &&
                             (S_secondary_x_diff <= (S_cluster_r >> 2) + 12'd6) &&
                             (S_secondary_r_diff <= (S_cluster_r >> 2) + 12'd5);
    wire S_use_secondary = !S_primary_match && S_secondary_match;
    wire S_selected_match = S_primary_match || S_secondary_match;
    wire [11:0] S_selected_x = S_use_secondary ? S_row_secondary_x :
                                                S_row_primary_x;
    wire [11:0] S_selected_r = S_use_secondary ? S_row_secondary_r :
                                                S_row_primary_r;
    wire [11:0] S_work_x_delta = (S_work_x > S_cluster_x) ?
                                 (S_work_x-S_cluster_x) :
                                 (S_cluster_x-S_work_x);
    wire [11:0] S_work_r_delta = (S_work_r > S_cluster_r) ?
                                 (S_work_r-S_cluster_r) :
                                 (S_cluster_r-S_work_r);
    wire [11:0] S_track_next_x = (S_work_x >= S_cluster_x) ?
                                 (S_cluster_x + ((S_work_x_delta + 12'd2) >> 2)) :
                                 (S_cluster_x - ((S_work_x_delta + 12'd2) >> 2));
    wire [11:0] S_track_next_r = (S_work_r >= S_cluster_r) ?
                                 (S_cluster_r + ((S_work_r_delta + 12'd2) >> 2)) :
                                 (S_cluster_r - ((S_work_r_delta + 12'd2) >> 2));
    wire [7:0]  S_cluster_next_lines = S_cluster_lines + 8'd1;
    wire [11:0] S_cluster_next_span = S_work_y - S_cluster_y0 + 12'd1;
    wire [11:0] S_required_lines_raw = S_track_next_r >> 2;
    wire [11:0] S_required_lines = (S_required_lines_raw < {4'd0,MIN_LINES}) ?
                                   {4'd0,MIN_LINES} : S_required_lines_raw;
    wire [11:0] S_cluster_next_peak_r = (S_work_r > S_cluster_peak_r) ?
                                        S_work_r : S_cluster_peak_r;
    wire [11:0] S_cluster_next_peak_x = (S_work_r > S_cluster_peak_r) ?
                                        S_work_x : S_cluster_peak_x;
    wire [11:0] S_cluster_next_peak_y = (S_work_r > S_cluster_peak_r) ?
                                        S_work_y : S_cluster_peak_y;
    // The row with the largest observed radius is the geometric horizontal
    // diameter of a circle/ellipse and is less biased by broken Sobel arcs
    // than averaging the first and last matching rows.
    wire [11:0] S_best_center_y = S_best_peak_y;
    wire [11:0] S_median_x = F_median3(S_meas_x_d1,S_meas_x_d2,S_best_x);
    wire [11:0] S_median_y = F_median3(S_meas_y_d1,S_meas_y_d2,S_best_center_y);
    wire [11:0] S_median_r = F_median3(S_meas_r_d1,S_meas_r_d2,S_best_r);
    wire [11:0] S_median_step_x = (S_median_x > S_last_median_x) ?
                                  (S_median_x-S_last_median_x) :
                                  (S_last_median_x-S_median_x);
    wire [11:0] S_median_step_y = (S_median_y > S_last_median_y) ?
                                  (S_median_y-S_last_median_y) :
                                  (S_last_median_y-S_median_y);
    wire [11:0] S_static_limit = 12'd10 + (O_radius >> 4);
    wire S_measurement_static = (S_median_step_x <= S_static_limit) &&
                                (S_median_step_y <= S_static_limit);
    wire S_static_lock = (S_static_frames >= 3'd2);
    wire [11:0] S_best_required_raw = S_best_r >> 2;
    wire [11:0] S_best_required = (S_best_required_raw < {4'd0,MIN_LINES}) ?
                                  {4'd0,MIN_LINES} : S_best_required_raw;
    wire S_best_valid = (S_best_lines >= MIN_LINES) &&
                        ({4'd0,S_best_lines} >= S_best_required) &&
                        (S_best_r >= 12'd8) &&
                        (S_best_x > S_best_r) &&
                        (S_best_x + S_best_r < 12'd1280) &&
                        (S_best_center_y > S_best_r) &&
                        (S_best_center_y + S_best_r < 12'd720);
    wire [11:0] S_output_x_diff = (S_best_x > O_center_x) ?
                                  (S_best_x-O_center_x) : (O_center_x-S_best_x);
    wire [11:0] S_output_y_diff = (S_best_center_y > O_center_y) ?
                                  (S_best_center_y-O_center_y) :
                                  (O_center_y-S_best_center_y);
    wire [11:0] S_output_r_diff = (S_best_r > O_radius) ?
                                  (S_best_r-O_radius) : (O_radius-S_best_r);
    wire [11:0] S_meas_x_diff = (S_best_x > S_last_meas_x) ?
                                (S_best_x-S_last_meas_x) :
                                (S_last_meas_x-S_best_x);
    wire [11:0] S_meas_y_diff = (S_best_center_y > S_last_meas_y) ?
                                (S_best_center_y-S_last_meas_y) :
                                (S_last_meas_y-S_best_center_y);
    wire [6:0] S_meas_x_step = (S_meas_x_diff > 12'd64) ?
                               7'd64 : S_meas_x_diff[6:0];
    wire [6:0] S_meas_y_step = (S_meas_y_diff > 12'd64) ?
                               7'd64 : S_meas_y_diff[6:0];
    wire [9:0] S_motion_x_acc = {3'd0,S_motion_x} +
                                ({3'd0,S_motion_x} << 1) +
                                {3'd0,S_meas_x_step} + 10'd2;
    wire [9:0] S_motion_y_acc = {3'd0,S_motion_y} +
                                ({3'd0,S_motion_y} << 1) +
                                {3'd0,S_meas_y_step} + 10'd2;
    wire [13:0] S_track_x_limit = {2'd0,(O_radius >> 1)} + 14'd24 +
                                  ({7'd0,S_motion_x} << 1);
    wire [13:0] S_track_y_limit = {2'd0,(O_radius >> 1)} + 14'd24 +
                                  ({7'd0,S_motion_y} << 1);
    wire [11:0] S_track_r_limit = (O_radius >> 1) + 12'd16;
    wire S_track_match = ({2'd0,S_output_x_diff} <= S_track_x_limit) &&
                         ({2'd0,S_output_y_diff} <= S_track_y_limit) &&
                         (S_output_r_diff <= S_track_r_limit);
    wire [11:0] S_pending_x_diff = (S_best_x > S_pending_x) ?
                                   (S_best_x-S_pending_x) :
                                   (S_pending_x-S_best_x);
    wire [11:0] S_pending_y_diff = (S_best_center_y > S_pending_y) ?
                                   (S_best_center_y-S_pending_y) :
                                   (S_pending_y-S_best_center_y);
    wire [11:0] S_pending_r_diff = (S_best_r > S_pending_r) ?
                                   (S_best_r-S_pending_r) :
                                   (S_pending_r-S_best_r);
    wire S_pending_match = (S_pending_x_diff <= ((S_pending_r >> 2) + 12'd12)) &&
                           (S_pending_y_diff <= ((S_pending_r >> 2) + 12'd12)) &&
                           (S_pending_r_diff <= ((S_pending_r >> 2) + 12'd8));
    wire [7:0] S_frame_confidence =
        ({4'd0,S_best_lines} >= (S_best_required << 1)) ? 8'd255 :
        ({4'd0,S_best_lines} >= S_best_required + 12'd4) ? 8'd192 :
        8'd128;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_dark_threshold <= DARK_THRESHOLD;
            S_frame_luma_sum <= 20'd0;
            S_frame_sample_count <= 11'd0;
            S_dark_d1 <= 1'b0; S_dark_d2 <= 1'b0;
            S_de_d <= 1'b0; S_vsync_d <= 1'b0;
            S_x <= 12'd0; S_y <= 12'd0;
            S_run_color <= 1'b0; S_run_len <= 12'd0;
            S_hist_color0 <= 1'b0; S_hist_color1 <= 1'b0;
            S_hist_color2 <= 1'b0; S_hist_color3 <= 1'b0;
            S_hist_color4 <= 1'b0;
            S_hist_len0 <= 12'd0; S_hist_len1 <= 12'd0;
            S_hist_len2 <= 12'd0; S_hist_len3 <= 12'd0;
            S_hist_len4 <= 12'd0;
            S_row_primary_valid <= 1'b0; S_row_secondary_valid <= 1'b0;
            S_row_primary_x <= 12'd0; S_row_primary_r <= 12'd0;
            S_row_primary_center_w <= 12'd0; S_row_primary_quality <= 14'd0;
            S_row_secondary_x <= 12'd0; S_row_secondary_r <= 12'd0;
            S_row_secondary_center_w <= 12'd0; S_row_secondary_quality <= 14'd0;
            S_cluster_x <= 12'd0; S_cluster_y0 <= 12'd0;
            S_cluster_y1 <= 12'd0; S_cluster_r <= 12'd0;
            S_cluster_peak_x <= 12'd0; S_cluster_peak_y <= 12'd0;
            S_cluster_peak_r <= 12'd0; S_cluster_lines <= 8'd0;
            S_row_commit_pending <= 1'b0; S_work_match <= 1'b0;
            S_work_x <= 12'd0; S_work_r <= 12'd0; S_work_y <= 12'd0;
            S_best_commit_pending <= 1'b0; S_commit_lines <= 8'd0;
            S_commit_x <= 12'd0; S_commit_r <= 12'd0;
            S_commit_y0 <= 12'd0; S_commit_y1 <= 12'd0;
            S_commit_peak_y <= 12'd0;
            S_best_apply_pending <= 1'b0; S_apply_lines <= 8'd0;
            S_apply_x <= 12'd0; S_apply_r <= 12'd0;
            S_apply_y0 <= 12'd0; S_apply_y1 <= 12'd0;
            S_apply_peak_y <= 12'd0;
            S_best_lines <= 8'd0; S_best_x <= 12'd0;
            S_best_y0 <= 12'd0; S_best_y1 <= 12'd0; S_best_r <= 12'd0;
            S_best_peak_y <= 12'd0;
            S_lost_frames <= 3'd0; S_mismatch_frames <= 2'd0;
            S_pending_x <= 12'd0; S_pending_y <= 12'd0;
            S_pending_r <= 12'd0;
            S_last_meas_x <= 12'd0; S_last_meas_y <= 12'd0;
            S_motion_x <= 7'd0; S_motion_y <= 7'd0;
            S_meas_x_d1 <= 12'd0; S_meas_x_d2 <= 12'd0;
            S_meas_y_d1 <= 12'd0; S_meas_y_d2 <= 12'd0;
            S_meas_r_d1 <= 12'd0; S_meas_r_d2 <= 12'd0;
            S_last_median_x <= 12'd0; S_last_median_y <= 12'd0;
            S_static_frames <= 3'd0;
            O_valid <= 1'b0; O_center_x <= 12'd0;
            O_center_y <= 12'd0; O_radius <= 12'd0;
            O_confidence <= 8'd0;
        end else begin
            S_de_d <= I_de;
            S_vsync_d <= I_vsync;

            if(I_vsync && !S_vsync_d) begin
                if(S_frame_sample_count >= 11'd256) begin
                    S_dark_threshold <= S_threshold_filter_acc[10:3];
                end
                S_frame_luma_sum <= 20'd0;
                S_frame_sample_count <= 11'd0;
                if(S_best_valid) begin
                    S_lost_frames <= 3'd0;
                    if(!O_valid) begin
                        // Fast acquisition: the first reliable frame is shown
                        // immediately rather than waiting for a history window.
                        O_valid <= 1'b1;
                        O_center_x <= S_best_x;
                        O_center_y <= S_best_center_y;
                        O_radius <= S_best_r;
                        // A geometrically valid measurement is a firm lock.
                        // Yellow is reserved for multi-frame coasting, rather
                        // than for a single low-line motion-blurred frame.
                        O_confidence <= (S_frame_confidence < 8'd192) ?
                                        8'd192 : S_frame_confidence;
                        S_mismatch_frames <= 2'd0;
                        S_last_meas_x <= S_best_x;
                        S_last_meas_y <= S_best_center_y;
                        S_motion_x <= 7'd0;
                        S_motion_y <= 7'd0;
                        S_meas_x_d1 <= S_best_x; S_meas_x_d2 <= S_best_x;
                        S_meas_y_d1 <= S_best_center_y; S_meas_y_d2 <= S_best_center_y;
                        S_meas_r_d1 <= S_best_r; S_meas_r_d2 <= S_best_r;
                        S_last_median_x <= S_best_x;
                        S_last_median_y <= S_best_center_y;
                        S_static_frames <= 3'd0;
                    end else if(S_track_match) begin
                        // Three-frame median filtering removes isolated centre
                        // and radius spikes before the adaptive IIR.  After
                        // three quiet measurements a wider static hysteresis
                        // freezes the displayed centre; real motion clears it.
                        O_valid <= 1'b1;
                        O_center_x <= F_smooth_position(O_center_x,S_median_x,O_radius,
                                                       S_motion_x,S_static_lock);
                        O_center_y <= F_smooth_position(O_center_y,S_median_y,O_radius,
                                                       S_motion_y,S_static_lock);
                        O_radius <= F_smooth_radius(O_radius,S_median_r);
                        O_confidence <= (S_frame_confidence < 8'd192) ?
                                        8'd192 : S_frame_confidence;
                        S_mismatch_frames <= 2'd0;
                        S_last_meas_x <= S_best_x;
                        S_last_meas_y <= S_best_center_y;
                        S_motion_x <= S_motion_x_acc[8:2];
                        S_motion_y <= S_motion_y_acc[8:2];
                        S_meas_x_d2 <= S_meas_x_d1; S_meas_x_d1 <= S_best_x;
                        S_meas_y_d2 <= S_meas_y_d1; S_meas_y_d1 <= S_best_center_y;
                        S_meas_r_d2 <= S_meas_r_d1; S_meas_r_d1 <= S_best_r;
                        S_last_median_x <= S_median_x;
                        S_last_median_y <= S_median_y;
                        if(S_measurement_static) begin
    if(S_static_frames != 3'd7)
        S_static_frames <= S_static_frames + 1'b1;
end
else begin
    // 单帧检测噪声只降低一级，不立即取消静止锁定
    if(S_static_frames != 3'd0)
        S_static_frames <= S_static_frames - 1'b1;
end
                    end else if((S_mismatch_frames != 0) && S_pending_match) begin
                        // A large but repeatable displacement is real motion:
                        // re-acquire on its second consistent frame.
                        O_valid <= 1'b1;
                        O_center_x <= S_best_x;
                        O_center_y <= S_best_center_y;
                        O_radius <= S_best_r;
                        O_confidence <= (S_frame_confidence < 8'd192) ?
                                        8'd192 : S_frame_confidence;
                        S_mismatch_frames <= 2'd0;
                        S_last_meas_x <= S_best_x;
                        S_last_meas_y <= S_best_center_y;
                        S_motion_x <= (S_pending_x_diff > 12'd64) ?
                                      7'd64 : S_pending_x_diff[6:0];
                        S_motion_y <= (S_pending_y_diff > 12'd64) ?
                                      7'd64 : S_pending_y_diff[6:0];
                        S_meas_x_d1 <= S_best_x; S_meas_x_d2 <= S_best_x;
                        S_meas_y_d1 <= S_best_center_y; S_meas_y_d2 <= S_best_center_y;
                        S_meas_r_d1 <= S_best_r; S_meas_r_d2 <= S_best_r;
                        S_last_median_x <= S_best_x;
                        S_last_median_y <= S_best_center_y;
                        S_static_frames <= 3'd0;
                    end else begin
                        // Reject isolated candidate jumps.  Three unrelated
                        // jumps release the stale lock so acquisition can restart.
                        S_pending_x <= S_best_x;
                        S_pending_y <= S_best_center_y;
                        S_pending_r <= S_best_r;
                        if(S_mismatch_frames == 2'd2) begin
                            O_valid <= 1'b0;
                            O_confidence <= 8'd0;
                            S_mismatch_frames <= 2'd0;
                            S_motion_x <= 7'd0;
                            S_motion_y <= 7'd0;
                            S_static_frames <= 3'd0;
                        end else begin
                            S_mismatch_frames <= S_mismatch_frames + 2'd1;
                            if(S_mismatch_frames == 2'd0)
                                O_confidence <= (O_confidence < 8'd192) ?
                                                O_confidence : 8'd192;
                            else
                                O_confidence <= 8'd160;
                        end
                    end
                end else begin
                    S_mismatch_frames <= 2'd0;
                    // Brief motion-blur gaps keep the stable coordinates.  A
                    // single missing frame remains green, two through four
                    // missing frames are explicitly shown as a yellow coast,
                    // and the fifth releases the lock.
                    if(O_valid && (S_lost_frames < 3'd4)) begin
                        S_lost_frames <= S_lost_frames + 2'd1;
                        if(S_lost_frames == 3'd0)
                            O_confidence <= (O_confidence < 8'd192) ?
                                            O_confidence : 8'd192;
                        else
                            O_confidence <= 8'd160;
                        if(S_motion_x != 0)
                            S_motion_x <= S_motion_x - 7'd1;
                        if(S_motion_y != 0)
                            S_motion_y <= S_motion_y - 7'd1;
                    end else begin
                        O_valid <= 1'b0;
                        O_confidence <= 8'd0;
                        S_lost_frames <= 3'd0;
                        S_motion_x <= 7'd0;
                        S_motion_y <= 7'd0;
                        S_static_frames <= 3'd0;
                    end
                end
                S_y <= 12'd0;
                S_cluster_lines <= 8'd0;
                S_best_lines <= 8'd0;
                S_row_commit_pending <= 1'b0;
                S_best_commit_pending <= 1'b0;
                S_best_apply_pending <= 1'b0;
                S_row_primary_valid <= 1'b0;
                S_row_secondary_valid <= 1'b0;
            end else if(I_de && !S_de_d) begin
                if((S_x[4:0] == 5'd0) && (S_y[4:0] == 5'd0)) begin
                    S_frame_luma_sum <= S_frame_luma_sum + {12'd0,S_luma};
                    if(S_frame_sample_count != 11'h7ff)
                        S_frame_sample_count <= S_frame_sample_count + 11'd1;
                end
                S_x <= 12'd1;
                S_dark_d1 <= S_dark_raw;
                S_dark_d2 <= S_dark_raw;
                S_run_color <= S_dark_raw;
                S_run_len <= 12'd1;
                S_hist_len0 <= 12'd0; S_hist_len1 <= 12'd0;
                S_hist_len2 <= 12'd0; S_hist_len3 <= 12'd0;
                S_hist_len4 <= 12'd0;
                S_hist_color0 <= 1'b0; S_hist_color1 <= 1'b0;
                S_hist_color2 <= 1'b0; S_hist_color3 <= 1'b0;
                S_hist_color4 <= 1'b0;
                S_row_primary_valid <= 1'b0;
                S_row_secondary_valid <= 1'b0;
            end else if(I_de) begin
                if((S_x[4:0] == 5'd0) && (S_y[4:0] == 5'd0)) begin
                    S_frame_luma_sum <= S_frame_luma_sum + {12'd0,S_luma};
                    if(S_frame_sample_count != 11'h7ff)
                        S_frame_sample_count <= S_frame_sample_count + 11'd1;
                end
                S_x <= S_x + 12'd1;
                S_dark_d2 <= S_dark_d1;
                S_dark_d1 <= S_dark_raw;
                if(S_dark == S_run_color) begin
                    if(S_run_len != 12'hfff)
                        S_run_len <= S_run_len + 12'd1;
                end else begin
                    if(S_signature &&
                       (S_candidate_x > S_candidate_r) &&
                       (S_candidate_x + S_candidate_r < 12'd1280)) begin
                        if(S_better_primary) begin
                            S_row_secondary_valid <= S_row_primary_valid;
                            S_row_secondary_x <= S_row_primary_x;
                            S_row_secondary_r <= S_row_primary_r;
                            S_row_secondary_center_w <= S_row_primary_center_w;
                            S_row_secondary_quality <= S_row_primary_quality;
                            S_row_primary_valid <= 1'b1;
                            S_row_primary_x <= S_candidate_x;
                            S_row_primary_r <= S_candidate_r;
                            S_row_primary_center_w <= S_hist_len2;
                            S_row_primary_quality <= S_candidate_quality;
                        end else if(S_better_secondary) begin
                            S_row_secondary_valid <= 1'b1;
                            S_row_secondary_x <= S_candidate_x;
                            S_row_secondary_r <= S_candidate_r;
                            S_row_secondary_center_w <= S_hist_len2;
                            S_row_secondary_quality <= S_candidate_quality;
                        end
                    end
                    S_hist_color4 <= S_hist_color3;
                    S_hist_color3 <= S_hist_color2;
                    S_hist_color2 <= S_hist_color1;
                    S_hist_color1 <= S_hist_color0;
                    S_hist_color0 <= S_run_color;
                    S_hist_len4 <= S_hist_len3;
                    S_hist_len3 <= S_hist_len2;
                    S_hist_len2 <= S_hist_len1;
                    S_hist_len1 <= S_hist_len0;
                    S_hist_len0 <= S_run_len;
                    S_run_color <= S_dark;
                    S_run_len <= 12'd1;
                end
            end else begin
                // The best target write-back is a third blanking-stage.  It
                // is unconditional here, keeping ranking comparators off the
                // S_best_* register inputs highlighted by timing analysis.
                if(S_best_apply_pending) begin
                    S_best_lines <= S_apply_lines;
                    S_best_x <= S_apply_x;
                    S_best_r <= S_apply_r;
                    S_best_y0 <= S_apply_y0;
                    S_best_y1 <= S_apply_y1;
                    S_best_peak_y <= S_apply_peak_y;
                    S_best_apply_pending <= 1'b0;
                end

                // Best-target comparison is isolated in its own blanking
                // clock and only latches a write-back payload.
                if(S_best_commit_pending) begin
                    if((S_commit_lines > S_best_lines) ||
                       ((S_commit_lines == S_best_lines) &&
                        (S_commit_r > S_best_r))) begin
                        S_best_apply_pending <= 1'b1;
                        S_apply_lines <= S_commit_lines;
                        S_apply_x <= S_commit_x;
                        S_apply_r <= S_commit_r;
                        S_apply_y0 <= S_commit_y0;
                        S_apply_y1 <= S_commit_y1;
                        S_apply_peak_y <= S_commit_peak_y;
                    end
                    S_best_commit_pending <= 1'b0;
                end

                // Row tracking is also performed during horizontal blanking.
                if(S_row_commit_pending) begin
                    if(S_work_match) begin
                        S_cluster_x <= S_track_next_x;
                        S_cluster_r <= S_track_next_r;
                        S_cluster_y1 <= S_work_y;
                        S_cluster_peak_x <= S_cluster_next_peak_x;
                        S_cluster_peak_y <= S_cluster_next_peak_y;
                        S_cluster_peak_r <= S_cluster_next_peak_r;
                        if(S_cluster_lines != 8'hff)
                            S_cluster_lines <= S_cluster_next_lines;
                        if(({4'd0,S_cluster_next_lines} >= S_required_lines) &&
                           (({4'd0,S_cluster_next_lines} << 1) >= S_cluster_next_span)) begin
                            S_best_commit_pending <= 1'b1;
                            S_commit_lines <= S_cluster_next_lines;
                            S_commit_x <= S_cluster_next_peak_x;
                            S_commit_r <= S_cluster_next_peak_r;
                            S_commit_y0 <= S_cluster_y0;
                            S_commit_y1 <= S_work_y;
                            S_commit_peak_y <= S_cluster_next_peak_y;
                        end
                    end else begin
                        S_cluster_x <= S_work_x;
                        S_cluster_r <= S_work_r;
                        S_cluster_y0 <= S_work_y;
                        S_cluster_y1 <= S_work_y;
                        S_cluster_peak_x <= S_work_x;
                        S_cluster_peak_y <= S_work_y;
                        S_cluster_peak_r <= S_work_r;
                        S_cluster_lines <= 8'd1;
                    end
                    S_row_commit_pending <= 1'b0;
                end

                // The end-of-line pulse only selects and latches a candidate.
                // The two following blanking clocks do the arithmetic above.
                if(S_de_d) begin
                    if(S_row_primary_valid && S_selected_match) begin
                        S_row_commit_pending <= 1'b1;
                        S_work_match <= 1'b1;
                        S_work_x <= S_selected_x;
                        S_work_r <= S_selected_r;
                        S_work_y <= S_y;
                    end else if(S_row_primary_valid &&
                                ((S_cluster_lines == 0) ||
                                 (S_y > S_cluster_y1 + MAX_ROW_GAP))) begin
                        S_row_commit_pending <= 1'b1;
                        S_work_match <= 1'b0;
                        S_work_x <= S_row_primary_x;
                        S_work_r <= S_row_primary_r;
                        S_work_y <= S_y;
                    end else begin
                        S_row_commit_pending <= 1'b0;
                    end
                    S_y <= S_y + 12'd1;
                    S_x <= 12'd0;
                    S_row_primary_valid <= 1'b0;
                    S_row_secondary_valid <= 1'b0;
                end
            end
        end
    end
endmodule

// Streaming detector for a printed black/white concentric-ring pattern.
// It recognizes a symmetric D-L-D-L-D horizontal run signature on consecutive
// image rows. Every row is scanned completely and only its strongest candidate
// is clustered, avoiding the first-hit bias that loses small distant targets.
// The best vertical cluster is published only at frame boundary.
module bw_ring_detector_stream #(
    parameter [7:0]  DARK_THRESHOLD = 8'd96,
    parameter [11:0] MIN_RUN        = 12'd3,
    parameter [11:0] MAX_RUN        = 12'd180,
    parameter [7:0]  MIN_LINES      = 8'd12
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_vsync,
    input  wire        I_de,
    input  wire [23:0] I_rgb,
    output reg         O_valid,
    output reg  [11:0] O_center_x,
    output reg  [11:0] O_center_y,
    output reg  [11:0] O_radius
);
    wire [9:0] S_luma_sum = {2'b0,I_rgb[23:16]} +
                            {2'b0,I_rgb[15:8]} +
                            {2'b0,I_rgb[7:0]};
    wire [7:0] S_rg_diff = (I_rgb[23:16] > I_rgb[15:8]) ?
                           (I_rgb[23:16]-I_rgb[15:8]) : (I_rgb[15:8]-I_rgb[23:16]);
    wire [7:0] S_gb_diff = (I_rgb[15:8] > I_rgb[7:0]) ?
                           (I_rgb[15:8]-I_rgb[7:0]) : (I_rgb[7:0]-I_rgb[15:8]);
    wire [7:0] S_rb_diff = (I_rgb[23:16] > I_rgb[7:0]) ?
                           (I_rgb[23:16]-I_rgb[7:0]) : (I_rgb[7:0]-I_rgb[23:16]);
    wire       S_neutral = (S_rg_diff < 8'd40) && (S_gb_diff < 8'd40) &&
                           (S_rb_diff < 8'd40);
    wire       S_dark = (S_luma_sum < ({2'b0,DARK_THRESHOLD} * 3)) && S_neutral;

    reg        S_de_d, S_vsync_d;
    reg [11:0] S_x, S_y;
    reg        S_run_color;
    reg [11:0] S_run_len;
    reg        S_hist_color0, S_hist_color1, S_hist_color2;
    reg        S_hist_color3, S_hist_color4;
    reg [11:0] S_hist_len0, S_hist_len1, S_hist_len2;
    reg [11:0] S_hist_len3, S_hist_len4;
    reg        S_row_candidate_valid;
    reg [11:0] S_row_candidate_x, S_row_candidate_r;
    reg [11:0] S_row_candidate_center_w;

    reg [11:0] S_cluster_x, S_cluster_y0, S_cluster_y1, S_cluster_r;
    reg [7:0]  S_cluster_lines;
    reg [7:0]  S_best_lines;
    reg [11:0] S_best_x, S_best_y0, S_best_y1, S_best_r;

    wire [11:0] S_outer_black_diff = (S_hist_len0 > S_hist_len4) ?
                                     (S_hist_len0-S_hist_len4) :
                                     (S_hist_len4-S_hist_len0);
    wire [11:0] S_inner_white_diff = (S_hist_len1 > S_hist_len3) ?
                                     (S_hist_len1-S_hist_len3) :
                                     (S_hist_len3-S_hist_len1);
    // End-of-pattern is the first light pixel after the right dark band.
    // The middle of hist_len2 is therefore the symmetry centre.
    wire [11:0] S_candidate_x = S_x - S_run_len - S_hist_len0 -
                                S_hist_len1 - (S_hist_len2 >> 1);
    wire [11:0] S_candidate_r = S_hist_len0 + S_hist_len1 +
                                (S_hist_len2 >> 1);
    wire [11:0] S_x_diff = (S_row_candidate_x > S_cluster_x) ?
                           (S_row_candidate_x-S_cluster_x) :
                           (S_cluster_x-S_row_candidate_x);
    wire [11:0] S_r_diff = (S_row_candidate_r > S_cluster_r) ?
                           (S_row_candidate_r-S_cluster_r) :
                           (S_cluster_r-S_row_candidate_r);
    wire [11:0] S_cluster_next_r = (S_cluster_r + S_row_candidate_r) >> 1;
    wire [7:0]  S_cluster_next_lines = S_cluster_lines + 8'd1;
    wire [11:0] S_required_lines = (S_cluster_next_r >> 2);

    // Match D-L-D-L-D (surrounded by light runs). Small-target recovery comes
    // from evaluating the full row, while these proven width/symmetry limits
    // remain strict enough to reject text and indoor object edges.
    wire S_signature = !S_run_color &&
                       S_hist_color0 && !S_hist_color1 &&
                       S_hist_color2 && !S_hist_color3 &&
                       S_hist_color4 &&
                       (S_hist_len0 >= 12'd8) &&
                       (S_hist_len4 >= 12'd8) &&
                       (S_hist_len2 >= 12'd12) &&
                       (S_hist_len0 <= MAX_RUN) &&
                       (S_hist_len4 <= MAX_RUN) &&
                       (S_hist_len2 <= (MAX_RUN << 1)) &&
                       (S_hist_len1 >= 12'd2) &&
                       (S_hist_len3 >= 12'd2) &&
                       (S_hist_len1 <= 12'd32) &&
                       (S_hist_len3 <= 12'd32) &&
                       (S_hist_len0 >= (S_hist_len1 << 1)) &&
                       (S_hist_len4 >= (S_hist_len3 << 1)) &&
                       (S_hist_len2 >= (S_hist_len1 << 1)) &&
                       (S_hist_len2 >= (S_hist_len3 << 1)) &&
                       (S_outer_black_diff <= ((S_hist_len0 >> 1) + 12'd4)) &&
                       (S_inner_white_diff <= ((S_hist_len1 >> 1) + 12'd3));

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_de_d <= 1'b0; S_vsync_d <= 1'b0;
            S_x <= 12'd0; S_y <= 12'd0;
            S_run_color <= 1'b0; S_run_len <= 12'd0;
            S_hist_color0 <= 1'b0; S_hist_color1 <= 1'b0;
            S_hist_color2 <= 1'b0; S_hist_color3 <= 1'b0;
            S_hist_color4 <= 1'b0;
            S_hist_len0 <= 12'd0; S_hist_len1 <= 12'd0;
            S_hist_len2 <= 12'd0; S_hist_len3 <= 12'd0;
            S_hist_len4 <= 12'd0;
            S_row_candidate_valid <= 1'b0;
            S_row_candidate_x <= 12'd0;
            S_row_candidate_r <= 12'd0;
            S_row_candidate_center_w <= 12'd0;
            S_cluster_x <= 12'd0; S_cluster_y0 <= 12'd0;
            S_cluster_y1 <= 12'd0; S_cluster_r <= 12'd0;
            S_cluster_lines <= 8'd0; S_best_lines <= 8'd0;
            S_best_x <= 12'd0; S_best_y0 <= 12'd0;
            S_best_y1 <= 12'd0; S_best_r <= 12'd0;
            O_valid <= 1'b0; O_center_x <= 12'd0;
            O_center_y <= 12'd0; O_radius <= 12'd0;
        end else begin
            S_de_d <= I_de;
            S_vsync_d <= I_vsync;

            if(I_vsync && !S_vsync_d) begin
                O_valid <= (S_best_lines >= MIN_LINES) &&
                           (S_best_r >= 12'd8) &&
                           (S_best_lines >= (S_best_r >> 2)) &&
                           (((S_best_y0 + S_best_y1) >> 1) > S_best_r) &&
                           ((((S_best_y0 + S_best_y1) >> 1) + S_best_r) < 12'd720);
                O_center_x <= S_best_x;
                O_center_y <= (S_best_y0 + S_best_y1) >> 1;
                O_radius <= (S_best_r < 12'd8) ? 12'd8 : S_best_r;
                S_y <= 12'd0;
                S_cluster_lines <= 8'd0;
                S_best_lines <= 8'd0;
                S_row_candidate_valid <= 1'b0;
                S_row_candidate_center_w <= 12'd0;
            end else if(I_de && !S_de_d) begin
                S_x <= 12'd1;
                S_run_color <= S_dark;
                S_run_len <= 12'd1;
                S_hist_len0 <= 12'd0; S_hist_len1 <= 12'd0;
                S_hist_len2 <= 12'd0; S_hist_len3 <= 12'd0;
                S_hist_len4 <= 12'd0;
                S_hist_color0 <= 1'b0; S_hist_color1 <= 1'b0;
                S_hist_color2 <= 1'b0; S_hist_color3 <= 1'b0;
                S_hist_color4 <= 1'b0;
                S_row_candidate_valid <= 1'b0;
                S_row_candidate_center_w <= 12'd0;
            end else if(I_de) begin
                S_x <= S_x + 12'd1;
                if(S_dark == S_run_color) begin
                    if(S_run_len != 12'hfff) S_run_len <= S_run_len + 12'd1;
                end else begin
                    if(S_signature &&
                       (S_candidate_x > S_candidate_r) &&
                       (S_candidate_x + S_candidate_r < 12'd1280)) begin
                        if(!S_row_candidate_valid ||
                           (S_hist_len2 > S_row_candidate_center_w) ||
                           ((S_hist_len2 == S_row_candidate_center_w) &&
                            (S_candidate_r > S_row_candidate_r))) begin
                            S_row_candidate_valid <= 1'b1;
                            S_row_candidate_x <= S_candidate_x;
                            S_row_candidate_r <= S_candidate_r;
                            S_row_candidate_center_w <= S_hist_len2;
                        end
                    end
                    S_hist_color4 <= S_hist_color3;
                    S_hist_color3 <= S_hist_color2;
                    S_hist_color2 <= S_hist_color1;
                    S_hist_color1 <= S_hist_color0;
                    S_hist_color0 <= S_run_color;
                    S_hist_len4 <= S_hist_len3;
                    S_hist_len3 <= S_hist_len2;
                    S_hist_len2 <= S_hist_len1;
                    S_hist_len1 <= S_hist_len0;
                    S_hist_len0 <= S_run_len;
                    S_run_color <= S_dark;
                    S_run_len <= 12'd1;
                end
            end else if(!I_de && S_de_d) begin
                if(S_row_candidate_valid) begin
                    if((S_cluster_lines != 0) &&
                       (S_y <= S_cluster_y1 + 12'd3) &&
                       (S_x_diff <= (S_cluster_r >> 2) + 12'd6) &&
                       (S_r_diff <= (S_cluster_r >> 2) + 12'd5)) begin
                        S_cluster_x <= (S_cluster_x + S_row_candidate_x) >> 1;
                        S_cluster_r <= S_cluster_next_r;
                        S_cluster_y1 <= S_y;
                        if(S_cluster_lines != 8'hff)
                            S_cluster_lines <= S_cluster_next_lines;
                        // Store only clusters that already satisfy their
                        // scale-dependent support. Rank support before radius.
                        if((S_cluster_next_lines >= MIN_LINES) &&
                           ({4'd0,S_cluster_next_lines} >= S_required_lines) &&
                           ((S_cluster_next_lines > S_best_lines) ||
                            ((S_cluster_next_lines == S_best_lines) &&
                             (S_cluster_next_r > S_best_r)))) begin
                            S_best_lines <= S_cluster_next_lines;
                            S_best_x <= (S_cluster_x + S_row_candidate_x) >> 1;
                            S_best_r <= S_cluster_next_r;
                            S_best_y0 <= S_cluster_y0;
                            S_best_y1 <= S_y;
                        end
                    end else begin
                        S_cluster_x <= S_row_candidate_x;
                        S_cluster_r <= S_row_candidate_r;
                        S_cluster_y0 <= S_y;
                        S_cluster_y1 <= S_y;
                        S_cluster_lines <= 8'd1;
                    end
                end
                S_y <= S_y + 12'd1;
                S_x <= 12'd0;
                S_row_candidate_valid <= 1'b0;
                S_row_candidate_center_w <= 12'd0;
            end
        end
    end
endmodule


