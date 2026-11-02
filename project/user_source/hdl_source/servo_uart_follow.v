`default_nettype none
// Pan: retain the user's controller. Tilt: filtered PD velocity + Q8 trajectory.
// Tilt captures and holds near centre; real target displacement releases it.
// Tilt: 100Hz far / 200Hz fine waypoints; retain120Hz integration.
// Near SCREEN centre: one-unit packets, proportional fractional accumulation,
// reduced velocity cap; no raised far speed, no start-up kick.
// TILT_PACKET_STEP remains a pulse-width budget per20ms reference interval.
// Guarded revision: NO edge overspeed or enlarged tilt packet step.
// Edge catch only shortens acceleration within the original safe rate cap.
module servo_uart_follow #(
 parameter integer CLK_HZ=75000000,BAUD=115200,CONTROL_HZ=120,
 parameter integer VISUAL_SAMPLE_DELAY=64,
 parameter [11:0] FRAME_CENTER_X=640,FRAME_CENTER_Y=360,
 parameter [11:0] PWM_CENTER=1500,PWM_MIN=1000,PWM_MAX=2000,
 // The servo is not required to hit one exact pixel.  Once the target enters
 // this screen-centre tolerance region, the centre-lock logic holds the last
 // transmitted endpoint.  Wider START values provide release hysteresis.
 // Horizontal tracking requests no motion at |error| <= 14 pixels.
 // The separate centre-lock state and its release hysteresis remain active.
 parameter [11:0] PAN_STOP=14,PAN_START=14,TILT_STOP=14,TILT_START=18,
 parameter [11:0] MAX_TRACK_STEP=8,TILT_MAX_STEP=4,
 parameter [11:0] PAN_REVERSE_ERROR=18,
 parameter [1:0] PAN_REVERSE_FRAMES=2'd3,
 // Pan centre hold is deliberately separate from the ordinary 8/12-pixel
 // tracking hysteresis.  Enter/cross starts a short observation hold; only a
 // persistent displacement releases it.  This prevents delayed vision from
 // commanding the opposite stroke while the previous UART move is settling.
 parameter PAN_CENTER_LOCK_ENABLE=1'b1,
 parameter integer PAN_LOCK_ENTER=16,PAN_LOCK_CROSS=24,PAN_LOCK_EXIT=28,
 parameter integer PAN_LOCK_DISPLACEMENT=12,PAN_LOCK_STABLE_FRAMES=3,
 parameter integer PAN_LOCK_RELEASE_SAMPLES=3,PAN_LOCK_JITTER=4,
 parameter integer TILT_STEP_SHIFT=1,TILT_MOVE_MS=10,
 parameter integer TILT_PACKET_STEP=4,
 parameter [15:0] PAN_ACCEL_Q4=16'd4,
 parameter integer TILT_KP_Q8=8,TILT_KD_Q8=32,
 parameter integer TILT_ACCEL_Q8=32,TILT_DECEL_Q8=64,
 parameter integer TILT_REVERSE_PIXELS=48,TILT_REVERSE_SAMPLES=2,
 parameter TILT_CENTER_LOCK_ENABLE=1'b1,
 parameter integer TILT_LOCK_CROSS=24,TILT_LOCK_EXIT=28,TILT_LOCK_DISPLACEMENT=12,
 parameter integer TILT_LOCK_SETTLE_FRAMES=4,TILT_LOCK_RELEASE_SAMPLES=3,
 parameter TILT_CONTROL_ENABLE=1'b1,
 parameter [11:0] TILT_PWM_CENTER=1500,TILT_PWM_MIN=1000,TILT_PWM_MAX=2000,
 parameter PAN_REVERSE=1'b1,TILT_REVERSE=1'b1,
 parameter integer TILT_PACKET_REFERENCE_MS=20,
 parameter integer SOFT_LIMIT_MARGIN=100,LIMIT_BRAKE_WINDOW=80,
 parameter EDGE_CATCH_ENABLE=1'b1,
 parameter integer FRAME_WIDTH=1280,FRAME_HEIGHT=720,
 parameter integer PAN_EDGE_MARGIN=160,TILT_EDGE_MARGIN=96,
 parameter integer EDGE_WARMUP_SAMPLES=8,EDGE_CONFIRM_SAMPLES=3,
 parameter integer EDGE_MIN_OUTWARD_PIXELS=2,EDGE_MAX_OUTWARD_PIXELS=48,
 parameter integer EDGE_BURST_MS=100,EDGE_COOLDOWN_MS=500,
 parameter [15:0] PAN_EDGE_ACCEL_Q4=PAN_ACCEL_Q4+(PAN_ACCEL_Q4>>2),
 parameter integer TILT_EDGE_ACCEL_Q8=TILT_ACCEL_Q8+(TILT_ACCEL_Q8>>2),
 // Fine scheduling changes packet spacing, not the original far speed cap.
 parameter integer TILT_FINE_PIXELS=48,TILT_FINE_MOVE_MS=5,
 parameter integer TILT_FINE_RATE_Q8=128,
 parameter integer CENTER30_TILT_RATE_Q8=64,
 // Only a large vertical error may use the faster tilt profile.  Medium,
 // fine and centre-region behaviour remains unchanged.
 parameter integer TILT_FAST_PIXELS=96,TILT_FAST_RATE_Q8=320,
 parameter integer TILT_FAST_ACCEL_Q8=40,
 // Progressive vertical-edge catch.  Zone 1 is 151..200 pixels from the
 // nearest image edge; zone 4 is the nearest 0..50 pixels.
 parameter integer TILT_EDGE200_RATE_Q8=352,TILT_EDGE150_RATE_Q8=384,
 parameter integer TILT_EDGE100_RATE_Q8=416,TILT_EDGE50_RATE_Q8=448,
 parameter integer TILT_EDGE200_ACCEL_Q8=48,TILT_EDGE150_ACCEL_Q8=56,
 parameter integer TILT_EDGE100_ACCEL_Q8=64,TILT_EDGE50_ACCEL_Q8=72,
 parameter TILT_MOTION_FOLLOW_ENABLE=1'b1,
 // Lost-target recovery is deliberately independent of the normal tracking
 // gains.  It is armed only by a target moving outward inside an edge band.
 // Search and return packets remain small and are hard-clamped to 1100..1900.
 parameter TARGET_RECOVERY_ENABLE=1'b1,
 parameter integer RECOVERY_LOST_FRAMES=3,RECOVERY_REACQUIRE_FRAMES=2,
 parameter integer RECOVERY_PAN_EDGE_MARGIN=200,RECOVERY_TILT_EDGE_MARGIN=200,
 parameter [11:0] RECOVERY_PWM_MIN=12'd1100,RECOVERY_PWM_MAX=12'd1900,
 parameter [11:0] RECOVERY_PAN_STEP=12'd2,RECOVERY_TILT_STEP=12'd2,
 parameter [11:0] RECENTER_PAN_STEP=12'd2,RECENTER_TILT_STEP=12'd2
)(input wire I_clk,I_rst_n,I_enable,I_vsync,I_target_valid,
 input wire [11:0] I_target_cx,I_target_cy,
 input wire [7:0] I_target_confidence,output reg O_uart_tx);
 localparam integer TICK_DIV=CLK_HZ/CONTROL_HZ,BAUD_DIV=CLK_HZ/BAUD,GAP_DIV=CLK_HZ/200;
 localparam [11:0] SAFE_PAN_MIN=PWM_MIN+SOFT_LIMIT_MARGIN,SAFE_PAN_MAX=PWM_MAX-SOFT_LIMIT_MARGIN;
 localparam [11:0] SAFE_TILT_MIN=TILT_PWM_MIN+SOFT_LIMIT_MARGIN,SAFE_TILT_MAX=TILT_PWM_MAX-SOFT_LIMIT_MARGIN;
 localparam LIMITS_OK=SOFT_LIMIT_MARGIN>=0 &&
   PWM_MIN+SOFT_LIMIT_MARGIN<PWM_MAX-SOFT_LIMIT_MARGIN &&
   TILT_PWM_MIN+SOFT_LIMIT_MARGIN<TILT_PWM_MAX-SOFT_LIMIT_MARGIN &&
   PWM_CENTER>=SAFE_PAN_MIN && PWM_CENTER<=SAFE_PAN_MAX &&
   TILT_PWM_CENTER>=SAFE_TILT_MIN && TILT_PWM_CENTER<=SAFE_TILT_MAX;
 localparam [2:0] IDLE=0,LOAD=1,SEND=2,GAP=3;
 localparam [1:0] REC_TRACK=2'd0,REC_SEARCH=2'd1,REC_CENTER=2'd2;
 reg [31:0] tick_count,age,gap_count,baud_count;
 reg vd,measured,history,pan_tracking,center30_sampled;
 reg [15:0] sample_wait;
 reg [11:0] sx,sy,prev_x,prev_ex;
 reg signed [12:0] vx;
 reg pan_last_add,pan_has_direction;
 reg [1:0] pan_reverse_count;
 reg [15:0] pan_q,pan_rate_q4;
 reg [11:0] sent_pan,sent_tilt,cmd_pan,cmd_tilt;
 reg [2:0] state;
 reg axis,pending_pan,cmd_tilt_fine;
 reg [3:0] cmd_index,bit_index;
 reg [8:0] shift;
 reg [1:0] recovery_state;
 reg [7:0] lost_frames,reacquire_frames;
 reg pan_edge_armed,tilt_edge_armed;
 reg search_pan_active,search_tilt_active;
 reg recovery_pan_add,recovery_tilt_add;
 reg last_pan_cmd_add,last_tilt_cmd_add;
 reg last_pan_cmd_valid,last_tilt_cmd_valid;
 reg trajectory_sync;
 reg [11:0] trajectory_sync_pwm;
 wire fresh=I_target_valid && I_target_confidence>=160;
 wire usable=I_enable && LIMITS_OK && recovery_state==REC_TRACK &&
             measured && fresh && age<CLK_HZ/30;
 wire control_tick=tick_count==TICK_DIV-1;
 wire signed [12:0] pan_measure_error=$signed({1'b0,I_target_cx})-
                                      $signed({1'b0,FRAME_CENTER_X});
 // Exact 40-pixel circular fine-control region without multipliers. For a
 // given |dx| the lookup returns floor(sqrt(40^2-dx^2)); this avoids adding
 // two squarers/DSPs to an already resource-constrained design.
 function center30_test;
 input [11:0] cx,cy;
 reg [11:0] dx,dy;
 reg [5:0] y_limit;
 begin
  dx=cx>=FRAME_CENTER_X ? cx-FRAME_CENTER_X : FRAME_CENTER_X-cx;
  dy=cy>=FRAME_CENTER_Y ? cy-FRAME_CENTER_Y : FRAME_CENTER_Y-cy;
  case(dx)
   12'd0:y_limit=6'd40;
   12'd1,12'd2,12'd3,12'd4,12'd5,12'd6,12'd7,12'd8:y_limit=6'd39;
   12'd9,12'd10,12'd11,12'd12:y_limit=6'd38;
   12'd13,12'd14,12'd15:y_limit=6'd37;
   12'd16,12'd17:y_limit=6'd36;
   12'd18,12'd19:y_limit=6'd35;
   12'd20,12'd21:y_limit=6'd34;
   12'd22:y_limit=6'd33;
   12'd23,12'd24:y_limit=6'd32;
   12'd25:y_limit=6'd31;
   12'd26:y_limit=6'd30;
   12'd27:y_limit=6'd29;
   12'd28:y_limit=6'd28;
   12'd29:y_limit=6'd27;
   12'd30:y_limit=6'd26;
   12'd31:y_limit=6'd25;
   12'd32:y_limit=6'd24;
   12'd33:y_limit=6'd22;
   12'd34:y_limit=6'd21;
   12'd35:y_limit=6'd19;
   12'd36:y_limit=6'd17;
   12'd37:y_limit=6'd15;
   12'd38:y_limit=6'd12;
   12'd39:y_limit=6'd8;
   12'd40:y_limit=6'd0;
   default:y_limit=6'd0;
  endcase
  center30_test=(dx<=12'd40) && (dy<={6'd0,y_limit});
 end
 endfunction
 wire center30_live=center30_test(I_target_cx,I_target_cy);
 wire pan_at_edge=I_target_cx<=RECOVERY_PAN_EDGE_MARGIN ||
                  I_target_cx>=FRAME_WIDTH-1-RECOVERY_PAN_EDGE_MARGIN;
 wire tilt_at_edge=I_target_cy<=RECOVERY_TILT_EDGE_MARGIN ||
                   I_target_cy>=FRAME_HEIGHT-1-RECOVERY_TILT_EDGE_MARGIN;
 wire pan_outward_sample=history &&
      ((I_target_cx<=RECOVERY_PAN_EDGE_MARGIN && I_target_cx<sx) ||
       (I_target_cx>=FRAME_WIDTH-1-RECOVERY_PAN_EDGE_MARGIN && I_target_cx>sx));
 wire tilt_outward_sample=history &&
      ((I_target_cy<=RECOVERY_TILT_EDGE_MARGIN && I_target_cy<sy) ||
       (I_target_cy>=FRAME_HEIGHT-1-RECOVERY_TILT_EDGE_MARGIN && I_target_cy>sy));
 wire pan_center_locked,pan_center_hold;
 axis_center_stability #(
  .ENTER_PIXELS(PAN_LOCK_ENTER),.CROSS_PIXELS(PAN_LOCK_CROSS),
  .EXIT_PIXELS(PAN_LOCK_EXIT),.DISPLACEMENT_PIXELS(PAN_LOCK_DISPLACEMENT),
  .STABLE_FRAMES(PAN_LOCK_STABLE_FRAMES),.RELEASE_SAMPLES(PAN_LOCK_RELEASE_SAMPLES),
  .JITTER_PIXELS(PAN_LOCK_JITTER)
 ) u_pan_center_stability (
  .clk(I_clk),.rst_n(I_rst_n),
  .enable(I_enable && recovery_state==REC_TRACK && PAN_CENTER_LOCK_ENABLE &&
          (age<CLK_HZ/30 || sample_wait==1)),
  .measure_strobe(sample_wait==1),.measure_valid(fresh),.error_in(pan_measure_error),
  .locked(pan_center_locked),.hold_now(pan_center_hold)
 );
 wire pan_edge_boost;
 wire pan_inside_braking_zone=pan_q[15:4]<=SAFE_PAN_MIN+LIMIT_BRAKE_WINDOW ||
                              pan_q[15:4]>=SAFE_PAN_MAX-LIMIT_BRAKE_WINDOW;
 servo_guarded_edge #(.FRAME_PIXELS(FRAME_WIDTH),.ENTER_MARGIN(PAN_EDGE_MARGIN),
  .CONTROL_HZ(CONTROL_HZ),.WARMUP_SAMPLES(EDGE_WARMUP_SAMPLES),.CONFIRM_SAMPLES(EDGE_CONFIRM_SAMPLES),
  .MIN_OUTWARD_PIXELS(EDGE_MIN_OUTWARD_PIXELS),.MAX_OUTWARD_PIXELS(EDGE_MAX_OUTWARD_PIXELS),
  .BURST_MS(EDGE_BURST_MS),.COOLDOWN_MS(EDGE_COOLDOWN_MS)) u_pan_guarded_edge (
  .clk(I_clk),.rst_n(I_rst_n),.control_tick(control_tick),
  .enable(I_enable && recovery_state==REC_TRACK && EDGE_CATCH_ENABLE &&
          (age<CLK_HZ/30 || sample_wait==1)),
  .allow_boost(!pan_inside_braking_zone),.measure_strobe(sample_wait==1),
  .measure_valid(fresh),.position(I_target_cx),.boost(pan_edge_boost));
 wire tilt_pending,tilt_ack,tilt_inhibit,tilt_packet_fine;
 wire [11:0] next_tilt;
 assign tilt_ack=usable && state==IDLE && tilt_pending && !tilt_inhibit;
 tilt_pd_trajectory #(
  .CLK_HZ(CLK_HZ),.CONTROL_HZ(CONTROL_HZ),.CENTER_Y(FRAME_CENTER_Y),
  .PWM_CENTER(TILT_PWM_CENTER),.PWM_MIN(SAFE_TILT_MIN),.PWM_MAX(SAFE_TILT_MAX),
  .STOP_PIXELS(TILT_STOP),.START_PIXELS(TILT_START),.REVERSE(TILT_REVERSE),
  .KP_Q8(TILT_KP_Q8),.KD_Q8(TILT_KD_Q8),.ACCEL_Q8(TILT_ACCEL_Q8),.DECEL_Q8(TILT_DECEL_Q8),
  .MAX_NOMINAL_STEP(TILT_MAX_STEP),.STEP_SHIFT(TILT_STEP_SHIFT),.PACKET_MS(TILT_MOVE_MS),
  .MAX_PACKET_STEP(TILT_PACKET_STEP),
  .PACKET_REFERENCE_MS(TILT_PACKET_REFERENCE_MS),
  .REVERSE_PIXELS(TILT_REVERSE_PIXELS),.REVERSE_SAMPLES(TILT_REVERSE_SAMPLES),
  .CENTER_LOCK_ENABLE(TILT_CENTER_LOCK_ENABLE),.LOCK_CROSS(TILT_LOCK_CROSS),.LOCK_EXIT(TILT_LOCK_EXIT),
  .LOCK_DISPLACEMENT(TILT_LOCK_DISPLACEMENT),.LOCK_SETTLE_FRAMES(TILT_LOCK_SETTLE_FRAMES),
  .LOCK_RELEASE_SAMPLES(TILT_LOCK_RELEASE_SAMPLES),
  .LIMIT_BRAKE_WINDOW(LIMIT_BRAKE_WINDOW),.EDGE_CATCH_ENABLE(EDGE_CATCH_ENABLE),
  .FRAME_PIXELS(FRAME_HEIGHT),.EDGE_MARGIN(TILT_EDGE_MARGIN),
  .EDGE_WARMUP_SAMPLES(EDGE_WARMUP_SAMPLES),.EDGE_CONFIRM_SAMPLES(EDGE_CONFIRM_SAMPLES),
  .EDGE_MIN_OUTWARD_PIXELS(EDGE_MIN_OUTWARD_PIXELS),.EDGE_MAX_OUTWARD_PIXELS(EDGE_MAX_OUTWARD_PIXELS),
  .EDGE_BURST_MS(EDGE_BURST_MS),.EDGE_COOLDOWN_MS(EDGE_COOLDOWN_MS),.EDGE_ACCEL_Q8(TILT_EDGE_ACCEL_Q8),
  .FINE_PIXELS(TILT_FINE_PIXELS),.FINE_PACKET_MS(TILT_FINE_MOVE_MS),.FINE_RATE_Q8(TILT_FINE_RATE_Q8),
  .CENTER_FINE_RATE_Q8(CENTER30_TILT_RATE_Q8),
  .FAST_PIXELS(TILT_FAST_PIXELS),.FAST_RATE_Q8(TILT_FAST_RATE_Q8),
  .FAST_ACCEL_Q8(TILT_FAST_ACCEL_Q8),
  .EDGE200_RATE_Q8(TILT_EDGE200_RATE_Q8),.EDGE150_RATE_Q8(TILT_EDGE150_RATE_Q8),
  .EDGE100_RATE_Q8(TILT_EDGE100_RATE_Q8),.EDGE50_RATE_Q8(TILT_EDGE50_RATE_Q8),
  .EDGE200_ACCEL_Q8(TILT_EDGE200_ACCEL_Q8),.EDGE150_ACCEL_Q8(TILT_EDGE150_ACCEL_Q8),
  .EDGE100_ACCEL_Q8(TILT_EDGE100_ACCEL_Q8),.EDGE50_ACCEL_Q8(TILT_EDGE50_ACCEL_Q8),
  .MOTION_FOLLOW_ENABLE(TILT_MOTION_FOLLOW_ENABLE)
 ) u_tilt_trajectory (
  .clk(I_clk),.rst_n(I_rst_n),
  .enable(I_enable && recovery_state==REC_TRACK && TILT_CONTROL_ENABLE && LIMITS_OK),
  .position_sync(trajectory_sync),.sync_pwm(trajectory_sync_pwm),
  .measure_strobe(sample_wait==1),.measure_valid(fresh),.fine_region(center30_live),.cy(I_target_cy),
  .control_tick(control_tick),.packet_ack(tilt_ack),.packet_pending(tilt_pending),.packet_pwm(next_tilt),
  .inhibit_packet(tilt_inhibit),.packet_fine(tilt_packet_fine)
 );
 function [11:0] absdiff;
 input [11:0] a,b;
 begin absdiff=a>=b ? a-b : b-a;end
 endfunction
 function signed [12:0] velocity;
 input [11:0] a,b;
 reg signed [12:0] delta;
 begin delta=$signed({1'b0,a})-$signed({1'b0,b});
 velocity=delta>16 ? 13'sd16 : (delta< -16 ? -13'sd16 : delta);end
 endfunction
 function [11:0] predict;
 input [11:0] pos,limit,lead_limit;
 input signed [12:0] vel;
 input enable;
 reg signed [15:0] lead,total;
 begin lead=enable ? (vel >>> 1) : 0;
 if(lead>$signed({1'b0,lead_limit})) lead=$signed({1'b0,lead_limit});
 if(lead< -$signed({1'b0,lead_limit})) lead=-$signed({1'b0,lead_limit});
 total=$signed({1'b0,pos})+lead;
 predict=total<0 ? 0 : (total>$signed({1'b0,limit}) ? limit : total[11:0]);end
 endfunction
 reg [3:0] pan_calc;
 reg [11:0] px,ex,raw_pan,pan_step;
 reg pan_add,pan_brake,pan_move,pan_direction_ok;
 reg pan_edge_calc;
 reg [15:0] desired_pan_q4,ramp_pan_q4,next_pan_q;
 reg [16:0] rounded_pan;
 function [11:0] gain;
 input [11:0] err,stop,maximum,near_maximum;
 reg [11:0] step;
 begin if(err<=stop) gain=0;
 else begin step=err<=128 ? 1+((err-stop)>>5) : 3+((err-128)>>5);
 if(err<=128 && step>near_maximum) step=near_maximum;
 gain=step>maximum ? maximum : step;end end
 endfunction
 // Preserve the original speed-band boundaries while raising only the stop
 // threshold: 15..39 => 1, 40..71 => 2, 72..159 => 3, etc.
 function [11:0] pan_band_gain;
 input [11:0] err,stop,maximum;
 begin
  pan_band_gain=err<=stop ? 12'd0 : gain(err,12'd8,maximum,12'd3);
 end
 endfunction
 function [15:0] ramp_up;
 input [15:0] current,target,maximum_increase;
 reg [16:0] increased;
 begin increased={1'b0,current}+{1'b0,maximum_increase};
 ramp_up=target<=current ? target : ({1'b0,target}>increased ? increased[15:0] : target);end
 endfunction
 wire [15:0] nominal_pan_q4=(CONTROL_HZ==120) ? {1'b0,pan_step,3'b0} :
                                                 (pan_step*32'd960)/CONTROL_HZ;
 wire [11:0] pan_position=pan_q[15:4];
 wire [11:0] pan_remaining=pan_add ?
     (pan_position>=SAFE_PAN_MAX ? 12'd0 : SAFE_PAN_MAX-pan_position) :
     (pan_position<=SAFE_PAN_MIN ? 12'd0 : pan_position-SAFE_PAN_MIN);
 function [15:0] brake_pan_rate;
 input [15:0] rate;
 input [11:0] remaining;
 begin
  if(LIMIT_BRAKE_WINDOW<=0) brake_pan_rate=rate;
  else if(remaining<=LIMIT_BRAKE_WINDOW/10) brake_pan_rate=rate>>3;
  else if(remaining<=LIMIT_BRAKE_WINDOW/2) brake_pan_rate=rate>>2;
  else if(remaining<=LIMIT_BRAKE_WINDOW) brake_pan_rate=rate>>1;
  else brake_pan_rate=rate;
 end
 endfunction
 // Remove the one-step speed floor ONLY in the8px band above the stop
 // threshold. Far-target speed, acceleration and reverse gate are unchanged.
 wire [15:0] landing_pan_q4=(CONTROL_HZ==120) ? (ex>PAN_STOP ? ex-PAN_STOP : 0) :
                                  ((ex>PAN_STOP ? ex-PAN_STOP : 0)*32'd120)/CONTROL_HZ;
 function [15:0] apply_q;
 input [15:0] q,increment;
 input add;
 input [11:0] minimum,maximum;
 reg [32:0] candidate;
 begin if(add) begin candidate={17'd0,q}+{1'b0,increment};
 apply_q=candidate>({21'd0,maximum}<<4) ? (maximum<<4) : candidate[15:0];end
 else apply_q={17'd0,q}<=({21'd0,minimum}<<4)+increment ? (minimum<<4) : q-increment;
 end endfunction
 function [11:0] bounded_move;
 input [11:0] current,step,minimum,maximum;
 input add;
 reg [12:0] candidate;
 begin
  if(add && current>=maximum) bounded_move=current;
  else if(!add && current<=minimum) bounded_move=current;
  else if(add) begin
   candidate={1'b0,current}+{1'b0,step};
   bounded_move=candidate>{1'b0,maximum} ? maximum : candidate[11:0];
  end else bounded_move=current<=minimum+step ? minimum : current-step;
 end
 endfunction
 function [11:0] move_to_center;
 input [11:0] current,center,step;
 begin
  if(current<center) move_to_center=current+step>=center ? center : current+step;
  else if(current>center) move_to_center=current<=center+step ? center : current-step;
  else move_to_center=current;
 end
 endfunction
 wire [11:0] next_pan=rounded_pan[15:4];
 wire pan_changed=next_pan!=sent_pan && !pan_center_hold;
 wire tilt_changed=TILT_CONTROL_ENABLE && tilt_pending && next_tilt!=sent_tilt && !tilt_inhibit;
 wire [11:0] recovery_pan_next=bounded_move(sent_pan,RECOVERY_PAN_STEP,
                                            RECOVERY_PWM_MIN,RECOVERY_PWM_MAX,recovery_pan_add);
 wire [11:0] recovery_tilt_next=bounded_move(sent_tilt,RECOVERY_TILT_STEP,
                                             RECOVERY_PWM_MIN,RECOVERY_PWM_MAX,recovery_tilt_add);
 wire recovery_pan_can_move=search_pan_active && sent_pan>=RECOVERY_PWM_MIN &&
                             sent_pan<=RECOVERY_PWM_MAX && recovery_pan_next!=sent_pan;
 wire recovery_tilt_can_move=search_tilt_active && sent_tilt>=RECOVERY_PWM_MIN &&
                              sent_tilt<=RECOVERY_PWM_MAX && recovery_tilt_next!=sent_tilt;
 wire [11:0] center_pan_next=move_to_center(sent_pan,PWM_CENTER,RECENTER_PAN_STEP);
 wire [11:0] center_tilt_next=move_to_center(sent_tilt,TILT_PWM_CENTER,RECENTER_TILT_STEP);
 wire center_pan_can_move=center_pan_next!=sent_pan;
 wire center_tilt_can_move=center_tilt_next!=sent_tilt;
 function [7:0] byte_at;
 input axis_in;
 input [3:0] idx;
 input [11:0] pwm;
 input fine;
 begin case(idx)
 0:byte_at=8'h23;1,2:byte_at=8'h30;3:byte_at=axis_in ? 8'h35 : 8'h30;
 4:byte_at=8'h50;5:byte_at=8'h30+pwm/1000;6:byte_at=8'h30+(pwm/100)%10;
 7:byte_at=8'h30+(pwm/10)%10;8:byte_at=8'h30+pwm%10;9:byte_at=8'h54;
 10:byte_at=axis_in ? 8'h30 : 8'h30+((fine ? TILT_FINE_MOVE_MS : TILT_MOVE_MS)/1000)%10;
 11:byte_at=axis_in ? 8'h30 : 8'h30+((fine ? TILT_FINE_MOVE_MS : TILT_MOVE_MS)/100)%10;
 12:byte_at=axis_in ? 8'h31 : 8'h30+((fine ? TILT_FINE_MOVE_MS : TILT_MOVE_MS)/10)%10;
 13:byte_at=axis_in ? 8'h30 : 8'h30+(fine ? TILT_FINE_MOVE_MS : TILT_MOVE_MS)%10;
 default:byte_at=8'h21;endcase end
 endfunction
 always @(posedge I_clk or negedge I_rst_n) begin
 if(!I_rst_n) begin
  tick_count<=0;age<=CLK_HZ;vd<=0;measured<=0;history<=0;sample_wait<=0;center30_sampled<=0;
  sx<=FRAME_CENTER_X;sy<=FRAME_CENTER_Y;prev_x<=0;prev_ex<=0;vx<=0;
  pan_tracking<=0;pan_last_add<=0;pan_reverse_count<=0;pan_has_direction<=0;
  pan_q<=PWM_CENTER<<4;pan_rate_q4<=0;
  pan_calc<=0;px<=FRAME_CENTER_X;ex<=0;raw_pan<=0;pan_step<=0;
  pan_add<=0;pan_brake<=0;pan_move<=0;pan_direction_ok<=0;
  pan_edge_calc<=0;
  desired_pan_q4<=0;ramp_pan_q4<=0;next_pan_q<=PWM_CENTER<<4;rounded_pan<=PWM_CENTER<<4;
  sent_pan<=PWM_CENTER;sent_tilt<=TILT_PWM_CENTER;cmd_pan<=PWM_CENTER;cmd_tilt<=TILT_PWM_CENTER;
  state<=IDLE;axis<=0;pending_pan<=0;cmd_tilt_fine<=0;cmd_index<=0;bit_index<=0;shift<=9'h1ff;baud_count<=0;gap_count<=0;O_uart_tx<=1;
  recovery_state<=REC_TRACK;lost_frames<=0;reacquire_frames<=0;
  pan_edge_armed<=0;tilt_edge_armed<=0;search_pan_active<=0;search_tilt_active<=0;
  recovery_pan_add<=0;recovery_tilt_add<=0;
  last_pan_cmd_add<=0;last_tilt_cmd_add<=0;last_pan_cmd_valid<=0;last_tilt_cmd_valid<=0;
  trajectory_sync<=0;trajectory_sync_pwm<=TILT_PWM_CENTER;
 end else begin
  trajectory_sync<=0;
  vd<=I_vsync;if(age<CLK_HZ) age<=age+1'b1;
  if(!I_enable || !measured || age>=CLK_HZ/30) pan_rate_q4<=0;
  if(!I_enable) sample_wait<=0;
  else if(I_vsync && !vd) sample_wait<=VISUAL_SAMPLE_DELAY;
  else if(sample_wait!=0) sample_wait<=sample_wait-1'b1;
  if(!I_enable) begin
   measured<=0;history<=0;vx<=0;pan_tracking<=0;pan_reverse_count<=0;pan_has_direction<=0;pending_pan<=0;center30_sampled<=0;
   recovery_state<=REC_TRACK;lost_frames<=0;reacquire_frames<=0;
   pan_edge_armed<=0;tilt_edge_armed<=0;search_pan_active<=0;search_tilt_active<=0;
   last_pan_cmd_valid<=0;last_tilt_cmd_valid<=0;
  end
  else if(sample_wait==1) begin
   measured<=fresh;
   if(fresh) begin
    age<=0;sx<=I_target_cx;sy<=I_target_cy;center30_sampled<=center30_live;
    if(history) vx<=velocity(I_target_cx,prev_x);else vx<=0;
    prev_x<=I_target_cx;prev_ex<=absdiff(sx,FRAME_CENTER_X);history<=1;
    if(absdiff(I_target_cx,FRAME_CENTER_X)<=PAN_REVERSE_ERROR ||
       (PAN_REVERSE ? !(I_target_cx>=FRAME_CENTER_X) : I_target_cx>=FRAME_CENTER_X)==pan_last_add) pan_reverse_count<=0;
    else if(pan_reverse_count<PAN_REVERSE_FRAMES) pan_reverse_count<=pan_reverse_count+1'b1;
    lost_frames<=0;
    if(recovery_state==REC_TRACK) begin
     reacquire_frames<=0;
     pan_edge_armed<=TARGET_RECOVERY_ENABLE && pan_at_edge && last_pan_cmd_valid &&
                     (pan_outward_sample || pan_edge_armed);
     tilt_edge_armed<=TARGET_RECOVERY_ENABLE && tilt_at_edge && last_tilt_cmd_valid &&
                      (tilt_outward_sample || tilt_edge_armed);
     if(TARGET_RECOVERY_ENABLE && pan_at_edge && last_pan_cmd_valid &&
        (pan_outward_sample || pan_edge_armed)) recovery_pan_add<=last_pan_cmd_add;
     if(TARGET_RECOVERY_ENABLE && tilt_at_edge && last_tilt_cmd_valid &&
        (tilt_outward_sample || tilt_edge_armed)) recovery_tilt_add<=last_tilt_cmd_add;
    end else begin
     // A single noisy detection pauses search; two consecutive valid video
     // measurements are required before normal tracking is re-enabled.
     if(reacquire_frames<RECOVERY_REACQUIRE_FRAMES) reacquire_frames<=reacquire_frames+1'b1;
     if(RECOVERY_REACQUIRE_FRAMES<=1 || reacquire_frames>=RECOVERY_REACQUIRE_FRAMES-1) begin
      recovery_state<=REC_TRACK;reacquire_frames<=0;
      search_pan_active<=0;search_tilt_active<=0;pan_edge_armed<=0;tilt_edge_armed<=0;
      // Require the next video measurement to start the ordinary controller;
      // do not dispatch a half-finished pre-recovery pan calculation.
      measured<=0;
      pan_q<=sent_pan<<4;pan_rate_q4<=0;pan_tracking<=0;pan_has_direction<=0;
      trajectory_sync<=1;trajectory_sync_pwm<=sent_tilt;
      last_pan_cmd_valid<=0;last_tilt_cmd_valid<=0;
     end
    end
   end else begin history<=0;vx<=0;pan_tracking<=0;pan_reverse_count<=0;pan_has_direction<=0;center30_sampled<=0;end
   if(!fresh) begin
    reacquire_frames<=0;
    if(recovery_state==REC_TRACK && TARGET_RECOVERY_ENABLE &&
       (pan_edge_armed || tilt_edge_armed)) begin
     if(lost_frames<RECOVERY_LOST_FRAMES) lost_frames<=lost_frames+1'b1;
     if(RECOVERY_LOST_FRAMES<=1 || lost_frames>=RECOVERY_LOST_FRAMES-1) begin
      recovery_state<=REC_SEARCH;lost_frames<=0;
      search_pan_active<=pan_edge_armed;search_tilt_active<=tilt_edge_armed;
      pan_rate_q4<=0;pan_tracking<=0;pending_pan<=0;
     end
    end else if(recovery_state==REC_TRACK) begin
     lost_frames<=0;pan_edge_armed<=0;tilt_edge_armed<=0;
    end
   end
  end
  if(control_tick) begin tick_count<=0;pan_calc<=1;end
  else begin
   tick_count<=tick_count+1'b1;
   case(pan_calc)
    1:begin px<=predict(sx,1279,8,vx,age>=TICK_DIV && history && absdiff(sx,FRAME_CENTER_X)>96);
      pan_edge_calc<=pan_edge_boost;pan_calc<=2;end
    2:begin ex<=absdiff(px,FRAME_CENTER_X);pan_add<=PAN_REVERSE ? !(px>=FRAME_CENTER_X) : px>=FRAME_CENTER_X;pan_calc<=3;end
    3:begin
     // Inside the true 40-pixel circle, retain movement only when the normal
     // hysteresis asks for it, but force that movement to the minimum step.
     raw_pan<=center30_sampled ? (ex>PAN_STOP ? 12'd1 : 12'd0) :
                               pan_band_gain(ex,PAN_STOP,MAX_TRACK_STEP);
     pan_brake<=history && pan_add==pan_last_add && ex<=128 && prev_ex>absdiff(sx,FRAME_CENTER_X)+12'd6;
     pan_move<=!pan_center_hold && (pan_tracking ? ex>PAN_STOP : ex>PAN_START);
     pan_direction_ok<=!pan_has_direction || ex>128 || pan_add==pan_last_add ||
                         (ex>PAN_REVERSE_ERROR && pan_reverse_count>=PAN_REVERSE_FRAMES);
     pan_calc<=4;
    end
    4:begin pan_step<=pan_move && pan_direction_ok ?
        ((pan_brake && raw_pan>1) ? raw_pan-1'b1 : raw_pan) : 0;pan_calc<=5;end
    5:begin desired_pan_q4<=brake_pan_rate((ex<PAN_STOP+12'd8 && landing_pan_q4<nominal_pan_q4) ?
                            landing_pan_q4 : nominal_pan_q4,pan_remaining);pan_calc<=6;end
    6:begin ramp_pan_q4<=ramp_up(pan_add==pan_last_add ? pan_rate_q4 : 16'd0,desired_pan_q4,
                                pan_edge_calc ? PAN_EDGE_ACCEL_Q4 : PAN_ACCEL_Q4);pan_calc<=7;end
    7:begin next_pan_q<=apply_q(pan_q,ramp_pan_q4,pan_add,SAFE_PAN_MIN,SAFE_PAN_MAX);pan_calc<=8;end
    8:begin rounded_pan<={1'b0,next_pan_q}+17'd8;pan_calc<=9;end
    9:pan_calc<=0;
    default:begin end
   endcase
  end
  // Recovery owns the UART only while normal tracking is disabled.  It sends
  // the same proven packets as ordinary tracking, but advances by two pulse
  // units per 120-Hz opportunity and never leaves 1100..1900.
  if(control_tick && state==IDLE && I_enable && LIMITS_OK &&
     recovery_state!=REC_TRACK && reacquire_frames==0 &&
     !(sample_wait==1 && fresh)) begin
   if(recovery_state==REC_SEARCH) begin
    if(recovery_pan_can_move || recovery_tilt_can_move) begin
     cmd_pan<=recovery_pan_can_move ? recovery_pan_next : sent_pan;
     cmd_tilt<=recovery_tilt_can_move ? recovery_tilt_next : sent_tilt;
     cmd_tilt_fine<=1'b1;
     pending_pan<=recovery_pan_can_move && recovery_tilt_can_move;
     axis<=recovery_tilt_can_move ? 1'b0 : 1'b1;
     cmd_index<=0;state<=LOAD;
    end else begin
     recovery_state<=REC_CENTER;
     search_pan_active<=0;search_tilt_active<=0;
    end
   end else if(recovery_state==REC_CENTER) begin
    if(center_pan_can_move || center_tilt_can_move) begin
     cmd_pan<=center_pan_can_move ? center_pan_next : sent_pan;
     cmd_tilt<=center_tilt_can_move ? center_tilt_next : sent_tilt;
     cmd_tilt_fine<=1'b1;
     pending_pan<=center_pan_can_move && center_tilt_can_move;
     axis<=center_tilt_can_move ? 1'b0 : 1'b1;
     cmd_index<=0;state<=LOAD;
    end else begin
     recovery_state<=REC_TRACK;pan_edge_armed<=0;tilt_edge_armed<=0;
     pan_q<=sent_pan<<4;pan_rate_q4<=0;pan_tracking<=0;pan_has_direction<=0;
     trajectory_sync<=1;trajectory_sync_pwm<=sent_tilt;
     last_pan_cmd_valid<=0;last_tilt_cmd_valid<=0;
    end
   end
  end
  // Commit once per120Hz calculation, after its registered datapath settles.
   if(pan_calc==9) begin
    if(pan_center_hold) begin
     // Freeze the numerical trajectory at the last endpoint that really went
     // onto the UART.  Do not let fractional residue become a later kick.
     pan_tracking<=0;pan_q<=sent_pan<<4;pan_rate_q4<=0;
     pan_reverse_count<=0;pan_has_direction<=0;
    end else if(usable && state==IDLE) begin
    pan_tracking<=pan_move;pan_q<=next_pan_q;pan_rate_q4<=ramp_pan_q4;
    if(pan_step!=0) begin pan_last_add<=pan_add;pan_has_direction<=1;end
    if(pan_changed || tilt_changed) begin
     cmd_pan<=next_pan;cmd_tilt<=next_tilt;cmd_tilt_fine<=tilt_packet_fine;pending_pan<=pan_changed && tilt_changed;
     axis<=tilt_changed ? 1'b0 : 1'b1;cmd_index<=0;state<=LOAD;
    end
   end
  end else begin
   // Newest tilt trajectory point may dispatch without advancing pan control.
   if(usable && state==IDLE && tilt_changed) begin
    cmd_tilt<=next_tilt;cmd_tilt_fine<=tilt_packet_fine;cmd_pan<=sent_pan;pending_pan<=0;axis<=0;cmd_index<=0;state<=LOAD;
   end
  end
  case(state)
  IDLE:O_uart_tx<=1;
  // Finish a command already on the wire,never start a queued command while
  // disabled. This prevents an SW3 mode change from leaving a partial packet.
   LOAD:begin
    if(cmd_index==0 && (!I_enable || (axis && pan_center_hold && recovery_state==REC_TRACK))) begin O_uart_tx<=1;pending_pan<=0;state<=IDLE;end
   else begin shift<={1'b1,byte_at(axis,cmd_index,axis ? cmd_pan : cmd_tilt,cmd_tilt_fine)};
    O_uart_tx<=0;baud_count<=BAUD_DIV-1;bit_index<=0;state<=SEND;end
  end
  SEND:begin
   if(baud_count!=0) baud_count<=baud_count-1'b1;
   else begin baud_count<=BAUD_DIV-1;
    if(bit_index==9) begin O_uart_tx<=1;
     if(cmd_index==14) begin cmd_index<=0;
      if(!axis) begin
       if(cmd_tilt!=sent_tilt) begin last_tilt_cmd_add<=cmd_tilt>sent_tilt;last_tilt_cmd_valid<=1;end
       sent_tilt<=cmd_tilt;
       if(pending_pan && I_enable) begin pending_pan<=0;axis<=1;gap_count<=GAP_DIV-1;state<=GAP;end
       else state<=IDLE;
      end else begin
       if(cmd_pan!=sent_pan) begin last_pan_cmd_add<=cmd_pan>sent_pan;last_pan_cmd_valid<=1;end
       sent_pan<=cmd_pan;pending_pan<=0;state<=IDLE;end
     end else begin cmd_index<=cmd_index+1'b1;state<=LOAD;end
    end else begin O_uart_tx<=shift[0];shift<={1'b1,shift[8:1]};bit_index<=bit_index+1'b1;end
   end
  end
   GAP:begin O_uart_tx<=1;
    if(!I_enable || (axis && pan_center_hold && recovery_state==REC_TRACK)) begin pending_pan<=0;state<=IDLE;end
   else if(gap_count==0) state<=LOAD;else gap_count<=gap_count-1'b1;
  end
  default:begin state<=IDLE;O_uart_tx<=1;end
  endcase
 end end
endmodule

// Near-centre stability state for the pan axis.
//
// A centre crossing or entry into ENTER_PIXELS starts CANDIDATE immediately,
// and hold_now blocks new position packets while the physical servo settles.
// The candidate becomes LOCKED after several low-jitter video measurements.
// Once locked, a single noisy coordinate cannot release it: either a large
// error or a coherent displacement from the captured reference must persist.
module axis_center_stability #(
 parameter integer ENTER_PIXELS=12,CROSS_PIXELS=24,EXIT_PIXELS=32,
 parameter integer DISPLACEMENT_PIXELS=14,STABLE_FRAMES=3,
 parameter integer RELEASE_SAMPLES=3,JITTER_PIXELS=4
)(input wire clk,rst_n,enable,measure_strobe,measure_valid,
 input wire signed [12:0] error_in,output reg locked,output wire hold_now);
 reg candidate,history;
 reg signed [12:0] previous_error,reference_error;
 reg [3:0] stable_count,release_count;
 reg release_negative;
 wire [12:0] magnitude=error_in[12] ? -error_in : error_in;
 wire [12:0] previous_magnitude=previous_error[12] ? -previous_error : previous_error;
 wire signed [13:0] frame_delta=$signed({error_in[12],error_in})-
                                  $signed({previous_error[12],previous_error});
 wire [13:0] frame_delta_abs=frame_delta[13] ? -frame_delta : frame_delta;
 wire crossed=history && error_in!=0 && previous_error!=0 &&
              error_in[12]!=previous_error[12] &&
              magnitude<=CROSS_PIXELS && previous_magnitude<=CROSS_PIXELS;
 wire enter_event=magnitude<=ENTER_PIXELS || crossed;
 wire candidate_inside=magnitude<EXIT_PIXELS;
 wire stable_sample=candidate_inside &&
                    (magnitude<=ENTER_PIXELS || frame_delta_abs<=JITTER_PIXELS);
 wire signed [13:0] displacement=$signed({error_in[12],error_in})-
                                    $signed({reference_error[12],reference_error});
 wire [13:0] displacement_abs=displacement[13] ? -displacement : displacement;
 wire release_candidate=magnitude>=EXIT_PIXELS ||
                        (magnitude>ENTER_PIXELS && displacement_abs>=DISPLACEMENT_PIXELS);
 wire release_now=locked && release_candidate &&
                  (RELEASE_SAMPLES<=1 ||
                   (release_count>=RELEASE_SAMPLES-1 && release_negative==displacement[13]));
 wire same_cycle_candidate=measure_strobe && measure_valid && !locked && !candidate && enter_event;
 wire same_cycle_cancel=measure_strobe && measure_valid && candidate && !candidate_inside;
 // Release is committed by the sequential persistence counter.  Keeping the
 // combinational output tied to the registered lock avoids dropping hold one
 // frame early when release_count is updated on the sampling edge.
 assign hold_now=enable &&
                 ((locked || candidate || same_cycle_candidate) && !same_cycle_cancel);
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   locked<=0;candidate<=0;history<=0;previous_error<=0;reference_error<=0;
   stable_count<=0;release_count<=0;release_negative<=0;
  end else if(!enable) begin
   locked<=0;candidate<=0;history<=0;stable_count<=0;release_count<=0;
  end else if(measure_strobe) begin
   if(!measure_valid) begin
    locked<=0;candidate<=0;history<=0;stable_count<=0;release_count<=0;
   end else begin
    history<=1;previous_error<=error_in;
    if(locked) begin
     candidate<=0;stable_count<=0;
     if(release_candidate) begin
      if(release_count==0 || release_negative!=displacement[13]) begin
       release_count<=1;release_negative<=displacement[13];
      end else if(release_count<RELEASE_SAMPLES) release_count<=release_count+1'b1;
      if(release_now) begin locked<=0;release_count<=0;end
     end else release_count<=0;
    end else if(candidate) begin
     release_count<=0;
     if(!candidate_inside) begin candidate<=0;stable_count<=0;end
     else if(stable_sample) begin
      if(STABLE_FRAMES<=1 || stable_count>=STABLE_FRAMES-1) begin
       locked<=1;candidate<=0;stable_count<=0;reference_error<=error_in;
      end else stable_count<=stable_count+1'b1;
     end else stable_count<=0;
    end else begin
     release_count<=0;stable_count<=0;
     if(enter_event) begin candidate<=1;reference_error<=error_in;end
    end
   end
  end
 end
endmodule

// Velocity is signedQ8 pulse-width microseconds per control tick. Derivative
// is low-pass filteredQ8 pixels per60Hz measurement, not physical shaft speed.
module tilt_pd_trajectory #(
 parameter integer CLK_HZ=75000000,CONTROL_HZ=120,PACKET_MS=10,
 parameter integer CENTER_Y=360,PWM_CENTER=1500,PWM_MIN=1000,PWM_MAX=2000,
 parameter integer STOP_PIXELS=10,START_PIXELS=14,
 parameter integer KP_Q8=8,KD_Q8=32,ACCEL_Q8=32,DECEL_Q8=64,
 parameter integer MAX_NOMINAL_STEP=4,STEP_SHIFT=1,MAX_PACKET_STEP=3,
 parameter integer REVERSE_PIXELS=48,REVERSE_SAMPLES=2,
 parameter CENTER_LOCK_ENABLE=1'b1,
 parameter integer LOCK_CROSS=24,LOCK_EXIT=32,LOCK_DISPLACEMENT=12,
 parameter integer LOCK_SETTLE_FRAMES=4,LOCK_RELEASE_SAMPLES=2,
 parameter REVERSE=1'b1,
 parameter integer PACKET_REFERENCE_MS=20,
 parameter integer LIMIT_BRAKE_WINDOW=80,
 parameter EDGE_CATCH_ENABLE=1'b1,
 parameter integer FRAME_PIXELS=720,EDGE_MARGIN=96,EDGE_WARMUP_SAMPLES=8,EDGE_CONFIRM_SAMPLES=3,
 parameter integer EDGE_MIN_OUTWARD_PIXELS=2,EDGE_MAX_OUTWARD_PIXELS=48,
 parameter integer EDGE_BURST_MS=100,EDGE_COOLDOWN_MS=500,
 parameter integer EDGE_ACCEL_Q8=ACCEL_Q8+(ACCEL_Q8>>2),
 parameter integer FINE_PIXELS=48,FINE_PACKET_MS=5,FINE_RATE_Q8=128,
 parameter integer CENTER_FINE_RATE_Q8=64,
 parameter integer FAST_PIXELS=96,FAST_RATE_Q8=320,FAST_ACCEL_Q8=40,
 parameter integer EDGE200_RATE_Q8=352,EDGE150_RATE_Q8=384,
 parameter integer EDGE100_RATE_Q8=416,EDGE50_RATE_Q8=448,
 parameter integer EDGE200_ACCEL_Q8=48,EDGE150_ACCEL_Q8=56,
 parameter integer EDGE100_ACCEL_Q8=64,EDGE50_ACCEL_Q8=72,
 parameter MOTION_FOLLOW_ENABLE=1'b1
)(input wire clk,rst_n,enable,position_sync,input wire [11:0] sync_pwm,
 input wire measure_strobe,measure_valid,
 input wire fine_region,input wire [11:0] cy,input wire control_tick,packet_ack,
 output reg packet_pending,output reg [11:0] packet_pwm,output wire inhibit_packet,
 output reg packet_fine);
 localparam integer PACKET_CYCLES=(CLK_HZ/1000)*PACKET_MS;
 // A3us/20ms packet limit becomes1.5us/10ms,not3us every10ms.
 // Fractional carry alternates1/2us. Only constant division/modulo is used;
 // there is no new runtime divider or longer combinational forecast path.
 localparam [12:0] PACKET_STEP_WHOLE=(MAX_PACKET_STEP*PACKET_MS)/PACKET_REFERENCE_MS;
 localparam integer PACKET_FRACTION_BITS=PACKET_REFERENCE_MS<=1 ? 1 : $clog2(PACKET_REFERENCE_MS);
 localparam [PACKET_FRACTION_BITS-1:0] PACKET_STEP_FRACTION=(MAX_PACKET_STEP*PACKET_MS)%PACKET_REFERENCE_MS;
 localparam integer FINE_CYCLES=(CLK_HZ/1000)*FINE_PACKET_MS;
 localparam [12:0] FINE_STEP_WHOLE=(MAX_PACKET_STEP*FINE_PACKET_MS)/PACKET_REFERENCE_MS;
 localparam [PACKET_FRACTION_BITS-1:0] FINE_STEP_FRACTION=(MAX_PACKET_STEP*FINE_PACKET_MS)%PACKET_REFERENCE_MS;
 wire signed [31:0] error=$signed({1'b0,cy})-CENTER_Y;
 wire signed [31:0] error_abs=error<0 ? -error : error;
 reg pd_fine,pd_center_fine,pd_fast;
 reg [2:0] pd_edge_zone;
 reg [PACKET_FRACTION_BITS-1:0] packet_step_remainder;
 reg [12:0] packet_step_limit;
 // Scheduling uses the last accepted visual measurement, never a transient
 // or unvalidated live coordinate between measurement strobes.
 wire fine_measure=(fine_region || error_abs<=FINE_PIXELS) &&
                   FINE_PACKET_MS>0 && FINE_PACKET_MS<PACKET_MS;
 wire fast_measure=!fine_region && error_abs>=FAST_PIXELS;
 wire [11:0] edge_distance=cy>=FRAME_PIXELS ? 12'd0 :
                         (cy<=(FRAME_PIXELS-1)/2 ? cy : FRAME_PIXELS-1-cy);
 wire [2:0] edge_zone_measure=fine_region || edge_distance>12'd200 ? 3'd0 :
                              (edge_distance<=12'd50 ? 3'd4 :
                              (edge_distance<=12'd100 ? 3'd3 :
                              (edge_distance<=12'd150 ? 3'd2 : 3'd1)));
 wire fine_now=pd_fine;
 wire [PACKET_FRACTION_BITS:0] packet_step_sum={1'b0,packet_step_remainder}+
     {1'b0,(fine_now ? FINE_STEP_FRACTION : PACKET_STEP_FRACTION)};
 wire packet_step_carry=packet_step_sum>=PACKET_REFERENCE_MS;
 localparam signed [31:0] RATE_CAP=(MAX_NOMINAL_STEP*15360/CONTROL_HZ)>>STEP_SHIFT;
 localparam signed [31:0] MIN_Q8=PWM_MIN*256,MAX_Q8=PWM_MAX*256;
 reg valid_seen,history,active;
 reg [31:0] age,packet_count;
 reg signed [31:0] position_q8,velocity_q8,target_q8,previous_error,derivative_q8;
 reg [2:0] pd_state;
 reg pd_history,pd_active;
 reg [1:0] pd_brake_zone;
 reg signed [31:0] pd_delta,pd_bound,pd_soft,pd_old_derivative;
 reg signed [31:0] pd_p,pd_d,pd_sum;
 reg [11:0] last_dispatched_pwm;
 reg motion_known,last_motion_negative;
 reg [3:0] reverse_count;
 wire signed [31:0] delta=error-previous_error;
 wire requested_negative=REVERSE ? error>0 : error<0;
 wire edge_accel;
 wire [11:0] tilt_position=position_q8[19:8];
 wire [11:0] tilt_remaining=requested_negative ?
     (tilt_position<=PWM_MIN ? 12'd0 : tilt_position-PWM_MIN) :
     (tilt_position>=PWM_MAX ? 12'd0 : PWM_MAX-tilt_position);
 wire [1:0] brake_zone=LIMIT_BRAKE_WINDOW<=0 ? 2'd0 :
    (tilt_remaining<=LIMIT_BRAKE_WINDOW/10 ? 2'd3 :
     (tilt_remaining<=LIMIT_BRAKE_WINDOW/2 ? 2'd2 :
      (tilt_remaining<=LIMIT_BRAKE_WINDOW ? 2'd1 : 2'd0)));
 servo_guarded_edge #(.FRAME_PIXELS(FRAME_PIXELS),.ENTER_MARGIN(EDGE_MARGIN),.CONTROL_HZ(CONTROL_HZ),
  .WARMUP_SAMPLES(EDGE_WARMUP_SAMPLES),.CONFIRM_SAMPLES(EDGE_CONFIRM_SAMPLES),
  .MIN_OUTWARD_PIXELS(EDGE_MIN_OUTWARD_PIXELS),.MAX_OUTWARD_PIXELS(EDGE_MAX_OUTWARD_PIXELS),
  .BURST_MS(EDGE_BURST_MS),.COOLDOWN_MS(EDGE_COOLDOWN_MS)) u_tilt_guarded_edge (
  .clk(clk),.rst_n(rst_n),.control_tick(control_tick),
  .enable(enable && EDGE_CATCH_ENABLE && (age<CLK_HZ/30 || measure_strobe)),
  .allow_boost(tilt_position>PWM_MIN+LIMIT_BRAKE_WINDOW && tilt_position<PWM_MAX-LIMIT_BRAKE_WINDOW),
  .measure_strobe(measure_strobe),.measure_valid(measure_valid),.position(cy),.boost(edge_accel));
 wire opposite_near=motion_known && requested_negative!=last_motion_negative && error_abs<=REVERSE_PIXELS;
 wire center_locked,center_hold,motion_follow,motion_release;
 wire reverse_allowed=motion_release || !opposite_near ||
                      (error_abs>START_PIXELS && reverse_count>=REVERSE_SAMPLES-1);
 tilt_center_lock #(.STOP_PIXELS(STOP_PIXELS),.START_PIXELS(START_PIXELS),
  .CROSS_PIXELS(LOCK_CROSS),.PREVIOUS_CROSS_PIXELS(REVERSE_PIXELS),.EXIT_PIXELS(LOCK_EXIT),
  .DISPLACEMENT_PIXELS(LOCK_DISPLACEMENT),.SETTLE_FRAMES(LOCK_SETTLE_FRAMES),
  .RELEASE_SAMPLES(LOCK_RELEASE_SAMPLES),.MOTION_ENABLE(MOTION_FOLLOW_ENABLE)) u_center_lock (
  .clk(clk),.rst_n(rst_n),
  .enable(enable && CENTER_LOCK_ENABLE && (age<CLK_HZ/30 || (measure_strobe && measure_valid))),
  .measure_strobe(measure_strobe),.measure_valid(measure_valid),.error_in(error[12:0]),
  .locked(center_locked),.hold_now(center_hold),.motion_follow(motion_follow),.motion_release(motion_release)
 );
 wire active_next=(motion_follow ? error_abs>4 : (active ? error_abs>STOP_PIXELS : error_abs>START_PIXELS)) && reverse_allowed && !center_hold;
 // Same-cycle veto prevents the UART from accepting an old pending waypoint
 // precisely when a fresh measurement says stop or unconfirmed near reversal.
 assign inhibit_packet=center_hold || (measure_strobe && measure_valid && !active_next);
 wire signed [31:0] effective_stop=motion_follow ? 32'sd4 : STOP_PIXELS;
 wire signed [31:0] soft_error=error>effective_stop ? error-effective_stop :
                                  (error< -effective_stop ? error+effective_stop : 32'sd0);
 // An approaching target may require braking, not a premature reverse pulse.
 // D may reduce P to zero; it cannot alone reverse the positional correction.
 wire signed [31:0] image_rate=(pd_soft>0 && pd_sum<0) ||
                             (pd_soft<0 && pd_sum>0) ? 32'sd0 : pd_sum;
 wire signed [31:0] command_rate=REVERSE ? -image_rate : image_rate;
 // No minimum velocity floor near the screen centre. Q8 carries sub-unit
 // corrections forward; the nearer speed cap only reduces existing limits.
 wire signed [31:0] fine_cap=pd_fine && FINE_RATE_Q8>0 && FINE_RATE_Q8<RATE_CAP ?
                               FINE_RATE_Q8 : RATE_CAP;
 function signed [31:0] edge_rate_limit;
 input [2:0] zone;
 begin case(zone)
  3'd1:edge_rate_limit=EDGE200_RATE_Q8;
  3'd2:edge_rate_limit=EDGE150_RATE_Q8;
  3'd3:edge_rate_limit=EDGE100_RATE_Q8;
  3'd4:edge_rate_limit=EDGE50_RATE_Q8;
  default:edge_rate_limit=32'sd0;
 endcase end
 endfunction
 function signed [31:0] edge_accel_limit;
 input [2:0] zone;
 begin case(zone)
  3'd1:edge_accel_limit=EDGE200_ACCEL_Q8;
  3'd2:edge_accel_limit=EDGE150_ACCEL_Q8;
  3'd3:edge_accel_limit=EDGE100_ACCEL_Q8;
  3'd4:edge_accel_limit=EDGE50_ACCEL_Q8;
  default:edge_accel_limit=32'sd0;
 endcase end
 endfunction
 wire signed [31:0] progressive_edge_cap=edge_rate_limit(pd_edge_zone);
 wire signed [31:0] progressive_edge_accel=edge_accel_limit(pd_edge_zone);
 wire signed [31:0] far_cap=pd_edge_zone!=0 && progressive_edge_cap>RATE_CAP ?
                               progressive_edge_cap :
                               (pd_fast && FAST_RATE_Q8>RATE_CAP ? FAST_RATE_Q8 : RATE_CAP);
 // Centre and fine caps have priority.  The fast cap is available only for a
 // genuinely large measured vertical error, so it cannot amplify centre
 // jitter or a one-pixel crossing.
 wire signed [31:0] selected_cap=pd_center_fine && CENTER_FINE_RATE_Q8>0 &&
                                  CENTER_FINE_RATE_Q8<fine_cap ? CENTER_FINE_RATE_Q8 :
                                  (pd_fine ? fine_cap : far_cap);
 wire signed [31:0] bounded_rate=command_rate>selected_cap ? selected_cap :
                                     (command_rate< -selected_cap ? -selected_cap : command_rate);
 wire slowing=(velocity_q8>0 && target_q8<velocity_q8) || (velocity_q8<0 && target_q8>velocity_q8);
 wire signed [31:0] change_limit=slowing ? DECEL_Q8 :
                                      (pd_edge_zone!=0 && progressive_edge_accel>ACCEL_Q8 ? progressive_edge_accel :
                                      (edge_accel ? EDGE_ACCEL_Q8 :
                                      (pd_fast && FAST_ACCEL_Q8>ACCEL_Q8 ? FAST_ACCEL_Q8 : ACCEL_Q8)));
 wire signed [31:0] rate_delta=target_q8-velocity_q8;
 wire signed [31:0] new_velocity=velocity_q8+(rate_delta>change_limit ? change_limit :
                                      (rate_delta< -change_limit ? -change_limit : rate_delta));
 wire signed [31:0] new_position=position_q8+new_velocity;
 // The routed failing path was the single-cycle signed divide-by1000 plus
 // forecast/clamp/packet limiter. Keep the exact quotient, but compute it
 // iteratively: one11-bit subtract per clock, not a49-level combinational path.
 reg [3:0] packet_state;
 reg [5:0] div_count;
 reg signed [31:0] snap_position,product,lead_q8,forecast_q8;
 reg signed [31:0] snap_velocity;
 reg div_negative,snap_fine;
 reg [31:0] quotient;
 reg [9:0] remainder;
 reg [31:0] rounded_forecast;
 reg [11:0] proposed;
 reg [12:0] upper_pwm,lower_pwm;
 reg [11:0] directed_pwm,packet_base_snapshot;
 wire [10:0] div_trial={remainder,quotient[31]};
 // Default fine period is exactly half the far period. Keep the existing
 // constant multiplier, then select a signed shift, not a variable multiply.
 wire signed [31:0] divide_product=(FINE_PACKET_MS*2==PACKET_MS && snap_fine) ? (product>>>1) : product;
 wire div_subtract=div_trial>=11'd1000;
 wire [11:0] base_pwm=packet_ack ? packet_pwm : last_dispatched_pwm;
 wire [12:0] bounded_packet=({1'b0,directed_pwm}>upper_pwm) ? upper_pwm :
                         ({1'b0,directed_pwm}<lower_pwm ? lower_pwm : {1'b0,directed_pwm});
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   valid_seen<=0;history<=0;active<=0;age<=CLK_HZ;packet_count<=0;
   position_q8<=PWM_CENTER*256;velocity_q8<=0;target_q8<=0;previous_error<=0;derivative_q8<=0;
   pd_state<=0;pd_history<=0;pd_active<=0;pd_fine<=0;pd_center_fine<=0;pd_fast<=0;pd_edge_zone<=0;pd_delta<=0;pd_bound<=0;pd_soft<=0;
   pd_brake_zone<=0;
   pd_old_derivative<=0;pd_p<=0;pd_d<=0;pd_sum<=0;
   packet_pending<=0;packet_pwm<=PWM_CENTER;packet_fine<=0;snap_fine<=0;last_dispatched_pwm<=PWM_CENTER;
   motion_known<=0;last_motion_negative<=0;reverse_count<=0;
   packet_step_remainder<=0;packet_step_limit<=PACKET_STEP_WHOLE;
   packet_state<=0;div_count<=0;snap_position<=0;snap_velocity<=0;
   product<=0;lead_q8<=0;forecast_q8<=0;div_negative<=0;quotient<=0;
   remainder<=0;rounded_forecast<=0;proposed<=PWM_CENTER;
   upper_pwm<=PWM_CENTER;lower_pwm<=PWM_CENTER;directed_pwm<=PWM_CENTER;
   packet_base_snapshot<=PWM_CENTER;
  end else if(position_sync) begin
   // Recovery packets bypass this trajectory generator.  Re-anchor every
   // internal state to the last UART endpoint before normal tracking resumes,
   // otherwise the first reacquired frame could jump back to a stale position.
   valid_seen<=0;history<=0;active<=0;age<=CLK_HZ;packet_count<=0;
   position_q8<=$signed({1'b0,sync_pwm,8'd0});velocity_q8<=0;target_q8<=0;
   previous_error<=0;derivative_q8<=0;packet_pending<=0;packet_pwm<=sync_pwm;
   packet_fine<=0;last_dispatched_pwm<=sync_pwm;packet_state<=0;pd_state<=0;
   pd_fine<=0;pd_center_fine<=0;pd_fast<=0;pd_edge_zone<=0;
   motion_known<=0;reverse_count<=0;
   packet_step_remainder<=0;packet_step_limit<=PACKET_STEP_WHOLE;
  end else if(!enable) begin
   valid_seen<=0;history<=0;active<=0;age<=CLK_HZ;packet_count<=0;
   velocity_q8<=0;target_q8<=0;derivative_q8<=0;packet_pending<=0;
   packet_state<=0;
   pd_state<=0;pd_fine<=0;pd_center_fine<=0;pd_fast<=0;pd_edge_zone<=0;
   motion_known<=0;reverse_count<=0;
   packet_step_remainder<=0;packet_step_limit<=PACKET_STEP_WHOLE;
  end else begin
   if(age<CLK_HZ) age<=age+1'b1;
   if(packet_ack) begin
    packet_pending<=0;last_dispatched_pwm<=packet_pwm;
    if(packet_pwm!=last_dispatched_pwm) begin
     motion_known<=1;last_motion_negative<=packet_pwm<last_dispatched_pwm;
    end
   end
   if(measure_strobe) begin
    if(measure_valid) begin
     age<=0;valid_seen<=1;history<=1;active<=active_next;
     previous_error<=error;
     // Release with a clean derivative, not stored motor ring-down history.
     pd_delta<=center_locked ? 32'sd0 : delta;pd_soft<=soft_error;pd_active<=active_next;
     pd_history<=history && !center_locked;pd_old_derivative<=center_locked ? 32'sd0 : derivative_q8;pd_state<=1;
     pd_brake_zone<=brake_zone;
     pd_fine<=fine_measure;pd_center_fine<=fine_region;pd_fast<=fast_measure;
     pd_edge_zone<=edge_zone_measure;
     // Count video measurements, never120Hz control ticks. A one-frame
     // opposite-side blip cannot request a return stroke near the centre.
     if(motion_release) begin motion_known<=0;reverse_count<=0;end
     else if(!opposite_near || error_abs<=START_PIXELS) reverse_count<=0;
     else if(reverse_count<REVERSE_SAMPLES) reverse_count<=reverse_count+1'b1;
    end else begin
     valid_seen<=0;history<=0;active<=0;velocity_q8<=0;target_q8<=0;derivative_q8<=0;
     packet_pending<=0;packet_count<=0;
     packet_state<=0;
     pd_state<=0;pd_fine<=0;pd_center_fine<=0;pd_fast<=0;pd_edge_zone<=0;
     motion_known<=0;reverse_count<=0;
     packet_step_remainder<=0;packet_step_limit<=PACKET_STEP_WHOLE;
    end
   end
   if(valid_seen && age<CLK_HZ/30 && !(measure_strobe && !measure_valid)) begin
    case(pd_state)
     1:begin pd_bound<=pd_delta>64 ? 32'sd64 : (pd_delta< -64 ? -32'sd64 : pd_delta);pd_state<=2;end
     2:begin derivative_q8<=pd_history ? pd_old_derivative+(((pd_bound <<< 8)-pd_old_derivative) >>> 2) : 32'sd0;
       pd_p<=pd_soft*KP_Q8;pd_state<=3;end
     3:begin pd_d<=(derivative_q8*KD_Q8) >>> 8;pd_state<=4;end
     4:begin pd_sum<=pd_p+pd_d;pd_state<=5;end
     5:begin
      if(!pd_active) target_q8<=0;
      else case(pd_brake_zone)
       2'd1:target_q8<=bounded_rate>>>1;
       2'd2:target_q8<=bounded_rate>>>2;
       2'd3:target_q8<=bounded_rate>>>3;
       default:target_q8<=bounded_rate;
      endcase
      pd_state<=0;
     end
     default:begin end
    endcase
    if(control_tick) begin
     if(new_position>MAX_Q8) begin position_q8<=MAX_Q8;velocity_q8<=0;end
     else if(new_position<MIN_Q8) begin position_q8<=MIN_Q8;velocity_q8<=0;end
     // Forecasted endpoints can be ahead of the integrator. At rest, anchor
     // the virtual trajectory to the last dispatched endpoint; do not slowly
     // command the shaft backwards to catch up with an obsolete virtual q.
     else if(!active) begin
      position_q8<=$signed({1'b0,base_pwm,8'd0});velocity_q8<=new_velocity;
     end else begin position_q8<=new_position;velocity_q8<=new_velocity;end
    end
    if(packet_count>=(fine_now ? FINE_CYCLES : PACKET_CYCLES)-1) begin
     packet_count<=0;
     if(active) begin
      snap_position<=position_q8;snap_velocity<=velocity_q8;snap_fine<=fine_now;packet_state<=1;
      // 3 units/20ms remains the budget at either cadence. Fine packets are
      // additionally limited to ONE integer pulse-width unit, never a kick.
      packet_step_limit<=fine_now ?
        ((FINE_STEP_WHOLE+(packet_step_carry ? 13'd1 : 13'd0))>1 ? 13'd1 :
          FINE_STEP_WHOLE+(packet_step_carry ? 13'd1 : 13'd0)) :
        PACKET_STEP_WHOLE+(packet_step_carry ? 13'd1 : 13'd0);
      packet_step_remainder<=packet_step_carry ?
          packet_step_sum-PACKET_REFERENCE_MS : packet_step_sum[PACKET_FRACTION_BITS-1:0];
     end
    end else packet_count<=packet_count+1'b1;
    case(packet_state)
     1:begin
      if(FINE_PACKET_MS*2==PACKET_MS) product<=snap_velocity*(CONTROL_HZ*PACKET_MS);
      else product<=snap_fine ? snap_velocity*(CONTROL_HZ*FINE_PACKET_MS) : snap_velocity*(CONTROL_HZ*PACKET_MS);
      packet_state<=2;
     end
     2:begin div_negative<=divide_product<0;quotient<=divide_product<0 ? -divide_product : divide_product;
      remainder<=0;div_count<=0;packet_state<=3;end
     3:begin
      remainder<=div_subtract ? div_trial-11'd1000 : div_trial[9:0];
      quotient<={quotient[30:0],div_subtract};
      if(div_count==31) packet_state<=4;else div_count<=div_count+1'b1;
     end
     4:begin lead_q8<=div_negative ? -$signed(quotient) : $signed(quotient);packet_state<=5;end
     5:begin forecast_q8<=snap_position+lead_q8;packet_state<=6;end
     6:begin rounded_forecast<=(forecast_q8<MIN_Q8 ? MIN_Q8 :
                             (forecast_q8>MAX_Q8 ? MAX_Q8 : forecast_q8))+32'sd128;
       packet_state<=7;end
     7:begin proposed<=rounded_forecast[19:8];packet_state<=8;end
     8:begin
      // Break the visual-valid -> UART-ack -> packet-limiter critical path.
      // If an older packet is accepted during this stage, resample its new
      // endpoint before computing limits, so the budget uses the latest base.
      packet_base_snapshot<=base_pwm;
      packet_state<=10;
     end
     10:begin
      if(packet_ack) begin
       packet_base_snapshot<=packet_pwm;
      end else begin
      upper_pwm<={1'b0,packet_base_snapshot}+packet_step_limit;
      lower_pwm<={1'b0,packet_base_snapshot}-packet_step_limit;
      // Decreasing look-ahead during braking must not become a reverse
      // position command. Reversal is permitted when real velocity reverses.
      if(snap_velocity>0) directed_pwm<=proposed<packet_base_snapshot ? packet_base_snapshot : proposed;
      else if(snap_velocity<0) directed_pwm<=proposed>packet_base_snapshot ? packet_base_snapshot : proposed;
      else directed_pwm<=packet_base_snapshot;
      packet_state<=9;
      end
     end
     9:begin packet_pwm<=bounded_packet[11:0];packet_fine<=snap_fine;packet_pending<=1;packet_state<=0;end
     default:begin end
    endcase
    // Do not integrate extra unsent coast after entering the stop region.
    // The last already-dispatched command can complete smoothly.
    if(inhibit_packet) begin
     packet_pending<=0;packet_state<=0;position_q8<=$signed({1'b0,base_pwm,8'd0});
     packet_step_remainder<=0;packet_step_limit<=PACKET_STEP_WHOLE;
    end
    if(center_hold) begin
     active<=0;velocity_q8<=0;target_q8<=0;derivative_q8<=0;pd_state<=0;
    end
   end
   if(age>=CLK_HZ/30 && !(measure_strobe && measure_valid)) begin
    valid_seen<=0;history<=0;active<=0;velocity_q8<=0;target_q8<=0;derivative_q8<=0;
    packet_pending<=0;packet_count<=0;
    packet_state<=0;
    pd_state<=0;
    motion_known<=0;reverse_count<=0;
    packet_step_remainder<=0;packet_step_limit<=PACKET_STEP_WHOLE;
   end
  end
 end
endmodule

// A latched hold, not another per-frame deadband test. Hold the LAST dispatched
// endpoint while motor interpolation and delayed vision settle. Reference is
// captured AFTER settling; a new persistent displacement or large error releases.
module tilt_center_lock #(
 parameter integer STOP_PIXELS=10,START_PIXELS=14,CROSS_PIXELS=24,PREVIOUS_CROSS_PIXELS=48,
 parameter integer EXIT_PIXELS=32,DISPLACEMENT_PIXELS=12,SETTLE_FRAMES=4,RELEASE_SAMPLES=2,
 parameter MOTION_ENABLE=1'b1
)(input wire clk,rst_n,enable,measure_strobe,measure_valid,
 input wire signed [12:0] error_in,output reg locked,output wire hold_now,
 output wire motion_follow,motion_release);
 reg history,following;
 reg signed [12:0] previous_error,reference_error,capture_error;
 reg [3:0] settle_left,change_count;
 reg [2:0] outward_count,offset_count,quiet_count;
 reg offset_negative;
 reg change_negative;
 wire [12:0] magnitude=error_in[12] ? -error_in : error_in;
 wire [12:0] previous_magnitude=previous_error[12] ? -previous_error : previous_error;
 wire crossed=history && error_in!=0 && previous_error!=0 &&
  error_in[12]!=previous_error[12] && magnitude<=CROSS_PIXELS && previous_magnitude<=PREVIOUS_CROSS_PIXELS;
 // Follow a demonstrated slow move down to four screen pixels; otherwise
 // retain the original noise-immune capture. Do not relock at every8px sample.
 wire fine_quiet=following && magnitude<=STOP_PIXELS && history && error_in==previous_error;
 wire capture=(following ? magnitude<=4 : magnitude<=STOP_PIXELS) || crossed ||
               (fine_quiet && quiet_count>=3);
 wire signed [13:0] displacement=$signed({error_in[12],error_in})-$signed({reference_error[12],reference_error});
 wire [13:0] displacement_abs=displacement[13] ? -displacement : displacement;
 // The inner capture band stays quiet even if its error changes sign twice.
 // The old two-frame reversal filter alone could not guarantee this hold.
 wire displaced=magnitude>CROSS_PIXELS && magnitude>START_PIXELS && displacement_abs>=DISPLACEMENT_PIXELS;
 wire release_small=settle_left==0 && displaced &&
  (RELEASE_SAMPLES<=1 || (change_count>=RELEASE_SAMPLES-1 && change_negative==displacement[13]));
 wire release_large=magnitude>=EXIT_PIXELS;
 wire signed [13:0] frame_delta=$signed({error_in[12],error_in})-$signed({previous_error[12],previous_error});
 wire [13:0] frame_delta_abs=frame_delta[13] ? -frame_delta : frame_delta;
 wire signed [13:0] from_capture=$signed({error_in[12],error_in})-$signed({capture_error[12],capture_error});
 wire [13:0] capture_displacement=from_capture[13] ? -from_capture : from_capture;
 // Frozen capture anchor avoids absorbing a genuine move into the settling
 // reference. Three small OUTWARD increments are evidence, not one noisy jump.
 wire outward=history && (previous_error==0 || error_in[12]==previous_error[12]) &&
               magnitude>previous_magnitude && frame_delta_abs>=1 && frame_delta_abs<=12;
 // During the settling observation window, 6..12 pixel outward motion is
 // treated as delayed mechanical/visual ring-down.  After that window the
 // original three coherent increments still release promptly, so a genuinely
 // moving target does not have to reach the large EXIT threshold.
 wire coherent=settle_left==0 && outward && outward_count>=2 &&
               magnitude>=8 && capture_displacement>=6;
 wire offset_candidate=magnitude>=CROSS_PIXELS &&
                       capture_displacement>=DISPLACEMENT_PIXELS;
 wire persistent_offset=settle_left==0 && offset_candidate && offset_count>=3 &&
                        offset_negative==error_in[12];
 wire early_release=MOTION_ENABLE && locked && (coherent || persistent_offset);
 wire next_locked=locked ? !(release_large || release_small || early_release) : (capture && !release_large);
 assign hold_now=enable && ((measure_strobe && measure_valid) ? next_locked : locked);
 assign motion_release=enable && measure_strobe && measure_valid && early_release;
 assign motion_follow=enable && MOTION_ENABLE && !((measure_strobe && measure_valid) ? next_locked : locked) &&
                      (following || motion_release);
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   locked<=0;history<=0;previous_error<=0;reference_error<=0;capture_error<=0;
   following<=0;outward_count<=0;offset_count<=0;offset_negative<=0;quiet_count<=0;
   settle_left<=0;change_count<=0;change_negative<=0;
  end else if(!enable) begin
   locked<=0;history<=0;settle_left<=0;change_count<=0;
   following<=0;outward_count<=0;offset_count<=0;quiet_count<=0;
  end else if(measure_strobe) begin
   if(!measure_valid) begin history<=0;change_count<=0;following<=0;outward_count<=0;offset_count<=0;quiet_count<=0;end
   else begin
    history<=1;previous_error<=error_in;locked<=next_locked;
    if(next_locked) following<=0;
    else if(early_release) following<=1;
    if(fine_quiet) begin if(quiet_count<4) quiet_count<=quiet_count+1'b1;end
    else quiet_count<=0;
    if(locked && outward) begin if(outward_count<7) outward_count<=outward_count+1'b1;end
    else outward_count<=0;
    if(locked && settle_left==0 && offset_candidate) begin
     if(offset_count==0 || offset_negative!=error_in[12]) begin offset_count<=1;offset_negative<=error_in[12];end
     else if(offset_count<7) offset_count<=offset_count+1'b1;
    end else offset_count<=0;
    if(!locked && next_locked) begin
     settle_left<=SETTLE_FRAMES;reference_error<=error_in;capture_error<=error_in;change_count<=0;
    end else if(locked && next_locked) begin
     if(settle_left!=0) begin
      settle_left<=settle_left-1'b1;change_count<=0;
      if(settle_left==1) reference_error<=error_in;
     end else if(!displaced) change_count<=0;
     else if(change_count==0 || change_negative!=displacement[13]) begin
      change_count<=1;change_negative<=displacement[13];
     end else if(change_count<RELEASE_SAMPLES-1) change_count<=change_count+1'b1;
    end else begin settle_left<=0;change_count<=0;end
   end
  end
 end
endmodule

// A bounded acceleration assist,not a higher speed controller. Control ticks
// expire each burst even if visual coordinates remain stuck near an edge.
module servo_guarded_edge #(
 parameter integer FRAME_PIXELS=1280,ENTER_MARGIN=160,CONTROL_HZ=120,
 parameter integer WARMUP_SAMPLES=8,CONFIRM_SAMPLES=3,
 parameter integer MIN_OUTWARD_PIXELS=2,MAX_OUTWARD_PIXELS=48,
 parameter integer BURST_MS=100,COOLDOWN_MS=500
)(input wire clk,rst_n,enable,allow_boost,control_tick,measure_strobe,measure_valid,
 input wire [11:0] position,output reg boost);
 localparam integer BURST_TICKS=(CONTROL_HZ*BURST_MS+999)/1000;
 localparam integer COOLDOWN_TICKS=(CONTROL_HZ*COOLDOWN_MS+999)/1000;
 reg history,left_evidence,side_left;
 reg [11:0] previous;
 reg [7:0] warmup,evidence,quiet_samples,burst_left,cooldown_left;
 wire signed [12:0] displacement=$signed({1'b0,position})-$signed({1'b0,previous});
 wire left_out=history && position<=ENTER_MARGIN && previous<=ENTER_MARGIN &&
       displacement<=-MIN_OUTWARD_PIXELS && displacement>=-MAX_OUTWARD_PIXELS;
 wire right_out=history && position>=FRAME_PIXELS-1-ENTER_MARGIN && previous>=FRAME_PIXELS-1-ENTER_MARGIN &&
       displacement>=MIN_OUTWARD_PIXELS && displacement<=MAX_OUTWARD_PIXELS;
 wire outward=left_out || right_out;
 wire inward=side_left ? displacement>=MIN_OUTWARD_PIXELS : displacement<=-MIN_OUTWARD_PIXELS;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   history<=0;previous<=0;left_evidence<=0;side_left<=0;warmup<=0;evidence<=0;
   quiet_samples<=0;burst_left<=0;cooldown_left<=0;boost<=0;
  end else if(!enable) begin
   history<=0;warmup<=0;evidence<=0;quiet_samples<=0;burst_left<=0;cooldown_left<=0;boost<=0;
  end else begin
   if(control_tick) begin
    if(cooldown_left!=0) cooldown_left<=cooldown_left-1'b1;
    if(boost) begin
     if(burst_left<=1) begin boost<=0;burst_left<=0;cooldown_left<=COOLDOWN_TICKS;evidence<=0;end
     else burst_left<=burst_left-1'b1;
    end
   end
   if(!allow_boost) begin
    boost<=0;burst_left<=0;evidence<=0;quiet_samples<=0;
    if(boost) cooldown_left<=COOLDOWN_TICKS;
   end
   if(measure_strobe) begin
    if(!measure_valid || position>=FRAME_PIXELS) begin
     history<=0;warmup<=0;evidence<=0;quiet_samples<=0;burst_left<=0;
     boost<=0;cooldown_left<=COOLDOWN_TICKS;
    end else begin
     history<=1;previous<=position;
     if(warmup<WARMUP_SAMPLES) warmup<=warmup+1'b1;
     if(!allow_boost || warmup<WARMUP_SAMPLES || cooldown_left!=0) begin evidence<=0;end
     else if(boost) begin
      if(inward || (outward && left_out!=side_left) || (!outward && quiet_samples>=1)) begin
       boost<=0;burst_left<=0;evidence<=0;quiet_samples<=0;cooldown_left<=COOLDOWN_TICKS;
      end else if(outward) quiet_samples<=0;
      else quiet_samples<=quiet_samples+1'b1;
     end else if(outward) begin
      if(CONFIRM_SAMPLES<=1 || (evidence!=0 && left_evidence==left_out && evidence>=CONFIRM_SAMPLES-1)) begin
       boost<=1;side_left<=left_out;evidence<=0;quiet_samples<=0;burst_left<=BURST_TICKS;
      end else begin
       left_evidence<=left_out;
       if(evidence==0 || left_evidence!=left_out) evidence<=1;
       else if(evidence<255) evidence<=evidence+1'b1;
      end
     end else evidence<=0;
    end
   end
  end
 end
endmodule
`default_nettype wire
