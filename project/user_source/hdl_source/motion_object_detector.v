`default_nettype none
// Stabilise a single motion box without changing the underlying detector.
// The centre follows adaptively; width/height change at a limited rate.
module motion_object_detector #(
 parameter [5:0] BG_THRESHOLD=6,MOVE_THRESHOLD=5,
 parameter integer MIN_AREA=12,MAX_AREA=7200,MIN_SEEDS=2,
 parameter integer GLOBAL_CHANGE_LIMIT=7200,HOLD_FRAMES=3,SLOTS=2,
 parameter integer SMOOTH_LOSS_FRAMES=6,
 parameter [11:0] SIZE_STEP=12'd4
)(input wire clk,rst_n,enable,vsync,de,input wire [23:0] rgb,
 output reg valid,output reg [11:0] left,right,top,bottom);
 wire raw_valid;
 wire [11:0] raw_left,raw_right,raw_top,raw_bottom;
 motion_object_detector_raw #(.BG_THRESHOLD(BG_THRESHOLD),.MOVE_THRESHOLD(MOVE_THRESHOLD),
  .MIN_AREA(MIN_AREA),.MAX_AREA(MAX_AREA),.MIN_SEEDS(MIN_SEEDS),
  .GLOBAL_CHANGE_LIMIT(GLOBAL_CHANGE_LIMIT),.HOLD_FRAMES(HOLD_FRAMES),.SLOTS(SLOTS)) detector(
  .clk(clk),.rst_n(rst_n),.enable(enable),.vsync(vsync),.de(de),.rgb(rgb),
  .valid(raw_valid),.left(raw_left),.right(raw_right),.top(raw_top),.bottom(raw_bottom));
 reg vd;
 reg [3:0] sample_wait;
 reg [7:0] loss_count;
 reg [11:0] cx,cy,half_w,half_h;
 wire [11:0] measured_cx=({1'b0,raw_left}+{1'b0,raw_right})>>1;
 wire [11:0] measured_cy=({1'b0,raw_top}+{1'b0,raw_bottom})>>1;
 wire [11:0] measured_hw=(raw_right-raw_left)>>1;
 wire [11:0] measured_hh=(raw_bottom-raw_top)>>1;
 function [11:0] follow_center;
  input [11:0] old_value,new_value;
  reg [11:0] diff,step;
  begin
   diff=(new_value>=old_value) ? new_value-old_value : old_value-new_value;
   // Slow near the estimate, faster when the target moves substantially.
   step=(diff>12'd64) ? (diff>>1) : (diff>>2);
   if(diff!=0 && step==0) step=1;
   follow_center=(new_value>=old_value) ? old_value+step : old_value-step;
  end
 endfunction
 function [11:0] follow_size;
  input [11:0] old_value,new_value;
  reg [11:0] diff;
  begin
   diff=(new_value>=old_value) ? new_value-old_value : old_value-new_value;
   if(diff<=SIZE_STEP) follow_size=new_value;
   else if(new_value>old_value) follow_size=old_value+SIZE_STEP;
   else follow_size=old_value-SIZE_STEP;
  end
 endfunction
 wire [11:0] next_cx=follow_center(cx,measured_cx);
 wire [11:0] next_cy=follow_center(cy,measured_cy);
 wire [11:0] next_hw=follow_size(half_w,measured_hw);
 wire [11:0] next_hh=follow_size(half_h,measured_hh);
 wire [12:0] next_right={1'b0,next_cx}+{1'b0,next_hw};
 wire [12:0] next_bottom={1'b0,next_cy}+{1'b0,next_hh};
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   vd<=0;sample_wait<=0;loss_count<=0;valid<=0;
   cx<=0;cy<=0;half_w<=0;half_h<=0;left<=0;right<=0;top<=0;bottom<=0;
  end else begin
   vd<=vsync;
   if(!enable) begin valid<=0;sample_wait<=0;loss_count<=0;end
   else if(vsync && !vd) sample_wait<=4'd8;
   else if(sample_wait!=0) begin
    sample_wait<=sample_wait-1'b1;
    if(sample_wait==1) begin
     if(raw_valid) begin
      loss_count<=0;
      if(!valid) begin
       valid<=1;cx<=measured_cx;cy<=measured_cy;half_w<=measured_hw;half_h<=measured_hh;
       left<=raw_left;right<=raw_right;top<=raw_top;bottom<=raw_bottom;
      end else begin
       cx<=next_cx;cy<=next_cy;half_w<=next_hw;half_h<=next_hh;
       left<=next_cx>=next_hw ? next_cx-next_hw : 12'd0;
       right<=next_right>13'd1279 ? 12'd1279 : next_right[11:0];
       top<=next_cy>=next_hh ? next_cy-next_hh : 12'd0;
       bottom<=next_bottom>13'd719 ? 12'd719 : next_bottom[11:0];
      end
     end else if(valid) begin
      if(SMOOTH_LOSS_FRAMES<=1 || loss_count>=SMOOTH_LOSS_FRAMES-1) begin
       valid<=0;loss_count<=0;
      end else loss_count<=loss_count+1'b1;
     end
    end
   end
  end
 end
endmodule

// Fixed camera: one largest background-difference component with fresh motion.
// This is motion detection,not semantic person detection. Grid=160x90.
module motion_object_detector_raw #(
 parameter [5:0] BG_THRESHOLD=6,MOVE_THRESHOLD=5,
 parameter integer MIN_AREA=12,MAX_AREA=7200,MIN_SEEDS=2,
 parameter integer GLOBAL_CHANGE_LIMIT=7200,HOLD_FRAMES=3,
 parameter integer SLOTS=2
)(input wire clk,rst_n,enable,vsync,de,input wire [23:0] rgb,
 output reg valid,output reg [11:0] left,right,top,bottom);
 (* ram_style="bram" *) reg [11:0] history_ram[0:14399];
 reg [13:0] read_addr,write_addr,row_base;
 reg [11:0] read_data,write_data;
 reg write_en;
 always @(posedge clk) begin
  if(write_en) history_ram[write_addr]<=write_data;
  read_data<=history_ram[read_addr];
 end
 reg vd,dd;
 reg [11:0] x,y;
 reg [1:0] warmup;
 reg [2:0] frame_phase;
 reg [13:0] changed_count;
 reg cell_strobe,row_end,cell_fg,cell_move;
 reg [7:0] cell_x;
 reg [6:0] cell_y;
 wire rise=vsync && !vd;
 wire [9:0] luma={2'd0,rgb[23:16]}+{1'b0,rgb[15:8],1'b0}+{2'd0,rgb[7:0]};
 wire [5:0] gray=luma[9:4];
 wire [5:0] previous=read_data[5:0],background=read_data[11:6];
 wire [5:0] move_diff=gray>=previous ? gray-previous : previous-gray;
 wire [5:0] bg_diff=gray>=background ? gray-background : background-gray;
 wire moving=warmup!=0 && move_diff>=MOVE_THRESHOLD;
 wire foreground=warmup!=0 && bg_diff>=BG_THRESHOLD;
 wire [5:0] adapted_bg=gray>background ? background+6'd1 :
                            (gray<background ? background-6'd1 : background);
 wire [5:0] new_bg=warmup==0 ? gray :
                    (frame_phase==0 && !moving ? adapted_bg : background);
 wire sampled_row=y[2:0]==4 && y<720;
 wire sample_pixel=de && sampled_row && x<1280 && x[2:0]==4;
 wire [13:0] sample_addr=row_base+{5'd0,x[11:3]};
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   vd<=0;dd<=0;x<=0;y<=0;warmup<=0;frame_phase<=0;changed_count<=0;
   read_addr<=0;write_addr<=0;row_base<=0;write_data<=0;write_en<=0;
   cell_strobe<=0;row_end<=0;cell_fg<=0;cell_move<=0;cell_x<=0;cell_y<=0;
  end else begin
   vd<=vsync;dd<=de;write_en<=0;cell_strobe<=0;row_end<=0;
   if(!enable) begin
    x<=0;y<=0;row_base<=0;dd<=0;warmup<=0;frame_phase<=0;changed_count<=0;
   end else if(rise) begin
    x<=0;y<=0;row_base<=0;changed_count<=0;frame_phase<=frame_phase+1'b1;
    if(y<720 || changed_count>=GLOBAL_CHANGE_LIMIT) warmup<=0;
    else if(warmup<2) warmup<=warmup+1'b1;
   end else begin
    if(de) x<=x+1'b1;
    else if(dd) begin
     x<=0;y<=y+1'b1;
     if(y[2:0]==7) row_base<=row_base+14'd160;
     if(sampled_row) row_end<=1;
    end
    // Read a cell two clocks before its sample; never depend on a RAM's
    // same-address read-during-write behaviour. Write commits the next clock.
    if(de && sampled_row && x<1280 && x[2:0]==2) read_addr<=sample_addr;
    if(sample_pixel) begin
     write_en<=1;write_addr<=sample_addr;write_data<={new_bg,gray};
     cell_strobe<=1;cell_x<=x[10:3];cell_y<=y[9:3];
     // A fresh frame-to-frame change must not be discarded merely because
     // the slower background model differs by one threshold step less.
     cell_fg<=(foreground || moving) && x>=16 && x<1264 && y>=16 && y<704;
     cell_move<=moving;
     if(moving) changed_count<=changed_count+1'b1;
    end
   end
  end
 end

 localparam IDLE=0,SEARCH=1,COMMIT=2,FINAL=3,PUBLISH=4;
 reg [2:0] state;
 localparam SLOT_BITS=SLOTS<=1 ? 1 : $clog2(SLOTS);
 reg [SLOT_BITS-1:0] index;
 reg used[0:SLOTS-1];
 reg [7:0] min_x[0:SLOTS-1],max_x[0:SLOTS-1],last_left[0:SLOTS-1],last_right[0:SLOTS-1];
 reg [6:0] min_y[0:SLOTS-1],max_y[0:SLOTS-1],last_y[0:SLOTS-1];
 reg [13:0] area[0:SLOTS-1];
 // Only the >=2 test is needed. Saturation avoids wide per-component adders.
 reg [1:0] seeds[0:SLOTS-1];
 reg run_active;
 reg gap;
 reg [7:0] run_start,run_end;
 reg [6:0] run_y;
 reg [1:0] run_seeds,query_seeds;
 reg [7:0] query_start,query_end,query_length;
 reg [6:0] query_y;
 reg match_found,free_found;
 reg [SLOT_BITS-1:0] match_slot,free_slot;
 reg [7:0] best_distance;
 reg [13:0] best_area;
 reg [7:0] best_left,best_right;
 reg [6:0] best_top,best_bottom;
 reg publish_allowed;
 reg [3:0] hold_left;
// Collect every coherent moving run in the frame.  Component tracking still
// confirms that motion is real; this envelope prevents detached arms, legs or
// other fragments from being clipped by the largest-component box.
reg envelope_active;
reg [7:0] envelope_min_x,envelope_max_x;
reg [6:0] envelope_min_y,envelope_max_y;
// Only threshold results are consumed, so saturating counters avoid carrying
// full-frame area and seed adders through this resource-tight design.
reg [3:0] envelope_area;
reg [1:0] envelope_seeds;
reg frame_envelope_active;
reg [7:0] frame_envelope_min_x,frame_envelope_max_x;
reg [6:0] frame_envelope_min_y,frame_envelope_max_y;
reg [3:0] frame_envelope_area;
reg [1:0] frame_envelope_seeds;
 integer j;
 wire [7:0] run_length=run_end-run_start+8'd1;
 wire close_run=run_active && (row_end || (cell_strobe && !cell_fg && gap));
 wire [8:0] query_previous={2'd0,query_y}-9'd1;
 wire [8:0] query_previous2={2'd0,query_y}-9'd2;
 wire [7:0] query_mid=({1'b0,query_start}+{1'b0,query_end})>>1;
 wire [7:0] old_mid=({1'b0,last_left[index]}+{1'b0,last_right[index]})>>1;
 wire [7:0] distance=query_mid>=old_mid ? query_mid-old_mid : old_mid-query_mid;
 wire stale=used[index] && {2'd0,last_y[index]}+9'd2<{2'd0,query_y};
 wire joins=used[index] &&
  ({2'd0,last_y[index]}==query_previous || {2'd0,last_y[index]}==query_previous2) &&
  {1'b0,query_start}<={1'b0,last_right[index]}+9'd2 &&
  {1'b0,query_end}+9'd2>={1'b0,last_left[index]};
 wire [8:0] component_width={1'b0,max_x[index]}-{1'b0,min_x[index]}+9'd1;
 wire [7:0] component_height={1'b0,max_y[index]}-{1'b0,min_y[index]}+8'd1;
 wire eligible=used[index] && area[index]>=MIN_AREA && area[index]<=MAX_AREA &&
  seeds[index]>=MIN_SEEDS && component_width>=3 && component_height>=3 &&
  !(component_width>=144 && component_height<=24);
 wire [SLOT_BITS-1:0] chosen=match_found ? match_slot : free_slot;
 wire [2:0] seed_sum={1'b0,seeds[chosen]}+{1'b0,query_seeds};
wire [8:0] envelope_width={1'b0,frame_envelope_max_x}-{1'b0,frame_envelope_min_x}+9'd1;
wire [7:0] envelope_height={1'b0,frame_envelope_max_y}-{1'b0,frame_envelope_min_y}+8'd1;
wire envelope_eligible=frame_envelope_active && frame_envelope_area>=MIN_AREA &&
 frame_envelope_seeds>=MIN_SEEDS && envelope_width>=3 && envelope_height>=3 &&
 !(envelope_width>=144 && envelope_height<=24);
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   state<=IDLE;index<=0;run_active<=0;gap<=0;run_start<=0;run_end<=0;run_y<=0;run_seeds<=0;
   query_start<=0;query_end<=0;query_y<=0;query_length<=0;query_seeds<=0;
   match_found<=0;free_found<=0;match_slot<=0;free_slot<=0;best_distance<=255;
   best_area<=0;best_left<=0;best_right<=0;best_top<=0;best_bottom<=0;
   publish_allowed<=0;hold_left<=0;valid<=0;left<=0;right<=0;top<=0;bottom<=0;
   envelope_active<=0;envelope_min_x<=0;envelope_max_x<=0;envelope_min_y<=0;envelope_max_y<=0;
   envelope_area<=0;envelope_seeds<=0;frame_envelope_active<=0;
   frame_envelope_min_x<=0;frame_envelope_max_x<=0;frame_envelope_min_y<=0;frame_envelope_max_y<=0;
   frame_envelope_area<=0;frame_envelope_seeds<=0;
   for(j=0;j<SLOTS;j=j+1) begin
    used[j]<=0;min_x[j]<=0;max_x[j]<=0;min_y[j]<=0;max_y[j]<=0;
    last_left[j]<=0;last_right[j]<=0;last_y[j]<=0;area[j]<=0;seeds[j]<=0;
   end
  end else if(!enable) begin
   state<=IDLE;run_active<=0;gap<=0;run_seeds<=0;best_area<=0;valid<=0;hold_left<=0;
   envelope_active<=0;envelope_area<=0;envelope_seeds<=0;frame_envelope_active<=0;
   for(j=0;j<SLOTS;j=j+1) used[j]<=0;
  end else if(rise) begin
   state<=FINAL;index<=0;run_active<=0;gap<=0;run_seeds<=0;
   publish_allowed<=y>=720 && warmup>=2 && changed_count<GLOBAL_CHANGE_LIMIT;
   frame_envelope_active<=envelope_active;
   frame_envelope_min_x<=envelope_min_x;frame_envelope_max_x<=envelope_max_x;
   frame_envelope_min_y<=envelope_min_y;frame_envelope_max_y<=envelope_max_y;
   frame_envelope_area<=envelope_area;frame_envelope_seeds<=envelope_seeds;
   envelope_active<=0;envelope_area<=0;envelope_seeds<=0;
  end else begin
   if(cell_strobe) begin
    if(cell_fg) begin
     if(!run_active) begin run_start<=cell_x;run_y<=cell_y;run_seeds<=cell_move ? 1 : 0;end
     else if(cell_move && run_seeds<3) run_seeds<=run_seeds+1'b1;
     run_end<=cell_x;run_active<=1;gap<=0;
    end else if(run_active) begin
     if(gap) begin run_active<=0;gap<=0;end else gap<=1;
    end
   end
   if(row_end) begin run_active<=0;gap<=0;end
   if(close_run && run_length>=2 && run_seeds!=0) begin
    if(!envelope_active) begin
     envelope_active<=1;envelope_min_x<=run_start;envelope_max_x<=run_end;
     envelope_min_y<=run_y;envelope_max_y<=run_y;
     envelope_area<=run_length>=15 ? 4'd15 : run_length[3:0];
     envelope_seeds<=run_seeds;
    end else begin
     if(run_start<envelope_min_x) envelope_min_x<=run_start;
     if(run_end>envelope_max_x) envelope_max_x<=run_end;
     if(run_y<envelope_min_y) envelope_min_y<=run_y;
     if(run_y>envelope_max_y) envelope_max_y<=run_y;
     if(run_length>=15 || ({4'd0,envelope_area}+run_length)>=15)
      envelope_area<=4'd15;
     else envelope_area<=envelope_area+run_length[3:0];
     if(({1'b0,envelope_seeds}+{1'b0,run_seeds})>=3) envelope_seeds<=2'd3;
     else envelope_seeds<=envelope_seeds+run_seeds;
    end
   end
   if(close_run && run_length>=2 && state==IDLE) begin
    query_start<=run_start;query_end<=run_end;query_y<=run_y;query_length<=run_length;query_seeds<=run_seeds;
    match_found<=0;free_found<=0;best_distance<=255;index<=0;state<=SEARCH;
   end
   case(state)
    SEARCH:begin
     if(stale) begin
      if(eligible && area[index]>best_area) begin
       best_area<=area[index];best_left<=min_x[index];best_right<=max_x[index];
       best_top<=min_y[index];best_bottom<=max_y[index];
      end
      used[index]<=0;
     end
     if((!used[index] || stale) && !free_found) begin free_found<=1;free_slot<=index;end
     if(joins && (!match_found || distance<best_distance)) begin match_found<=1;match_slot<=index;best_distance<=distance;end
     if(index==SLOTS-1) state<=COMMIT;else index<=index+1'b1;
    end
    COMMIT:begin
     if(match_found || free_found) begin
      for(j=0;j<SLOTS;j=j+1) if(chosen==j) begin
       used[j]<=1;last_left[j]<=query_start;last_right[j]<=query_end;last_y[j]<=query_y;
       if(match_found) begin
        if(query_start<min_x[j]) min_x[j]<=query_start;
        if(query_end>max_x[j]) max_x[j]<=query_end;
        max_y[j]<=query_y;area[j]<=area[j]+query_length;seeds[j]<=seed_sum>3 ? 2'd3 : seed_sum[1:0];
       end else begin
        min_x[j]<=query_start;max_x[j]<=query_end;min_y[j]<=query_y;max_y[j]<=query_y;
        area[j]<=query_length;seeds[j]<=query_seeds;
       end
      end
     end
     state<=IDLE;
    end
    FINAL:begin
     if(eligible && area[index]>best_area) begin
      best_area<=area[index];best_left<=min_x[index];best_right<=max_x[index];
      best_top<=min_y[index];best_bottom<=max_y[index];
     end
     used[index]<=0;
     if(index==SLOTS-1) state<=PUBLISH;else index<=index+1'b1;
    end
    PUBLISH:begin
     if(!publish_allowed) begin valid<=0;hold_left<=0;end
     else if(best_area!=0) begin
      valid<=1;hold_left<=HOLD_FRAMES;
      if(envelope_eligible) begin
       left<=frame_envelope_min_x>2 ? ({4'd0,frame_envelope_min_x}-12'd2)<<3 : 12'd0;
       right<=frame_envelope_max_x<157 ? (({4'd0,frame_envelope_max_x}+12'd3)<<3)-12'd1 : 12'd1279;
       top<=frame_envelope_min_y>2 ? ({5'd0,frame_envelope_min_y}-12'd2)<<3 : 12'd0;
       bottom<=frame_envelope_max_y<87 ? (({5'd0,frame_envelope_max_y}+12'd3)<<3)-12'd1 : 12'd719;
      end else begin
       left<=best_left>2 ? ({4'd0,best_left}-12'd2)<<3 : 12'd0;
       right<=best_right<157 ? (({4'd0,best_right}+12'd3)<<3)-12'd1 : 12'd1279;
       top<=best_top>2 ? ({5'd0,best_top}-12'd2)<<3 : 12'd0;
       bottom<=best_bottom<87 ? (({5'd0,best_bottom}+12'd3)<<3)-12'd1 : 12'd719;
      end
     end else if(hold_left!=0) hold_left<=hold_left-1'b1;
     else valid<=0;
     best_area<=0;state<=IDLE;
    end
    default:begin end
   endcase
  end
 end
endmodule

// Low-cost electronic fence monitor.  It consumes the already-computed motion
// box; no extra image buffer or pixel arithmetic is required.
module motion_fence_monitor #(
 parameter [11:0] FENCE_LEFT=12'd400,FENCE_RIGHT=12'd880,
 parameter [11:0] FENCE_TOP=12'd160,FENCE_BOTTOM=12'd560,
 parameter [11:0] LINE_A=12'd520,LINE_B=12'd760,
 parameter integer FRAME_HZ=60,LEAVE_HOLD_FRAMES=30,
 parameter integer ROUTE_TIMEOUT_FRAMES=120,EVENT_HOLD_FRAMES=120,
 parameter integer STAY_FRAMES=120,LOITER_SECONDS=10,
 parameter [11:0] DIRECTION_THRESHOLD=12'd4
)(input wire clk,rst_n,enable,vsync,motion_valid,
  input wire [11:0] motion_left,motion_right,motion_top,motion_bottom,
  output reg [1:0] state,output reg [2:0] direction,
  output reg inside_fence,output reg [11:0] entry_bcd,output reg [11:0] dwell_bcd,
  output reg [2:0] passage_event,
  output reg [11:0] in_bcd,output reg [11:0] out_bcd);
 localparam [1:0] SAFE=2'd0,ENTER=2'd1,LEAVE=2'd2,INTRUSION=2'd3;
 localparam [2:0] DIR_NONE=3'd0,DIR_LEFT=3'd1,DIR_RIGHT=3'd2,DIR_UP=3'd3,DIR_DOWN=3'd4;
 localparam [2:0] EVENT_NONE=3'd0,EVENT_PASS=3'd1,EVENT_WRONG=3'd2,
                  EVENT_LOITER=3'd3,EVENT_STAY=3'd4;
 localparam [1:0] ROUTE_NONE=2'd0,ROUTE_RIGHT=2'd1,ROUTE_LEFT=2'd2;
 reg vd,prev_valid,prev_inside;
 reg [4:0] sample_wait;
 reg [11:0] prev_cx,prev_cy;
 reg [5:0] dwell_div;
 reg [5:0] leave_hold;
 reg [1:0] route_phase;
 reg [7:0] route_timeout,event_hold,stationary_frames;
 wire rise=vsync && !vd;
 wire [11:0] center_x=({1'b0,motion_left}+{1'b0,motion_right})>>1;
 wire [11:0] center_y=({1'b0,motion_top}+{1'b0,motion_bottom})>>1;
 wire center_inside=center_x>=FENCE_LEFT && center_x<=FENCE_RIGHT &&
                    center_y>=FENCE_TOP && center_y<=FENCE_BOTTOM;
 wire [11:0] abs_dx=center_x>=prev_cx ? center_x-prev_cx : prev_cx-center_x;
 wire [11:0] abs_dy=center_y>=prev_cy ? center_y-prev_cy : prev_cy-center_y;

 function [11:0] F_bcd_inc;
  input [11:0] value;
  begin
   if(value==12'h999) F_bcd_inc=value;
   else if(value[3:0]<9) F_bcd_inc=value+12'h001;
   else if(value[7:4]<9) F_bcd_inc={value[11:4]+8'h01,4'h0};
   else F_bcd_inc={value[11:8]+4'h1,8'h00};
  end
 endfunction

 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   vd<=0;sample_wait<=0;state<=SAFE;direction<=DIR_NONE;inside_fence<=0;
   prev_valid<=0;prev_inside<=0;prev_cx<=0;prev_cy<=0;
   entry_bcd<=0;dwell_bcd<=0;dwell_div<=0;leave_hold<=0;
   passage_event<=EVENT_NONE;in_bcd<=0;out_bcd<=0;
   route_phase<=ROUTE_NONE;route_timeout<=0;event_hold<=0;stationary_frames<=0;
  end else begin
   vd<=vsync;
   if(!enable) begin
    sample_wait<=0;state<=SAFE;direction<=DIR_NONE;inside_fence<=0;
    prev_valid<=0;prev_inside<=0;entry_bcd<=0;dwell_bcd<=0;
    dwell_div<=0;leave_hold<=0;
    passage_event<=EVENT_NONE;in_bcd<=0;out_bcd<=0;
    route_phase<=ROUTE_NONE;route_timeout<=0;event_hold<=0;stationary_frames<=0;
   end else begin
    if(rise) sample_wait<=5'd16;
    else if(sample_wait!=0) begin
     sample_wait<=sample_wait-1'b1;
     if(sample_wait==1) begin
      if(!motion_valid) begin
       inside_fence<=0;prev_valid<=0;prev_inside<=0;dwell_bcd<=0;dwell_div<=0;
       stationary_frames<=0;
       if(route_timeout!=0) route_timeout<=route_timeout-1'b1;
       else route_phase<=ROUTE_NONE;
       if(event_hold!=0) event_hold<=event_hold-1'b1;
       else passage_event<=EVENT_NONE;
       if(prev_inside) begin state<=LEAVE;leave_hold<=LEAVE_HOLD_FRAMES;end
       else if(leave_hold!=0) begin state<=LEAVE;leave_hold<=leave_hold-1'b1;end
       else begin state<=SAFE;direction<=DIR_NONE;end
      end else begin
       inside_fence<=center_inside;
       if(route_phase!=ROUTE_NONE) begin
        if(route_timeout!=0) route_timeout<=route_timeout-1'b1;
        else route_phase<=ROUTE_NONE;
       end
       if(event_hold!=0) event_hold<=event_hold-1'b1;
       else passage_event<=EVENT_NONE;
       if(prev_valid && (abs_dx>=DIRECTION_THRESHOLD || abs_dy>=DIRECTION_THRESHOLD)) begin
        if(abs_dx>=abs_dy) direction<=center_x>=prev_cx ? DIR_RIGHT : DIR_LEFT;
        else direction<=center_y>=prev_cy ? DIR_DOWN : DIR_UP;
       end
       if(prev_valid && abs_dx<DIRECTION_THRESHOLD && abs_dy<DIRECTION_THRESHOLD) begin
        if(stationary_frames<STAY_FRAMES) stationary_frames<=stationary_frames+1'b1;
       end else stationary_frames<=0;

       // Two-line passage sequence.  A complete A->B traversal is the normal
       // direction; B->A is still counted as OUT and raises WRONG.
       if(prev_valid && prev_cx<LINE_A && center_x>=LINE_B) begin
        in_bcd<=F_bcd_inc(in_bcd);
        passage_event<=EVENT_PASS;event_hold<=EVENT_HOLD_FRAMES;route_phase<=ROUTE_NONE;
       end else if(prev_valid && prev_cx>LINE_B && center_x<=LINE_A) begin
        out_bcd<=F_bcd_inc(out_bcd);
        passage_event<=EVENT_WRONG;event_hold<=EVENT_HOLD_FRAMES;route_phase<=ROUTE_NONE;
       end else if(prev_valid && prev_cx<LINE_A && center_x>=LINE_A) begin
        route_phase<=ROUTE_RIGHT;route_timeout<=ROUTE_TIMEOUT_FRAMES;
       end else if(prev_valid && prev_cx>LINE_B && center_x<=LINE_B) begin
        route_phase<=ROUTE_LEFT;route_timeout<=ROUTE_TIMEOUT_FRAMES;
       end else if(route_phase==ROUTE_RIGHT && center_x>=LINE_B) begin
        in_bcd<=F_bcd_inc(in_bcd);
        passage_event<=EVENT_PASS;event_hold<=EVENT_HOLD_FRAMES;route_phase<=ROUTE_NONE;
       end else if(route_phase==ROUTE_LEFT && center_x<=LINE_A) begin
        out_bcd<=F_bcd_inc(out_bcd);
        passage_event<=EVENT_WRONG;event_hold<=EVENT_HOLD_FRAMES;route_phase<=ROUTE_NONE;
       end else if(route_phase==ROUTE_RIGHT && center_x<LINE_A) route_phase<=ROUTE_NONE;
       else if(route_phase==ROUTE_LEFT && center_x>LINE_B) route_phase<=ROUTE_NONE;

       if(center_inside) begin
        leave_hold<=0;
        if(!prev_inside) begin
         // Intrusion counter removed; preserve fence state and dwell timer.
         state<=ENTER;dwell_bcd<=0;dwell_div<=0;
        end else if(dwell_div==FRAME_HZ-1) begin
         dwell_div<=0;dwell_bcd<=F_bcd_inc(dwell_bcd);state<=INTRUSION;
        end else begin
         dwell_div<=dwell_div+1'b1;
         state<=dwell_bcd!=0 ? INTRUSION : ENTER;
        end
        if(dwell_bcd>=LOITER_SECONDS) begin
         passage_event<=EVENT_LOITER;event_hold<=0;
        end else if(event_hold==0 && stationary_frames>=STAY_FRAMES-1)
         passage_event<=EVENT_STAY;
       end else begin
        dwell_bcd<=0;dwell_div<=0;
        if(prev_inside) begin state<=LEAVE;leave_hold<=LEAVE_HOLD_FRAMES;end
        else if(leave_hold!=0) begin state<=LEAVE;leave_hold<=leave_hold-1'b1;end
        else state<=SAFE;
       end
       prev_valid<=1;prev_inside<=center_inside;prev_cx<=center_x;prev_cy<=center_y;
      end
     end
    end
   end
  end
 end
endmodule

// Frame-based acquisition latency monitor.  The counter starts when the
// target box first becomes valid and freezes when the target centre enters
// the same near-centre band used by the servo controller.  It is display-only
// and has no feedback path to detection or motion control.
module target_latency_monitor #(
 parameter [11:0] CENTER_X=12'd640,CENTER_Y=12'd360,
 parameter [11:0] TOL_X=12'd24,TOL_Y=12'd24,
 parameter [11:0] REARM_X=12'd40,REARM_Y=12'd40,
 parameter integer CENTER_CONFIRM_FRAMES=3,
 parameter integer LOSS_GRACE_FRAMES=30,
 parameter integer REARM_LOSS_FRAMES=5
)(input wire clk,rst_n,enable,target_valid,
  input wire [11:0] target_cx,target_cy,
  input wire [11:0] cnt_bcd,
  output reg timing,output reg done,output reg [11:0] latency_bcd,
  output reg [11:0] running_bcd);
 reg [11:0] start_bcd,cnt_d,candidate_end_bcd;
 reg [2:0] center_count;
 reg [5:0] loss_count,rearm_loss_count;
 reg rearm_ready,require_outside;
 wire centered=target_cx>=CENTER_X-TOL_X && target_cx<=CENTER_X+TOL_X &&
               target_cy>=CENTER_Y-TOL_Y && target_cy<=CENTER_Y+TOL_Y;
 wire outside_rearm=target_cx<CENTER_X-REARM_X || target_cx>CENTER_X+REARM_X ||
                    target_cy<CENTER_Y-REARM_Y || target_cy>CENTER_Y+REARM_Y;
 wire start_request=rearm_ready && target_valid && (!require_outside || outside_rearm);

 function [11:0] F_bcd_delta;
  input [11:0] now_bcd;
  input [11:0] old_bcd;
  reg [4:0] work;
  reg borrow_u,borrow_t;
  reg [3:0] units,tens,hundreds;
  begin
   if(now_bcd[3:0]>=old_bcd[3:0]) begin
    units=now_bcd[3:0]-old_bcd[3:0];borrow_u=0;
   end else begin
    units=now_bcd[3:0]+4'd10-old_bcd[3:0];borrow_u=1;
   end
   work={1'b0,old_bcd[7:4]}+borrow_u;
   if({1'b0,now_bcd[7:4]}>=work) begin
    tens=now_bcd[7:4]-work[3:0];borrow_t=0;
   end else begin
    tens=now_bcd[7:4]+4'd10-work[3:0];borrow_t=1;
   end
   work={1'b0,old_bcd[11:8]}+borrow_t;
   if({1'b0,now_bcd[11:8]}>=work)
    hundreds=now_bcd[11:8]-work[3:0];
   else
    hundreds=now_bcd[11:8]+4'd10-work[3:0];
   F_bcd_delta={hundreds,tens,units};
  end
 endfunction

 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   start_bcd<=0;cnt_d<=0;candidate_end_bcd<=0;
   center_count<=0;loss_count<=0;rearm_loss_count<=0;
   rearm_ready<=1;require_outside<=0;
   timing<=0;done<=0;latency_bcd<=0;running_bcd<=0;
  end else if(!enable) begin
   start_bcd<=0;cnt_d<=cnt_bcd;candidate_end_bcd<=0;
   center_count<=0;loss_count<=0;rearm_loss_count<=0;
   rearm_ready<=1;require_outside<=0;
   timing<=0;done<=0;latency_bcd<=0;running_bcd<=0;
  end else begin
   cnt_d<=cnt_bcd;
   // Once timing has started it has priority over target_rise.  A detector
   // dropout followed by reacquisition must not overwrite the original CNT
   // snapshot and restart the result at 001F.
   if(timing) begin
    if(cnt_bcd!=cnt_d) begin
     running_bcd<=F_bcd_delta(cnt_bcd,start_bcd);
     if(!target_valid) begin
      center_count<=0;
      if(loss_count==LOSS_GRACE_FRAMES-1) begin
       loss_count<=0;
       timing<=0;
       running_bcd<=0;
       rearm_ready<=1;
       require_outside<=1;
      end else begin
       loss_count<=loss_count+1'b1;
      end
     end else begin
      loss_count<=0;
      if(centered) begin
       if(center_count==0) candidate_end_bcd<=cnt_bcd;
       if(center_count==CENTER_CONFIRM_FRAMES-1) begin
        latency_bcd<=F_bcd_delta(candidate_end_bcd,start_bcd);
        running_bcd<=0;
        center_count<=center_count;
        timing<=0;
        done<=1;
       end else begin
        center_count<=center_count+1'b1;
       end
      end else if(center_count!=0) begin
       // Leaky confirmation tolerates one small centre oscillation without
       // accepting a single spurious centre sample.
       center_count<=center_count-1'b1;
      end
     end
    end
   end else if(start_request) begin
    start_bcd<=cnt_bcd;
    candidate_end_bcd<=cnt_bcd;
    center_count<=centered ? 3'd1 : 3'd0;
    loss_count<=0;
    rearm_loss_count<=0;
    rearm_ready<=0;
    require_outside<=0;
    // The old frozen result is cleared only when a genuine new measurement
    // starts, never when the preceding measurement merely completes.
    latency_bcd<=0;
    running_bcd<=0;
    timing<=1;
    done<=0;
   end else if(!rearm_ready && cnt_bcd!=cnt_d) begin
    // After a completed measurement, ignore short target-valid dropouts.
    // Five consecutive video-frame samples of real loss are required before
    // reacquisition can arm another run.  A centred reacquisition remains
    // idle until it moves outside the wider rearm band.
    if(!target_valid) begin
     if(rearm_loss_count>=REARM_LOSS_FRAMES-1) begin
      rearm_loss_count<=0;
      rearm_ready<=1;
      require_outside<=1;
     end else begin
      rearm_loss_count<=rearm_loss_count+1'b1;
     end
    end else begin
     rearm_loss_count<=0;
    end
   end
  end
 end
endmodule

module motion_mode_switch #(parameter integer STABLE_CYCLES=300000)(
 input wire clk,rst_n,raw_switch,output reg requested);
 (* ASYNC_REG="TRUE",syn_preserve=1,syn_keep=1 *) reg switch_meta;
 (* ASYNC_REG="TRUE",syn_preserve=1,syn_keep=1 *) reg switch_sync;
 reg [18:0] stable_count;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin switch_meta<=0;switch_sync<=0;stable_count<=0;requested<=0;end
  else begin
   switch_meta<=raw_switch;switch_sync<=switch_meta;
   if(switch_sync==requested) stable_count<=0;
   else if(stable_count==STABLE_CYCLES-1) begin requested<=switch_sync;stable_count<=0;end
   else stable_count<=stable_count+1'b1;
  end
 end
endmodule

// Active-low KEY2. Debounce BOTH transitions, arm only after a stable release.
// A held key during reset cannot enable recognition; no autorepeat on hold.
module detection_key_toggle #(parameter integer STABLE_CYCLES=750000)(
 input wire clk,rst_n,key_n,output reg enabled);
 localparam COUNT_BITS=STABLE_CYCLES<=1 ? 1 : $clog2(STABLE_CYCLES);
 (* ASYNC_REG="TRUE",syn_preserve=1,syn_keep=1 *) reg key_meta,key_sync;
 reg [COUNT_BITS-1:0] stable_count;
 reg stable_key,armed;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin key_meta<=0;key_sync<=0;stable_count<=0;stable_key<=0;armed<=0;enabled<=0;end
  else begin
   key_meta<=key_n;key_sync<=key_meta;
   if(key_sync==stable_key)stable_count<=0;
   else if(stable_count==STABLE_CYCLES-1)begin
    stable_key<=key_sync;stable_count<=0;
    if(key_sync)armed<=1;
    else if(armed)begin enabled<=~enabled;armed<=0;end
   end else stable_count<=stable_count+1'b1;
  end
 end
endmodule
`default_nettype wire
