// V6: recycle completed run tracks; verify the concentric artwork before valid.
// Servo controller is unchanged. Results complete in vertical blanking.
module simple_black_disk_detector #(
 // Twelve reusable run tracks are sufficient for the concentric-target
 // workload and avoid four complete parallel match banks.  The fixed 16-leaf
 // selector below pads the unused leaves with zero, preserving its depth.
 parameter integer TRACKS=12,
 parameter [7:0] DARK_THRESHOLD=8'd80,
 parameter [19:0] MIN_AREA=20'd80,MAX_AREA=20'd180000,
 parameter [11:0] MIN_DIAMETER=12'd24
)(input wire I_clk,I_rst_n,I_vsync,I_de,input wire [23:0] I_rgb,
 output reg O_valid,output reg [11:0] O_center_x,O_center_y,O_radius,
 output reg [7:0] O_confidence);
 localparam [2:0] STREAM=0,DRAIN=1,FINISH=2,CHECK=3,PREPARE=4,DIVIDE=5;
 reg [2:0] state;
 reg [2:0] drain_count;
 reg vd,dd,run_active,frame_complete;
 reg [11:0] x,y,run_start,run_end;
 reg [1:0] gap;
 reg used[0:TRACKS-1];
 reg [11:0] min_x[0:TRACKS-1],max_x[0:TRACKS-1],min_y[0:TRACKS-1],max_y[0:TRACKS-1];
 reg [11:0] last_left[0:TRACKS-1],last_right[0:TRACKS-1],last_y[0:TRACKS-1];
 reg [19:0] area[0:TRACKS-1];
 reg [31:0] moment_x[0:TRACKS-1],moment_y[0:TRACKS-1];
 reg select_pending,update_pending;
 reg [16:0] select_l2[0:3],select_free;
 reg [11:0] select_start,select_end,select_y,update_start,update_end,update_y;
 reg [12:0] select_length,update_length;
 reg [31:0] select_mx,select_my,update_mx,update_my;
 reg [3:0] update_slot;
 // One run commits per clock. Share arithmetic across the indexed track bank.
 // Keep the original register write enables, widths and cycle timing.
 wire [19:0] update_area_sum = area[update_slot] + update_length;
 wire [31:0] update_moment_x_sum = moment_x[update_slot] + update_mx;
 wire [31:0] update_moment_y_sum = moment_y[update_slot] + update_my;
 wire update_min_x = update_start < min_x[update_slot];
 wire update_max_x = update_end > max_x[update_slot];
 reg [3:0] scan_index;
 reg [1:0] scan_phase;
 reg scan_eligible,scan_inside;
 reg [12:0] scan_width,scan_height;
 reg [11:0] scan_cx,scan_cy;
 reg [19:0] scan_area;
 reg [31:0] scan_mx,scan_my,scan_area100,scan_box60,scan_box88;
 reg [25:0] scan_box;
 reg [2:0] replace_slot;
 reg [19:0] replace_area;
 reg [19:0] candidate_area[0:7];
 // Candidate moments are not needed until the geometric verifier finishes.
 // One synchronous, unreset memory read therefore has ample latency and
 // avoids a pair of large dynamically indexed register banks.
 (* ram_style="bram" *) reg [63:0] candidate_moments[0:7];
 reg [63:0] candidate_moment_q;
 reg [11:0] candidate_x[0:7],candidate_y[0:7],candidate_rx[0:7],candidate_ry[0:7];
 reg [3:0] check_index;
 reg check_wait,verify_start;
 wire verify_done,verify_pass;
 reg [19:0] best_area;
 reg [31:0] best_x,best_y;
 reg [11:0] best_radius;
 reg [31:0] dividend_x,dividend_y,divisor,qx,qy;
 reg [32:0] rem_x,rem_y;
 reg [5:0] bit_index;
 reg [15:0] mask_row_base;
 wire [15:0] mask_write_addr=mask_row_base+{6'd0,x[11:2]};
 wire mask_write_en=state==STREAM && I_de && x<1280 && y<720 && x[1:0]==2 && y[1:0]==2;
 integer j;
 wire [TRACKS-1:0] track_match,track_free;
 wire [11:0] track_distance[0:TRACKS-1];
 wire [16:0] match_leaf[0:15],free_leaf[0:15];
 wire [16:0] match_l1[0:7],free_l1[0:7],match_l2[0:3],free_l2[0:3];
 wire [16:0] match_l3[0:1],free_l3[0:1],match_root,free_root,selected_root;
 wire selected_valid=selected_root[16];
 wire [3:0] selected_slot=selected_root[15:12];
 wire [9:0] luma={2'b0,I_rgb[23:16]}+{1'b0,I_rgb[15:8],1'b0}+{2'b0,I_rgb[7:0]};
 wire dark=luma[9:2]<DARK_THRESHOLD;
 wire [11:0] run_mid=({1'b0,run_start}+{1'b0,run_end})>>1;
 wire [11:0] previous_y=y-12'd1;
 wire [11:0] previous_y2=y-12'd2;
 wire [12:0] length={1'b0,run_end}-{1'b0,run_start}+13'd1;
 wire [25:0] pair_product=({1'b0,run_start}+{1'b0,run_end})*length;
 wire [31:0] run_mx={6'd0,pair_product}>>1;
 wire [31:0] run_my=y*length;
 wire close_run=run_active && ((!I_de && dd) || (I_de && !dark && gap==2));
 wire [32:0] shift_x={rem_x[31:0],dividend_x[31]},shift_y={rem_y[31:0],dividend_y[31]};
 wire ge_x=shift_x>={1'b0,divisor},ge_y=shift_y>={1'b0,divisor};
 wire [31:0] next_qx={qx[30:0],ge_x},next_qy={qy[30:0],ge_y};

 // Candidate format: {valid, slot[3:0], distance[11:0]}.
 // The old procedural loop formed a TRACKS-deep priority/compare chain.  This
 // balanced selector has four compare levels for the fixed 16-track bank.
 function [16:0] F_pick_nearest;
   input [16:0] a,b;
   begin
     if(!a[16]) F_pick_nearest=b;
     else if(!b[16]) F_pick_nearest=a;
     else if(b[11:0]<a[11:0]) F_pick_nearest=b;
     else F_pick_nearest=a;
   end
 endfunction

 genvar g;
 generate
   for(g=0;g<TRACKS;g=g+1) begin: G_TRACK_MATCH
     wire [11:0] old_mid=({1'b0,last_left[g]}+{1'b0,last_right[g]})>>1;
     assign track_distance[g]=(run_mid>=old_mid) ? run_mid-old_mid : old_mid-run_mid;
     // Equivalent to y>last_y && y-last_y<=2, but equality comparators keep a
     // subtract/relational chain out of the timing path from last_y.
     assign track_match[g]=used[g] &&
                           ((last_y[g]==previous_y)||(last_y[g]==previous_y2)) &&
                           ({1'b0,run_start}<={1'b0,last_right[g]}+13'd2) &&
                           ({1'b0,run_end}+13'd2>={1'b0,last_left[g]});
     assign track_free[g]=!used[g];
   end
   for(g=0;g<16;g=g+1) begin: G_LEAF
     if(g<TRACKS) begin
       assign match_leaf[g]={track_match[g],g[3:0],track_distance[g]};
       assign free_leaf[g]={track_free[g],g[3:0],12'd0};
     end else begin
       assign match_leaf[g]=17'd0;
       assign free_leaf[g]=17'd0;
     end
   end
   for(g=0;g<8;g=g+1) begin: G_TREE_L1
     assign match_l1[g]=F_pick_nearest(match_leaf[g*2],match_leaf[g*2+1]);
     assign free_l1[g]=F_pick_nearest(free_leaf[g*2],free_leaf[g*2+1]);
   end
   for(g=0;g<4;g=g+1) begin: G_TREE_L2
     assign match_l2[g]=F_pick_nearest(match_l1[g*2],match_l1[g*2+1]);
     assign free_l2[g]=F_pick_nearest(free_l1[g*2],free_l1[g*2+1]);
   end
   for(g=0;g<2;g=g+1) begin: G_TREE_L3
     assign match_l3[g]=F_pick_nearest(match_l2[g*2],match_l2[g*2+1]);
     assign free_l3[g]=F_pick_nearest(free_l2[g*2],free_l2[g*2+1]);
   end
 endgenerate
 assign match_root=F_pick_nearest(match_l3[0],match_l3[1]);
 assign free_root=F_pick_nearest(free_l3[0],free_l3[1]);
 assign selected_root=match_root[16] ? match_root : free_root;
 wire [16:0] delayed_match=F_pick_nearest(F_pick_nearest(select_l2[0],select_l2[1]),
                                       F_pick_nearest(select_l2[2],select_l2[3]));
 wire [16:0] delayed_selected=delayed_match[16] ? delayed_match : select_free;

 function [22:0] pick_smallest;
 input [22:0] a,b;
 begin pick_smallest=(b[22:3]<a[22:3]) ? b : a;end
 endfunction
 // Pipeline the eight-candidate minimum tree.  The old three-level
 // candidate_area -> replace_area path was the routed critical path.  Phase 1
 // consumes this registered first level, leaving two short compare levels.
 reg [22:0] small_pipe[0:3];
 wire [22:0] small_l2[0:1],smallest;
 assign small_l2[0]=pick_smallest(small_pipe[0],small_pipe[1]);
 assign small_l2[1]=pick_smallest(small_pipe[2],small_pipe[3]);
 assign smallest=pick_smallest(small_l2[0],small_l2[1]);
 wire pending_conflict=(update_pending && update_slot==scan_index) ||
  (select_pending && delayed_selected[16] && delayed_selected[15:12]==scan_index);
 wire stale=used[scan_index] && last_y[scan_index]<y &&
  last_y[scan_index]!=previous_y && last_y[scan_index]!=previous_y2;
 wire shape_ok=scan_area>=MIN_AREA && scan_area<=MAX_AREA &&
  scan_width>=MIN_DIAMETER && scan_height>=MIN_DIAMETER && scan_width<=500 && scan_height<=500 &&
  scan_width*3>=scan_height*2 && scan_height*3>=scan_width*2 &&
  scan_area100>=scan_box60 && scan_area100<=scan_box88 && scan_inside;
 black_ring_verifier u_ring_check(
  .clk(I_clk),.rst_n(I_rst_n),.abort_check(state==DRAIN),
  .write_en(mask_write_en),.write_addr(mask_write_addr),.write_dark(dark),
  .start(verify_start),.cx(candidate_x[check_index[2:0]]),.cy(candidate_y[check_index[2:0]]),
  .rx(candidate_rx[check_index[2:0]]),.ry(candidate_ry[check_index[2:0]]),
  .done(verify_done),.passed(verify_pass));
 always @(posedge I_clk) begin
  candidate_moment_q<=candidate_moments[check_index[2:0]];
  if(!(I_vsync && !vd) && (state==STREAM || state==FINISH) &&
     scan_phase==3 && scan_eligible && shape_ok && scan_area>replace_area)
   candidate_moments[replace_slot]<={scan_mx,scan_my};
 end
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   state<=STREAM;drain_count<=0;vd<=0;dd<=0;frame_complete<=0;
   x<=0;y<=0;run_start<=0;run_end<=0;run_active<=0;gap<=0;mask_row_base<=0;
   select_pending<=0;update_pending<=0;select_free<=0;update_slot<=0;
   select_start<=0;select_end<=0;select_y<=0;select_length<=0;select_mx<=0;select_my<=0;
   update_start<=0;update_end<=0;update_y<=0;update_length<=0;update_mx<=0;update_my<=0;
   scan_index<=0;scan_phase<=0;scan_eligible<=0;scan_inside<=0;
   scan_width<=0;scan_height<=0;scan_cx<=0;scan_cy<=0;scan_area<=0;
   scan_mx<=0;scan_my<=0;scan_area100<=0;scan_box<=0;scan_box60<=0;scan_box88<=0;
   replace_slot<=0;replace_area<=0;check_index<=0;check_wait<=0;verify_start<=0;
   best_area<=0;best_x<=0;best_y<=0;best_radius<=0;
   dividend_x<=0;dividend_y<=0;divisor<=1;qx<=0;qy<=0;rem_x<=0;rem_y<=0;bit_index<=0;
   O_valid<=0;O_center_x<=640;O_center_y<=360;O_radius<=0;O_confidence<=0;
   for(j=0;j<4;j=j+1) select_l2[j]<=0;
   for(j=0;j<8;j=j+1) begin
    candidate_area[j]<=0;
    candidate_x[j]<=0;candidate_y[j]<=0;candidate_rx[j]<=0;candidate_ry[j]<=0;
   end
   for(j=0;j<4;j=j+1) small_pipe[j]<=0;
   for(j=0;j<TRACKS;j=j+1) begin
    used[j]<=0;min_x[j]<=0;max_x[j]<=0;min_y[j]<=0;max_y[j]<=0;
    last_left[j]<=0;last_right[j]<=0;last_y[j]<=0;area[j]<=0;moment_x[j]<=0;moment_y[j]<=0;
   end
  end else begin
   vd<=I_vsync;dd<=I_de;verify_start<=0;select_pending<=0;
   for(j=0;j<4;j=j+1)
    small_pipe[j]<=pick_smallest({candidate_area[j*2],(j[2:0]<<1)},
                                {candidate_area[j*2+1],(j[2:0]<<1)|3'd1});
   update_pending<=select_pending && delayed_selected[16];
   if(state==STREAM && !(I_vsync && !vd) && close_run && length>=4) begin
    select_pending<=1;select_free<=free_root;
    for(j=0;j<4;j=j+1) select_l2[j]<=match_l2[j];
    select_start<=run_start;select_end<=run_end;select_y<=y;select_length<=length;
    select_mx<=run_mx;select_my<=run_my;
   end
   if(select_pending) begin
    update_slot<=delayed_selected[15:12];update_start<=select_start;update_end<=select_end;
    update_y<=select_y;update_length<=select_length;update_mx<=select_mx;update_my<=select_my;
   end
   if(I_vsync && !vd) begin
    state<=DRAIN;drain_count<=3;scan_index<=0;scan_phase<=0;scan_eligible<=0;
    frame_complete<=y>=720;
    x<=0;y<=0;mask_row_base<=0;run_active<=0;gap<=0;
    best_area<=0;best_x<=0;best_y<=0;best_radius<=0;
    check_index<=0;check_wait<=0;O_valid<=0;O_confidence<=0;
   end else begin
    if(state==STREAM) begin
     if(I_de) begin
      x<=x+1'b1;
      if(dark && x<1280 && y<720) begin
       if(!run_active) begin run_active<=1;run_start<=x;end
       run_end<=x;gap<=0;
      end else if(run_active) begin
       if(gap==2) begin run_active<=0;gap<=0;end else gap<=gap+1'b1;
      end
     end else if(dd) begin
      x<=0;y<=y+1'b1;run_active<=0;gap<=0;
      if(y[1:0]==3) mask_row_base<=mask_row_base+16'd320;
     end
    end
    if(state==DRAIN) begin
     if(drain_count==0) begin state<=FINISH;scan_index<=0;scan_phase<=0;end
     else drain_count<=drain_count-1'b1;
    end
    if(state==STREAM || state==FINISH) begin
     case(scan_phase)
      0:begin
       scan_eligible<=used[scan_index] && (state==FINISH || (stale && !pending_conflict));
       scan_width<={1'b0,max_x[scan_index]}-{1'b0,min_x[scan_index]}+13'd1;
       scan_height<={1'b0,max_y[scan_index]}-{1'b0,min_y[scan_index]}+13'd1;
       scan_cx<=({1'b0,min_x[scan_index]}+{1'b0,max_x[scan_index]})>>1;
       scan_cy<=({1'b0,min_y[scan_index]}+{1'b0,max_y[scan_index]})>>1;
       scan_inside<=min_x[scan_index]>0 && max_x[scan_index]<1279 && min_y[scan_index]>0 && max_y[scan_index]<719;
       scan_area<=area[scan_index];scan_mx<=moment_x[scan_index];scan_my<=moment_y[scan_index];
       scan_phase<=1;
      end
      1:begin
       replace_slot<=smallest[2:0];replace_area<=smallest[22:3];
       scan_box<=scan_width*scan_height;scan_area100<={12'd0,scan_area}*32'd100;scan_phase<=2;
      end
      2:begin scan_box60<={6'd0,scan_box}*32'd60;scan_box88<={6'd0,scan_box}*32'd88;scan_phase<=3;end
      3:begin
       if(scan_eligible) begin
        used[scan_index]<=0;
        if(shape_ok && scan_area>replace_area) begin
         for(j=0;j<8;j=j+1) if(replace_slot==j[2:0]) begin
          candidate_area[j]<=scan_area;
          candidate_x[j]<=scan_cx;candidate_y[j]<=scan_cy;
          candidate_rx[j]<=(scan_width+13'd1)>>1;candidate_ry[j]<=(scan_height+13'd1)>>1;
         end
        end
       end
       scan_phase<=0;
       if(scan_index==TRACKS-1) begin
        scan_index<=0;
        if(state==FINISH) begin state<=CHECK;check_index<=0;check_wait<=0;end
       end else scan_index<=scan_index+1'b1;
      end
     endcase
    end
    if(state==CHECK) begin
     if(check_index==8) state<=PREPARE;
     else if(!check_wait) begin
      if(candidate_area[check_index[2:0]]==0 || !frame_complete) check_index<=check_index+1'b1;
      else begin verify_start<=1;check_wait<=1;end
     end else if(verify_done) begin
      if(verify_pass && candidate_area[check_index[2:0]]>best_area) begin
       best_area<=candidate_area[check_index[2:0]];
       best_x<=candidate_moment_q[63:32];best_y<=candidate_moment_q[31:0];
       best_radius<=(candidate_rx[check_index[2:0]]+candidate_ry[check_index[2:0]])>>1;
      end
      check_index<=check_index+1'b1;check_wait<=0;
     end
    end
    if(state==PREPARE) begin
     if(best_area!=0 && frame_complete) begin
      dividend_x<=best_x;dividend_y<=best_y;divisor<={12'd0,best_area};
      qx<=0;qy<=0;rem_x<=0;rem_y<=0;bit_index<=0;O_radius<=best_radius;state<=DIVIDE;
     end else begin
      state<=STREAM;
      for(j=0;j<8;j=j+1) candidate_area[j]<=0;
     end
    end
    if(state==DIVIDE) begin
     dividend_x<={dividend_x[30:0],1'b0};dividend_y<={dividend_y[30:0],1'b0};
     rem_x<=ge_x ? shift_x-{1'b0,divisor} : shift_x;rem_y<=ge_y ? shift_y-{1'b0,divisor} : shift_y;
     qx<=next_qx;qy<=next_qy;
     if(bit_index==31) begin
      O_valid<=next_qx<1280 && next_qy<720;O_center_x<=next_qx[11:0];O_center_y<=next_qy[11:0];
      O_confidence<=255;state<=STREAM;
      for(j=0;j<8;j=j+1) candidate_area[j]<=0;
     end else bit_index<=bit_index+1'b1;
    end
   end
   // Completed selection commits after two clocks; eligible closes are >=5 apart.
   if(update_pending) begin
    for(j=0;j<TRACKS;j=j+1) if(update_slot==j[3:0]) begin
     used[j]<=1;last_left[j]<=update_start;last_right[j]<=update_end;last_y[j]<=update_y;
     if(!used[j]) begin
      min_x[j]<=update_start;max_x[j]<=update_end;min_y[j]<=update_y;max_y[j]<=update_y;
      area[j]<=update_length;moment_x[j]<=update_mx;moment_y[j]<=update_my;
     end else begin
      if(update_min_x) min_x[j]<=update_start;
      if(update_max_x) max_x[j]<=update_end;
      max_y[j]<=update_y;area[j]<=update_area_sum;
      moment_x[j]<=update_moment_x_sum;moment_y[j]<=update_moment_y_sum;
     end
    end
   end
  end
 end
endmodule

// Quarter-resolution binary frame store. No RAM reset: every active frame
// overwrites all 57600 locations before verification begins in blanking.
module black_ring_verifier(
 input wire clk,rst_n,abort_check,write_en,write_dark,start,
 input wire [15:0] write_addr,
 input wire [11:0] cx,cy,rx,ry,
 output reg done,passed
);
 (* ram_style="bram" *) reg mask[0:57599];
 reg [15:0] read_addr;
 reg read_value;
 always @(posedge clk) begin
  if(write_en) mask[write_addr]<=write_dark;
  read_value<=mask[read_addr];
 end
 localparam IDLE=0,SCALE=1,DIRECTION=2,COORD=3,ADDRESS=4,WAIT_READ=5,CONSUME=6,EVALUATE=7;
 reg [2:0] state,band,ray;
 reg [11:0] center_x,center_y,radius_x,radius_y;
 reg [12:0] ox,oy;
 reg signed [13:0] dx,dy,point_x,point_y;
 reg point_valid;
 reg [3:0] hits[0:4];
 wire [14:0] diag_x_product={2'd0,ox}+{1'd0,ox,1'b0};
 wire [15:0] diag_y_product={3'd0,oy}+{1'd0,oy,2'd0};
 wire signed [13:0] px=$signed({1'b0,ox}),py=$signed({1'b0,oy});
 wire signed [13:0] qx=$signed({1'b0,diag_x_product[14:2]});
 wire signed [13:0] qy=$signed({1'b0,diag_y_product[15:3]});
 wire in_bounds=point_x>=0 && point_x<1280 && point_y>=0 && point_y<720;
 wire [15:0] sample_row={8'd0,point_y[9:2]};
 wire [15:0] sample_col={7'd0,point_x[10:2]};
 integer k;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   state<=IDLE;band<=0;ray<=0;done<=0;passed<=0;read_addr<=0;
   center_x<=0;center_y<=0;radius_x<=0;radius_y<=0;ox<=0;oy<=0;
   dx<=0;dy<=0;point_x<=0;point_y<=0;point_valid<=0;
   for(k=0;k<5;k=k+1) hits[k]<=0;
  end else begin
   done<=0;
   if(abort_check) begin state<=IDLE;passed<=0;end
   else case(state)
    IDLE:if(start) begin
     center_x<=cx;center_y<=cy;radius_x<=rx;radius_y<=ry;band<=0;ray<=0;passed<=0;
     for(k=0;k<5;k=k+1) hits[k]<=0;
     state<=SCALE;
    end
    SCALE:begin
     case(band)
      0:begin ox<=radius_x>>1;oy<=radius_y>>1;end
      1:begin ox<={1'b0,radius_x}+(radius_x>>2);oy<={1'b0,radius_y}+(radius_y>>2);end
      2:begin ox<={1'b0,radius_x}+(radius_x>>1)+(radius_x>>2);oy<={1'b0,radius_y}+(radius_y>>1)+(radius_y>>2);end
      3:begin ox<=({1'b0,radius_x}<<1)+(radius_x>>2);oy<=({1'b0,radius_y}<<1)+(radius_y>>2);end
      default:begin ox<=({1'b0,radius_x}<<1)+(radius_x>>1)+(radius_x>>3);oy<=({1'b0,radius_y}<<1)+(radius_y>>1)+(radius_y>>3);end
     endcase
     state<=DIRECTION;
    end
    DIRECTION:begin
     case(ray)
      0:begin dx<=px;dy<=0;end
      1:begin dx<=-px;dy<=0;end
      2:begin dx<=0;dy<=py;end
      3:begin dx<=0;dy<=-py;end
      4:begin dx<=qx;dy<=qy;end
      5:begin dx<=-qx;dy<=qy;end
      6:begin dx<=qx;dy<=-qy;end
      default:begin dx<=-qx;dy<=-qy;end
     endcase
     state<=COORD;
    end
    COORD:begin point_x<=$signed({2'd0,center_x})+dx;point_y<=$signed({2'd0,center_y})+dy;state<=ADDRESS;end
    ADDRESS:begin
     point_valid<=in_bounds;
     read_addr<=in_bounds ? (sample_row<<8)+(sample_row<<6)+sample_col : 16'd0;
     state<=WAIT_READ;
    end
    WAIT_READ:state<=CONSUME;
    CONSUME:begin
     // Out-of-image points never count as white or black evidence.
     if(point_valid && read_value==!band[0]) hits[band]<=hits[band]+1'b1;
     if(ray==7) begin
      ray<=0;
      if(band==4) state<=EVALUATE;
      else begin band<=band+1'b1;state<=SCALE;end
     end else begin ray<=ray+1'b1;state<=DIRECTION;end
    end
    EVALUATE:begin
     passed<=hits[0]>=6 && hits[1]>=6 && hits[2]>=6 && hits[3]>=5 && hits[4]>=4;
     done<=1;state<=IDLE;
    end
    default:state<=IDLE;
   endcase
  end
 end
endmodule
