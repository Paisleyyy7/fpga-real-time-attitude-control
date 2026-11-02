// 25ms target generator. Keeps the existing inner balance loop in control.
// Q8 counts/25ms. Continuous:20/40/60 counts/s,50 counts/s^2.
// Step speed configurable60/120/180/240counts/s, acceleration200counts/s^2; unchanged tilt pause.
module position_profile(input step_request,input signed[15:0] step_delta,key_step_counts,output step_active,input[9:0] cruise_q8,input clk,rst_n,tick,active,input signed[15:0] base_target,
 input[15:0] angle_distance,input signed[31:0] d_q8,input set_goal,hold,input signed[15:0] goal_offset,
 input key_plus,key_minus,output reg ready,output moving,paused,
 output signed[15:0] reference,output reg signed[15:0] goal,origin,offset,output signed[15:0] velocity_q8,input[12:0] step_speed_q8);
 reg signed[31:0] ref_q8;reg signed[12:0] velocity;
 reg signed[31:0] error_q8;
 reg signed[12:0] desired,next_velocity;
 reg[11:0] speed;reg[31:0] distance,brake;
 wire[23:0] speed_square=speed*speed;
 reg[2:0] pipeline;reg snap;reg step_motion;
 wire signed[12:0] acceleration=step_motion ? 13'sd32:13'sd8;
 wire signed[12:0] max_velocity=step_motion ? $signed(step_speed_q8):$signed({3'b000,cruise_q8});
 wire signed[16:0] next_step_offset=$signed(offset)+$signed(step_delta);
 wire signed[16:0] key_plus_offset=$signed(offset)+$signed(key_step_counts);
 wire signed[16:0] key_minus_offset=$signed(offset)-$signed(key_step_counts);
 assign step_active=ready && step_motion;
 wire[31:0] rod_speed=d_q8<0?-d_q8:d_q8;
 assign reference=ref_q8>>>8;
 assign velocity_q8=velocity;
 assign moving=ready && (velocity!=0 || reference!=goal);
 reg pause_latched;reg[3:0] pause_bad,pause_good;
 assign paused=ready && pause_latched;
 // Debounced hysteresis: do not start/stop a cruise on every ordinary balance correction.
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin pause_latched<=0;pause_bad<=0;pause_good<=0;end
  else if(!active || !ready)begin pause_latched<=0;pause_bad<=0;pause_good<=0;end
  else if(tick)begin
   if(!pause_latched)begin
    pause_good<=0;
    if(angle_distance>240 || rod_speed>12288)begin
     if(pause_bad<3)pause_bad<=pause_bad+1'b1;
     if(pause_bad>=2 || angle_distance>400)begin pause_latched<=1;pause_bad<=0;end
    end else pause_bad<=0;
   end else begin
    pause_bad<=0;
    if(angle_distance<160 && rod_speed<8192)begin
     if(pause_good<8)pause_good<=pause_good+1'b1;
     if(pause_good>=7)begin pause_latched<=0;pause_good<=0;end
    end else pause_good<=0;
   end
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin step_motion<=0;ready<=0;goal<=10000;origin<=10000;offset<=0;ref_q8<=2560000;velocity<=0;pipeline<=0;error_q8<=0;desired<=0;next_velocity<=0;speed<=0;distance<=0;brake<=0;snap<=0;end
  else if(!active)begin step_motion<=0;ready<=0;velocity<=0;pipeline<=0;end
  else if(!ready)begin step_motion<=0;ready<=1;origin<=base_target;goal<=base_target;offset<=0;ref_q8<=$signed(base_target)*32'sd256;velocity<=0;pipeline<=0;end
  else if(set_goal)begin step_motion<=0;offset<=goal_offset;goal<=origin+goal_offset;pipeline<=0;end
  else if(hold)begin goal<=reference;offset<=reference-origin;pipeline<=0;end
  else if(step_request)begin
   pipeline<=0;
   if(next_step_offset<=3000 && next_step_offset>=-3000)begin offset<=next_step_offset;goal<=origin+next_step_offset;step_motion<=1;end
  end
  else if(key_plus)begin
   pipeline<=0;if(key_plus_offset<=3000 && key_plus_offset>=-3000)begin offset<=key_plus_offset;goal<=origin+key_plus_offset;step_motion<=1;end
   /* legacy continuous keys moved to calibrated steps
   if(offset<=250)begin offset<=offset+50;goal<=goal+50;end else begin offset<=300;goal<=origin+300;end */
  end else if(key_minus)begin
   pipeline<=0;if(key_minus_offset<=3000 && key_minus_offset>=-3000)begin offset<=key_minus_offset;goal<=origin+key_minus_offset;step_motion<=1;end
   /* if(offset>=-250)begin offset<=offset-50;goal<=goal-50;end else begin offset<=-300;goal<=origin-300;end */
  end else case(pipeline)
   0:if(tick)begin error_q8<=$signed(goal)*32'sd256-ref_q8;speed<=velocity<0?-velocity:velocity;pipeline<=1;end
   1:begin distance<=error_q8<0?-error_q8:error_q8;brake<=(step_motion ? (speed_square>>6):(speed_square>>4))+({20'd0,speed}<<1)+32'd32;pipeline<=2;end
   2:begin
    desired<=error_q8>0 ? max_velocity : (error_q8<0 ? -max_velocity:13'sd0);
    if(paused || (((velocity>0 && error_q8>=0)||(velocity<0 && error_q8<=0)) && distance<=brake))desired<=0;
    snap<=distance<=32 && speed<=acceleration;pipeline<=3;
   end
   3:begin
    if(velocity<desired)next_velocity<=(desired-velocity<=acceleration)?desired:velocity+acceleration;
    else if(velocity>desired)next_velocity<=(velocity-desired<=acceleration)?desired:velocity-acceleration;
    else next_velocity<=velocity;
    pipeline<=4;
   end
   4:begin
    if(snap)begin ref_q8<=$signed(goal)*32'sd256;velocity<=0;end
    else begin ref_q8<=ref_q8+next_velocity;velocity<=next_velocity;end
    pipeline<=0;
   end
   default:pipeline<=0;
  endcase
 end
endmodule
