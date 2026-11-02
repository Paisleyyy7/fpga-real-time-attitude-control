// R1-derived repeating same-direction kick. Return is an active position servo, not a stop latch.
module r1_auto(input position_enable,input signed[15:0] position_reference,input manual_mode,input signed[20:0] position_effort,input signed[20:0] angle_effort,input signed[31:0] d_used_q8,output reg capture_rebase,output[1:0] state_out,output fault_out,output[15:0] success_out,
 output reg[15:0] attempts_out,output[7:0] detail_out,
 input clk_50m,rst_n,input[2:0] swing_key,input[15:0] theta_rr,input[17:0] cnt_5ms,input[3:0] cnt_25ms,
 input signed[20:0] duty_out,input signed[12:0] err,err_bias,po_err_bias,po_err_bias_r,
 input signed[15:0] encoder,output reg signed[15:0] step_target,output reg signed[20:0] final_duty_out);
 localparam IDLE=0,BALANCE=1,AUTO=2,LAUNCH=0,RETURN=1,CAPTURE=2;
 reg[1:0] state,stage;reg[7:0] age,near_count,rest_count,outside_count,stable_count;
 reg[4:0] capture_age;
 reg[15:0] previous_angle;
 reg signed[26:0] capture_outer_scaled,capture_mix;

 reg[39:0] stable_history;reg[5:0] stable_votes;

 wire signed[32:0] predicted_error=$signed(err)*33'sd256+$signed(d_used_q8)*33'sd3;
 wire[32:0] predicted_abs=predicted_error<0 ? -predicted_error:predicted_error;
 wire approaching=(err<0 && d_used_q8>0)||(err>0 && d_used_q8<0);
 reg[15:0] good_count;reg signed[15:0] last_encoder;
 reg signed[16:0] pos_err,pos_old;reg signed[31:0] pd;reg signed[20:0] pd_limited;
 wire[15:0] distance=theta_rr>=3200 ? theta_rr-3200 : 3200-theta_rr;
 wire[15:0] bottom_distance=theta_rr>=1152 ? theta_rr-1152 : 1152-theta_rr;
 wire signed[16:0] home_error=$signed(encoder)-17'sd10000;
 wire signed[16:0] enc_delta=$signed(encoder)-$signed(last_encoder);
 wire[16:0] home_abs=home_error<0 ? -home_error : home_error;
 wire[16:0] speed_abs=enc_delta<0 ? -enc_delta : enc_delta;
 wire[12:0] angle_delta_abs=err_bias<0 ? -err_bias : err_bias;
 wire[31:0] filtered_speed=d_used_q8<0 ? -d_used_q8 : d_used_q8;
 wire crossing=((previous_angle<3200 && theta_rr>=3200)||(previous_angle>3200 && theta_rr<=3200)) && distance<=120 && filtered_speed<=23040 && angle_delta_abs<=180;
 wire stable_sample=(distance<=120 && filtered_speed<=10240 && angle_delta_abs<=120);
 wire capture_entry=angle_delta_abs<=180 && ((distance<=250 && filtered_speed<=32'd10240) || (approaching && distance<=600 && filtered_speed<=32'd23040 && predicted_abs<=33'd51200));
 function signed[20:0] limit_angle;
 input signed[20:0] value;
 begin if(value>=7200)limit_angle=6900;else if(value<=-7200)limit_angle=-6900;else limit_angle=value;end
 endfunction
 assign state_out=state;assign fault_out=1'b0;assign success_out=good_count;
 assign detail_out={capture_age,stage,1'b0};
 always @(posedge clk_50m or negedge rst_n)begin
  if(!rst_n)begin
   previous_angle<=1152;capture_outer_scaled<=0;capture_mix<=0;stable_history<=0;stable_votes<=0;capture_rebase<=0;capture_age<=0;state<=IDLE;stage<=LAUNCH;step_target<=10000;final_duty_out<=0;attempts_out<=0;
   age<=0;near_count<=0;rest_count<=0;outside_count<=0;stable_count<=0;good_count<=0;
   last_encoder<=10000;pos_err<=0;pos_old<=0;pd<=0;pd_limited<=0;
  end else begin
   capture_rebase<=0;
   if(state==IDLE)begin
    final_duty_out<=0;
    if(swing_key[0])begin
     near_count<=0;good_count<=0;stable_count<=0;outside_count<=0;rest_count<=0;age<=0;
     if(distance<=500)begin state<=BALANCE;step_target<=10000;end
     else if(!manual_mode)begin state<=AUTO;stage<=RETURN;step_target<=10000;attempts_out<=0;
      pos_err<=0;pos_old<=0;pd<=0;pd_limited<=0;end
    end
   end
   if(state==BALANCE)begin
    if(position_enable)step_target<=position_reference;
    else if(swing_key[1])step_target<=step_target+50;
    else if(swing_key[2])step_target<=step_target-50;
   end
   if(state!=IDLE && cnt_5ms==0)pos_old<=pos_err;
   if(state!=IDLE && cnt_5ms==2)pos_err<=$signed(encoder)-$signed(step_target);
   if(state!=IDLE && cnt_5ms==6)pd<=32'sd90*pos_err+32'sd180*(pos_err-pos_old);
   if(state!=IDLE && cnt_5ms==8)begin
    if(pd>6900)pd_limited<=6900;else if(pd< -6900)pd_limited<=-6900;else pd_limited<=pd;
   end
   // Pipeline the outer-loop blend; no change to the existing balanced PID arithmetic.
   if(cnt_5ms==6)capture_outer_scaled<=position_effort*$signed({1'b0,capture_age});
   if(cnt_5ms==8)capture_mix<=$signed(angle_effort)-(capture_outer_scaled>>>4);
   if(cnt_5ms==12)begin
    previous_angle<=theta_rr;
    last_encoder<=encoder;
    if(state==BALANCE)begin
     final_duty_out<=duty_out;
     // Automatic modes recover a fallen rod through the existing active return.
     if(!manual_mode)begin
      if(distance>600)begin if(outside_count<20)outside_count<=outside_count+1'b1;end else outside_count<=0;
      if(outside_count>=19 && distance>600)begin
       state<=AUTO;stage<=RETURN;step_target<=10000;near_count<=0;rest_count<=0;good_count<=0;stable_count<=0;outside_count<=0;
       stable_history<=0;stable_votes<=0;pos_err<=home_error;pos_old<=home_error;
      end
     end
    end
    else if(state==AUTO)case(stage)
     LAUNCH:begin
      final_duty_out<=pd_limited;
      if(age<255)age<=age+1'b1;
      if(age>=150)begin
       stage<=RETURN;step_target<=10000;near_count<=0;rest_count<=0;good_count<=0;stable_count<=0;
       pos_err<=home_error;pos_old<=home_error;
      end
     end
     RETURN:begin
      final_duty_out<=pd_limited;
      if(home_abs<=20 && speed_abs<=4 && bottom_distance<=250 && filtered_speed<=4096 && angle_delta_abs<=24)begin if(rest_count<10)rest_count<=rest_count+1'b1;end else rest_count<=0;
      if(rest_count>=10 && home_abs<=20 && speed_abs<=4 && bottom_distance<=250 && filtered_speed<=4096 && angle_delta_abs<=24)begin
       stage<=LAUNCH;step_target<=10383;age<=0;near_count<=0;good_count<=0;stable_count<=0;
       pos_err<=-383;pos_old<=-383;pd<=-34470;pd_limited<=-6900;
       if(attempts_out!=65535)attempts_out<=attempts_out+1'b1;
      end
     end
     CAPTURE:begin
      if(capture_age<16)begin
       if(distance<=600)begin
        if(capture_mix>=7200)final_duty_out<=6900;else if(capture_mix<=-7200)final_duty_out<=-6900;else final_duty_out<=capture_mix;
       end else final_duty_out<=duty_out;
       capture_age<=capture_age+1'b1;end
      else final_duty_out<=duty_out;
      if(distance<=500)begin if(good_count<110)good_count<=good_count+1'b1;end else good_count<=0;
      stable_history<={stable_history[38:0],stable_sample};
      stable_votes<=stable_votes+{5'd0,stable_sample}-{5'd0,stable_history[39]};
      if(distance>600)begin if(outside_count<20)outside_count<=outside_count+1'b1;end else outside_count<=0;
      if(good_count>=109 && stable_votes>=30 && stable_sample)state<=BALANCE;
      else if(outside_count>=19)begin
       stage<=RETURN;step_target<=10000;near_count<=0;rest_count<=0;good_count<=0;stable_count<=0;outside_count<=0;
       pos_err<=home_error;pos_old<=home_error;
      end
     end
     default:begin stage<=RETURN;step_target<=10000;end
    endcase
    // Both launch and active return may catch the rod. Two valid samples, not55ms fly-through wait.
    if(state==AUTO && stage!=CAPTURE)begin
     if(capture_entry || crossing)begin
      if(near_count>=1 || crossing)begin
       stable_history<=0;stable_votes<=0;stage<=CAPTURE;step_target<=encoder;capture_rebase<=1;capture_age<=0;
       near_count<=0;good_count<=0;stable_count<=0;outside_count<=0;
       final_duty_out<=limit_angle(angle_effort);
      end else near_count<=near_count+1'b1;
     end else near_count<=0;
    end
   end
  end
 end
endmodule
