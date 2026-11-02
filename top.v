module top(input clk_50m,rst_n,input [3:0] key_sw,
 output uart_tx,input uart_rx,output adc_clk,adc_oe_n,adc_stby_n,
 input [9:0] adc_data_in,input enc_a,enc_b,
 output BN1,BN2,PWMB,pwmtest,led3);
 wire [3:0] keys;genvar g;
 generate for(g=0;g<4;g=g+1)begin:KEYS
 key_filter k(clk_50m,rst_n,key_sw[g],keys[g]);end endgenerate
 wire [11:0] theta;wire [15:0] encoder_raw,theta_rr;
 ADC adc(clk_50m,rst_n,adc_clk,adc_oe_n,adc_stby_n,,adc_data_in,theta);
 wire start_rebase;
 encoder enc(clk_50m,rst_n && !start_rebase,enc_a,enc_b,encoder_raw);
 reg [15:0] center,kp,kd,kpp,kdp,led_band,launch_step;reg median_enable;reg [1:0] d_filter;
 wire signed [20:0] duty_out,final_duty,angle_loop,pos_loop,applied;
 wire signed [15:0] encoder_sample,step_target;
 wire signed [12:0] err,err_bias,pe,pe_old;
 wire signed [31:0] d_q8;
 wire [17:0] phase;wire [3:0] phase25;wire [1:0] state;wire auto_fault;
 reg cal_busy,cal_valid;reg[1:0] control_mode;reg[2:0] mode_reset_count;wire capture_rebase;
 reg stopped;reg [7:0] fault_code;
 reg step_request;reg signed[15:0] step_delta,key_step_counts;reg[15:0] counts_per_revolution,step_sequence;
 wire step_active;reg[12:0] step_speed_q8;reg[2:0] step_speed_level;
 reg[9:0] cruise_q8;reg[15:0] cruise_speed;reg position_command,position_hold;reg signed[15:0] position_command_offset;
 wire position_ready,position_moving,position_paused;
 wire signed[15:0] position_reference,position_goal,position_origin,position_goal_offset,position_velocity;
 wire[15:0] rod_distance=theta_rr>=3200?theta_rr-3200:3200-theta_rr;
 position_profile trajectory(.step_speed_q8(step_speed_q8),.step_request(step_request),.step_delta(step_delta),.key_step_counts(key_step_counts),.step_active(step_active),.cruise_q8(cruise_q8),.clk(clk_50m),.rst_n(rst_n),.tick(phase==30 && phase25==0),
 .active(state==1 && control_mode==2 && !stopped),.base_target(step_target),.angle_distance(rod_distance),.d_q8(d_q8),
 .set_goal(position_command),.hold(position_hold),.goal_offset(position_command_offset),.key_plus(keys[1]),.key_minus(keys[2]),
 .ready(position_ready),.moving(position_moving),.paused(position_paused),.reference(position_reference),.goal(position_goal),.origin(position_origin),.offset(position_goal_offset),.velocity_q8(position_velocity));
 pid_ctrl pid(clk_50m,rst_n,theta,encoder_raw,center,kp,kd,kpp,kdp,d_filter,start_rebase,capture_rebase,(state==1),median_enable,
 step_target,duty_out,theta_rr,phase,phase25,err,err_bias,pe,pe_old,encoder_sample,angle_loop,pos_loop,d_q8);
 wire [2:0] control_keys={(control_mode==2 ? 2'b00:keys[2:1]),keys[0] && !stopped && !cal_busy && mode_reset_count==0};
 wire [15:0] attempts;wire [7:0] capture_count,auto_detail;
 wire auto_reset_n=rst_n && mode_reset_count==0;
 auto_pid_ctrl auto_ctrl(.position_enable(position_ready && control_mode==2),.position_reference(position_reference),.manual_mode(control_mode==1),.angle_effort(angle_loop),.position_effort(pos_loop),.d_used_q8(d_q8),.capture_rebase(capture_rebase),.launch_step(launch_step),.attempts(attempts),.capture_count(capture_count),.detail(auto_detail),.center(center),.state_out(state),.fault_out(auto_fault),
 .clk_50m(clk_50m),.rst_n(auto_reset_n),.swing_key(control_keys),.theta_rr(theta_rr),
 .cnt_5ms(phase),.cnt_25ms(phase25),.duty_out(duty_out),.err(err),.err_bias(err_bias),
 .po_err_bias(pe),.po_err_bias_r(pe_old),.encoder(encoder_sample),.step_target(step_target),.final_duty_out(final_duty));
 assign start_rebase=keys[0] && state==0 && !stopped && mode_reset_count==0;
 wire motor_enable=rst_n && !stopped && (state!=0) && mode_reset_count==0;
 motor_ctrl motor(clk_50m,rst_n,motor_enable,final_duty,PWMB,BN1,BN2,applied);
 assign pwmtest=PWMB;
 reg near_top;assign led3=near_top;
 wire [15:0] led_angle={4'b0,theta}+16'd135;
 wire [15:0] led_distance=led_angle>=16'd3200 ? led_angle-16'd3200:16'd3200-led_angle;
 reg [22:0] led_hold;
 reg [31:0] boot_ticks;reg [15:0] ms_div;reg [31:0] milliseconds;
 always @(posedge clk_50m or negedge rst_n)begin
  if(!rst_n)begin boot_ticks<=0;near_top<=0;led_hold<=0;ms_div<=0;milliseconds<=0;end
  else begin
   if(boot_ticks<500000)boot_ticks<=boot_ticks+1'b1;
   if(ms_div==49999)begin ms_div<=0;milliseconds<=milliseconds+1'b1;end else ms_div<=ms_div+1'b1;
   // R1 ADC coordinate, fixed3200, no R20 median/phase qualification.
   if(boot_ticks>=500000)begin
    if(led_distance<=40)begin near_top<=1;led_hold<=23'd5000000;end
    else if(led_hold!=0)led_hold<=led_hold-1'b1;
    else if(led_distance>=80)near_top<=0;
   end
  end
 end
 wire cmd_valid;wire [7:0] cmd,cmd_seq;wire [15:0] value;
 wire signed[16:0] requested_step_offset=$signed(position_goal_offset)+$signed(value);
 reg [7:0] ack_seq,ack_status;reg link_armed;reg [24:0] heartbeat_age;
 reg [6:0] cal_count;reg [21:0] cal_sum;
 reg [15:0] cal_min,cal_max;wire [21:0] cal_total=cal_sum+theta_rr;
 wire [15:0] next_min=theta_rr<cal_min?theta_rr:cal_min;
 wire [15:0] next_max=theta_rr>cal_max?theta_rr:cal_max;
 wire [15:0] cal_mean=(cal_total+32)>>6;
 always @(posedge clk_50m or negedge rst_n)begin
  if(!rst_n)begin
   center<=3200;kp<=50;kd<=244;kpp<=25;kdp<=600;led_band<=40;d_filter<=1;median_enable<=0;launch_step<=383;
   step_request<=0;step_delta<=0;key_step_counts<=83;counts_per_revolution<=1000;step_speed_q8<=768;step_speed_level<=2;step_sequence<=0;cruise_q8<=256;cruise_speed<=40;position_command<=0;position_hold<=0;position_command_offset<=0;control_mode<=0;mode_reset_count<=0;stopped<=0;fault_code<=0;ack_seq<=0;ack_status<=0;link_armed<=0;heartbeat_age<=0;
   cal_busy<=0;cal_valid<=0;cal_count<=0;cal_sum<=0;cal_min<=65535;cal_max<=0;
  end else begin
   step_request<=0;position_command<=0;position_hold<=0;
   if(mode_reset_count!=0)mode_reset_count<=mode_reset_count-1'b1;
   if(link_armed && heartbeat_age<25000000)heartbeat_age<=heartbeat_age+1'b1;
   if(link_armed && heartbeat_age>=25000000)begin stopped<=1;fault_code<=2;end
   if(keys[0] && state!=0)begin stopped<=1;fault_code<=1;end
   if(control_mode==2 && state==1 && position_ready && !stopped)begin
    if(keys[1] && ($signed(position_goal_offset)+$signed(key_step_counts)<=3000) && ($signed(position_goal_offset)+$signed(key_step_counts)>=-3000))step_sequence<=step_sequence+1'b1;
    else if(keys[2] && ($signed(position_goal_offset)-$signed(key_step_counts)<=3000) && ($signed(position_goal_offset)-$signed(key_step_counts)>=-3000))step_sequence<=step_sequence+1'b1;
   end
   if(cmd_valid)begin
    // Heartbeats do not overwrite a pending configuration acknowledgement.
    if(cmd==1 && value==16'h4a28)begin link_armed<=1;heartbeat_age<=0;end
    else begin
     ack_seq<=cmd_seq;ack_status<=0;
     if(cmd==2)begin stopped<=1;fault_code<=1;end
     else if(cmd==48)begin
      if(stopped)ack_status<=7;
      else if(value>2)ack_status<=3;
      else begin
       control_mode<=value[1:0];
       // A confirmed balance keeps its inner loop running during mode handover.
       // The position generator adopts the existing target without a step.
       if(state!=1)mode_reset_count<=3;
      end
     end
     else if(cmd==52)begin
      if(stopped)ack_status<=7;
      else if(control_mode!=2 || state!=1 || !position_ready)ack_status<=2;
      else if($signed(value)==0 || $signed(value)>1000 || $signed(value)<-1000 || requested_step_offset>3000 || requested_step_offset< -3000)ack_status<=3;
      else begin step_request<=1;step_delta<=value;step_sequence<=step_sequence+1'b1;end
     end
     else if(cmd==53 || cmd==54)begin
      if(stopped)ack_status<=7;
      else if(control_mode!=2 || position_moving)ack_status<=2;
      else if(cmd==53)begin
       if(value<120 || value>12000)ack_status<=3;else counts_per_revolution<=value;
      end else if($signed(value)==0 || $signed(value)>1000 || $signed(value)<-1000)ack_status<=3;
      else key_step_counts<=value;
     end
     else if(cmd==55)begin
      if(stopped)ack_status<=7;
      else if(control_mode!=2)ack_status<=2;
      else case(value)
       60:begin step_speed_q8<=384;step_speed_level<=1;end
       120:begin step_speed_q8<=768;step_speed_level<=2;end
       180:begin step_speed_q8<=1152;step_speed_level<=3;end
       240:begin step_speed_q8<=1536;step_speed_level<=4;end
       default:ack_status<=3;
      endcase
     end
     else if(cmd==51)begin
      if(stopped)ack_status<=7;
      else if(control_mode!=2)ack_status<=2;
      else case(value)
       20:begin cruise_q8<=128;cruise_speed<=20;end
       40:begin cruise_q8<=256;cruise_speed<=40;end
       60:begin cruise_q8<=384;cruise_speed<=60;end
       default:ack_status<=3;
      endcase
     end
     else if(cmd==49 || cmd==50)begin
      if(stopped)ack_status<=7;
      else if(control_mode!=2 || state!=1 || !position_ready || mode_reset_count!=0)ack_status<=2;
      else if(cmd==49 && ($signed(value)>300 || $signed(value)<-300))ack_status<=3;
      else if(cmd==49)begin position_command<=1;position_command_offset<=value;end
      else position_hold<=1;
     end
     else if(state!=0 || cal_busy || keys[0])ack_status<=2;
     else case(cmd)
      default:ack_status<=6; // fixed original R1 parameters, center3200 cannot be written
     endcase
    end
   end
  end
 end
 reg [15:0] telemetry_sequence;reg toggle;reg send_sample;reg [767:0] payload;
 reg signed [18:0] signed_on;reg [17:0] absolute_on;
 reg signed [31:0] measured_on;reg [31:0] measured_absolute;
 reg pwm_observed,direction_observed;
 // Measurement-only input register; no register is inserted into the motor drive path.
 always @(posedge clk_50m or negedge rst_n)begin
  if(!rst_n)begin pwm_observed<=0;direction_observed<=0;end
  else begin pwm_observed<=PWMB;direction_observed<=BN1;end
 end
 always @(posedge clk_50m or negedge rst_n)begin
  if(!rst_n)begin signed_on<=0;absolute_on<=0;measured_on<=0;measured_absolute<=0;end
  else if(phase==7499)begin measured_on<=signed_on;measured_absolute<=absolute_on;
   signed_on<=pwm_observed?(direction_observed?32'sd1:-32'sd1):0;absolute_on<=pwm_observed?1:0;end
  else if(pwm_observed)begin signed_on<=signed_on+(direction_observed?32'sd1:-32'sd1);absolute_on<=absolute_on+1'b1;end
 end
 always @(posedge clk_50m or negedge rst_n)begin
  if(!rst_n)begin telemetry_sequence<=0;toggle<=0;send_sample<=0;payload<=0;end
  else begin
   send_sample<=0;
   if(phase==7500)begin
    toggle<=!toggle;
    if(boot_ticks>=500000)begin
     send_sample<=1;telemetry_sequence<=telemetry_sequence+1'b1;
     payload[0+:32]<=32'h62285aa5;payload[32+:16]<=telemetry_sequence;payload[48+:32]<=milliseconds;
     payload[80+:16]<=theta_rr;payload[96+:16]<=center;payload[112+:16]<=encoder_sample;
     payload[128+:16]<=step_target;
     payload[144+:32]<={{11{final_duty[20]}},final_duty};
     payload[176+:32]<={{11{applied[20]}},applied};
     payload[208+:32]<={{11{angle_loop[20]}},angle_loop};
     payload[240+:32]<={{11{pos_loop[20]}},pos_loop};
     payload[272+:16]<={{3{err_bias[12]}},err_bias};payload[288+:32]<=d_q8;
     payload[320+:8]<={4'b0,state};
     payload[328+:8]<={1'b0,cal_valid,cal_busy,link_armed,auto_fault,stopped,near_top,motor_enable};
     payload[336+:16]<=kp;payload[352+:16]<=kd;payload[368+:16]<=kpp;payload[384+:16]<=kdp;
     payload[400+:8]<={6'b0,d_filter};payload[408+:16]<=led_band;
     payload[424+:8]<=ack_seq;payload[432+:8]<=ack_status;payload[440+:8]<=fault_code;
     payload[448+:16]<=25_000;payload[464+:16]<=theta;payload[480+:16]<=16'h4a28;
     payload[496+:16]<={4'b0,theta}+16'd135;
     payload[512+:32]<=measured_on;payload[544+:32]<=measured_absolute;
     payload[576+:16]<=attempts;payload[592+:8]<=capture_count;
     payload[600+:8]<={position_ready,position_moving,control_mode,position_paused,auto_detail[2:1],median_enable};payload[608+:16]<=launch_step;
     payload[624+:16]<=position_goal;payload[640+:16]<=position_velocity;
     payload[656+:16]<=position_origin;payload[672+:16]<=position_goal_offset;payload[688+:16]<=cruise_speed;
     payload[704+:16]<=counts_per_revolution;payload[720+:16]<=key_step_counts;payload[736+:16]<=step_sequence;
     payload[752+:8]<=auto_detail;payload[760+:8]<={4'b0,step_speed_level,step_active};
    end
   end
  end
 end
 serial_link link(clk_50m,rst_n,uart_rx,send_sample,payload,uart_tx,cmd_valid,cmd,cmd_seq,value);
endmodule


