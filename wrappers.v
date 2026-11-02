// Adapter ports keep measurement/stop separate from original R1 control arithmetic.
module pid_ctrl(input clk_50m,rst_n,input [11:0] theta,input[15:0] encoder,
 input[15:0] center,kp,kd,kpp,kdp,input[1:0] d_filter,input rebase_position,capture_rebase,balance_active,median_enable,
 input signed[15:0] step_target,output signed[20:0] duty_out,output[15:0] theta_rr,
 output[17:0] cnt_5ms,output[3:0] cnt_25ms,output signed[12:0] err,err_bias,po_err_bias,po_err_bias_r,
 output signed[15:0] encoder_r,output signed[20:0] duty,po_duty,output signed[31:0] d_q8);
 reg signed[15:0] position_origin;
 wire signed[15:0] local_encoder=$signed(encoder)-position_origin+16'sd10000;
 wire signed[15:0] local_target=step_target-position_origin+16'sd10000;
 wire signed[15:0] legacy_encoder;
 always @(posedge clk_50m or negedge rst_n)begin
  if(!rst_n)position_origin<=10000;
  else if(rebase_position)position_origin<=10000;
  else if(capture_rebase)position_origin<=encoder;
 end
 assign encoder_r=legacy_encoder+position_origin-16'sd10000;
 r1_pid original(.d_used_report(d_q8),.rebase_position(rebase_position || capture_rebase),.clk_50m(clk_50m),.rst_n(rst_n),.swing_key(1'b0),
 .theta(theta),.encoder(local_encoder),.duty_out(duty_out),.theta_rr(theta_rr),.cnt_5ms(cnt_5ms),.cnt_25ms(cnt_25ms),
 .err(err),.err_bias(err_bias),.po_err_bias(po_err_bias),.po_err_bias_r(po_err_bias_r),.encoder_r(legacy_encoder),
 .step_target(local_target),.duty(duty),.po_duty(po_duty));

endmodule

module auto_pid_ctrl(input position_enable,input signed[15:0] position_reference,input manual_mode,input signed[20:0] position_effort,input signed[20:0] angle_effort,input signed[31:0] d_used_q8,output capture_rebase,input[15:0] center,launch_step,output[1:0] state_out,output fault_out,
 output[15:0] attempts,output[7:0] capture_count,detail,
 input clk_50m,rst_n,input[2:0] swing_key,input[15:0] theta_rr,input[17:0] cnt_5ms,input[3:0] cnt_25ms,
 input signed[20:0] duty_out,input signed[12:0] err,err_bias,po_err_bias,po_err_bias_r,
 input signed[15:0] encoder,output signed[15:0] step_target,output signed[20:0] final_duty_out);
 wire[15:0] success;
 r1_auto original(.position_enable(position_enable),.position_reference(position_reference),.manual_mode(manual_mode),.angle_effort(angle_effort),.position_effort(position_effort),.d_used_q8(d_used_q8),.capture_rebase(capture_rebase),.state_out(state_out),.fault_out(fault_out),.success_out(success),.attempts_out(attempts),.detail_out(detail),.clk_50m(clk_50m),.rst_n(rst_n),
 .swing_key(swing_key),.theta_rr(theta_rr),.cnt_5ms(cnt_5ms),.cnt_25ms(cnt_25ms),.duty_out(duty_out),
 .err(err),.err_bias(err_bias),.po_err_bias(po_err_bias),.po_err_bias_r(po_err_bias_r),.encoder(encoder),
 .step_target(step_target),.final_duty_out(final_duty_out));

 assign capture_count=success[7:0];
endmodule

module motor_ctrl(input clk_50m,rst_n,enable,input signed[20:0] duty_out,
 output pwm,ain1,ain2,output signed[20:0] applied);
 wire raw_pwm,raw_a,raw_b;
 wire signed[20:0] gated=enable?duty_out:21'sd0;
 // Gate physical outputs once. Removing the redundant input gate shortens the
 // enable-to-PWM path without changing enabled PWM or disabled pin levels.
 r1_motor original(.clk_50m(clk_50m),.rst_n(rst_n),.duty_out(duty_out),.pwm(raw_pwm),.ain1(raw_a),.ain2(raw_b),.stby());
 assign pwm=enable && raw_pwm;assign ain1=enable && raw_a;assign ain2=enable && raw_b;
 assign applied=gated;
endmodule
