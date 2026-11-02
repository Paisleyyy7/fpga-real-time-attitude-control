// 25kHz carrier; original R1 duty-to-average-voltage gain is preserved.
// 2000 clocks/period at50MHz. Compare 5*phase with2*legacy_threshold:
// threshold/5000 == (2*threshold)/(5*2000). Rounding error <1/2000.
module r1_motor(input clk_50m,rst_n,input signed[20:0] duty_out,
 output stby,pwm,ain1,ain2);
 reg[10:0] carrier;
 wire[20:0] magnitude=duty_out<0?-duty_out:duty_out;
 wire[20:0] threshold=(magnitude>>1)+(magnitude>>3)+(magnitude>>4)+(magnitude>>7)+(magnitude>>8);
 wire[13:0] phase5={3'b0,carrier}+({3'b0,carrier}<<2);
 wire[21:0] threshold2={threshold,1'b0};
 always @(posedge clk_50m or negedge rst_n)
  if(!rst_n)carrier<=0;else if(carrier==1999)carrier<=0;else carrier<=carrier+1'b1;
 assign pwm=phase5<threshold2;
 assign ain1=!duty_out[20];assign ain2=duty_out[20];
 assign stby=1'b1;
endmodule
