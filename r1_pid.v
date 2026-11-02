


module r1_pid(output signed [31:0] d_used_report,input rebase_position,

    input     					clk_50m ,	//时钟输入
    input     					rst_n   ,	//复位输入
	
	input				[00:0]	swing_key,
	
    input				[11:0]	theta 	,
    input				[15:0]	encoder	,
    output	reg signed	[20:0]	duty_out,
	
	output				[15:0]	theta_rr,
	output	reg			[17:0]	cnt_5ms	,
	output	reg			[03:0]	cnt_25ms,
	//angle
	output	reg	signed	[12:0]	err		,
	output	reg	signed  [12:0]	err_bias,
	//position
	output	reg	signed  [12:0]	po_err_bias,
	output	reg	signed  [12:0]	po_err_bias_r,
	//encoder
	output	reg	signed  [15:0]	encoder_r 	,
	//目标位置
	input		signed	[15:0]	step_target	,
	
	output		signed	[20:0]	duty		,
	output		signed	[20:0]	po_duty			
	
    );
	
//参数定义
	reg signed [12:0] d_previous;
reg signed [21:0] d_used_q8;
wire signed [13:0] d_sum=$signed(err_bias)+$signed(d_previous);
wire signed [31:0] d_product=32'sd244*$signed(d_used_q8);
assign d_used_report=d_used_q8;
reg		signed  [12:0]	err_bias_r/*synthesis syn_keep=1*/;
	reg		signed  [12:0]	po_least/*synthesis syn_keep=1*/;
			
	reg		signed	[15:0]	theta_r/*synthesis syn_keep=1*/;


	wire	signed	[20:0]	duty_sum;


//信号定义
//5ms 计数器
always@(posedge clk_50m or negedge rst_n)begin
    if(!rst_n)begin
        cnt_5ms <= 0;
    end else if(cnt_5ms == 249999)begin
        cnt_5ms <= 0;
    end else
        cnt_5ms <= cnt_5ms +1;
end

//25ms 计数器
always@(posedge clk_50m or negedge rst_n)begin
    if(!rst_n)begin
        cnt_25ms <= 0;
    end else if(cnt_5ms == 249999 && cnt_25ms == 4 )begin
        cnt_25ms <= 0;
    end else if(cnt_5ms == 249999 && cnt_25ms <= 3)begin
        cnt_25ms <= cnt_25ms + 1;
    end else begin
        cnt_25ms <= cnt_25ms;
    end
end
//保存当前theta_r、encoder_r值,并符号化
always@(posedge clk_50m or negedge rst_n)begin
    if(!rst_n)begin
        theta_r <= 0;
    end else if(cnt_5ms == 1)begin
        theta_r <= {4'b0,theta};
    end else 
        theta_r <= theta_r;
end

assign theta_rr = theta_r + $signed(135);//$signed(130)

// 固定最高点3200；坐标为平均ADC乘4再加135
always@(posedge clk_50m or negedge rst_n)begin
    if(!rst_n)begin
        err <= 0;
    end else if(cnt_5ms == 2)begin
        //err <= $signed(3200) - theta_rr;
		err <= theta_rr - $signed(3200);
    end
end

//计算err的差值
always@(posedge clk_50m or negedge rst_n)begin
    if(!rst_n)begin
        err_bias_r <= 0;
        err_bias   <= 0;
    end else if(cnt_5ms == 0)begin
		err_bias_r <= err;
    end else if(cnt_5ms == 3)begin
        err_bias <= err - err_bias_r;
    end else begin
        err_bias_r <= err_bias_r;
        err_bias   <= err_bias;
    end
end

//倾角环计算公式
assign duty = -(32'sd50 * err) - (d_product >>> 8);

// Derivative-only two-interval FIR. P and LED use original undelayed angle.
always @(posedge clk_50m or negedge rst_n)begin
 if(!rst_n)begin d_previous<=0;d_used_q8<=0;end
 else begin
  if(cnt_5ms==0)d_previous<=err_bias;
  if(cnt_5ms==4)d_used_q8<=d_sum*22'sd128;
 end
end

//位置环计算
always@(posedge clk_50m or negedge rst_n)begin
    if(!rst_n || rebase_position)begin
        encoder_r <= 'd10000;	//0
	end else if(cnt_5ms == 1)begin
		encoder_r <= encoder;
    end else 
        encoder_r <= encoder_r;
end

always@(posedge clk_50m or negedge rst_n)begin
    if(!rst_n || rebase_position)begin
        po_least <= 0;
    end else if(cnt_5ms == 2 && cnt_25ms == 0)begin
		po_least <= $signed(encoder_r) - $signed(step_target);
    end else begin
        po_least <= po_least;
    end
end

reg signed  [12:0]  temp1;
reg signed  [12:0]  temp2;
 
 always@(posedge clk_50m or negedge rst_n) begin
    if(!rst_n || rebase_position)begin
        temp1 <= 0;
    end else if(cnt_5ms == 3 && cnt_25ms == 0) begin	//0.2
        //temp1 <= (po_least >>>3) + (po_least >>>4) + (po_least >>>6);
        temp1 <= (po_least * $signed(2) ) /$signed(10);
    end else 
        temp1 <= temp1;
 end

//assign temp2 = (po_least >>>3) + (po_least >>>4) + (po_least >>>6);
 always@(posedge clk_50m or negedge rst_n) begin
    if(!rst_n || rebase_position)begin
        temp2 <= 0;
    end else if(cnt_5ms == 3 && cnt_25ms == 0) begin	//0.8
       // temp2 <= po_err_bias_r - ((po_err_bias_r >>>3) + (po_err_bias_r >>>4) + (po_err_bias_r >>>6));  
        temp2 <= ((po_err_bias_r * $signed(8) ) / $signed(10)) ;
    end else 
        temp2 <= temp2;
 end

always@(posedge clk_50m or negedge rst_n)begin
    if(!rst_n || rebase_position)begin
        po_err_bias <= 0;
        po_err_bias_r <= 0;
    end else if(cnt_5ms == 0 && cnt_25ms == 0) begin
        po_err_bias_r <= po_err_bias;
    end else if(cnt_5ms == 4 && cnt_25ms == 0) begin
        po_err_bias <= temp1 + temp2;
    end else begin
        po_err_bias <= po_err_bias;
        po_err_bias_r <= po_err_bias_r;
    end
end

assign po_duty =  $signed(32'sd25) * po_err_bias + $signed(32'sd600) * (po_err_bias - po_err_bias_r);

assign duty_sum =  duty - po_duty;

reg	hand_start	;

always@(posedge clk_50m or negedge rst_n)begin
    if(!rst_n)begin
        duty_out <= 0;
	end else if(cnt_5ms == 5 && (theta_rr >= 3200 + 800 || theta_rr <= 3200 - 950 ))begin
        duty_out <= 0;
    end else if(cnt_5ms == 5 && $signed(duty_sum) >= $signed(7200)) begin
        duty_out <= $signed(6900);
    end else if(cnt_5ms == 5 && $signed(duty_sum) <= $signed(-7200)) begin
        duty_out <= $signed(-6900);
    end else if(cnt_5ms == 5 ) begin
		duty_out <= duty_sum;
    end else begin
		duty_out <= duty_out;
	end	
end





endmodule

