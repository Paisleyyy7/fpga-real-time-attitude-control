module serial_link(input clk,rst_n,rx,input sample,input [767:0] payload,
 output tx,output reg cmd_valid,output reg [7:0] cmd,cmd_seq,output reg [15:0] value);
 function [15:0] crc_byte;
 input [15:0] c;input [7:0] b;reg [15:0] x;integer j;
 begin x=c^{b,8'b0};for(j=0;j<8;j=j+1)x=x[15]?(x<<1)^16'h1021:(x<<1);crc_byte=x;end
 endfunction
 wire [7:0] rxdata;wire rxvalid;
 uart_rx #(.UART_BPS(230400),.CLK_FREQ(50000000)) receiver(clk,rst_n,rx,rxdata,rxvalid);
 reg [3:0] ri;reg [15:0] rcrc;reg [7:0] lowcrc,rseq,rcmd;reg [15:0] rval;
 reg [19:0] timeout;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin ri<=0;rcrc<=16'hffff;cmd_valid<=0;cmd<=0;cmd_seq<=0;value<=0;timeout<=0;lowcrc<=0;rseq<=0;rcmd<=0;rval<=0;end
  else begin
   cmd_valid<=0;
   if(ri!=0)begin if(timeout==499999)begin ri<=0;timeout<=0;end else timeout<=timeout+1'b1;end
   if(rxvalid)begin
    timeout<=0;
    case(ri)
     0: if(rxdata==8'ha5)begin ri<=1;rcrc<=crc_byte(16'hffff,rxdata);end
     1: if(rxdata==8'h5a)begin ri<=2;rcrc<=crc_byte(rcrc,rxdata);end else ri<=0;
     2: if(rxdata==8'hc1)begin ri<=3;rcrc<=crc_byte(rcrc,rxdata);end else ri<=0;
     3:begin rseq<=rxdata;ri<=4;rcrc<=crc_byte(rcrc,rxdata);end
     4:begin rcmd<=rxdata;ri<=5;rcrc<=crc_byte(rcrc,rxdata);end
     5:begin rval[7:0]<=rxdata;ri<=6;rcrc<=crc_byte(rcrc,rxdata);end
     6:begin rval[15:8]<=rxdata;ri<=7;rcrc<=crc_byte(rcrc,rxdata);end
     7:if(rxdata==0)begin ri<=8;rcrc<=crc_byte(rcrc,rxdata);end else ri<=0;
     8:begin lowcrc<=rxdata;ri<=9;end
     9:begin ri<=0;if({rxdata,lowcrc}==rcrc)begin cmd<=rcmd;cmd_seq<=rseq;value<=rval;cmd_valid<=1;end end
     default:ri<=0;
    endcase
   end
  end
 end
 reg [767:0] snapshot;reg [6:0] index;reg [15:0] crc;
 reg sending,launch;reg [7:0] data;wire busy;
 uart_byte_tx sender(clk,rst_n,launch,data,tx,busy);
 wire [7:0] selected=snapshot[index*8+:8];
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin snapshot<=0;index<=0;crc<=16'hffff;sending<=0;launch<=0;data<=0;end
  else begin
   launch<=0;
   if(sample && !sending && !busy)begin snapshot<=payload;index<=0;crc<=16'hffff;sending<=1;end
   if(sending && !busy && !launch)begin
    launch<=1;
    if(index<96)begin data<=selected;crc<=crc_byte(crc,selected);end
    else if(index==96)data<=crc[7:0];else data<=crc[15:8];
    if(index==97)sending<=0;else index<=index+1'b1;
   end
  end
 end
endmodule

module uart_byte_tx(input clk,rst_n,start,input [7:0] data,output reg tx,output reg busy);
 reg [8:0] baud;reg [3:0] bitno;reg [9:0] bits;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin baud<=0;bitno<=0;bits<=10'h3ff;tx<=1;busy<=0;end
  else if(!busy)begin
   if(start)begin bits<={1'b1,data,1'b0};tx<=0;busy<=1;baud<=0;bitno<=0;end
  end else if(baud==216)begin
   baud<=0;
   if(bitno==9)begin busy<=0;tx<=1;end
   else begin bitno<=bitno+1'b1;bits<={1'b1,bits[9:1]};tx<=bits[1];end
  end else baud<=baud+1'b1;
 end
endmodule
