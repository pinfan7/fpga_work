`timescale 1ns/1ps
// Rear-wheel axle midpoint. X: starting heading, Y: left. Position: signed Q8 mm.
// Fuse wheel distance with relative gyro heading, NOT accelerometer integration.
module nav_pose #(
 parameter integer ENC_CPR=1560,WHEEL_UM=65000,YAW_SIGN=1
)(
 input clk,rst_n,clear,track,
 input signed [31:0] yaw_q16,count_l,count_r,
 input valid_l,valid_r,
 output reg signed [31:0] x_q8,y_q8,
 output reg signed [31:0] heading_q16,
 output reg updated, fault
);
 localparam signed [31:0] HALF_MM_Q16=(64'd1*WHEEL_UM*3141593*65536+64'd1000000000*ENC_CPR)/(64'd2000000000*ENC_CPR);
 reg signed [31:0] yaw_zero;
 reg signed [32:0] yaw_diff;
 wire signed [32:0] yaw_wrap=yaw_diff>=33'sd11796480 ? yaw_diff-33'sd23592960 :
                                 yaw_diff< -33'sd11796480 ? yaw_diff+33'sd23592960:yaw_diff;
 function signed [15:0] sin_quadrant;
   input integer d;
   begin case(d)
       0: sin_quadrant=16'sd0;
       1: sin_quadrant=16'sd572;
       2: sin_quadrant=16'sd1144;
       3: sin_quadrant=16'sd1715;
       4: sin_quadrant=16'sd2286;
       5: sin_quadrant=16'sd2856;
       6: sin_quadrant=16'sd3425;
       7: sin_quadrant=16'sd3993;
       8: sin_quadrant=16'sd4560;
       9: sin_quadrant=16'sd5126;
       10: sin_quadrant=16'sd5690;
       11: sin_quadrant=16'sd6252;
       12: sin_quadrant=16'sd6813;
       13: sin_quadrant=16'sd7371;
       14: sin_quadrant=16'sd7927;
       15: sin_quadrant=16'sd8481;
       16: sin_quadrant=16'sd9032;
       17: sin_quadrant=16'sd9580;
       18: sin_quadrant=16'sd10126;
       19: sin_quadrant=16'sd10668;
       20: sin_quadrant=16'sd11207;
       21: sin_quadrant=16'sd11743;
       22: sin_quadrant=16'sd12275;
       23: sin_quadrant=16'sd12803;
       24: sin_quadrant=16'sd13328;
       25: sin_quadrant=16'sd13848;
       26: sin_quadrant=16'sd14364;
       27: sin_quadrant=16'sd14876;
       28: sin_quadrant=16'sd15383;
       29: sin_quadrant=16'sd15886;
       30: sin_quadrant=16'sd16383;
       31: sin_quadrant=16'sd16876;
       32: sin_quadrant=16'sd17364;
       33: sin_quadrant=16'sd17846;
       34: sin_quadrant=16'sd18323;
       35: sin_quadrant=16'sd18794;
       36: sin_quadrant=16'sd19260;
       37: sin_quadrant=16'sd19720;
       38: sin_quadrant=16'sd20173;
       39: sin_quadrant=16'sd20621;
       40: sin_quadrant=16'sd21062;
       41: sin_quadrant=16'sd21497;
       42: sin_quadrant=16'sd21925;
       43: sin_quadrant=16'sd22347;
       44: sin_quadrant=16'sd22762;
       45: sin_quadrant=16'sd23170;
       46: sin_quadrant=16'sd23571;
       47: sin_quadrant=16'sd23964;
       48: sin_quadrant=16'sd24351;
       49: sin_quadrant=16'sd24730;
       50: sin_quadrant=16'sd25101;
       51: sin_quadrant=16'sd25465;
       52: sin_quadrant=16'sd25821;
       53: sin_quadrant=16'sd26169;
       54: sin_quadrant=16'sd26509;
       55: sin_quadrant=16'sd26841;
       56: sin_quadrant=16'sd27165;
       57: sin_quadrant=16'sd27481;
       58: sin_quadrant=16'sd27788;
       59: sin_quadrant=16'sd28087;
       60: sin_quadrant=16'sd28377;
       61: sin_quadrant=16'sd28659;
       62: sin_quadrant=16'sd28932;
       63: sin_quadrant=16'sd29196;
       64: sin_quadrant=16'sd29451;
       65: sin_quadrant=16'sd29697;
       66: sin_quadrant=16'sd29934;
       67: sin_quadrant=16'sd30162;
       68: sin_quadrant=16'sd30381;
       69: sin_quadrant=16'sd30591;
       70: sin_quadrant=16'sd30791;
       71: sin_quadrant=16'sd30982;
       72: sin_quadrant=16'sd31163;
       73: sin_quadrant=16'sd31335;
       74: sin_quadrant=16'sd31498;
       75: sin_quadrant=16'sd31650;
       76: sin_quadrant=16'sd31794;
       77: sin_quadrant=16'sd31927;
       78: sin_quadrant=16'sd32051;
       79: sin_quadrant=16'sd32165;
       80: sin_quadrant=16'sd32269;
       81: sin_quadrant=16'sd32364;
       82: sin_quadrant=16'sd32448;
       83: sin_quadrant=16'sd32523;
       84: sin_quadrant=16'sd32587;
       85: sin_quadrant=16'sd32642;
       86: sin_quadrant=16'sd32687;
       87: sin_quadrant=16'sd32722;
       88: sin_quadrant=16'sd32747;
       89: sin_quadrant=16'sd32762;
       90: sin_quadrant=16'sd32767;
     default: sin_quadrant=0;
   endcase end
 endfunction
 reg [2:0] state;
 reg signed [15:0] steps,sin_h,cos_h;
 reg signed [8:0] degrees;
 reg [8:0] abs_degrees;
 reg [6:0] quadrant_degrees;
 reg negative_sin,negative_cos;
 reg signed [47:0] distance_q16;
 reg signed [63:0] prod_x,prod_y;
 wire event_count=track && (valid_l || valid_r);
 wire signed [32:0] sum_counts=(valid_l ? $signed({count_l[31],count_l}):33'sd0)+
                                      (valid_r ? $signed({count_r[31],count_r}):33'sd0);
 always @(posedge clk or negedge rst_n)begin
   if(!rst_n)begin
     yaw_zero<=0;yaw_diff<=0;heading_q16<=0;x_q8<=0;y_q8<=0;state<=0;updated<=0;fault<=0;
     steps<=0;sin_h<=0;cos_h<=32767;distance_q16<=0;prod_x<=0;prod_y<=0;
     degrees<=0;abs_degrees<=0;quadrant_degrees<=0;negative_sin<=0;negative_cos<=0;
   end else if(clear)begin
     yaw_zero<=yaw_q16;yaw_diff<=0;heading_q16<=0;x_q8<=0;y_q8<=0;state<=0;updated<=0;fault<=0;
   end else begin
     updated<=0;
     // Separate subtraction, wrapping, degree folding, ROM and products into
     // clocked stages. No 32-bit quadrant arithmetic followed by a ROM chain.
     yaw_diff<=$signed(yaw_q16)-$signed(yaw_zero);
     heading_q16<=YAW_SIGN==1 ? yaw_wrap[31:0] : -yaw_wrap[31:0];
     if(event_count && state!=0)fault<=1;
     case(state)
       0:if(event_count)begin
         if(sum_counts>2048 || sum_counts< -2048)fault<=1;
         else begin
           steps<=sum_counts[15:0];
           degrees<=$signed(heading_q16)/65536;
           state<=1;
         end
       end
       1:begin
         distance_q16<=steps*HALF_MM_Q16;
         abs_degrees<=degrees<0 ? -degrees:degrees;
         negative_sin<=degrees<0;
         negative_cos<=degrees>90 || degrees< -90;
         state<=2;
       end
       2:begin quadrant_degrees<=abs_degrees>90 ? 9'd180-abs_degrees:abs_degrees;state<=3;end
       3:begin
         sin_h<=negative_sin ? -sin_quadrant(quadrant_degrees):sin_quadrant(quadrant_degrees);
         cos_h<=negative_cos ? -sin_quadrant(90-quadrant_degrees):sin_quadrant(90-quadrant_degrees);
         state<=4;
       end
       4:begin prod_x<=distance_q16*cos_h;prod_y<=distance_q16*sin_h;state<=5;end
       5:begin
         x_q8<=x_q8+($signed(prod_x)>>>23);
         y_q8<=y_q8+($signed(prod_y)>>>23);
         updated<=1;state<=0;
       end
       default:begin fault<=1;state<=0;end
     endcase
   end
 end
endmodule
