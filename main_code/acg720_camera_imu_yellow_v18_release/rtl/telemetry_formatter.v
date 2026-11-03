`timescale 1ns/1ps
// Format a coherent telemetry snapshot every 50ms in the 50MHz domain.
// Iterative binary-to-BCD conversion avoids decimal division on pixel paths.
module telemetry_formatter #(parameter integer PERIOD_CYCLES=2500000)(
    input clk,input rst_n,
    input signed [31:0] roll,pitch,yaw,
    input [31:0] sample_count,input [15:0] sample_hz,
    input [7:0] who_am_i,error_code,input [8:0] cal_count,
    input [7:0] flags,input [7:0] retry_total,
    input signed [15:0] ax,ay,az,gx,gy,gz,
    input [20:0] diagnostic,
    output reg [287:0] packet,output reg packet_valid
);
    reg [31:0] timer;
    reg [2:0] state;
    reg [3:0] index;
    reg [4:0] bit_index;
    reg [15:0] binary,bcd;
    reg [15:0] roll_tenth,pitch_tenth,yaw_tenth,hz_latch,cal_latch;
    reg sign_r,sign_p,sign_y;
    reg [31:0] count_latch;
    reg [7:0] id_latch,error_latch,flags_latch,retry_latch;
    reg [287:0] working;
    reg [16:0] axes[0:5];
    reg [20:0] diag_latch;
    reg [15:0] adjusted;
    integer k;
    function [15:0] tenth;
        input signed [31:0] angle;
        reg signed [63:0] magnitude;
        begin
            magnitude=angle<0 ? -$signed(angle) : $signed(angle);
            magnitude=(magnitude*64'sd10+64'sd32768)>>>16;
            tenth=magnitude>9999 ? 16'd9999 : magnitude[15:0];
        end
    endfunction
    // Four decimal digits plus sign. Acceleration is mg; rate is 0.1 deg/s.
    // +/-4g: 8192 LSB/g. +/-500 dps: 65.5 LSB/(deg/s).
    // 10005/65536 approximates 10/65.5 with <0.005% scale error.
    function [16:0] axis_units;
        input signed [15:0] raw;
        input gyro;
        reg [16:0] magnitude;
        reg [31:0] product,value;
        begin
            magnitude=raw[15] ? -$signed({raw[15],raw}) : {1'b0,raw};
            if(gyro)begin product=magnitude*32'd10005+32'd32768;value=product>>16;end
            else begin product=magnitude*32'd125+32'd512;value=product>>10;end
            axis_units={(raw[15] && value!=0),(value>9999 ? 16'd9999 : value[15:0])};
        end
    endfunction
    always @* begin
        adjusted=bcd;
        for(k=0;k<4;k=k+1)
            if(bcd[k*4+:4]>=5) adjusted[k*4+:4]=bcd[k*4+:4]+4'd3;
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            timer<=0;state<=0;index<=0;bit_index<=0;binary<=0;bcd<=0;
            roll_tenth<=0;pitch_tenth<=0;yaw_tenth<=0;hz_latch<=0;cal_latch<=0;
            sign_r<=0;sign_p<=0;sign_y<=0;count_latch<=0;id_latch<=0;error_latch<=0;flags_latch<=0;
            working<=0;packet<=0;packet_valid<=0;
            diag_latch<=0;retry_latch<=0;
            axes[0]<=0;axes[1]<=0;axes[2]<=0;axes[3]<=0;axes[4]<=0;axes[5]<=0;
        end else begin
            packet_valid<=0;
            if(timer==PERIOD_CYCLES-1) timer<=0;else timer<=timer+1'b1;
            case(state)
            0: if(timer==0) begin
                roll_tenth<=tenth(roll);pitch_tenth<=tenth(pitch);yaw_tenth<=tenth(yaw);
                sign_r<=roll<0;sign_p<=pitch<0;sign_y<=yaw<0;
                count_latch<=sample_count;hz_latch<=sample_hz>9999 ? 16'd9999 : sample_hz;
                cal_latch<={7'd0,cal_count};id_latch<=who_am_i;error_latch<=error_code;flags_latch<=flags;
                diag_latch<=diagnostic;retry_latch<=retry_total;
                axes[0]<=axis_units(ax,0);axes[1]<=axis_units(ay,0);axes[2]<=axis_units(az,0);
                axes[3]<=axis_units(gx,1);axes[4]<=axis_units(gy,1);axes[5]<=axis_units(gz,1);
                working<=0;index<=0;state<=1;
            end
            1: begin
                case(index)
                0:binary<=roll_tenth;1:binary<=pitch_tenth;2:binary<=yaw_tenth;
                3:binary<=hz_latch;4:binary<=cal_latch;
                default:binary<=axes[index-5][15:0];
                endcase
                bcd<=0;bit_index<=0;state<=2;
            end
            2: begin
                bcd<={adjusted[14:0],binary[15]};binary<={binary[14:0],1'b0};
                if(bit_index==15) state<=3;else bit_index<=bit_index+1'b1;
            end
            3: begin
                case(index)
                0:working[16:0]<={sign_r,bcd};1:working[33:17]<={sign_p,bcd};
                2:working[50:34]<={sign_y,bcd};3:working[98:83]<=bcd;
                4:working[130:115]<=bcd;
                default:working[160+(index-5)*17+:17]<={axes[index-5][16],bcd};
                endcase
                if(index==10) state<=4;else begin index<=index+1'b1;state<=1;end
            end
            4: begin
                working[82:51]<=count_latch;working[106:99]<=id_latch;
                working[114:107]<=error_latch;working[138:131]<=flags_latch;state<=5;
                working[159:139]<=diag_latch;working[269:262]<=retry_latch;
            end
            5: begin packet<=working;packet_valid<=1;state<=0;end
            default:state<=0;
            endcase
        end
    end
endmodule
