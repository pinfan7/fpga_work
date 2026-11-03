`timescale 1ns/1ps
// Three-clock aligned RGB/HS/VS/DE pipeline. 32 columns, 11 lines, 16x16 cells.
module imu_video_overlay(
    input clk,input rst_n,input [223:0] packet,
    input [23:0] rgb_in,input hs_in,vs_in,de_in,
    output reg [23:0] rgb_out,output reg hs_out,vs_out,de_out
);
    reg de_delayed,vs_delayed;
    reg [11:0] x_cnt,y_line; // 修复：独立X像素计数器，移除反馈环
    reg [15:0] frame_count;
    wire [11:0] px = x_cnt; // px直接使用稳定x_cnt
    
    wire [11:0] sx=px-12'd16, sy=y_line-12'd16;
    wire panel_now=de_in && px>=16 && px<528 && y_line>=16 && y_line<192;
    wire [16:0] dr=packet[16:0],dp=packet[33:17],dy=packet[50:34];
    wire [31:0] samples=packet[82:51];
    wire [15:0] hz=packet[98:83];
    wire [7:0] id=packet[106:99],err=packet[114:107],flags=packet[138:131];
    wire [15:0] cal=packet[130:115];
    reg [7:0] code1;
    reg [2:0] row1,col1;
    reg panel1,panel2,ink2;
    reg [23:0] rgb1,rgb2;
    reg hs1,vs1,de1,hs2,vs2,de2;
    wire [7:0] glyph_row=font(code1,row1);
    
    function [7:0] digit;
        input [3:0] n;
        begin digit=n<10 ? 8'd48+n : 8'd55+n;end
    endfunction
    
    function [7:0] text_char;
        input [3:0] row;
        input [4:0] col;
        reg [255:0] line;
        reg [16:0] angle_digits;
        reg [31:0] trace;
        begin
            line="                                ";angle_digits=0;trace=0;
            case(row)
            0:line="FPGA V9 I2C 25K MANUAL START    ";
            1:begin
                if(packet[155])line="STOPPED - SEE ERR / PINS        ";
                else if(!packet[156])line="PASSIVE - S1 START              ";
                else if(err!=0) line="IMU: ERROR - SEE ID/ERR         ";
                else if(flags[5]) line="IMU: COMPUTE OVERRUN            ";
                else if(!flags[0]) line="IMU: WAITING FOR DATA           ";
                else if(!flags[1]) line="CAL: 000/256 KEEP STILL         ";
                else if(!flags[2]) line="IMU: WAITING FOR ANGLES         ";
                else if(flags[3]) line="IMU: READY / TILT WARNING       ";
                else if(!flags[4]) line="IMU: READY / ACCEL DYNAMIC      ";
                else line="IMU: READY                      ";
            end
            2:begin line="ROLL :  +000.0 DEG              ";angle_digits=dr;end
            3:begin line="PITCH:  +000.0 DEG              ";angle_digits=dp;end
            4:begin line="YAW* :  +000.0 DEG              ";angle_digits=dy;end
            5:line="SCL:0 SDA:0 RUN:0 DRV:00        ";
            6:line="SAMPLE: 00000000                ";
            7:line="ID: 00  ERR: 00  S1 RUN/STOP    ";
            8:begin
                line="REG: 00 RX: 00 ADDR: 68         ";
            end
            9:begin line="68 ERR:-- TX:-- ACK:-           ";trace=packet[191:160];end
            10:begin line="69 ERR:-- TX:-- ACK:-           ";trace=packet[223:192];end
            endcase
            text_char=line[(31-col)*8+:8];
            if(row==1 && flags[0] && !flags[1] && err==0 && col>=5 && col<=7)
                text_char=digit(cal[(7-col)*4+:4]);
            if(row>=2 && row<=4 && col>=8 && col<=13) begin
                if(!flags[0] || !flags[2] || flags[5]) text_char=(col==12)?8'd46:8'd45;
                else case(col)
                    8:text_char=angle_digits[16]?8'd45:8'd43;
                    9:text_char=digit(angle_digits[15:12]);
                    10:text_char=digit(angle_digits[11:8]);
                    11:text_char=digit(angle_digits[7:4]);
                    12:text_char=8'd46;
                    13:text_char=digit(angle_digits[3:0]);
                endcase
            end
            if(row==5 && col==4)text_char=packet[158] ? "1" : "0";
            if(row==5 && col==10)text_char=packet[157] ? "1" : "0";
            if(row==5 && col==16)text_char=packet[156] ? "1" : "0";
            if(row==5 && col==22)text_char=flags[7] ? "1" : "0";
            if(row==5 && col==23)text_char=flags[6] ? "1" : "0";
            if(row==6 && col>=8 && col<=15) text_char=digit(samples[(15-col)*4+:4]);
            if(row==7 && col>=4 && col<=5) text_char=digit(id[(5-col)*4+:4]);
            if(row==7 && col>=13 && col<=14) text_char=digit(err[(14-col)*4+:4]);
            if(row==8) begin
                case(col)
                    5:text_char=digit(packet[154:151]);
                    6:text_char=digit(packet[150:147]);
                    12:text_char=digit(packet[146:143]);
                    13:text_char=digit(packet[142:139]);
                    21:text_char="6";
                    22:text_char=packet[159] ? "9" : "8";
                endcase
            end
            if((row==9 || row==10) && trace[31]) begin
                case(col)
                    7:text_char=digit(trace[23:20]);
                    8:text_char=digit(trace[19:16]);
                    13:text_char=digit(trace[15:12]);
                    14:text_char=digit(trace[11:8]);
                    20:text_char=digit(trace[7:4]);
                endcase
            end
        end
    endfunction
    
    function [7:0] font;
        input [7:0] code;
        input [2:0] row;
        reg [55:0] bits;
        begin
            case(code)
            8'd48: bits=56'h38444c54644438;
            8'd49: bits=56'h10301010101038;
            8'd50: bits=56'h3844040810207c;
            8'd51: bits=56'h78040438040478;
            8'd52: bits=56'h081828487c0808;
            8'd53: bits=56'h7c404078040478;
            8'd54: bits=56'h38404078444438;
            8'd55: bits=56'h7c040810202020;
            8'd56: bits=56'h38444438444438;
            8'd57: bits=56'h3844443c040438;
            8'd65: bits=56'h3844447c444444;
            8'd66: bits=56'h78444478444478;
            8'd67: bits=56'h3c40404040403c;
            8'd68: bits=56'h78444444444478;
            8'd69: bits=56'h7c40407840407c;
            8'd70: bits=56'h7c404078404040;
            8'd71: bits=56'h3c40405c44443c;
            8'd72: bits=56'h4444447c444444;
            8'd73: bits=56'h38101010101038;
            8'd74: bits=56'h04040404444438;
            8'd75: bits=56'h44485060504844;
            8'd76: bits=56'h4040404040407c;
            8'd77: bits=56'h446c5454444444;
            8'd78: bits=56'h4464544c444444;
            8'd79: bits=56'h38444444444438;
            8'd80: bits=56'h78444478404040;
            8'd81: bits=56'h38444444544834;
            8'd82: bits=56'h78444478504844;
            8'd83: bits=56'h3c404038040478;
            8'd84: bits=56'h7c101010101010;
            8'd85: bits=56'h44444444444438;
            8'd86: bits=56'h44444444442810;
            8'd87: bits=56'h44444454546c44;
            8'd88: bits=56'h44442810284444;
            8'd89: bits=56'h44442810101010;
            8'd90: bits=56'h7c04081020407c;
            8'd58: bits=56'h00101000101000;
            8'd46: bits=56'h00000000001010;
            8'd43: bits=56'h0010107c101000;
            8'd45: bits=56'h0000007c000000;
            8'd47: bits=56'h04080810202040;
            8'd42: bits=56'h0054387c385400;
            default:bits=0;
            endcase
            font=row==7 ? 8'd0 : bits[(6-row)*8+:8];
        end
    endfunction
    
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            de_delayed<=0;vs_delayed<=0;
            x_cnt<=0;y_line<=0;frame_count<=0;
            code1<=32;row1<=0;col1<=0;panel1<=0;panel2<=0;ink2<=0;
            rgb1<=0;rgb2<=0;rgb_out<=0;hs1<=0;vs1<=0;de1<=0;
            hs2<=0;vs2<=0;de2<=0;hs_out<=0;vs_out<=0;de_out<=0;
        end else begin
            de_delayed<=de_in;
            vs_delayed<=vs_in;

            // ==========修复：独立X计数器==========
            if(de_in) begin
                x_cnt <= x_cnt + 12'd1;
            end else begin
                x_cnt <= 12'd0;
            end

            // Y行计数保持原有逻辑不变
            if(vs_in && !vs_delayed) begin 
                y_line <= 12'd0;
                frame_count <= frame_count + 1'b1;
            end
            else if(!de_in && de_delayed) begin 
                y_line <= y_line + 12'd1;
            end

            code1<=text_char(sy[7:4],sx[8:4]);
            row1<=sy[3:1];
            col1<=sx[3:1];
            
            panel1<=panel_now;
            rgb1<=rgb_in;
            hs1<=hs_in;
            vs1<=vs_in;
            de1<=de_in;
            
            ink2<=glyph_row[7-col1];
            panel2<=panel1;
            rgb2<=rgb1;
            hs2<=hs1;
            vs2<=vs1;
            de2<=de1;
            
            hs_out<=hs2;
            vs_out<=vs2;
            de_out<=de2;
            
            // 输出逻辑保持原样
            if(!de2) begin
                rgb_out<=24'd0;
            end
            else if(panel2 && ink2) begin
                rgb_out<=24'he0ffff; //白色文字
            end
            else if(panel2) begin
                rgb_out<={2'b00,rgb2[23:18],2'b00,rgb2[15:10],2'b00,rgb2[7:2]}; //OSD区域画面变暗
            end
            else begin
                rgb_out<=rgb2; //正常摄像头画面
            end
        end
    end
endmodule
