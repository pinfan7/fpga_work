`timescale 1ns/1ps
// 8x8 characters, 32 columns x 6 rows: 256x48 pixels at (8,8).
// Fixed three-clock RGB/HS/VS/DE pipeline, same latency as V10.
module imu_compact_overlay(
    input clk,rst_n,input [287:0] packet,
    input [23:0] rgb_in,input hs_in,vs_in,de_in,
    output reg [23:0] rgb_out,output reg hs_out,vs_out,de_out
);
    reg de_delayed,vs_delayed;
    reg [11:0] x_cnt,y_line;
    wire [11:0] sx=x_cnt-12'd8,sy=y_line-12'd8;
    wire panel_now=de_in && x_cnt>=8 && x_cnt<264 && y_line>=8 && y_line<56;
    wire [7:0] flags=packet[138:131],err=packet[114:107];
    wire data_fresh=flags[0] && !flags[6] && err==0;
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
    function [7:0] signed_char;
        input [16:0] value;input [2:0] pos;input decimal,valid;
        begin
            if(!valid)signed_char=(decimal && pos==4) ? "." : "-";
            else case(pos)
            0:signed_char=value[16] ? "-" : "+";
            1:signed_char=digit(value[15:12]);
            2:signed_char=digit(value[11:8]);
            3:signed_char=digit(value[7:4]);
            4:signed_char=decimal ? "." : digit(value[3:0]);
            default:signed_char=digit(value[3:0]);
            endcase
        end
    endfunction
    function [7:0] text_char;
        input [2:0] row;input [4:0] col;
        reg [255:0] line;
        reg [39:0] status;
        begin
            line="                                ";
            if(packet[155] || flags[5])status="FAULT";
            else if(flags[6])status="RETRY";
            else if(err!=0)status="FAULT";
            else if(!packet[156])status="PAUSE";
            else if(!flags[0])status="INIT ";
            else if(!flags[1] || !flags[2])status="CAL  ";
            else if(flags[3])status="TILT ";
            else if(!flags[4])status="DYNAM";
            else status="READY";
            case(row)
            0:line="V23 READY HZ:0000 ID:00 ERR:00  ";
            1:line="R:+000.0 P:+000.0 Y:+000.0 DEG  ";
            2:line="A:+0000 +0000 +0000 MG          ";
            3:line="G:+000.0 +000.0 +000.0 DPS      ";
            4:line="N:00000000 C:000/256 ADDR:68    ";
            5:line=(err!=0) ? "REG:00 DX:00 SCL:1 SDA:1 RT:00  " :
                              "S1 RUN/STOP YAW REL RT:00       ";
            endcase
            text_char=line[(31-col)*8+:8];
            if(row==0)begin
                if(col>=4 && col<=8)text_char=status[(8-col)*8+:8];
                if(col>=13 && col<=16)text_char=digit(packet[83+(16-col)*4+:4]);
                if(col==21)text_char=digit(packet[106:103]);
                if(col==22)text_char=digit(packet[102:99]);
                if(col==28)text_char=digit(err[7:4]);
                if(col==29)text_char=digit(err[3:0]);
            end
            if(row==1)begin
                if(col>=2 && col<=7)text_char=signed_char(packet[16:0],col-2,1,data_fresh&&flags[2]&&!flags[5]);
                if(col>=11 && col<=16)text_char=signed_char(packet[33:17],col-11,1,data_fresh&&flags[2]&&!flags[5]);
                if(col>=20 && col<=25)text_char=signed_char(packet[50:34],col-20,1,data_fresh&&flags[2]&&!flags[5]);
            end
            if(row==2)begin
                if(col>=2 && col<=6)text_char=signed_char(packet[176:160],col-2,0,data_fresh);
                if(col>=8 && col<=12)text_char=signed_char(packet[193:177],col-8,0,data_fresh);
                if(col>=14 && col<=18)text_char=signed_char(packet[210:194],col-14,0,data_fresh);
            end
            if(row==3)begin
                if(col>=2 && col<=7)text_char=signed_char(packet[227:211],col-2,1,data_fresh);
                if(col>=9 && col<=14)text_char=signed_char(packet[244:228],col-9,1,data_fresh);
                if(col>=16 && col<=21)text_char=signed_char(packet[261:245],col-16,1,data_fresh);
            end
            if(row==4)begin
                if(col>=2 && col<=9)text_char=digit(packet[51+(9-col)*4+:4]);
                if(col>=13 && col<=15)text_char=digit(packet[115+(15-col)*4+:4]);
                if(col==27)text_char=packet[159] ? "9" : "8";
            end
            if(row==5 && err!=0)begin
                case(col)
                4:text_char=digit(packet[154:151]);5:text_char=digit(packet[150:147]);
                10:text_char=digit(packet[146:143]);11:text_char=digit(packet[142:139]);
                17:text_char=packet[158] ? "1" : "0";
                23:text_char=packet[157] ? "1" : "0";
                endcase
            end
            if(row==5)begin
                if(col==(err!=0 ? 28 : 23))text_char=digit(packet[269:266]);
                if(col==(err!=0 ? 29 : 24))text_char=digit(packet[265:262]);
            end
        end
    endfunction
    function [7:0] font;
        input [7:0] code;input [2:0] row;reg [55:0] bits;
        begin
            case(code)
            "0":bits=56'h38444c54644438;"1":bits=56'h10301010101038;
            "2":bits=56'h3844040810207c;"3":bits=56'h78040438040478;
            "4":bits=56'h081828487c0808;"5":bits=56'h7c404078040478;
            "6":bits=56'h38404078444438;"7":bits=56'h7c040810202020;
            "8":bits=56'h38444438444438;"9":bits=56'h3844443c040438;
            "A":bits=56'h3844447c444444;"B":bits=56'h78444478444478;
            "C":bits=56'h3c40404040403c;"D":bits=56'h78444444444478;
            "E":bits=56'h7c40407840407c;"F":bits=56'h7c404078404040;
            "G":bits=56'h3c40405c44443c;"H":bits=56'h4444447c444444;
            "I":bits=56'h38101010101038;"J":bits=56'h04040404444438;
            "K":bits=56'h44485060504844;"L":bits=56'h4040404040407c;
            "M":bits=56'h446c5454444444;"N":bits=56'h4464544c444444;
            "O":bits=56'h38444444444438;"P":bits=56'h78444478404040;
            "Q":bits=56'h38444444544834;"R":bits=56'h78444478504844;
            "S":bits=56'h3c404038040478;"T":bits=56'h7c101010101010;
            "U":bits=56'h44444444444438;"V":bits=56'h44444444442810;
            "W":bits=56'h44444454546c44;"X":bits=56'h44442810284444;
            "Y":bits=56'h44442810101010;"Z":bits=56'h7c04081020407c;
            ":":bits=56'h00101000101000;".":bits=56'h00000000001010;
            "+":bits=56'h0010107c101000;"-":bits=56'h0000007c000000;
            "/":bits=56'h04080810202040;"|":bits=56'h10101010101010;
            default:bits=0;
            endcase
            font=row==7 ? 8'd0 : bits[(6-row)*8+:8];
        end
    endfunction
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            de_delayed<=0;vs_delayed<=0;x_cnt<=0;y_line<=0;
            code1<=32;row1<=0;col1<=0;panel1<=0;panel2<=0;ink2<=0;
            rgb1<=0;rgb2<=0;rgb_out<=0;hs1<=0;vs1<=0;de1<=0;
            hs2<=0;vs2<=0;de2<=0;hs_out<=0;vs_out<=0;de_out<=0;
        end else begin
            de_delayed<=de_in;vs_delayed<=vs_in;
            if(de_in)x_cnt<=x_cnt+1'b1;else x_cnt<=0;
            if(vs_in && !vs_delayed)y_line<=0;
            else if(!de_in && de_delayed)y_line<=y_line+1'b1;
            code1<=text_char(sy[5:3],sx[7:3]);row1<=sy[2:0];col1<=sx[2:0];
            panel1<=panel_now;rgb1<=rgb_in;hs1<=hs_in;vs1<=vs_in;de1<=de_in;
            ink2<=glyph_row[7-col1];panel2<=panel1;rgb2<=rgb1;hs2<=hs1;vs2<=vs1;de2<=de1;
            hs_out<=hs2;vs_out<=vs2;de_out<=de2;
            if(!de2)rgb_out<=0;
            else if(panel2 && ink2)rgb_out<=24'he0ffff;
            else if(panel2)rgb_out<={1'b0,rgb2[23:17],1'b0,rgb2[15:9],1'b0,rgb2[7:1]};
            else rgb_out<=rgb2;
        end
    end
endmodule
