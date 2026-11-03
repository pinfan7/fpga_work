`timescale 1ns/1ps
// Three-clock RGB/HS/VS/DE alignment retained from working V9 video overlay.
// 32 columns x 11 rows, 16x16 cells, screen x16..527 / y16..191.
// Record fields: address cols0..1, ID75 cols6..7, ID0F cols12..13,
// flags/READ ACK cols22..23. Row3: scan5..6, found14..15, error21..22.
// All numeric fields are hexadecimal; absent/failed reads show "--".
module imu_discovery_overlay(
    input clk,input rst_n,input [159:0] packet,
    input [23:0] rgb_in,input hs_in,vs_in,de_in,
    output reg [23:0] rgb_out,output reg hs_out,vs_out,de_out
);
    reg de_delayed,vs_delayed;
    reg [11:0] x_cnt,y_line;
    wire [11:0] sx=x_cnt-12'd16,sy=y_line-12'd16;
    wire panel_now=de_in && x_cnt>=16 && x_cnt<528 && y_line>=16 && y_line<192;
    reg [7:0] code1;
    reg [2:0] row1,col1;
    reg panel1,panel2,ink2;
    reg [23:0] rgb1,rgb2;
    reg hs1,vs1,de1,hs2,vs2,de2;
    wire [7:0] glyph_row=font(code1,row1);
    wire [7:0] error_code=packet[156:149];
    wire [7:0] found_count=packet[148:141];
    wire [7:0] scan_address=packet[140:133];

    function [7:0] digit;
        input [3:0] n;
        begin digit=n<10 ? 8'd48+n : 8'd55+n;end
    endfunction

    function [7:0] text_char;
        input [3:0] row;
        input [4:0] col;
        reg [255:0] line;
        reg [31:0] record;
        begin
            line="                                ";record=0;
            case(row)
            0:line="FPGA V10 VENDOR I2C SCAN        ";
            1:begin
                if(packet[158] && found_count==0 && error_code==8'h21)
                    line="DONE NO ADDRESS ACK             ";
                else if(error_code!=0)line="STOP BUS ERROR                  ";
                else if(packet[159])line="SCANNING                        ";
                else if(packet[158])begin
                    if(packet[157])line="DONE ICM ID47 FOUND             ";
                    else if(found_count==0)line="DONE NO ADDRESS ACK             ";
                    else line="DONE OTHER ID SEE TABLE         ";
                end else line="PASSIVE S1 SCAN                 ";
            end
            2:line="SCL:0 SDA:0 RUN:0               ";
            3:line="SCAN:00 FOUND:00 ERR:00         ";
            4:line="ADDR  ID75  ID0F  READ ACK      ";
            5,6,7,8:begin
                line="--    --    --        --        ";
                case(row)
                    5:record=packet[31:0];
                    6:record=packet[63:32];
                    7:record=packet[95:64];
                    8:record=packet[127:96];
                endcase
            end
            9:line="I2C 08-77 / READ ONLY           ";
            10:line="S1 RETRY / S2 VIDEO / S3 OSD    ";
            endcase
            text_char=line[(31-col)*8+:8];
            if(row==2)case(col)
                4:text_char=packet[132] ? "1":"0";
                10:text_char=packet[131] ? "1":"0";
                16:text_char=packet[130] ? "1":"0";
            endcase
            if(row==3)case(col)
                5:text_char=digit(scan_address[7:4]);
                6:text_char=digit(scan_address[3:0]);
                14:text_char=digit(found_count[7:4]);
                15:text_char=digit(found_count[3:0]);
                21:text_char=digit(error_code[7:4]);
                22:text_char=digit(error_code[3:0]);
            endcase
            if(row>=5 && row<=8 && record[0])begin
                case(col)
                    0:text_char=digit(record[31:28]);
                    1:text_char=digit(record[27:24]);
                    6:if(record[1])text_char=digit(record[23:20]);
                    7:if(record[1])text_char=digit(record[19:16]);
                    12:if(record[2])text_char=digit(record[15:12]);
                    13:if(record[2])text_char=digit(record[11:8]);
                    22:text_char=digit(record[7:4]);
                    23:text_char=digit(record[3:0]);
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
        if(!rst_n)begin
            de_delayed<=0;vs_delayed<=0;x_cnt<=0;y_line<=0;
            code1<=32;row1<=0;col1<=0;panel1<=0;panel2<=0;ink2<=0;
            rgb1<=0;rgb2<=0;rgb_out<=0;hs1<=0;vs1<=0;de1<=0;
            hs2<=0;vs2<=0;de2<=0;hs_out<=0;vs_out<=0;de_out<=0;
        end else begin
            de_delayed<=de_in;vs_delayed<=vs_in;
            if(de_in)x_cnt<=x_cnt+12'd1;else x_cnt<=0;
            if(vs_in && !vs_delayed)y_line<=0;
            else if(!de_in && de_delayed)y_line<=y_line+12'd1;
            code1<=text_char(sy[7:4],sx[8:4]);row1<=sy[3:1];col1<=sx[3:1];
            panel1<=panel_now;rgb1<=rgb_in;hs1<=hs_in;vs1<=vs_in;de1<=de_in;
            ink2<=glyph_row[7-col1];panel2<=panel1;
            rgb2<=rgb1;hs2<=hs1;vs2<=vs1;de2<=de1;
            hs_out<=hs2;vs_out<=vs2;de_out<=de2;
            if(!de2)rgb_out<=24'd0;
            else if(panel2 && ink2)rgb_out<=24'he0ffff;
            else if(panel2)rgb_out<={2'b00,rgb2[23:18],2'b00,rgb2[15:10],2'b00,rgb2[7:2]};
            else rgb_out<=rgb2;
        end
    end
endmodule
