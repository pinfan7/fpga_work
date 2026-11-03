`timescale 1ns/1ps
module yellow_cup_tb;
    reg clk=0;always #20 clk=~clk;
    reg rst_n=0,enable=1,de=0,vs=0;reg [23:0] rgb=0;
    wire valid;wire [11:0] x0,y0,x1,y1;
    yellow_cup_detector #(.WIDTH(160),.HEIGHT(128)) dut(
        .clk(clk),.rst_n(rst_n),.enable(enable),.rgb(rgb),.de(de),.vs(vs),
        .box_valid(valid),.box_x0(x0),.box_y0(y0),.box_x1(x1),.box_y1(y1));
    integer xx,yy,k;
    task frame(input integer mode);begin
        @(negedge clk);vs=1;de=0;repeat(4)@(negedge clk);vs=0;
        for(yy=0;yy<128;yy=yy+1)begin
            for(xx=0;xx<160;xx=xx+1)begin
                de=1;rgb=24'h202030;
                // White and skin are not the yellow cup.
                if(xx<24)rgb=24'he0e0e0;
                if(xx>=88 && xx<112)rgb=24'hd09c80;
                if(mode==1 && xx>=40 && xx<80 && yy>=24 && yy<96)rgb=24'he0d090;
                if(mode==2 && xx>=0 && xx<32 && yy>=64 && yy<120)rgb=24'he0d090;
                if(mode==3 && xx>=43 && xx<79 && yy>=27 && yy<94)rgb=24'he0d090;
                // A near-white lid that passed V17's absolute RGB thresholds.
                if(mode==3 && xx>=43 && xx<79 && yy>=10 && yy<27)rgb=24'he8e0ca;
                // Disconnected small yellow patch and isolated speckles.
                if(xx>=120 && xx<136 && yy>=8 && yy<16)rgb=24'hf0e080;
                if(xx%19==0 && yy%13==0)rgb=24'hffff00;
                @(negedge clk);
            end
            de=0;rgb=0;repeat(12)@(negedge clk);
        end
        repeat(15000)@(negedge clk);
    end endtask
    task expect_box(input integer ax,input integer ay,input integer bx,input integer by);begin
        if(valid!==1 || x0!==ax || y0!==ay || x1!==bx || y1!==by)begin
            $display("FAIL box valid=%b (%d,%d)-(%d,%d) expected (%d,%d)-(%d,%d)",valid,x0,y0,x1,y1,ax,ay,bx,by);
            $fatal(1);
        end
    end endtask
    initial begin
        repeat(5)@(negedge clk);rst_n=1;
        repeat(5)frame(1);expect_box(40,24,79,95);
        // New component touching the left image boundary.
        repeat(5)frame(2);expect_box(0,64,31,119);
        repeat(3)frame(0);if(valid!==0)$fatal(1,"box did not disappear");
        enable=0;repeat(4)frame(1);if(valid!==0)$fatal(1,"test pattern enabled detector");
        enable=1;repeat(5)frame(1);expect_box(40,24,79,95);
        repeat(5)frame(3);expect_box(43,27,78,93);
        $display("PASS yellow_cup_tb: filtering, components, border, no target, disable/reacquire");$finish;
    end
    initial begin #100000000;$fatal(1,"timeout");end
endmodule
