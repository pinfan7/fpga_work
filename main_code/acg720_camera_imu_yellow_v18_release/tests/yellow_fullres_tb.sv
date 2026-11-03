`timescale 1ns/1ps
module yellow_fullres_tb;
    reg clk=0;always #20 clk=~clk;
    reg rst_n=0,enable=1,de=0,vs=0;reg [23:0] rgb=0;
    wire valid;wire [11:0] x0,y0,x1,y1;
    yellow_cup_detector dut(.clk(clk),.rst_n(rst_n),.enable(enable),.rgb(rgb),.de(de),.vs(vs),
        .box_valid(valid),.box_x0(x0),.box_y0(y0),.box_x1(x1),.box_y1(y1));
    integer x,y,cycles,maxcycles=0;
    always @(posedge clk)begin
        if(!rst_n || (vs && !dut.vd))cycles=0;
        else if(dut.state!=0)cycles=cycles+1;
        if(cycles>maxcycles)maxcycles=cycles;
    end
    task frame(input integer mode);begin
        @(negedge clk);vs=1;de=0;repeat(1600)@(negedge clk);vs=0;
        for(y=0;y<480;y=y+1)begin
            for(x=0;x<640;x=x+1)begin
                de=1;rgb=24'h202030;
                if(mode==1 && x>=240 && x<400 && y>=120 && y<384)rgb=24'he0d090;
                if(mode==2)rgb=24'he0d090;
                @(negedge clk);
            end
            de=0;rgb=0;repeat(160)@(negedge clk);
        end
        repeat(9000)@(negedge clk);
        if(dut.have_frame && dut.state!=0)$fatal(1,"analysis did not complete within frame");
    end endtask
    initial begin
        repeat(5)@(negedge clk);rst_n=1;
        repeat(5)frame(1);
        if(!valid || x0!=240 || x1!=399 || y0!=120 || y1!=383)$fatal(1,"640x480 bbox mismatch");
        repeat(4)frame(2);if(valid)$fatal(1,"full yellow background was accepted");
        repeat(3)frame(0);if(valid)$fatal(1,"empty frame retained box");
        $display("PASS yellow_fullres_tb: 640x480, cup, full-yellow rejection, empty; max analysis cycles=%0d",maxcycles);$finish;
    end
    initial begin #250000000;$fatal(1,"timeout");end
endmodule
