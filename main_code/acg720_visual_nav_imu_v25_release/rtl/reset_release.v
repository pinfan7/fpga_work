`timescale 1ns/1ps
module reset_release(input clk,arst_n,output rst_n);
    (* syn_preserve=1 *) reg [3:0] pipe=0;
    always @(posedge clk or negedge arst_n)
        if(!arst_n)pipe<=0;else pipe<={pipe[2:0],1'b1};
    assign rst_n=pipe[3];
endmodule
