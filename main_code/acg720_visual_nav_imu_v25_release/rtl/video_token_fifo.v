`timescale 1ns/1ps
// Atomic {HS,VS,DE,RGB} asynchronous FIFO. Gray pointers cross two FF stages.
// Both resets must assert together; release is synchronized in each clock domain.
// Memory is written before its Gray pointer becomes visible to the reader.
module video_token_fifo #(
    parameter integer WIDTH=27, ADDR_BITS=5
)(
    input wclk,wrst_n,wen,input [WIDTH-1:0] wdata,
    output reg full,output reg overflow,
    input rclk,rrst_n,ren,output reg [WIDTH-1:0] rdata,
    output empty,output [ADDR_BITS:0] available,
    output reg underflow
);
    reg [WIDTH-1:0] memory[0:(1<<ADDR_BITS)-1];
    reg [ADDR_BITS:0] wbin,wgray,rbin,rgray;
    (* syn_preserve=1 *) reg [ADDR_BITS:0] rg_meta,rg_sync,wg_meta,wg_sync;
    wire [ADDR_BITS:0] wnext=wbin+(wen && !full);
    wire [ADDR_BITS:0] wgray_next=(wnext>>1)^wnext;
    wire [ADDR_BITS:0] rnext=rbin+(ren && !empty);
    wire [ADDR_BITS:0] rgray_next=(rnext>>1)^rnext;
    wire full_next=(wgray_next=={~rg_sync[ADDR_BITS:ADDR_BITS-1],rg_sync[ADDR_BITS-2:0]});
    assign empty=(rgray==wg_sync);
    function [ADDR_BITS:0] gray_to_binary;
        input [ADDR_BITS:0] gray;
        integer i;
        begin
            gray_to_binary[ADDR_BITS]=gray[ADDR_BITS];
            for(i=ADDR_BITS-1;i>=0;i=i-1)
                gray_to_binary[i]=gray_to_binary[i+1]^gray[i];
        end
    endfunction
    assign available=gray_to_binary(wg_sync)-rbin;
    always @(posedge wclk or negedge wrst_n) begin
        if(!wrst_n)begin wbin<=0;wgray<=0;rg_meta<=0;rg_sync<=0;full<=0;overflow<=0;end
        else begin
            rg_meta<=rgray;rg_sync<=rg_meta;
            wbin<=wnext;wgray<=wgray_next;full<=full_next;
            if(wen && full)overflow<=1;
        end
    end
    // No reset on memory array: data is accessible only behind synchronized pointers.
    always @(posedge wclk)if(wrst_n && wen && !full)memory[wbin[ADDR_BITS-1:0]]<=wdata;
    always @(posedge rclk or negedge rrst_n)begin
        if(!rrst_n)begin rbin<=0;rgray<=0;wg_meta<=0;wg_sync<=0;rdata<=0;underflow<=0;end
        else begin
            wg_meta<=wgray;wg_sync<=wg_meta;rbin<=rnext;rgray<=rgray_next;
            if(ren && !empty)rdata<=memory[rbin[ADDR_BITS-1:0]];
            if(ren && empty)underflow<=1;
        end
    end
endmodule
