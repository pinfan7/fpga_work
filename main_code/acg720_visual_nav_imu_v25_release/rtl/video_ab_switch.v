`timescale 1ns/1ps
// In-place A/B: identical clocks, placement, source pipeline latency and bitstream.
// S2 toggles colorbars/camera. S3 toggles OSD. Changes commit in frame blanking.
module video_ab_switch #(parameter integer DEBOUNCE_CYCLES=1500000,
    parameter integer ACTIVE_WIDTH=1280, parameter DEFAULT_OSD=1'b0)(
    input clk,rst_n,source_key_n,osd_key_n,
    input [23:0] camera_rgb,
    input hs,vs,de,
    output [23:0] overlay_input_rgb,
    input [23:0] overlay_rgb,input overlay_hs,overlay_vs,overlay_de,
    output [23:0] tx_rgb,output tx_hs,tx_vs,tx_de,
    output reg pattern_mode,osd_mode
);
    wire source_press,osd_press;
    video_key_press #(.CYCLES(DEBOUNCE_CYCLES)) ks(.clk(clk),.rst_n(rst_n),.key_n(source_key_n),.pressed(source_press));
    video_key_press #(.CYCLES(DEBOUNCE_CYCLES)) ko(.clk(clk),.rst_n(rst_n),.key_n(osd_key_n),.pressed(osd_press));
    reg pattern_request,osd_request,vs_d,de_d;
    reg [11:0] x,y;
    reg [23:0] pattern_rgb;
    reg [23:0] rgb0,rgb1,rgb2;
    reg [2:0] hpipe,vpipe,dpipe;
    always @* begin
        if(!de)pattern_rgb=0;
        else if(x<ACTIVE_WIDTH/8)pattern_rgb=24'hffffff;
        else if(x<ACTIVE_WIDTH*2/8)pattern_rgb=24'hffff00;
        else if(x<ACTIVE_WIDTH*3/8)pattern_rgb=24'h00ffff;
        else if(x<ACTIVE_WIDTH*4/8)pattern_rgb=24'h00ff00;
        else if(x<ACTIVE_WIDTH*5/8)pattern_rgb=24'hff00ff;
        else if(x<ACTIVE_WIDTH*6/8)pattern_rgb=24'hff0000;
        else if(x<ACTIVE_WIDTH*7/8)pattern_rgb=24'h0000ff;
        else pattern_rgb=24'h202020;
    end
    assign overlay_input_rgb=pattern_mode?pattern_rgb:camera_rgb;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            pattern_request<=1;osd_request<=DEFAULT_OSD;pattern_mode<=1;osd_mode<=DEFAULT_OSD;
            vs_d<=0;de_d<=0;x<=0;y<=0;
            rgb0<=0;rgb1<=0;rgb2<=0;hpipe<=0;vpipe<=0;dpipe<=0;
        end else begin
            vs_d<=vs;de_d<=de;
            if(source_press)pattern_request<=~pattern_request;
            if(osd_press)osd_request<=~osd_request;
            if(vs && !vs_d)begin
                pattern_mode<=pattern_request;osd_mode<=osd_request;y<=0;
            end else if(!de && de_d)y<=y+1'b1;
            if(de)x<=x+1'b1;else x<=0;
            rgb0<=overlay_input_rgb;rgb1<=rgb0;rgb2<=rgb1;
            hpipe<={hpipe[1:0],hs};vpipe<={vpipe[1:0],vs};dpipe<={dpipe[1:0],de};
        end
    end
    assign tx_rgb=osd_mode?overlay_rgb:rgb2;
    assign tx_hs=osd_mode?overlay_hs:hpipe[2];
    assign tx_vs=osd_mode?overlay_vs:vpipe[2];
    assign tx_de=osd_mode?overlay_de:dpipe[2];
endmodule

module video_key_press #(parameter integer CYCLES=1500000)(
    input clk,rst_n,key_n,output reg pressed
);
    localparam integer W=(CYCLES<2)?1:$clog2(CYCLES);
    (* syn_preserve=1 *) reg meta,sync;
    reg stable;
    reg [W-1:0] count;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin meta<=1;sync<=1;stable<=1;count<=0;pressed<=0;end
        else begin
            meta<=key_n;sync<=meta;pressed<=0;
            if(sync==stable)count<=0;
            else if(count==CYCLES-1)begin
                count<=0;stable<=sync;if(!sync)pressed<=1;
            end else count<=count+1'b1;
        end
    end
endmodule
