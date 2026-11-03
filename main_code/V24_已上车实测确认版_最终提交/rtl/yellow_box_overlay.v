`timescale 1ns/1ps
// One-clock output, RGB and all sync signals delayed together.
module yellow_box_overlay(input clk,rst_n,input [23:0] rgb_in,
    input hs_in,vs_in,de_in,box_valid,input [11:0] x0,y0,x1,y1,
    output reg [23:0] rgb_out,output reg hs_out,vs_out,de_out);
    reg [11:0] x,y;reg vd,dd;
    reg valid;reg [11:0] bx0,by0,bx1,by1;
    wire [12:0] center_x_sum={1'b0,bx0}+bx1,center_y_sum={1'b0,by0}+by1;
    wire [11:0] mx=center_x_sum[12:1],my=center_y_sum[12:1];
    wire edge_pixel=valid && x>=bx0 && x<=bx1 && y>=by0 && y<=by1 &&
        (x<bx0+2 || x+2>bx1 || y<by0+2 || y+2>by1);
    wire cross_pixel=valid && ((x==mx && y+5>=my && y<=my+5)||
                                              (y==my && x+5>=mx && x<=mx+5));
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            x<=0;y<=0;vd<=0;dd<=0;valid<=0;bx0<=0;by0<=0;bx1<=0;by1<=0;
            rgb_out<=0;hs_out<=0;vs_out<=0;de_out<=0;
        end else begin
            vd<=vs_in;dd<=de_in;
            if(de_in)x<=x+1'b1;else x<=0;
            if(vs_in && !vd)begin
                y<=0;valid<=box_valid;bx0<=x0;by0<=y0;bx1<=x1;by1<=y1;
            end else if(!de_in && dd)y<=y+1'b1;
            hs_out<=hs_in;vs_out<=vs_in;de_out<=de_in;
            if(!de_in)rgb_out<=0;
            // Protect the existing top-left IMU panel from box paint.
            else if(x>=8 && x<264 && y>=8 && y<56)rgb_out<=rgb_in;
            else if(edge_pixel || cross_pixel)rgb_out<=24'h00ff00;
            else rgb_out<=rgb_in;
        end
    end
endmodule
