`timescale 1ns/1ps
// RGB888 threshold -> 3-pixel run filter -> 8x8 tile occupancy ->
// four-connected components -> size/aspect/fill rejection -> 2-frame confirmation.
// Two tile banks allow analysis during the next frame; no camera CDC is added.
module yellow_cup_detector #(
    parameter integer WIDTH=640, HEIGHT=480,
    parameter integer R_MIN=120,G_MIN=110,RB_MIN=24,GB_MIN=20,
    parameter integer RG_MAX=40,GR_MAX=20,TILE_MIN=12,
    parameter integer MIN_TILES=12,MIN_W=3,MIN_H=4
)(input clk,rst_n,enable,input [23:0] rgb,input de,vs,
  output reg box_valid,output reg [11:0] box_x0,box_y0,box_x1,box_y1);
    localparam integer COLS=WIDTH/8,ROWS=HEIGHT/8,N=COLS*ROWS;
    reg [11:0] x,y;
    reg vd,dd,yp1,yp2,write_bank,read_bank,have_frame;
    wire [8:0] r={1'b0,rgb[23:16]},g={1'b0,rgb[15:8]},b={1'b0,rgb[7:0]};
    wire yellow=enable && r>=R_MIN && g>=G_MIN && r>=b+RB_MIN &&
        g>=b+GB_MIN && r<=g+RG_MAX && g<=r+GR_MAX &&
        (({3'd0,r}-{3'd0,b})<<3)>=r &&
        (({3'd0,g}-{3'd0,b})<<3)>=g; // Widen before shift: avoid overflow
    wire clean=yellow && yp1 && yp2 && x>=2;
    reg [3:0] row_hits;
    reg [6:0] col_hits[0:COLS-1];
    reg [11:0] row_bounds,col_bounds[0:COLS-1];
    // Bounds pack: xmin,xmax,ymin,ymax, each a 3-bit tile-local offset.
    function [11:0] merge_bounds;
        input [11:0] a,b;
        begin merge_bounds={a[11:9]<b[11:9]?a[11:9]:b[11:9],
            a[8:6]>b[8:6]?a[8:6]:b[8:6],
            a[5:3]<b[5:3]?a[5:3]:b[5:3],a[2:0]>b[2:0]?a[2:0]:b[2:0]};end
    endfunction
    wire [2:0] run_x0=x[2:0]>=2 ? x[2:0]-3'd2 : 3'd0;
    wire [11:0] pixel_bounds={run_x0,x[2:0],y[2:0],y[2:0]};
    wire [11:0] rb=clean ? (row_hits!=0 ? merge_bounds(row_bounds,pixel_bounds) : pixel_bounds) : row_bounds;
    wire [11:0] tile_bounds=col_hits[x[11:3]]!=0 ?
        ((row_hits!=0 || clean) ? merge_bounds(col_bounds[x[11:3]],rb) : col_bounds[x[11:3]]) : rb;
    wire [7:0] tile_sum=col_hits[x[11:3]]+row_hits+clean;
    wire tile_write=de && x<WIDTH && y<HEIGHT && x[2:0]==7 && y[2:0]==7;
    wire [12:0] tile_addr=(y>>3)*COLS+(x>>3);
    wire tile_bit=tile_sum>=TILE_MIN;
    reg map0[0:N-1],map1[0:N-1];
    reg [11:0] bounds0[0:N-1],bounds1[0:N-1],bounds_q0,bounds_q1;
    wire [11:0] bounds_q=read_bank ? bounds_q1 : bounds_q0;
    reg [12:0] addr,scan,queue[0:N-1],head,tail,current,queue_q;
    reg map_q0,map_q1;
    wire map_q=read_bank ? map_q1 : map_q0;
    reg erase;
    reg [3:0] state;
    localparam IDLE=0,SCAN_REQ=1,SCAN_GET=2,POP=3,POP_GET=4,
        NEIGHBOR=5,NEIGHBOR_GET=6,FINISH=7,NEXT_SCAN=8;
    reg [2:0] direction;
    reg [11:0] cx,cy,minx,miny,maxx,maxy;
    reg [11:0] pxmin,pymin,pxmax,pymax;
    wire [11:0] local_x0=(cx<<3)+bounds_q[11:9],local_x1=(cx<<3)+bounds_q[8:6];
    wire [11:0] local_y0=(cy<<3)+bounds_q[5:3],local_y1=(cy<<3)+bounds_q[2:0];
    reg [12:0] count,best_count;
    reg [11:0] best_x0,best_y0,best_x1,best_y1;
    reg done;
    wire [12:0] bw=maxx-minx+1'b1,bh=maxy-miny+1'b1;
    wire [25:0] rect_area=bw*bh;
    wire candidate=count>=MIN_TILES && bw>=MIN_W && bh>=MIN_H &&
        (bh<<1)>=bw && bh<=(bw<<2) && count*5>=rect_area*2 && count<N*3/4;
    reg prev_valid;
    reg [11:0] prev_x0,prev_y0,prev_x1,prev_y1;
    wire overlap=best_x0<=prev_x1+24 && prev_x0<=best_x1+24 &&
                 best_y0<=prev_y1+24 && prev_y0<=best_y1+24;
    integer i;
    // Each bank has one synchronous read and one write port. Never read the bank being captured.
    wire capture0=tile_write && !write_bank,capture1=tile_write && write_bank;
    wire we0=rst_n && (capture0 || (erase && !read_bank));
    wire we1=rst_n && (capture1 || (erase && read_bank));
    wire [12:0] wa0=capture0 ? tile_addr : addr,wa1=capture1 ? tile_addr : addr;
    always @(posedge clk) begin
        map_q0<=map0[addr];map_q1<=map1[addr];
        bounds_q0<=bounds0[queue_q];bounds_q1<=bounds1[queue_q];
        if(rst_n && capture0)bounds0[tile_addr]<=tile_bounds;
        if(rst_n && capture1)bounds1[tile_addr]<=tile_bounds;
        if(we0)map0[wa0]<=capture0 && tile_bit;
        if(we1)map1[wa1]<=capture1 && tile_bit;
    end
    // Mark visited at the same edge as queue insertion.
    always @* begin
        erase=0;
        if(state==SCAN_GET && map_q)erase=1;
        if(state==NEIGHBOR_GET && map_q && tail<N)erase=1;
    end
    wire seed=rst_n && !(vs && !vd) && state==SCAN_GET && map_q;
    wire append=rst_n && !(vs && !vd) && state==NEIGHBOR_GET && map_q && tail<N;
    wire [12:0] queue_wa=seed ? 13'd0 : tail;
    wire [12:0] queue_wd=seed ? scan : addr;
    always @(posedge clk)begin
        queue_q<=queue[head];
        if(seed || append)queue[queue_wa]<=queue_wd;
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            x<=0;y<=0;vd<=0;dd<=0;yp1<=0;yp2<=0;row_hits<=0;
            write_bank<=0;read_bank<=1;have_frame<=0;
            state<=IDLE;addr<=0;scan<=0;head<=0;tail<=0;current<=0;direction<=0;
            cx<=0;cy<=0;minx<=0;miny<=0;maxx<=0;maxy<=0;count<=0;best_count<=0;done<=0;
            best_x0<=0;best_y0<=0;best_x1<=0;best_y1<=0;
            box_valid<=0;box_x0<=0;box_y0<=0;box_x1<=0;box_y1<=0;
            prev_valid<=0;prev_x0<=0;prev_y0<=0;prev_x1<=0;prev_y1<=0;
            row_bounds<=0;pxmin<=0;pymin<=0;pxmax<=0;pymax<=0;
            for(i=0;i<COLS;i=i+1)begin col_hits[i]<=0;col_bounds[i]<=0;end
        end else begin
            vd<=vs;dd<=de;
            if(de)begin
                x<=x+1'b1;yp1<=yellow;yp2<=yp1;
                if(x[2:0]==7)begin
                    row_hits<=0;row_bounds<=0;col_bounds[x[11:3]]<=tile_bounds;
                    if(y[2:0]==7)col_hits[x[11:3]]<=0;
                    else col_hits[x[11:3]]<=tile_sum;
                end else begin row_hits<=row_hits+clean;row_bounds<=rb;end
            end else begin x<=0;yp1<=0;yp2<=0;row_hits<=0;row_bounds<=0;end
            if(!de && dd)y<=y+1'b1;
            if(vs && !vd)begin
                y<=0;write_bank<=~write_bank;read_bank<=write_bank;
                // The first VS may precede a complete captured frame; discard it.
                have_frame<=1;scan<=0;addr<=0;head<=0;tail<=0;best_count<=0;done<=0;
                state<=have_frame ? SCAN_REQ : IDLE;
                box_valid<=enable && done && best_count!=0 && prev_valid && overlap;
                if(done && best_count!=0)begin
                    box_x0<=best_x0;box_y0<=best_y0;box_x1<=best_x1;box_y1<=best_y1;
                    prev_x0<=best_x0;prev_y0<=best_y0;prev_x1<=best_x1;prev_y1<=best_y1;
                end
                prev_valid<=enable && done && best_count!=0;
            end else case(state)
                IDLE:;
                SCAN_REQ:state<=SCAN_GET;
                SCAN_GET:begin
                    if(map_q)begin
                        head<=0;tail<=1;count<=0;
                        minx<=COLS-1;miny<=ROWS-1;maxx<=0;maxy<=0;state<=POP;
                        pxmin<=WIDTH-1;pymin<=HEIGHT-1;pxmax<=0;pymax<=0;
                    end else state<=NEXT_SCAN;
                end
                POP:begin if(head==tail)state<=FINISH;else state<=POP_GET;end
                POP_GET:begin
                    current<=queue_q;cx<=queue_q%COLS;cy<=queue_q/COLS;
                    if(queue_q%COLS<minx)minx<=queue_q%COLS;
                    if(queue_q%COLS>maxx)maxx<=queue_q%COLS;
                    if(queue_q/COLS<miny)miny<=queue_q/COLS;
                    if(queue_q/COLS>maxy)maxy<=queue_q/COLS;
                    count<=count+1'b1;head<=head+1'b1;direction<=0;state<=10;
                end
                10:begin
                    if(local_x0<pxmin)pxmin<=local_x0;
                    if(local_x1>pxmax)pxmax<=local_x1;
                    if(local_y0<pymin)pymin<=local_y0;
                    if(local_y1>pymax)pymax<=local_y1;
                    state<=NEIGHBOR;
                end
                NEIGHBOR:begin
                    if(direction==4)state<=POP;
                    else if((direction==0 && cx==0)||(direction==1 && cx==COLS-1)||
                        (direction==2 && cy==0)||(direction==3 && cy==ROWS-1))direction<=direction+1'b1;
                    else begin
                        case(direction)
                            0:addr<=current-1'b1;1:addr<=current+1'b1;
                            2:addr<=current-COLS;default:addr<=current+COLS;
                        endcase
                        state<=9;
                    end
                end
                9:state<=NEIGHBOR_GET; // synchronous map read latency
                NEIGHBOR_GET:begin
                    if(map_q && tail<N)tail<=tail+1'b1;
                    direction<=direction+1'b1;state<=NEIGHBOR;
                end
                FINISH:begin
                    if(candidate && count>best_count)begin
                        best_count<=count;best_x0<=pxmin;best_y0<=pymin;
                        best_x1<=pxmax;best_y1<=pymax;
                    end
                    state<=NEXT_SCAN;
                end
                NEXT_SCAN:begin
                    if(scan==N-1)begin done<=1;state<=IDLE;end
                    else begin scan<=scan+1'b1;addr<=scan+1'b1;state<=SCAN_REQ;end
                end
                default:state<=IDLE;
            endcase
            if(!enable)begin box_valid<=0;prev_valid<=0;end
        end
    end
endmodule
