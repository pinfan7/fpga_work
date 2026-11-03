`timescale 1ns/1ps
// Bundled-data request/ack handshake. src_hold remains stable until ack returns.
// dst_pending captures only after a two-FF synchronized request. Display updates
// exclusively at frame_start so digits/status cannot tear within a video frame.
module telemetry_cdc # (parameter integer WIDTH=160)(
    input src_clk,input src_rst_n,input src_valid,input [WIDTH-1:0] src_data,
    output src_ready,
    input dst_clk,input dst_rst_n,input frame_start,
    output reg [WIDTH-1:0] frame_data
);
    reg [WIDTH-1:0] src_hold;
    reg request,acknowledge;
    (* syn_preserve=1 *) reg ack_meta,ack_sync,req_meta,req_sync;
    reg [WIDTH-1:0] dst_pending;
    assign src_ready=request==ack_sync;
    always @(posedge src_clk or negedge src_rst_n) begin
        if(!src_rst_n) begin src_hold<=0;request<=0;ack_meta<=0;ack_sync<=0;end
        else begin
            ack_meta<=acknowledge;ack_sync<=ack_meta;
            if(src_valid && src_ready) begin src_hold<=src_data;request<=~request;end
        end
    end
    always @(posedge dst_clk or negedge dst_rst_n) begin
        if(!dst_rst_n) begin req_meta<=0;req_sync<=0;acknowledge<=0;dst_pending<=0;frame_data<=0;end
        else begin
            req_meta<=request;req_sync<=req_meta;
            if(req_sync!=acknowledge) begin dst_pending<=src_hold;acknowledge<=req_sync;end
            if(frame_start) frame_data<=dst_pending;
        end
    end
endmodule
