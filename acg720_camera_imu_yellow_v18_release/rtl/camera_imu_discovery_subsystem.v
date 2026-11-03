`timescale 1ns/1ps
// Independent supplier-core bus discovery, with camera and HDMI preserved.
module camera_imu_discovery_subsystem(
    input clk50,input reset_n,input pixel_clk,input pixel_locked,input recal_n,
    inout imu_scl,imu_sda,
    input [23:0] rgb_in,input hs_in,vs_in,de_in,
    output [23:0] rgb_out,output hs_out,vs_out,de_out,
    output [3:0] status_led
);
    reg [15:0] por_count=0;
    reg [1:0] core_reset_pipe=0,pixel_reset_pipe=0;
    wire core_reset_n=core_reset_pipe[1];
    wire pixel_reset_n=pixel_reset_pipe[1];
    always @(posedge clk50 or negedge reset_n)
        if(!reset_n)por_count<=0;else if(!(&por_count))por_count<=por_count+1'b1;
    always @(posedge clk50 or negedge reset_n)
        if(!reset_n)core_reset_pipe<=0;else core_reset_pipe<={core_reset_pipe[0],(&por_count)};
    wire all_ready=core_reset_n&&pixel_locked;
    reg [1:0] cdc_src_reset_pipe=0;
    always @(posedge clk50 or negedge all_ready)
        if(!all_ready)cdc_src_reset_pipe<=0;
        else cdc_src_reset_pipe<={cdc_src_reset_pipe[0],1'b1};
    always @(posedge pixel_clk or negedge all_ready)
        if(!all_ready)pixel_reset_pipe<=0;
        else pixel_reset_pipe<={pixel_reset_pipe[0],1'b1};
    (* syn_preserve=1 *) reg key_meta,key_sync;
    reg key_stable;
    reg [19:0] key_count;
    reg start_scan,session_enabled;
    reg [25:0] heartbeat;
    always @(posedge clk50 or negedge core_reset_n) begin
        if(!core_reset_n)begin
            key_meta<=1;key_sync<=1;key_stable<=1;key_count<=0;
            start_scan<=0;session_enabled<=0;heartbeat<=0;
        end else begin
            heartbeat<=heartbeat+1'b1;
            key_meta<=recal_n;key_sync<=key_meta;start_scan<=0;
            if(key_sync==key_stable)key_count<=0;
            else if(key_count==999999)begin
                key_count<=0;key_stable<=key_sync;
                if(!key_sync)begin start_scan<=1;session_enabled<=1;end
            end else key_count<=key_count+1'b1;
        end
    end
    wire scan_busy,scan_done,verified_icm;
    wire [7:0] scan_error,found_count,scan_address;
    wire [127:0] records;
    imu_reference_discovery scanner(.clk(clk50),.rst_n(core_reset_n),.start(start_scan),
        .scl(imu_scl),.sda(imu_sda),.busy(scan_busy),.done(scan_done),
        .verified_icm(verified_icm),.error(scan_error),.found_count(found_count),
        .scan_address(scan_address),.records(records));
    (* syn_preserve=1 *) reg scl_meta,scl_sync,sda_meta,sda_sync;
    always @(posedge clk50 or negedge core_reset_n)
        if(!core_reset_n)begin scl_meta<=1;scl_sync<=1;sda_meta<=1;sda_sync<=1;end
        else begin scl_meta<=imu_scl;scl_sync<=scl_meta;sda_meta<=imu_sda;sda_sync<=sda_meta;end
    reg [21:0] packet_count;
    reg packet_tick;
    always @(posedge clk50 or negedge core_reset_n)
        if(!core_reset_n)begin packet_count<=0;packet_tick<=0;end
        else begin
            packet_tick<=0;
            if(packet_count==2499999)begin packet_count<=0;packet_tick<=1;end
            else packet_count<=packet_count+1'b1;
        end
    wire [159:0] packet={scan_busy,scan_done,verified_icm,scan_error,found_count,
        scan_address,scl_sync,sda_sync,session_enabled,2'b00,records};
    wire [159:0] frame_packet;
    wire packet_ready;
    reg vs_delayed;
    always @(posedge pixel_clk or negedge pixel_reset_n)
        if(!pixel_reset_n)vs_delayed<=0;else vs_delayed<=vs_in;
    wire frame_start=vs_in&&!vs_delayed;
    telemetry_cdc #(.WIDTH(160)) u_cdc(.src_clk(clk50),.src_rst_n(cdc_src_reset_pipe[1]),
        .src_valid(packet_tick),.src_data(packet),.src_ready(packet_ready),
        .dst_clk(pixel_clk),.dst_rst_n(pixel_reset_n),.frame_start(frame_start),.frame_data(frame_packet));
    imu_discovery_overlay u_overlay(.clk(pixel_clk),.rst_n(pixel_reset_n),.packet(frame_packet),
        .rgb_in(rgb_in),.hs_in(hs_in),.vs_in(vs_in),.de_in(de_in),
        .rgb_out(rgb_out),.hs_out(hs_out),.vs_out(vs_out),.de_out(de_out));
    assign status_led={heartbeat[25],(scan_error!=0 || (scan_done&&!verified_icm)),scan_busy,verified_icm};
endmodule
