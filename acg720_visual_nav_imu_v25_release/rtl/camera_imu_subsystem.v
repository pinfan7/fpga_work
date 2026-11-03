`timescale 1ns/1ps
module camera_imu_subsystem #(
    parameter integer CLK_HZ=50000000,
    parameter integer BOOT_CYCLES=CLK_HZ,
    parameter integer CAL_LOG2=8,
    parameter integer AUTO_START=0
)(
    input clk50,input reset_n,input pixel_clk,input pixel_locked,input recal_n,
    inout imu_scl,imu_sda,
    input [23:0] rgb_in,input hs_in,vs_in,de_in,
    output [23:0] rgb_out,output hs_out,vs_out,de_out,
    output [3:0] status_led,
    output signed [31:0] nav_yaw,output nav_imu_ok,nav_imu_update
);
    reg [15:0] por_count=0;
    reg [1:0] core_reset_pipe=0,pixel_reset_pipe=0;
    wire core_reset_n=core_reset_pipe[1];
    wire pixel_reset_n=pixel_reset_pipe[1];
    always @(posedge clk50 or negedge reset_n)
        if(!reset_n) por_count<=0;else if(!(&por_count)) por_count<=por_count+1'b1;
    always @(posedge clk50 or negedge reset_n)
        if(!reset_n) core_reset_pipe<=0;else core_reset_pipe<={core_reset_pipe[0],(&por_count)};
    wire all_ready=core_reset_n && pixel_locked;
    reg [1:0] cdc_src_reset_pipe=0;
    always @(posedge clk50 or negedge all_ready)
        if(!all_ready) cdc_src_reset_pipe<=0;
        else cdc_src_reset_pipe<={cdc_src_reset_pipe[0],1'b1};
    always @(posedge pixel_clk or negedge all_ready)
        if(!all_ready) pixel_reset_pipe<=0;else pixel_reset_pipe<={pixel_reset_pipe[0],1'b1};
    (* syn_preserve=1 *) reg key_meta,key_sync;
    reg key_stable;
    reg [19:0] key_count;
    reg recalibrate;
    reg [25:0] heartbeat;
    always @(posedge clk50 or negedge core_reset_n) begin
        if(!core_reset_n) begin key_meta<=1;key_sync<=1;key_stable<=1;key_count<=0;recalibrate<=0;heartbeat<=0;end
        else begin
            heartbeat<=heartbeat+1'b1;key_meta<=recal_n;key_sync<=key_meta;recalibrate<=0;
            if(key_sync==key_stable) key_count<=0;
            else if(key_count==CLK_HZ/50-1) begin key_count<=0;key_stable<=key_sync;if(!key_sync) recalibrate<=1;end
            else key_count<=key_count+1'b1;
        end
    end
    wire signed [15:0] ax,ay,az,gx,gy,gz;
    wire [15:0] dt_us,sample_hz;
    wire [31:0] sample_count;
    wire [7:0] who_am_i,error_code;
    wire sample_valid,link_ok,configured,fault_pulse;
    wire retrying;
    wire [7:0] retry_total;
    reg [31:0] saved_count;
    reg [7:0] saved_retries;
    wire [20:0] diagnostic;
    wire [1:0] drive_low;
    wire session_active,session_stopped;
    wire [7:0] saved_error,saved_id;
    wire [20:0] saved_diagnostic;
    imu_session_guard #(.AUTO_START(AUTO_START)) guard(.clk(clk50),.rst_n(core_reset_n),.press(recalibrate),
        .fault_pulse(fault_pulse),.link_ok(link_ok),.configured(configured),.error_code(error_code),
        .who_am_i(who_am_i),.diagnostic(diagnostic),.active(session_active),
        .stopped(session_stopped),.saved_error(saved_error),.saved_id(saved_id),
        .saved_diagnostic(saved_diagnostic));
    // Synchronous release in the acquisition clock domain; asynchronous stop.
    wire acquisition_enable=core_reset_n && session_active;
    reg [1:0] acquisition_reset_pipe=0;
    always @(posedge clk50 or negedge acquisition_enable)
        if(!acquisition_enable)acquisition_reset_pipe<=0;
        else acquisition_reset_pipe<={acquisition_reset_pipe[0],1'b1};
    wire acquisition_reset_n=acquisition_reset_pipe[1];
    (* syn_preserve=1 *) reg scl_meta,scl_sync,sda_meta,sda_sync;
    always @(posedge clk50 or negedge core_reset_n)
        if(!core_reset_n)begin scl_meta<=1;scl_sync<=1;sda_meta<=1;sda_sync<=1;end
        else begin scl_meta<=imu_scl;scl_sync<=scl_meta;sda_meta<=imu_sda;sda_sync<=sda_meta;end
    wire [7:0] shown_error=session_stopped ? saved_error : error_code;
    wire [7:0] shown_id=session_stopped ? saved_id : who_am_i;
    wire [20:0] shown_diag=session_stopped ? saved_diagnostic : diagnostic;
    always @(posedge clk50 or negedge core_reset_n)
        if(!core_reset_n)begin saved_count<=0;saved_retries<=0;end
        else if(recalibrate)begin saved_count<=0;saved_retries<=0;end
        else if(session_active && fault_pulse)begin
            saved_count<=sample_count;saved_retries<=retry_total;
        end
    wire [31:0] shown_count=session_stopped ? saved_count : sample_count;
    wire [7:0] shown_retries=session_stopped ? saved_retries : retry_total;
    // Preserve register/readback/address, use former bus-error nibble for live pins.
    wire [20:0] status_diag={shown_diag[20],scl_sync,sda_sync,session_active,session_stopped,shown_diag[15:0]};
    icm42688_acquire #(.CLK_HZ(CLK_HZ),.BOOT_CYCLES(BOOT_CYCLES)) u_imu(.clk(clk50),.rst_n(acquisition_reset_n),
        .scl(imu_scl),.sda(imu_sda),
        .sample_valid(sample_valid),.ax(ax),.ay(ay),.az(az),.gx(gx),.gy(gy),.gz(gz),
        .dt_us(dt_us),.sample_count(sample_count),.sample_hz(sample_hz),
        .who_am_i(who_am_i),.error_code(error_code),.link_ok(link_ok),.configured(configured),
        .diagnostic(diagnostic),.fault_pulse(fault_pulse),.drive_low(drive_low),
        .trace_valid(),.trace_data(),.retrying(retrying),.retry_total(retry_total));
    wire signed [31:0] roll,pitch,yaw;
    wire calibrated,attitude_valid,update,accel_trusted,tilt_warning,overrun;
    wire [8:0] cal_count;
    imu_attitude #(.CAL_LOG2(CAL_LOG2)) u_attitude(.clk(clk50),.rst_n(core_reset_n),.recalibrate(recalibrate),
        .link_ok(link_ok),.sample_valid(sample_valid),.ax(ax),.ay(ay),.az(az),
        .gx(gx),.gy(gy),.gz(gz),.dt_us(dt_us),.roll(roll),.pitch(pitch),.yaw(yaw),
        .calibrated(calibrated),.attitude_valid(attitude_valid),.update(update),.cal_count(cal_count),
        .accel_trusted(accel_trusted),.tilt_warning(tilt_warning),.overrun(overrun));
    assign nav_yaw=yaw;
    assign nav_imu_update=update;
    assign nav_imu_ok=core_reset_n && session_active && acquisition_reset_n && link_ok &&
      calibrated && attitude_valid && !retrying && !overrun && !tilt_warning;
    wire [287:0] formatted;
    wire [287:0] frame_packet;
    wire formatted_valid,packet_ready;
    wire [7:0] flags={1'b0,retrying,overrun,accel_trusted,tilt_warning,attitude_valid,calibrated,link_ok};
    telemetry_formatter #(.PERIOD_CYCLES(CLK_HZ/20)) u_format(.clk(clk50),.rst_n(core_reset_n),.roll(roll),.pitch(pitch),.yaw(yaw),
        .sample_count(shown_count),.sample_hz(sample_hz),.who_am_i(shown_id),.error_code(shown_error),
        .retry_total(shown_retries),
        .cal_count(cal_count),.flags(flags),.ax(ax),.ay(ay),.az(az),.gx(gx),.gy(gy),.gz(gz),
        .diagnostic(status_diag),.packet(formatted),.packet_valid(formatted_valid));
    reg vs_delayed;
    always @(posedge pixel_clk or negedge pixel_reset_n)
        if(!pixel_reset_n) vs_delayed<=0;else vs_delayed<=vs_in;
    wire frame_start=vs_in && !vs_delayed;
    // Assert both resets together on PLL loss; deassert synchronously per domain.
    telemetry_cdc #(.WIDTH(288)) u_cdc(.src_clk(clk50),.src_rst_n(cdc_src_reset_pipe[1]),
        .src_valid(formatted_valid),.src_data(formatted),.src_ready(packet_ready),
        .dst_clk(pixel_clk),.dst_rst_n(pixel_reset_n),.frame_start(frame_start),.frame_data(frame_packet));
    imu_compact_overlay u_overlay(.clk(pixel_clk),.rst_n(pixel_reset_n),.packet(frame_packet),
        .rgb_in(rgb_in),.hs_in(hs_in),.vs_in(vs_in),.de_in(de_in),
        .rgb_out(rgb_out),.hs_out(hs_out),.vs_out(vs_out),.de_out(de_out));
    assign status_led={heartbeat[25],(shown_error!=0 || overrun),session_active,link_ok};
endmodule
