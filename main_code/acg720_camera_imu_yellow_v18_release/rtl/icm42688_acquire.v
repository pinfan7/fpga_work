`timescale 1ns/1ps
// TDK DS-000347: bank0, gyro +/-500dps, accel +/-4g, both 100Hz LN.
// Poll DATA_RDY (not a fabricated periodic sample), then burst-read all 12 bytes.
// No DMP, MCU, CPU, or on-sensor attitude output is used.
module icm42688_acquire #(
    parameter integer CLK_HZ=50000000,
    parameter integer I2C_HZ=25000,
    parameter integer BOOT_CYCLES=CLK_HZ
)(
    input clk, input rst_n,
    inout scl,sda,
    output reg sample_valid,
    output reg signed [15:0] ax,ay,az,gx,gy,gz,
    output reg [15:0] dt_us,
    output reg [31:0] sample_count,
    output reg [15:0] sample_hz,
    output reg [7:0] who_am_i, output reg [7:0] error_code,
    output link_ok, output reg configured,
    output reg [20:0] diagnostic,
    output reg fault_pulse,
    output [1:0] drive_low,
    output reg trace_valid,
    output reg [31:0] trace_data,
    output reg retrying,
    output reg [7:0] retry_total
);
    localparam integer US_CYCLES=CLK_HZ/1000000;
    localparam integer STALE_CYCLES=CLK_HZ*3/10; // Freshness only; dt remains bounded
    localparam integer HARD_STALE_CYCLES=CLK_HZ;
    localparam integer RETRY_CYCLES=CLK_HZ/200;
    localparam integer MAX_RETRIES=6;
    localparam [5:0] BOOT=0, RESET_WAIT=1, CHECK_ID=2, CONFIG_WRITE=3,
        CONFIG_NEXT=4, WARMUP=5, VERIFY=6, VERIFY_NEXT=7, POLL=8,
        POLL_DONE=9, SAMPLE_DONE=10, PERIODIC_ID=11, RETRY=12, BUS_WAIT=13,
        RESET_COMMAND=14, READ_ID=15, CONFIG_BEGIN=16, CHECK_RESET=17,
        DISCOVER_ID=18, SOFT_RETRY=19;
    reg [5:0] state, return_state;
    reg [2:0] cfg_index;
    reg [31:0] timer, age, bus_watchdog, hz_timer;
    reg [15:0] hz_count;
    reg [7:0] us_div;
    reg [31:0] now_us, last_us;
    reg have_time;
    reg [2:0] retry_streak;
    reg bus_start;
    reg [3:0] bus_len;
    reg [15:0] bus_cmd;
    reg [6:0] device_address;
    wire bus_busy,bus_done;
    wire [3:0] bus_error;
    wire [95:0] bus_rx;
    wire [7:0] bus_echo;
    wire [7:0] bus_failure_detail;
    wire [3:0] bus_acks;
    wire [31:0] elapsed_us=now_us-last_us;
    // A transient error does not erase calibration. No new sample is published
    // until a complete valid burst; the overlay masks values while retrying.
    assign link_ok=configured && have_time && age<STALE_CYCLES;
    function [15:0] config_word;
        input [2:0] idx;
        begin case(idx)
            0: config_word=16'h4c32; // Big-endian, disable SPI; KEEP I2C enabled.
            1: config_word=16'h4f48; // +/-500dps, 100Hz.
            2: config_word=16'h5048; // +/-4g, 100Hz.
            3: config_word=16'h5244; // gyro/accel UI lowpass about 40Hz.
            default: config_word=16'h4e0f; // accel + gyro LN mode.
        endcase end
    endfunction
    wire [15:0] cfg_word=config_word(cfg_index);
    wire raw_bad = (bus_rx==96'd0) || (&bus_rx) ||
        bus_rx[95:80]==16'h8000 || bus_rx[79:64]==16'h8000 ||
        bus_rx[63:48]==16'h8000 || bus_rx[47:32]==16'h8000 ||
        bus_rx[31:16]==16'h8000 || bus_rx[15:0]==16'h8000;
    imu_recovery_i2c_master #(.CLK_HZ(CLK_HZ),.I2C_HZ(I2C_HZ)) u_i2c(
        .clk(clk),.rst_n(rst_n),.start(bus_start),.address(device_address),
        .read_op(bus_cmd[15]),.reg_address({1'b0,bus_cmd[14:8]}),
        .write_data(bus_cmd[7:0]),.read_count(bus_len-4'd1),
        .scl(scl),.sda(sda),.busy(bus_busy),.done(bus_done),.error(bus_error),.rx_data(bus_rx),
        .drive_low(drive_low),.address_echo(bus_echo),.ack_map(bus_acks),
        .failure_detail(bus_failure_detail));
    task issue;
        input [15:0] first_two;
        input [3:0] length;
        input [5:0] next_state;
        begin
            // Bit7 here is an internal read flag, stripped before I2C transmission.
            bus_cmd<=first_two; bus_len<=length; bus_start<=1;
            bus_watchdog<=CLK_HZ/20; return_state<=next_state; state<=BUS_WAIT;
        end
    endtask
    task fault;
        input [7:0] code;
        begin
            error_code<=code; configured<=0; have_time<=0;fault_pulse<=1;
            retrying<=1;
            timer<=CLK_HZ/2; state<=RETRY;
        end
    endtask
    task recover;
        input [7:0] code;
        begin
            if(configured && retry_streak<MAX_RETRIES && age<HARD_STALE_CYCLES)begin
                error_code<=code;retrying<=1;retry_streak<=retry_streak+1'b1;
                if(retry_total!=8'hff)retry_total<=retry_total+1'b1;
                timer<=RETRY_CYCLES;state<=SOFT_RETRY;
            end else fault(code);
        end
    endtask
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=BOOT; return_state<=BOOT; cfg_index<=0;
            timer<=BOOT_CYCLES; age<=STALE_CYCLES; bus_watchdog<=0;
            bus_start<=0; bus_len<=2; bus_cmd<=0;device_address<=7'h68;
            sample_valid<=0; ax<=0; ay<=0; az<=0; gx<=0; gy<=0; gz<=0;
            configured<=0; error_code<=0; who_am_i<=0;
            diagnostic<=0;fault_pulse<=0;trace_valid<=0;trace_data<=0;
            dt_us<=10000; sample_count<=0; have_time<=0;
            retrying<=0;retry_total<=0;retry_streak<=0;
            us_div<=0; now_us<=0; last_us<=0;
            hz_timer<=0; hz_count<=0; sample_hz<=0;
        end else begin
            sample_valid<=0; bus_start<=0;fault_pulse<=0;trace_valid<=0;
            if(timer!=0) timer<=timer-1'b1;
            if(age<HARD_STALE_CYCLES) age<=age+1'b1;
            if(us_div==US_CYCLES-1) begin us_div<=0; now_us<=now_us+1'b1; end
            else us_div<=us_div+1'b1;
            if(hz_timer==CLK_HZ-1) begin
                hz_timer<=0; sample_hz<=hz_count; hz_count<=0;
            end else hz_timer<=hz_timer+1'b1;
            case(state)
            // Cold-power-cycle required: default bank0; no writes until ID47.
            BOOT: if(timer==0) issue(16'hf500,2,DISCOVER_ID);
            DISCOVER_ID:begin
                who_am_i<=bus_rx[7:0];
                if(bus_rx[7:0]!=8'h47)begin device_address<=device_address^7'h01;fault(8'h01);end
                else issue(16'h7600,2,RESET_COMMAND);
            end
            RESET_COMMAND: issue(16'h1101,2,RESET_WAIT);
            // Match supplier HAL example's 100ms reset settling period.
            RESET_WAIT: begin timer<=CLK_HZ/10; state<=CHECK_ID; end
            CHECK_ID: if(timer==0) issue(16'h7600,2,READ_ID);
            READ_ID: issue(16'hf500,2,CONFIG_WRITE);
            CONFIG_WRITE: begin
                who_am_i<=bus_rx[7:0];
                if(bus_rx[7:0]!=8'h47)begin device_address<=device_address^7'h01;fault(8'h01);end
                else issue(16'had00,2,CHECK_RESET);
            end
            CHECK_RESET:if(!bus_rx[4])fault(8'h14);else state<=CONFIG_BEGIN;
            CONFIG_BEGIN:begin cfg_index<=0;error_code<=0;issue(16'h4c32,2,CONFIG_NEXT);end
            CONFIG_NEXT: if(cfg_index==4) begin
                timer<=CLK_HZ/20; state<=WARMUP;
            end else begin
                cfg_index<=cfg_index+1'b1;
                issue(config_word(cfg_index+1'b1),2,CONFIG_NEXT);
            end
            WARMUP: if(timer==0) begin cfg_index<=0; state<=VERIFY; end
            VERIFY: issue({(cfg_word[15:8]|8'h80),8'h00},2,VERIFY_NEXT);
            VERIFY_NEXT: if(bus_rx[7:0]!=cfg_word[7:0]) fault(8'h02);
                else if(cfg_index==4) begin
                    configured<=1; age<=0; timer<=CLK_HZ/1000; state<=POLL;
                end else begin cfg_index<=cfg_index+1'b1; state<=VERIFY; end
            // 100ms => stale/invalid; 500ms => terminal no-data fault.
            POLL: if(age>=HARD_STALE_CYCLES) fault(8'h04);
                else if(timer==0) issue(16'had00,2,POLL_DONE);
            POLL_DONE: if(bus_rx[7:0]==8'hff) recover(8'h03);
                else if(bus_rx[3]) issue(16'h9f00,13,SAMPLE_DONE);
                else begin timer<=CLK_HZ/1000; state<=POLL; end
            SAMPLE_DONE: if(raw_bad) recover(8'h03);
                else if(have_time && elapsed_us>50000)begin
                    // Do not truncate a long gap to 16 bits or integrate unknown
                    // motion. This is a valid completed sensor read, NOT a fourth
                    // bus failure. Reset the error streak and re-seed separately.
                    have_time<=0;retry_streak<=0;error_code<=8'h05;retrying<=1;
                    timer<=RETRY_CYCLES;state<=SOFT_RETRY;
                end
                else if(have_time && elapsed_us<1000)begin have_time<=0;recover(8'h05);end
                else begin
                    ax<=bus_rx[95:80]; ay<=bus_rx[79:64]; az<=bus_rx[63:48];
                    gx<=bus_rx[47:32]; gy<=bus_rx[31:16]; gz<=bus_rx[15:0];
                    dt_us<=have_time ? elapsed_us[15:0] : 16'd10000;
                    have_time<=1; last_us<=now_us; age<=0; sample_valid<=1;
                    retrying<=0;retry_streak<=0;error_code<=0;
                    sample_count<=sample_count+1'b1;
                    hz_count<=(hz_timer==CLK_HZ-1) ? 16'd1 : hz_count+1'b1;
                    if(sample_count[7:0]==8'hff) issue(16'hf500,2,PERIODIC_ID);
                    else begin timer<=CLK_HZ/1000; state<=POLL; end
                end
            PERIODIC_ID: begin
                who_am_i<=bus_rx[7:0];
                if(bus_rx[7:0]!=8'h47) fault(8'h01);
                else begin timer<=CLK_HZ/1000; state<=POLL; end
            end
            RETRY: if(timer==0) begin timer<=0; state<=BOOT; end
            SOFT_RETRY:if(timer==0)begin timer<=0;state<=POLL;end
            BUS_WAIT: if(bus_done) begin
                    // Keep the last failure context until a valid sample recovers.
                    if(bus_error!=0 || !retrying)
                        diagnostic<={device_address[0],bus_error,1'b0,bus_cmd[14:8],
                            (bus_error!=0 ? bus_failure_detail : bus_rx[7:0])};
                    trace_valid<=1;
                    trace_data<={1'b1,device_address[0],6'd0,
                        (bus_error!=0 ? (8'h20+{4'd0,bus_error}) : 8'd0),bus_echo,bus_acks,4'd0};
                    if(bus_error!=0)begin
                        // Never switch slave address on a runtime NACK.
                        if(bus_error==1 && !configured)device_address<=device_address^7'h01;
                        recover(8'h20+{4'd0,bus_error});
                    end else state<=return_state;
                end
                else if(bus_watchdog==0) fault(8'h06);
                else bus_watchdog<=bus_watchdog-1'b1;
            default: fault(8'h06);
            endcase
        end
    end
endmodule
