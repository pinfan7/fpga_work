`timescale 1ns/1ps
// LD-1501MG standalone steering BENCH TEST, not integrated camera/IMU firmware.
// V2: S0 reset; S1 enable/disable; S2 LEFT preset; S3 center; S4 RIGHT preset.
// Repeated button presses do not accumulate. Confirm installation direction.
// Requested pulse width is NOT measured wheel/servo angle.
module acg720_servo_top #(
    parameter integer CLK_HZ = 50000000,
    parameter integer BOOT_MS = 100,
    parameter integer DEBOUNCE_MS = 20,
    parameter integer FRAME_US = 20000,
    parameter integer CENTER_US = 1500,
    parameter integer MIN_US = 1389,
    parameter integer MAX_US = 1611,
    parameter integer TURN_DEG = 10,
    parameter integer LEFT_SMALLER_PULSE = 1
)(
    input wire clk50M, reset_n,
    input wire key_enable_n, key_minus_n, key_center_n, key_plus_n,
    output wire servo_pwm_out,
    output wire [7:0] led
);
    localparam integer BOOT_CYCLES = (CLK_HZ/1000)*BOOT_MS;
    localparam integer BW = (BOOT_CYCLES < 2) ? 1 : $clog2(BOOT_CYCLES);
    localparam integer DB_CYCLES = (CLK_HZ/1000)*DEBOUNCE_MS;
    localparam integer HEART_CYCLES = CLK_HZ/2;
    localparam integer HW = $clog2(HEART_CYCLES);
    // Nominal mapping from the supplied specification: 2000us per 180 degrees.
    // Round to the nearest microsecond. 5 degrees -> 56us, NOT measured angle.
    localparam integer OFFSET_US = (TURN_DEG*2000+90)/180;
    localparam integer LEFT_RAW = LEFT_SMALLER_PULSE ? CENTER_US-OFFSET_US : CENTER_US+OFFSET_US;
    localparam integer RIGHT_RAW = LEFT_SMALLER_PULSE ? CENTER_US+OFFSET_US : CENTER_US-OFFSET_US;
    localparam [15:0] LEFT_US = LEFT_RAW < MIN_US ? MIN_US : LEFT_RAW > MAX_US ? MAX_US : LEFT_RAW;
    localparam [15:0] RIGHT_US = RIGHT_RAW < MIN_US ? MIN_US : RIGHT_RAW > MAX_US ? MAX_US : RIGHT_RAW;
    reg [2:0] reset_pipe = 0;
    always @(posedge clk50M or negedge reset_n) begin
        if (!reset_n) reset_pipe <= 0;
        else reset_pipe <= {reset_pipe[1:0], 1'b1};
    end
    wire reset = !reset_pipe[2];
    reg [BW-1:0] boot_count = 0;
    reg boot_done = 1'b0;
    reg [HW-1:0] heart_count = 0;
    reg heart = 1'b0;
    always @(posedge clk50M) begin
        if (reset) begin
            boot_count <= 0; boot_done <= 1'b0;
            heart_count <= 0; heart <= 1'b0;
        end else begin
            if (!boot_done) begin
                if (boot_count == BOOT_CYCLES-1) boot_done <= 1'b1;
                else boot_count <= boot_count + 1'b1;
            end
            if (heart_count == HEART_CYCLES-1) begin
                heart_count <= 0; heart <= !heart;
            end else heart_count <= heart_count + 1'b1;
        end
    end
    wire [3:0] released, pressed;
    wire [3:0] keys_n = {key_plus_n,key_center_n,key_minus_n,key_enable_n};
    genvar k;
    generate for (k=0;k<4;k=k+1) begin: buttons
        button_event #(.STABLE_CYCLES(DB_CYCLES)) u_key (
            .clk(clk50M),.reset(reset),.key_n(keys_n[k]),
            .released(released[k]),.press(pressed[k]));
    end endgenerate
    reg enabled = 1'b0;
    reg manual_ready = 1'b0;
    reg [15:0] target_us = CENTER_US;
    always @(posedge clk50M) begin
        if (reset || !boot_done) begin
            enabled <= 1'b0; manual_ready <= 1'b0; target_us <= CENTER_US;
        end else begin
            // BOOT_MS > DEBOUNCE_MS: a key held during configuration cannot arm.
            if (&released) manual_ready <= 1'b1;
            if (pressed[0]) begin
                manual_ready <= 1'b0;
                target_us <= CENTER_US;
                if (enabled) enabled <= 1'b0;
                else if (manual_ready && (&released[3:1])) enabled <= 1'b1;
            end else if (enabled) begin
                if (pressed[2]) target_us <= CENTER_US;
                else if (pressed[1] && !pressed[3] && released[3]) begin
                    target_us <= LEFT_US;
                end else if (pressed[3] && !pressed[1] && released[1]) begin
                    target_us <= RIGHT_US;
                end
            end
        end
    end
    wire pwm_internal;
    servo_pwm #(.CLK_HZ(CLK_HZ),.FRAME_US(FRAME_US),.MIN_US(MIN_US),
        .MAX_US(MAX_US),.CENTER_US(CENTER_US)) u_pwm (
        .clk(clk50M),.reset(reset || !boot_done),.enable(enabled),
        .request_us(target_us),.pwm(pwm_internal));
    // S0 can suppress the output immediately; it does not remove servo power.
    assign servo_pwm_out = reset_n && !reset && pwm_internal;
    // ACG720 LEDs active high. D6 deliberately off; D7 is a 1Hz heartbeat.
    assign led = {heart,1'b0,
                  (target_us==MIN_US || target_us==MAX_US),
                  (target_us==RIGHT_US),(target_us==LEFT_US),
                  (target_us==CENTER_US),manual_ready,enabled};
`ifndef SYNTHESIS
    initial begin
        if (BOOT_MS <= DEBOUNCE_MS || TURN_DEG < 1 || TURN_DEG > 10 ||
            LEFT_RAW < MIN_US || LEFT_RAW > MAX_US || RIGHT_RAW < MIN_US || RIGHT_RAW > MAX_US) begin
            $display("Invalid bench-control parameters"); $finish;
        end
    end
`endif
endmodule
