`timescale 1ns/1ps
// Single clock domain. Command and enable are sampled ONLY at a frame boundary.
// Disable therefore finishes the current pulse, then emits no further pulses.
// Emergency reset may intentionally truncate a pulse. No position feedback.
module servo_pwm #(
    parameter integer CLK_HZ = 50000000,
    parameter integer FRAME_US = 20000,
    parameter integer MIN_US = 1400,
    parameter integer MAX_US = 1600,
    parameter integer CENTER_US = 1500
)(
    input wire clk, reset, enable,
    input wire [15:0] request_us,
    output reg pwm = 1'b0
);
    localparam integer TICKS_US = CLK_HZ / 1000000;
    localparam integer FRAME_TICKS = TICKS_US * FRAME_US;
    localparam integer CW = (FRAME_TICKS < 2) ? 1 : $clog2(FRAME_TICKS);
    reg [CW-1:0] counter = FRAME_TICKS-1;
    reg [CW-1:0] width_ticks = CENTER_US * TICKS_US;
    reg active = 1'b0;
    wire [15:0] bounded_us = (request_us < MIN_US) ? MIN_US :
                            (request_us > MAX_US) ? MAX_US : request_us;
    always @(posedge clk) begin
        if (reset) begin
            counter <= FRAME_TICKS-1;
            width_ticks <= CENTER_US*TICKS_US;
            active <= 1'b0;
            pwm <= 1'b0;
        end else if (counter == FRAME_TICKS-1) begin
            counter <= 0;
            width_ticks <= bounded_us*TICKS_US;
            active <= enable;
            pwm <= enable;
        end else begin
            counter <= counter + 1'b1;
            if (!active || (counter >= width_ticks-1)) pwm <= 1'b0;
        end
    end
`ifndef SYNTHESIS
    initial begin
        if (CLK_HZ < 1000000 || CLK_HZ % 1000000 != 0 ||
            MIN_US <= 0 || MIN_US > CENTER_US || CENTER_US > MAX_US ||
            MAX_US >= FRAME_US || MAX_US > 65535) begin
            $display("Invalid servo timing parameters");
            $finish;
        end
    end
`endif
endmodule
