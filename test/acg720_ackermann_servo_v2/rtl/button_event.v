`timescale 1ns/1ps
// Synchronize each asynchronous key; emit one event per debounced press.
module button_event #(
    parameter integer STABLE_CYCLES = 1000000
)(
    input wire clk, reset, key_n,
    output reg released = 1'b1,
    output reg press = 1'b0
);
    localparam integer CW = (STABLE_CYCLES < 2) ? 1 : $clog2(STABLE_CYCLES);
    reg [2:0] sync_n = 3'b111;
    reg [CW-1:0] count = 0;
    always @(posedge clk) begin
        if (reset) begin
            sync_n <= 3'b111;
            released <= 1'b1;
            count <= 0;
            press <= 1'b0;
        end else begin
            sync_n <= {sync_n[1:0], key_n};
            press <= 1'b0;
            if (sync_n[2] == released) count <= 0;
            else if (count == STABLE_CYCLES-1) begin
                released <= sync_n[2];
                press <= !sync_n[2];
                count <= 0;
            end else count <= count + 1'b1;
        end
    end
endmodule
