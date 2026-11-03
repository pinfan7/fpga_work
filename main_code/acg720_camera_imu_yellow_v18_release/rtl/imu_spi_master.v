`timescale 1ns/1ps
// Mode 3, MSB first. Default 50MHz / (2*500) = 50kHz.
// Explicit setup/hold/gap counters keep CS stable around the complete transfer.
// TX is left-aligned; RX is right-aligned. Supports 2..13-byte transactions.
module imu_spi_master #(
    parameter integer HALF_CYCLES=500,
    parameter integer CS_GUARD_CYCLES=HALF_CYCLES*2
)(
    input clk, input rst_n, input start,
    input [3:0] byte_count, input [103:0] tx_data,
    output reg busy, output reg done, output reg [103:0] rx_data,
    output reg sclk, output reg mosi, input miso, output reg cs_n
);
    reg [103:0] tx_shift;
    reg [6:0] bits_left;
    reg [15:0] divider;
    reg [2:0] state;
    (* syn_preserve=1 *) reg miso_meta, miso_sync;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) begin miso_meta<=0; miso_sync<=0; end
        else begin miso_meta<=miso; miso_sync<=miso_meta; end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=0; divider<=0; bits_left<=0; tx_shift<=0;
            busy<=0; done<=0; rx_data<=0; cs_n<=1; sclk<=1; mosi<=0;
        end else begin
            done<=0;
            case(state)
            0: if(start && byte_count>=2 && byte_count<=13) begin
                busy<=1; cs_n<=0; sclk<=1; mosi<=0;
                tx_shift<=tx_data; rx_data<=0; bits_left<={byte_count,3'b000};
                divider<=CS_GUARD_CYCLES-1; state<=1;
            end
            1: if(divider!=0) divider<=divider-1'b1;
               else begin
                   divider<=HALF_CYCLES-1;
                   if(sclk) begin
                       // Falling edge launches a bit; next rising edge samples it.
                       sclk<=0; mosi<=tx_shift[103];
                   end else begin
                       sclk<=1; rx_data<={rx_data[102:0],miso_sync};
                       if(bits_left==1) begin
                           divider<=CS_GUARD_CYCLES-1; state<=2;
                       end
                       else begin
                           bits_left<=bits_left-1'b1;
                           tx_shift<={tx_shift[102:0],1'b0};
                       end
                   end
               end
            2: if(divider!=0) divider<=divider-1'b1;
               else begin cs_n<=1; mosi<=0; divider<=CS_GUARD_CYCLES-1; state<=3; end
            3: if(divider!=0) divider<=divider-1'b1;
               else begin busy<=0; done<=1; state<=0; end
            default: begin state<=0; busy<=0; cs_n<=1; sclk<=1; mosi<=0; end
            endcase
        end
    end
endmodule
