`timescale 1ns/1ps
// Single-master, open-drain register write / repeated-START burst read.
// Never drives either line high. Supports clock stretching and bounded timeout.
module imu_i2c_master #(
    parameter integer CLK_HZ=50000000,
    parameter integer I2C_HZ=25000,
    parameter integer TIMEOUT_CYCLES=CLK_HZ/50
)(
    input clk,rst_n,start,
    input [6:0] address,
    input read_op,
    input [7:0] reg_address,write_data,
    input [3:0] read_count,
    inout scl,sda,
    output reg busy,done,
    output reg [3:0] error,
    output reg [95:0] rx_data,
    output [1:0] drive_low,
    output reg [7:0] address_echo,
    output reg [3:0] ack_map
);
    localparam integer Q=CLK_HZ/(I2C_HZ*4), H=Q*2;
    localparam [5:0] IDLE=0,FREE=1,START_HOLD=2,TX_GUARD=3,TX_SETUP=4,
        TX_RISE=5,TX_HIGH=6,ACK_GUARD=7,ACK_SETUP=8,ACK_RISE=9,ACK_HIGH=10,
        RESTART_LOW=11,RESTART_RISE=12,RESTART_HIGH=13,
        RX_LOW=14,RX_RISE=15,RX_HIGH=16,MA_GUARD=17,MA_SETUP=18,
        MA_RISE=19,MA_HIGH=20,STOP_GUARD=21,STOP_LOW=22,STOP_RISE=23,
        STOP_HIGH=24,STOP_FREE=25,RX_RELEASE=26;
    reg [5:0] state;
    reg scl_low,sda_low;
    // Explicit gate: asserting acquisition reset releases the physical outputs.
    assign scl=(rst_n && scl_low) ? 1'b0 : 1'bz;
    assign sda=(rst_n && sda_low) ? 1'b0 : 1'bz;
    assign drive_low={rst_n && scl_low,rst_n && sda_low};
    (* syn_preserve=1 *) reg scl_meta,scl_sync,sda_meta,sda_sync;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)begin scl_meta<=1;scl_sync<=1;sda_meta<=1;sda_sync<=1;end
        else begin scl_meta<=scl;scl_sync<=scl_meta;sda_meta<=sda;sda_sync<=sda_meta;end
    reg [31:0] timer,watchdog;
    reg [6:0] addr_latch;
    reg read_latch;
    reg [7:0] reg_latch,data_latch,tx_byte;
    reg [3:0] count_latch,byte_index;
    reg [2:0] bit_index;
    reg [1:0] tx_phase; // addrW, register, dataW, addrR
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;scl_low<=0;sda_low<=0;busy<=0;done<=0;error<=0;rx_data<=0;
            timer<=0;watchdog<=0;addr_latch<=0;read_latch<=0;reg_latch<=0;
            data_latch<=0;tx_byte<=0;count_latch<=0;byte_index<=0;bit_index<=7;tx_phase<=0;
            address_echo<=0;ack_map<=0;
        end else begin
            done<=0;
            if(timer!=0)timer<=timer-1'b1;
            if(busy && watchdog!=0)watchdog<=watchdog-1'b1;
            if(busy && watchdog==0)begin
                // Includes stuck SCL, missing pull-up, and a bus that cannot idle.
                error<=!scl_sync ? 4'd5 : 4'd6;
                scl_low<=0;sda_low<=0;busy<=0;done<=1;state<=IDLE;
            end else case(state)
            IDLE:begin
                scl_low<=0;sda_low<=0;
                if(start)begin
                    error<=0;rx_data<=0;address_echo<=0;ack_map<=0;
                    if(read_op && (read_count==0 || read_count>12))begin error<=7;done<=1;end
                    else begin
                        busy<=1;watchdog<=TIMEOUT_CYCLES;timer<=H-1;state<=FREE;
                        addr_latch<=address;read_latch<=read_op;reg_latch<=reg_address;
                        data_latch<=write_data;count_latch<=read_count;byte_index<=0;
                        tx_phase<=0;tx_byte<={address,1'b0};bit_index<=7;
                    end
                end
            end
            FREE:if(!scl_sync || !sda_sync)timer<=H-1;
                else if(timer==0)begin sda_low<=1;timer<=H-1;state<=START_HOLD;end
            START_HOLD:if(timer==0)begin scl_low<=1;timer<=Q-1;state<=TX_GUARD;end
            TX_GUARD:if(timer==0)begin
                if(scl_sync)begin
                    error<=8;scl_low<=0;sda_low<=0;busy<=0;done<=1;state<=IDLE;
                end else begin sda_low<=!tx_byte[bit_index];timer<=Q-1;state<=TX_SETUP;end
            end
            TX_SETUP:if(timer==0)begin scl_low<=0;state<=TX_RISE;end
            TX_RISE:if(scl_sync)begin timer<=H-1;state<=TX_HIGH;end
            TX_HIGH:if(timer==0)begin
                if(tx_phase==0)address_echo<={address_echo[6:0],sda_sync};
                if(sda_sync!=tx_byte[bit_index])begin
                    error<=9;scl_low<=0;sda_low<=0;busy<=0;done<=1;state<=IDLE;
                end else begin
                    scl_low<=1;timer<=Q-1;
                    if(bit_index==0)state<=ACK_GUARD;
                    else begin bit_index<=bit_index-1'b1;state<=TX_GUARD;end
                end
            end
            ACK_GUARD:if(timer==0)begin sda_low<=0;timer<=Q-1;state<=ACK_SETUP;end
            ACK_SETUP:if(timer==0)begin scl_low<=0;state<=ACK_RISE;end
            ACK_RISE:if(scl_sync)begin timer<=H-1;state<=ACK_HIGH;end
            ACK_HIGH:if(timer==0)begin
                scl_low<=1;timer<=Q-1;
                if(sda_sync)begin error<={2'b00,tx_phase}+1'b1;state<=STOP_GUARD;end
                else begin
                  ack_map[tx_phase]<=1'b1;
                  case(tx_phase)
                    0:begin tx_phase<=1;tx_byte<=reg_latch;bit_index<=7;state<=TX_GUARD;end
                    1:if(read_latch)begin timer<=H-1;state<=RESTART_LOW;end
                      else begin tx_phase<=2;tx_byte<=data_latch;bit_index<=7;state<=TX_GUARD;end
                    2:state<=STOP_GUARD;
                    3:begin bit_index<=7;timer<=H-1;state<=RX_LOW;end
                  endcase
                end
            end
            RESTART_LOW:if(timer==0)begin scl_low<=0;state<=RESTART_RISE;end
            RESTART_RISE:if(scl_sync)begin timer<=H-1;state<=RESTART_HIGH;end
            RESTART_HIGH:if(timer==0)begin
                sda_low<=1;timer<=H-1;state<=START_HOLD;
                tx_phase<=3;tx_byte<={addr_latch,1'b1};bit_index<=7;
            end
            RX_LOW:if(timer==0)begin scl_low<=0;state<=RX_RISE;end
            RX_RISE:if(scl_sync)begin timer<=H-1;state<=RX_HIGH;end
            RX_HIGH:if(timer==0)begin
                rx_data<={rx_data[94:0],sda_sync};scl_low<=1;
                if(bit_index==0)begin timer<=Q-1;state<=MA_GUARD;end
                else begin bit_index<=bit_index-1'b1;timer<=H-1;state<=RX_LOW;end
            end
            MA_GUARD:if(timer==0)begin
                sda_low<=(byte_index+1<count_latch);timer<=Q-1;state<=MA_SETUP;
            end
            MA_SETUP:if(timer==0)begin scl_low<=0;state<=MA_RISE;end
            MA_RISE:if(scl_sync)begin timer<=H-1;state<=MA_HIGH;end
            MA_HIGH:if(timer==0)begin
                scl_low<=1;timer<=Q-1;
                if(byte_index+1==count_latch)state<=STOP_GUARD;
                else begin byte_index<=byte_index+1'b1;state<=RX_RELEASE;end
            end
            // After ACK of a read byte, release SDA during SCL low, then receive.
            // This path is separate from the slave-ACK states above.
            RX_RELEASE:if(timer==0)begin sda_low<=0;bit_index<=7;timer<=Q-1;state<=RX_LOW;end
            STOP_GUARD:if(timer==0)begin sda_low<=1;timer<=Q-1;state<=STOP_LOW;end
            STOP_LOW:if(timer==0)begin scl_low<=0;state<=STOP_RISE;end
            STOP_RISE:if(scl_sync)begin timer<=H-1;state<=STOP_HIGH;end
            STOP_HIGH:if(timer==0)begin sda_low<=0;timer<=H-1;state<=STOP_FREE;end
            STOP_FREE:if(timer==0)begin busy<=0;done<=1;state<=IDLE;end
            default:begin scl_low<=0;sda_low<=0;busy<=0;done<=1;error<=7;state<=IDLE;end
            endcase
        end
    end
endmodule
