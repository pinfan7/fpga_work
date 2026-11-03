`timescale 1ns/1ps
// Register transactions using the EXACT byte engine proven on hardware in V10.
// The vendor engine is unmodified. No clock-stretch support is claimed.
module imu_vendor_i2c_master #(
    parameter integer CLK_HZ=50000000,
    parameter integer I2C_HZ=25000
)(
    input clk,rst_n,start,
    input [6:0] address,input read_op,
    input [7:0] reg_address,write_data,input [3:0] read_count,
    inout scl,sda,
    output reg busy,done,output reg [3:0] error,
    output reg [95:0] rx_data,
    output [1:0] drive_low,
    output reg [7:0] address_echo,output reg [3:0] ack_map,
    output reg [7:0] failure_detail
);
    localparam [3:0] IDLE=0,FREE=1,ADDR_W=2,POINTER=3,WRITE=4,
        ADDR_R=5,READ=6,ABORT_LOW=7,ABORT_HIGH=8,ABORT_RELEASE=9;
    localparam integer FREE_CYCLES=CLK_HZ/I2C_HZ;
    localparam integer STOP_CYCLES=CLK_HZ/I2C_HZ/2;
    localparam integer TIMEOUT_CYCLES=CLK_HZ/50;
    reg [3:0] state,remaining,completed_reads;
    reg [31:0] watchdog,free_count,stop_timer;
    reg [6:0] addr;
    reg rd,go,core_enable,force_scl_low,force_sda_low;
    reg [7:0] pointer,wdata,tx_byte;
    reg [5:0] command;
    wire byte_done,byte_nack,vendor_scl;
    wire [7:0] rx_byte;
    wire core_reset_n=rst_n && core_enable;
    assign scl=(rst_n && (force_scl_low || (busy && !vendor_scl))) ? 1'b0 : 1'bz;
    assign sda=(rst_n && force_sda_low) ? 1'b0 : 1'bz;
    assign drive_low={rst_n && (force_scl_low || (busy && !vendor_scl)),1'b0};
    (* syn_preserve=1 *) reg scl_meta,scl_sync,sda_meta,sda_sync;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)begin scl_meta<=1;scl_sync<=1;sda_meta<=1;sda_sync<=1;end
        else begin scl_meta<=scl;scl_sync<=scl_meta;sda_meta<=sda;sda_sync<=sda_meta;end
    i2c_bit_shift #(.SYS_CLOCK(CLK_HZ),.SCL_CLOCK(I2C_HZ)) vendor_core(
        .Clk(clk),.Rst_n(core_reset_n),.Cmd(command),.Go(go),
        .Rx_DATA(rx_byte),.Tx_DATA(tx_byte),.Trans_Done(byte_done),
        .ack_o(byte_nack),.i2c_sclk(vendor_scl),.i2c_sdat(sda));
    task launch;
        input [5:0] cmd;input [7:0] data;input [3:0] next_state;
        begin command<=cmd;tx_byte<=data;go<=1;state<=next_state;end
    endtask
    task finish;
        begin busy<=0;done<=1;go<=0;core_enable<=0;state<=IDLE;
            force_scl_low<=0;force_sda_low<=0;end
    endtask
    task abort_transaction;
        input [3:0] code;
        begin
            error<=code;core_enable<=0;go<=0;
            failure_detail<={state,completed_reads};
            // Generate a bounded STOP outside the byte engine on NACK/timeout.
            force_scl_low<=1;force_sda_low<=1;
            stop_timer<=STOP_CYCLES;state<=ABORT_LOW;
        end
    endtask
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;remaining<=0;completed_reads<=0;failure_detail<=0;
            watchdog<=0;free_count<=0;stop_timer<=0;
            addr<=0;rd<=0;pointer<=0;wdata<=0;tx_byte<=0;command<=0;
            go<=0;core_enable<=0;force_scl_low<=0;force_sda_low<=0;
            busy<=0;done<=0;error<=0;rx_data<=0;address_echo<=0;ack_map<=0;
        end else begin
            go<=0;done<=0;
            if(busy && watchdog!=0)watchdog<=watchdog-1'b1;
            if(stop_timer!=0)stop_timer<=stop_timer-1'b1;
            if(busy && state<ABORT_LOW && watchdog==0)
                abort_transaction(!scl_sync ? 4'd5 : !sda_sync ? 4'd6 : 4'd7);
            else case(state)
            IDLE:if(start)begin
                busy<=1;error<=0;rx_data<=0;ack_map<=0;completed_reads<=0;failure_detail<=0;
                // This is the commanded address, NOT a pad-sampled echo.
                address_echo<={address,1'b0};addr<=address;rd<=read_op;
                pointer<=reg_address;wdata<=write_data;remaining<=read_count;
                watchdog<=TIMEOUT_CYCLES;free_count<=0;core_enable<=1;
                state<=FREE;
                if(read_op && (read_count==0 || read_count>12))begin error<=4'd7;finish();end
            end
            FREE:if(!scl_sync || !sda_sync)free_count<=0;
                else if(free_count==FREE_CYCLES-1)launch(6'h03,{addr,1'b0},ADDR_W);
                else free_count<=free_count+1'b1;
            ADDR_W:if(byte_done)begin
                if(byte_nack)abort_transaction(4'd1);
                else begin ack_map[0]<=1;launch(6'h01,pointer,POINTER);end
            end
            POINTER:if(byte_done)begin
                if(byte_nack)abort_transaction(4'd2);
                else begin
                    ack_map[1]<=1;
                    if(rd)launch(6'h03,{addr,1'b1},ADDR_R);
                    else launch(6'h09,wdata,WRITE);
                end
            end
            WRITE:if(byte_done)begin
                if(byte_nack)begin error<=4'd3;failure_detail<={state,completed_reads};end
                else ack_map[2]<=1;
                finish();
            end
            ADDR_R:if(byte_done)begin
                if(byte_nack)abort_transaction(4'd4);
                else begin ack_map[3]<=1;launch(remaining==1 ? 6'h2c : 6'h14,8'd0,READ);end
            end
            READ:if(byte_done)begin
                completed_reads<=completed_reads+1'b1;
                rx_data<={rx_data[87:0],rx_byte};
                if(remaining==1)finish();
                else begin
                    remaining<=remaining-1'b1;
                    launch(remaining==2 ? 6'h2c : 6'h14,8'd0,READ);
                end
            end
            ABORT_LOW:if(stop_timer==0)begin
                force_scl_low<=0;stop_timer<=STOP_CYCLES;state<=ABORT_HIGH;
            end
            ABORT_HIGH:if(stop_timer==0)begin
                force_sda_low<=0;stop_timer<=STOP_CYCLES;state<=ABORT_RELEASE;
            end
            ABORT_RELEASE:if(stop_timer==0)finish();
            default:begin error<=4'd7;finish();end
            endcase
        end
    end
endmodule
