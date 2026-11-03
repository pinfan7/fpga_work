`timescale 1ns/1ps
// IMU-only, open-drain pads. All SDA sampling uses synchronised pad input.
// Quarter-period phases, stretch wait, verified STOP, bounded bus clear.
module imu_recovery_i2c_master #(parameter integer CLK_HZ=50000000,I2C_HZ=25000)(
    input clk,rst_n,start,input [6:0] address,input read_op,
    input [7:0] reg_address,write_data,input [3:0] read_count,
    inout scl,sda,output reg busy,done,output reg [3:0] error,
    output reg [95:0] rx_data,output [1:0] drive_low,
    output reg [7:0] address_echo,output reg [3:0] ack_map,
    output reg [7:0] failure_detail
);
    localparam integer Q=CLK_HZ/I2C_HZ/4,TIMEOUT=CLK_HZ/50,FREE_LIMIT=CLK_HZ/1000;
    localparam [3:0] IDLE=0,FREE=1,START_BIT=2,TX=3,ACK_BIT=4,
        RX=5,MASTER_ACK=6,STOP_BIT=7,CLEAR=8;
    reg [3:0] state,remaining,completed_reads,clear_count;
    reg [1:0] phase,stage; reg [2:0] bit_index;
    reg [31:0] divider,watchdog,free_count,free_age,stop_age;
    reg scl_low,sda_low,rd,clear_attempted,clearing_stop;
    reg [6:0] addr; reg [7:0] pointer,wdata,shift,received;
    (* syn_preserve=1 *) reg scl_meta,scl_sync,sda_meta,sda_sync;
    assign scl=(rst_n && scl_low) ? 1'b0 : 1'bz;
    assign sda=(rst_n && sda_low) ? 1'b0 : 1'bz;
    assign drive_low={rst_n && scl_low,rst_n && sda_low};
    wire tick=divider==Q-1;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)begin scl_meta<=1;scl_sync<=1;sda_meta<=1;sda_sync<=1;end
        else begin scl_meta<=scl;scl_sync<=scl_meta;sda_meta<=sda;sda_sync<=sda_meta;end
    task finish;
        begin busy<=0;done<=1;state<=IDLE;scl_low<=0;sda_low<=0;end
    endtask
    task stop_with_error; input [3:0] code;
        begin error<=code;failure_detail<={state,completed_reads};
            state<=STOP_BIT;phase<=0;stop_age<=0;clearing_stop<=0;end
    endtask
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;phase<=0;stage<=0;remaining<=0;completed_reads<=0;bit_index<=7;
            clear_count<=0;divider<=0;watchdog<=0;free_count<=0;free_age<=0;stop_age<=0;
            scl_low<=0;sda_low<=0;rd<=0;clear_attempted<=0;clearing_stop<=0;
            addr<=0;pointer<=0;wdata<=0;shift<=0;received<=0;busy<=0;done<=0;error<=0;
            rx_data<=0;address_echo<=0;ack_map<=0;failure_detail<=0;
        end else begin
            done<=0;
            if(!busy || tick)divider<=0;else divider<=divider+1'b1;
            if(busy && watchdog!=0)watchdog<=watchdog-1'b1;
            if(state==STOP_BIT)stop_age<=stop_age+1'b1;
            if(busy && state!=STOP_BIT && watchdog==0)
                stop_with_error(!scl_sync ? 4'd5 : !sda_sync ? 4'd6 : 4'd7);
            else if(state==STOP_BIT && stop_age>=FREE_LIMIT)begin
                if(error==0)begin error<=!scl_sync ? 4'd5 : 4'd6;failure_detail<={state,completed_reads};end
                finish();
            end else case(state)
            IDLE:if(start)begin
                busy<=1;error<=0;ack_map<=0;failure_detail<=0;rx_data<=0;
                addr<=address;rd<=read_op;pointer<=reg_address;wdata<=write_data;
                address_echo<={address,1'b0};remaining<=read_count;completed_reads<=0;
                watchdog<=TIMEOUT;free_count<=0;free_age<=0;clear_attempted<=0;
                clearing_stop<=0;scl_low<=0;sda_low<=0;state<=FREE;phase<=0;
                if(read_op && (read_count==0 || read_count>12))begin error<=7;finish();end
            end
            FREE:begin
                free_age<=free_age+1'b1;
                if(!scl_sync || !sda_sync)free_count<=0;
                else if(free_count==Q*4-1)begin
                    shift<={addr,1'b0};stage<=0;phase<=0;state<=START_BIT;
                end else free_count<=free_count+1'b1;
                if(free_age>=FREE_LIMIT && !clear_attempted && scl_sync && !sda_sync)begin
                    clear_attempted<=1;clear_count<=0;phase<=0;state<=CLEAR;
                end
            end
            START_BIT:if(tick)case(phase)
                0:begin sda_low<=0;phase<=1;end
                1:begin scl_low<=0;phase<=2;end
                2:if(scl_sync)begin sda_low<=1;phase<=3;end
                3:begin scl_low<=1;bit_index<=7;phase<=0;state<=TX;end
            endcase
            TX:if(tick)case(phase)
                0:begin sda_low<=!shift[bit_index];phase<=1;end
                1:begin scl_low<=0;phase<=2;end
                2:if(scl_sync)phase<=3;
                3:begin scl_low<=1;phase<=0;
                    if(bit_index==0)state<=ACK_BIT;else bit_index<=bit_index-1'b1;end
            endcase
            ACK_BIT:if(tick)case(phase)
                0:begin sda_low<=0;phase<=1;end
                1:begin scl_low<=0;phase<=2;end
                2:if(scl_sync)begin received<={7'd0,sda_sync};phase<=3;end
                3:begin scl_low<=1;phase<=0;bit_index<=7;
                    if(received[0])stop_with_error(stage==0 ? 4'd1 : stage==1 ? 4'd2 : stage==2 ? 4'd3 : 4'd4);
                    else case(stage)
                        0:begin ack_map[0]<=1;shift<=pointer;stage<=1;state<=TX;end
                        1:begin ack_map[1]<=1;
                            if(rd)begin shift<={addr,1'b1};stage<=3;state<=START_BIT;end
                            else begin shift<=wdata;stage<=2;state<=TX;end
                        end
                        2:begin ack_map[2]<=1;state<=STOP_BIT;stop_age<=0;end
                        3:begin ack_map[3]<=1;received<=0;state<=RX;end
                    endcase
                end
            endcase
            RX:if(tick)case(phase)
                0:begin sda_low<=0;phase<=1;end
                1:begin scl_low<=0;phase<=2;end
                2:if(scl_sync)begin received<={received[6:0],sda_sync};phase<=3;end
                3:begin scl_low<=1;phase<=0;
                    if(bit_index==0)begin rx_data<={rx_data[87:0],received};
                        completed_reads<=completed_reads+1'b1;state<=MASTER_ACK;end
                    else bit_index<=bit_index-1'b1;
                end
            endcase
            MASTER_ACK:if(tick)case(phase)
                0:begin sda_low<=remaining!=1;phase<=1;end
                1:begin scl_low<=0;phase<=2;end
                2:if(scl_sync)phase<=3;
                3:begin scl_low<=1;phase<=0;bit_index<=7;
                    if(remaining==1)begin state<=STOP_BIT;stop_age<=0;end
                    else begin remaining<=remaining-1'b1;received<=0;state<=RX;end
                end
            endcase
            CLEAR:if(tick)case(phase)
                0:begin scl_low<=1;sda_low<=0;phase<=1;end
                1:begin scl_low<=0;phase<=2;end
                2:if(scl_sync)phase<=3;
                3:begin scl_low<=1;phase<=0;
                    if(clear_count==8 || sda_sync)begin state<=STOP_BIT;stop_age<=0;clearing_stop<=1;end
                    else clear_count<=clear_count+1'b1;
                end
            endcase
            STOP_BIT:if(tick)case(phase)
                0:begin scl_low<=1;sda_low<=1;phase<=1;end
                1:begin scl_low<=0;phase<=2;end
                2:if(scl_sync)begin sda_low<=0;phase<=3;end
                3:if(scl_sync && sda_sync)begin
                    if(clearing_stop)begin clearing_stop<=0;free_count<=0;free_age<=0;state<=FREE;end
                    else finish();
                end
            endcase
            default:stop_with_error(4'd7);
            endcase
        end
    end
endmodule
