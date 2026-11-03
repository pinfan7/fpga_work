`timescale 1ns/1ps
// Independent discovery sequencer around the unmodified board-vendor byte core.
// Address probes and register-pointer writes only: never configures a sensor.
module imu_reference_discovery #(
    parameter integer CLK_HZ=50000000,
    parameter integer I2C_HZ=25000,
    parameter integer BOOT_CYCLES=CLK_HZ/10
)(
    input clk,rst_n,start,
    inout scl,sda,
    output reg busy,done,verified_icm,
    output reg [7:0] error,found_count,scan_address,
    output reg [127:0] records
);
    localparam [3:0] IDLE=0,BOOT=1,BUS_FREE=2,PROBE_WAIT=3,
        READ_ADDR_WAIT=4,POINTER_WAIT=5,READ_ADDR_R_WAIT=6,
        READ_BYTE_WAIT=7,NEXT_ADDRESS=8;
    localparam integer FREE_CYCLES=CLK_HZ/I2C_HZ;
    localparam integer TIMEOUT_CYCLES=CLK_HZ/50;
    reg [3:0] state;
    reg [31:0] timer,watchdog,free_count;
    reg [5:0] command;
    reg [7:0] tx_byte;
    reg go,core_enable,read_other,read_acked;
    reg [1:0] slot;
    wire byte_done,byte_nack,vendor_scl;
    wire [7:0] rx_byte;
    wire core_reset_n=rst_n && core_enable;
    // The vendor core's SCL is converted to open-drain; SDA already is open-drain.
    assign scl=(rst_n && busy && !vendor_scl) ? 1'b0 : 1'bz;
    (* syn_preserve=1 *) reg scl_meta,scl_sync,sda_meta,sda_sync;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)begin scl_meta<=1;scl_sync<=1;sda_meta<=1;sda_sync<=1;end
        else begin scl_meta<=scl;scl_sync<=scl_meta;sda_meta<=sda;sda_sync<=sda_meta;end
    i2c_bit_shift #(.SYS_CLOCK(CLK_HZ),.SCL_CLOCK(I2C_HZ)) vendor_core(
        .Clk(clk),.Rst_n(core_reset_n),.Cmd(command),.Go(go),
        .Rx_DATA(rx_byte),.Tx_DATA(tx_byte),.Trans_Done(byte_done),
        .ack_o(byte_nack),.i2c_sclk(vendor_scl),.i2c_sdat(sda));
    task launch;
        input [5:0] cmd;
        input [7:0] data;
        input [3:0] next_state;
        begin command<=cmd;tx_byte<=data;go<=1;watchdog<=TIMEOUT_CYCLES;state<=next_state;end
    endtask
    task stop_fault;
        input [7:0] code;
        begin error<=code;busy<=0;done<=1;core_enable<=0;go<=0;state<=IDLE;end
    endtask
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;busy<=0;done<=0;verified_icm<=0;error<=0;found_count<=0;
            scan_address<=8'h08;records<=0;timer<=0;watchdog<=0;free_count<=0;
            command<=0;tx_byte<=0;go<=0;core_enable<=0;read_other<=0;read_acked<=0;slot<=0;
        end else begin
            go<=0;
            if(timer!=0)timer<=timer-1'b1;
            if(busy && watchdog!=0)watchdog<=watchdog-1'b1;
            if(busy && state!=BOOT && watchdog==0)
                stop_fault(!scl_sync ? 8'h25 : !sda_sync ? 8'h26 : 8'h27);
            else case(state)
            IDLE:if(start)begin
                busy<=1;done<=0;verified_icm<=0;error<=0;found_count<=0;records<=0;
                scan_address<=8'h08;timer<=BOOT_CYCLES;core_enable<=1;
                watchdog<=TIMEOUT_CYCLES;free_count<=0;state<=BOOT;
            end
            BOOT:if(timer==0)begin watchdog<=TIMEOUT_CYCLES;free_count<=0;state<=BUS_FREE;end
            BUS_FREE:begin
                if(!scl_sync || !sda_sync)free_count<=0;
                else if(free_count==FREE_CYCLES-1)
                    launch(6'h0b,{scan_address[6:0],1'b0},PROBE_WAIT);
                else free_count<=free_count+1'b1;
            end
            PROBE_WAIT:if(byte_done)begin
                if(!byte_nack)begin
                    found_count<=found_count+1'b1;
                    if(found_count<4)begin
                        slot<=found_count[1:0];read_other<=0;read_acked<=1;
                        records[found_count[1:0]*32 +:32]<={scan_address,16'd0,8'h01};
                        launch(6'h03,{scan_address[6:0],1'b0},READ_ADDR_WAIT);
                    end else state<=NEXT_ADDRESS;
                end else state<=NEXT_ADDRESS;
            end
            READ_ADDR_WAIT:if(byte_done)begin
                read_acked<=read_acked && !byte_nack;
                launch(6'h01,read_other ? 8'h0f : 8'h75,POINTER_WAIT);
            end
            POINTER_WAIT:if(byte_done)begin
                read_acked<=read_acked && !byte_nack;
                launch(6'h03,{scan_address[6:0],1'b1},READ_ADDR_R_WAIT);
            end
            READ_ADDR_R_WAIT:if(byte_done)begin
                read_acked<=read_acked && !byte_nack;
                launch(6'h2c,8'd0,READ_BYTE_WAIT);
            end
            READ_BYTE_WAIT:if(byte_done)begin
                if(!read_other)begin
                    records[slot*32+16 +:8]<=rx_byte;
                    records[slot*32+1]<=read_acked;
                    if((scan_address==8'h68 || scan_address==8'h69) && read_acked && rx_byte==8'h47)
                        verified_icm<=1;
                    read_other<=1;read_acked<=1;
                    launch(6'h03,{scan_address[6:0],1'b0},READ_ADDR_WAIT);
                end else begin
                    records[slot*32+8 +:8]<=rx_byte;records[slot*32+2]<=read_acked;
                    state<=NEXT_ADDRESS;
                end
            end
            NEXT_ADDRESS:if(scan_address==8'h77)begin
                busy<=0;done<=1;core_enable<=0;error<=found_count==0 ? 8'h21 : 8'h00;state<=IDLE;
            end else begin
                scan_address<=scan_address+1'b1;free_count<=0;watchdog<=TIMEOUT_CYCLES;state<=BUS_FREE;
            end
            default:stop_fault(8'h27);
            endcase
        end
    end
endmodule
