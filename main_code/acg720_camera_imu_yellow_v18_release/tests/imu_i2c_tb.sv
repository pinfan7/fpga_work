`timescale 1ns/1ps
module imu_i2c_tb;
    reg clk=0;always #500 clk=~clk;
    reg rst_n=0,start=0,rd=1;reg [3:0] length=1;
    tri1 scl,sda;reg slave_low=0,stuck_sda=0,stuck_scl=0;
    assign sda=(slave_low || stuck_sda) ? 1'b0 : 1'bz;
    assign scl=stuck_scl ? 1'b0 : 1'bz;
    wire busy,done;wire [3:0] error;wire [95:0] data;wire [3:0] acks;
    imu_recovery_i2c_master #(.CLK_HZ(1000000),.I2C_HZ(25000)) dut(
        .clk(clk),.rst_n(rst_n),.start(start),.address(7'h68),.read_op(rd),
        .reg_address(8'h1f),.write_data(8'h55),.read_count(length),
        .scl(scl),.sda(sda),.busy(busy),.done(done),.error(error),.rx_data(data),
        .drive_low(),.address_echo(),.ack_map(acks),.failure_detail());
    task wait_start;reg seen;begin
        seen=0;while(!seen)begin @(negedge sda);if(scl===1'b1)seen=1;end
    end endtask
    task receive_byte(output reg [7:0] value);integer i;begin
        for(i=7;i>=0;i=i-1)begin @(posedge scl);value[i]=sda;end
    end endtask
    task ack(input bit nack);begin
        @(negedge scl);slave_low=!nack;@(posedge scl);@(negedge scl);slave_low=0;
    end endtask
    task send_byte(input reg [7:0] value,input bit last);integer i;begin
        for(i=7;i>=0;i=i-1)begin slave_low=!value[i];@(posedge scl);@(negedge scl);end
        slave_low=0;@(posedge scl);
        if(sda!==last)$fatal(1,"master ACK/NACK wrong: last=%b sda=%b",last,sda);
        @(negedge scl);
    end endtask
    task slave(input integer n,input bit write_op);reg [7:0] v;integer j;begin
        wait_start();receive_byte(v);if(v!=8'hd0)$fatal(1,"write address %h",v);ack(0);
        receive_byte(v);if(v!=8'h1f)$fatal(1,"register %h",v);ack(0);
        if(write_op)begin receive_byte(v);if(v!=8'h55)$fatal(1,"write value %h",v);ack(0);end
        else begin
            wait_start();receive_byte(v);if(v!=8'hd1)$fatal(1,"read address %h",v);ack(0);
            for(j=0;j<n;j=j+1)send_byte(8'h40+j,j==n-1);
        end
        begin : wait_stop
            reg stopped;stopped=0;
            while(!stopped)begin @(posedge sda);if(scl===1'b1)stopped=1;end
        end
    end endtask
    task command(input bit readmode,input integer n,input integer err);begin
        @(negedge clk);rd=readmode;length=n;start=1;@(negedge clk);start=0;
        wait(done);if(error!==err)$fatal(1,"error expected %d got %d",err,error);
        repeat(4)@(negedge clk);if(dut.scl_low || dut.sda_low)$fatal(1,"pads not released");
    end endtask
    reg [7:0] v;integer pulses;reg [95:0] expected;integer k;
    initial begin
        repeat(5)@(negedge clk);rst_n=1;
        $display("test single");fork command(1,1,0);slave(1,0);join
        if(data!=96'h40 || acks!=4'b1011)$fatal(1,"single read mismatch %h %h",data,acks);
        $display("test burst");fork command(1,12,0);slave(12,0);join
        expected=0;for(k=0;k<12;k=k+1)expected={expected[87:0],8'h40+k[7:0]};
        if(data!==expected)$fatal(1,"burst mismatch %h %h",data,expected);
        $display("test write");fork command(0,1,0);slave(1,1);join
        if(acks!=4'b0111)$fatal(1,"write ACK map");
        // Address NACK exits with STOP and bounded done.
        command(1,1,1);
        // SDA held by interrupted slave: release after three clear clocks.
        $display("test clear");stuck_sda=1;
        fork
            command(1,1,0);
            begin repeat(3)@(posedge scl);@(negedge scl);stuck_sda=0;slave(1,0);end
        join
        $display("clear passed");
        // Stretch SCL for 150us during a real data transaction.
        fork
            command(1,1,0);slave(1,0);
            begin wait_start();@(negedge scl);stuck_scl=1;#150000;stuck_scl=0;end
        join
        $display("stretch passed");
        // Permanent SDA low: nine clear pulses only, then explicit error.
        stuck_sda=1;pulses=0;
        command(1,1,6);if(dut.clear_count!=8)$fatal(1,"clear not bounded at nine");
        stuck_sda=0;repeat(5)@(negedge clk);
        stuck_scl=1;command(1,1,5);stuck_scl=0;
        fork command(1,1,0);slave(1,0);join
        $display("PASS imu_i2c_tb: read/write,12-byte burst,final NACK,STOP,address NACK,bus clear,stretch,stuck SDA/SCL,recovery");$finish;
    end
    initial begin #100000000;$fatal(1,"test timeout state=%d",dut.state);end
endmodule
