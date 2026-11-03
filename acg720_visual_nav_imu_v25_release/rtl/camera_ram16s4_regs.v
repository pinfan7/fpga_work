`timescale 1ns/1ps
// Functional equivalent of Gowin RAM16S4, explicitly using flip-flops.
// Required by the vendor's old FIFO netlist on GW5AT-60B / Gowin 1.9.12.03.
// Match INIT bit ordering, synchronous writes and asynchronous read exactly.
module camera_ram16s4_regs #(
    parameter [15:0] INIT_0=0, INIT_1=0, INIT_2=0, INIT_3=0
)(input CLK, input WRE, input [3:0] AD, input [3:0] DI, output [3:0] DO);
    genvar i;
    wire [3:0] words [0:15];
    generate for(i=0;i<16;i=i+1) begin: word_reg
        reg [3:0] data = {INIT_3[i],INIT_2[i],INIT_1[i],INIT_0[i]};
        always @(posedge CLK) if(WRE && AD == i) data <= DI;
        assign words[i] = data;
    end endgenerate
    assign DO = words[AD];
endmodule
