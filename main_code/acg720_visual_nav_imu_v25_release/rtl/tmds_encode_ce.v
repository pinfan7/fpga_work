`timescale 1ns/1ps
// Three CE stages. Every register advances only on pixel CE.
// DVI transition-minimization and running-disparity encoding.
module tmds_encode_ce(input clk,rst_n,ce,input [7:0] din,
                      input c0,c1,de,output reg [9:0] dout);
    reg [7:0] din_q;
    reg [3:0] n1d,n1q,n0q;
    reg [1:0] de_q,c0_q,c1_q;
    reg [8:0] qm_q;
    reg [4:0] disparity;
    wire use_xnor=(n1d>4) || ((n1d==4) && !din_q[0]);
    wire [8:0] qm;
    assign qm[0]=din_q[0];
    genvar i;
    generate for(i=1;i<8;i=i+1) begin: chain
        assign qm[i]=qm[i-1] ^ din_q[i] ^ use_xnor;
    end endgenerate
    assign qm[8]=!use_xnor;
    wire [3:0] ones=qm[0]+qm[1]+qm[2]+qm[3]+qm[4]+qm[5]+qm[6]+qm[7];
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            din_q<=0;n1d<=0;n1q<=0;n0q<=8;de_q<=0;c0_q<=0;c1_q<=0;
            qm_q<=0;disparity<=0;dout<=10'b1101010100;
        end else if(ce) begin
            din_q<=din;
            n1d<=din[0]+din[1]+din[2]+din[3]+din[4]+din[5]+din[6]+din[7];
            qm_q<=qm;n1q<=ones;n0q<=4'd8-ones;
            de_q<={de_q[0],de};c0_q<={c0_q[0],c0};c1_q<={c1_q[0],c1};
            if(!de_q[1]) begin
                disparity<=0;
                case({c1_q[1],c0_q[1]})
                    0:dout<=10'b1101010100;
                    1:dout<=10'b0010101011;
                    2:dout<=10'b0101010100;
                    3:dout<=10'b1010101011;
                endcase
            end else if(disparity==0 || n1q==n0q) begin
                dout<={~qm_q[8],qm_q[8],(qm_q[8]?qm_q[7:0]:~qm_q[7:0])};
                if(qm_q[8]) disparity<=disparity+n1q-n0q;
                else disparity<=disparity+n0q-n1q;
            end else if((!disparity[4] && n1q>n0q) || (disparity[4] && n0q>n1q)) begin
                dout<={1'b1,qm_q[8],~qm_q[7:0]};
                disparity<=disparity+{qm_q[8],1'b0}+n0q-n1q;
            end else begin
                dout<={1'b0,qm_q[8],qm_q[7:0]};
                disparity<=disparity-{~qm_q[8],1'b0}+n1q-n0q;
            end
        end
    end
endmodule
