`timescale 1ns/1ps
// Diagnostic 25 MHz pixels -> atomic FIFO -> single 125 MHz encoding/serialization.
// The clocks must have exactly 1:5 frequency ratio (same PLL reference). The FIFO
// absorbs reset/phase differences, NOT sustained frequency mismatch.
module hdmi_stream_tx(
    input pixel_clk,pixel_rst_n,fast_clk,fast_rst_n,
    input [23:0] rgb,input hs,vs,de,
    output tmds_clk_p,tmds_clk_n,output [2:0] tmds_data_p,tmds_data_n,
    output overflow_pixel,output reg underrun_fast
);
    (* syn_preserve=1 *) reg [4:0] phase;
    reg started,token_valid;
    wire full,empty,fifo_underflow;
    wire [5:0] available;
    wire [26:0] token;
    wire read_word=phase[0] && started;
    wire encode_ce=phase[1];
    wire load_word=phase[3];
    video_token_fifo fifo(.wclk(pixel_clk),.wrst_n(pixel_rst_n),.wen(1'b1),
        .wdata({hs,vs,de,rgb}),.full(full),.overflow(overflow_pixel),
        .rclk(fast_clk),.rrst_n(fast_rst_n),.ren(read_word),
        .rdata(token),.empty(empty),.available(available),.underflow(fifo_underflow));
    always @(posedge fast_clk or negedge fast_rst_n)begin
        if(!fast_rst_n)begin phase<=5'b00001;started<=0;token_valid<=0;underrun_fast<=0;end
        else begin
            phase<={phase[3:0],phase[4]};
            if(!started && available>=8)started<=1;
            if(phase[0])begin
                token_valid<=started && !empty;
                if(started && empty)underrun_fast<=1;
            end
        end
    end
    // Until primed, emit valid blanking controls instead of unknown memory values.
    wire [26:0] pixel=token_valid?token:{1'b1,1'b1,1'b0,24'd0};
    wire [9:0] blue,green,red;
    tmds_encode_ce eb(.clk(fast_clk),.rst_n(fast_rst_n),.ce(encode_ce),
        .din(pixel[7:0]),.c0(pixel[26]),.c1(pixel[25]),.de(pixel[24]),.dout(blue));
    tmds_encode_ce eg(.clk(fast_clk),.rst_n(fast_rst_n),.ce(encode_ce),
        .din(pixel[15:8]),.c0(1'b0),.c1(1'b0),.de(pixel[24]),.dout(green));
    tmds_encode_ce er(.clk(fast_clk),.rst_n(fast_rst_n),.ce(encode_ce),
        .din(pixel[23:16]),.c0(1'b0),.c1(1'b0),.de(pixel[24]),.dout(red));
    stream_tmds_lane b(.clk(fast_clk),.rst_n(fast_rst_n),.load_word(load_word),.word(blue),.p(tmds_data_p[0]),.n(tmds_data_n[0]));
    stream_tmds_lane g(.clk(fast_clk),.rst_n(fast_rst_n),.load_word(load_word),.word(green),.p(tmds_data_p[1]),.n(tmds_data_n[1]));
    stream_tmds_lane r(.clk(fast_clk),.rst_n(fast_rst_n),.load_word(load_word),.word(red),.p(tmds_data_p[2]),.n(tmds_data_n[2]));
    stream_tmds_lane c(.clk(fast_clk),.rst_n(fast_rst_n),.load_word(load_word),.word(10'b1111100000),.p(tmds_clk_p),.n(tmds_clk_n));
endmodule

// Same ODDR topology as the hardware-confirmed standalone 01 firmware.
module stream_tmds_lane(input clk,rst_n,load_word,input [9:0] word,output p,n);
    reg [4:0] ep,op,en,on;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin ep<=0;op<=0;en<=5'b11111;on<=5'b11111;end
        else if(load_word)begin
            ep<={word[8],word[6],word[4],word[2],word[0]};
            op<={word[9],word[7],word[5],word[3],word[1]};
            en<=~{word[8],word[6],word[4],word[2],word[0]};
            on<=~{word[9],word[7],word[5],word[3],word[1]};
        end else begin
            ep<={1'b0,ep[4:1]};op<={1'b0,op[4:1]};
            en<={1'b1,en[4:1]};on<={1'b1,on[4:1]};
        end
    end
    ODDR pos_out(.Q0(p),.Q1(),.D0(ep[0]),.D1(op[0]),.TX(1'b0),.CLK(clk));
    ODDR neg_out(.Q0(n),.Q1(),.D0(en[0]),.D1(on[0]),.TX(1'b0),.CLK(clk));
endmodule
