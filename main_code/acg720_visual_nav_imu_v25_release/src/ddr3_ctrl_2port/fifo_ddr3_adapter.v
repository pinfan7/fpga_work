// Two DDR banks. Ownership is released only after display blanking and
// return of every old-bank read. Addresses use 16-bit words, 8 per command.
module fifo_ddr3_adapter #(
    parameter integer FRAME_PIXELS=640*480,
    parameter integer RESET_CYCLES=200
)(
    input ui_clk,rst_n,init_calib_complete,app_rdy,app_wdf_rdy,
    input app_rd_data_valid,input [127:0] app_rd_data,
    output [27:0] app_addr,output app_en,app_wdf_wren,app_wdf_end,
    output [2:0] app_cmd,output [127:0] app_wdf_data,
    input rd_load,wr_load,
    input [27:0] app_addr_rd_min,app_addr_rd_max,
    input [7:0] rd_bust_len,
    input [27:0] app_addr_wr_min,app_addr_wr_max,
    input [7:0] wr_bust_len,
    input wr_clk,wfifo_wren,input [15:0] wfifo_din,
    output wrfifo_full,
    input rd_clk,rfifo_rden,output rdfifo_empty,
    output [15:0] rfifo_dout,output video_frame_valid,
    output reg frame_commit_toggle
);
    localparam integer WORDS=FRAME_PIXELS/8;
    reg permit,ack_toggle;
    (* syn_preserve=1 *) reg permit_m,permit_s,ack_m,ack_s;
    reg ack_seen,waiting,collecting,wr_vs_d;
    reg [27:0] pixel_count;
    reg capture_bad,start_toggle,done_toggle,done_bad;
    wire [9:0] wfifo_rcount,rfifo_wcount;
    wire [127:0] wfifo_dout;
    wire wfifo_empty;
    reg [15:0] write_reset_count,read_reset_count;
    wire write_fifo_reset=!rst_n || write_reset_count!=0;
    wire read_fifo_reset=!rst_n || read_reset_count!=0;
    wire accept_pixel=collecting && wfifo_wren && pixel_count<FRAME_PIXELS && !wrfifo_full;

    // Stable payloads accompany toggle handshakes across clock domains.
    always @(posedge wr_clk or negedge rst_n) begin
        if(!rst_n)begin
            permit_m<=0;permit_s<=0;ack_m<=0;ack_s<=0;ack_seen<=0;
            waiting<=0;collecting<=0;wr_vs_d<=0;pixel_count<=0;
            capture_bad<=0;start_toggle<=0;done_toggle<=0;done_bad<=0;
        end else begin
            permit_m<=permit;permit_s<=permit_m;ack_m<=ack_toggle;ack_s<=ack_m;
            wr_vs_d<=wr_load;
            if(ack_s!=ack_seen)begin ack_seen<=ack_s;waiting<=0;end
            if(!wr_load && wr_vs_d && collecting)begin
                collecting<=0;done_bad<=capture_bad || pixel_count!=FRAME_PIXELS;
                done_toggle<=~done_toggle;
            end
            if(wr_load && !wr_vs_d && !collecting && !waiting && permit_s)begin
                collecting<=1;waiting<=1;pixel_count<=0;capture_bad<=0;
                start_toggle<=~start_toggle;
            end
            if(collecting && wfifo_wren)begin
                if(pixel_count<FRAME_PIXELS+1)pixel_count<=pixel_count+1'b1;
                if(wrfifo_full || pixel_count>=FRAME_PIXELS)capture_bad<=1;
            end
        end
    end

    (* syn_preserve=1 *) reg start_m,start_s,done_m,done_s,bad_m,bad_s;
    (* syn_preserve=1 *) reg vs_m,vs_s;
    reg start_seen,done_seen,vs_d;
    reg write_bank,read_bank,read_valid,pending_frame;
    reg capture_active,capture_ended,capture_rejected,release_wait;
    reg [27:0] write_words,read_words;
    reg [10:0] outstanding;
    wire frame_edge=vs_s && !vs_d;
    wire [11:0] read_occupancy={2'b0,rfifo_wcount}+{1'b0,outstanding};
    wire want_write=capture_active && !capture_rejected && write_reset_count==0 &&
                    write_words<WORDS && wfifo_rcount!=0 && !wfifo_empty;
    wire want_read=read_valid && read_reset_count==0 && !frame_edge &&
                   read_words<WORDS && read_occupancy<128;
    wire select_read=want_read && (!want_write || read_occupancy<32);
    wire take_read=init_calib_complete && select_read && app_rdy;
    wire take_write=init_calib_complete && !select_read && want_write && app_rdy && app_wdf_rdy;
    assign app_en=take_read || take_write;
    assign app_cmd=select_read?3'd1:3'd0;
    assign app_addr=select_read ? ((read_bank?FRAME_PIXELS:0)+(read_words<<3)) :
                                 ((write_bank?FRAME_PIXELS:0)+(write_words<<3));
    assign app_wdf_wren=take_write;
    assign app_wdf_end=take_write;
    assign app_wdf_data=wfifo_dout;
    wire fifo_return=app_rd_data_valid && read_reset_count==0 && !frame_edge;

    always @(posedge ui_clk or negedge rst_n)begin
        if(!rst_n)begin
            start_m<=0;start_s<=0;done_m<=0;done_s<=0;bad_m<=0;bad_s<=0;
            vs_m<=0;vs_s<=0;vs_d<=0;start_seen<=0;done_seen<=0;
            permit<=0;ack_toggle<=0;write_bank<=1;read_bank<=0;read_valid<=0;
            pending_frame<=0;capture_active<=0;capture_ended<=0;capture_rejected<=0;
            frame_commit_toggle<=0;
            release_wait<=0;write_words<=0;read_words<=0;outstanding<=0;
            write_reset_count<=RESET_CYCLES;read_reset_count<=RESET_CYCLES;
        end else begin
            start_m<=start_toggle;start_s<=start_m;
            done_m<=done_toggle;done_s<=done_m;bad_m<=done_bad;bad_s<=bad_m;
            vs_m<=rd_load;vs_s<=vs_m;vs_d<=vs_s;
            if(write_reset_count!=0)write_reset_count<=write_reset_count-1'b1;
            if(read_reset_count!=0)read_reset_count<=read_reset_count-1'b1;
            if(init_calib_complete && !capture_active && !pending_frame &&
               !release_wait && write_reset_count==0)permit<=1;
            if(start_s!=start_seen)begin
                start_seen<=start_s;permit<=0;capture_active<=1;capture_ended<=0;
                capture_rejected<=0;write_words<=0;
            end
            if(done_s!=done_seen)begin
                done_seen<=done_s;capture_ended<=1;capture_rejected<=bad_s;
            end
            if(take_write)write_words<=write_words+1'b1;
            if(capture_active && capture_ended)begin
                if(capture_rejected)begin
                    capture_active<=0;capture_ended<=0;
                    write_reset_count<=RESET_CYCLES;release_wait<=1;
                end else if(write_words==WORDS)begin
                    // Publish only after every write command/data handshake.
                    capture_active<=0;capture_ended<=0;pending_frame<=1;
                end
            end
            case({take_read,app_rd_data_valid})
                2'b10:outstanding<=outstanding+1'b1;
                2'b01:if(outstanding!=0)outstanding<=outstanding-1'b1;
                default:outstanding<=outstanding;
            endcase
            if(take_read)read_words<=read_words+1'b1;
            if(frame_edge)begin
                read_reset_count<=RESET_CYCLES;read_words<=0;
                if(pending_frame)begin
                    frame_commit_toggle<=~frame_commit_toggle;
                    read_bank<=write_bank;write_bank<=read_bank;
                    read_valid<=1;pending_frame<=0;release_wait<=1;
                end
            end
            // Flush and discard old reads even if latency exceeds reset time.
            if(read_reset_count==1 && outstanding!=0)read_reset_count<=1;
            if(release_wait && !frame_edge && read_reset_count==0 &&
               outstanding==0 && write_reset_count==0)begin
                release_wait<=0;ack_toggle<=~ack_toggle;permit<=1;
            end
        end
    end

    (* syn_preserve=1 *) reg valid_m,valid_s;
    reg rd_vs_d,underflow;
    wire valid_ui=read_valid && read_reset_count==0;
    always @(posedge rd_clk or negedge rst_n)begin
        if(!rst_n)begin valid_m<=0;valid_s<=0;rd_vs_d<=0;underflow<=0;end
        else begin
            valid_m<=valid_ui;valid_s<=valid_m;rd_vs_d<=rd_load;
            if(rd_load && !rd_vs_d)underflow<=0;
            else if(rfifo_rden && (!valid_s || rdfifo_empty))underflow<=1;
        end
    end
    // Blank remainder of raster after underflow rather than shift all pixels.
    assign video_frame_valid=valid_s && !underflow && !rdfifo_empty;
    rd_data_fifo rd_data_fifo(
        .Data(app_rd_data),.Reset(read_fifo_reset),.WrClk(ui_clk),.RdClk(rd_clk),
        .WrEn(fifo_return),.RdEn(rfifo_rden && valid_s && !underflow && !rdfifo_empty),
        .Wnum(rfifo_wcount),.Rnum(),.Almost_Empty(),.Almost_Full(),
        .Q(rfifo_dout),.Empty(rdfifo_empty),.Full());
    wr_data_fifo wr_data_fifo(
        .Data(wfifo_din),.Reset(write_fifo_reset),.WrClk(wr_clk),.RdClk(ui_clk),
        .WrEn(accept_pixel),.RdEn(take_write),.Wnum(),.Rnum(wfifo_rcount),
        .Almost_Empty(),.Almost_Full(),.Q(wfifo_dout),.Empty(wfifo_empty),.Full(wrfifo_full));
endmodule
