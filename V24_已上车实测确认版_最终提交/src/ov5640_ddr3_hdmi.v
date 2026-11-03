module gao_ov5640_ddr_hdmi(
    //System clock reset
    input       clk50M        , //系统时钟输入，50MHz
    input       reset_n       , //复位信号输入
    input       imu_recal_n,
    output signed [31:0] nav_yaw, output nav_imu_ok,nav_imu_update,
    input       source_key_n,
    input       osd_key_n,
    output      hdmi_oe,
    // Camera-only application: explicitly disable the installed drivetrain.
    output motor_left_pwm,motor_left_in1,motor_left_in2,
    output motor_right_pwm,motor_right_in1,motor_right_in2,motor_stby,servo_safe,
    output rgmii_gtxc,
    output [3:0] rgmii_txd,
    output rgmii_txen,eth_mdc,eth_rst_n,
    inout eth_mdio,
    inout       imu_scl,
    inout       imu_sda,

    //camera interface
    output      camera_sclk   ,
    inout       camera_sdat   ,
    input       camera_vsync  ,
    input       camera_href   ,
    input       camera_pclk   ,
    output      camera_xclk   ,
    input  [7:0]camera_data   ,
    output      camera_rst_n  ,
    output      camera_pwdn   ,
    //led
    output [7:0]led           ,

    //hdmi output
	output       	tmds_clk_p ,
	output       	tmds_clk_n ,
	output 	[2:0]   tmds_data_p,    //rgb
	output 	[2:0]   tmds_data_n,     //rgb

    //ddr
    output [13:0] O_ddr_addr        ,
    output [2:0] O_ddr_ba           ,
    output O_ddr_cs_n               ,
    output O_ddr_ras_n              ,
    output O_ddr_cas_n              ,
    output O_ddr_we_n               ,
    output O_ddr_clk                ,
    output O_ddr_clk_n              ,
    output O_ddr_cke                ,
    output O_ddr_odt                ,
    output O_ddr_reset_n            ,
    output [1:0] O_ddr_dqm          ,
    inout [15:0] IO_ddr_dq          ,
    inout [1:0] IO_ddr_dqs          ,
    inout [1:0] IO_ddr_dqs_n        

) /* synthesis syn_srlstyle="registers" */;
    //Set IMAGE Size  
    parameter IMAGE_WIDTH  = 640;
    parameter IMAGE_HEIGHT = 480;
    parameter IMU_AUTO_START = 0;
    assign motor_left_pwm=1'b0;
    assign motor_left_in1=1'b0;
    assign motor_left_in2=1'b0;
    assign motor_right_pwm=1'b0;
    assign motor_right_in1=1'b0;
    assign motor_right_in2=1'b0;
    assign motor_stby=1'b0;
    assign servo_safe=1'b0;

    // Guaranteed reset after SRAM load; S0 restarts the complete startup sequence.
    reg [19:0] startup_count=0;
    always @(posedge clk50M or negedge reset_n)
        if(!reset_n)startup_count<=0;
        else if(!(&startup_count))startup_count<=startup_count+1'b1;
    wire system_reset_n=reset_n && (&startup_count);
    assign hdmi_oe=system_reset_n;

    wire          loc_clk24m;
    wire          g_rst_p;
    wire          loc_clk400m;
    wire          pll_lock;
    wire          loc_clk50m;
    wire          pll_locked;
    //camera
    wire          camera_init_done;
    wire          pclk_bufg_o;

    //ddr
	wire fifo_aclr;
	wire fifo_wrreq;
	wire [15:0] fifo_wrdata;
    wire init_calib_complete;
    
    //hdmi
    wire clk_vga;//74.25m
	wire clk_vgax5;//371.25m
    wire hdmi_pll_lock;

    wire video_reset_n,serial_reset_n,camera_control_reset_n,camera_capture_reset_n;
    reset_release rv(.clk(clk_vga),.arst_n(system_reset_n && pll_locked),.rst_n(video_reset_n));
    reset_release rs(.clk(clk_vgax5),.arst_n(system_reset_n && pll_locked),.rst_n(serial_reset_n));
    reset_release rc(.clk(loc_clk50m),.arst_n(system_reset_n && pll_locked),.rst_n(camera_control_reset_n));
    reset_release rdvp(.clk(camera_pclk),.arst_n(system_reset_n && camera_init_done),.rst_n(camera_capture_reset_n));

	wire [11:0] hcount;
	wire [11:0] vcount;
	wire [23:0] disp_data;
 
	wire video_hs;
	wire video_vs;
	wire video_de;
	wire [7:0] video_r;
	wire [7:0] video_g;
	wire [7:0] video_b;


    assign  g_rst_p = ~pll_locked;

    assign led[3:0] = {~g_rst_p,camera_init_done,init_calib_complete,pll_lock};
    wire [23:0] overlay_rgb;
    wire overlay_hs,overlay_vs,overlay_de;
    wire [23:0] selected_rgb,transmit_rgb;
    wire transmit_hs,transmit_vs,transmit_de;
    wire [23:0] boxed_rgb;
    wire boxed_hs,boxed_vs,boxed_de;
    wire cup_valid;
    wire [11:0] cup_x0,cup_y0,cup_x1,cup_y1;
    yellow_cup_detector #(.WIDTH(IMAGE_WIDTH),.HEIGHT(IMAGE_HEIGHT)) cup_detect(
        .clk(clk_vga),.rst_n(video_reset_n),.enable(!pattern_mode),
        .rgb({video_r,video_g,video_b}),.de(video_de),.vs(video_vs),
        .box_valid(cup_valid),.box_x0(cup_x0),.box_y0(cup_y0),
        .box_x1(cup_x1),.box_y1(cup_y1));
    yellow_box_overlay cup_box(.clk(clk_vga),.rst_n(video_reset_n),
        .rgb_in(transmit_rgb),.hs_in(transmit_hs),.vs_in(transmit_vs),.de_in(transmit_de),
        .box_valid(cup_valid && !pattern_mode),.x0(cup_x0),.y0(cup_y0),.x1(cup_x1),.y1(cup_y1),
        .rgb_out(boxed_rgb),.hs_out(boxed_hs),.vs_out(boxed_vs),.de_out(boxed_de));
    wire pattern_mode,osd_mode;
    wire eth_ready,net_reset_n,net_pixel_reset_n;
    phy_reg_config network_phy(.Clk(loc_clk50m),.Rst_n(camera_control_reset_n),
        .Phy_rst_n(eth_rst_n),.Rd_Data(),.Phy_init_done(eth_ready),.mdc(eth_mdc),.mdio(eth_mdio));
    reset_release net_reset(.clk(clk_vgax5),.arst_n(system_reset_n && pll_locked && eth_ready),.rst_n(net_reset_n));
    reset_release net_pixels(.clk(clk_vga),.arst_n(system_reset_n && pll_locked && eth_ready),.rst_n(net_pixel_reset_n));
    wire net_start,net_done,net_req,net_drop;
    wire [7:0] net_byte,gmii_byte;
    wire [15:0] net_length;
    wire gmii_clock,gmii_enable;
    video_udp_mirror #(.WIDTH(IMAGE_WIDTH),.HEIGHT(IMAGE_HEIGHT)) network_pixels(
        .pclk(clk_vga),.prst_n(net_pixel_reset_n),.rgb(boxed_rgb),.de(boxed_de),.vs(boxed_vs),
        .nclk(clk_vgax5),.nrst_n(net_reset_n),.tx_done(net_done),.payload_req(net_req),
        .tx_start(net_start),.payload(net_byte),.payload_length(net_length),.dropped_line(net_drop));
    eth_udp_tx_gmii network_udp(.clk125m(clk_vgax5),.reset_p(!net_reset_n),
        .tx_en_pulse(net_start),.tx_done(net_done),
        .dst_mac(48'hffffffffffff),.src_mac(48'h020000000002),
        .dst_ip(32'hc0a80003),.src_ip(32'hc0a80002),.dst_port(16'd6102),.src_port(16'd5000),
        .data_length(net_length),.payload_req_o(net_req),.payload_dat_i(net_byte),
        .gmii_tx_clk(gmii_clock),.gmii_txen(gmii_enable),.gmii_txd(gmii_byte));
    gmii_to_rgmii network_output(.reset_n(net_reset_n),.gmii_tx_clk(gmii_clock),
        .gmii_txd(gmii_byte),.gmii_txen(gmii_enable),.gmii_txer(1'b0),
        .rgmii_tx_clk(rgmii_gtxc),.rgmii_txd(rgmii_txd),.rgmii_txen(rgmii_txen));
    video_ab_switch #(.DEBOUNCE_CYCLES(500000),.ACTIVE_WIDTH(640),.DEFAULT_OSD(1'b1))
    u_video_test(.clk(clk_vga),.rst_n(video_reset_n),
        .source_key_n(source_key_n),.osd_key_n(osd_key_n),
        .camera_rgb({video_r,video_g,video_b}),.hs(video_hs),.vs(video_vs),.de(video_de),
        .overlay_input_rgb(selected_rgb),.overlay_rgb(overlay_rgb),
        .overlay_hs(overlay_hs),.overlay_vs(overlay_vs),.overlay_de(overlay_de),
        .tx_rgb(transmit_rgb),.tx_hs(transmit_hs),.tx_vs(transmit_vs),.tx_de(transmit_de),
        .pattern_mode(pattern_mode),.osd_mode(osd_mode));
    wire [3:0] imu_leds;
    wire tx_overflow,tx_underrun;
    (* syn_preserve=1 *) reg tx_error_meta,tx_error_sync;
    always @(posedge clk50M or negedge system_reset_n)
        if(!system_reset_n)begin tx_error_meta<=0;tx_error_sync<=0;end
        else begin tx_error_meta<=tx_overflow || tx_underrun;tx_error_sync<=tx_error_meta;end
    assign led[7:4]=imu_leds | {1'b0,tx_error_sync,2'b00};
    camera_imu_subsystem #(.AUTO_START(IMU_AUTO_START)) u_camera_imu(
        .clk50(clk50M),.reset_n(system_reset_n),.pixel_clk(clk_vga),.pixel_locked(pll_locked),
        .recal_n(imu_recal_n),.imu_scl(imu_scl),.imu_sda(imu_sda),
        .rgb_in(selected_rgb),.hs_in(video_hs),.vs_in(video_vs),.de_in(video_de),
        .rgb_out(overlay_rgb),.hs_out(overlay_hs),.vs_out(overlay_vs),.de_out(overlay_de),
        .nav_yaw(nav_yaw),.nav_imu_ok(nav_imu_ok),.nav_imu_update(nav_imu_update),
        .status_led(imu_leds));
    

        camera_pll your_instance_name(
        .clkin(clk50M), //input  clkin
        .clkout0(clk_vgax5), //output  clkout0
        .clkout1(loc_clk24m), //output  clkout1
        .clkout2(loc_clk50m), //output  clkout2
        .lock(pll_locked), //output  lock
        .mdclk(clk50M), //input  mdclk
        .reset(~system_reset_n) //input reset
    );


    // Both outputs come from the same 750 MHz PLL VCO: 25 MHz and 125 MHz.
    // Atomic video FIFO handles their phase/reset relationship; no fabric 375 MHz.
    assign clk_vga=loc_clk24m;


    assign camera_xclk = loc_clk24m;
    //摄像头
    camera_init
    #(
        .SYS_CLOCK      ( 50_000_000   ),//系统时钟采用50MHz
        .SCL_CLOCK      ( 100_000      ),// V13: slower camera SCCB for extension cable
        .CAMERA_TYPE    ( "ov5640"     ),//"ov5640" or "ov7725"
        .IMAGE_TYPE     ( 0            ),// 0: RGB; 1: JPEG
        .IMAGE_WIDTH    ( IMAGE_WIDTH  ),// 图片宽度
        .IMAGE_HEIGHT   ( IMAGE_HEIGHT ),// 图片高度
        .IMAGE_FLIP_EN  ( 0            ),// 0: 不翻转，1: 上下翻转
        .IMAGE_MIRROR_EN( 0            ) // 0: 不镜像，1: 左右镜像
    )camera_init
    (
        .Clk         (loc_clk50m       ),
        .Rst_n       (camera_control_reset_n),
        .Init_Done   (camera_init_done ),
        .camera_rst_n(camera_rst_n     ),
        .camera_pwdn (camera_pwdn      ),
        .i2c_sclk    (camera_sclk      ),
        .i2c_sdat    (camera_sdat      )
    );
    
    assign pclk_bufg_o = camera_pclk;

    DVP_Capture DVP_Capture(
        .Rst_n      (camera_capture_reset_n),//input
        .PCLK       (pclk_bufg_o      ),//input
        .Vsync      (camera_vsync     ),//input
        .Href       (camera_href      ),//input
        .Data       (camera_data      ),//input     [7:0]

        .ImageState (fifo_aclr        ),//output reg
        .DataValid  (fifo_wrreq       ),//output
        .DataPixel  (fifo_wrdata      ),//output    [15:0]
        .DataHs     (                 ),//output
        .DataVs     (wr_load          ),//output
        .Xaddr      (                 ),//output    [11:0],start is 1
        .Yaddr      (                 )   //output    [11:0],start is 1
    );
    

    wire rdfifo_rden;
    wire wrfifo_clr;
    wire [15:0] wrfifo_din;
    wire wrfifo_clk;
    wire rdfifo_clr;
    wire rdfifo_clk;
    wire [15:0] rdfifo_dout;
    wire video_frame_valid;
    
    
    assign wrfifo_clr = fifo_aclr;
    assign wrfifo_clk = pclk_bufg_o;
    assign rdfifo_clr = 0;
    assign rdfifo_clk = clk_vga;
    assign wrfifo_wren = fifo_wrreq;
    assign wrfifo_din = fifo_wrdata;
    assign disp_data = video_frame_valid ? {rdfifo_dout[15:11],3'd0,rdfifo_dout[10:5],2'd0,rdfifo_dout[4:0],3'd0} : 24'd0;

    wire [27:0] app_addr_max = IMAGE_WIDTH * IMAGE_HEIGHT;
    wire [7:0] burst_len = IMAGE_WIDTH[10:3];

    wire mdrp_inc;     
    wire [1:0] mdrp_op;     
    wire [7:0] mdrp_wdata;     
    wire [7:0] mdrp_rdata;     
    wire pll_stop;     
    (* syn_preserve=1 *) reg pll_stop_meta,pll_stop_sync;
    reg pll_stop_r;
    reg wr;

    reg  pll_lock_r;
    reg  pll_lock_rr;

    always@(posedge clk50M or negedge system_reset_n)begin
        if(!system_reset_n)begin pll_lock_r<=0;pll_lock_rr<=0;end
        else begin pll_lock_r <= pll_lock;pll_lock_rr <= pll_lock_r;end
    end

    always@(posedge clk50M or negedge system_reset_n)
        if(!system_reset_n)begin pll_stop_meta<=0;pll_stop_sync<=0;pll_stop_r<=0;end
        else begin pll_stop_meta<=pll_stop;pll_stop_sync<=pll_stop_meta;pll_stop_r<=pll_stop_sync;end

    pll_mDRP_intf u_pll_mDRP_intf(
        .clk(clk50M),
        .rst_n(system_reset_n),
        .pll_lock(pll_lock_rr),
        .wr(wr),
        .mdrp_inc(mdrp_inc),
        .mdrp_op(mdrp_op),
        .mdrp_wdata(mdrp_wdata),
        .mdrp_rdata(mdrp_rdata)
    );    

    always@(posedge clk50M or negedge system_reset_n)
    if(!system_reset_n)
        wr <= 1'b0;
    else if(!pll_lock_rr)
        wr <= 1'b0;
    else if(~pll_stop_sync & pll_stop_r)begin
        wr <= 1'b1;
    end
    else if(pll_stop_sync & (~pll_stop_r))begin
        wr <= 1'b1;
    end
    else begin
        wr <= 1'b0;
    end

    ddr_pll ddr_pll(
        .clkin(clk50M), //input  clkin
        .clkout0(), //output  clkout0
        .clkout1(), //output  clkout1
        .clkout2(loc_clk400m), //output  clkout2
        .lock(pll_lock), //output  lock
        .mdopc(mdrp_op), //input [1:0] mdopc
        .mdainc(mdrp_inc), //input  mdainc
        .mdwdi(mdrp_wdata), //input [7:0] mdwdi
        .mdrdo(mdrp_rdata), //output [7:0] mdrdo
        .pll_init_bypass(1'b0), //input  pll_init_bypass
        .mdclk(clk50M), //input  mdclk
        .reset(~system_reset_n) //input reset
    );

    ddr3_ctrl_2port #(.FRAME_PIXELS(IMAGE_WIDTH*IMAGE_HEIGHT)) ddr3_ctrl_2port(
        .clk(clk50M)                     ,      //Same 50M reference as DDR PLL management
        .pll_lock(pll_lock)            ,
        .pll_stop(pll_stop)                ,
        .clk_200m(loc_clk400m)            ,      //DDR3参考时钟信号
        .sys_rst_n(system_reset_n)    ,      //Power-on and S0 reset
        .init_calib_complete(init_calib_complete) ,    //DDR初始化完成信号

        //用户接口
        .rd_load(video_vs)             ,   //输出源更新信号
        .wr_load(wr_load)             ,   //输入源更新信号
        .app_addr_rd_min(28'd0)     ,   //读DDR3的起始地址
        .app_addr_rd_max(app_addr_max)     ,   //读DDR3的结束地址
        .rd_bust_len(burst_len)         ,   //从DDR3中读数据时的突发长度
        .app_addr_wr_min(28'd0)     ,   //写DD3的起始地址
        .app_addr_wr_max(app_addr_max)     ,   //写DDR的结束地址
        .wr_bust_len(burst_len)         ,   //向DDR3中写数据时的突发长度

        .wr_clk(wrfifo_clk)             ,//wr_fifo的写时钟信号
        .wfifo_wren(wrfifo_wren)          , //wr_fifo的写使能信号
        .wfifo_din(wrfifo_din)           , //写入到wr_fifo中的数据
        .rd_clk(rdfifo_clk)              , //rd_fifo的读时钟信号
        .rfifo_rden(rdfifo_rden)          , //rd_fifo的读使能信号
        .rfifo_dout(rdfifo_dout)          , //rd_fifo读出的数据信号 
        .video_frame_valid(video_frame_valid),

        //DDR3   
        .ddr3_dq(IO_ddr_dq)             ,   //DDR3 数据
        .ddr3_dqs_n(IO_ddr_dqs_n)          ,   //DDR3 dqs负
        .ddr3_dqs_p(IO_ddr_dqs)          ,   //DDR3 dqs正  
        .ddr3_addr(O_ddr_addr)           ,   //DDR3 地址   
        .ddr3_ba(O_ddr_ba)             ,   //DDR3 banck 选择
        .ddr3_ras_n(O_ddr_ras_n)          ,   //DDR3 行选择
        .ddr3_cas_n(O_ddr_cas_n)          ,   //DDR3 列选择
        .ddr3_we_n(O_ddr_we_n)           ,   //DDR3 读写选择
        .ddr3_reset_n(O_ddr_reset_n)        ,   //DDR3 复位
        .ddr3_ck_p(O_ddr_clk)          ,   //DDR3 时钟正
        .ddr3_ck_n(O_ddr_clk_n)           ,   //DDR3 时钟负
        .ddr3_cke(O_ddr_cke)            ,   //DDR3 时钟使能
        .ddr3_cs_n(O_ddr_cs_n)           ,   //DDR3 片选
        .ddr3_dm(O_ddr_dqm)             ,   //DDR3_dm
        .ddr3_odt(O_ddr_odt)                //DDR3_odt   
    );
   

    //HDMI显示
    disp_driver disp_driver(
		.ClkDisp(clk_vga),
		.Rst_n(video_reset_n),
		.Data(disp_data),
		.DataReq(rdfifo_rden),
		.H_Addr(hcount),
		.V_Addr(vcount),
		.Disp_HS(video_hs),
		.Disp_VS(video_vs),
		.Disp_Red(video_r),
		.Disp_Green(video_g),
		.Disp_Blue(video_b),
        .Frame_Begin(),
		.Disp_DE(video_de),
		.Disp_PCLK()
	);
//	
	hdmi_stream_tx u_dvi_encoder(
		.pixel_clk(clk_vga),.pixel_rst_n(video_reset_n),
		.fast_clk(clk_vgax5),.fast_rst_n(serial_reset_n),
		.rgb(boxed_rgb),.hs(boxed_hs),.vs(boxed_vs),
		.de(boxed_de),// data enable, delayed with RGB by the box overlay
		.overflow_pixel(tx_overflow),.underrun_fast(tx_underrun),
		.tmds_clk_p(tmds_clk_p),
		.tmds_clk_n(tmds_clk_n),
		.tmds_data_p(tmds_data_p),//rgb
		.tmds_data_n(tmds_data_n) //rgb
	);

//	DVI_TX_Top DVI_TX_Top(
//		.I_rst_n(reset_n), //input I_rst_n
//		.I_serial_clk(clk_vga * 5), //input I_serial_clk
//		.I_rgb_clk(clk_vga), //input I_rgb_clk
//		.I_rgb_vs(video_vs), //input I_rgb_vs
//		.I_rgb_hs(video_hs), //input I_rgb_hs
//		.I_rgb_de(video_de), //input I_rgb_de
//		.I_rgb_r(video_r), //input [7:0] I_rgb_r
//		.I_rgb_g(video_g), //input [7:0] I_rgb_g
//		.I_rgb_b(video_b), //input [7:0] I_rgb_b
//		.O_tmds_clk_p(tmds_clk_p), //output O_tmds_clk_p
//		.O_tmds_clk_n(tmds_clk_n), //output O_tmds_clk_n
//		.O_tmds_data_p(tmds_data_p), //output [2:0] O_tmds_data_p
//		.O_tmds_data_n(tmds_data_n) //output [2:0] O_tmds_data_n
//	);


endmodule
