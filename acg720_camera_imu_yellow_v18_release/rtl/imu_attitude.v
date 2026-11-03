`timescale 1ns/1ps
// Ground-vehicle first version, not a general 3-D aerospace AHRS.
// Roll/pitch: accel tilt + gyro complementary filter (1/64 correction/sample).
// Yaw: bias-corrected body-Z rate integration, only near-level heading estimate.
// Q16 degrees internally. No DMP/CPU; iterative CORDIC shared by two tilt axes.
module imu_attitude #(parameter integer CAL_LOG2=8)(
    input clk, input rst_n, input recalibrate, input link_ok, input sample_valid,
    input signed [15:0] ax,ay,az,gx,gy,gz, input [15:0] dt_us,
    output reg signed [31:0] roll,pitch,yaw,
    output reg calibrated, output reg attitude_valid, output reg update,
    output reg [8:0] cal_count, output reg accel_trusted,
    output reg tilt_warning, output reg overrun
);
    localparam integer CAL_SAMPLES=1<<CAL_LOG2;
    reg [3:0] state;
    reg signed [15:0] ax_hold,ay_hold,az_hold;
    reg signed [15:0] previous_ax,previous_ay,previous_az;
    reg signed [15:0] bias_x,bias_y,bias_z;
    reg signed [31:0] sum_x,sum_y,sum_z;
    reg signed [32:0] norm2;
    reg signed [31:0] ax2,ay2,az2;
    reg signed [47:0] prod_x,prod_y,prod_z;
    reg signed [47:0] scaled_x,scaled_y,scaled_z;
    reg signed [31:0] delta_x,delta_y,delta_z;
    reg signed [31:0] accel_roll,accel_pitch;
    reg signed [31:0] predicted_r,predicted_p,predicted_y;
    reg signed [31:0] corrected_r,corrected_p;
    reg signed [63:0] yz_magnitude;
    reg sample_cal_ok, seed;
    reg cordic_start;
    reg signed [31:0] cordic_x,cordic_y;
    wire cordic_done,cordic_busy;
    wire signed [31:0] cordic_angle,cordic_mag;
    wire signed [16:0] rate_x=$signed(gx)-$signed(bias_x);
    wire signed [16:0] rate_y=$signed(gy)-$signed(bias_y);
    wire signed [16:0] rate_z=$signed(gz)-$signed(bias_z);
    wire signed [16:0] dax=$signed(ax)-$signed(previous_ax);
    wire signed [16:0] day=$signed(ay)-$signed(previous_ay);
    wire signed [16:0] daz=$signed(az)-$signed(previous_az);
    function signed [31:0] wrap;
        input signed [31:0] a;
        begin
            if(a>=32'sd11796480) wrap=a-32'sd23592960;
            else if(a < -32'sd11796480) wrap=a+32'sd23592960;
            else wrap=a;
        end
    endfunction
    cordic_atan2 u_cordic(.clk(clk),.rst_n(rst_n),.start(cordic_start),
        .x_in(cordic_x),.y_in(cordic_y),.busy(cordic_busy),.done(cordic_done),
        .angle(cordic_angle),.magnitude_scaled(cordic_mag));
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=0; roll<=0;pitch<=0;yaw<=0;calibrated<=0;attitude_valid<=0;
            update<=0;cal_count<=0;accel_trusted<=0;tilt_warning<=0;overrun<=0;
            sum_x<=0;sum_y<=0;sum_z<=0;bias_x<=0;bias_y<=0;bias_z<=0;
            ax_hold<=0;ay_hold<=0;az_hold<=0;previous_ax<=0;previous_ay<=0;previous_az<=0;
            ax2<=0;ay2<=0;az2<=0;norm2<=0;prod_x<=0;prod_y<=0;prod_z<=0;
            scaled_x<=0;scaled_y<=0;scaled_z<=0;delta_x<=0;delta_y<=0;delta_z<=0;
            accel_roll<=0;accel_pitch<=0;predicted_r<=0;predicted_p<=0;predicted_y<=0;
            corrected_r<=0;corrected_p<=0;yz_magnitude<=0;sample_cal_ok<=0;seed<=0;
            cordic_start<=0;cordic_x<=0;cordic_y<=0;
        end else begin
            update<=0; cordic_start<=0;
            if(recalibrate || !link_ok) begin
                // Let an already-running CORDIC finish before accepting another sample.
                state<=0;calibrated<=0;attitude_valid<=0;cal_count<=0;
                sum_x<=0;sum_y<=0;sum_z<=0;bias_x<=0;bias_y<=0;bias_z<=0;
                roll<=0;pitch<=0;yaw<=0;seed<=0;overrun<=0;
            end else begin
                if(sample_valid && (state!=0 || cordic_busy)) overrun<=1;
                case(state)
                0: if(sample_valid && !cordic_busy) begin
                    ax_hold<=ax;ay_hold<=ay;az_hold<=az;
                    previous_ax<=ax;previous_ay<=ay;previous_az<=az;
                    ax2<=$signed(ax)*$signed(ax);ay2<=$signed(ay)*$signed(ay);az2<=$signed(az)*$signed(az);
                    // 65536/(65.5*1e6) approximated by 1049/2^20 (0.015% scale error).
                    prod_x<=rate_x*$signed({1'b0,dt_us});
                    prod_y<=rate_y*$signed({1'b0,dt_us});
                    prod_z<=rate_z*$signed({1'b0,dt_us});
                    sample_cal_ok <= gx<196 && gx> -196 && gy<196 && gy> -196 && gz<196 && gz> -196 &&
                        (cal_count==0 || (dax<200 && dax> -200 && day<200 && day> -200 && daz<200 && daz> -200));
                    if(!calibrated) begin sum_x<=sum_x+gx;sum_y<=sum_y+gy;sum_z<=sum_z+gz; end
                    seed<=0;state<=1;
                end
                1: begin
                    norm2<=$signed({1'b0,ax2})+$signed({1'b0,ay2})+$signed({1'b0,az2});
                    scaled_x<=prod_x*48'sd1049;scaled_y<=prod_y*48'sd1049;scaled_z<=prod_z*48'sd1049;
                    state<=2;
                end
                2: begin
                    delta_x<=scaled_x>>>20;delta_y<=scaled_y>>>20;delta_z<=scaled_z>>>20;
                    // +/-4g scale: 1g = 8192; trust norm within about 0.85..1.15g.
                    accel_trusted<=norm2>=33'sd48486154 && norm2<=33'sd88751472;
                    if(!calibrated) begin
                        if(!sample_cal_ok || norm2<33'sd48486154 || norm2>33'sd88751472) begin
                            cal_count<=0;sum_x<=0;sum_y<=0;sum_z<=0;state<=0;
                        end else if(cal_count==CAL_SAMPLES-1) begin
                            bias_x<=sum_x>>>CAL_LOG2;bias_y<=sum_y>>>CAL_LOG2;bias_z<=sum_z>>>CAL_LOG2;
                            cal_count<=CAL_SAMPLES;calibrated<=1;seed<=1;state<=3;
                        end else begin cal_count<=cal_count+1'b1;state<=0;end
                    end else state<=3;
                end
                3: begin
                    cordic_x<=$signed(az_hold)*32'sd256;
                    cordic_y<=$signed(ay_hold)*32'sd256;
                    cordic_start<=1;state<=4;
                end
                4: if(cordic_done) begin
                    accel_roll<=cordic_angle;
                    yz_magnitude<=$signed(cordic_mag)*64'sd39797; // divide CORDIC gain.
                    state<=5;
                end
                5: begin
                    cordic_x<=yz_magnitude>>>16;
                    cordic_y<=-$signed(ax_hold)*32'sd256;
                    cordic_start<=1;state<=6;
                end
                6: if(cordic_done) begin accel_pitch<=cordic_angle;state<=7;end
                7: begin
                    predicted_r<=wrap(roll+delta_x);predicted_p<=wrap(pitch+delta_y);
                    predicted_y<=wrap(yaw+delta_z);state<=8;
                end
                8: begin
                    corrected_r<=wrap(predicted_r+(wrap(accel_roll-predicted_r)>>>6));
                    corrected_p<=wrap(predicted_p+(wrap(accel_pitch-predicted_p)>>>6));state<=9;
                end
                9: begin
                    if(seed) begin roll<=wrap(accel_roll);pitch<=accel_pitch;yaw<=0;end
                    else begin
                        roll<=accel_trusted ? corrected_r : predicted_r;
                        pitch<=accel_trusted ? corrected_p : predicted_p;
                        yaw<=predicted_y;
                    end
                    attitude_valid<=1;update<=1;state<=0;
                    tilt_warning <= accel_roll>32'sd2949120 || accel_roll< -32'sd2949120 ||
                        accel_pitch>32'sd2949120 || accel_pitch< -32'sd2949120;
                end
                default: begin state<=0;attitude_valid<=0;overrun<=1;end
                endcase
            end
        end
    end
endmodule
