// JGB37-520R30-12 + TB6612. All speed values are signed output-shaft RPM.
// One output-shaft revolution = 13 * 30 * 4 = 1560 quadrature counts.
module car_closed_loop #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer ENC_PUL_PER_ROT = 1560,
    parameter [15:0] PI_KP = 16'd8192, // Q12: 2.0 PWM units / RPM
    parameter [15:0] PI_KI = 16'd410,  // Q12: 0.1001 per 10 ms update
    parameter integer CONTROL_HZ = 100,
    parameter integer PWM_HZ = 50000,
    parameter integer LEFT_TRIM_PERCENT = 105,
    parameter integer RIGHT_TRIM_PERCENT = 100,
    parameter integer MOTOR_L_REVERSE = 0,
    parameter integer MOTOR_R_REVERSE = 0,
    parameter integer ENC_L_REVERSE = 0,
    parameter integer ENC_R_REVERSE = 0
)(
    input clk, rst_n,
    input drive_enable,
    input signed [31:0] target_rpm,
    output wire signed [31:0] count_left, count_right,
    output wire signed [31:0] speed_left, speed_right,
    output wire rpm_valid_left, rpm_valid_right,
    output wire signed [31:0] debug_target_left, debug_target_right,
    output wire signed [15:0] debug_pi_left, debug_pi_right,
    output wire [15:0] debug_duty_left, debug_duty_right,
    output wire [15:0] debug_invalid_left, debug_invalid_right,
    output wire debug_control_valid,
    output reg PWMA, AIN1, AIN2,
    output reg PWMB, BIN1, BIN2,
    output STBY,
    input ENC_LA, ENC_LB, ENC_RA, ENC_RB,
    output reg servo_pwm
);
    // Asynchronous assertion, synchronous release of internal reset.
    reg [1:0] reset_sync;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) reset_sync <= 2'b00;
        else reset_sync <= {reset_sync[0], 1'b1};
    wire run_n = reset_sync[1] && drive_enable;
    assign STBY = run_n;

    localparam integer SAMPLE_CLKS = CLK_FREQ / CONTROL_HZ;
    localparam integer PWM_CLKS = CLK_FREQ / PWM_HZ;
    localparam integer DUTY_MAX = 999;
    wire valid_left, valid_right;
    encoder_quad #(.SAMPLE_CLKS(SAMPLE_CLKS), .REVERSE(ENC_L_REVERSE)) enc_left(
        .clk(clk), .rst_n(run_n), .A(ENC_LA), .B(ENC_LB),
        .speed_out(count_left), .sample_valid(valid_left),.invalid_total(debug_invalid_left));
    encoder_quad #(.SAMPLE_CLKS(SAMPLE_CLKS), .REVERSE(ENC_R_REVERSE)) enc_right(
        .clk(clk), .rst_n(run_n), .A(ENC_RA), .B(ENC_RB),
        .speed_out(count_right), .sample_valid(valid_right),.invalid_total(debug_invalid_right));

    // Reduce the constant fraction, then divide over 64 clock cycles.
    function [63:0] gcd;
        input [63:0] a, b;
        reg [63:0] x, y, t;
        begin
            x=a; y=b;
            while(y!=0) begin t=x%y; x=y; y=t; end
            gcd=x;
        end
    endfunction
    localparam signed [63:0] RPM_DENOM = 64'sd1 * ENC_PUL_PER_ROT * SAMPLE_CLKS;
    localparam [63:0] RPM_NUM = 64'd60 * CLK_FREQ;
    localparam [63:0] RPM_GCD = gcd(RPM_NUM,RPM_DENOM);
    // Preserve +5% left trim, with the SAME 64-cycle latency as RPM scaling.
    // Serial division avoids a long combinational /100 path into the PI loop.
    wire signed [31:0] target_left;
    wire target_left_valid;
    signed_scale #(.NUMERATOR(LEFT_TRIM_PERCENT),.DENOMINATOR(100)) left_target_scale(
        .clk(clk),.rst_n(run_n),.start(valid_left),.value(target_rpm),
        .result(target_left),.valid(target_left_valid));
    wire signed [31:0] target_right;
    wire target_right_valid;
    signed_scale #(.NUMERATOR(RIGHT_TRIM_PERCENT),.DENOMINATOR(100)) right_target_scale(
        .clk(clk),.rst_n(run_n),.start(valid_right),.value(target_rpm),
        .result(target_right),.valid(target_right_valid));
    signed_scale #(.NUMERATOR(RPM_NUM/RPM_GCD),.DENOMINATOR(RPM_DENOM/RPM_GCD)) rpm_left(
        .clk(clk),.rst_n(run_n),.start(valid_left),.value(count_left),
        .result(speed_left),.valid(rpm_valid_left));
    signed_scale #(.NUMERATOR(RPM_NUM/RPM_GCD),.DENOMINATOR(RPM_DENOM/RPM_GCD)) rpm_right(
        .clk(clk),.rst_n(run_n),.start(valid_right),.value(count_right),
        .result(speed_right),.valid(rpm_valid_right));
    (* keep = 1 *) wire signed [15:0] out_pi_left, out_pi_right;
    wire pi_valid_left,pi_valid_right;
    pi_control pi_left(.clk(clk), .rst_n(run_n), .enable(rpm_valid_left && target_left_valid),
        .kp(PI_KP), .ki(PI_KI), .target(target_left),
        .feedback(speed_left), .pi_out(out_pi_left),.out_valid(pi_valid_left));
    pi_control pi_right(.clk(clk), .rst_n(run_n), .enable(rpm_valid_right && target_right_valid),
        .kp(PI_KP), .ki(PI_KI), .target(target_right),
        .feedback(speed_right), .pi_out(out_pi_right),.out_valid(pi_valid_right));

    wire signed [31:0] cmd_left = {{16{out_pi_left[15]}}, out_pi_left};
    wire signed [31:0] cmd_right = {{16{out_pi_right[15]}}, out_pi_right};
    wire signed [31:0] drive_left, drive_right;
    wire drive_valid_left, drive_valid_right;
    signed_scale #(.NUMERATOR(PWM_CLKS),.DENOMINATOR(DUTY_MAX)) pwm_left(
        .clk(clk),.rst_n(run_n),.start(pi_valid_left),.value(cmd_left),
        .result(drive_left),.valid(drive_valid_left));
    signed_scale #(.NUMERATOR(PWM_CLKS),.DENOMINATOR(DUTY_MAX)) pwm_right(
        .clk(clk),.rst_n(run_n),.start(pi_valid_right),.value(cmd_right),
        .result(drive_right),.valid(drive_valid_right));
    wire [31:0] mag_left = drive_left < 0 ? -drive_left : drive_left;
    wire [31:0] mag_right = drive_right < 0 ? -drive_right : drive_right;
    reg [31:0] pwm_cnt, duty_a, duty_b;
    assign debug_target_left=target_left;
    assign debug_target_right=target_right;
    assign debug_pi_left=out_pi_left;
    assign debug_pi_right=out_pi_right;
    assign debug_duty_left=duty_a[15:0];
    assign debug_duty_right=duty_b[15:0];
    assign debug_control_valid=drive_valid_left && drive_valid_right;
    // Latch both direction and duty only at PWM frame boundaries.
    always @(posedge clk or negedge run_n) begin
        if (!run_n) begin
            pwm_cnt <= 0; duty_a <= 0; duty_b <= 0;
            PWMA <= 0; PWMB <= 0;
            AIN1 <= 0; AIN2 <= 0; BIN1 <= 0; BIN2 <= 0;
        end else begin
            pwm_cnt <= pwm_cnt == PWM_CLKS-1 ? 0 : pwm_cnt+1'b1;
            if (pwm_cnt == PWM_CLKS-1) begin
                duty_a <= mag_left;
                duty_b <= mag_right;
                AIN1 <= (drive_left != 0) && ((!drive_left[31]) ^ (MOTOR_L_REVERSE != 0));
                AIN2 <= (drive_left != 0) && (( drive_left[31]) ^ (MOTOR_L_REVERSE != 0));
                BIN1 <= (drive_right != 0) && ((!drive_right[31]) ^ (MOTOR_R_REVERSE != 0));
                BIN2 <= (drive_right != 0) && (( drive_right[31]) ^ (MOTOR_R_REVERSE != 0));
                // End the old frame before changing direction.
                PWMA <= 0; PWMB <= 0;
            end else begin
                PWMA <= pwm_cnt < duty_a;
                PWMB <= pwm_cnt < duty_b;
            end
        end
    end

    // Fixed neutral servo: 1.5 ms high every 20 ms.
    localparam integer SERVO_CLKS = CLK_FREQ / 50;
    localparam integer SERVO_HIGH = (CLK_FREQ / 1000) * 3 / 2;
    reg [31:0] cnt_servo;
    always @(posedge clk or negedge run_n) begin
        if (!run_n) begin cnt_servo <= 0; servo_pwm <= 0; end
        else begin
            cnt_servo <= cnt_servo == SERVO_CLKS-1 ? 0 : cnt_servo+1'b1;
            servo_pwm <= cnt_servo < SERVO_HIGH;
        end
    end
endmodule

module encoder_quad #(
    parameter integer SAMPLE_CLKS = 500000,
    parameter integer REVERSE = 0
)(
    input clk, rst_n, A, B,
    output reg signed [31:0] speed_out, // Signed counts per sample window
    output reg sample_valid,
    output reg [15:0] invalid_total
);
    (* keep = 1 *) reg [1:0] sync_A, sync_B;
    reg [1:0] pre_state;
    reg signed [31:0] cnt_pulse;
    reg [31:0] cnt_time;
    reg signed [31:0] delta;
    always @* begin
        case ({pre_state, sync_A[1], sync_B[1]})
            4'b0010,4'b1011,4'b1101,4'b0100: delta = REVERSE ? -32'sd1 : 32'sd1;
            4'b0001,4'b0111,4'b1110,4'b1000: delta = REVERSE ? 32'sd1 : -32'sd1;
            default: delta = 0; // Ignore unchanged or illegal two-bit transitions.
        endcase
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync_A <= 0; sync_B <= 0; pre_state <= 0;
            cnt_pulse <= 0; cnt_time <= 0; speed_out <= 0; sample_valid <= 0;invalid_total<=0;
        end else begin
            sync_A <= {sync_A[0], A}; sync_B <= {sync_B[0], B};
            pre_state <= {sync_A[1], sync_B[1]};
            // Both AB bits changing together is not a legal quadrature edge.
            // This detects some bad transitions; it cannot prove all pulses arrived.
            if((pre_state ^ {sync_A[1],sync_B[1]})==2'b11 && invalid_total!=16'hffff)
                invalid_total<=invalid_total+1'b1;
            sample_valid <= 0;
            if (cnt_time == SAMPLE_CLKS-1) begin
                cnt_time <= 0;
                speed_out <= cnt_pulse + delta; // Include an edge at the sample boundary.
                cnt_pulse <= 0;
                sample_valid <= 1;
            end else begin
                cnt_time <= cnt_time + 1'b1;
                cnt_pulse <= cnt_pulse + delta;
            end
        end
    end
endmodule

module pi_control(
    input clk, rst_n, enable,
    input [15:0] kp, ki, // Positive Q12 coefficients
    input signed [31:0] target, feedback,
    output reg signed [15:0] pi_out,
    output reg out_valid
);
    // Requests must be separated by at least 6 clocks (10 ms in this design).
    localparam signed [50:0] LIMIT_Q = 51'sd999 * 51'sd4096;
    reg signed [23:0] integral;
    reg signed [32:0] error;
    reg signed [49:0] p_term, i_step, p_delayed;
    reg signed [23:0] i_next, i_delayed;
    reg signed [50:0] raw;
    wire signed [50:0] i_sum = $signed(integral) + $signed(i_step);
    reg [3:0] pending;
    reg [3:0] zero_target, positive_error, negative_error;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            integral<=0; pi_out<=0; out_valid<=0; pending<=0;
            error<=0; p_term<=0; i_step<=0; p_delayed<=0;
            i_next<=0; i_delayed<=0; raw<=0;
            zero_target<=0; positive_error<=0; negative_error<=0;
        end else begin
            pending <= {pending[2:0],enable};
            zero_target <= {zero_target[2:0],target==0};
            positive_error <= {positive_error[2:0],target>feedback};
            negative_error <= {negative_error[2:0],target<feedback};
            out_valid <= pending[3];
            if(enable) error <= {target[31],target} - {feedback[31],feedback};
            if(pending[0]) begin
                p_term <= error * $signed({1'b0,kp});
                i_step <= error * $signed({1'b0,ki});
            end
            if(pending[1]) begin
                p_delayed <= p_term;
                if(i_sum>LIMIT_Q) i_next <= 24'sd4091904;
                else if(i_sum< -LIMIT_Q) i_next <= -24'sd4091904;
                else i_next <= i_sum[23:0];
            end
            if(pending[2]) begin
                raw <= $signed(p_delayed) + $signed(i_next);
                i_delayed <= i_next;
            end
            if(pending[3]) begin
                if(zero_target[3]) begin integral<=0; pi_out<=0; end
                else begin
                    if(!((raw>LIMIT_Q && positive_error[3]) || (raw< -LIMIT_Q && negative_error[3])))
                        integral<=i_delayed;
                    if(raw>LIMIT_Q) pi_out<=16'sd999;
                    else if(raw< -LIMIT_Q) pi_out<= -16'sd999;
                    else pi_out<=raw[27:12];
                end
            end
        end
    end
endmodule

// Exact signed scaling with truncation toward zero. No combinational divider.
// Input magnitude * NUMERATOR must fit in 64 bits, result in signed 32 bits.
module signed_scale #(
    parameter [63:0] NUMERATOR=50,
    parameter [63:0] DENOMINATOR=13
)(
    input clk,rst_n,start,
    input signed [31:0] value,
    output reg signed [31:0] result,
    output reg valid
);
    reg busy, negative;
    reg [6:0] bits_left;
    reg [63:0] dividend, remainder, quotient;
    wire [32:0] magnitude = value[31] ? -$signed({value[31],value}) : {1'b0,value};
    wire [64:0] trial = {remainder,dividend[63]};
    wire subtract = trial >= {1'b0,DENOMINATOR};
    wire [64:0] residue = subtract ? trial-{1'b0,DENOMINATOR} : trial;
    wire [63:0] q_next = {quotient[62:0],subtract};
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            busy<=0; negative<=0; bits_left<=0; dividend<=0;
            remainder<=0; quotient<=0; result<=0; valid<=0;
        end else begin
            valid<=0;
            if(start && !busy) begin
                busy<=1; bits_left<=64; negative<=value[31];
                dividend<=magnitude*NUMERATOR; remainder<=0; quotient<=0;
            end else if(busy) begin
                dividend<={dividend[62:0],1'b0};
                remainder<=residue[63:0]; quotient<=q_next;
                bits_left<=bits_left-1'b1;
                if(bits_left==1) begin
                    busy<=0; valid<=1;
                    result<=negative ? -$signed(q_next[31:0]) : $signed(q_next[31:0]);
                end
            end
        end
    end
endmodule
