`timescale 1ns/1ps
module servo_pwm_tb;
    reg clk=0; always #10 clk=~clk; // ACTUAL production clock: 50MHz
    reg reset=1, enable=0;
    reg [15:0] request_us=1500;
    wire pwm;
    integer rises=0;
    time started, previous;
    servo_pwm dut(.clk(clk),.reset(reset),.enable(enable),
        .request_us(request_us),.pwm(pwm));
    always @(posedge pwm) rises=rises+1;
    task pulse(input integer expected_us);
        time t;
        begin
            @(posedge pwm); t=$time;
            @(negedge pwm);
            if ($time-t != expected_us*1000)
                $fatal(1,"width: got %0t expected %0d us",$time-t,expected_us);
        end
    endtask
    task quiet(input integer us);
        integer previous_rises;
        begin
            previous_rises=rises; #(us*1000);
            if (rises!=previous_rises || pwm!==0) $fatal(1,"unexpected pulse when disabled");
        end
    endtask
    initial begin
        #100; @(negedge clk); reset=0;
        quiet(25000);
        @(negedge clk); enable=1;
        pulse(1500);
        @(posedge pwm); previous=$time;
        @(posedge pwm);
        if ($time-previous!=20000000) $fatal(1,"frame is not 20ms");
        // Changing the command in a pulse cannot shorten this pulse.
        started=$time; #300000; request_us=1400;
        @(negedge pwm);
        if ($time-started!=1500000) $fatal(1,"mid-frame command made a runt");
        pulse(1400);
        request_us=1600; pulse(1600);
        request_us=1444; pulse(1444);
        request_us=1556; pulse(1556);
        request_us=0; pulse(1400);
        request_us=65535; pulse(1600);
        request_us=1500; pulse(1500);
        @(posedge pwm); started=$time;
        #250000; enable=0;
        @(negedge pwm);
        if ($time-started!=1500000) $fatal(1,"normal disable truncated the pulse");
        quiet(25000);
        enable=1; pulse(1500);
        @(posedge pwm); #200000; reset=1;
        repeat(2) @(negedge clk);
        if(pwm!==0) $fatal(1,"reset did not suppress output");
        enable=0; reset=0; quiet(22000);
        $display("PASS production 50MHz PWM: 20ms, 1400/1444/1500/1556/1600us, hard clamps, frame-latching, disable, reset");
        $finish;
    end
    initial begin #600000000; $fatal(1,"simulation timeout"); end
endmodule
