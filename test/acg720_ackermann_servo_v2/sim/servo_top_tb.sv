`timescale 1ns/1ps
module servo_top_tb;
    reg clk=0; always #500 clk=~clk; // 1MHz accelerated control test
    reg reset_n=0;
    reg [3:0] keys_n=4'b1110; // S1 held during initial configuration
    wire pwm; wire [7:0] led;
    wire pwm_reverse; wire [7:0] led_reverse;
    integer i, n;
    acg720_servo_top #(.CLK_HZ(1000000),.BOOT_MS(10),.DEBOUNCE_MS(2)) dut(
        .clk50M(clk),.reset_n(reset_n),.key_enable_n(keys_n[0]),
        .key_minus_n(keys_n[1]),.key_center_n(keys_n[2]),.key_plus_n(keys_n[3]),
        .servo_pwm_out(pwm),.led(led));
    acg720_servo_top #(.CLK_HZ(1000000),.BOOT_MS(10),.DEBOUNCE_MS(2),
        .LEFT_SMALLER_PULSE(0)) reversed_dut(
        .clk50M(clk),.reset_n(reset_n),.key_enable_n(keys_n[0]),
        .key_minus_n(keys_n[1]),.key_center_n(keys_n[2]),.key_plus_n(keys_n[3]),
        .servo_pwm_out(pwm_reverse),.led(led_reverse));
    task press_key(input integer key);
        begin keys_n[key]=0; #3000000; keys_n[key]=1; #3000000; end
    endtask
    task expect_target(input integer value);
        begin
            if(dut.target_us!==value) $fatal(1,"target got %0d expected %0d",dut.target_us,value);
            if(reversed_dut.target_us!==(3000-value)) $fatal(1,"reversed installation command incorrect");
            if(led_reverse!==led) $fatal(1,"direction LED meanings changed in reverse mode");
        end
    endtask
    always @(negedge clk) begin
        if(dut.target_us<1389 || dut.target_us>1611) $fatal(1,"target out of 10-degree range");
        if(reversed_dut.target_us<1389 || reversed_dut.target_us>1611) $fatal(1,"reversed target out of range");
        if(led[6]!==0) $fatal(1,"reserved LED high");
    end
    initial begin
        #10000; reset_n=1; #30000000;
        if(led[0] || pwm || dut.manual_ready) $fatal(1,"boot-held S1 armed");
        keys_n=15; #3000000;
        if(!dut.manual_ready) $fatal(1,"release did not arm manual input");
        // sub-debounce chatter must not generate an enable event.
        for(i=0;i<5;i=i+1) begin keys_n[0]=0; #200000; keys_n[0]=1; #200000; end
        #3000000;
        if(dut.enabled) $fatal(1,"bounce enabled motion");
        press_key(0);
        if(!dut.enabled) $fatal(1,"S1 failed to enable");
        expect_target(1500);
        // Held/repeated direction keys request the SAME absolute preset.
        keys_n[3]=0; #40000000; expect_target(1611);
        keys_n[3]=1; #3000000;
        for(i=0;i<20;i=i+1) press_key(1);
        expect_target(1389);
        if(!led[5] || !led[3] || led[4]) $fatal(1,"left preset/limit LEDs incorrect");
        for(i=0;i<25;i=i+1) press_key(3);
        expect_target(1611);
        if(!led[5] || !led[4] || led[3]) $fatal(1,"right preset/limit LEDs incorrect");
        press_key(2); expect_target(1500);
        keys_n[1]=0; keys_n[3]=0; #3000000;
        keys_n=15; #3000000; expect_target(1500);
        press_key(0);
        if(dut.enabled) $fatal(1,"S1 failed to disable");
        #21000000;
        if(pwm) $fatal(1,"PWM persists while disabled");
        press_key(3); expect_target(1500); // Disabled commands ignored
        // Reset must not restart with S1 held.
        press_key(0); #22000000;
        if(!dut.enabled) $fatal(1,"second enable failed");
        keys_n[0]=0; reset_n=0; #1;
        if(pwm!==0) $fatal(1,"S0 did not immediately suppress output");
        #10000; reset_n=1; #30000000;
        if(dut.enabled || dut.manual_ready || pwm) $fatal(1,"restart while held");
        keys_n=15; #3000000; press_key(0);
        if(!dut.enabled) $fatal(1,"fresh press after reset failed");
        $display("PASS controls: boot-held interlock, bounce, enable/disable, fixed +/-10deg presets (1389/1611us), limit LEDs, no accumulation, center, reversed installation, simultaneous keys, reset");
        $finish;
    end
    initial begin #1000000000; $fatal(1,"simulation timeout"); end
endmodule
