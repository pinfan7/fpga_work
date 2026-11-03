`timescale 1ns/1ps
// Independent kinematic plant, real encoder quantization, ray/circle TOF model,
// swept car rectangle collision check. No forced controller state or pose.
module navigation_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,start=0,stop=0,imu_ok=1,imu_update=0,vl=0,vr=0;
 reg signed [31:0] yaw=0,cl=0,cr=0,sl=0,sr=0;
 reg [15:0] dl=4000,df=4000,dr=4000,nl=0,nf=0,nr=0;
 reg [2:0] tv=7;
 wire signed [31:0] xq,yq,hq,rpm,desired,pass_x;
 wire [15:0] pulse,fl,fr;wire [3:0] phase,code;
 wire motor,busy,done,fault,ready,avoid,left,posefault,updated;
 wire [31:0] elapsed;wire [7:0] avoids;
 nav_pose pose(.clk(clk),.rst_n(rst),.clear(start),.track(motor),.yaw_q16(yaw),
   .count_l(cl),.count_r(cr),.valid_l(vl),.valid_r(vr),.x_q8(xq),.y_q8(yq),
   .heading_q16(hq),.updated(updated),.fault(posefault));
 nav_controller #(.CLK_HZ(1000),.WAIT_MS(20)) dut(.clk(clk),.rst_n(rst),.start(start),.stop(stop),
   .x_q8(xq),.y_q8(yq),.heading_q16(hq),.imu_ok(imu_ok),.imu_update(imu_update),.pose_fault(posefault),
   .count_l(cl),.count_r(cr),.speed_l(sl),.speed_r(sr),.valid_l(vl),.valid_r(vr),
   .tof_l(dl),.tof_f(df),.tof_r(dr),.frames_l(nl),.frames_f(nf),.frames_r(nr),.tof_valid(tv),
   .motor_enable(motor),.busy(busy),.done(done),.fault(fault),.ready(ready),.avoid_mode(avoid),
   .target_rpm(rpm),.servo_us(pulse),.phase(phase),.fault_code(code),.bypass_left(left),
   .elapsed_ms(elapsed),.desired_heading_q16(desired),.pass_x_mm(pass_x),.fresh_l(fl),.fresh_r(fr),.avoid_count(avoids));
 real px,py,theta,ds,delta,curve,acc_l,acc_r,ox,oy;
 real a,b,c,disc,root,sx,sy,angle,dx,dy,bx,by,cx,cy;
 real abs_y,max_y,min_y,path_mm,plant_delta,servo_hold;
 integer tick_ms,il,ir,mode,k,previous_phase,first_side,rows;
 integer trace;
 reg plant=0,obstacle=0,feed_enc=1,feed_imu=1,feed_tof=1,collision=0;
 reg override_tof=0;
 localparam real PI=3.141592653589793;
 function integer beam;
   input real bearing;
   real rsx,rsy,rx,ry,along,perp2,distance;
   begin
     rsx=px+230.0*$cos(theta);rsy=py+230.0*$sin(theta);
     rx=ox-rsx;ry=oy-rsy;
     along=rx*$cos(bearing)+ry*$sin(bearing);
     perp2=rx*rx+ry*ry-along*along;
     distance=4000;
     if(obstacle && along>0 && perp2<44.0*44.0)begin
       distance=along-$sqrt(44.0*44.0-perp2);
       if(distance<50)distance=50;
     end
     beam=$rtoi(distance);
   end
 endfunction
 // One accelerated clock is one millisecond in the independent plant.
 always @(negedge clk)begin
   if(plant)begin
     tick_ms=tick_ms+1;vl=0;vr=0;imu_update=0;
     if(motor)begin
       ds=$itor(rpm)*65.0*PI/60000.0;
       if(tick_ms%20==0)servo_hold=$itor(pulse);
       // One common pulse-to-wheel gain: unequal pulse limits produce unequal
       // physical radii (28.88deg left / 36deg right), not normalized symmetry.
       delta=(1400.0-servo_hold)*0.08*PI/180.0;
       // First-order steering response: a physical servo does not move instantly.
       plant_delta=plant_delta+(delta-plant_delta)*0.02;
       curve=$sin(plant_delta)/$cos(plant_delta)/180.0;
       theta=theta+ds*curve;
       px=px+ds*$cos(theta);py=py+ds*$sin(theta);path_mm=path_mm+ds;
       acc_l=acc_l+ds*(1.0-curve*80.0)*1560.0/(65.0*PI);
       acc_r=acc_r+ds*(1.0+curve*80.0)*1560.0/(65.0*PI);
     end
     if(tick_ms%10==0)begin
       if(feed_enc)begin
         il=$rtoi(acc_l);ir=$rtoi(acc_r);acc_l=acc_l-il;acc_r=acc_r-ir;
         cl=il;cr=ir;sl=rpm;sr=rpm;vl=1;vr=1;
       end
       if(feed_imu)begin yaw=$rtoi(theta*180.0/PI*65536.0);imu_update=1;end
     end
     if(tick_ms%20==0 && feed_tof)begin
       nl=nl+1;nf=nf+1;nr=nr+1;
       if(!override_tof)begin
         dl=beam(theta+PI/4.0);df=beam(theta);dr=beam(theta-PI/4.0);
       end
     end
     if(obstacle && motor)begin
       // Rear axle at (0,0); body extends -35..235 forward, +/-100 sideways.
       dx=ox-px;dy=oy-py;
       bx=dx*$cos(theta)+dy*$sin(theta);by=-dx*$sin(theta)+dy*$cos(theta);
       cx=bx< -35 ? -35:(bx>235 ? 235:bx);
       cy=by< -100 ? -100:(by>100 ? 100:by);
       if((bx-cx)*(bx-cx)+(by-cy)*(by-cy)<44.0*44.0)collision=1;
     end
     if(py>max_y)max_y=py;if(py<min_y)min_y=py;
     if(phase==3 && first_side==0)first_side=left ? 1:-1;
     if(tick_ms%100==0 && mode<=3)
       $fdisplay(trace,"%0d,%0d,%0d,%0f,%0f,%0f,%0f,%0f,%0d,%0d,%0d,%0d,%0d",mode,tick_ms,phase,px,py,$itor($signed(xq))/256.0,$itor($signed(yq))/256.0,theta*180/PI,pulse,dl,df,dr,code);
   end
 end
 task reset_case;
   input integer scenario;
   begin
     plant=0;rst=0;start=0;stop=0;imu_ok=1;feed_imu=1;feed_enc=1;feed_tof=1;override_tof=0;
     tv=7;vl=0;vr=0;imu_update=0;yaw=0;dl=4000;df=4000;dr=4000;cl=0;cr=0;nl=0;nf=0;nr=0;
     px=0;py=0;theta=0;acc_l=0;acc_r=0;plant_delta=0;servo_hold=1400;tick_ms=0;path_mm=0;
     max_y=0;min_y=0;first_side=0;collision=0;mode=scenario;obstacle=0;ox=1000;oy=0;
     repeat(6)@(posedge clk);@(negedge clk);rst=1;
     repeat(3)@(posedge clk);@(negedge clk);imu_update=1;
     @(posedge clk);@(negedge clk);imu_update=0;start=1;
     @(posedge clk);@(negedge clk);start=0;plant=1;
   end
 endtask
 task run_trip;
   input integer scenario;
   begin
     reset_case(scenario);
     if(scenario>0)obstacle=1;
     if(scenario==2)oy=150;
     if(scenario==3)oy=-150;
     k=0;
     while(!done && !fault && k<100000)begin @(posedge clk);#1;k=k+1;end
     plant=0;
     if(fault || !done)$fatal(1,"trip %0d failed: phase=%0d code=%0d x=%f y=%f h=%f pass=%0d",scenario,phase,code,px,py,theta*180/PI,pass_x);
     if(collision)$fatal(1,"trip %0d swept body collided",scenario);
     if(px<1970 || px>2035 || py>40 || py< -40 || theta>0.07 || theta< -0.07)
       $fatal(1,"trip %0d actual endpoint wrong x=%f y=%f h=%f",scenario,px,py,theta*180/PI);
     if(scenario==0 && avoids!=0)$fatal(1,"false avoidance");
     if(scenario>0 && avoids==0)$fatal(1,"obstacle was ignored");
     if(scenario==2 && first_side!=-1)$fatal(1,"left obstacle did not choose right bypass");
     if(scenario==3 && first_side!=1)$fatal(1,"right obstacle did not choose left bypass");
     $display("trip %0d: x=%0.1f y=%0.1f heading=%0.2f path=%0.1f first_side=%0d",scenario,px,py,theta*180/PI,path_mm,first_side);
   end
 endtask
 initial begin
   trace=$fopen("reports/navigation_trace.csv","w");
   $fdisplay(trace,"scenario,ms,phase,true_x_mm,true_y_mm,est_x_mm,est_y_mm,heading_deg,servo_us,tof_l,tof_f,tof_r,fault");
   run_trip(0);run_trip(1);run_trip(2);run_trip(3);
   reset_case(4);override_tof=1;dl=200;df=200;dr=200;
   wait(fault);#1;if(code!=2 || motor)$fatal(1,"blocked passage did not stop");plant=0;
   reset_case(5);repeat(100)@(posedge clk);tv=3'b101;
   @(posedge clk);#1;if(motor)$fatal(1,"invalid TOF did not inhibit motor");
   repeat(3)@(posedge clk);#1;if(!fault || code!=1)$fatal(1,"TOF fault missing");plant=0;
   reset_case(6);repeat(100)@(posedge clk);feed_imu=0;
   repeat(300)@(posedge clk);#1;if(!fault || code!=8 || motor)$fatal(1,"stale IMU did not stop");plant=0;
   reset_case(7);repeat(100)@(posedge clk);feed_enc=0;
   repeat(350)@(posedge clk);#1;if(!fault || code!=5 || motor)$fatal(1,"stale encoder did not stop");plant=0;
   reset_case(8);override_tof=1;dl=4000;df=300;dr=4000;
   wait(avoid);@(negedge clk);stop=1;#1;if(motor)$fatal(1,"stop not immediate");
   repeat(3)@(posedge clk);#1;if(phase!=12)$fatal(1,"stop did not abort");plant=0;
   // Re-reading the same sensor frame cannot count as three confirmations.
   reset_case(9);feed_tof=0;override_tof=1;dl=4000;df=300;dr=4000;
   repeat(60)@(posedge clk);@(negedge clk);nl=1;nf=1;nr=1;
   repeat(100)@(posedge clk);#1;if(avoid || fault)$fatal(1,"one frame falsely confirmed");plant=0;
   $fclose(trace);
   $display("PASS: 2m straight; front/left/right raycast obstacle; asymmetric steering and actuator lag; swept collision; physical vs estimated endpoint; blocked/invalid/stale/stop; frame confirmation");
   $finish;
 end
 initial begin #10000000;$fatal(1,"navigation test timeout");end
endmodule
