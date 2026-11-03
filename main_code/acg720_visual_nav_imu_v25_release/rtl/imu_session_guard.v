`timescale 1ns/1ps
// User start/stop; acquisition owns retry. Fault must not cancel recovery.
module imu_session_guard #(parameter integer AUTO_START=0)(
    input clk,rst_n,press,fault_pulse,link_ok,configured,
    input [7:0] error_code,who_am_i,
    input [20:0] diagnostic,
    output reg active,stopped,
    output reg [7:0] saved_error,saved_id,
    output reg [20:0] saved_diagnostic
);
    reg address_missed,ever_linked;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            active<=(AUTO_START!=0);stopped<=0;address_missed<=0;ever_linked<=0;
            saved_error<=0;saved_id<=0;saved_diagnostic<=0;
        end else if(press)begin
            active<=!active;stopped<=0;address_missed<=0;ever_linked<=0;
            saved_error<=0;saved_id<=0;saved_diagnostic<=0;
        end else if(active)begin
            if(link_ok || configured)begin address_missed<=0;ever_linked<=1;end
            if(fault_pulse)begin
                // A first address NACK is expected when a sensor is at 0x69.
                if((error_code==8'h21 || error_code==8'h01) && !address_missed && !ever_linked)address_missed<=1;
                else begin
                    stopped<=0;saved_error<=error_code;
                    saved_id<=who_am_i;saved_diagnostic<=diagnostic;
                end
            end
        end
    end
endmodule
