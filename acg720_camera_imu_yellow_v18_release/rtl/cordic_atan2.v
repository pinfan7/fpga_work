`timescale 1ns/1ps
// Iterative vectoring CORDIC, degrees Q16, 16 rotations.
// Vector input/output use the caller's scale; magnitude includes CORDIC gain.
module cordic_atan2(
    input clk, input rst_n, input start,
    input signed [31:0] x_in, y_in,
    output reg busy, output reg done,
    output reg signed [31:0] angle, magnitude_scaled
);
    reg signed [31:0] x,y,z;
    reg [4:0] iteration;
    function signed [31:0] lut;
        input [4:0] index;
        begin case(index)
            0:lut=2949120; 1:lut=1740967; 2:lut=919879; 3:lut=466945;
            4:lut=234379; 5:lut=117304; 6:lut=58666; 7:lut=29335;
            8:lut=14668; 9:lut=7334; 10:lut=3667; 11:lut=1833;
            12:lut=917; 13:lut=458; 14:lut=229; default:lut=115;
        endcase end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin x<=0;y<=0;z<=0;iteration<=0;busy<=0;done<=0;angle<=0;magnitude_scaled<=0; end
        else begin
            done<=0;
            if(start && !busy) begin
                busy<=1; iteration<=0;
                if(x_in<0) begin
                    x<=-x_in; y<=-y_in;
                    z<=(y_in<0) ? -32'sd11796480 : 32'sd11796480;
                end else begin x<=x_in; y<=y_in; z<=0; end
            end else if(busy) begin
                if(iteration==16) begin
                    angle<=z; magnitude_scaled<=x; done<=1; busy<=0;
                end else begin
                    if(y>=0) begin x<=x+(y>>>iteration); y<=y-(x>>>iteration); z<=z+lut(iteration); end
                    else begin x<=x-(y>>>iteration); y<=y+(x>>>iteration); z<=z-lut(iteration); end
                    iteration<=iteration+1'b1;
                end
            end
        end
    end
endmodule
