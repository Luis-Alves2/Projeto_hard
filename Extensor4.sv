module Extensor4 
    (input  wire  [31:0]inSignal,        
     output reg  [63:0]outSignal
    );

always @(inSignal)begin
	outSignal[31:12] <= inSignal[31:12];
	outSignal[11:0] <= 12'b0;
	outSignal[63:32] <= {32{inSignal[31]}};
	//$display("%d",outSignal);
end
endmodule
