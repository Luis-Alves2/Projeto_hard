module Extensor3 
    (input  wire  [31:0]inSignal,        
     output reg  [63:0]outSignal
    );

always @(inSignal)begin
	outSignal[11:5] <= inSignal[31:25];
	outSignal[4:0] <= inSignal[11:7];
	outSignal[63:12] <= {52{inSignal[31]}};
end
endmodule
