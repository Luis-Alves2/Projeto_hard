module Extensor 
    (input  wire  [31:0]inSignal,        
     output reg  [63:0]outSignal
    );

always @(inSignal)begin
	outSignal[11:0] <= inSignal[31:20];
	outSignal[63:12] <= {52{inSignal[31]}};
end
endmodule
