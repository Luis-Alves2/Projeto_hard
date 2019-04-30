module extensorLB (input  wire  [63:0]inSignal, output reg  [63:0]outSignal);

	always @(inSignal)begin
		outSignal[7:0] <= inSignal[7:0];
		outSignal[63:8] <= 0;	
	end
endmodule;

