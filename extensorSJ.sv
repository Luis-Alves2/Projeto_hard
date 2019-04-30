
module extensorSJ(input  wire  [31:0]inSignal, output reg  [63:0]outSignal);

always @(inSignal)begin

	outSignal[20] <= inSignal[31];
	outSignal[10:1] <= inSignal[30:21];
	outSignal[11] <= inSignal[20];
	outSignal[19:12] <= inSignal[19:12];
	outSignal[64:21] = {44{outSignal[20]}};

end
endmodule

