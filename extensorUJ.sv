module extensorUJ(input  wire  [31:0]inSignal, output reg  [63:0]outSignal);

	always @(inSignal)begin
		outSignal[21] <= inSignal[31];
		outSignal[11:2] <= inSignal[30:21];
		outSignal[12] <= inSignal[20];
		outSignal[20:13] <= inSignal[19:12];
		outSignal[63:22] <= {43{outSignal[20]}};
		outSignal[0] <= 0;
		outSignal[1] <= 0;
		
	end
endmodule;