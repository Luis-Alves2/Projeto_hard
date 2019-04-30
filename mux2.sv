module mux2
	(input wire [63:0]a, [63:0]b, [63:0]c, [63:0]d,[63:0]e,[63:0]f, [63:0]g, [2:0]sel,
	output reg [63:0]outMux
	);

always @( a or b or c or d or sel)begin
 	if(sel == 3'b000) outMux = a;           // regB
 	else if(sel == 3'b001) outMux = b;      // const = 4 
 	else if(sel == 3'b010) outMux = c;      // SB
	else if(sel == 3'b011) outMux = d;      // Shift left 1
   	else if(sel == 3'b100) outMux = e;      // Extensor S
	else if(sel == 3'b101) outMux = f;	// Extensor UJ
	else if(sel == 3'b110) outMux = g; 	// AluOut
end
endmodule
