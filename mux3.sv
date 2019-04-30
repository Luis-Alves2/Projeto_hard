module mux3
	(input wire [63:0]a, [63:0]b, [63:0]c, 
	input wire [2:0]sel,
	output reg [63:0]outMux
	);

always @( a or b or c or sel)begin
 	if(sel == 3'b000) outMux = a;          // PC
 	else if(sel == 3'b001) outMux = b;     // reg A 
 	else if(sel == 3'b010) outMux = c;     // Extensor Lui
end
endmodule
