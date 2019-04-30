module ModReg
    (input  wire  [63:0]inVal,        
     input  wire  loadReg,
     output reg  [63:0]outVal
    );

always @(inVal or loadReg)begin
	if(loadReg == 1) outVal = inVal;
end
endmodule
