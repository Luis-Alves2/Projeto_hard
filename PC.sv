module PC 
    (input  wire  [63:0]raddress,        
     input  wire  loadPC,
     input wire Rst,
     output reg  [63:0]Dataout
    );

always @(loadPC or Rst)begin
	if(Rst)
		Dataout <= 0;
	else if(loadPC)
		Dataout <= raddress;
end
endmodule
