module bancoReg(
            input write, 
            input clock,
            input reset,
            input logic [5-1:0] regreader1,
            input logic [5-1:0] regreader2,
            input logic [5-1:0] regwriteaddress,
            input logic [64-1:0] datain,
            output logic [64-1:0] dataout1,
	    //output logic [63:0] regOut,
            output logic [64-1:0] dataout2
        );

//timeunit        1ns ;
//timeprecision 100ps ;
/*always @(regreader1 or regreader2)begin
	dataout1[4:0] <= regreader1;
	dataout1[63:5] <= 59'b0;
	dataout2[4:0] <= regreader2;
	dataout2[63:5] <= 59'b0;
end*/

logic [64-1:0] regs [0:31]; 
assign dataout1 = regs[regreader1];
assign dataout2 = regs[regreader2];

always_ff@(posedge clock or posedge reset)
begin
	if(reset)
  	begin
  		regs <= '{default:32'd0};
	end
  	else begin
		if(write && regwriteaddress != 0)
		begin  	   
		    regs[regwriteaddress] <= datain;
		    $display("%d - %d", regwriteaddress, regs[regwriteaddress]);
		end
	end
end

endmodule
