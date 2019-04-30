`timescale 1ps/1ps
module UC;

// PC
logic load_PC;
reg [63:0]PC_in;
reg [63:0]PC_out;

// MEMORIA32
logic clk;
logic rst;
reg [31:0]wdaddress;	
reg [31:0]data;
reg Wr;
wire [31:0]dataPath; // OUT

// REGISTRADOR DE INSTRU??O
logic load_ir;
wire [4:0]instr19_15;
wire [4:0]instr24_20;
wire [4:0]instr11_7;
wire [6:0]instr6_0;
wire [31:0]instr31_0;

// ULA
wire overFlow;
wire negativo;
wire z;
wire [63:0]s;
wire igual;
wire menor;
wire maior;
reg [63:0]d4;
reg [2:0]ALUfct; // Seletor de Opera??o

// SIGNAL EXTENSOR
wire [63:0]extensorI;

// BANCO DE REGS
reg [63:0]readData1;
reg [63:0]readData2;
logic writeReg; 

// REG A E REG B
logic loadRegA;
logic loadRegB;
wire [63:0]outRegA;
wire [63:0]outRegB;

// MUX ULA
logic [2:0]ALUSrcA;
logic [2:0]ALUSrcB;
wire [63:0]saidaMuxALUA;
wire [63:0]saidaMuxALUB;

// ALUOUT
logic loadAOut;
wire [63:0]Aout;

reg [63:0]writeData;

// MEM?RIA 64
wire [63:0]waddress64;         
wire [63:0]Datain64;
wire [63:0]Dataout64;
reg [63:0]entrada64;
logic Wr64;

// REGISTER
logic loadMDR;
wire [63:0]dataOutReg;

// MUX REGISTER
logic memToReg;
wire [63:0]saidaMuxReg;

wire [63:0]extensorSB;
wire [63:0]extensorU;
wire [63:0]extensorS;


// LIGANDO OS ModULOS
PC ulapc (.raddress(PC_in), .loadPC(load_PC), .Rst(rst), .Dataout(PC_out));

Memoria32 pcmem (.raddress(PC_out), .waddress(wdaddress),
.Clk(clk), .Datain(data), .Dataout(dataPath), .Wr(Wr) );

Instr_Reg_RISC_V meminst (.Clk(clk), .Reset(rst), .Load_ir(load_ir),
.Entrada(dataPath), .Instr19_15(instr19_15), .Instr24_20(instr24_20), .Instr11_7(instr11_7), .Instr6_0(instr6_0), .Instr31_0(instr31_0)); // Insere no registrador de instru??es

bancoReg instbanco (.write(writeReg), .clock(clk), .reset(rst), .regreader1(instr19_15), .regreader2(instr24_20), .regwriteaddress(instr11_7), 
.datain(saidaMuxReg), .dataout1(readData1), .dataout2(readData2));

ModReg regA (.inVal(readData1), .loadReg(loadRegA), .outVal(outRegA));

ModReg regB (.inVal(readData2), .loadReg(loadRegB), .outVal(outRegB));

Extensor signextendI (.inSignal(instr31_0), .outSignal(extensorI));

Extensor2 signextendSB (.inSignal(instr31_0), .outSignal(extensorSB));

Extensor3 extendeS(.inSignal(instr31_0),.outSignal(extensorS));

Extensor4 signextendU(.inSignal(instr31_0), .outSignal(extensorU));

mux3 muxUlaA (.a(PC_out), .b(outRegA), .c(extensorU), .sel(ALUSrcA),.outMux(saidaMuxALUA));

mux2 muxUlaB (.a(outRegB), .b(d4), .c(extensorI), .d(extensorSB), .sel(ALUSrcB),.e(extensorS), .outMux(saidaMuxALUB));

ula64 aritmop (.A(saidaMuxALUA), .B(saidaMuxALUB), .Seletor(ALUfct), .S(s), .Overflow(overFlow), .Negativo(negativo), .Z(z), .Igual(igual), .Maior(maior), .Menor(menor)); // Soma mais 4com o PC

ModReg ALUout (.inVal(s), .loadReg(loadAOut), .outVal(Aout));

Memoria64 ALUtomem(.raddress(Aout), .waddress(Aout), .Clk(clk), .Datain(outRegB), .Dataout(Dataout64), .Wr(Wr64));

register memtoreg(.clk(clk), .reset(rst), .regWrite(loadMDR), .DadoIn(Dataout64), .DadoOut(dataOutReg));

mux muxWriteDataReg (.a(Aout), .b(dataOutReg), .sel(memToReg), .f(saidaMuxReg));

// ESTADOS DA MAQUINA DE ESTADOS
enum{S_reset1,S_reset2,S_incPC_loadRegs,S_opcode,S_aluOp_I,S_loadMDR_I,S_memToReg_I,S_addi,S_writeReg_I,S_updatePC,S_Sb_1,S_Sb_2,S_Sb_3,S_branch,S_beginBranch,S_add, S_sub,S_updateReg,S_lui,S_Store1,S_Store2}state;

//gerador de clock e reset
localparam CLKPERIOD = 10000;
localparam CLKDELAY = CLKPERIOD / 2;

initial begin
clk = 1'b1;
Wr = 0;
d4 = 64'd4;
rst = 1'b1;
writeData = 0;
end

always #(CLKDELAY) clk = ~clk;

always_ff @(posedge clk or posedge rst)
begin
	if(rst) begin
		state <= S_reset1;
		rst = 1'b0;
	end
	else begin
		if(state == S_reset1) state <= S_reset2;
		else if(state == S_reset2) state <= S_incPC_loadRegs;
		else if(state == S_incPC_loadRegs) state <=  S_opcode;
		else if(state ==  S_opcode) begin
			if(instr6_0 == 7'b0000011 || instr6_0 == 7'b0010011) state <= S_aluOp_I;
			else if(instr6_0 == 7'b1100011 || instr6_0 == 7'b1100111) state <= S_Sb_1;
			else if(instr6_0 == 7'b0110011 && instr31_0[31:25] == 7'b0000000) state <= S_add;
			else if(instr6_0 == 7'b0110011 && instr31_0[31:25] == 7'b0100000) state <= S_sub;
			else if(instr6_0 == 7'b0110111) state <= S_lui;
			else if (instr6_0 == 7'b0100011) state <= S_Store1;
		end
		else if (state == S_aluOp_I) begin
			if(instr6_0 == 7'b0000011) state <= S_loadMDR_I;
			else state <= S_addi;
		end
      		else if(state == S_loadMDR_I) state <= S_memToReg_I;
		else if(state == S_memToReg_I) state <= S_writeReg_I;
		else if(state == S_addi) state <= S_writeReg_I;
		else if(state == S_writeReg_I) state <= S_updatePC;
		else if(state == S_Sb_1) state <= S_Sb_2;
		else if (state == S_Store1) state <= S_Store2;
		else if(state == S_Store2) state <= S_updatePC;
		else if(state == S_Sb_2)begin
			if(instr6_0 == 7'b1100011)begin
				if(z == 1) state <= S_branch;   //BEQ
				else state <= S_updatePC;
			end
			else if(instr6_0 == 7'b1100111) begin  //BNE
				if(z == 0) state <= S_branch;
				else state <= S_updatePC;
			end
		end
		else if(state == S_Sb_3) state <= S_branch;
		else if(state == S_add) state <= S_updateReg;
		else if(state == S_sub) state <= S_updateReg;
		else if(state == S_lui) state <= S_updateReg;
		else if(state == S_updateReg) state <= S_updatePC;
		else if(state == S_branch) state <= S_updatePC;
		else if(state == S_beginBranch) state <= S_opcode;
		else if(state == S_updatePC) state <= S_incPC_loadRegs;

		
		end
	end

always_comb begin
	
	if(state == S_reset1)begin
		writeReg = 0;
		ALUSrcA = 3'b0;
		ALUSrcB = 3'b001;
		ALUfct = 3'd1; 
		PC_in = s;
		entrada64 = -8;
	end
	else if(state == S_reset2) begin
		load_ir = 1;
	end
	else if(state == S_incPC_loadRegs) begin
		Wr64 = 0;
		writeReg = 0;
		load_PC = 1;
		if(PC_out > 64) begin
			$stop;
		end
		PC_in = s;
		loadRegA = 1;	
		loadRegB = 1;
		load_ir = 1;	
	end
	else if(state == S_Store1) begin
		load_ir = 0;
		load_PC = 0;
		loadMDR = 0;
		loadAOut = 1;	
	  	ALUSrcA = 3'b001;
       		ALUSrcB = 3'b100;	
		ALUfct = 3'd1;
	end
	else if (state == S_Store2)begin
		load_PC = 0;
		memToReg = 0;
		loadMDR = 0;
		load_ir = 0;
		Wr64 = 1;
	end
	else if(state == S_beginBranch) begin
		writeReg = 0;
		Wr64 = 0;
		load_PC = 0;
		load_ir = 0;	
		//PC_in = PC_in + 4;
		loadRegA = 1;	
		loadRegB = 1;
	end
	else if(state == S_opcode) begin
		load_ir = 0;
	end 
	else if(state == S_aluOp_I) begin
		load_ir = 0;
		load_PC = 0;
		loadAOut = 1;	
	  	ALUSrcA = 3'b001;
       		ALUSrcB = 3'b010;	
		ALUfct = 3'd1; 		
	end
	else if(state == S_loadMDR_I) begin
		loadMDR = 1;
	end	
	else if(state == S_memToReg_I)begin
		memToReg = 1;
	end
	else if(state == S_addi) begin
		//loadAOut = 0;
		memToReg = 0;
		loadMDR = 0;
	end
	else if(state == S_updateReg) begin
		loadAOut = 0;
		load_PC = 0;
		memToReg = 0;
		loadMDR = 0;
		load_ir = 0;
	end
	else if(state == S_writeReg_I) begin
		writeReg = 1;
	end
	else if(state == S_Sb_1) begin
		load_ir = 0;
		load_PC = 0;
		loadAOut = 1;
		ALUSrcA = 3'b0;
		ALUSrcB = 3'b011;
		ALUfct = 3'd1; 
	end
	else if(state == S_Sb_2) begin
		loadAOut = 0;
		ALUSrcA = 3'b001;
		ALUSrcB = 3'b000;
		ALUfct = 3'b010;
		//PC_out = Aout;
		//PC_in = PC_out + 4;
		//load_PC = 1;
		load_ir = 0;
		
	end
	else if(state == S_Sb_3) begin
		load_ir = 1;	
	end
	else if(state == S_branch) begin
		writeReg = 0;
		PC_out = Aout;
		PC_in = PC_out + 4;
		//$display("%d",PC_out);
		if(PC_out > 64) begin
			$stop;
		end
		//Wr64 = 0;
		//load_PC = 0;
		//PC_in = PC_in + 4;
		//loadRegA = 1;	
		//loadRegB = 1;
		//PC_in = s;
		
	end
	else if(state == S_updatePC) begin
		loadAOut = 0;
		load_PC = 0;
        	ALUSrcA = 3'b0;
		ALUSrcB = 3'b001;
		ALUfct = 3'd1; 
		load_ir = 1;
		
	end
	else if(state == S_add)begin
		load_PC = 0;
		load_ir = 0;
		loadAOut = 1;
		ALUSrcA = 3'b001;
		ALUSrcB = 3'b0;
		ALUfct = 3'b001;
		writeReg = 1;	
	end
	else if(state == S_sub)begin
		load_PC = 0;
		loadAOut = 1;
		load_ir = 0;
		ALUSrcA = 3'b001;
		ALUSrcB = 3'b0;
		ALUfct = 3'b010;	
		writeReg = 1;
	end
	else if(state == S_lui)begin
		load_PC = 0;
		loadAOut = 1;
		load_ir = 0;
		ALUSrcA = 3'b010;
		ALUfct = 3'b000;	
		writeReg = 1;
	end
end
endmodule


