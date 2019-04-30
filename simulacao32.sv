`timescale 1ps/1ps
module simulacao32;

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
wire [63:0]outEPC;

// MUX REGISTER
logic [2:0]memToReg;
wire [63:0]saidaMuxReg;

wire [63:0]extensorSB;
wire [63:0]extensorU;
wire [63:0]extensorS;
wire [63:0]extensorUJ;


reg [63:0]var1;
reg [63:0]var0;


//SHIFT
wire [63:0] srliOut;
wire [63:0] sraiOut;
wire [63:0] slliOut;
wire [63:0] RegsrliOut;
wire [63:0] RegsraiOut;
wire [63:0] RegslliOut;
logic loadSRLIout, loadSRAIout, loadSLLIout;


reg [2:0]seletorB;
wire [63:0]outValB;

//LOAD
wire [63:0] loadOut;
reg [2:0] selLoad;


// FIOS E REGS DAS EXCECOES
reg selCausa;
reg loadCausa;
wire outCausa;
reg loadEPC;
reg [63:0] PC_inv;
reg [63:0] var255;
reg [63:0] var254;
reg [2:0] seladd;
reg [2:0] selPC;
wire [63:0] entPC;
wire [63:0]muxadd64s;

//Pinos

reg [63:0] MemData;
assign MemData = Dataout64;



reg [63:0] WAdress;
reg [63:0] RAdress;
assign WAdress = muxadd64s;
assign RAdress = Aout;

reg [63:0] WriteDataMem;
assign WriteDataMem = outValB;

reg [4:0] WriteRegister;
assign WriteRegister = instr11_7;

reg [63:0]WriteDataReg;
assign WriteDataReg = saidaMuxReg;

reg [63:0] MDR;
assign MDR = dataOutReg;

reg [63:0] Alu;
assign Alu = s;

reg [63:0]AluOut;
assign AluOut = Aout;

reg [63:0] PC;
assign PC = PC_out;

reg wr;
assign wr = Wr64;


reg RegWrite;
assign RegWrite = writeReg;

reg IRWrite;
assign IRWrite = load_ir;

reg [63:0] EPC;
assign EPC = outEPC;







mux3 muxPC (.a(PC_in), .b(loadOut),.c(qualquercoisa), .sel(selPC),.outMux(entPC));

ModReg regCause(.inVal(selCausa),.loadReg(loadCausa),.outVal(outCausa));

// LIGANDO OS ModULOS
PC ulapc (.raddress(entPC), .loadPC(load_PC), .Rst(rst), .Dataout(PC_out));

Memoria32 pcmem (.raddress(PC_out), .waddress(wdaddress),
.Clk(clk), .Datain(data), .Dataout(dataPath), .Wr(Wr) );

Instr_Reg_RISC_V meminst (.Clk(clk), .Reset(rst), .Load_ir(load_ir),
.Entrada(dataPath), .Instr19_15(instr19_15), .Instr24_20(instr24_20), .Instr11_7(instr11_7), .Instr6_0(instr6_0), .Instr31_0(instr31_0)); // Insere no registrador de instru??es

bancoReg instbanco (.write(writeReg), .clock(clk), .reset(rst), .regreader1(instr19_15), .regreader2(instr24_20), .regwriteaddress(instr11_7), 
.datain(saidaMuxReg), .dataout1(readData1), .dataout2(readData2));

ModReg regA (.inVal(readData1), .loadReg(loadRegA), .outVal(outRegA));

ModReg regB (.inVal(readData2), .loadReg(loadRegB), .outVal(outRegB));

ModReg EPCmod (.inVal(PC_inv),.loadReg(loadEPC),.outVal(outEPC));

Extensor signextendI (.inSignal(instr31_0), .outSignal(extensorI));

Extensor2 signextendSB (.inSignal(instr31_0), .outSignal(extensorSB));

Extensor3 extendeS(.inSignal(instr31_0),.outSignal(extensorS));

Extensor4 signextendU(.inSignal(instr31_0), .outSignal(extensorU));

Extensor5 signextendLoad(.inSignal(Dataout64),.select(selLoad),.outSignal(loadOut));

extensorUJ signextendUJ(.inSignal(instr31_0), .outSignal(extensorUJ));

//SHIFTS
shiftRL srli(.rin(outRegA),.immed(instr31_0[25:20]),.rd(srliOut));

shiftRA srai(.rin(outRegA),.immed(instr31_0[25:20]),.rd(sraiOut));

shift slli(.rin(outRegA),.immed(instr31_0[25:20]),.rd(slliOut));

ModReg SHIFToutSRLI (.inVal(srliOut), .loadReg(loadSRLIout), .outVal(RegsrliOut));

ModReg SHIFToutSRAI (.inVal(sraiOut), .loadReg(loadSRAIout), .outVal(RegsraiOut));

ModReg SHIFToutSLLI (.inVal(slliOut), .loadReg(loadSLLIout), .outVal(RegslliOut));
//--------

mux3 muxUlaA (.a(PC_out), .b(outRegA), .c(extensorU), .sel(ALUSrcA),.outMux(saidaMuxALUA));

mux3 muxAdd64 (.a(Aout), .b(var255), .c(var254), .sel(seladd),.outMux(muxadd64s));

mux2 muxUlaB (.a(outRegB), .b(d4), .c(extensorI), .d(extensorSB), .e(extensorS),.f(extensorUJ), .g(Aout), .sel(ALUSrcB), .outMux(saidaMuxALUB) );

ula64 aritmop (.A(saidaMuxALUA), .B(saidaMuxALUB), .Seletor(ALUfct), .S(s), .Overflow(overFlow), .Negativo(negativo), .Z(z), .Igual(igual), .Maior(maior), .Menor(menor)); // Soma mais 4com o PC

ModReg ALUout (.inVal(s), .loadReg(loadAOut), .outVal(Aout));

ValorLido valB (.outMem64(Dataout64), .outB(outRegB), .seletor(seletorB), .outVal(outValB)); 

Memoria64 ALUtomem(.raddress(muxadd64s), .waddress(Aout), .Clk(clk), .Datain(outValB), .Dataout(Dataout64), .Wr(Wr64));

register MDRmod(.clk(clk), .reset(rst), .regWrite(loadMDR), .DadoIn(loadOut), .DadoOut(dataOutReg));

mux4 muxWriteDataReg(.a(Aout), .b(dataOutReg), .c(var1), .d(var0),.e(RegsrliOut),.f(RegsraiOut),.g(RegslliOut),.h(PC_out),.sel(memToReg), .outMux(saidaMuxReg));

// ESTADOS DA MAQUINA DE ESTADOS
enum{S_reset1,S_reset2,S_incPC_loadRegs,S_opcode,S_aluOp_I,S_loadMDR_I,S_memToReg_I,S_addi,S_writeReg_I,S_updatePC,S_Sb_1,S_Sb_2,S_Sb_3,S_branch,S_beginBranch,S_add, S_sub,S_and,S_slt,S_updateReg,S_lui,S_Store1,S_Store2,S_Store2_w,S_Store2_h,S_Store2_b,S_srli,S_srai,S_slli,S_stli,S_jal_1,S_jal_2,S_jal_3,S_jal_Branch,S_jalr_1,S_jalr_2,S_jalr_3,S_jalr_Branch, S_opindef, S_opindef2,S_over,S_over2}Estado;

//gerador de clock e reset
localparam CLKPERIOD = 10000;
localparam CLKDELAY = CLKPERIOD / 2;

initial begin
clk = 1'b1;
Wr = 0;
d4 = 64'd4;
var1 = 64'd1;
var0 = 64'd0;
rst = 1'b1;
writeData = 0;
var255 = 64'd255;
var254 = 64'd254;
end

always #(CLKDELAY) clk = ~clk;

always_ff @(posedge clk or posedge rst)
begin
	if(rst) begin
		Estado <= S_reset1;
		rst = 1'b0;
	end
	else begin
		if(overFlow)begin
			selLoad = 3'b100;
			Estado <= S_over;
		end
		else if(Estado == S_reset1) Estado <= S_reset2;
		else if(Estado == S_reset2) Estado <= S_incPC_loadRegs;
		else if(Estado == S_incPC_loadRegs) Estado <=  S_opcode;
		else if(Estado ==  S_opcode) begin
			if(instr6_0 == 7'b0010011 && instr31_0[14:12] == 3'b000 && instr31_0[11:7] == 5'b00000) Estado <= S_updatePC;
			else if(instr6_0 == 7'b1110011) $stop;
			else if(instr31_0[14:12] == 3'b010 && instr6_0 == 7'b0010011) Estado <= S_stli;
			else if(instr6_0 == 7'b0000011 || instr6_0 == 7'b0010011) Estado <= S_aluOp_I;
			else if(instr6_0 == 7'b1100111 && instr31_0[14:12] == 3'b000) Estado <= S_jalr_1;
			else if(instr6_0 == 7'b1100011 || instr6_0 == 7'b1100111) Estado <= S_Sb_1;
			else if(instr6_0 == 7'b0110011 && instr31_0[14:12] == 3'b000 &&  instr31_0[31:25] == 7'b0000000) Estado <= S_add;
			else if(instr6_0 == 7'b0110011 && instr31_0[14:12] == 3'b000 && instr31_0[31:25] == 7'b0100000) Estado <= S_sub;
			else if(instr6_0 == 7'b0110011 && instr31_0[14:12] == 3'b111) Estado <= S_and;
			else if(instr6_0 == 7'b0110011 && instr31_0[14:12] == 3'b010) Estado <= S_slt;
			else if(instr6_0 == 7'b0110111) Estado <= S_lui;
			else if(instr6_0 == 7'b0100011) Estado <= S_Store1;
			else if(instr6_0 == 7'b1101111) Estado <= S_jal_1;
			else begin
				selLoad = 3'b100;
				Estado <= S_opindef;
			end
		end
		else if (Estado == S_aluOp_I) begin
			if(instr6_0 == 7'b0000011 && instr31_0[14:12] == 3'b000)begin //lb
				selLoad <= 3'b000;
				Estado <= S_loadMDR_I;
			end
			else if(instr6_0 == 7'b0000011 && instr31_0[14:12] == 3'b001)begin //lh
				selLoad <= 3'b001;
				Estado <= S_loadMDR_I;
			end
			else if(instr6_0 == 7'b0000011 && instr31_0[14:12] == 3'b010)begin //lw
				selLoad <= 3'b010;
				Estado <= S_loadMDR_I;
			end
			else if(instr6_0 == 7'b0000011 && instr31_0[14:12] == 3'b011)begin //ld
				selLoad <= 3'b011;
				Estado <= S_loadMDR_I;
			end
			else if(instr6_0 == 7'b0000011 && instr31_0[14:12] == 3'b100)begin //lbu
				selLoad <= 3'b100;
				Estado <= S_loadMDR_I;
			end
			else if(instr6_0 == 7'b0000011 && instr31_0[14:12] == 3'b101)begin //lhu
				selLoad <= 3'b101;
				Estado <= S_loadMDR_I;
			end
			else if(instr6_0 == 7'b0000011 && instr31_0[14:12] == 3'b110)begin //lhu
				selLoad <= 3'b110;
				Estado <= S_loadMDR_I; 
			end

			else if(instr31_0[14:12] == 3'b101 && instr31_0[31:26] == 6'b000000) Estado <= S_srli;
			else if(instr31_0[14:12] == 3'b101 && instr31_0[31:26] == 6'b010000) Estado <= S_srai;
			else if(instr31_0[14:12] == 3'b001 && instr31_0[31:26] == 6'b000000) Estado <= S_slli;
			else if(instr31_0[14:12] == 3'b00 && instr31_0[6:0] == 7'b0010011) Estado <= S_addi;
		end
      		else if(Estado == S_loadMDR_I) Estado <= S_memToReg_I;
		else if(Estado == S_memToReg_I) Estado <= S_writeReg_I;
		else if(Estado == S_addi) Estado <= S_writeReg_I;
		else if(Estado == S_writeReg_I) Estado <= S_updatePC;
		else if(Estado == S_Sb_1) Estado <= S_Sb_2;
		else if (Estado == S_Store1)begin
			if(instr31_0[14:12] == 3'b111) Estado <= S_Store2;
			else if(instr31_0[14:12] == 3'b010) Estado <= S_Store2_w;
			else if(instr31_0[14:12] == 3'b001) Estado <= S_Store2_h;
			else if(instr31_0[14:12] == 3'b000) Estado <= S_Store2_b;
		end
		else if(Estado==S_opindef)Estado <= S_opindef2;
		else if(Estado == S_opindef2) Estado <= S_updatePC; 
		else if(Estado == S_over) Estado <= S_over2;
		else if(Estado == S_over2) Estado <= S_updatePC;
		else if(Estado == S_Store2) Estado <= S_updatePC;
		else if(Estado == S_Store2_w) Estado <= S_updatePC;
		else if(Estado == S_Store2_h) Estado <= S_updatePC;
		else if(Estado == S_Store2_b) Estado <= S_updatePC;
		else if(Estado == S_jal_1) Estado <= S_jal_2;
		else if(Estado == S_jal_2) Estado <= S_jal_3;
		else if(Estado == S_jal_3) Estado <= S_jal_Branch;
		else if(Estado == S_jal_Branch) Estado <= S_updatePC;
		else if(Estado == S_jalr_1) Estado <= S_jalr_2;
		else if(Estado == S_jalr_2) Estado <= S_jalr_Branch;
		else if(Estado == S_jalr_Branch) Estado <= S_updatePC;
		else if(Estado == S_Sb_2)begin
			if(instr6_0 == 7'b1100011)begin
				if(z == 1) Estado <= S_branch;   //BEQ
				else Estado <= S_updatePC;
			end
			else if(instr6_0 == 7'b1100111 && instr31_0[14:12] == 3'b001) begin  //BNE
				if(z == 0) Estado <= S_branch;
				else Estado <= S_updatePC;
			end
			else if(instr6_0 == 7'b1100111 && instr31_0[14:12] == 3'b100) begin  //BLT
				if(menor) Estado <= S_branch;
				else Estado <= S_updatePC;
			end
			else if(instr6_0 == 7'b1100111 && instr31_0[14:12] == 3'b101) begin  //BGE
				if(menor) Estado <= S_updatePC;
				else Estado <= S_branch;
			end
		end
		else if(Estado == S_Sb_3) Estado <= S_branch;
		else if(Estado == S_add) Estado <= S_updateReg;
		else if(Estado == S_sub) Estado <= S_updateReg;
		else if(Estado == S_and) Estado <= S_updateReg;
		else if(Estado == S_slt) Estado <= S_updateReg;
		else if(Estado == S_stli) Estado <= S_updateReg;
		else if(Estado == S_updateReg) Estado <= S_updatePC;
		else if(Estado == S_lui) Estado <= S_updateReg;
		else if(Estado == S_updateReg) Estado <= S_updatePC;
		else if(Estado == S_branch) Estado <= S_updatePC;
		else if(Estado == S_beginBranch) Estado <= S_opcode;
		else if(Estado == S_updatePC) Estado <= S_incPC_loadRegs;
		else if(Estado == S_srli || Estado == S_srai || Estado == S_slli) Estado <= S_writeReg_I; 

		end
end

always_comb begin

	if(Estado == S_over)begin
		selCausa = 1;
		loadCausa = 1; 
		PC_inv = PC_out;
		loadEPC = 1;

		seladd = 3'b001;
		selPC = 3'b001;
	end
	else if(Estado == S_over2) begin
		loadEPC = 0;
		load_PC = 0;
	end
	else if(Estado == S_reset1)begin
		writeReg = 0;
		ALUSrcA = 3'b0;
		ALUSrcB = 3'b001;
		ALUfct = 3'd1; 
		PC_in = s;
		entrada64 = -8;
		selPC = 3'b000;
	end
	else if(Estado == S_reset2) begin
		load_ir = 1;
	end
	else if(Estado == S_incPC_loadRegs) begin
	// espero q n de merda	
		seladd = 3'b000;
		selPC = 3'b000;

		Wr64 = 0;
		writeReg = 0;
		load_PC = 1;
		PC_in = s;
		loadRegA = 1;	
		loadRegB = 1;
		load_ir = 0;
	end
	else if(Estado == S_Store1) begin
		load_ir = 0;
		load_PC = 0;
		loadMDR = 0;
		loadAOut = 1;	
	  	ALUSrcA = 3'b001;
       		ALUSrcB = 3'b100;	
		ALUfct = 3'd1;
		seletorB = 3'b000;
		Wr64 = 0;
	end
	else if (Estado == S_Store2)begin
		load_PC = 0;
		memToReg = 3'b000;
		loadMDR = 0;
		load_ir = 0;
		Wr64 = 1;
	end
	else if(Estado == S_Store2_w)begin
		seletorB = 3'b001;
		load_PC = 0;
		memToReg = 3'b000;
		loadMDR = 0;
		load_ir = 0;
		Wr64 = 1;		
	end
	else if(Estado == S_Store2_h)begin
		seletorB = 3'b010;
		load_PC = 0;
		memToReg = 3'b000;
		loadMDR = 0;
		load_ir = 0;
		Wr64 = 1;		
	end
	else if(Estado == S_Store2_b)begin
		seletorB = 3'b011;
		load_PC = 0;
		memToReg = 3'b000;
		loadMDR = 0;
		load_ir = 0;
		Wr64 = 1;		
	end
	else if(Estado == S_beginBranch) begin
		writeReg = 0;
		Wr64 = 0;
		load_PC = 0;
		load_ir = 0;	
		loadRegA = 1;	
		loadRegB = 1;
	end
	else if(Estado == S_opcode) begin
		load_ir = 0;
	end 
	else if(Estado == S_aluOp_I) begin
	
		load_ir = 0;
		load_PC = 0;
		loadAOut = 1;

		loadSRLIout = 1;
		loadSRAIout = 1;
		loadSLLIout = 1;

	  	ALUSrcA = 3'b001;
       		ALUSrcB = 3'b010;	
		ALUfct = 3'd1; 
	end
	else if(Estado == S_opindef) begin
		selCausa = 0;
		loadCausa = 1; 
		PC_inv = PC_out ;
		loadEPC = 1;

		seladd = 3'b010;
		selPC = 3'b001;
		load_PC = 1;
	end
	else if(Estado == S_opindef2) begin
		loadEPC = 0;
		load_PC = 0;
		loadCausa = 0; 

	end
	else if(Estado == S_loadMDR_I) begin
		loadMDR = 1;
	end	
	else if(Estado == S_memToReg_I)begin
		memToReg = 1;
	end
	else if(Estado == S_addi) begin
		//loadAOut = 0;
		memToReg = 3'b000;
		loadMDR = 0;
	end
	else if(Estado == S_updateReg) begin
		loadAOut = 0;
		load_PC = 0;
		memToReg = 3'b000;
		loadMDR = 0;
		load_ir = 0;
		writeReg = 1;
	end
	else if(Estado == S_writeReg_I) begin
		writeReg = 1;
	end
	else if(Estado == S_Sb_1) begin
		load_ir = 0;
		load_PC = 0;
		loadAOut = 1;
		ALUSrcA = 3'b0;
		ALUSrcB = 3'b011;
		ALUfct = 3'd1; 
	end
	else if(Estado == S_jal_1) begin
		load_ir = 0;
		load_PC = 0;
		loadAOut = 1;
		ALUSrcA = 3'b0;
		ALUSrcB = 3'b101;
		ALUfct = 3'd1; 
	end
	else if(Estado == S_jal_2) begin
		load_ir = 0;
		loadAOut = 0;
		memToReg = 3'b110;
		writeReg = 1;
	end
	else if(Estado == S_jal_Branch) begin
		writeReg = 0;
		PC_out = Aout;
		PC_in = PC_out + 4;
		//$display("%d",PC_out);
	end
	else if(Estado == S_jalr_1) begin
		load_ir = 0;
		load_PC = 0;
		writeReg = 0;
		memToReg = 3'b110;
		loadAOut = 1;
		ALUSrcA = 3'b1;
		ALUSrcB = 3'b010;
		ALUfct = 3'd1; 
	end
	else if(Estado == S_jalr_2) begin
		load_ir = 0;
		loadAOut = 0;
		writeReg = 1;
		ALUSrcA = 3'b0;
		ALUSrcA = 3'b110;
		ALUfct = 3'd1;
	end
	else if(Estado == S_jalr_Branch) begin
		writeReg = 0;
		PC_out = s;
		PC_in = PC_out + 4;
		//$display("%d",PC_out);	
	end
	else if(Estado == S_Sb_2) begin
		loadAOut = 0;
		ALUSrcA = 3'b001;
		ALUSrcB = 3'b000;
		ALUfct = 3'b010;
		load_ir = 0;		
	end
	else if(Estado == S_Sb_3) begin
		load_ir = 1;	
	end
	else if(Estado == S_branch) begin
		writeReg = 0;
		PC_out = Aout;
		PC_in = PC_out + 4;
		//$display("%d",PC_out);		
	end
	else if(Estado == S_updatePC) begin

		//teste
		loadSRLIout = 0;
		loadSRAIout = 0;
		loadSLLIout = 0;
		//--------

		loadAOut = 0;
		load_PC = 0;
        	ALUSrcA = 3'b0;
		ALUSrcB = 3'b001;
		ALUfct = 3'd1; 
		load_ir = 1;
		writeReg = 0;
		Wr64 = 0;
	end
	else if(Estado == S_add)begin
		load_PC = 0;
		load_ir = 0;
		loadAOut = 1;
		ALUSrcA = 3'b001;
		ALUSrcB = 3'b000;
		ALUfct = 3'b001;
		writeReg = 1;	
	end
	else if(Estado == S_sub)begin
		load_PC = 0;
		loadAOut = 1;
		load_ir = 0;
		ALUSrcA = 3'b001;
		ALUSrcB = 3'b000;
		ALUfct = 3'b010;	
		writeReg = 1;
	end
	else if(Estado == S_and)begin
		load_PC = 0;
		loadAOut = 1;
		load_ir = 0;
		ALUSrcA = 3'b001;
		ALUSrcB = 3'b000;
		ALUfct = 3'b011;	
		writeReg = 1;
	end
	else if(Estado == S_slt)begin
		load_PC = 0;
		loadAOut = 1;
		load_ir = 0;
		ALUSrcA = 3'b001;
		ALUSrcB = 3'b000;
		ALUfct = 3'b011;
		if(menor) memToReg = 3'b010;
		else memToReg = 3'b011;
		writeReg = 0;
	end
	else if(Estado == S_lui)begin
		load_PC = 0;
		loadAOut = 1;
		load_ir = 0;
		ALUSrcA = 3'b010;
		ALUfct = 3'b000;	
		writeReg = 1;
	end
	else if(Estado == S_srli)begin
		memToReg = 3'b100;
		loadMDR = 0;
	end
	else if(Estado == S_srai)begin
		memToReg = 3'b101;
		loadMDR = 0;	
	end
	else if(Estado == S_slli)begin
		memToReg = 3'b111;
		loadMDR = 0;
	end
	else if(Estado == S_stli)begin
			load_PC = 0;
			loadAOut = 1;
			load_ir = 0;
			ALUSrcA = 3'b001;
			ALUSrcB = 3'b010;
			ALUfct = 3'b011;
			if(menor) memToReg = 3'b010;
			else memToReg = 3'b011;
			writeReg = 0;
	end	
end
endmodule
