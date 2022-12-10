module register #(
	parameter REG_ADDR = 5,
	parameter DATA_W = 64 
)(
	input			 			 i_clk,		 //input signal
	input  			 			 i_rst_n,	 //input signal
	input						 i_RegWrite, //input signal
	input						 i_RegRead1, //input signal
	input						 i_RegRead2, //input signal
	input   [REG_ADDR - 1 : 0]	 rs1,
	input   [REG_ADDR - 1 : 0]   rs2,
	input	[REG_ADDR - 1 : 0]	 rd,
	input	[DATA_W - 1 : 0] 	 i_data,
	output	[DATA_W - 1 : 0]	 o_data_rs1,
	output	[DATA_W - 1 : 0]	 o_data_rs2,
	output						 o_valid  	 //output signal
);
	reg [DATA_W - 1 : 0] registers [0 : 31];
	reg o_data_rs1;
	reg o_data_rs2;
	reg o_valid_1;
	reg o_valid_2;
	integer i;

	assign o_valid = o_valid_1 & o_valid_2;

	always @(posedge i_clk) begin
		registers[0] <= 0;
		if(~i_rst_n) begin
			for(i = 0; i < 32; i = i + 1)
				registers[i] <= 0;
		end	
		if(i_rst_n && i_RegWrite) begin
			registers[rd] <= i_data;
		end
	end

	always @(*) begin
		if(~i_rst_n || rs1 == 0) begin
			o_data_rs1 <= 0;
			o_valid_1 <= i_rst_n;
		end
		else if(i_RegRead1) begin
			o_data_rs1 <= registers[rs1];
			o_valid_1 <= 1;
		end else begin
			o_valid_1 <= 0;
		end
	end

	always @(*) begin
		if(~i_rst_n || rs2 == 0) begin
			o_data_rs2 <= 0;
			o_valid_2 <= i_rst_n;
		end
		else if(i_RegRead2) begin
			o_data_rs2 <= registers[rs2];
			o_valid_2 <= 1;
		end else begin
			o_valid_2 <= 0;
		end
	end
endmodule

module immgen #(
	parameter INST_W = 32,
	parameter IMM_W = 64,
	parameter SIMM_W = 12
)(
	input					i_valid,	//input signal
	input                   i_clk,      //input signal
    input                   i_rst_n,	//input signal
    input  [INST_W - 1 : 0] i_inst,
	output [IMM_W - 1 : 0]  o_imm,
	output				 	o_valid,     //output signal
	output [SIMM_W - 1 : 0] o_simm
);
	wire [6 : 0]  opcode;
	wire [2 : 0]  funct_3;
	reg  [11 : 0] immediate;	
	reg			  o_valid_w;		
	wire [51 : 0] zero;

	assign opcode = i_inst[6 : 0];
	assign funct_3 = i_inst[14 : 12];
	assign zero = 0;

	always @(*) begin
		case(opcode)
			7'b0000011: //LD
				begin
					immediate = i_inst[31 : 20];
				end
			7'b0100011: //SD
				begin
					immediate = {i_inst[31 : 25], i_inst[11 : 7]};
				end
			7'b1100011: //BEQ, BNE
				begin
					immediate = {i_inst[31], i_inst[7], i_inst[30 : 25], i_inst[11 : 8]};
				end
			7'b0010011: //ALU
				begin
					if((funct_3 == 3'b001) || (funct_3 == 3'b101))
						immediate = i_inst[24 : 20];
					else
						immediate = i_inst[31 : 20];
				end
			default:
				begin
					immediate = 0;
				end
		endcase 
		o_valid_w <= 1;
	end

	always @(posedge i_clk or negedge i_rst_n) begin
        if(~i_rst_n) begin
        	immediate <= 0;
        	o_valid_w <= 0;
        end 
    end
    assign o_imm = {zero, immediate};
    assign o_simm = immediate;
    assign o_valid = o_valid_w;
endmodule