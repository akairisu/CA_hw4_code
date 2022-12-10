module mux #(
    parameter DATA_W = 64
)(
    input  [DATA_W - 1 : 0] i_data_a,
    input  [DATA_W - 1 : 0] i_data_b,
    input                   control,
    output [DATA_W - 1 : 0] o_data
);
    assign o_data = (control) ? i_data_a : i_data_b;
endmodule

module control #(
    parameter INST_W = 7
)(
    input  [INST_W - 1 : 0] opcode, 
    output                  branch,   //signal for branch
    output                  MemRead,  //signal for ld
    output                  MemtoReg, //signal for ld
    output                  ALUOp,    //signal for alu or branch
    output                  MemWrite, //signal for sd
    output                  ALUSrc,   //signal for immediate
    output                  RegWrite  //signal for write result
);
    assign branch   = (opcode == 7'b1100011) ? 1 : 0; 
    assign MemRead  = (opcode == 7'b0000011) ? 1 : 0;
    assign MemtoReg = (opcode == 7'b0000011) ? 1 : 0;
    assign ALUOp    = (opcode != 7'b1100011) ? 1 : 0; //not branch
    assign MemWrite = (opcode == 7'b0100011) ? 1 : 0;
    assign ALUSrc   = ((opcode != 7'b0110011) && (opcode != 7'b1100011)) ? 1 : 0; // not R-type or branch 
    assign RegWrite = ((opcode != 7'b0100011) && (opcode != 7'b1100011)) ? 1 : 0; // not SD or branch
endmodule

module cpu #( // Do not modify interface
	parameter ADDR_W = 64,
	parameter INST_W = 32,
	parameter DATA_W = 64
)(
    input                   i_clk,
    input                   i_rst_n,
    input                   i_i_valid_inst, // from instruction memory
    input  [ INST_W-1 : 0 ] i_i_inst,       // from instruction memory
    input                   i_d_valid_data, // from data memory
    input  [ DATA_W-1 : 0 ] i_d_data,       // from data memory
    output                  o_i_valid_addr, // to instruction memory
    output [ ADDR_W-1 : 0 ] o_i_addr,       // to instruction memory
    output [ DATA_W-1 : 0 ] o_d_w_data,       // to data memory
    output [ ADDR_W-1 : 0 ] o_d_w_addr,     // to data memory
    output [ ADDR_W-1 : 0 ] o_d_r_addr,     // to data memory
    output                  o_d_MemRead,    // to data memory
    output                  o_d_MemWrite,   // to data memory
    output                  o_finish
);
    reg [INST_W-1 : 0] valid_inst;
    reg [DATA_W-1 : 0] valid_data;
    reg [DATA_W-1 : 0] o_d_w_data;
    reg [ADDR_W-1 : 0] o_d_w_addr;
    reg [ADDR_W-1 : 0] o_d_r_addr;
    reg [ADDR_W-1 : 0] o_i_addr;
    reg                o_i_valid_addr;
    reg                o_d_MemRead;
    reg                o_d_MemWrite;
    reg                o_finish;
    reg                o_r_data;

    reg [INST_W-1 : 0] valid_inst_w;
    reg [DATA_W-1 : 0] valid_data_w;
    reg [DATA_W-1 : 0] o_d_data_w;
    reg [ADDR_W-1 : 0] o_d_addr_w;
    reg [ADDR_W-1 : 0] o_i_addr_w;
    reg                o_i_valid_addr_w;
    reg                o_d_MemRead_w;
    reg                o_d_MemWrite_w;
    reg                o_finish_w;
    reg [DATA_W-1 : 0] o_r_data_w;

    always @(*) begin
        if(i_i_inst == 32'b11111111111111111111111111111111) begin
            o_finish_w <= 1;
        end else begin
        	o_finish_w <= 0;
        end
    end

    always @(posedge i_clk) begin
        if(~i_rst_n) begin
            o_d_w_data <= 0;
            o_d_w_addr <= 0;
            o_d_r_addr <= 0;
            o_i_addr <= 0;
            o_i_valid_addr <= 0;
            o_d_MemRead <= 0;
            o_d_MemWrite <= 0;
            o_finish <= 0;
            valid_inst <= 0;
            valid_data <= 0;
        end else begin
            o_d_w_data <= o_d_data_w;
            if(o_d_MemRead_w) begin
            	o_d_r_addr <= o_d_addr_w;
            end
            if(o_d_MemWrite_w) begin
            	o_d_w_addr <= o_d_addr_w;
            end
            o_i_addr <= o_i_addr_w;
            o_i_valid_addr <= o_i_valid_addr_w;
            o_d_MemRead <= o_d_MemRead_w;
            o_d_MemWrite <= o_d_MemWrite_w;
            o_finish <= o_finish_w;
            valid_inst <= valid_inst_w;
            valid_data <= valid_data_w;
        end 
    end

    always @(*) begin
        if (i_i_valid_inst) begin
            valid_inst_w <= i_i_inst;
        end else begin
            valid_inst_w <= valid_inst;
        end
    end

    always @(*) begin
        if (i_d_valid_data) begin
            valid_data_w <= i_d_data;
        end else begin
            valid_data_w <= valid_data;
        end
    end

    //instruction
    wire    [4 : 0]   rs1;
    wire    [4 : 0]   rs2;
    wire    [4 : 0]   rd;
    wire    [31 : 0]  instruction;
    wire    [6 : 0]   opcode;
    wire              sub;
    wire    [2 : 0]   funct_3;
    //control
    wire    branch;
    wire    MemRead;
    wire    MemtoReg;
    wire    ALUOp;
    wire    MemWrite;
    wire    ALUSrc;
    wire    RegWrite;
    wire    i_RegRead1;
    wire    i_RegRead2;

    assign  rs1 = valid_inst_w[19 : 15];
    assign  rs2 = valid_inst_w[24 : 20];
    assign  rd  = valid_inst_w[11 : 7];
    assign  instruction = valid_inst_w[31 : 0];
    assign  opcode = valid_inst_w[6 : 0];
    assign  sub = valid_inst_w[30];
    assign  funct_3 = valid_inst_w[14 : 12];

    assign i_RegRead1 = i_i_valid_inst;
    assign i_RegRead2 = i_i_valid_inst;

    control  #(
        .INST_W(7)
    ) CONTROL (
        .opcode(opcode),
        .branch(branch),
        .MemRead(MemRead),
        .MemtoReg(MemtoReg),
        .ALUOp(ALUOp),
        .MemWrite(MemWrite),
        .ALUSrc(ALUSrc),
        .RegWrite(RegWrite)
    );

    always @(*) begin
        o_d_MemRead_w <= MemRead;
        o_d_MemWrite_w <= MemWrite;
    end

    wire [DATA_W-1 : 0] o_data_rs1;
    wire [DATA_W-1 : 0] o_data_rs2;
    wire                reg_valid;

    register #(
        .REG_ADDR(5),
        .DATA_W(DATA_W)
    ) registers (
        .i_clk(i_clk),
        .i_rst_n(i_rst_n),
        .i_RegWrite(RegWrite),
        .i_RegRead1(i_RegRead1),
        .i_RegRead2(i_RegRead2),
        .rs1(rs1),
        .rs2(rs2),
        .rd(rd),
        .i_data(o_r_data_w),
        .o_data_rs1(o_data_rs1),
        .o_data_rs2(o_data_rs2),
        .o_valid(reg_valid)
    );

    always @(*) begin
        o_d_data_w <= o_data_rs2;
    end

    wire [DATA_W-1 : 0] immediate;
    wire [11 : 0]		short_immediate;
    wire                imm_valid;
    
    immgen #(
        .INST_W(INST_W),
        .IMM_W(DATA_W)
    ) ImmGen (
        .i_valid(i_i_valid_inst),
        .i_clk(i_clk),
        .i_rst_n(i_rst_n),
        .i_inst(instruction),
        .o_imm(immediate),
        .o_valid(imm_valid),
        .o_simm(short_immediate)
    );

    wire [DATA_W-1 : 0] rs2_data;

    mux #(
        .DATA_W(DATA_W)
    ) rs2mux (
        .i_data_a(immediate),
        .i_data_b(o_data_rs2),
        .control(ALUSrc),
        .o_data(rs2_data)
    );

    wire alu_in_valid;
    wire o_zero;
    wire [DATA_W-1 : 0] o_a_data;
    wire alu_valid;

    assign alu_in_valid = reg_valid & imm_valid;

    alu #(
        .DATA_W(DATA_W),
        .INST_W(3)
    ) ALU (
        .i_clk(i_clk),
        .i_rst_n(i_rst_n),
        .i_alu_op(ALUOp),
        .i_sub(sub),
        .i_valid(alu_in_valid),
        .i_data_a(o_data_rs1),
        .i_data_b(rs2_data),
        .i_inst(funct_3),
        .o_data(o_a_data),
        .o_zero(o_zero),
        .o_valid(alu_valid)
    );

    always @(*) begin
        o_d_addr_w <= o_a_data;
    end

    wire [DATA_W-1 : 0] o_memmux;

    mux #(
        .DATA_W(DATA_W)
    ) memmux (
        .i_data_a(valid_data_w),
        .i_data_b(o_a_data),
        .control(MemtoReg),
        .o_data(o_memmux)
    );

    always @(*) begin
        o_r_data_w <= o_memmux;
    end

    wire [ADDR_W-1 : 0] i_i_addr;
    wire                o_pc_valid;
    wire [ADDR_W-1 : 0] o_pc_addr;

    program_counter #(
        .ADDR_W(ADDR_W)
    ) PC (
        .i_clk(i_clk),
        .i_rst_n(i_rst_n),
        .i_addr(i_i_addr),
        .i_valid(i_i_valid_inst),
        .o_valid(o_pc_valid),
        .o_addr(o_pc_addr)
    );

    always @(*) begin
        o_i_valid_addr_w <= o_pc_valid;
        o_i_addr_w <= o_pc_addr;
    end

    wire [DATA_W-1 : 0] four;
    wire [DATA_W-1 : 0] next_inst;
    
    assign four = 4;

    adder #(.DATA_W(DATA_W)) PCADD (.a(o_i_addr_w), .b(four), .out(next_inst));

    wire zero;
    wire [12 : 0] branch_imm;
    wire [DATA_W-1 : 0] branch_inst;

    assign zero = 0;
    assign branch_imm = {short_immediate, zero};

    signed_adder #(.ADDR_W(ADDR_W), .IMM_W(13)) BRANCHADD (.a(o_i_addr_w), .b(branch_imm), .out(branch_inst));

    wire                pcmux_control;

    assign pcmux_control = branch & o_zero;

    mux #(
        .DATA_W(DATA_W)
    ) pcmux (
        .i_data_a(branch_inst),
        .i_data_b(next_inst),
        .control(pcmux_control),
        .o_data(i_i_addr)
    );
endmodule
