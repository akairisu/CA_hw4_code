module adder #(parameter DATA_W = 64)(a, b, out);
    input  [DATA_W - 1 : 0] a;
    input  [DATA_W - 1 : 0] b;
    output [DATA_W - 1 : 0] out;

    assign out = a + b;
endmodule

module signed_adder #(parameter ADDR_W = 64, parameter IMM_W = 13)(a, b, out);
    input  [ADDR_W - 1 : 0] a;
    input  [IMM_W - 1 : 0] b;
    output [ADDR_W - 1 : 0] out;

    wire b_sign;
    wire [ADDR_W - IMM_W - 1 : 0] one;
    wire [ADDR_W - IMM_W - 1 : 0] zero;
    wire [ADDR_W - 1 : 0]         signed_b;

    assign b_sign = b[IMM_W - 1];

    assign one = -1;
    assign zero = 0;
    assign signed_b = (b_sign) ? {one, b[11 : 0]} : {zero, b[11 : 0]};

    assign out = a + signed_b;
endmodule

module subtracter #(parameter DATA_W = 64)(a, b, out);
    input  [DATA_W - 1 : 0] a;
    input  [DATA_W - 1 : 0] b;
    output [DATA_W - 1 : 0] out;

    assign out = a - b;
endmodule

module alu #(
    parameter DATA_W = 64,
    parameter INST_W = 3
)(
    input                   i_clk,      //input signal
    input                   i_rst_n,    //input signal
    input                   i_alu_op,   //input signal
    input                   i_sub,      //input signal
    input                   i_valid,    //input signal
    input  [DATA_W-1:0]     i_data_a,
    input  [DATA_W-1:0]     i_data_b,
    input  [INST_W-1:0]     i_inst,
    output [DATA_W-1:0]     o_data,
    output                  o_zero,     //output signal
    output                  o_valid     //output signal
);
    reg [DATA_W-1:0]     o_data_w;
    reg                  o_zero_w;
    reg                  o_valid_w;

    wire [63 : 0]addout;
    wire [63 : 0]subout;

    adder #(.DATA_W(DATA_W)) ADD (.a(i_data_a), .b(i_data_b), .out(addout));
    subtracter #(.DATA_W(DATA_W)) SUB(.a(i_data_a), .b(i_data_b), .out(subout));

    always @(*) begin
        if(i_alu_op) begin
            case (i_inst)
                3'b000: //ADD, SUB, ADDI
                    begin
                        if(~i_sub) begin
                            o_data_w <= addout;
                        end else begin
                            o_data_w <= subout;
                        end
                    end
                3'b100: //XOR, XORI
                    begin
                        o_data_w <= (i_data_a ^ i_data_b);
                    end
                3'b110: //OR, ORI
                    begin
                        o_data_w <= (i_data_a | i_data_b);
                    end
                3'b111: //AND, ANDI
                    begin
                        o_data_w <= (i_data_a & i_data_b);
                    end
                3'b001: //SLLI
                    begin
                        o_data_w <= (i_data_a << i_data_b);
                    end
                3'b101: //SRLI
                    begin
                        o_data_w <= (i_data_a >> i_data_b);
                    end
                3'b011: //LD, SD
                    begin
                        o_data_w <= addout;
                    end
                default :
                    begin
                        o_data_w <= 0;
                    end
            endcase    
            o_zero_w <= 0;
            o_valid_w <= 1;  
        end else begin
            case (i_inst) 
                3'b000: //BEQ
                    begin
                        o_zero_w <= (i_data_a == i_data_b);
                    end
                3'b001: //BNE
                    begin
                        o_zero_w <= (i_data_a != i_data_b);
                    end
                default :
                    begin
                        o_zero_w <= 0;
                    end
            endcase
            o_data_w <= 0;
            o_valid_w <= 1;
        end
    end

    always @(posedge i_clk) begin
        if(~i_rst_n) begin
             o_data_w <= 0;
             o_zero_w <= 0;
             o_valid_w <= 0;
        end 
    end

    assign o_data = o_data_w;
    assign o_zero = o_zero_w;
    assign o_valid = o_valid_w;
endmodule