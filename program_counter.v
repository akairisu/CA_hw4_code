module program_counter #(
	parameter ADDR_W = 64
)(
	input					i_clk,		//input signal
	input					i_rst_n,	//input signal
	input  [ADDR_W - 1 : 0] i_addr,
	input  					i_valid,	//input signal
	output					o_valid,	//output signal
	output [ADDR_W - 1 : 0] o_addr	
);
	reg	[ADDR_W - 1 : 0] o_addr_w;
	reg		  			 o_valid_w;
	reg [4:0] 		     cs, ns;

	always @(*) begin
        case (cs)
        	0: ns = (i_valid) ? 1 : 0;
        	1: ns = 2;
        	2: ns = 3;
        	3: ns = 0;
            default: ns = (i_valid) ? 1 : 0;
        endcase
    end

	always @(posedge i_clk) begin
        if(~i_rst_n) begin
        	o_addr_w <= 0;
        	cs <= 0;
        	o_valid_w <= 1;
        end else begin
        	cs = ns;
        	if(cs == 3) begin
	    		o_addr_w = i_addr;
	    		o_valid_w = 1;
	    	end else begin
	    		o_valid_w = 0;
	    	end
        end 
    end

    assign o_valid = o_valid_w;
    assign o_addr = o_addr_w;
endmodule