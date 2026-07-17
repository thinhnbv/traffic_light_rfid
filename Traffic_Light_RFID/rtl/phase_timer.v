//
//
//

module phase_timer (
    input clk,
    input rst_n,
    input load,
    input [31:0] value,
    output expired
);

    reg [31:0] counter;

    always @(posedge clk) begin
        if (!rst_n) 
            counter <= 0;
        else if (load) 
            counter <= value;
        else if (counter != 0) 
            counter <= counter - 1;
    end

    assign expired = (counter == 32'd1) && !load;

endmodule