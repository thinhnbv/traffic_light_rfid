module tag_priority_lut (
    input wire [31:0] uid,
    output reg [2:0] priority_class
);

    always @(*) begin
        case (uid) 
            32'hA1B2C3D4: priority_class = 3'b101;
            32'hE5F6A7B8: priority_class = 3'b100;
            32'h12345678: priority_class = 3'b011;
            32'h9ABCDEF0: priority_class = 3'b010;
            32'h11223344: priority_class = 3'b001;
            default:      priority_class = 3'b000;
        endcase
    end

endmodule