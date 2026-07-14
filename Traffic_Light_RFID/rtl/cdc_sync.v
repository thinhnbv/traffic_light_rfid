module cdc_sync #(parameter W = 1) (
    input wire clk_dst,
    input wire rst_n,
    input wire [W-1:0] d,
    output reg [W-1:0] q
);

    reg [W-1:0] meta;

    always @(posedge clk_dst or negedge rst_n) begin
        if (!rst_n) begin
            meta <= {W{1'b0}};
            q    <= {W{1'b0}};
        end else begin
            meta <= d;
            q    <= meta;
        end
    end

endmodule
