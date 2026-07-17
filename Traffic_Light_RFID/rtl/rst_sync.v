//
//
//

module rst_sync (
    input clk,
    input rst_async_n,
    output reg rst_sync_n
);

    reg rst_meta;

    always @(posedge clk or negedge rst_async_n) begin
        if (!rst_async_n) begin
            rst_meta   <= 1'b0;
            rst_sync_n <= 1'b0;
        end else begin
            rst_meta   <= 1'b1;
            rst_sync_n <= rst_meta;
        end
    end

endmodule