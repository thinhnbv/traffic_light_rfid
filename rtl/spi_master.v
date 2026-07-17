module spi_master (
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire [7:0] tx_data,
    input wire [2:0] clk_div,
    input wire spi_miso,
    output reg spi_sck,
    output reg spi_mosi,
    output reg [7:0] rx_data,
    output reg done
);

    reg [7:0] shift_reg;
    reg [2:0] bit_cnt;
    reg [3:0] clk_cnt;
    reg sck_internal;
    reg running;

    // Reset block
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shift_reg <= 8'b0;
            bit_cnt   <= 3'b0;
            clk_cnt   <= 4'b0;
            sck_internal <= 1'b0;
            running   <= 1'b0;
            done      <= 1'b0;
            spi_sck   <= 1'b0;
            spi_mosi  <= 1'b0;
            rx_data   <= 8'b0;
        end else begin
            if (!running) begin
                done <= 0;
                spi_sck <= 0;
                if (start) begin
                    shift_reg <= tx_data;
                    spi_mosi <= tx_data[7];
                    bit_cnt <= 0;
                    clk_cnt <= 0;
                    running <= 1;
                end
            end else begin
                clk_cnt <= clk_cnt + 1'b1;

                if (clk_cnt == clk_div) begin
                    clk_cnt <= 0;
                    sck_internal <= !sck_internal;

                    if (!sck_internal) begin
                        shift_reg <= {shift_reg[6:0], spi_miso};
                        spi_sck <= 1;
                    end else begin
                        spi_sck <= 0;
                        if (bit_cnt == 7) begin
                            rx_data <= shift_reg;
                            done <= 1;
                            running <= 0;
                        end else begin
                            bit_cnt <= bit_cnt + 1'b1;
                            spi_mosi <= shift_reg[7];
                        end
                    end
                end
            end
        end
    end

endmodule
