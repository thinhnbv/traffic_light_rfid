//
//
//

module uart_tx_core (
    input wire clk,
    input wire rst_n,
    input wire [7:0] tx_data,
    input wire tx_start, 
    output wire tx_serial,
    output wire tx_busy,
    output wire tx_done
);
// FSM State 
    parameter IDLE = 3'd0;
    parameter START = 3'd1;
    parameter DATA = 3'd2;
    parameter STOP = 3'd3;
    parameter DONE = 3'd4;

    // Baudrate 
    parameter BAUD_DIV = 5208;

    // Internal signals
    reg [2:0] state, next_state;
    reg [12:0] baud_counter;
    reg [2:0] bit_index;
    reg [7:0] tx_shift;
    reg tx_reg;
    wire bit_timer_done;

    assign bit_timer_done = (baud_counter == BAUD_DIV - 1);

    always @(posedge clk) begin
        if (!rst_n) begin
            baud_counter <= 0;
        end else if (state == IDLE) begin
            baud_counter <= 0;
        end else if (bit_timer_done) begin
            baud_counter <= 0;
        end else begin
            baud_counter <= baud_counter + 1;
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            bit_index <= 0;
            tx_shift <= 0;
        end else if (state == IDLE && tx_start) begin
            tx_shift <= tx_data;
            bit_index <= 0;
        end else if (state == DATA && bit_timer_done) begin
            tx_shift <= tx_shift >> 1;
            bit_index <= bit_index + 1;
        end
    end

    // Output logic
    always @(posedge clk) begin
        if (!rst_n) begin
            tx_reg <= 1'b1;
        end else begin
            case (state) 
                IDLE:       tx_reg <= 1'b1;
                START:      tx_reg <= 1'b0;
                DATA:       tx_reg <= tx_shift[0];
                STOP:       tx_reg <= 1'b1;
                DONE:       tx_reg <= 1'b1;
                default:    tx_reg <= 1'b1;
            endcase
        end
    end

    // FSM: State storage logic (Sequential)
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end
    end

    // FSM: Next state logic (Combinational)
    always @(*) begin
        case (state)
            IDLE: begin
                if (tx_start) 
                    next_state = START;
                else 
                    next_state = IDLE;
            end

            START: begin
                if (bit_timer_done) 
                    next_state = DATA;
                else 
                    next_state = START;
            end

            DATA: begin
                if (bit_timer_done && bit_index == 7)
                    next_state = STOP;
                else
                    next_state = DATA;
            end

            STOP: begin
                if (bit_timer_done)
                    next_state = DONE;
                else 
                    next_state = STOP;
            end

            DONE: begin
                next_state = IDLE;
            end

            default: next_state = IDLE;
        endcase
    end

    // Output
    assign tx_busy = (state != IDLE);
    assign tx_done = (state == DONE);
    assign tx_serial = tx_reg;

endmodule
    
    
