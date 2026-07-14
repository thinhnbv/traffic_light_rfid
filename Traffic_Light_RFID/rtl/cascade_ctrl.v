// =============================================================================
// Module     : cascade_ctrl
// Project    : TLC-RFID v4.4
// Description: Green Wave coordinator. Receives local priority events and
//              cascade packets from LEFT/RIGHT neighbors. Forwards packets
//              with ETA increment. Generates pre-clear request to FSM.
// =============================================================================

module cascade_ctrl (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [1:0] node_id,            // DIP switch: this node's ID (0-3)

    // Local signals from traffic_fsm / priority_arbiter / direction_detector
    input  wire       prio_active,
    input  wire [2:0] winner_class,
    input  wire [1:0] dir_out,
    input  wire       dir_valid,
    input  wire       dir_confidence,

    // From LEFT cascade_rx
    input  wire       left_pkt_valid,
    input  wire       left_pkt_flag_dir,
    input  wire       left_pkt_flag_clear,
    input  wire [2:0] left_pkt_class,
    input  wire [7:0] left_pkt_eta,

    // From RIGHT cascade_rx
    input  wire       right_pkt_valid,
    input  wire       right_pkt_flag_dir,
    input  wire       right_pkt_flag_clear,
    input  wire [2:0] right_pkt_class,
    input  wire [7:0] right_pkt_eta,

    // To LEFT cascade_tx
    output reg        left_tx_send,
    output reg        left_tx_valid,
    output reg        left_tx_dir,
    output reg        left_tx_clear,
    output reg  [2:0] left_tx_class,
    output reg  [7:0] left_tx_eta,

    // To RIGHT cascade_tx
    output reg        right_tx_send,
    output reg        right_tx_valid,
    output reg        right_tx_dir,
    output reg        right_tx_clear,
    output reg  [2:0] right_tx_class,
    output reg  [7:0] right_tx_eta,

    // To traffic_fsm
    output reg        preclear_req
);

    // Inter-node delay added to ETA when forwarding (seconds)
    parameter [7:0] INTER_NODE_DELAY = 8'd5;

    // Direction encoding
    localparam DIR_NORTH = 2'b00;
    localparam DIR_SOUTH = 2'b01;
    localparam DIR_EAST  = 2'b10;
    localparam DIR_WEST  = 2'b11;

    // ETA saturation helper
    function [7:0] saturate_add;
        input [7:0] a, b;
        reg [8:0] sum;
        begin
            sum = {1'b0, a} + {1'b0, b};
            saturate_add = (sum > 9'd255) ? 8'd255 : sum[7:0];
        end
    endfunction

    // Track previous prio_active for edge detection
    reg prio_active_prev;
    wire prio_falling = prio_active_prev & ~prio_active;  // Vehicle has passed

    always @(posedge clk) begin
        if (!rst_n)
            prio_active_prev <= 1'b0;
        else
            prio_active_prev <= prio_active;
    end

    // =========================================================================
    // Main Logic
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            left_tx_send   <= 1'b0;
            left_tx_valid  <= 1'b0;
            left_tx_dir    <= 1'b0;
            left_tx_clear  <= 1'b0;
            left_tx_class  <= 3'b000;
            left_tx_eta    <= 8'd0;
            right_tx_send  <= 1'b0;
            right_tx_valid <= 1'b0;
            right_tx_dir   <= 1'b0;
            right_tx_clear <= 1'b0;
            right_tx_class <= 3'b000;
            right_tx_eta   <= 8'd0;
            preclear_req   <= 1'b0;
        end else begin
            // Default: clear strobes
            left_tx_send  <= 1'b0;
            right_tx_send <= 1'b0;
            preclear_req  <= 1'b0;

            // ─── LOCAL EVENT: Vehicle detected at this node ───
            if (dir_valid && prio_active) begin
                case (dir_out)
                    DIR_EAST: begin
                        // Vehicle heading EAST → notify RIGHT neighbor
                        right_tx_send  <= 1'b1;
                        right_tx_valid <= 1'b1;
                        right_tx_dir   <= 1'b1;  // EW
                        right_tx_clear <= 1'b0;
                        right_tx_class <= winner_class;
                        right_tx_eta   <= INTER_NODE_DELAY;
                    end
                    DIR_WEST: begin
                        // Vehicle heading WEST → notify LEFT neighbor
                        left_tx_send  <= 1'b1;
                        left_tx_valid <= 1'b1;
                        left_tx_dir   <= 1'b1;  // EW
                        left_tx_clear <= 1'b0;
                        left_tx_class <= winner_class;
                        left_tx_eta   <= INTER_NODE_DELAY;
                    end
                    default: begin
                        // N/S direction → broadcast to both neighbors
                        right_tx_send  <= 1'b1;
                        right_tx_valid <= 1'b1;
                        right_tx_dir   <= 1'b0;  // NS
                        right_tx_clear <= 1'b0;
                        right_tx_class <= winner_class;
                        right_tx_eta   <= INTER_NODE_DELAY;

                        left_tx_send  <= 1'b1;
                        left_tx_valid <= 1'b1;
                        left_tx_dir   <= 1'b0;
                        left_tx_clear <= 1'b0;
                        left_tx_class <= winner_class;
                        left_tx_eta   <= INTER_NODE_DELAY;
                    end
                endcase
            end

            // ─── LOCAL EVENT: Vehicle has passed (clear) ───
            if (prio_falling) begin
                // Notify both neighbors that vehicle cleared this node
                right_tx_send  <= 1'b1;
                right_tx_valid <= 1'b1;
                right_tx_dir   <= 1'b0;
                right_tx_clear <= 1'b1;
                right_tx_class <= 3'b000;
                right_tx_eta   <= 8'd0;

                left_tx_send  <= 1'b1;
                left_tx_valid <= 1'b1;
                left_tx_dir   <= 1'b0;
                left_tx_clear <= 1'b1;
                left_tx_class <= 3'b000;
                left_tx_eta   <= 8'd0;
            end

            // ─── REMOTE EVENT: Packet from LEFT neighbor ───
            if (left_pkt_valid) begin
                if (left_pkt_flag_clear) begin
                    // Vehicle cleared upstream — no action needed
                    // (FSM will naturally recover)
                end else begin
                    // Vehicle approaching from left — pre-clear!
                    preclear_req <= 1'b1;
                    // Forward to RIGHT with incremented ETA
                    right_tx_send  <= 1'b1;
                    right_tx_valid <= 1'b1;
                    right_tx_dir   <= left_pkt_flag_dir;
                    right_tx_clear <= 1'b0;
                    right_tx_class <= left_pkt_class;
                    right_tx_eta   <= saturate_add(left_pkt_eta, INTER_NODE_DELAY);
                end
            end

            // ─── REMOTE EVENT: Packet from RIGHT neighbor ───
            if (right_pkt_valid) begin
                if (right_pkt_flag_clear) begin
                    // Vehicle cleared downstream — no action needed
                end else begin
                    preclear_req <= 1'b1;
                    // Forward to LEFT with incremented ETA
                    left_tx_send  <= 1'b1;
                    left_tx_valid <= 1'b1;
                    left_tx_dir   <= right_pkt_flag_dir;
                    left_tx_clear <= 1'b0;
                    left_tx_class <= right_pkt_class;
                    left_tx_eta   <= saturate_add(right_pkt_eta, INTER_NODE_DELAY);
                end
            end
        end
    end

endmodule
