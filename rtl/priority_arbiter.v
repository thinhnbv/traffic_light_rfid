// =============================================================================
// Module     : priority_arbiter
// Project    : TLC-RFID v4.4
// Description: Combinational priority resolver. Merges 4 directions into
//              NS/EW axes, compares priority classes, outputs grant signals.
//              Tie-break uses current FSM state.
// =============================================================================

module priority_arbiter (
    input  wire [3:0] tag_detected,   // [3]=N [2]=S [1]=E [0]=W
    input  wire [2:0] class_n,
    input  wire [2:0] class_s,
    input  wire [2:0] class_e,
    input  wire [2:0] class_w,
    input  wire [3:0] cur_state,      // FSM state for tiebreak
    output reg        ev_detect_ns,
    output reg        ev_detect_ew,
    output reg  [2:0] winner_class
);

    // Merge into NS and EW — take max class per axis
    wire [2:0] class_ns = (class_n > class_s) ? class_n : class_s;
    wire [2:0] class_ew = (class_e > class_w) ? class_e : class_w;

    // Check if each axis actually has a priority vehicle
    wire ns_has_prio = (tag_detected[3] && class_n != 3'b000) ||
                       (tag_detected[2] && class_s != 3'b000);
    wire ew_has_prio = (tag_detected[1] && class_e != 3'b000) ||
                       (tag_detected[0] && class_w != 3'b000);

    // State encodings for tiebreak (from traffic_fsm)
    localparam S_NS_GREEN = 4'd0;
    localparam S_EW_GREEN = 4'd3;

    always @(*) begin
        ev_detect_ns = 1'b0;
        ev_detect_ew = 1'b0;
        winner_class = 3'b000;

        if (ns_has_prio || ew_has_prio) begin
            if (class_ns > class_ew) begin
                ev_detect_ns = 1'b1;
                winner_class = class_ns;
            end
            else if (class_ew > class_ns) begin
                ev_detect_ew = 1'b1;
                winner_class = class_ew;
            end
            else begin
                // Same class — tiebreak by current FSM state
                winner_class = class_ns;
                if (cur_state == S_NS_GREEN)
                    ev_detect_ns = 1'b1;
                else if (cur_state == S_EW_GREEN)
                    ev_detect_ew = 1'b1;
                else
                    ev_detect_ns = 1'b1;  // default: NS wins
            end
        end
    end

endmodule
