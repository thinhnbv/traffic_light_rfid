module tag_priority_lut (
    input wire [31:0] uid,
    output reg [2:0] priority_class
);

    // UID mapping from actual RFID cards (uid.txt)
    // Chỉ một vài thẻ đặc biệt được đăng ký ưu tiên.
    // Tất cả thẻ còn lại → class 0 (xe thường, không ưu tiên).

    always @(*) begin
        case (uid)
            // Tier 5 — Xe cứu hỏa
            32'h69C3D106: priority_class = 3'd5;  // Card 1
            32'h7B8ECE06: priority_class = 3'd5;  // Card 2

            // Tier 4 — Quân sự / Công an
            32'h9E48CE06: priority_class = 3'd4;  // Card 4
            32'h01A5CB06: priority_class = 3'd4;  // Card 5

            // Tier 3 — Xe cứu thương
            32'h1E2CD006: priority_class = 3'd3;  // Card 7
            32'h691DD506: priority_class = 3'd3;  // Card 8

            // Tier 2 — Xe hộ đê / thiên tai
            32'h21F44D06: priority_class = 3'd2;  // Card 10
            32'h8E831006: priority_class = 3'd2;  // Card 11

            // Tier 1 — Xe tang
            32'h92D94D06: priority_class = 3'd1;  // Card 13
            32'h8ABA4C06: priority_class = 3'd1;  // Card 14

            default:      priority_class = 3'd0;  // Xe thường (6 thẻ còn lại + mọi thẻ khác)
        endcase
    end

endmodule