`timescale 1ns / 1ps
// ======================================================================
// uart_led_sensor_top (Optimized)
// - Immediate LED ON/OFF when a full 3-char command is received
// - Accepts only ON1..ON8 and OF1..OF8 (others ignored)
// - No newline required
// - Robust against command overlap (buffer cleared after decode)
// ======================================================================
module uart_led_sensor_top(
    input  wire        i_Clock,    // 50 MHz clock
    input  wire        i_Rst,      // synchronous active-high reset
    input  wire        FPGA_RXD,   // UART RX (to FPGA)
    output wire        FPGA_TXD,   // UART TX (from FPGA)
    output reg  [7:0]  o_LEDs      // 8 LEDs (o_LEDs[0] -> LED1)
);

    // ---------------- PARAMETERS ----------------
    parameter integer CLOCK_RATE = 50_000_000;
    parameter integer BAUD_RATE  = 1_000_000; // fast for simulation
    localparam integer CLKS_PER_BIT = CLOCK_RATE / BAUD_RATE;

    // ---------------- TX (simple) ----------------
    reg [15:0] tx_cnt;
    reg [3:0]  tx_bit;
    reg [9:0]  tx_shift;
    reg        tx_line = 1'b1;
    reg        tx_busy = 1'b0;
    reg        tx_done = 1'b0;
    assign FPGA_TXD = tx_line;

    // ---------------- RX ----------------
    reg [15:0] rx_cnt;
    reg [2:0]  rx_bit;
    reg [7:0]  rx_byte;
    reg        rx_dv;
    reg [2:0]  rx_state;
    reg r1, r2;
    wire rx_sync;
    always @(posedge i_Clock) begin r1 <= FPGA_RXD; r2 <= r1; end
    assign rx_sync = r2;

    localparam RX_IDLE  = 3'd0,
               RX_START = 3'd1,
               RX_DATA  = 3'd2,
               RX_STOP  = 3'd3;

    always @(posedge i_Clock) begin
        if (i_Rst) begin
            rx_state <= RX_IDLE;
            rx_cnt   <= 0;
            rx_bit   <= 0;
            rx_byte  <= 8'd0;
            rx_dv    <= 1'b0;
        end else begin
            rx_dv <= 1'b0;
            case (rx_state)
                RX_IDLE: if (!rx_sync) begin rx_state <= RX_START; rx_cnt <= 0; end
                RX_START: begin
                    if (rx_cnt == (CLKS_PER_BIT/2)) begin
                        if (!rx_sync) begin
                            rx_state <= RX_DATA;
                            rx_bit <= 0;
                        end else begin
                            rx_state <= RX_IDLE;
                        end
                        rx_cnt <= 0;
                    end else rx_cnt <= rx_cnt + 1;
                end
                RX_DATA: begin
                    if (rx_cnt < CLKS_PER_BIT-1) rx_cnt <= rx_cnt + 1;
                    else begin
                        rx_cnt <= 0;
                        rx_byte[rx_bit] <= rx_sync;
                        if (rx_bit < 7) rx_bit <= rx_bit + 1;
                        else rx_state <= RX_STOP;
                    end
                end
                RX_STOP: begin
                    if (rx_cnt < CLKS_PER_BIT-1) rx_cnt <= rx_cnt + 1;
                    else begin
                        rx_cnt <= 0;
                        rx_state <= RX_IDLE;
                        rx_dv <= 1'b1; // one-cycle pulse: byte received
                    end
                end
            endcase
        end
    end

    // ---------------- Command buffer (sliding 3-byte window) ----------------
    // We always form next_cmd = {cmd_buf[15:0], rx_byte} on rx_dv and evaluate it.
    reg [23:0] cmd_buf; // holds last upto-3 bytes; cleared after decode

    // temporary wire to form the candidate 3-char command (uses current cmd_buf + new byte)
    wire [23:0] next_cmd = {cmd_buf[15:0], rx_byte};

    // ---------------- main command decode & LED control ----------------
    // We act immediately when rx_dv pulses and next_cmd matches ONx or OFx.
    always @(posedge i_Clock) begin
        if (i_Rst) begin
            o_LEDs  <= 8'b0;
            cmd_buf <= 24'b0;
            // TX initial state
            tx_line <= 1'b1;
            tx_busy <= 1'b0;
            tx_done <= 1'b0;
            tx_cnt  <= 0;
            tx_bit  <= 0;
            tx_shift<= 10'b1111111111;
        end else begin
            tx_done <= 1'b0;

            if (rx_dv) begin
                // Evaluate candidate command formed by existing buffer plus new byte
                // If it matches, act and clear buffer. Otherwise, store next_cmd and wait.
                // Match "ONx"
                if ((next_cmd[23:8] == {8'h4F, 8'h4E}) && (next_cmd[7:0] >= "1") && (next_cmd[7:0] <= "8")) begin
                    // turn ON the requested LED immediately
                    integer idx;
                    idx = next_cmd[7:0] - "1";
                    o_LEDs[idx] <= 1'b1;
                    // clear buffer for next command
                    cmd_buf <= 24'b0;
                end
                // Match "OFx"
                else if ((next_cmd[23:8] == {8'h4F, 8'h46}) && (next_cmd[7:0] >= "1") && (next_cmd[7:0] <= "8")) begin
                    integer idx;
                    idx = next_cmd[7:0] - "1";
                    o_LEDs[idx] <= 1'b0;
                    cmd_buf <= 24'b0;
                end
                // Optionally accept STATUS (starts with "STA") -> begin TX later (simple example)
                else if (next_cmd[23:8] == {8'h53,8'h54}) begin
                    // If "STA" is being typed, we won't act until more bytes (not used here)
                    // Save next_cmd to buffer so future bytes can complete STATUS sequence
                    cmd_buf <= next_cmd;
                end
                else begin
                    // Not recognised yet - keep sliding window (store next_cmd), this allows overlapping commands
                    cmd_buf <= next_cmd;
                    // To prevent cmd_buf from growing stale with non-ASCII, we could add extra checks:
                    // If next_cmd contains non-printable bytes, we could clear — for simplicity we allow sliding.
                end
            end
            // (TX logic can be added/kept here if STATUS response is needed; omitted for clarity)
        end
    end

    // (Optional) Simple TX engine placeholder - kept minimal so design compiles
    // Here we don't automatically send STATUS bytes; TX logic can be added as needed.

endmodule
