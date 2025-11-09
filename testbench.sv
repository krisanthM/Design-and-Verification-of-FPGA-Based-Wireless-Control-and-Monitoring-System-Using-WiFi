`timescale 1ns / 1ps
// ======================================================================
// tb_uart_led_sensor - Testbench for optimized design
// - Sends ON/OFF commands, including invalid ones to verify ignore behavior
// - Dumps Wave.vcd for GTKWave/EPWave
// ======================================================================
module tb_uart_led_sensor;

    // DUT signals
    logic i_Clock;
    logic i_Rst;
    logic FPGA_RXD;
    wire  FPGA_TXD;
    wire [7:0] o_LEDs;

    // Instantiate DUT
    uart_led_sensor_top dut (
        .i_Clock(i_Clock),
        .i_Rst(i_Rst),
        .FPGA_RXD(FPGA_RXD),
        .FPGA_TXD(FPGA_TXD),
        .o_LEDs(o_LEDs)
    );

    // Clock: 50 MHz
    localparam real CLK_PERIOD_NS = 20.0;
    initial begin
        i_Clock = 1'b0;
        forever #(CLK_PERIOD_NS/2.0) i_Clock = ~i_Clock;
    end

    // UART timing: must match design BAUD_RATE = 1_000_000
    localparam int BAUD_RATE = 1_000_000;
    real BIT_PERIOD_NS = 1_000_000_000.0 / BAUD_RATE; // ns per bit

    // Send a single UART byte (8N1)
    task automatic send_byte(input [7:0] b);
        int i;
        begin
            // start bit
            FPGA_RXD <= 1'b0; #(BIT_PERIOD_NS);
            // data LSB first
            for (i = 0; i < 8; i = i + 1) begin
                FPGA_RXD <= b[i]; #(BIT_PERIOD_NS);
            end
            // stop bit
            FPGA_RXD <= 1'b1; #(BIT_PERIOD_NS);
        end
    endtask

    // Send ASCII command (no newline required)
    task automatic send_command(input string s);
        int i;
        begin
            for (i = 0; i < s.len(); i = i + 1) begin
                send_byte(s[i]);
                // small gap between characters (not required but keeps things clean)
                #(BIT_PERIOD_NS * 2);
            end
            // brief gap after command so decode happens in waveform nicely
            #(BIT_PERIOD_NS * 8);
        end
    endtask

    // Build test sequence
    initial begin
        // waveform dump
        $dumpfile("Wave.vcd");
        $dumpvars(0, tb_uart_led_sensor);

        // init
        FPGA_RXD = 1'b1; // idle line high
        i_Rst = 1'b1;
        #200;
        i_Rst = 1'b0;
        #2000;

        $display("--- TEST START ---");

        // Valid: ON1 -> LED0
        send_command("ON1");
        #1000;
        // Valid: ON2 -> LED1
        send_command("ON2");
        #1000;
        // Valid: ON8 -> LED7
        send_command("ON8");
        #1000;
        // Invalid: ON9 -> should be ignored (no LED change)
        send_command("ON9");
        #1000;
        // Invalid random -> ignored
        send_command("ABC");
        #1000;
        // Turn OFF a LED: OF2 -> LED1 clear
        send_command("OF2");
        #1000;
        // Turn OFF LED8: OF8
        send_command("OF8");
        #1000;
        // Turn ON LED3
        send_command("ON3");
        #1000;

        $display("--- CHECK LED VALUES IN LOG / WAVEFORM ---");
        #5000;

        $display("--- TEST COMPLETE ---");
        #100;
        $finish;
    end

    // Monitor concise output
    initial begin
        $monitor("[%0t ns] LEDs=%b RXD=%b TXD=%b", $time, o_LEDs, FPGA_RXD, FPGA_TXD);
    end

endmodule
