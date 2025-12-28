`timescale 1ns/1ps

module uart_tb;

    //====================================================
    // Clock & Reset
    //====================================================
    reg clk = 0;
    reg rst = 1;

    //====================================================
    // DUT Interface
    //====================================================
    reg  [7:0] din = 0;
    reg        wr_en = 0;
    reg        rdy_clr = 0;

    wire       tx;
    wire       tx_busy;
    wire       rdy;
    wire [7:0] dout;

    // Loopback connection
    wire loopback = tx;

    //====================================================
    // DUT
    //====================================================
    uart dut (
        .din(din),
        .wr_en(wr_en),
        .clk_50m(clk),
        .rst(rst),
        .tx(tx),
        .tx_busy(tx_busy),
        .rx(loopback),
        .rdy(rdy),
        .rdy_clr(rdy_clr),
        .dout(dout)
    );

    //====================================================
    // 50 MHz Clock
    //====================================================
    always #10 clk = ~clk;   // 50 MHz

    //====================================================
    // UART Send Task
    //====================================================
    task uart_send(input [7:0] data);
        begin
            // Wait for transmitter idle
            while (tx_busy)
                @(posedge clk);

            din   <= data;
            wr_en <= 1'b1;
            @(posedge clk);
            wr_en <= 1'b0;
        end
    endtask

    //====================================================
    // UART Receive & Check Task
    //====================================================
    task uart_expect(input [7:0] exp);
        integer timeout;
        begin
            timeout = 0;
            while (!rdy) begin
                @(posedge clk);
                timeout = timeout + 1;
                if (timeout > 2_000_000) begin
                    $display("❌ TIMEOUT waiting for RX");
                    $finish;
                end
            end

            if (dout !== exp) begin
                $display("❌ DATA MISMATCH: expected=%h got=%h", exp, dout);
                $finish;
            end else begin
                $display("✅ RX OK: %h", dout);
            end

            // Clear ready
            rdy_clr <= 1'b1;
            @(posedge clk);
            rdy_clr <= 1'b0;
        end
    endtask

    //====================================================
    // Test Sequence
    //====================================================
    integer i;
    reg [7:0] expected;

    initial begin
        $dumpfile("uart_tb.vcd");
        $dumpvars(0, uart_tb);

        // Apply reset pulse
        rst = 1;
        repeat (10) @(posedge clk);
        rst = 0;

        $display("\n=== UART TEST START ===");

        // -------------------------
        // Test 1: Single Byte
        // -------------------------
        expected = 8'h55;
        uart_send(expected);
        uart_expect(expected);

        // -------------------------
        // Test 2: Edge Values
        // -------------------------
        uart_send(8'h00);
        uart_expect(8'h00);

        uart_send(8'hFF);
        uart_expect(8'hFF);

        // -------------------------
        // Test 3: Incrementing Bytes
        // -------------------------
        for (i = 0; i < 16; i = i + 1) begin
            uart_send(i[7:0]);
            uart_expect(i[7:0]);
        end

        // -------------------------
        // Test 4: Random Stress Test
        // -------------------------
        for (i = 0; i < 20; i = i + 1) begin
            expected = $random;
            uart_send(expected);
            uart_expect(expected);
        end

        $display("\n🎉 ALL UART TESTS PASSED 🎉");
        $finish;
    end

endmodule
