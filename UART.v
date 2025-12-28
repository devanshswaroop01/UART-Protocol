//===========================================================
// 50MHz UART with 115200 baud, 8N1, reset-safe, 16x RX oversampling
//===========================================================

`timescale 1ns / 1ps

//===========================================================
// Baud Rate Generator
// Generates RX/TX clock enable signals
//===========================================================
module baud_rate_gen(
    input  wire clk_50m,
    input  wire rst,
    output wire rxclk_en,
    output wire txclk_en
);

    parameter RX_ACC_MAX = 50000000 / (115200 * 16);
    parameter TX_ACC_MAX = 50000000 / 115200;
    parameter RX_ACC_WIDTH = $clog2(RX_ACC_MAX);
    parameter TX_ACC_WIDTH = $clog2(TX_ACC_MAX);

    reg [RX_ACC_WIDTH-1:0] rx_acc;
    reg [TX_ACC_WIDTH-1:0] tx_acc;

    assign rxclk_en = (rx_acc == 0);
    assign txclk_en = (tx_acc == 0);

    always @(posedge clk_50m) begin
        if (rst)
            rx_acc <= 0;
        else if (rx_acc == RX_ACC_MAX-1)
            rx_acc <= 0;
        else
            rx_acc <= rx_acc + 1'b1;
    end

    always @(posedge clk_50m) begin
        if (rst)
            tx_acc <= 0;
        else if (tx_acc == TX_ACC_MAX-1)
            tx_acc <= 0;
        else
            tx_acc <= tx_acc + 1'b1;
    end

endmodule

//===========================================================
// Transmitter Module
// 8N1, LSB first, reset safe
//===========================================================
module transmitter(
    input  wire [7:0] din,
    input  wire       wr_en,
    input  wire       clk_50m,
    input  wire       clken,
    input  wire       rst,
    output reg        tx,
    output wire       tx_busy
);

    localparam STATE_IDLE  = 2'b00;
    localparam STATE_START = 2'b01;
    localparam STATE_DATA  = 2'b10;
    localparam STATE_STOP  = 2'b11;

    reg [1:0] state_reg;
    reg [2:0] bitpos;
    reg [7:0] data_reg;

    assign tx_busy = (state_reg != STATE_IDLE);

    always @(posedge clk_50m) begin
        if (rst) begin
            state_reg <= STATE_IDLE;
            tx        <= 1'b1;
            bitpos    <= 0;
            data_reg  <= 0;
        end else begin
            case (state_reg)
                STATE_IDLE: begin
                    tx <= 1'b1;
                    if (wr_en) begin
                        data_reg  <= din;
                        bitpos    <= 3'd0;
                        state_reg <= STATE_START;
                    end
                end
                STATE_START: begin
                    if (clken) begin
                        tx <= 1'b0;
                        state_reg <= STATE_DATA;
                    end
                end
                STATE_DATA: begin
                    if (clken) begin
                        tx <= data_reg[bitpos];
                        if (bitpos == 3'd7)
                            state_reg <= STATE_STOP;
                        else
                            bitpos <= bitpos + 1'b1;
                    end
                end
                STATE_STOP: begin
                    if (clken) begin
                        tx <= 1'b1;
                        state_reg <= STATE_IDLE;
                    end
                end
                default: begin
                    state_reg <= STATE_IDLE;
                    tx        <= 1'b1;
                end
            endcase
        end
    end

endmodule

//===========================================================
// Receiver Module
// 8N1, LSB first, 16x oversampling, reset safe, minor baud mismatch tolerance
//===========================================================
module receiver(
    input  wire       rx,
    input  wire       clk_50m,
    input  wire       clken,
    input  wire       rst,
    input  wire       rdy_clr,
    output reg [7:0]  data,
    output reg        rdy
);

    localparam RX_STATE_START = 2'b00;
    localparam RX_STATE_DATA  = 2'b01;
    localparam RX_STATE_STOP  = 2'b10;

    reg [1:0] state;
    reg [3:0] sample;
    reg [3:0] bitpos;
    reg [7:0] scratch;
    reg rx_d;

    always @(posedge clk_50m) begin
        if (rst) begin
            state   <= RX_STATE_START;
            sample  <= 0;
            bitpos  <= 0;
            scratch <= 0;
            rdy     <= 0;
            data    <= 0;
            rx_d    <= 1'b1;
        end else begin
            rx_d <= rx;

            if (rdy_clr)
                rdy <= 0;

            if (clken) begin
                case (state)
                    RX_STATE_START: begin
                        // Recenter sampling on start bit
                        if (rx_d && !rx)
                            sample <= 0;
                        else if (!rx || sample != 0)
                            sample <= sample + 1'b1;

                        if (sample == 4'd15) begin
                            state   <= RX_STATE_DATA;
                            bitpos  <= 0;
                            sample  <= 0;
                            scratch <= 0;
                        end
                    end
                    RX_STATE_DATA: begin
                        sample <= sample + 1'b1;
                        if (sample == 4'd8) begin
                            scratch[bitpos[2:0]] <= rx;
                            bitpos <= bitpos + 1'b1;
                        end
                        if (bitpos == 8 && sample == 4'd15)
                            state <= RX_STATE_STOP;
                    end
                    RX_STATE_STOP: begin
                        if (sample == 4'd15 || (sample >= 4'd8 && !rx)) begin
                            state <= RX_STATE_START;
                            data  <= scratch;
                            rdy   <= 1'b1;
                            sample <= 0;
                        end else begin
                            sample <= sample + 1'b1;
                        end
                    end
                    default: state <= RX_STATE_START;
                endcase
            end
        end
    end

endmodule

//===========================================================
// Top-Level UART Module
//===========================================================
module uart(
    input  wire [7:0] din,
    input  wire       wr_en,
    input  wire       clk_50m,
    input  wire       rst,
    input  wire       rx,
    input  wire       rdy_clr,
    output wire       tx,
    output wire       tx_busy,
    output wire [7:0] dout,
    output wire       rdy
);

    wire rxclk_en, txclk_en;

    // Baud generator
    baud_rate_gen baudgen(
        .clk_50m(clk_50m),
        .rst(rst),
        .rxclk_en(rxclk_en),
        .txclk_en(txclk_en)
    );

    // TX
    transmitter uart_tx(
        .din(din),
        .wr_en(wr_en),
        .clk_50m(clk_50m),
        .clken(txclk_en),
        .rst(rst),
        .tx(tx),
        .tx_busy(tx_busy)
    );

    // RX
    receiver uart_rx(
        .rx(rx),
        .clk_50m(clk_50m),
        .clken(rxclk_en),
        .rst(rst),
        .rdy_clr(rdy_clr),
        .data(dout),
        .rdy(rdy)
    );

endmodule
