// =============================================================
// UART Receiver (RX) - 동기 리셋, LSB-first, valid 레벨 유지형
// =============================================================
module uart_rx #(
    parameter p_CLK_FREQ  = 100_000_000,
    parameter p_BAUD_RATE = 9600
)(
    input  wire       clk_i,
    input  wire       rst_i,            // 동기 리셋 (active-high)
    input  wire       serial_data_i,
    input  wire       ready_i,          // TX ready (=IDLE=1)
    output reg  [7:0] parallel_data_o,  // 수신 데이터
    output reg        valid_o           // 데이터 유효 (레벨 유지)
);

    localparam ST_IDLE  = 3'b000;
    localparam ST_START = 3'b001;
    localparam ST_DATA  = 3'b010;
    localparam ST_STOP  = 3'b011;

    localparam integer c_OVERSAMPLE_RATE = 16;
    localparam integer c_CYCLES_PER_TICK = p_CLK_FREQ / (p_BAUD_RATE * c_OVERSAMPLE_RATE);
    localparam integer c_SAMPLE_MID      = c_OVERSAMPLE_RATE/2 - 1; // 7
    localparam integer c_SAMPLE_END      = c_OVERSAMPLE_RATE - 1;   // 15

    reg [2:0] current_state, next_state;
    reg [2:0] bit_cnt;
    reg [7:0] rx_shift;
    reg [9:0] tick_cnt;   // 100MHz/9600/16 = 651 (<1024)
    reg [3:0] sample_cnt;

    wire sample_tick = (tick_cnt == c_CYCLES_PER_TICK - 1);
    wire mid_tick    = sample_tick && (sample_cnt == c_SAMPLE_MID);
    wire counting_en = (current_state != ST_IDLE);

    // 비동기 입력 동기화(2FF)
    reg rx_sync1, rx_sync2;
    always @(posedge clk_i) begin
        if (rst_i) begin
            rx_sync1 <= 1'b1;
            rx_sync2 <= 1'b1;
        end else begin
            rx_sync1 <= serial_data_i;
            rx_sync2 <= rx_sync1;
        end
    end
    wire rx_d = rx_sync2;

    // 타이머 & FSM 순차
    always @(posedge clk_i) begin
        if (rst_i) begin
            current_state   <= ST_IDLE;
            bit_cnt         <= 3'd0;
            rx_shift        <= 8'd0;
            parallel_data_o <= 8'd0;
            valid_o         <= 1'b0;
            tick_cnt        <= 10'd0;
            sample_cnt      <= 4'd0;
        end else begin
            current_state <= next_state;

            // 오버샘플 타이머
            if (counting_en) begin
                if (sample_tick) begin
                    tick_cnt   <= 10'd0;
                    sample_cnt <= (sample_cnt == c_SAMPLE_END) ? 4'd0 : (sample_cnt + 1'b1);
                end else begin
                    tick_cnt   <= tick_cnt + 1'b1;
                end
            end else begin
                tick_cnt   <= 10'd0;
                sample_cnt <= 4'd0;
            end

            // ST_START -> ST_DATA 진입 시 비트카운터 초기화
            if (current_state == ST_START && mid_tick && (rx_d == 1'b0)) begin
                bit_cnt <= 3'd0;
            end

            // 데이터 시프트 (LSB-first)
            if (current_state == ST_DATA && mid_tick) begin
                rx_shift <= {rx_shift[6:0], rx_d};
                bit_cnt  <= bit_cnt + 1'b1;
            end

            // STOP에서 데이터 확정 및 valid set
            if (current_state == ST_STOP && mid_tick) begin
                if (rx_d == 1'b1) begin
                    parallel_data_o <= rx_shift;
                    valid_o         <= 1'b1;
                end
            end

            // 상위(TX)가 수락하면 valid clear
            if (valid_o && ready_i) begin
                valid_o <= 1'b0;
            end
        end
    end

    // FSM 조합
    always @(*) begin
        next_state = current_state;
        case (current_state)
            ST_IDLE : begin
                if (rx_d == 1'b0) next_state = ST_START;
            end
            ST_START: begin
                if (mid_tick) begin
                    next_state = (rx_d == 1'b0) ? ST_DATA : ST_IDLE;
                end
            end
            ST_DATA : begin
                if (mid_tick && (bit_cnt == 3'd7)) next_state = ST_STOP;
            end
            ST_STOP : begin
                if (mid_tick) next_state = ST_IDLE;
            end
            default: next_state = ST_IDLE;
        endcase
    end
endmodule

// =============================================================
// UART Transmitter (TX) - 동기 리셋, ready 레벨, valid 상승엣지 수락
// =============================================================
module uart_tx #(
    parameter integer CLK_FREQ  = 100_000_000,
    parameter integer BAUD_RATE = 9600
)(
    input  wire       clk_i,
    input  wire       rst_i,            // 동기 리셋 (active-high)
    input  wire [7:0] parallel_data_i,  // 보낼 데이터
    input  wire       valid_i,          // RX valid (레벨)
    output reg        serial_data_o,    // UART TX 라인
    output reg        ready_o           // IDLE=1, 전송 중=0
);
    // Baud divider
    localparam integer BAUD_CNT_MAX_PRE = (CLK_FREQ + (BAUD_RATE/2)) / BAUD_RATE;
    localparam integer BAUD_CNT_MAX     = (BAUD_CNT_MAX_PRE < 1) ? 1 : BAUD_CNT_MAX_PRE;
    localparam integer CNTW             = (BAUD_CNT_MAX <= 1) ? 1 : $clog2(BAUD_CNT_MAX);

    reg  [CNTW-1:0] baud_cnt;
    wire            baud_tick = (baud_cnt == BAUD_CNT_MAX - 1);

    // FSM
    localparam [1:0] IDLE        = 2'b00;
    localparam [1:0] START_STATE = 2'b01;
    localparam [1:0] DATA_STATE  = 2'b10;
    localparam [1:0] STOP_STATE  = 2'b11;

    reg [1:0] current_state;
    reg [9:0] tx_packet;  // [0]=start(0), [1..8]=data bits, [9]=stop(1)
    reg [3:0] bit_cnt;

    // valid 상승엣지 검출 → 중복 수락 방지
    reg prev_valid;
    always @(posedge clk_i) begin
        if (rst_i) prev_valid <= 1'b0;
        else       prev_valid <= valid_i;
    end
    wire accept = (ready_o && valid_i && !prev_valid);

    // FSM + baud 카운터
    always @(posedge clk_i) begin
        if (rst_i) begin
            baud_cnt      <= {CNTW{1'b0}};
            current_state <= IDLE;
            serial_data_o <= 1'b1;   // idle = 1
            ready_o       <= 1'b1;
            tx_packet     <= 10'h3FF;
            bit_cnt       <= 4'd0;
        end else begin
            // Baud 카운터
            if (current_state != IDLE) begin
                baud_cnt <= baud_tick ? {CNTW{1'b0}} : (baud_cnt + 1'b1);
            end else begin
                baud_cnt <= {CNTW{1'b0}};
            end

            // FSM
            case (current_state)
                IDLE: begin
                    serial_data_o <= 1'b1;
                    ready_o       <= 1'b1;
                    bit_cnt       <= 4'd0;

                    if (accept) begin
                        tx_packet     <= {1'b1, parallel_data_i, 1'b0};
                        ready_o       <= 1'b0;
                        serial_data_o <= 1'b0; // start bit
                        current_state <= START_STATE;
                    end
                end

                START_STATE: begin
                    if (baud_tick) begin
                        bit_cnt       <= 4'd0;
                        serial_data_o <= tx_packet[1]; // 첫 데이터 비트(LSB)
                        current_state <= DATA_STATE;
                    end
                end

                DATA_STATE: begin
                    if (baud_tick) begin
                        if (bit_cnt == 4'd7) begin
                            serial_data_o <= tx_packet[9]; // stop bit
                            current_state <= STOP_STATE;
                        end else begin
                            bit_cnt       <= bit_cnt + 1'b1;
                            serial_data_o <= tx_packet[1 + (bit_cnt + 1'b1)];
                        end
                    end
                end

                STOP_STATE: begin
                    if (baud_tick) begin
                        serial_data_o <= 1'b1;
                        ready_o       <= 1'b1;
                        current_state <= IDLE;
                    end
                end

                default: begin
                    current_state  <= IDLE;
                    serial_data_o  <= 1'b1;
                    ready_o        <= 1'b1;
                end
            endcase
        end
    end
endmodule

// =============================================================
// UART Echo-Back Top
// =============================================================
module uart_echoback #(
    parameter integer CLK_FREQ  = 100_000_000,
    parameter integer BAUD_RATE = 9600
)(
    input  wire i_clk,        // 100 MHz
    input  wire i_rst,        // 동기 리셋 (BTN0 등)
    input  wire i_rx_serial,  // Basys3 USB-UART RX (B18)
    output wire o_tx_serial   // Basys3 USB-UART TX (A18)
);
    wire [7:0] w_data;
    wire       w_valid;
    wire       w_ready;

    uart_rx #(
        .p_CLK_FREQ (CLK_FREQ),
        .p_BAUD_RATE(BAUD_RATE)
    ) U_RX (
        .clk_i          (i_clk),
        .rst_i          (i_rst),
        .serial_data_i  (i_rx_serial),
        .ready_i        (w_ready),
        .parallel_data_o(w_data),
        .valid_o        (w_valid)
    );

    uart_tx #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) U_TX (
        .clk_i           (i_clk),
        .rst_i           (i_rst),
        .parallel_data_i (w_data),
        .valid_i         (w_valid),
        .serial_data_o   (o_tx_serial),
        .ready_o         (w_ready)
    );
endmodule
