module zrb_sync_fifo
/*
zrb_sync_fifo #(2,8) instance_name (
    RESET,
    WR_CLK,
    WR_EN,
    WR_DATA[7:0],
    RD_EN,
    RD_DATA[7:0],
    FIFO_FULL,
    FIFO_EMPTY
    );
*/
    #(parameter ADDR_WIDTH = 2, DATA_WIDTH = 8)
    (
    input   wire                            reset,

    input   wire                            clk,
    input   wire                            wr_en,
    input   wire    [ (DATA_WIDTH-1) :  0 ] data_in,

    input   wire                            rd_en,
    output  wire    [ (DATA_WIDTH-1) :  0 ] data_out,

    output  wire                            fifo_full,
    output  wire                            fifo_empty
    );
localparam DEPTH = 1 << ADDR_WIDTH;
reg     [ (ADDR_WIDTH) :  0 ] wr_ptr = {(ADDR_WIDTH+1){1'b0}};
reg     [ (ADDR_WIDTH) :  0 ] rd_ptr = {(ADDR_WIDTH+1){1'b0}};
wire    [ (ADDR_WIDTH-1) :  0 ] wr_loc = wr_ptr[ (ADDR_WIDTH-1) :  0 ];
wire    [ (ADDR_WIDTH-1) :  0 ] rd_loc = rd_ptr[ (ADDR_WIDTH-1) :  0 ];

reg     [ (DATA_WIDTH-1) :  0 ] mem [ (DEPTH-1) :  0 ];

reg full = 1'b0;
reg empty = 1'b0;
assign data_out = mem[rd_loc];
assign fifo_empty = empty;
assign fifo_full = full;

always@(wr_ptr or rd_ptr)
begin
    full <= 1'b0;
    empty <= 1'b0;
    if(wr_ptr[ (ADDR_WIDTH-1) :  0 ] == rd_ptr[ (ADDR_WIDTH-1) :  0 ])
        if(rd_ptr[ADDR_WIDTH] == wr_ptr[ADDR_WIDTH])
            empty <= 1'b1;
        else
            full <= 1'b1;
end

always@(posedge clk or posedge reset)
if(reset)
begin
    wr_ptr <= {(ADDR_WIDTH+1){1'b0}};
    rd_ptr <= {(ADDR_WIDTH+1){1'b0}};
end
else
begin
    if(wr_en & !fifo_full)
    begin
        mem[wr_loc] <= data_in;
        wr_ptr <= wr_ptr + 1'b1;
    end
    
    if(rd_en & !fifo_empty)
        rd_ptr <= rd_ptr + 1'b1;
end
endmodule


module zrb_baud_generator
/*
zrb_baud_generator #(INPUT_CLK,BAUD) instance_name(input_clk, baud_clk, baud_clk_8);
*/
    #(parameter INPUT_CLK = 50000000, parameter BAUD = 9600)
    (
    input   wire                clk,
    output  wire                baud_clk_tx_en,
    output  wire                baud_clk_rx_en
    );
localparam BAUD_RX = 8*BAUD;
reg     [ 28 :  0 ] r_tx = 29'b0;
reg     [ 28 :  0 ] r_rx = 29'b0;
wire    [ 28 :  0 ] inc_tx = r_tx[28] ? (BAUD) : (BAUD-INPUT_CLK);
wire    [ 28 :  0 ] inc_rx = r_rx[28] ? (BAUD_RX) : (BAUD_RX-INPUT_CLK);
wire    [ 28 :  0 ] tx_tic = r_tx + inc_tx;
wire    [ 28 :  0 ] rx_tic = r_rx + inc_rx;
always@(posedge clk)
begin
    r_tx <= tx_tic;
    r_rx <= rx_tic;
end
assign baud_clk_tx_en = ~r_tx[28];
assign baud_clk_rx_en = ~r_rx[28];
endmodule


module zrb_uart_tx
/*
zrb_uart_tx #(8,"NO",1) instance_name(
    CLK,
    CLK_EN, //BAUD RATE
    RESET,
    NEW_DATA,
    INPUT_DATA[7:0],
    OUTPUT_TX,
    OUTPUT_BUSY,
	READ
    );
*/
    #(parameter NUM_BITS = 4'd8, parameter PARITY = "NO", parameter STOP_BIT = 4'd1)
    (
    input   wire                clk,
    input   wire                clk_en,
    input   wire                reset,
    input   wire                new_data,
    input   wire    [  7 :  0 ] data,

    output  wire                tx,
    output  wire                busy,
	output	wire				read
    );
localparam START_BIT = 4'd1;
localparam WIDTH =  PARITY == "NO"   ?  NUM_BITS + START_BIT + STOP_BIT :
                    PARITY == "EVEN" ?  NUM_BITS + START_BIT + STOP_BIT + 1 :
                    PARITY == "ODD"  ?  NUM_BITS + START_BIT + STOP_BIT + 1 : 1;

reg     [  8 :  0 ] r_data = 9'b0;
reg     [  3 :  0 ] r_cnt = 4'b0;
reg                 r_tx = 1'b1;
reg		    r_rd = 1'b0;

wire                sending = |r_cnt;
assign              busy = sending;
assign              tx = r_tx;
assign				read = r_rd;

always@(posedge clk)
if(reset)
begin
    r_tx <= 1'b1;
    r_cnt <= 4'b0;
    r_data <= 9'b0;
	r_rd <= 1'b0;
end
else
begin
	r_rd <= 1'b0;
    if(new_data & ~busy)
    begin
		r_rd <= 1'b1;
        r_data <= {data, 1'b0};
        r_cnt <= WIDTH;
    end
    
    if(sending & clk_en)
    begin
        {r_data, r_tx} <= {1'b1, r_data};
        r_cnt <= r_cnt - 1'b1;
    end
end
endmodule


module zrb_uart_rx
/*
zrb_uart_rx #(8,"NO",1) instance_name(
    INPUT_CLK, //HIGH FREQ
    CLK_EN,
    RESET
    INPUT_RX,
    OUTPUT_DATA[7:0],
    WRITE_EN,
    OUTPUT_BUSY,
    );
*/
    #(parameter [3:0] NUM_BITS = 8, parameter PARITY = "NO", parameter [3:0]STOP_BIT = 1)
    (
    input   wire                clk,
    input   wire                clk_en,
    input   wire                reset,
    
    input   wire                rx,

    output  wire    [  7 :  0 ] data_out,
    output  wire                write_en,
    output  wire                busy
    );

localparam [3:0] START_BIT = 1;
localparam [3:0] WIDTH = PARITY == "NO"   ?     NUM_BITS + START_BIT + STOP_BIT : 
                         PARITY == "EVEN" ?     NUM_BITS + START_BIT + STOP_BIT + 1 :
                         PARITY == "ODD"  ?     NUM_BITS + START_BIT + STOP_BIT + 1 : 1;
reg                 start_sync = 1'b0;
reg                 start_en = 1'b0;
wire                start = ~start_sync & start_en;
reg     [  9 :  0 ] r_data = 10'b0;
reg     [  3 :  0 ] r_cnt = 4'b0;
reg     [  2 :  0 ] clk_en_cnt = 3'b0;
wire                receiving = |r_cnt;
assign              busy = receiving;
assign              write_en = clk_en & r_cnt == 1 & clk_en_cnt == 3'd3;
assign              data_out = r_data[(WIDTH-2)-:8];

always@(posedge clk)
if(reset)
begin
    start_sync <= 1'b0;
    start_en <= 1'b0;
    r_data <= 10'b0;
    r_cnt <= 4'b0;
    clk_en_cnt <= 3'b0;
end
else
begin
    start_sync <= rx;
    start_en <= start_sync;
    
    if(start & ~receiving)
    begin
        r_cnt <= WIDTH;
        clk_en_cnt <= 3'b0;
    end
    if(receiving & clk_en)
    begin   
        clk_en_cnt <= clk_en_cnt + 1'b1;
        if(clk_en_cnt == 3'd3)
        begin
            //r_data <= {r_data[(WIDTH-2):0], rx};
            r_data <= {rx, r_data[(WIDTH-2):1]};
            r_cnt <= r_cnt - 1'b1;
        end
    end
end
endmodule

module zrb_uart_top
/*
zrb_uart_top #(50000000, 115200, 8, "NO", 1)
*/
    #(parameter INPUT_CLK = 50000000, parameter BAUD = 115200, parameter NUM_BITS = 8, parameter PARITY = "NO", parameter STOP_BIT = 1)
    (
    input   wire                clk,
    input   wire                wr,
    input   wire                rd,
    input   wire                uart_in,
    input   wire    [  7 :  0 ] data_in,
    output  wire                uart_out,
    output  wire    [  7 :  0 ] data_out,
    output   wire                rx_isr,
    output   wire                tx_en    
    );

wire baud;
wire baudx8;
zrb_baud_generator #(INPUT_CLK,BAUD) u0(clk, baud, baudx8);

wire [7:0] rx_data;
wire rx_write;
wire rx_busy;
wire rx_full;
wire rx_empty;
zrb_uart_rx #(NUM_BITS, PARITY, STOP_BIT) u1(
    clk,                    //input clock
    baudx8,                 //clock_enable
    1'b0,                   //reset
    uart_in,                //data_in
    rx_data,                //data_out[7:0]
    rx_write,               //write_enable_out
    rx_busy                 //rx_busy_out
    );
zrb_sync_fifo #(2, NUM_BITS) u2(
    1'b0,                   //reset
    clk,                    //write_clock
    rx_write,               //write_enable
    rx_data,                //data_in
    rd,                    //read_enable
    data_out,               //data_out
    rx_full,                //fifo_full_out
    rx_empty                //fifo_empty_out
    );
assign rx_isr = ~rx_empty;
wire [7:0] tx_data;
wire tx_write;
wire tx_read;
wire tx_busy;
wire tx_full;
wire tx_empty;
zrb_sync_fifo #(2,NUM_BITS) u3(
    1'b0,                   //reset
    clk,                    //clock
    wr,                     //write_enable
    data_in,                //data_in
    tx_read,                //read_enable
    tx_data,                //data_out
    tx_full,                //fifo_full_out
    tx_empty                //fifo_empty_out
    );
assign tx_en = ~tx_full;
zrb_uart_tx #(NUM_BITS, PARITY, STOP_BIT) u4(
    clk,                    //input clock
    baud,                   //BAUD RATE
    1'b0,                   //reset
    ~tx_empty,              //write_in
    tx_data,                //input_data[7:0]
    uart_out,               //data_out
    tx_busy,                //tx_busy_out
	tx_read                 //read_enable
    );
endmodule
