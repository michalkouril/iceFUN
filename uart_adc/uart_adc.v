/*
 *  
 *  Copyright(C) 2022 Michal Kouril
 * 
 *  Permission to use, copy, modify, and/or distribute this software for any purpose with or
 *  without fee is hereby granted, provided that the above copyright notice and 
 *  this permission notice appear in all copies.
 * 
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO
 *  THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. 
 *  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL 
 *  DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN
 *  AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN 
 *  CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 * 
 */


// Use UART to query all 4 ADCs and show lower portions on the LED array
/* module */
module top (clk, led1, led2, led3, led4, led5, led6, led7, led8, lcol1, lcol2, lcol3, lcol4, led9, led10, uart_txd, uart_rxd );
    /* I/O */
    input clk;
    output led1;
    output led2;
    output led3;
    output led4;
    output led5;
    output led6;
    output led7;
    output led8;
    output lcol1;
    output lcol2;
    output lcol3;
    output lcol4;
    output led9;
    output led10;
    output uart_txd;
    input  uart_rxd;

    /* LED drivers - are inverted for display because leds are active low */

    /* UART TX AXI */
    wire [7:0] uart_tx_axis_tdata;
    wire uart_tx_axis_tvalid;
    wire uart_tx_axis_tready;

    /* UART RX AXI */
    wire [7:0] uart_rx_axis_tdata;
    wire uart_rx_axis_tvalid;
    wire uart_rx_axis_tready;

    assign uart_rx_axis_tready = 1; // auto accept

    /* Response Counter register */
    reg [1:0] resp_counter = 0;

    /* ADC selector 0..3 */
    reg [1:0] adc = 2'b00;

    /* FSM
       0 -- idle
       1 -- in-flight (waiting for ADC result) */
    reg [0:0] state = 1'b0;

    /* initial reset */
    reg       rst = 1;

    always @ (posedge clk) begin
    if(rst) begin
      state <= 1'b0;
      adc <= 2'b00; 
      {lcol4, lcol3, lcol2, lcol1} = 4'b0111; // init state
      {led8, led7, led6, led5, led4, led3, led2, led1} = 8'ha5; // init state -- pattern to show if ADC is not responding
      rst <= 0;
    end else begin
      uart_tx_axis_tvalid <= 0;

      if (uart_rx_axis_tvalid == 1) begin
	 if (resp_counter == 0) begin
	     // only show the lower byte
	     case (adc)
		2'b00 : {lcol4, lcol3, lcol2, lcol1} <= 4'b0111;
		2'b01 : {lcol4, lcol3, lcol2, lcol1} <= 4'b1011;
		2'b10 : {lcol4, lcol3, lcol2, lcol1} <= 4'b1101;
		2'b11 : {lcol4, lcol3, lcol2, lcol1} <= 4'b1110;
	     endcase
	     {led8, led7, led6, led5, led4, led3, led2, led1} <= uart_rx_axis_tdata[7:0] ^ 8'hff;
         end
	 resp_counter <= resp_counter + 1;
      end

      if (state == 1'b0) begin
	  // state is idle
	  if (uart_tx_axis_tready == 1) begin

	     case (adc)
		2'b00 : uart_tx_axis_tdata <= 8'hA1;
		2'b01 : uart_tx_axis_tdata <= 8'hA2;
		2'b10 : uart_tx_axis_tdata <= 8'hA3;
		2'b11 : uart_tx_axis_tdata <= 8'hA4;
	     endcase
             uart_tx_axis_tvalid <= 1; // send UART request

	     resp_counter <= 0;
	     state <= 1'b1; // in flight // wait for response
	  end
      end

      if (state == 1'b1 && resp_counter == 2) begin
	  // have two bytes in response -- move on to the next one
	  state <= 1'b0; // transition back to idle
	  adc <= adc + 1; // move on to the next ADC
      end
     end // non rst
    end // always @clk
  
uart
uart_inst (
    .clk(clk),
    .rst(rst),
    // AXI input
    .s_axis_tdata(uart_tx_axis_tdata),
    .s_axis_tvalid(uart_tx_axis_tvalid),
    .s_axis_tready(uart_tx_axis_tready),
    // AXI output
    .m_axis_tdata(uart_rx_axis_tdata),
    .m_axis_tvalid(uart_rx_axis_tvalid),
    .m_axis_tready(uart_rx_axis_tready),
    // uart
    .rxd(uart_rxd),
    .txd(uart_txd),
    // status
    .tx_busy(),
    .rx_busy(),
    .rx_overrun_error(),
    .rx_frame_error(),
    // configuration
    .prescale(12000000/(250000*8)) // old PIC firmware
    // .prescale(12000000/(115200*8))
    // .prescale(12000000/(1000000*8)) // new PIC firmware
);


endmodule
