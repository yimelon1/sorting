// ============================================================================
// Ver      : 2.0
// Designer : Yi_Yuan Chen
// Create   : 2021.9.29
// Func     : get axi dma signal 
// description  : connect fifo and convertor.
//		new port of current state
// ============================================================================

module convert_top #(
	    parameter TBITS = 64 ,		//parameter for if of stream
        parameter TBYTE = 8 		//parameter for if of stream
)
(
	// from dma signal
		input  wire             S_AXIS_MM2S_TVALID,
        output wire             S_AXIS_MM2S_TREADY,
        input  wire [TBITS-1:0] S_AXIS_MM2S_TDATA,
        input  wire [TBYTE-1:0] S_AXIS_MM2S_TKEEP,
        input  wire [1-1:0]     S_AXIS_MM2S_TLAST,

        output wire             M_AXIS_S2MM_TVALID,
        input  wire             M_AXIS_S2MM_TREADY,
        output wire [TBITS-1:0] M_AXIS_S2MM_TDATA,
        output wire [TBYTE-1:0] M_AXIS_S2MM_TKEEP,
        output wire [1-1:0]     M_AXIS_S2MM_TLAST,  

        input  wire             S_AXIS_MM2S_ACLK,	//for if stream
        input  wire             M_AXIS_S2MM_ACLK,	//for of stream
        input  wire             aclk,
        input  wire             aresetn ,


		output 	wire			debug_osif_full_n,
		output	wire				debug_osif_write ,
		output	wire [TBITS - 1:0] debug_osif_data_din,
		output wire	[10:0 ]		debug_cnt_output_addr,
		output wire [2:0]		current_state

);

parameter RESET_ACTIVE_LOW = 1;	// parameter for yolo_reset_if
wire ap_rst;					// wire for yolo_reset_if

wire [TBITS - 1:0] isif_data_dout;
wire [TBYTE - 1:0] isif_strb_dout;
wire [1 - 1:0]     isif_last_dout;
wire [1 - 1:0]     isif_user_dout;
wire               isif_empty_n;
wire               isif_read;

wire [TBITS - 1:0] osif_data_din;//uc
wire [TBYTE - 1:0] osif_strb_din;
wire               osif_full_n;
wire               osif_write;//uc
wire [1 - 1:0]     osif_last_din;
wire [1 - 1:0]     osif_user_din;

wire [10:0 ] link_cnt_output_addr ;


assign	debug_osif_full_n	= osif_full_n	;
assign	debug_osif_write	=	osif_write ;
assign	debug_osif_data_din 	=	osif_data_din ;
assign debug_cnt_output_addr	= link_cnt_output_addr ;

yolo_rst_if #(
        .RESET_ACTIVE_LOW ( RESET_ACTIVE_LOW ) )
yolo_rst_if_U(
        .dout ( ap_rst ) ,
        .din ( aresetn ) );  // yolo_rst_if_U

INPUT_STREAM_if #(
        .TBITS (TBITS) ,
        .TBYTE (TBYTE)
)
INPUT_STREAM_if_U (

        .ACLK ( S_AXIS_MM2S_ACLK ) ,
        .ARESETN ( aresetn ) ,
        .TVALID ( S_AXIS_MM2S_TVALID ) ,
        .TREADY ( S_AXIS_MM2S_TREADY ) ,
        .TDATA ( S_AXIS_MM2S_TDATA ) ,
        .TKEEP ( S_AXIS_MM2S_TKEEP ) ,
        .TLAST ( S_AXIS_MM2S_TLAST ) ,      
        .TUSER ( 1'b0 ) ,

        .isif_data_dout ( isif_data_dout ) ,
        .isif_strb_dout ( isif_strb_dout ) ,
        .isif_last_dout ( isif_last_dout ) ,
        .isif_user_dout ( isif_user_dout ) ,
        .isif_empty_n ( isif_empty_n ) ,
        .isif_read ( isif_read )
);  // input_stream_if_U

OUTPUT_STREAM_if #(
        .TBITS (TBITS) ,
        .TBYTE (TBYTE)
)
OUTPUT_STREAM_if_U (

        .ACLK ( M_AXIS_S2MM_ACLK ) ,
        .ARESETN ( aresetn ) ,
        .TVALID ( M_AXIS_S2MM_TVALID ) ,
        .TREADY ( M_AXIS_S2MM_TREADY ) ,
        .TDATA ( M_AXIS_S2MM_TDATA ) ,
        .TKEEP ( M_AXIS_S2MM_TKEEP ) ,
        .TLAST ( M_AXIS_S2MM_TLAST ) ,      
        .TUSER (  ) ,

        .osif_data_din ( osif_data_din ) ,
        .osif_strb_din ( osif_strb_din ) ,
        .osif_last_din ( osif_last_din ) ,
        .osif_user_din ( osif_user_din ) ,
        .osif_full_n ( osif_full_n ) ,
        .osif_write ( osif_write )
);  // output_stream_if_U

hw_cvtor #(
	.TRANS_BYTE_SIZE( TBYTE ),
	.TRANS_BITS( TBITS )
)
hw_cvtor_port (
	.clk( aclk),
	.reset( ap_rst ),

	.din_isif_data ( isif_data_dout ) ,
	.din_isif_strb ( isif_strb_dout ) ,
	.din_isif_last ( isif_last_dout ) ,
	.din_isif_user ( isif_user_dout ) ,
	.din_isif_empty_n ( isif_empty_n ) ,
	.dout_isif_read ( isif_read ),

	.dout_osif_data ( osif_data_din ) ,
	.dout_osif_strb ( osif_strb_din ) ,
	.dout_osif_last ( osif_last_din ) ,
	.dout_osif_user ( osif_user_din ) ,
	.din_osif_full_n ( osif_full_n ) ,
	.dout_osif_write ( osif_write ) ,
	//temp
	.debug_cnt_output_addr( link_cnt_output_addr),
	.current_state (current_state )
);


endmodule