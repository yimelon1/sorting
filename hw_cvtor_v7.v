
`define BIG_ENDIAN		// when little endian is loaded by sdk or software
// ============================================================================
// Ver      : 7.0
// Designer : Yi_Yuan Chen
// Create   : 2021.10.25
// Func     : convert the 3x3 data structure to 1x1 type
//	2021/11/05: our design allow big endian data to be sorted. But , what we got
//	the data on SDK platform is "little endian" so we need to transfer the 
//	endian problem. By the way , our output data need to transfer in to little endian ?
//	2021/11/18: make tlast always =0 ;
// ============================================================================


module hw_cvtor #(
    parameter TRANS_BYTE_SIZE = 8 ,
    parameter TRANS_BITS = 64
)(
    
	input wire	clk,
    input wire	reset,

	// to INPUT_STREAM IF
	
	input wire [TRANS_BITS-1 :0 ]	din_isif_data ,
	input wire 						din_isif_last ,
	input wire						din_isif_empty_n,
	input wire	[TRANS_BYTE_SIZE-1:0]				din_isif_strb,		//can't be none
	input wire 						din_isif_user,		//can't be none

	output wire						dout_isif_read ,		// convertor wanna read

	// to OUTPUT_STREAM IF
	input wire						din_osif_full_n,

	output wire [TRANS_BITS-1 :0 ]	dout_osif_data ,
	output wire [TRANS_BYTE_SIZE-1:0]				dout_osif_strb,		//can't be none
	output wire 					dout_osif_last ,	// the last output data 
	output wire 					dout_osif_user,		//can't be none
	output wire						dout_osif_write, 	// convertor wanna write


//------------------debug pin-----------------------------
	output wire 							debug_en_soz1 ,
	output wire [  7 : 0]					debug_wea_soz1 ,
	output wire [  11 : 0] 					debug_addr_sram_soz1 ,
	output wire [  TRANS_BITS-1 : 0] 		debug_din_sram_soz1 ,
	output wire [  TRANS_BITS-1 : 0] 		debug_dout_sram_soz1 ,

	output wire 									debug_en_sto1 ,
	output wire 									debug_wea_sto1 ,
	output wire [  SRAM_STORING_BITS-1 : 0] 		debug_addr_sram_sto1 ,
	output wire [  TRANS_BITS-1 : 0] 				debug_din_sram_sto1 ,
	output wire [  TRANS_BITS-1 : 0] 				debug_dout_sram_sto1 ,

	output wire 									debug_en_sto2 ,
	output wire 									debug_wea_sto2 ,
	output wire [  SRAM_STORING_BITS-1 : 0] 		debug_addr_sram_sto2 ,
	output wire [  TRANS_BITS-1 : 0] 				debug_din_sram_sto2 ,
	output wire [  TRANS_BITS-1 : 0] 				debug_dout_sram_sto2 ,

	output wire [	9	:	0	]	debug_ch_selector	,	//	(0~3)
	output wire [	9	:	0	]	debug_ch_part 		,	//	(0~1) current ofmap ch is distribute 2 parts
	output wire [	9	:	0	] 	debug_col_part 		,
	output wire [	9	:	0	]	debug_col_in_trans 	,
	output wire [	9	:	0	]	debug_sram_choo 	,

	output wire 	[10:0 ] 		debug_cnt_output_addr,
	output 		 [2:0]				current_state


);



// temp  =====================================================
//	1 rnd = 32ch --> 32ch 1row size = 256*32 Byte = 256*32/8 entries	=1024 entries
//	

parameter IF_BUFFER_ENTRY_SIZE = 1024 ;
parameter IF_ENTRY_SIZE = 2048;   // template    256*256*64/8 = 524288  need 19 bits address
parameter REG_AY_SIZE = 2 ;
parameter BITS_OF_REG_AY_SIZE = 10;
parameter OUTPUT_ENTRIES = 1664 ;	// 208*64/8=1664
parameter SRAM_STORING_BITS = 11 ;	// 256*64/8=2048


//============================================================

//-------  parameter  ------------  parameter  ------  parameter  ------  parameter  ---//

parameter SIZE_OF_ROW 	= 208 ;
parameter SIZE_OF_COL 	= 208 ;
parameter SIZE_OF_CH	= 64 ;
parameter NUM_OF_1ROW_DATA	=	2048	;	// 256*64/8=1664


//---------------------------------------------------------------------------//




//---- BRAM_SORT  -------
wire en_soz1 ;
wire [  7 : 0]	wea_soz1 ;
wire [  11 : 0] addr_sram_soz1 ;
wire [  TRANS_BITS-1 : 0] din_sram_soz1 ;
wire [  TRANS_BITS-1 : 0] dout_sram_soz1 ;
//---- BRAM_ST  -------
wire en_sto1 ;
wire wea_sto1 ;
wire [  SRAM_STORING_BITS-1 : 0] addr_sram_sto1 ;
wire [  TRANS_BITS-1 : 0] din_sram_sto1 ;
wire [  TRANS_BITS-1 : 0] dout_sram_sto1 ;

wire en_sto2 ;
wire wea_sto2 ;
wire [  SRAM_STORING_BITS-1 : 0] addr_sram_sto2 ;
wire [  TRANS_BITS-1 : 0] din_sram_sto2 ;
wire [  TRANS_BITS-1 : 0] dout_sram_sto2 ;
//-------------------------


//------  wire  -------------  wire  ------  wire  ------  wire  ------  wire  ------  wire  ---//
//
//---- control_signal ----
wire mach_start ;	//machine start

//---- sort state output ----
wire row1_sort_done ;

//---- output state output ----
wire row1_output_done ;



//---- data_sort_wire ----



//----  store_wire ----
wire [ 9:0 ]buf_1024_cnt ;
wire [ 8:0 ]buf_array_cnt ;
wire empty_n_and_read ;


//----	reset signal	----
wire cnt_rst ;

//----	SRAM address 	----
wire [ 10 -1 : 0] store_addr ;
wire [ 10 -1 : 0] store_times ;
wire en_cnt_store_addr ;
wire en_cnt_store_times ;
wire store_end ;
wire store_addr_last ;
wire st_cho_sram ;	// store state choose sram siganl "0"=st1  "1"=st2 


//---------------------------------------------------------------------------//

//---------------------  reg  ------  reg  ------  reg  ------  reg  ------  reg  ------  reg  ---//

//---- data_store ----
reg reset_store_cnt ;
reg en_store_read ;

// data_sort 

// data_output
reg [10:0 ] cnt_output_addr ;
reg en_output_write ;

//---- last_signal ----
reg cnt_output_last ;

//---- delay logic ----
reg dly0_en_store_data , dly1_en_store_data;
reg dly0_en_sort_data , dly1_en_sort_data ;
reg dly0_en_output_data ,dly1_en_output_data ;
reg dly0_reset_store_cnt , dly1_reset_store_cnt , dly2_reset_store_cnt ;	//hope not use
reg dly0_cnt_output_last , dly1_cnt_output_last	;		

//---- ending_signal ----
reg full_data_output_done ;

//---- 	reset cnt logic  ----
reg rst_a , rst_b , rst_cnt_last	;


reg store_last ;

// set to zero 
reg zero_reg ;

//---------------------------------------------------------------------------//


//====================		FOR GPIO 				===========================
reg [ 4:0] status_reg_fsm ;

wire [ TRANS_BITS -1: 0] endian_trans_data_in ;
wire [ TRANS_BITS -1: 0] endian_trans_data_out ;
wire [ TRANS_BITS -1: 0]	output_data ;

`ifdef BIG_ENDIAN       
assign endian_trans_data_in ={ 	din_isif_data[ 7 :  0], 
								din_isif_data[15 :  8],
								din_isif_data[23 : 16],
								din_isif_data[31 : 24],
								din_isif_data[39 : 32],
								din_isif_data[47 : 40],
								din_isif_data[55 : 48],
								din_isif_data[63 : 56]
                          };
                    
assign endian_trans_data_out ={ 	output_data[ 7 :  0], 
								output_data[15 :  8],
								output_data[23 : 16],
								output_data[31 : 24],
								output_data[39 : 32],
								output_data[47 : 40],
								output_data[55 : 48],
								output_data[63 : 56]
                          }; 
                
`else
assign endian_trans_data_in         = din_isif_data;
assign endian_trans_data_out    	= output_data;
`endif


assign dout_osif_data = endian_trans_data_out ;
assign dout_osif_strb    = 'hff;	// have to be 'hff
assign dout_osif_user    = 'b0;		// no anything for user message


//------------------debug pin-----------------------------
assign debug_cnt_output_addr = cnt_output_addr ;


assign debug_en_soz1 			= en_soz1 			;
assign debug_wea_soz1 			= wea_soz1 			;
assign debug_addr_sram_soz1 	= addr_sram_soz1 	;
assign debug_din_sram_soz1 		= din_sram_soz1 	;
assign debug_dout_sram_soz1 	= dout_sram_soz1 	;

assign debug_en_sto1 			= en_sto1 			;
assign debug_wea_sto1 			= wea_sto1 			;
assign debug_addr_sram_sto1 	= addr_sram_sto1	;
assign debug_din_sram_sto1 		= din_sram_sto1 	;
assign debug_dout_sram_sto1 	= dout_sram_sto1	;

assign debug_en_sto2 			= en_sto2 			;
assign debug_wea_sto2 			= wea_sto2 			;
assign debug_addr_sram_sto2 	= addr_sram_sto2 	;
assign debug_din_sram_sto2 		= din_sram_sto2 	;
assign debug_dout_sram_sto2 	= dout_sram_sto2 	;


assign debug_ch_selector		= ch_selector			;
assign debug_ch_part 			= ch_part 				;
assign debug_col_part 			= col_part 				;
assign debug_col_in_trans 		= col_in_trans 			;
assign debug_sram_choo 			= sram_choo 			;
//------------------debug pin-----------------------------

//=============================================================================
//====================								===========================
//====================	 finite state machine		===========================
//====================								===========================
//=============================================================================
//============ FSM ======================================================================
// Designer : Yi_Yuan Chen
// Create   : 2021.9.27
// Func     : FSM for this module
// description  : next state logic , state 
//=======================================================================================
//----- FSM declare ------------
    parameter IDLE= 0;
    parameter STORE = 1 ;   //??????????????? reg (???????????????)
    parameter SORT = 2 ;    // ?????? STORE reg ???????????? ??????????????? sram (???????????????)
    parameter OUTPUT = 3 ;  
	parameter RESET_CNT = 4;	// Do after output state and reset all cnt for next data convert 

    reg [2:0] current_state , next_state ;
    reg en_store_data ,en_sort_data , en_output_data , en_reset_cnt	;

// FSM  state reg
always@( posedge clk or posedge reset )begin
    if ( reset )begin
        current_state <= IDLE;

    end else begin
        current_state <= next_state ;
    end
end

// FSM  next state logic
always@(*)begin
    case( current_state)
        IDLE : next_state = ( mach_start )? STORE : IDLE ;
        STORE : next_state = ( store_end ) ? SORT : STORE ;
        SORT : next_state = ( row1_sort_done ) ? OUTPUT: SORT; 
		OUTPUT : next_state = ( dly1_cnt_output_last ) ? RESET_CNT  : OUTPUT ;		// not yet
		RESET_CNT : next_state = ( rst_cnt_last ) ? IDLE  : RESET_CNT ;
        default : next_state = IDLE ;
    endcase
end

// FSM  output logic
// description : output enable signal for every state
always@(*)begin
    case(current_state )
    IDLE : begin
        status_reg_fsm = 5'b00001;
		en_store_data <= 0; 
        en_sort_data <= 0;
        en_output_data <= 0;
		en_reset_cnt <= 0	;
    end
    STORE : begin
		status_reg_fsm = 5'b00010;
        en_store_data <= 1; 
        en_sort_data <= 0;
        en_output_data <= 0;
		en_reset_cnt <= 0	;
    end
    SORT :begin
		status_reg_fsm = 5'b00100;
        en_store_data <= 0; 
        en_sort_data <= 1;
        en_output_data <= 0;
		en_reset_cnt <= 0	;
    end
    OUTPUT :begin
		status_reg_fsm = 5'b01000;
        en_store_data <= 0; 
        en_sort_data <= 0;
        en_output_data <= 1; 
		en_reset_cnt <= 0	;
    end
	RESET_CNT : begin
		status_reg_fsm = 5'b10000	;
        en_store_data <= 0	; 
        en_sort_data <= 0	;
        en_output_data <= 0	;
		en_reset_cnt <= 1	;
	end
    default :begin
		status_reg_fsm = 5'b00000;
        en_store_data <= 0; 
        en_sort_data <= 0;
        en_output_data <= 0;
		en_reset_cnt <= 0	;
    end
    endcase

end

//============================================================== END OF     FSM =========




//===   BLOCK RAM declare  ==============================================================
// write first or read first
BRAM_SORT SOZ1(
  .clka(    clk      ),    // input wire clka
  .ena(     en_soz1      ),      // input wire ena
  .wea(     wea_soz1     ),      // input wire [0 : 0] wea
  .addra(   addr_sram_soz1   ),  // input wire [ 11 : 0] addra
  .dina(    din_sram_soz1    ),    // input wire [ 63 : 0] dina
  .douta(   dout_sram_soz1   )  // output wire [ 63: 0] douta
);

BRAM_ST ST1(
  .clka(    clk      ),    // input wire clka
  .ena(     en_sto1      ),      // input wire ena
  .wea(     wea_sto1     ),      // input wire [0 : 0] wea
  .addra(   addr_sram_sto1   ),  // input wire [ 10 : 0] addra
  .dina(    din_sram_sto1    ),    // input wire [ 63 : 0] dina
  .douta(   dout_sram_sto1   )  // output wire [ 63: 0] douta
);

BRAM_ST ST2(
  .clka(    clk      ),    // input wire clka
  .ena(     en_sto2      ),      // input wire ena
  .wea(     wea_sto2     ),      // input wire [7 : 0] wea
  .addra(   addr_sram_sto2   ),  // input wire [ 10 : 0] addra
  .dina(    din_sram_sto2    ),    // input wire [ 63 : 0] dina
  .douta(   dout_sram_sto2   )  // output wire [ 63: 0] douta
);
//============================================= END OF BLOCK RAM declare ===============
//---- mux of BRAM_ST signal ----

assign en_sto1 = ( en_store_data & ~st_cho_sram )	? 	en_cnt_store_addr :
								( en_sort_data )	? 	1'd1 :
														1'd0 ;

assign en_sto2 = ( en_store_data & st_cho_sram )	? 	en_cnt_store_addr :
								( en_sort_data )	? 	1'd1 :
														1'd0 ;

assign wea_sto1 = ( en_store_data )	?  en_sto1 : 1'd0	;
assign wea_sto2 = ( en_store_data )	?  en_sto2 : 1'd0	;	

assign addr_sram_sto1 = ( en_store_data  & ~st_cho_sram) ? store_addr : 
										( en_sort_data ) ? sto_addr_temp :
															10'd0 ;

assign addr_sram_sto2 = ( en_store_data  & st_cho_sram) ?  store_addr : 
										( en_sort_data )?  sto_addr_temp :
															10'd0 ;

assign din_sram_sto1 = endian_trans_data_in ;		
assign din_sram_sto2 = endian_trans_data_in ;



assign addr_sram_soz1 = ( en_sort_data )	?  	cnt_soz1_addr 	:
						( en_output_data)	?	cnt_output_addr	:
												11'd0 ;
assign wea_soz1	= ( en_sort_data )?  wea_soz1_choo :
											8'd0 ;
assign en_soz1 = ( en_sort_data )	? 	dly1_en_sort_data : 
				( en_output_data)	?	dly0_en_output_data	:
										1'd0 ;	//use delay1 match sram read data

assign din_sram_soz1 = ( en_sort_data )? re_size_reg : 64'heeee_eeee_eeee_eeee ;
//----------------------------------















//========================================================================================
//========================================================================================
//===================		  Delay logic  				==================================
//========================================================================================
//function : delay enable signal
//description : delay the state enable signal 2~3 clocks 
//========================================================
//----------------------------
//always block : delay en_store_data 
//----------------------------
always@( posedge clk or posedge reset )begin
	if( reset )begin
		dly0_en_store_data <= 0;
		dly1_en_store_data <= 0;
	end
	else begin
		dly0_en_store_data <= en_store_data ;	// delay0 en_store_data for :
		dly1_en_store_data <= dly0_en_store_data ;
	end
end	

//----------------------------
//always block : delay en_sort_data 
//----------------------------
always@( posedge clk or posedge reset )begin
	if( reset )begin
		dly0_en_sort_data <= 0;
		dly1_en_sort_data <= 0;
	end
	else begin
		dly0_en_sort_data <= en_sort_data ;	// delay0 en_sort for : BRAM STORE , data_sort
		dly1_en_sort_data <= dly0_en_sort_data ;
	end
end	

//----------------------------
//always block : delay en_output_data 
//----------------------------
always@( posedge clk or posedge reset )begin
	if( reset )begin
		dly0_en_output_data <= 0;
		dly1_en_output_data <= 0;
	end
	else begin
		dly0_en_output_data <= en_output_data ;
		dly1_en_output_data <= dly0_en_output_data ;
	end
end

//====================================================	end of delay	==================
//========================================================================================




//========================================================================================
//==================							==========================================
//==================		BRAM control		==========================================
//==================							==========================================
//========================================================================================
// this block make BRAM address and wea 
// row0 col0 	ch0		ch1		ch2	...	ch31	|	ch32				ch33	...
// address		0		32		64	...	31*32	|	32*32(sw_handled)	33*32	...
// wea			1000_0000

// row0 col7 	ch0		ch1		ch2	...	ch31	|	ch32				ch33	...
// address		0		32		64	...	31*32	|	32*32(sw_handled)	33*32	...
// wea			0000_0001

// row0 col8 	ch0		ch1		ch2	...	ch31	|	ch32				ch33	...
// address		1		33		65	...	31*32+1	|	32*32+1(sw_handled)	33*32+1	...
// wea			1000_0000





count_yi_v2 #(
    .BITS_OF_END_NUMBER( 10  ) ,
    .END_NUMBER ( 1024 )
)cnt_store_addr(
    .clk ( clk ),
    .reset ( cnt_rst ), 
    .enable ( en_cnt_store_addr ), 
    .cnt_q ( store_addr )
);

count_yi_v2 #(
    .BITS_OF_END_NUMBER( 10  ) ,
    .END_NUMBER ( 1024 )
)cnt_st_times(
    .clk ( clk ),
    .reset ( cnt_rst ), 
    .enable ( en_cnt_store_times ), 
    .cnt_q ( store_times )
);

// state turning logic
assign mach_start = din_isif_empty_n && ( current_state == IDLE ) ;
assign store_end = store_last ;



//========================================================================================
//==================							==========================================
//==================		store control		==========================================
//==================							==========================================
//========================================================================================
// description : generate controling signal 
//		read , store_end , enable of cnt ,st_cho_sram
//-----------------------------------------------------------------------------------------
assign store_addr_last = store_addr == 'd1023 ;
always@( *) begin
	if( en_store_data )begin
		if ( (store_times =='d1) && store_addr_last )begin
			store_last = 'd1 ;
		end 
		else begin
			store_last = 'd0 ;
		end
	end else begin
		store_last = 'd0 ;
	end
end

//----------------------------------------------------
// store_control : read signal generator
//	function : empty_n ????????? read ???????????????
//----------------------------------------------------
assign dout_isif_read = en_store_read ;		// Segmented the reading data
assign empty_n_and_read = en_store_read & din_isif_empty_n;
always@( posedge clk or posedge reset ) begin
	if( reset )begin
		en_store_read <= 'd0 ;
	end
	else begin
		if ( din_isif_empty_n & en_store_data )begin
			en_store_read <= 'd1 ;
		end
		else begin
			en_store_read <= 'd0 ;
		end
	end
end


//----------------------------------------------------
// store_control : sram address generator
//	function : ??????????????????????????? sram ?????? sram address
//----------------------------------------------------
assign en_cnt_store_addr = empty_n_and_read &  en_store_data ;
assign en_cnt_store_times =  empty_n_and_read & ( store_addr == 'd1023 ) ;
assign st_cho_sram = ( store_times == 10'd1 )? 'd1 : 'd0 ;		// choose st1 or st2 for storing



//========================================================================================
//==================							==========================================
//==================		sort control		==========================================
//==================							==========================================
//========================================================================================
//	description : generate st_en,wea  soz1_en soz1_wea , cnt8 , st ram counter , 

wire [ 14:0 ] temp01 ;
wire [ 9:0] temp02;
wire [ 9:0] temp03;
wire [ 9:0] temp04;
wire [	9	:	0	]	ch_selector	;	//	(0~3)
wire [	9	:	0	]	ch_part ;		//	(0~1) current ofmap ch is distribute 2 parts
wire [	9	:	0	] 	col_part 	;
wire [ 18:0 ] multi_row ;
wire [ 18:0 ] multi_ch_part ;
wire [ 22:0] multi_ch_selec ;
wire [ 23:0 ]sto_addr_temp ;
//----temp
reg [ 63 :0] re_size_reg ;	
reg [ 7 : 0] col_choo_reg ;// 8bits
wire  [ 63 :0] sram_sto_choose ;
wire [ 9:0 ]col_in_trans ;
wire [ 9:0 ]sram_choo ;

wire [ 10:0 ] cnt_soz1_addr ;
reg [7:0 ] wea_soz1_choo ;
// counter enable_signal
reg	en_ch_selector ;
reg	en_ch_part  ;
reg en_col_in_trans ;
reg en_sram_choo ;
reg en_col_part ;


reg en_sort_addr ;





reg [ 9:0 ]dly0_ch_selector ;
reg [ 9:0 ]dly1_ch_selector ;
reg [ 9:0 ]dly2_ch_selector ;

reg [9:0] dly0_sram_choo ;
reg [9:0] dly1_sram_choo ;
reg [9:0] dly2_sram_choo ;

reg [9:0] dly0_col_in_trans ;
reg [9:0] dly1_col_in_trans ;
reg [9:0] dly2_col_in_trans ;


count_yi_v3 #(
    .BITS_OF_END_NUMBER( 15  ) 
)cnt_8(
    .clk ( clk ),
    .reset ( cnt_rst ), 
    .enable ( en_ch_selector ), 
	.final_number(	4096	),
    .cnt_q ( temp01 )			// temp ending
);

count_yi_v3 #(
    .BITS_OF_END_NUMBER( 10  ) 
)cnt_sram_chooser(
    .clk ( clk ),
    .reset ( cnt_rst ), 
    .enable ( en_sram_choo ), 	//temp
	.final_number(	'd2	),			//temp
    .cnt_q ( sram_choo )			//sram choose  sort
);

count_yi_v3 #(
    .BITS_OF_END_NUMBER( 10  ) 
)cnt_9(
    .clk ( clk ),
    .reset ( cnt_rst ), 
    .enable ( en_col_in_trans ), 
	.final_number(	'd8	),		// col only choose to 208 not 256
    .cnt_q ( col_in_trans )
);

count_yi_v3 #(
    .BITS_OF_END_NUMBER( 10  ) 
)cnt_11(
    .clk ( clk ),
    .reset ( cnt_rst ), 
    .enable ( en_ch_selector ), 	//temp
	.final_number(	'd8	),			//temp
    .cnt_q ( ch_selector )			// ch 0~7 choose
);
count_yi_v3 #(
    .BITS_OF_END_NUMBER( 10  ) 
)cnt_12(
    .clk ( clk ),
    .reset ( cnt_rst ), 
    .enable ( en_ch_part ), 	//temp
	.final_number(	'd4 	),			//temp
    .cnt_q ( ch_part )			// ch part 
);

count_yi_v3 #(
    .BITS_OF_END_NUMBER( 10  ) 
)cnt_14(
    .clk ( clk ),
    .reset ( cnt_rst ), 
    .enable ( en_col_part ), 	//temp
	.final_number(	'd32	),			//temp
    .cnt_q ( col_part )			// col part
);


//---- sram soz addr counter ----
count_yi_v3 #(
    .BITS_OF_END_NUMBER( 11  ) 
)cnt_13(
    .clk ( clk ),
    .reset ( cnt_rst ), 
    .enable ( en_sort_addr ), 	//temp
	.final_number(	'd2048	),			//temp
    .cnt_q ( cnt_soz1_addr )			// temp
);


//---- SORT counter enable control ----

assign multi_ch_selec = ch_selector *32 ;
assign multi_ch_part = ch_part * 256 ;
assign sto_addr_temp = multi_ch_selec + multi_ch_part + col_part;

always@( * )begin
	if ( current_state == SORT )begin
		en_ch_selector 	= 1'd1 ;
		if ( ch_selector == 'd7)  	en_ch_part = 1'd1 ; 		
							else 	en_ch_part = 1'd0 ;

		if ( (ch_selector == 'd7 ) & (ch_part == 'd3 )) 	en_sram_choo =1'd1 ; 	
													else 	en_sram_choo =1'd0 ;

		if ( (ch_selector == 'd7 ) & (ch_part == 'd3 ) & (sram_choo == 'd1 )) 	en_col_in_trans =1'd1 ; 	
																		else 	en_col_in_trans =1'd0 ;
		if( (ch_selector == 'd7 ) & (ch_part == 'd3 ) & (sram_choo == 'd1 ) & ( col_in_trans == 7))	en_col_part = 1'd1;
																							else	en_col_part = 1'd0;

	end else begin 
		en_ch_selector 	= 1'd0 ;	 
		en_ch_part 		= 1'd0 ; 	 	
		en_col_in_trans = 1'd0 ;
		en_col_part		= 1'd0 ;
		en_sram_choo 	= 1'd0 ;

	end
end

always@(*) begin
	if( current_state == SORT  )begin
		if ( dly1_ch_selector == 'd7)  	en_sort_addr = 1'd1 ; 		
							else 	en_sort_addr = 1'd0 ;
	end else begin
		en_sort_addr = 1'd0 ;
	end
end

//---- dout chooser ----
assign row1_sort_done = ( (cnt_soz1_addr == 'd2047 ) & ( dly1_ch_selector == 3'd7 ));

assign sram_sto_choose = ( dly1_sram_choo == 10'd0)?	 	dout_sram_sto1	:
							( dly1_sram_choo == 10'd1)?	dout_sram_sto2	:
													64'hffff_ffff_ffff_ffff	;



//[ 63 :0] re_size_reg ;	
//[ 7 : 0] col_choo_reg// 8bits
always@( * )begin
	case( dly1_col_in_trans )
		3'd0: col_choo_reg = sram_sto_choose [63 -: 8] 	;
		3'd1: col_choo_reg = sram_sto_choose [55 -: 8] 	;
		3'd2: col_choo_reg = sram_sto_choose [47 -: 8] 	;
		3'd3: col_choo_reg = sram_sto_choose [39 -: 8] 	;
		3'd4: col_choo_reg = sram_sto_choose [31 -: 8] 	;
		3'd5: col_choo_reg = sram_sto_choose [23 -: 8] 	;
		3'd6: col_choo_reg = sram_sto_choose [15 -: 8] 	;
		3'd7: col_choo_reg = sram_sto_choose [7 -: 8] 	;
		default : col_choo_reg = 8'd0 ;
	endcase
end
//--- generate 64bits data structure
always@( * )begin
	case( dly1_ch_selector )
		3'd0: re_size_reg = { 			col_choo_reg	 , 		56'd0 	};
		3'd1: re_size_reg = { 8'd0 , 	col_choo_reg	 ,		48'd0	};
		3'd2: re_size_reg = { 16'd0 , 	col_choo_reg	 ,		40'd0	};
		3'd3: re_size_reg = { 24'd0 , 	col_choo_reg	 ,		32'd0	};
		3'd4: re_size_reg = { 32'd0 , 	col_choo_reg	 ,		24'd0	};
		3'd5: re_size_reg = { 40'd0 , 	col_choo_reg	 ,		16'd0	};
		3'd6: re_size_reg = { 48'd0 , 	col_choo_reg	 ,		8'd0	};
		3'd7: re_size_reg = { 56'd0 , 	col_choo_reg					};
		default : re_size_reg = 64'hffff_ffff_ffff_ffff ;
	endcase
end

//---- shifte counter for SRAM_SOZ CHoose data 
always@( posedge clk or posedge reset )begin
	if ( reset )begin
		dly0_ch_selector <= 0;
		dly1_ch_selector <= 0;
		dly2_ch_selector <= 0;
	end else begin
		dly0_ch_selector <= ch_selector ;
		dly1_ch_selector <= dly0_ch_selector	;
		dly2_ch_selector <= dly1_ch_selector	;
	end
end


always@( posedge clk or posedge reset )begin
	if ( reset )begin
		dly0_sram_choo <= 0;
		dly1_sram_choo <= 0;
		dly2_sram_choo <= 0;
	end else begin
		dly0_sram_choo <= sram_choo ;
		dly1_sram_choo <= dly0_sram_choo	;
		dly2_sram_choo <= dly1_sram_choo	;
	end
end


always@( posedge clk or posedge reset )begin
	if ( reset )begin
		dly0_col_in_trans <= 0;
		dly1_col_in_trans <= 0;
		dly2_col_in_trans <= 0;
	end else begin
		dly0_col_in_trans <= col_in_trans		;
		dly1_col_in_trans <= dly0_col_in_trans	;
		dly2_col_in_trans <= dly1_col_in_trans	;
	end
end

//---- soz wea generator ----
always@( * )begin
	case( dly1_ch_selector )
		3'd0: wea_soz1_choo = 	8'b1000_0000		;
		3'd1: wea_soz1_choo = 	8'b0100_0000		;
		3'd2: wea_soz1_choo = 	8'b0010_0000		;
		3'd3: wea_soz1_choo = 	8'b0001_0000		;
		3'd4: wea_soz1_choo = 	8'b0000_1000		;
		3'd5: wea_soz1_choo = 	8'b0000_0100		;
		3'd6: wea_soz1_choo = 	8'b0000_0010		;
		3'd7: wea_soz1_choo = 	8'b0000_0001		;
		default : wea_soz1_choo = 8'hff ;
	endcase
end


//=============================================================================
//======================== 								=======================
//======================== 		output data state 		=======================
//======================== 								=======================
//=============================================================================
// description : count and generate osif_write and reading SRAM data 
assign output_data = ( en_output_write ) ? dout_sram_soz1 : 'd0 ;
assign row1_output_done = dly1_cnt_output_last ;
assign dout_osif_last = dly1_cnt_output_last ;		// 10/14 need to wait for BRAM's reading data process about 2 clk
assign dout_osif_write = en_output_write &  en_output_data;

//--------------------------------
//always block : row1 data output
//--------------------------------
always@( posedge clk or posedge reset )begin
	if ( reset )begin
		en_output_write <= 0;
	end 
	else begin
		if ( dly1_en_output_data )begin
			en_output_write <= 'd1 ;
		end
		else begin
			en_output_write <= 'd0 ;
		end
	end
end

//--------------------------------
//always block : counter data output
//--------------------------------
always@( posedge clk or posedge cnt_rst )begin
    if ( cnt_rst	)begin
        cnt_output_addr<= 'd0; 
    end else begin
		if ( dly0_en_output_data )begin
			if( cnt_output_addr >= OUTPUT_ENTRIES-1 )begin
				cnt_output_addr<= 'd0 ;
			end 
			else begin
				cnt_output_addr	<= cnt_output_addr + 'd1 ;
			end
		end 
		else begin
			cnt_output_addr <= cnt_output_addr ;
			end
    end
end 

//--------------------------------
//always block : last signal of counter data output
//--------------------------------
always@( *)begin
	if( cnt_output_addr == OUTPUT_ENTRIES-1) begin
		cnt_output_last = 1;
	end else begin
		cnt_output_last = 0;
	end
end

//--------------------------------
//always block : delay of last signal for counter data output
//--------------------------------
always@( posedge clk or posedge reset ) begin
	if( reset )begin
		dly0_cnt_output_last <= 0;
		dly1_cnt_output_last <= 0;
	end else begin
		dly0_cnt_output_last <= cnt_output_last ;
		dly1_cnt_output_last <= dly0_cnt_output_last ;
	end
end


//=============================================================================
//======================== 							===========================
//======================== 		END LOGIC 			===========================
//======================== 							===========================
//=============================================================================
// no used
always@( posedge clk or posedge reset  )begin
	if( reset )begin
		full_data_output_done <= 0 ;
	end
	else begin
		if ( ch_selector >= 'd3 && ch_part >= 'd1  )begin
			full_data_output_done <= 1 ;
		end else begin
			full_data_output_done <= 0 ;
		end	
	end

end

//================================================================================================

//=============================================================================
//======================== 							===========================
//======================== 		 RESET LOGIC		===========================
//======================== 							===========================
//=============================================================================
always@( posedge clk )begin
	if( en_reset_cnt )begin
		if( ~rst_cnt_last)begin
			rst_a <= 1 ;
		end else begin
			rst_a <= 0 ;
		end
		rst_b <= rst_a ;
		rst_cnt_last <= rst_b ;
	end	else begin
		rst_a <= 0	;
		rst_b <= 0 	;
		rst_cnt_last <= 0;
	end
end


assign cnt_rst = reset | rst_a ;


//========================================================================================
//==================							==========================================
//==================		unit region			==========================================
//==================							==========================================
//========================================================================================
// sort counter control signal
always@(*)begin
	if(reset)begin
		zero_reg = 0;
	end else begin
		zero_reg = 0;
	end
end


endmodule 



//================================================================================================
//====================  counter module  ==============================
//  Developer: YI
//  description : enable for count , disable for stall
//      reset for initial . every cycle count 1 .
//================================================================================================

module count_yi #(
	parameter BITS_OF_END_NUMBER = 10 
)	(
    clk,
    reset , 
    enable , 
    nu_end , 
    cnt_q
);
input clk ;
input reset ;
input enable ;
input  wire [   BITS_OF_END_NUMBER -1 : 0]      nu_end ;
output  reg [   BITS_OF_END_NUMBER -1 : 0]       cnt_q ;

always@( posedge clk or posedge reset )begin
    if ( reset )begin
        cnt_q<= 'd0; 
    end else begin
		if ( enable )begin
			if( cnt_q >= nu_end )begin
				cnt_q<= 'd0 ;
			end 
			else begin
				cnt_q<= cnt_q + 'd1 ;
			end
		end 
		else begin
			cnt_q <= cnt_q ;
			end

    end
end 

endmodule

//================================================================================================
//====================  counter module  ==============================
//  Developer: YI
//  description : enable for count , disable for stall
//      reset for initial . every cycle count 1 .
//      count end number is defined by parameter .
//================================================================================================
module count_yi_v2 #(
    parameter BITS_OF_END_NUMBER = 10   ,
    parameter END_NUMBER = 1024
)(
    clk,
    reset , 
    enable , 
    cnt_q
);
input clk ;
input reset ;
input enable ;
output reg     [   BITS_OF_END_NUMBER -1 : 0]   cnt_q ;

always@( posedge clk or posedge reset )begin
    if ( reset )begin
        cnt_q<= 'd0; 
    end else begin
		if ( enable )begin
			if( cnt_q >= END_NUMBER-1 )begin
				cnt_q<= 'd0 ;
			end 
			else begin
				cnt_q<= cnt_q + 'd1 ;
			end
		end 
		else begin
			cnt_q <= cnt_q ;
			end

    end
end 

endmodule

//================================================================================================
//====================  counter module  ==============================
//  Developer: YI
//  description : enable for count , disable for stall
//      reset for initial . every cycle count 1 .
//      count end number is defined by final_number .
//================================================================================================
module count_yi_v3 #(
    parameter BITS_OF_END_NUMBER = 20
)(
    clk,
    reset , 
    enable , 
	final_number,
    cnt_q	
);
input 	clk ;
input 	reset ;
input 	enable ;
input		[   BITS_OF_END_NUMBER -1 : 0] 	final_number ;
output reg	[   BITS_OF_END_NUMBER -1 : 0]  cnt_q ;

always@( posedge clk or posedge reset )begin
    if ( reset )begin
        cnt_q<= 'd0; 
    end else begin
		if ( enable )begin
			if( cnt_q >= final_number-1 )begin
				cnt_q<= 'd0 ;
			end 
			else begin
				cnt_q<= cnt_q + 'd1 ;
			end
		end 
		else begin
			cnt_q <= cnt_q ;
			end

    end
end 

endmodule