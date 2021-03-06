
`define BIG_ENDIAN		// when little endian is loaded by sdk or software
// ============================================================================
// Ver      : 7.0
// Designer : Yi_Yuan Chen
// Create   : 2021.10.25
// Func     : convert the 3x3 data structure to 1x1 type
//	2021/11/05: our design allow big endian data to be sorted. But , what we got
//	the data on SDK platform is "little endian" so we need to transfer the 
//	endian problem. By the way , our output data need to transfer in to little endian ?
//	2021/11/18: make TLAST = "1" at the right time
//	2021/12/02: col_part is the sort mem address and it has fianl num =32  but it can suit 
//		to below 32 address, and it's not belong soz1 address
//	2021/12/07:	we need two type 3x3 transform  (not yet)
//		1. transform 3x3 ofmap into channel-major order map	for 1x1 convolution
//		2. transform 3x3 ofmap into row-major order map for shortcut and maxpool
//	2022/01/11: made the 31 to 32 transform success on simulation by "cfg_mode=32'd2"
//	2022/01/19: think about 3x3 shortcut 
//		1. choose cfg_mode for 3 (normal size layer) and 4 (layer3x)
//	2022/02/09: new mode 4 ,5 ,6 sim ok
//	2022/02/17: new mode 7 filter mode 
//	2022/02/22: new mode 8 : for 1x1 to 3x3
//		1.	input size changed to 832 and 416( for 13*256)
//		2.	counter hierarchy need to change.
//----------------------------------------------------------------------
//	cfg_mode = 1 transform 3x3 ofmap into channel-major order map	for 1x1 convolution
//	cfg_mode = 2 made the 31 to 32 transform success on simulation 
//	cfg_mode = 3 transform 3x3 ofmap into row-major order map for shortcut and maxpool 
//					just size 104 and 208. unsorting mode 
//	cfg_mode = 4	for size13 row-major order		---2022/02/08: new mode 4
//	cfg_mode = 5	for size26 row-major order		---2022/02/08: new mode 5 and 6
//	cfg_mode = 6	for size52 row-major order		---2022/02/08: new mode 5 and 6
//	cfg_mode = 7	gather mem non filter	---2022/02/17: new mode 7  for Conv1x1_transform_complete
//	cfg_mode = 8	1x1 transform 3x3 	---2022/02/22:	size 208 input 832 , sim ok
//	cfg_mode = 9	1x1 transform 3x3 	---2022/03/01:	size 104 input 832 ,without sim
//	cfg_mode = 10	1x1 transform 3x3 	---2022/03/01:	size 52 input 832 ,without sim
//	cfg_mode = 11	1x1 transform 3x3 	---2022/03/01:	size 26 input 832 ,without sim
//	cfg_mode = 12	1x1 transform 3x3 	---2022/03/01:	size 13 input 416 ,without sim
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
parameter OUTPUT_ENTRIES = 1664 ;	// 208*64/8=1664
parameter SRAM_STORING_BITS = 11 ;	// 256*64/8=2048

//============================================================

//-------  parameter  ------------  parameter  ------  parameter  ------  parameter  ---//

//layer2 to layer3 parameter
/*
parameter	ROWBASE_CH_STEP 	= 'd32 	;		// col_size / 8 = 256/8 =	32  after this step, hw can get current column's next channel
parameter	ROWBASE_CHPART_STEP = 'd256	;		// after 8 ch is sort , next 8ch address start at ROWBASE_CH_STEP * 8 , 32*8 =256
parameter 	CH_PART_NUM 		= 'd4		;		// total ch / 8 / 2sram = 64/8/2 = 4
*/

//layer7 to layer8 parameter
// output ch once = 3*3 * lay7_if_ch * lay7_of_ch / 3*3 *1024  , 64*128/1024 = 8
parameter	ROWBASE_CH_STEP 	= 'd16 	;		// col_size / 8 = 128/8 =16	 after this step, hw can get current column's next channel
parameter	ROWBASE_CHPART_STEP = 'd128	;		// after 8 ch is sort , next 8ch address start at ROWBASE_CH_STEP * 8 , 16*8 = 128
parameter 	CH_PART_NUM 		= 'd8		;		// total ch / 8 / 2sram = 128/8/2 = 8


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
wire [  SRAM_STORING_BITS-1 : 0] addr_sram_sto1 ;	//depth 2048
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
//---- config control----
wire cfg_end ;
wire cvtr_mode1 ;
wire cvtr_mode2 ;
wire cvtr_mode3 ;
wire cvtr_mode4 ;
wire cvtr_mode5 ;
wire cvtr_mode6 ;
wire cvtr_mode7 ;
wire cvtr_mode8 ;


//---- control_signal ----
wire mach_start ;	//machine start

//---- sort state output ----
wire row1_sort_done ;

//---- output state output ----
wire row1_output_done ;
wire [ 10 : 0 ] out_addr_num ;


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
wire [ 10: 0 ] 	st_addr_end_num ;

//---------------------------------------------------------------------------//

//---------------------  reg  ------  reg  ------  reg  ------  reg  ------  reg  ------  reg  ---//


//---- config_store ----
reg en_cfg_read ;

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
reg [ 5:0 ] dly0_soz_state_cnt , dly1_soz_state_cnt;

//---- ending_signal ----
reg full_data_output_done ;

//---- 	reset cnt logic  ----
reg rst_a , rst_b , rst_cnt_last	;


reg store_last ;

// set to zero 
reg zero_reg ;

//---------------------------------------------------------------------------//


//====================		FOR GPIO 				===========================
reg [ 5:0] status_reg_fsm ;

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
	parameter CFG_READ = 1;	// config reading
    parameter STORE = 2 ;   //??????????????? reg (???????????????)
    parameter SORT = 3 ;    // ?????? STORE reg ???????????? ??????????????? sram (???????????????)
    parameter OUTPUT = 4 ;  
	parameter RESET_CNT = 5;	// Do after output state and reset all cnt for next data convert 
	

    reg [2:0] current_state , next_state ;
    reg en_store_data ,en_sort_data , en_output_data , en_reset_cnt	, en_cfg ;

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
        IDLE : next_state = ( mach_start )? CFG_READ : IDLE ;
		CFG_READ : next_state = (cfg_end)?  STORE	:  	CFG_READ	;
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
        status_reg_fsm = 6'b000001;
		en_store_data 	= 0	; 
        en_sort_data 	= 0	;
        en_output_data 	= 0	;
		en_reset_cnt 	= 0	;
		en_cfg			= 0	;	
    end
	CFG_READ : begin
		status_reg_fsm = 6'b000010;
		en_store_data 	= 0	; 
        en_sort_data 	= 0	;
        en_output_data 	= 0	;
		en_reset_cnt 	= 0	;
		en_cfg			= 1	;
	end
    STORE : begin
		status_reg_fsm = 6'b000100;
        en_store_data 	= 1	; 
        en_sort_data 	= 0	;
        en_output_data 	= 0	;
		en_reset_cnt 	= 0	;
		en_cfg			= 0	;
    end
    SORT :begin
		status_reg_fsm = 6'b001000;
        en_store_data 	= 0	; 
        en_sort_data 	= 1	;
        en_output_data 	= 0	;
		en_reset_cnt 	= 0	;
		en_cfg			= 0	;
    end
    OUTPUT :begin
		status_reg_fsm = 6'b010000;
        en_store_data 	= 0	; 
        en_sort_data 	= 0	;
        en_output_data 	= 1	; 
		en_reset_cnt 	= 0	;
		en_cfg			= 0	;
    end
	RESET_CNT : begin
		status_reg_fsm = 6'b100000	;
        en_store_data 	= 0	; 
        en_sort_data 	= 0	;
        en_output_data 	= 0	;
		en_reset_cnt 	= 1	;
		en_cfg			= 0	;
	end
    default :begin
		status_reg_fsm = 6'b000000;
        en_store_data 	= 0	; 
        en_sort_data 	= 0	;
        en_output_data 	= 0	;
		en_reset_cnt 	= 0	;
		en_cfg			= 0	;
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
//==================		config control		==========================================
//==================							==========================================
//========================================================================================
// description : read config and store to reg with clk and count.
// config format : 
//	32bit for 1 option
//	1. mode choose 			: "0" for 3x3 to 1x1 , "1" 3x3 to normal row based
//	2. ROWBASE_CH_STEP 		:	// col_size / 8 = 128/8 =16	 after this step, hw can get current column's next channel
//	3. ROWBASE_CHPART_STEP 	:	// after 8 ch is sort , next 8ch address start at ROWBASE_CH_STEP * 8 , 16*8 = 128
//	4. CH_PART_NUM			:	// total ch / 8 / 2sram = 128/8/2 = 8
//-----------------------------------------------------------------------------------------
wire en_cfg_cnt ;
wire [2:0]	cfg_cnt ;
wire [31:0] cfg_mode ,  cfg_rowbase_ch_step , cfg_rowbase_chpart_step ,cfg_ch_part_num ;
wire [31:0] cfg_fcol_in_trans , cfg_nothing	;
reg [TRANS_BITS-1 : 0 ] cfg_data0	;	//for mode choose and rowbase_ch_step	.
reg [TRANS_BITS-1 : 0 ] cfg_data1	;	//for ROWBASE_CHPART_STEP and CH_PART_NUM.
reg [TRANS_BITS-1 : 0 ] cfg_data2	;	//none .


count_yi_v3 #(
    .BITS_OF_END_NUMBER( 3  ) 
)cnt_cfg0(
    .clk ( clk ),
    .reset ( cnt_rst ), 
    .enable ( en_cfg_cnt ), 
	.final_number(	'd3	),		// col only choose to 208 not 256
    .cnt_q ( cfg_cnt )
);

assign en_cfg_cnt = ( empty_n_and_read && en_cfg	) ? 1'd1 : 1'd0;

always@(posedge clk or posedge reset)begin
	if(reset)begin
		cfg_data0 <= 64'd0;
		cfg_data1 <= 64'd0;
		cfg_data2 <= 64'd0;
	end
	else begin
		if ( en_cfg && (cfg_cnt==3'd0))	cfg_data0 <= endian_trans_data_in ; else cfg_data0 <= cfg_data0;
		if ( en_cfg && (cfg_cnt==3'd1))	cfg_data1 <= endian_trans_data_in ; else cfg_data1 <= cfg_data1;
		if ( en_cfg && (cfg_cnt==3'd2))	cfg_data2 <= endian_trans_data_in ; else cfg_data2 <= cfg_data2;
	end
end


assign cfg_mode 				= cfg_data0[ 63 -: 32];
assign cfg_rowbase_ch_step 		= cfg_data0[ 31 -: 32];
assign cfg_rowbase_chpart_step 	= cfg_data1[ 63 -: 32];
assign cfg_ch_part_num			= cfg_data1[ 31 -: 32];
assign cfg_fcol_in_trans		= cfg_data2[ 63 -: 32];
assign cfg_nothing		 		= cfg_data2[ 31 -: 32];

assign cvtr_mode1	=	(cfg_mode == 32'd1 )?	1'b1 : 1'b0 ;
assign cvtr_mode2	=	(cfg_mode == 32'd2 )?	1'b1 : 1'b0 ;
assign cvtr_mode3	=	(cfg_mode == 32'd3 )?	1'b1 : 1'b0 ;
assign cvtr_mode4	=	(cfg_mode == 32'd4 )?	1'b1 : 1'b0 ;
assign cvtr_mode5	=	(cfg_mode == 32'd5 )?	1'b1 : 1'b0 ;
assign cvtr_mode6	=	(cfg_mode == 32'd6 )?	1'b1 : 1'b0 ;
assign cvtr_mode7	=	(cfg_mode == 32'd7 )?	1'b1 : 1'b0 ;
assign cvtr_mode8	=	(cfg_mode == 32'd8 )?	1'b1 : 1'b0 ;
assign cvtr_mode9	=	(cfg_mode == 32'd9 )?	1'b1 : 1'b0 ;
assign cvtr_mode10	=	(cfg_mode == 32'd10 )?	1'b1 : 1'b0 ;
assign cvtr_mode11	=	(cfg_mode == 32'd11 )?	1'b1 : 1'b0 ;
assign cvtr_mode12	=	(cfg_mode == 32'd12 )?	1'b1 : 1'b0 ;
//--------- config ending -------
assign cfg_end = en_cfg && (	cfg_cnt==3'd2 );
//------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------

// state turning logic
assign mach_start = din_isif_empty_n && ( current_state == IDLE ) ;
assign store_end = store_last ;


count_yi_v3 #(
    .BITS_OF_END_NUMBER( 11  ) 
)cnt_store_addr(
    .clk ( clk ),
    .reset ( cnt_rst ), 
    .enable ( en_cnt_store_addr ), 	// count at input reading
	.final_number(	st_addr_end_num 	),			//temp
    .cnt_q ( store_addr )			// store_addr
);


/*
count_yi_v2 #(
    .BITS_OF_END_NUMBER( 10  ) ,
    .END_NUMBER ( 1024 )
)cnt_store_addr(
    .clk ( clk ),
    .reset ( cnt_rst ), 
    .enable ( en_cnt_store_addr ), 
    .cnt_q ( store_addr )
);
*/
count_yi_v2 #(
    .BITS_OF_END_NUMBER( 10  ) ,
    .END_NUMBER ( 2 )
)cnt_st_times(
    .clk ( clk ),
    .reset ( cnt_rst ), 
    .enable ( en_cnt_store_times ), 
    .cnt_q ( store_times )
);

//========================================================================================
//==================							==========================================
//==================		store control		==========================================
//==================							==========================================
//========================================================================================
// description : generate controling signal 
//		read , store_end , enable of cnt ,st_cho_sram
//		config mode = 2 :
//			doing 3x3 to 1x1 for layer3x , because the smaller input feature map
//			need change store data size 
//-----------------------------------------------------------------------------------------
assign store_addr_last =	( cvtr_mode2 | cvtr_mode4 )?									store_addr == 'd511 : 
								(cvtr_mode8 | cvtr_mode9 | cvtr_mode10 | cvtr_mode11 )?		store_addr == 'd415 : 
									(cvtr_mode12 )?  										store_addr == 'd207 : 
																							store_addr == 'd1023 ;
assign st_addr_end_num = ( cvtr_mode2 | cvtr_mode4 )? 									'd512 :
							(cvtr_mode8 | cvtr_mode9 | cvtr_mode10 | cvtr_mode11 )?		'd416 :  
								(cvtr_mode12 )?   										'd208 :
																						'd1024 ;

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
assign dout_isif_read = en_store_read | en_cfg_read ;		// Segmented the reading data
assign empty_n_and_read = dout_isif_read & din_isif_empty_n;
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
always@( posedge clk or posedge reset ) begin
	if( reset )begin
		en_cfg_read <= 'd0 ;
	end
	else begin
		if ( din_isif_empty_n & en_cfg )begin
			en_cfg_read <= 'd1 ;
		end
		else begin
			en_cfg_read <= 'd0 ;
		end
	end
end


//----------------------------------------------------
// store_control : sram address generator
//	function : ??????????????????????????? sram ?????? sram address
//----------------------------------------------------
assign en_cnt_store_addr = empty_n_and_read &  en_store_data ;
assign en_cnt_store_times =  empty_n_and_read & store_addr_last ;
assign st_cho_sram = ( store_times == 10'd1 )? 'd1 : 'd0 ;		// choose st1 or st2 for storing



//========================================================================================
//==================							==========================================
//==================		sort control		==========================================
//==================							==========================================
//========================================================================================
//	description : generate st_en,wea  soz1_en soz1_wea , cnt8 , st ram counter , 
//		now we need SCHORTCUT mode.
//----------------------------------------------------------------------------------------

wire [ 14:0 ] temp01 ;
wire [ 9:0] temp02;
wire [ 9:0] temp03;
wire [ 9:0] temp04;
wire [	4	:	0	]	ch_selector	;	//	(0~3)
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



reg [ 4:0 ]dly0_ch_selector ;
reg [ 4:0 ]dly1_ch_selector ;
reg [ 4:0 ]dly2_ch_selector ;

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
)cnt_9(
    .clk ( clk ),
    .reset ( cnt_rst ), 
    .enable ( en_col_in_trans ), 
	.final_number(	cfg_fcol_in_trans	),		// col only choose to 208 not 256
    .cnt_q ( col_in_trans )
);

wire [4:0]fnum_ch_selector;
assign fnum_ch_selector = (cvtr_mode6 ) ? 5'd2 : 5'd8;


count_yi_v3 #(
    .BITS_OF_END_NUMBER( 5  ) 
)cnt_11(
    .clk ( clk ),
    .reset ( cnt_rst ), 
    .enable ( en_ch_selector ), 	//temp
	.final_number(	fnum_ch_selector	),			//temp
    .cnt_q ( ch_selector )			// ch 0~7 choose
);



count_yi_v3 #(
    .BITS_OF_END_NUMBER( 10  ) 
)cnt_14(
    .clk ( clk ),
    .reset ( cnt_rst ), 
    .enable ( en_col_part ), 	//temp
	.final_number(	fnum_col_part	),			//temp
    .cnt_q ( col_part )			// col part
);

wire [9:0]fnum_col_part;
assign fnum_col_part = (cvtr_mode8 ) ? 			10'd208		: 
							(cvtr_mode9 ) ? 	10'd104		: 
							(cvtr_mode10) ?		10'd52		:
							(cvtr_mode11) ?		10'd26		:
							(cvtr_mode12) ?		10'd13		:
												10'd32		;

count_yi_v3 #(
    .BITS_OF_END_NUMBER( 10  ) 
)cnt_12(
    .clk ( clk ),
    .reset ( cnt_rst ), 
    .enable ( en_ch_part ), 	//temp
	.final_number(	cfg_ch_part_num 	),			//temp
    .cnt_q ( ch_part )			// ch part 
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


//---- sram soz addr counter ----
wire [ 10:0 ]	fnum_cnt_soz1_addr;	// final number of cnt_soz1_addr
assign fnum_cnt_soz1_addr = (cvtr_mode8 ) ? 11'd1024: 11'd2048 ; 
count_yi_v3 #(
    .BITS_OF_END_NUMBER( 11  ) 
)cnt_13(
    .clk ( clk ),
    .reset ( cnt_rst ), 
    .enable ( en_sort_addr ), 	//temp
	.final_number(	'd2048	),			//temp
    .cnt_q ( cnt_soz1_addr )			// temp
);

//---- soz stage counter ----
wire [ 5:0 ]	fnum_soz_state_cnt , soz_state_cnt;	// final number of soz state counter
assign fnum_soz_state_cnt = (cvtr_mode4 ) ? 	6'd27: 
								(cvtr_mode5 ) ? 	6'd25: 
									(cvtr_mode6 ) ? 	6'd20: 
										(cvtr_mode7 ) ? 	6'd64: 
											(cvtr_mode8 ) ? 	6'd32	: 
											(cvtr_mode9 ) ? 	6'd16	: 
											(cvtr_mode10 ) ? 	6'd8	: 
											(cvtr_mode11 ) ? 	6'd4	:
											(cvtr_mode12 ) ? 	6'd2	:
																	6'd1	;

reg en_soz_state_cnt ;

always@(*)begin
	if( en_sort_data )begin
		case( cfg_mode[3-:4] )
			4'd8 : begin 
				if( col_in_trans == 'd7 )begin
					en_soz_state_cnt = 1'd1 ;
				end
				else begin
					en_soz_state_cnt = 1'd0 ;
				end
			end
			4'd9 , 4'd10 , 4'd11 , 4'd12 : begin 
				if( col_in_trans == 'd7 )begin
					en_soz_state_cnt = 1'd1 ;
				end
				else begin
					en_soz_state_cnt = 1'd0 ;
				end
			end
			default : begin
				en_soz_state_cnt = en_sort_data ;
			end
		endcase

	end 
	else begin
		en_soz_state_cnt = 1'd0 ;
	end

end

count_yi_v3 #(
    .BITS_OF_END_NUMBER( 6  ) 
)cnt_20(
    .clk ( clk ),
    .reset ( cnt_rst ), 
    .enable ( en_soz_state_cnt ), 
	.final_number(	fnum_soz_state_cnt	),
    .cnt_q ( soz_state_cnt )			// temp ending
);


// ---- SORT_state : sto address generator----
assign multi_ch_selec 	= ch_selector * cfg_rowbase_ch_step ;		// use config
assign multi_ch_part 	= ch_part * cfg_rowbase_chpart_step ;		// use config
assign sto_addr_temp =  (cvtr_mode1 | cvtr_mode2 |cvtr_mode8 |cvtr_mode9 | cvtr_mode10 | cvtr_mode11 | cvtr_mode12 )?			multi_ch_selec + multi_ch_part + col_part : 
							( cvtr_mode3 | cvtr_mode7 )?					multi_ch_part 	+ col_in_trans :
							( cvtr_mode4 | cvtr_mode5 )?			multi_ch_part 	+ col_in_trans :
							( cvtr_mode6 )?					multi_ch_part 	+ col_in_trans + multi_ch_selec: 
															10'd0	;
//---- SORT counter enable control ----
always@( * )begin
	case( cfg_mode[3-:4] )
		4'd1 , 4'd2 : begin
			en_ch_selector 	= 	(en_sort_data)?		1'd1	: 1'd0;
			en_ch_part		=	( en_sort_data & ( ch_selector == 'd7 ) )	?	1'd1	: 1'd0;
			en_sram_choo	=	( en_sort_data & ( ch_selector == 'd7 ) & (ch_part == cfg_ch_part_num - 'd1 ) )	?	1'd1	: 1'd0;
			en_col_in_trans	=	( en_sort_data & ( ch_selector == 'd7 ) & (ch_part == cfg_ch_part_num - 'd1 ) & (sram_choo == 'd1 ) )	?	1'd1	: 1'd0;
			en_col_part		=	( en_sort_data & ( ch_selector == 'd7 ) & (ch_part == cfg_ch_part_num - 'd1 ) & (sram_choo == 'd1 ) & ( col_in_trans == 7))	?	1'd1	: 1'd0;
		end
		
		4'd3 , 4'd7 : begin
			en_ch_selector 	= 	1'd0 ;
			en_ch_part 		= 	(en_sort_data & ( col_in_trans == cfg_fcol_in_trans - 'd1 ))?		1'd1	: 1'd0; 	 	
			en_sram_choo 	= 	(en_sort_data & ( col_in_trans == cfg_fcol_in_trans - 'd1 ) & (ch_part == cfg_ch_part_num - 'd1) )?		1'd1	: 1'd0;
			en_col_in_trans = 	(en_sort_data)?		1'd1	: 1'd0; 	
			en_col_part		= 	1'd0 ;
		end

		4'd4:begin			// row major size 13 , soz_state_cnt 0~26
			en_ch_selector 	= 	1'd0 ;
			en_col_part		= 	1'd0 ;
			en_ch_part 		= 	(en_sort_data & ( col_in_trans == cfg_fcol_in_trans - 'd1 ))?		1'd1	: 1'd0; 	 	
			en_sram_choo 	= 	(en_sort_data & ( col_in_trans == cfg_fcol_in_trans - 'd1 ) & (ch_part == cfg_ch_part_num - 'd1) )?		1'd1	: 1'd0;
			if( en_sort_data )begin
				case( soz_state_cnt )	// check enable setting on excel
					6'd0	: en_col_in_trans =	1'd1	;
					6'd1	: en_col_in_trans =	1'd1	;		
					6'd2	: en_col_in_trans =	1'd0	;	
					6'd3	: en_col_in_trans =	1'd1	;
					6'd4	: en_col_in_trans =	1'd0	;
					6'd5	: en_col_in_trans =	1'd1	;		
					6'd6	: en_col_in_trans =	1'd0	;
					6'd7	: en_col_in_trans =	1'd1	;		// data has not filled in 64 bits
					6'd8	: en_col_in_trans =	1'd1	;		// data has not filled in 64 bits	//channel end	
					6'd9	: en_col_in_trans =	1'd0	;	
					6'd10	: en_col_in_trans =	1'd1	;	
					6'd11	: en_col_in_trans =	1'd0	;	
					6'd12	: en_col_in_trans =	1'd1	;	
					6'd13	: en_col_in_trans =	1'd0	;
					6'd14	: en_col_in_trans =	1'd1	;
					6'd15	: en_col_in_trans =	1'd0	;
					6'd16	: en_col_in_trans =	1'd1	;		
					6'd17	: en_col_in_trans =	1'd0	;
					6'd18	: en_col_in_trans =	1'd1	;	// data has not filled in 64 bits
					6'd19	: en_col_in_trans =	1'd1	;	// data has not filled in 64 bits	
					6'd20	: en_col_in_trans =	1'd0	;
					6'd21	: en_col_in_trans =	1'd1	;
					6'd22	: en_col_in_trans =	1'd0	;
					6'd23	: en_col_in_trans =	1'd1	;		
					6'd24	: en_col_in_trans =	1'd0	;
					6'd25	: en_col_in_trans =	1'd1	;
					6'd26	: en_col_in_trans =	1'd1	;		
					
					default : en_col_in_trans =	 1'd0 ;		
				endcase
			end 
			else begin
					en_col_in_trans = 1'd0 ;
			end

		end

		4'd5:begin		// size 26 , soz_state_cnt 0~24
			en_ch_selector 	= 	1'd0 ;
			en_col_part		= 	1'd0 ;
			en_ch_part 		= 	(en_sort_data & ( col_in_trans == cfg_fcol_in_trans - 'd1 ) )?		1'd1	: 1'd0; 	 	
			en_sram_choo 	= 	(en_sort_data & ( col_in_trans == cfg_fcol_in_trans - 'd1 ) & (ch_part == cfg_ch_part_num - 'd1) )?		1'd1	: 1'd0;
			if( en_sort_data )begin
				case( soz_state_cnt )	
					6'd0	: en_col_in_trans =	1'd1	;
					6'd1	: en_col_in_trans =	1'd1	;		
					6'd2	: en_col_in_trans =	1'd1	;	
					6'd3	: en_col_in_trans =	1'd1	;
					6'd4	: en_col_in_trans =	1'd0	;
					6'd5	: en_col_in_trans =	1'd1	;		
					6'd6	: en_col_in_trans =	1'd0	;
					6'd7	: en_col_in_trans =	1'd1	;		// data has not filled in 64 bits
					6'd8	: en_col_in_trans =	1'd0	;		// data has not filled in 64 bits	//channel end	
					6'd9	: en_col_in_trans =	1'd1	;	
					6'd10	: en_col_in_trans =	1'd1	;	
					6'd11	: en_col_in_trans =	1'd0	;	
					6'd12	: en_col_in_trans =	1'd1	;	
					6'd13	: en_col_in_trans =	1'd0	;
					6'd14	: en_col_in_trans =	1'd1	;
					6'd15	: en_col_in_trans =	1'd0	;
					6'd16	: en_col_in_trans =	1'd1	;		
					6'd17	: en_col_in_trans =	1'd1	;
					6'd18	: en_col_in_trans =	1'd0	;	// data has not filled in 64 bits
					6'd19	: en_col_in_trans =	1'd1	;	// data has not filled in 64 bits	
					6'd20	: en_col_in_trans =	1'd0	;
					6'd21	: en_col_in_trans =	1'd1	;
					6'd22	: en_col_in_trans =	1'd0	;
					6'd23	: en_col_in_trans =	1'd1	;		
					6'd24	: en_col_in_trans =	1'd1	;
			
					default : en_col_in_trans =	 1'd0 ;		
				endcase
			end 
			else begin
					en_col_in_trans = 1'd0 ;
			end

		end

		4'd6:begin		// for size 52 , soz_state_cnt 0~19
			en_ch_selector 	= 	(en_sort_data & ( col_in_trans == cfg_fcol_in_trans - 'd1 ))?		1'd1	: 1'd0; 
			en_col_part		= 	1'd0 ;
			en_ch_part 		= 	(en_sort_data & ( col_in_trans == cfg_fcol_in_trans - 'd1 ) & ( ch_selector == 5'd1 ))?		1'd1	: 1'd0; 	 	
			en_sram_choo 	= 	(en_sort_data & ( col_in_trans == cfg_fcol_in_trans - 'd1 ) & (ch_part == cfg_ch_part_num - 'd1) & ( ch_selector == 5'd1 ))?		1'd1	: 1'd0;
			if( en_sort_data )begin
				case( soz_state_cnt )	
					6'd0	: en_col_in_trans =	1'd1	;
					6'd1	: en_col_in_trans =	1'd1	;		
					6'd2	: en_col_in_trans =	1'd1	;	
					6'd3	: en_col_in_trans =	1'd1	;
					6'd4	: en_col_in_trans =	1'd1	;
					6'd5	: en_col_in_trans =	1'd1	;		
					6'd6	: en_col_in_trans =	1'd1	;
					6'd7	: en_col_in_trans =	1'd0	;		// data has not filled in 64 bits
					6'd8	: en_col_in_trans =	1'd1	;		// data has not filled in 64 bits	//channel end	
					6'd9	: en_col_in_trans =	1'd0	;	
					6'd10	: en_col_in_trans =	1'd1	;	
					6'd11	: en_col_in_trans =	1'd0	;	
					6'd12	: en_col_in_trans =	1'd1	;	
					6'd13	: en_col_in_trans =	1'd0	;
					6'd14	: en_col_in_trans =	1'd1	;
					6'd15	: en_col_in_trans =	1'd0	;
					6'd16	: en_col_in_trans =	1'd1	;		
					6'd17	: en_col_in_trans =	1'd0	;
					6'd18	: en_col_in_trans =	1'd1	;	// data has not filled in 64 bits
					6'd19	: en_col_in_trans =	1'd1	;	// data has not filled in 64 bits	
					
					default : en_col_in_trans =	 1'd0 ;		
				endcase
			end 
			else begin
					en_col_in_trans = 1'd0 ;
			end

		end

		4'd8	: begin

			en_col_in_trans	=	( en_sort_data )?		1'd1	: 1'd0	;
				en_col_part		= 	( en_sort_data &  (soz_state_cnt <= 6'd25) )?		1'd1	: 1'd0	;
				en_ch_selector 	= 	( en_sort_data & ( col_part == fnum_col_part - 'd1))?		1'd1	: 1'd0	;
				en_ch_part		=	( en_sort_data & ( col_part == fnum_col_part - 'd1) & ( ch_selector == 'd7 ))?		1'd1	: 1'd0	;
				en_sram_choo	=	( en_sort_data & ( col_part == fnum_col_part - 'd1) & ( ch_selector == 'd7 ) & (ch_part == cfg_ch_part_num - 'd1 )  )	?	1'd1	: 1'd0;
			
		end
		4'd9	: begin
			en_col_in_trans	=	( en_sort_data )?		1'd1	: 1'd0	;
				en_col_part		= 	( en_sort_data &  (soz_state_cnt <= 6'd12) )?		1'd1	: 1'd0	;
				en_ch_selector 	= 	( en_sort_data & ( col_part == fnum_col_part - 'd1))?		1'd1	: 1'd0	;
				en_ch_part		=	( en_sort_data & ( col_part == fnum_col_part - 'd1) & ( ch_selector == 'd7 ))?		1'd1	: 1'd0	;
				en_sram_choo	=	( en_sort_data & ( col_part == fnum_col_part - 'd1) & ( ch_selector == 'd7 ) & (ch_part == cfg_ch_part_num - 'd1 )  )	?	1'd1	: 1'd0;
		end
		4'd10	: begin
			en_col_in_trans	=	( en_sort_data )?		1'd1	: 1'd0	;
				
				en_ch_selector 	= 	( en_sort_data & ( col_part == fnum_col_part - 'd1))?		1'd1	: 1'd0	;
				en_ch_part		=	( en_sort_data & ( col_part == fnum_col_part - 'd1) & ( ch_selector == 'd7 ))?		1'd1	: 1'd0	;
				en_sram_choo	=	( en_sort_data & ( col_part == fnum_col_part - 'd1) & ( ch_selector == 'd7 ) & (ch_part == cfg_ch_part_num - 'd1 )  )	?	1'd1	: 1'd0;

				//en_col_part		= 	( en_sort_data &  (soz_state_cnt <= 6'd6) & (col_in_trans <= 10'd3 ))?		1'd1	: 1'd0	;
				if(en_sort_data )begin
					if( soz_state_cnt <= 6'd5)begin
						en_col_part = 1'd1 ;
					end
					else if( soz_state_cnt == 6'd6) begin
						if( col_in_trans <= 10'd3) begin
							en_col_part = 1'd1 ;
						end
						else begin
							en_col_part = 1'd0 ;
						end
					end
					else begin
						en_col_part = 1'd0 ;
					end
				end 
				else begin
					en_col_part = 1'd0 ;
				end
		end
		4'd11	: begin
				en_col_in_trans	=	( en_sort_data )?		1'd1	: 1'd0	;
				en_ch_selector 	= 	( en_sort_data & ( col_part == fnum_col_part - 'd1))?		1'd1	: 1'd0	;
				en_ch_part		=	( en_sort_data & ( col_part == fnum_col_part - 'd1) & ( ch_selector == 'd7 ))?		1'd1	: 1'd0	;
				en_sram_choo	=	( en_sort_data & ( col_part == fnum_col_part - 'd1) & ( ch_selector == 'd7 ) & (ch_part == cfg_ch_part_num - 'd1 )  )	?	1'd1	: 1'd0;
				
				if(en_sort_data )begin
					if( soz_state_cnt <= 6'd2)begin
						en_col_part = 1'd1 ;
					end
					else if( soz_state_cnt == 6'd3) begin
						if( col_in_trans <= 10'd1) begin
							en_col_part = 1'd1 ;
						end
						else begin
							en_col_part = 1'd0 ;
						end
					end
					else begin
						en_col_part = 1'd0 ;
					end
				end 
				else begin
					en_col_part = 1'd0 ;
				end
		end
		4'd12	: begin
				en_col_in_trans	=	( en_sort_data )?		1'd1	: 1'd0	;
				en_ch_selector 	= 	( en_sort_data & ( col_part == fnum_col_part - 'd1))?		1'd1	: 1'd0	;
				en_ch_part		=	( en_sort_data & ( col_part == fnum_col_part - 'd1) & ( ch_selector == 'd7 ))?		1'd1	: 1'd0	;
				en_sram_choo	=	( en_sort_data & ( col_part == fnum_col_part - 'd1) & ( ch_selector == 'd7 ) & (ch_part == cfg_ch_part_num - 'd1 )  )	?	1'd1	: 1'd0;
				
				if(en_sort_data )begin
					if( soz_state_cnt == 6'd0) begin
						en_col_part = 1'd1 ;
					end
					else if( soz_state_cnt == 6'd1) begin
						if( col_in_trans <= 10'd4) begin
							en_col_part = 1'd1 ;
						end
						else begin
							en_col_part = 1'd0 ;
						end
					end
					else begin
						en_col_part = 1'd0 ;
					end
				end 
				else begin
					en_col_part = 1'd0 ;
				end
		end
		default:  begin 
			en_ch_selector 	= 1'd0 ;	 
			en_ch_part 		= 1'd0 ; 	 	
			en_col_in_trans = 1'd0 ;
			en_col_part		= 1'd0 ;
			en_sram_choo 	= 1'd0 ;

			end

	endcase

end

//---- soz address number counter enable ----
always@(*) begin
	case( cfg_mode[3-:4] )
		4'd1: begin
			en_sort_addr = ( en_sort_data &  ( dly1_ch_selector == 'd7) )	?	1'd1 : 1'd0 ;
		end
		4'd2 : begin
			en_sort_addr = ( en_sort_data &  ( dly1_ch_selector == 'd7) )	?	1'd1 : 1'd0 ;
		end
		4'd3 , 4'd7 : begin
			en_sort_addr = (dly1_en_sort_data  )	?	1'd1 : 1'd0 ;
		end
		4'd4:begin
			if( dly1_en_sort_data )begin
				case( dly1_soz_state_cnt )	
					6'd0	: en_sort_addr =	1'd1	;
					6'd1	: en_sort_addr =	1'd0	;		
					6'd2	: en_sort_addr =	1'd1	;	
					6'd3	: en_sort_addr =	1'd0	;
					6'd4	: en_sort_addr =	1'd1	;
					6'd5	: en_sort_addr =	1'd0	;		
					6'd6	: en_sort_addr =	1'd1	;
					6'd7	: en_sort_addr =	1'd0	;		// data has not filled in 64 bits
					6'd8	: en_sort_addr =	1'd0	;		// data has not filled in 64 bits	//channel end	
					6'd9	: en_sort_addr =	1'd1	;	
					6'd10	: en_sort_addr =	1'd0	;	
					6'd11	: en_sort_addr =	1'd1	;	
					6'd12	: en_sort_addr =	1'd0	;	
					6'd13	: en_sort_addr =	1'd1	;
					6'd14	: en_sort_addr =	1'd0	;
					6'd15	: en_sort_addr =	1'd1	;
					6'd16	: en_sort_addr =	1'd0	;		
					6'd17	: en_sort_addr =	1'd1	;
					6'd18	: en_sort_addr =	1'd0	;	// data has not filled in 64 bits
					6'd19	: en_sort_addr =	1'd0	;	// data has not filled in 64 bits	
					6'd20	: en_sort_addr =	1'd1	;
					6'd21	: en_sort_addr =	1'd0	;
					6'd22	: en_sort_addr =	1'd1	;
					6'd23	: en_sort_addr =	1'd0	;		
					6'd24	: en_sort_addr =	1'd1	;
					6'd25	: en_sort_addr =	1'd0	;
					6'd26	: en_sort_addr =	1'd1	;		
					
					default : en_sort_addr =	 1'd0 ;		
				endcase
			end 
			else begin
					en_sort_addr = 1'd0 ;
			end
			

		end

		4'd5:begin		//size26
			if( dly1_en_sort_data )begin
				case( dly1_soz_state_cnt )	
					6'd0	: en_sort_addr =	1'd1	;
					6'd1	: en_sort_addr =	1'd1	;		
					6'd2	: en_sort_addr =	1'd1	;	
					6'd3	: en_sort_addr =	1'd0	;
					6'd4	: en_sort_addr =	1'd1	;
					6'd5	: en_sort_addr =	1'd0	;		
					6'd6	: en_sort_addr =	1'd1	;
					6'd7	: en_sort_addr =	1'd0	;		
					6'd8	: en_sort_addr =	1'd1	;		
					6'd9	: en_sort_addr =	1'd0	;	// data has not filled in 64 bits
					6'd10	: en_sort_addr =	1'd0	;	// data has not filled in 64 bits	//channel end	
					6'd11	: en_sort_addr =	1'd1	;	
					6'd12	: en_sort_addr =	1'd0	;	
					6'd13	: en_sort_addr =	1'd1	;
					6'd14	: en_sort_addr =	1'd0	;
					6'd15	: en_sort_addr =	1'd1	;
					6'd16	: en_sort_addr =	1'd0	;	// data has not filled in 64 bits	
					6'd17	: en_sort_addr =	1'd0	;	// data has not filled in 64 bits
					6'd18	: en_sort_addr =	1'd1	;	
					6'd19	: en_sort_addr =	1'd0	;		
					6'd20	: en_sort_addr =	1'd1	;
					6'd21	: en_sort_addr =	1'd0	;
					6'd22	: en_sort_addr =	1'd1	;
					6'd23	: en_sort_addr =	1'd0	;		
					6'd24	: en_sort_addr =	1'd1	;	
					default : en_sort_addr =	 1'd0 ;		
				endcase
			end 
			else begin
					en_sort_addr = 1'd0 ;
			end
			

		end

		4'd6:begin			// size52
			if( dly1_en_sort_data )begin
				case( dly1_soz_state_cnt )	
					6'd0	: en_sort_addr =	1'd1	;
					6'd1	: en_sort_addr =	1'd1	;		
					6'd2	: en_sort_addr =	1'd1	;	
					6'd3	: en_sort_addr =	1'd1	;
					6'd4	: en_sort_addr =	1'd1	;
					6'd5	: en_sort_addr =	1'd1	;		
					6'd6	: en_sort_addr =	1'd0	;
					6'd7	: en_sort_addr =	1'd1	;	
					6'd8	: en_sort_addr =	1'd0	;	
					6'd9	: en_sort_addr =	1'd1	;	
					6'd10	: en_sort_addr =	1'd0	;	
					6'd11	: en_sort_addr =	1'd1	;	
					6'd12	: en_sort_addr =	1'd0	;	
					6'd13	: en_sort_addr =	1'd1	;
					6'd14	: en_sort_addr =	1'd0	;
					6'd15	: en_sort_addr =	1'd1	;
					6'd16	: en_sort_addr =	1'd0	;		
					6'd17	: en_sort_addr =	1'd1	;
					6'd18	: en_sort_addr =	1'd0	;	
					6'd19	: en_sort_addr =	1'd1	;		
					default : en_sort_addr =	 1'd0 ;		
				endcase
			end 
			else begin
					en_sort_addr = 1'd0 ;
			end
			
		end
		
		4'd8 ,4'd9,4'd10,4'd11,4'd12 : begin
			en_sort_addr = ( en_sort_data &  ( dly1_col_in_trans == 'd7) )	?	1'd1 : 1'd0 ;
		end
		default: en_sort_addr = 1'd0 ;

	endcase

end

//---- soz wea generator ----
always@( * )begin
	case( cfg_mode[3 -: 4] )
		4'd1,4'd2:begin
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
		4'd8,4'd9,4'd10,4'd11,4'd12:begin
				case( dly1_col_in_trans )
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
		4'd4:begin
				case( dly1_soz_state_cnt )	
					6'd9	,6'd11	: wea_soz1_choo = 	8'b0000_0001		;
					6'd20	,6'd22	: wea_soz1_choo = 	8'b0000_0011		;
					6'd2	,6'd4	: wea_soz1_choo = 	8'b0000_0111		;
					6'd13	,6'd15	: wea_soz1_choo = 	8'b0000_1111		;
					6'd24	,6'd26	: wea_soz1_choo = 	8'b0001_1111		;
					6'd8			: wea_soz1_choo =	8'b0011_1110		;
					6'd6			: wea_soz1_choo =	8'b0011_1111		;
					6'd19			: wea_soz1_choo = 	8'b0111_1100		;
					6'd17			: wea_soz1_choo = 	8'b0111_1111		;
					6'd16	,6'd18	: wea_soz1_choo = 	8'b1000_0000		;
					6'd23	,6'd25	: wea_soz1_choo = 	8'b1110_0000		;
					6'd5	,6'd7	: wea_soz1_choo = 	8'b1100_0000		;
					6'd23	,6'd25	: wea_soz1_choo = 	8'b1110_0000		;
					6'd12	,6'd14	: wea_soz1_choo = 	8'b1111_0000		;
					6'd1	,6'd3	: wea_soz1_choo = 	8'b1111_1000		;
					6'd21			: wea_soz1_choo = 	8'b1111_1100		;
					6'd10			: wea_soz1_choo = 	8'b1111_1110		;
					6'd0			: wea_soz1_choo = 	8'b1111_1111		;
					default : wea_soz1_choo = 8'hff ;
				endcase
			end
		4'd5:begin
				case( dly1_soz_state_cnt )	
					6'd3	: wea_soz1_choo = 	8'b1100_0000		;
					6'd4	: wea_soz1_choo =	8'b0011_1111		;
					6'd5	: wea_soz1_choo =	8'b1100_0000		;
					6'd6	: wea_soz1_choo = 	8'b0011_1111		;
					6'd7	: wea_soz1_choo = 	8'b1100_0000		;
					6'd8	: wea_soz1_choo =	8'b0011_1111		;
					6'd9	: wea_soz1_choo =	8'b1100_0000		;
					6'd10	: wea_soz1_choo = 	8'b0011_0000		;
					6'd11	: wea_soz1_choo = 	8'b0000_1111		;
					6'd12	: wea_soz1_choo = 	8'b1111_0000		;
					6'd13	: wea_soz1_choo = 	8'b0000_1111		;
					6'd14	: wea_soz1_choo = 	8'b1111_0000		;
					6'd15	: wea_soz1_choo = 	8'b0000_1111		;
					6'd16	: wea_soz1_choo = 	8'b1111_0000		;
					6'd17	: wea_soz1_choo = 	8'b0000_1100		;
					6'd18	: wea_soz1_choo = 	8'b0000_0011		;
					6'd19	: wea_soz1_choo = 	8'b1111_1100		;
					6'd20	: wea_soz1_choo = 	8'b0000_0011		;
					6'd21	: wea_soz1_choo = 	8'b1111_1100		;
					6'd22	: wea_soz1_choo = 	8'b0000_0011		;
					6'd23	: wea_soz1_choo = 	8'b1111_1100		;
					6'd24	: wea_soz1_choo = 	8'b0000_0011		;
					default : wea_soz1_choo = 8'hff ;		// include 0,1,2
				endcase
			end
		4'd6:begin
				case( dly1_soz_state_cnt )	
					6'd6	: wea_soz1_choo = 	8'b1111_0000		;
					6'd7	: wea_soz1_choo = 	8'b0000_1111		;
					6'd8	: wea_soz1_choo =	8'b1111_0000		;
					6'd9	: wea_soz1_choo =	8'b0000_1111		;
					6'd10	: wea_soz1_choo = 	8'b1111_0000		;
					6'd11	: wea_soz1_choo = 	8'b0000_1111		;
					6'd12	: wea_soz1_choo = 	8'b1111_0000		;
					6'd13	: wea_soz1_choo = 	8'b0000_1111		;
					6'd14	: wea_soz1_choo = 	8'b1111_0000		;
					6'd15	: wea_soz1_choo = 	8'b0000_1111		;
					6'd16	: wea_soz1_choo = 	8'b1111_0000		;
					6'd17	: wea_soz1_choo = 	8'b0000_1111		;
					6'd18	: wea_soz1_choo = 	8'b1111_0000		;
					6'd19	: wea_soz1_choo = 	8'b0000_1111		;

					default : wea_soz1_choo = 8'hff ;		// include 0,1,2,3,4,5
				endcase
			end
		default : begin		// include mode 3 , 7
			wea_soz1_choo = 8'hff ;
		end

	endcase
end


//----------------------------------------------------------------------- 
//---- sto dout chooser ----
//	description : state done signal generate
//				and choose which sram data is needed. 
//				data position should be changed by following logic.
//----------------------------------------------------------------------- 
assign row1_sort_done = ( cvtr_mode3 | cvtr_mode5 | cvtr_mode6 | cvtr_mode7 )						?	cnt_soz1_addr == 'd2047  :
							( cvtr_mode4 )															? 	(cnt_soz1_addr == 'd1023 ) :
								( cvtr_mode2 )														? 	( (cnt_soz1_addr == 'd1023 ) & ( dly1_ch_selector == 3'd7 )) :
									( cvtr_mode8 | cvtr_mode9 | cvtr_mode10 | cvtr_mode11 )			? 	( (cnt_soz1_addr == 'd1023 ) & ( dly1_col_in_trans == 3'd7 )) :
										( cvtr_mode12 )												?	( (cnt_soz1_addr == 'd511 ) & ( dly1_col_in_trans == 3'd7 )) :
																		( (cnt_soz1_addr == 'd2047 ) & ( dly1_ch_selector == 3'd7 ))	;

assign sram_sto_choose = (  (dly1_sram_choo == 10'd0) 	)?	 	dout_sram_sto1	:
							(  (dly1_sram_choo == 10'd1)	)?		dout_sram_sto2	:
																					64'hffff_ffff_ffff_ffff	;


always@( * )begin
	case( cfg_mode[3 -: 4] )
	4'd8 , 4'd9 , 4'd10 , 4'd11 , 4'd12 : begin
			case( dly1_ch_selector )
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
	default :begin
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

	endcase
end
//--- generate 64bits data structure

always@( * )begin
	case( cfg_mode[3 -: 4] )
		4'd1 , 
		4'd2 :begin
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

		4'd3 ,4'd7 :begin
			re_size_reg = sram_sto_choose ;
			end

		4'd4:begin
			case( dly1_soz_state_cnt )	
				6'd0	: re_size_reg =		sram_sto_choose		;
				6'd1	: re_size_reg =		{	sram_sto_choose[63 -:40]	,		24'h000						};	//channel end	
				6'd2	: re_size_reg =		{		40'h0_0000				,	sram_sto_choose[63 -:24]		};
				6'd3	: re_size_reg =		{	sram_sto_choose[39 -:40]	,		24'h000						};
				6'd4	: re_size_reg =		{		40'h0_0000				,	sram_sto_choose[63 -:24]		};
				6'd5	: re_size_reg =		{	sram_sto_choose[39 -:16]	,		48'h00_0000					};	//channel end	
				6'd6	: re_size_reg =		{	16'h00						,	sram_sto_choose[63 -:48]		};
				6'd7	: re_size_reg =		{	sram_sto_choose[15 -:16]	,		48'h00_0000					};
				6'd8	: re_size_reg =		{	16'h00						,	sram_sto_choose[63 -:40]	, 8'd0 };	//channel end	
				6'd9	: re_size_reg =		{ 		56'h000_0000			,	sram_sto_choose[63 -:8]			};	
				6'd10	: re_size_reg =		{ 	sram_sto_choose[55 -:56]	,	8'd0 							};	
				6'd11	: re_size_reg =		{		56'h000_0000			,	sram_sto_choose[63 -:8]			};	
				6'd12	: re_size_reg =		{	sram_sto_choose[55 -:32]	,		32'h0000					};	//channel end	
				6'd13	: re_size_reg =		{ 		32'h0000				,	sram_sto_choose[63 -:32]		};
				6'd14	: re_size_reg =		{ 	sram_sto_choose[31 -:32]	,		32'h0000					};
				6'd15	: re_size_reg =		{		32'h0000				,	sram_sto_choose[63 -:32]		};
				6'd16	: re_size_reg =		{	sram_sto_choose[31 -:8]		,		56'h000_0000				};	//channel end	
				6'd17	: re_size_reg =		{	8'd0						,	sram_sto_choose[63 -:56]		};
				6'd18	: re_size_reg =		{	sram_sto_choose[7 -:8]		,		56'h000_0000				};
				6'd19	: re_size_reg =		{	8'd0						,	sram_sto_choose[63 -:40]	,16'h00 };	//channel end	
				6'd20	: re_size_reg =		{		48'h00_0000				,	sram_sto_choose[63 -:16]		};
				6'd21	: re_size_reg =		{	sram_sto_choose[47 -:48]	,	16'h00							};
				6'd22	: re_size_reg =		{		48'h00_0000				,	sram_sto_choose[63 -:16]		};
				6'd23	: re_size_reg =		{	sram_sto_choose[47 -:24]	,		40'h0_0000					};	//channel end	
				6'd24	: re_size_reg =		{		24'h000					,	sram_sto_choose[63 -:40]	 	};
				6'd25	: re_size_reg =		{ 	sram_sto_choose[23 -:24]	,		40'h0_0000					};
				6'd26	: re_size_reg =		{		24'h000					,	sram_sto_choose[63 -:40]	 	};	//channel end	
				
				default : re_size_reg =	 64'hffff_ffff_ffff_ffff ;		// include 0,1,2
			endcase
		end	

		4'd5:begin
			case( dly1_soz_state_cnt )	
				6'd0	: re_size_reg =		sram_sto_choose		;
				6'd1	: re_size_reg =		sram_sto_choose		;
				6'd2	: re_size_reg =		sram_sto_choose		;
				6'd3	: re_size_reg =		{			sram_sto_choose[63 -:16]	,	48'd0	};
				6'd4	: re_size_reg =		{ 16'd0	,	sram_sto_choose[63 -:48]		};
				6'd5	: re_size_reg =		{			sram_sto_choose[15 -:16]	,	48'd0	};
				6'd6	: re_size_reg =		{ 16'd0	,	sram_sto_choose[63 -:48]		};
				6'd7	: re_size_reg =		{			sram_sto_choose[15 -:16]	,	48'd0	};
				6'd8	: re_size_reg =		{ 16'd0	,	sram_sto_choose[63 -:48]		};
				6'd9	: re_size_reg =		{			sram_sto_choose[15 -:16]	,	48'd0	};	
				6'd10	: re_size_reg =		{ 16'd0	,	sram_sto_choose[63 -:16]	,	32'd0	};	//chend
				6'd11	: re_size_reg =		{ 32'd0	,	sram_sto_choose[63 -:32]		};
				6'd12	: re_size_reg =		{ 			sram_sto_choose[31 -:32]	,	32'd0	};
				6'd13	: re_size_reg =		{ 32'd0	,	sram_sto_choose[63 -:32]		};
				6'd14	: re_size_reg =		{ 			sram_sto_choose[31 -:32]	,	32'd0	};
				6'd15	: re_size_reg =		{ 32'd0	,	sram_sto_choose[63 -:32]		};
				6'd16	: re_size_reg =		{ 			sram_sto_choose[31 -:32]	,	32'd0	};
				6'd17	: re_size_reg =		{ 32'd0	,	sram_sto_choose[63 -:16]	,	16'd0	};
				6'd18	: re_size_reg =		{ 48'd0	,	sram_sto_choose[63 -:16]	 	};
				6'd19	: re_size_reg =		{ 			sram_sto_choose[47 -:48]	,	16'd0	};
				6'd20	: re_size_reg =		{ 48'd0	,	sram_sto_choose[63 -:16]	 	};
				6'd21	: re_size_reg =		{ 			sram_sto_choose[47 -:48]	,	16'd0	};
				6'd22	: re_size_reg =		{ 48'd0	,	sram_sto_choose[63 -:16]	 	};
				6'd23	: re_size_reg =		{ 			sram_sto_choose[47 -:48]	,	16'd0	};
				6'd24	: re_size_reg =		{ 48'd0	,	sram_sto_choose[63 -:16]	 	};
				
				default : re_size_reg =	 64'hffff_ffff_ffff_ffff ;		// include 0,1,2
			endcase
		end	

		4'd6:begin
			case( dly1_soz_state_cnt )	
				6'd0	: re_size_reg =		sram_sto_choose		;
				6'd1	: re_size_reg =		sram_sto_choose		;
				6'd2	: re_size_reg =		sram_sto_choose		;
				6'd3	: re_size_reg =		sram_sto_choose		;
				6'd4	: re_size_reg =		sram_sto_choose		;
				6'd5	: re_size_reg =		sram_sto_choose		;
				6'd6	: re_size_reg =		{			sram_sto_choose[63 -:32]	 , 	32'd0 	};
				6'd7	: re_size_reg =		{ 32'd0	,	sram_sto_choose[63 -:32]		 		};
				6'd8	: re_size_reg =		{			sram_sto_choose[31 -:32]	 , 	32'd0 	};
				6'd9	: re_size_reg =		{ 32'd0	,	sram_sto_choose[63 -:32]		 		};
				6'd10	: re_size_reg =		{			sram_sto_choose[31 -:32]	 , 	32'd0 	};
				6'd11	: re_size_reg =		{ 32'd0	,	sram_sto_choose[63 -:32]				};
				6'd12	: re_size_reg =		{			sram_sto_choose[31 -:32]	 , 	32'd0 	};
				6'd13	: re_size_reg =		{ 32'd0	,	sram_sto_choose[63 -:32]				};
				6'd14	: re_size_reg =		{			sram_sto_choose[31 -:32]	 , 	32'd0 	};
				6'd15	: re_size_reg =		{ 32'd0	,	sram_sto_choose[63 -:32]				};
				6'd16	: re_size_reg =		{			sram_sto_choose[31 -:32]	 , 	32'd0 	};
				6'd17	: re_size_reg =		{ 32'd0	,	sram_sto_choose[63 -:32]				};
				6'd18	: re_size_reg =		{			sram_sto_choose[31 -:32]	 , 	32'd0 	};
				6'd19	: re_size_reg =		{ 32'd0	,	sram_sto_choose[63 -:32]				};

				default : re_size_reg =	 64'hffff_ffff_ffff_ffff ;		// include 0,1,2
			endcase
		end

		4'd8 :begin
			if( dly1_soz_state_cnt <= 6'd25 )begin
				case( dly1_col_in_trans )
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
			else begin
				re_size_reg = 64'h0;			
			end
		end
		4'd9 :begin
			if( dly1_soz_state_cnt <= 6'd12 )begin
				case( dly1_col_in_trans )
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
			else begin
				re_size_reg = 64'h0;	
			end
		end
		4'd10 :begin
			if( dly1_soz_state_cnt <= 6'd5 )begin
				case( dly1_col_in_trans )
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
			else if( dly1_soz_state_cnt == 6'd6)begin
				case( dly1_col_in_trans )
					3'd0: re_size_reg = { 			col_choo_reg	 , 		56'd0 	};
					3'd1: re_size_reg = { 8'd0 , 	col_choo_reg	 ,		48'd0	};
					3'd2: re_size_reg = { 16'd0 , 	col_choo_reg	 ,		40'd0	};
					3'd3: re_size_reg = { 24'd0 , 	col_choo_reg	 ,		32'd0	};
					3'd4: re_size_reg = 64'h0 ;
					3'd5: re_size_reg = 64'h0 ;
					3'd6: re_size_reg = 64'h0 ;
					3'd7: re_size_reg = 64'h0 ;
					default : re_size_reg = 64'hffff_ffff_ffff_ffff ;
				endcase			
			end
			else begin
				re_size_reg = 64'h0 ;
			end
		end
		4'd11 :begin
			if( dly1_soz_state_cnt <= 6'd2 )begin
				case( dly1_col_in_trans )
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
			else if( dly1_soz_state_cnt == 6'd3)begin
				case( dly1_col_in_trans )
					3'd0: re_size_reg = { 			col_choo_reg	 , 		56'd0 	};
					3'd1: re_size_reg = { 8'd0 , 	col_choo_reg	 ,		48'd0	};
					3'd2: re_size_reg = 64'h0 ;
					3'd3: re_size_reg = 64'h0 ;
					3'd4: re_size_reg = 64'h0 ;
					3'd5: re_size_reg = 64'h0 ;
					3'd6: re_size_reg = 64'h0 ;
					3'd7: re_size_reg = 64'h0 ;
					default : re_size_reg = 64'hffff_ffff_ffff_ffff ;
				endcase			
			end
			else begin
				re_size_reg = 64'h0 ;
			end
		end
		4'd12 :begin
			if( dly1_soz_state_cnt == 6'd0 )begin
				case( dly1_col_in_trans )
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
			else if( dly1_soz_state_cnt == 6'd1)begin
				case( dly1_col_in_trans )
					3'd0: re_size_reg = { 			col_choo_reg	 , 		56'd0 	};
					3'd1: re_size_reg = { 8'd0 , 	col_choo_reg	 ,		48'd0	};
					3'd2: re_size_reg = { 16'd0 , 	col_choo_reg	 ,		40'd0	};
					3'd3: re_size_reg = { 24'd0 , 	col_choo_reg	 ,		32'd0	};
					3'd4: re_size_reg = { 32'd0 , 	col_choo_reg	 ,		24'd0	};
					3'd5: re_size_reg = 64'd0;
					3'd6: re_size_reg = 64'd0;
					3'd7: re_size_reg = 64'd0;
					default : re_size_reg = 64'hffff_ffff_ffff_ffff ;
				endcase			
			end
			else begin
				re_size_reg = 64'h0 ;
			end
		end
		default : begin
			re_size_reg = 64'hffff_ffff_ffff_ffff ;
		end

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

//---- delay soz state count ----
always@(posedge clk) begin
	if( reset )begin
		dly0_soz_state_cnt <= 6'd0;
		dly1_soz_state_cnt <= 6'd0;
	end 
	else begin
		dly0_soz_state_cnt <= soz_state_cnt;
		dly1_soz_state_cnt <= dly0_soz_state_cnt;
	end
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
assign out_addr_num = (cvtr_mode2 | cvtr_mode4 )?	11'd831 : 
							( cvtr_mode7 )?			11'd2047 :
							( cvtr_mode8 | cvtr_mode9 | cvtr_mode10 | cvtr_mode11 )?			11'd1023 :
							( cvtr_mode12 )?			11'd511 :
														11'd1663 ;
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
			if( cnt_output_addr >= out_addr_num )begin
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
	if( cnt_output_addr == out_addr_num ) begin
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