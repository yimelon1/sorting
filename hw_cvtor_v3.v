// ============================================================================
// Ver      : 1.0
// Designer : Yi_Yuan Chen
// Create   : 2021.9.27
// Func     : ifmap_convertor  3x3 to 1x1
// description  : layer2算完的結果 轉成 channel base 作 1x1 
// layer2 ofmap : 256*256*64ch / 8 = 524288
// 3x3 ker_repeat_once : 9/1024/9/32=32  layer2_3x3 output 分兩段 32ch 
// layer 3 1x1 kernel 放的下  依 c code 只要 load 一遍 ker 就好
// ============================================================================
// 2021/09/29
//	without consider the reg or bram size just write and synthesis .
//	data don't pass thought the yolo_hw.
//	當 empty_n =1 read 要盡快等於 1 才能 handshake 
//	當 handshaking 開始就要存取 no.0 data , 得知 read要與 en_store_data 同時開啟
// ============================================================================
//	2021/10/05
//	初筆 DATA 輸出 OK 需要續 SORT AND OUTPUT 
//	使用 多 array 進行 map 存取 


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
	input wire 						din_isif_strb,		//none
	input wire 						din_isif_user,		//none

	output wire						dout_isif_read ,		// convertor wanna read

	// to OUTPUT_STREAM IF
	input wire						din_osif_full_n,

	output wire [TRANS_BITS-1 :0 ]	dout_osif_data ,
	output wire 					dout_osif_strb,		//none
	output wire 					dout_osif_last ,	// the last output data 
	output wire 					dout_osif_user,		//none
	output wire						dout_osif_write 	// convertor wanna write

);


// temp  =====================================================
//	1 rnd = 32ch --> 32ch 1row size = 256*32 Byte = 256*32/8 entries	=1024 entries
//	
//	

reg [18:0] r_addr_soz1 ; // template
parameter IF_BUFFER_ENTRY_SIZE = 1024 ;
parameter IF_ENTRY_SIZE = 2048;   // template    256*256*64/8 = 524288  need 19 bits address
parameter REG_AY_SIZE = 2 ;
parameter BITS_OF_REG_AY_SIZE = 10;
parameter OUTPUT_ENTRIES = 1664 ;	// 208*64/8=1664


reg [ TRANS_BITS-1 : 0 ] ifmap_buffer_array [0 : REG_AY_SIZE -1 ]  [ 0 : IF_BUFFER_ENTRY_SIZE-1 ]  ;


reg [ TRANS_BITS -1 : 0] temp_yi01 ,temp_yi02 ,temp_yi03 ,temp_yi04 ,temp_yi05 ,temp_yi06  ;
/*

*/

//============================================================

//-------  parameter  ------------  parameter  ------  parameter  ------  parameter  ---//

parameter SIZE_OF_ROW 	= 208 ;
parameter SIZE_OF_COL 	= 208 ;
parameter SIZE_OF_CH	= 64 ;
parameter NUM_OF_1ROW_DATA	=	2048	;	// 256*64/8=1664


//---------------------------------------------------------------------------//



//------  wire  -------------  wire  ------  wire  ------  wire  ------  wire  ------  wire  ---//
//---
//---- BRAM_SORT wire -------
wire en_soz1 ;
wire wea_soz1 ;
wire [  11 : 0] addr_sram_soz1 ;
wire [  TRANS_BITS-1 : 0] din_sram_soz1 ;
wire [  TRANS_BITS-1 : 0] dout_sram_soz1 ;
//-------------------------

//---- control_signal ----
wire isif_valid ;

//---- sort state output ----
wire row1_sort_done ;

//---- output state output ----
wire row1_output_done ;



//---- data_sort_wire ----
wire [ 22:0] sort_buf_addres00	;
wire [ 19:0] sort_buf_addres01	;
wire [ 18:0] sort_buf_addres02	;
wire [ 18:0] sort_buf_addres03	;
wire [ 18:0] sort_buf_addres04	;
wire [ 18:0] sort_buf_addres05	;
wire [ 18:0] sort_buf_addres06	;
wire [ 22:0] sort_buf_addres07	;
wire [ 18:0 ] multi_row ;
wire [ 18:0 ] multi_ch_part ;
wire [ 22:0] multi_ch_selec ;


//----  store_wire ----
wire [ 9:0 ]buf_1024_cnt ;
wire [ 8:0 ]buf_array_cnt ;


//----	reset signal	----
wire cnt_rst ;


//---------------------------------------------------------------------------//

//---------------------  reg  ------  reg  ------  reg  ------  reg  ------  reg  ------  reg  ---//
//---
//--- store_data_counter ---
reg [18:0 ] data_counter ; // 

//REG of counter for SORT
reg [	2	:	0	] 	pin_addr ;		//	(0~7)
reg [	1	:	0	]	ch_selector	;	//	(0~3)
reg [	2	:	0	]	ch_part ;		//	(0~1) current ofmap ch is distribute 2 parts
reg [	4	:	0	]	col_addr ;		//	(0~31)	256/8=32
reg [	7	:	0	]	row_addr ;		//	(0~255)

// counter last_signal
reg pin_addr_last , 
	ch_selector_last , 
	ch_part_last , 
	col_addr_last , 
	row_addr_last ;
reg buf_1024_cnt_last ,
	buf_array_cnt_last ;

// counter enable_signal
reg en_pin_addr, 
	en_ch_selector , 
	en_ch_part , 
	en_col_addr , 
	en_row_addr ;

reg en_buf_array , 
	en_buf_1024 ;


//---- data_store ----
reg reset_store_cnt ;
reg en_store_read ;

// data_sort  
reg [ 18 : 0] ifmap_buffer_address;
reg [ 11 : 0 ] cnt_addr_soz1 ;
reg [ TRANS_BITS -1 : 0] data_sorted ;

// data_output
reg [10:0 ] cnt_output_addr ;
reg en_output_write ;

//---- last_signal ----
reg data_counter_last ;
reg data_sort_last ;
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


//---------------------------------------------------------------------------//

//=============================================================================
//====================		FOR GPIO 				===========================
//=============================================================================
reg [ 4:0] status_reg_fsm ;


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
    parameter STORE = 1 ;   //先全部存到 reg (後面再改掉)
    parameter SORT = 2 ;    // 先等 STORE reg 全部存完 再一一寫入 sram (後面再改掉)
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
        IDLE : next_state = ( isif_valid )? STORE : IDLE ;
        STORE : next_state = ( store_end ) ? SORT : STORE ;
        SORT : next_state = ( row1_sort_done ) ? OUTPUT: SORT; 
		OUTPUT : next_state = ( dout_osif_last ) ? RESET_CNT  : OUTPUT ;		// not yet
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
BRAM_SORT SOZ1(
  .clka(    clk      ),    // input wire clka
  .ena(     en_soz1      ),      // input wire ena
  .wea(     wea_soz1     ),      // input wire [0 : 0] wea
  .addra(   addr_sram_soz1   ),  // input wire [ 11 : 0] addra
  .dina(    din_sram_soz1    ),    // input wire [ 63 : 0] dina
  .douta(   dout_sram_soz1   )  // output wire [ 63: 0] douta
);

//============================================= END OF BLOCK RAM declare ===============


//=============================================================================
//================  combinational logic  ===================
//=============================================================================
//----------------------------------
// state turning logic
assign isif_valid = din_isif_empty_n && ( current_state == IDLE ) ;


//----------------------------------
// Block RAM input logic
// description : delay 1 clk from sort chooser
//-----------------------------------
assign en_soz1 = (current_state == SORT )? dly0_en_sort_data : (current_state == OUTPUT )? dly0_en_output_data : 0 	 ;
assign wea_soz1 = (current_state == SORT )? dly0_en_sort_data : 0 	;		//只有 en_sort_data 時才有 "1" 其餘都 "0"
assign din_sram_soz1 = data_sorted ;
assign addr_sram_soz1 = ( current_state == SORT )? cnt_addr_soz1 : cnt_output_addr	;
//----------------------------------


//=============================================================================
//----------------------------------------------------
// read signal generator
//	function : en_store_data啟動後同時啟動 read
//		當 counter 讀完所有所需資料後，關閉 read 
//		待data 寫入 reg 完成後 切換 state to sort data 
//		read 要分兩段
//----------------------------------------------------
assign dout_isif_read = en_store_read ;		// Segmented the reading data


assign cnt_rst = reset | rst_a ;


//=============================================================================
//===================  Delay logic  ========================
//function : delay enable signal
//description : delay the state enable signal 2~3 clocks 
//=============================================================================
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


//=============================================================================
//======================== 							===========================
//======================== 		store  data 		===========================
//======================== 							===========================
//=============================================================================
//----------------------------------
// Data store control 
// description : need two read_signal be distributed 
//	and 1024_cnt control 512_cnt control 
//	收到 empty_n 才可以接 read 
//	在 empty_n 結束 read 也要結束
// store_end 要改掉
//-----------------------------------
always@( posedge clk or posedge reset )begin
	if ( reset )begin
		en_store_read <= 'd0 ;
	end else begin
		if ( en_store_data )begin
			if( din_isif_empty_n ) begin
				en_store_read <= 'd1 ;
			end else begin
				en_store_read <= 'd0 ;
			end
		end else begin
			en_store_read <= 'd0 ;
		end
	end
end

assign store_end = ( buf_1024_cnt_last && buf_array_cnt_last ) ;
//----------------------------------
// Data store enable logic , last logic , reset cnt logic
// description : enable for count 1024 and 512
//-----------------------------------
always@( * )begin
	if ( en_store_read && din_isif_empty_n ) begin
		en_buf_1024 <= 1;
		if ( buf_1024_cnt == 1023 )begin
			en_buf_array <= 1 ;
		end
		else begin
			en_buf_array <= 0 ;
		end
	end	else begin
		en_buf_array <= 0 ;
		en_buf_1024 <= 0	;
	end
end

always@(*)begin
	if ( reset )begin
		buf_1024_cnt_last =0;
		buf_array_cnt_last =0;
	end else begin

		if( buf_1024_cnt == 1023 ) begin
			buf_1024_cnt_last = 1;
		end else begin
			buf_1024_cnt_last =0;
		end

		if( buf_array_cnt == REG_AY_SIZE -1 && buf_1024_cnt == 1023) begin
			buf_array_cnt_last = 1;
		end else begin
			buf_array_cnt_last =0;
		end

	end
end

//----------------------------------------------------
// always block : data_store buffer array
// description : store data at every posedge clk
//----------------------------------------------------
always@( posedge clk or posedge reset )begin
    if ( reset )begin
        
    end else begin
        if ( en_store_read && din_isif_empty_n )begin
            ifmap_buffer_array [buf_array_cnt ][ buf_1024_cnt ] <= din_isif_data ;
        end 
        else begin
            ifmap_buffer_array [buf_array_cnt ][ buf_1024_cnt ] <=ifmap_buffer_array [buf_array_cnt]  [ buf_1024_cnt] ;
        end

    end
end 

count_yi_v2 #(
    .BITS_OF_END_NUMBER (	'd10 ) ,	// default 10
    .END_NUMBER (	'd1024	)			// default 1024
) STORE_1024_u(
	.clk	(	clk		),
	.reset (	cnt_rst		), 
	.enable (	en_buf_1024	), 
	.cnt_q	(	buf_1024_cnt	)
);

count_yi_v2 #(
    .BITS_OF_END_NUMBER (	BITS_OF_REG_AY_SIZE  ) ,	// default 10
    .END_NUMBER (	REG_AY_SIZE	)		// default 1024
) STORE_512_u(
	.clk	(	clk		),
	.reset (	cnt_rst 	), 
	.enable (	en_buf_array	), 
	.cnt_q	(	buf_array_cnt	)
);

//----------------------------------------------------
// always block : store_data_counter 
// description : check all data should be stored.
//  and the last count need last signal at the same time
//  reset the counter to "0" .
//----------------------------------------------------
/*
always@( posedge clk or posedge reset )begin
    if ( reset )begin
        data_counter<= 'd0; 
    end else begin
        
		if ( en_store_data )begin
			if( data_counter >= IF_ENTRY_SIZE-1 )begin		//fixme
				data_counter<= 'd0 ;
			end 
			else begin
				data_counter<= data_counter + 'd1 ;
			end
		end 
		else begin
			data_counter <= data_counter ;
		end
    end
end 

always@(*)begin
	if( data_counter == IF_ENTRY_SIZE-1) begin
		data_counter_last = 1;
	end else begin
		data_counter_last =0;
	end
end
*/

//=============================================================================
//======================== 							===========================
//======================== 		sort data	 		===========================
//======================== 							===========================
//=============================================================================
//----------------------------------------------------
// always block : data_sort
// description : switch address and store data we want
//  to SRAM_SORT
//----------------------------------------------------


//multiple
assign multi_row = row_addr * 'd1024 ;
assign multi_ch_part = ch_part * 'd1024 * 'd256 ;
assign multi_ch_selec = ch_selector*8*32	;
//[	col_addr + (ch_selector*8+7)*32 + row_addr*1024 + 1024*256*half  ]
/*
assign sort_buf_addres00 = col_addr + multi_ch_selec + multi_row + multi_ch_part	;
assign sort_buf_addres01 = col_addr + multi_ch_selec + 32*'d1 + multi_row + multi_ch_part	;
assign sort_buf_addres02 = col_addr + multi_ch_selec + 32*'d2 + multi_row + multi_ch_part	;
assign sort_buf_addres03 = col_addr + multi_ch_selec + 32*'d3 + multi_row + multi_ch_part	;
assign sort_buf_addres04 = col_addr + multi_ch_selec + 32*'d4 + multi_row + multi_ch_part	;
assign sort_buf_addres05 = col_addr + multi_ch_selec + 32*'d5 + multi_row + multi_ch_part	;
assign sort_buf_addres06 = col_addr + multi_ch_selec + 32*'d6 + multi_row + multi_ch_part	;
assign sort_buf_addres07 = col_addr + multi_ch_selec + 32*'d7 + multi_row + multi_ch_part	;
*/

assign sort_buf_addres00 = col_addr + multi_ch_selec 	;
assign sort_buf_addres01 = col_addr + multi_ch_selec + 32*'d1 	;
assign sort_buf_addres02 = col_addr + multi_ch_selec + 32*'d2 	;
assign sort_buf_addres03 = col_addr + multi_ch_selec + 32*'d3 	;
assign sort_buf_addres04 = col_addr + multi_ch_selec + 32*'d4 	;
assign sort_buf_addres05 = col_addr + multi_ch_selec + 32*'d5 	;
assign sort_buf_addres06 = col_addr + multi_ch_selec + 32*'d6 	;
assign sort_buf_addres07 = col_addr + multi_ch_selec + 32*'d7 	;


always@( posedge clk or posedge reset )begin
	if ( reset )begin
        data_sorted	<=	'd0;
    end else begin
		if ( en_sort_data )begin
			case ( pin_addr	)
				'd0 : begin
					data_sorted <= { 	ifmap_buffer_array [ ch_part ][ sort_buf_addres00 ][63:56] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres01 ][63:56] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres02 ][63:56] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres03 ][63:56] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres04 ][63:56] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres05 ][63:56] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres06 ][63:56] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres07 ][63:56] 	};
				end
				'd1 : begin
					data_sorted <= { 	ifmap_buffer_array [ ch_part ][ sort_buf_addres00 ][ 55:48 ],
					ifmap_buffer_array[ ch_part ][ sort_buf_addres01 ][ 55:48 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres02 ][ 55:48 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres03 ][ 55:48 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres04 ][ 55:48 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres05 ][ 55:48 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres06 ][ 55:48 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres07 ][ 55:48 ] 	};
				end
				'd2 : begin
					data_sorted <= { 	ifmap_buffer_array [ ch_part ][ sort_buf_addres00 ][ 47:40 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres01 ][ 47:40 ]  ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres02 ][ 47:40 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres03 ][ 47:40 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres04 ][ 47:40 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres05 ][ 47:40 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres06 ][ 47:40 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres07 ][ 47:40 ] 	};
				end
				'd3 : begin
					data_sorted <= { 	ifmap_buffer_array [ ch_part ][ sort_buf_addres00 ][ 39:32 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres01 ][ 39:32 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres02 ][ 39:32 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres03 ][ 39:32 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres04 ][ 39:32 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres05 ][ 39:32 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres06 ][ 39:32 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres07 ][ 39:32 ] 	};
				end
				'd4 : begin
					data_sorted <= { 	ifmap_buffer_array [ ch_part ][ sort_buf_addres00 ][ 31:24 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres01 ][ 31:24 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres02 ][ 31:24 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres03 ][ 31:24 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres04 ][ 31:24 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres05 ][ 31:24 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres06 ][ 31:24 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres07 ][ 31:24 ] 	};
				end
				'd5 : begin
					data_sorted <= { 	ifmap_buffer_array [ ch_part ][ sort_buf_addres00 ][ 23:16 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres01 ][ 23:16 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres02 ][ 23:16 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres03 ][ 23:16 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres04 ][ 23:16 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres05 ][ 23:16 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres06 ][ 23:16 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres07 ][ 23:16 ] 	};
				end
				'd6 : begin
					data_sorted <= { 	ifmap_buffer_array [ ch_part ][ sort_buf_addres00 ][ 15:8 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres01 ][ 15:8 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres02 ][ 15:8 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres03 ][ 15:8 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres04 ][ 15:8 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres05 ][ 15:8 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres06 ][ 15:8 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres07 ][ 15:8 ] 	};
				end
				'd7 : begin
					data_sorted <= { 	ifmap_buffer_array [ ch_part ][ sort_buf_addres00 ][ 7:0 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres01 ][ 7:0 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres02 ][ 7:0 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres03 ][ 7:0 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres04 ][ 7:0 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres05 ][ 7:0 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres06 ][ 7:0 ] ,
					ifmap_buffer_array[ ch_part ][ sort_buf_addres07 ][ 7:0 ] 	};
				end
				default : data_sorted <= 'hffff_ffff_ffff_ffff ;

			endcase	

		end else begin
			data_sorted <= 'hffff_ffff_ffff_ffff ;
		end
	end
end

//-----------test --------------------------------
//-----------test --------------------------------
always@( posedge clk or posedge reset )begin
	if( reset ) begin
		temp_yi01 <= 'd0 ;
	end else begin
		temp_yi01 <= { 	ifmap_buffer_array [ 'd0 ][ sort_buf_addres00 ][63:56] ,
				ifmap_buffer_array[ 'd0 ][ sort_buf_addres01 ][63:56] ,
				ifmap_buffer_array[ 'd0 ][ sort_buf_addres02 ][63:56] ,
				ifmap_buffer_array[ 'd0 ][ sort_buf_addres03 ][63:56] ,
				ifmap_buffer_array[ 'd0 ][ sort_buf_addres04 ][63:56] ,
				ifmap_buffer_array[ 'd0 ][ sort_buf_addres05 ][63:56] ,
				ifmap_buffer_array[ 'd0 ][ sort_buf_addres06 ][63:56] ,
				ifmap_buffer_array[ 'd0 ][ sort_buf_addres07 ][63:56] 	};

		temp_yi02 <= { 	ifmap_buffer_array [ 'd1 ][ sort_buf_addres00 ][63:56] ,
				ifmap_buffer_array[ 'd1 ][ sort_buf_addres01 ][63:56] ,
				ifmap_buffer_array[ 'd1 ][ sort_buf_addres02 ][63:56] ,
				ifmap_buffer_array[ 'd1 ][ sort_buf_addres03 ][63:56] ,
				ifmap_buffer_array[ 'd1 ][ sort_buf_addres04 ][63:56] ,
				ifmap_buffer_array[ 'd1 ][ sort_buf_addres05 ][63:56] ,
				ifmap_buffer_array[ 'd1 ][ sort_buf_addres06 ][63:56] ,
				ifmap_buffer_array[ 'd1 ][ sort_buf_addres07 ][63:56] 	};
		
	end
end
//-----------test --------------------------------
//-----------test --------------------------------


//--------------------------------
//always block : counter sorting
//--------------------------------
always@( posedge clk or negedge cnt_rst )begin
    if ( cnt_rst )begin
        cnt_addr_soz1<= 'd0; 
    end else begin
		if ( dly0_en_sort_data )begin
			if( cnt_addr_soz1 >= NUM_OF_1ROW_DATA -1 )begin
				cnt_addr_soz1<= 'd0 ;
			end 
			else begin
				cnt_addr_soz1<= cnt_addr_soz1 + 'd1 ;
			end
		end 
		else begin
			cnt_addr_soz1 <= cnt_addr_soz1 ;
			end

    end
end 

//-------------------------------------------
// output state signal of SORT
//  sort full size of ifmap data  need 2047
//-------------------------------------------
assign row1_sort_done =  cnt_addr_soz1 == 2047 ;	


//================================================================================================


//=============================================================================
//======================== 								=======================
//======================== 		output data state 		=======================
//======================== 								=======================
//=============================================================================
// description : count and generate osif_write and reading SRAM data 
assign dout_osif_data = ( en_output_write ) ? dout_sram_soz1 : 'd0 ;
assign row1_output_done = dout_osif_last ;
assign dout_osif_last = dly0_cnt_output_last ;		// 10/14 need to wait for BRAM's reading data process about 2 clk
assign dout_osif_write = en_output_write &  dly0_en_output_data;

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
always@( posedge clk or posedge reset  )begin
	if( reset )begin
		full_data_output_done <= 0 ;
	end
	else begin
		if ( ch_selector >= 'd3 && ch_part >= 'd1 && pin_addr >= 'd7 && col_addr >= 'd31 && row_addr >= 'd255 )begin
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


//================================================================================================
//address sort

//	sram|	if_buffer_address
// column_0 
//	(0) [	0	|	32	|	64	|	96	|	128	|	160	|	192	|	224	]
//	(1)	[	256	|	288	|	320	|	352	|	..	|	..	|	..	|	480	]
//	(2)	[	512	|	544	|	..	|	..	|	..	|	..	|	..	|	736	]
//	(3)	[	768	|	800	|	832	|	864	|	896	|	928	|	960	|	992	]

// column_0 ch32~ch63
//	(4)	[	0*32		|	1*32		|	2*32		|	3*32		|	4*32		|	5*32		|	6*32		|	7*32		]
//		[	+1024*256	|	+1024*256	|	+1024*256	|	+1024*256	|	+1024*256	|	+1024*256	|	+1024*256	|	+1024*256	]

//	(5)	[	8*32		|	9*32		|	10*32		|	11*32		|	12*32		|	13*32		|	14*32		|	15*32		]
//		[	+1024*256	|	+1024*256	|	+1024*256	|	+1024*256	|	+1024*256	|	+1024*256	|	+1024*256	|	+1024*256	]
//	...
//	(7)	[	24*32		|	25*32		|	26*32		|	27*32		|	28*32		|	29*32		|	30*32		|	31*32		]
//		[	+1024*256	|	+1024*256	|	+1024*256	|	+1024*256	|	+1024*256	|	+1024*256	|	+1024*256	|	+1024*256	]

// column_n row_0 
// ex: n=174  取 (174+1)/8=21...7	get col_addr= 20 取 data[7:0]	前半 ch 取 half=0 後半 ch 取 half=1
//	ch0~ch7
//	(174*8)		[	col_addr+0*32	|	col_addr+1*32	|	....col_addr+6*32	|	col_addr+7*32	]=	[	20	|	20+1*32	|	20+2*32	|	20+3*32	|	...	|	20+7*32	]
//	(174*8+1)	[	col_addr+8*32	|	col_addr+9*32	|	....col_addr+14*32	|	col_addr+15*32	]
//					...					...					...						...
//----ch32~ch63	
//	(174*8+4)	[	col_addr+0*32	|	col_addr+1*32	|	....col_addr+6*32	|	col_addr+7*32	]
//				[	+1024*256		|	+1024*256		|	....+1024*256		|	+1024*256		]
//					...					...					...						...
//	(174*8+7)	[	col_addr+24*32	|	col_addr+25*32	|	....col_addr+30*32	|	col_addr+31*32	]
//				[	+1024*256		|	+1024*256		|	....+1024*256		|	+1024*256		]


// column_n row_20  取 row=35 
// ex: n=174  取 (174+1)/8=21...7	get col_addr= 20 取 data[7:0]	前半 ch 取 half=0 後半 ch 取 half=1
//	ch0~ch31	half=0
//	(174*8)		[	col_addr+0*32+row*1024	|	col_addr+1*32+row*1024	|	...	|	col_addr+6*32+row*1024	|	col_addr+7*32+row*1024	]
//				====[	20+0+35*1024		|	20+1*32+35*1024			|	...	|	20		+6*32+35*1024	|	20		+7*32+35*1024	]
//	(174*8+1)	[	col_addr+8*32+row*1024	|	col_addr+9*32+row*1024	|	...	|	col_addr+14*32+row*1024	|	col_addr+15*32+row*1024	]
//					...					...					...						...
//----ch32~ch63	 half=1 
//	(174*8+4)	[	col_addr+0*32+row*1024		|	col_addr+1*32+row*1024		|	...	|	col_addr+6*32+row*1024		|	col_addr+7*32+row*1024	]
//				[	+1024*256*half				|	+1024*256*half				|	...	|	+1024*256*half				|	+1024*256*half		]
//					...					...					...						...
//	(174*8+7)	[	col_addr+24*32+row*1024		|	col_addr+25*32+row*1024		|	...	|	col_addr+30*32+row*1024		|	col_addr+31*32+row*1024	]
//				[	+1024*256*half				|	+1024*256*half				|	...	|	+1024*256*half				|	+1024*256*half		]


//================================================================================================
// need counter reg = col_addr , pin_addr , row_addr , half , ch_selector 

//formula as follows	
//....col_addr= n/8 (取商) ,實際使用 counter+1 即可
//....腳位部分 做 pin_addr 使用 case( pin_addr) 挑選腳位輸出
// (n*8)	[	col_addr + (ch_selector*8+0)*32 + row_addr*1024 + 1024*256*half  ][ case(pin_addr) -: 8] 
//			[	col_addr + (ch_selector*8+1)*32 + row_addr*1024 + 1024*256*half  ][ case(pin_addr) -: 8] 
//			[	col_addr + (ch_selector*8+2)*32 + row_addr*1024 + 1024*256*half  ][ case(pin_addr) -: 8]
//			[	col_addr + (ch_selector*8+3)*32 + row_addr*1024 + 1024*256*half  ][ case(pin_addr) -: 8]
//			[	col_addr + (ch_selector*8+4)*32 + row_addr*1024 + 1024*256*half  ][ case(pin_addr) -: 8]
//			[	col_addr + (ch_selector*8+5)*32 + row_addr*1024 + 1024*256*half  ][ case(pin_addr) -: 8]
//			[	col_addr + (ch_selector*8+6)*32 + row_addr*1024 + 1024*256*half  ][ case(pin_addr) -: 8]
//			[	col_addr + (ch_selector*8+7)*32 + row_addr*1024 + 1024*256*half  ][ case(pin_addr) -: 8]


//	ch_selector(0~3)	-->	ch_selector(0~3)		-->	ch_selector(0~3)		-->	ch_selector(0~3)		-->	ch_selector(0~3)
//	half=0				-->>	half=1				-->>	half=0				-->>	half=1				-->>	half=0			
//	pin_addr=0			-->	pin_addr=0				-->>>	pin_addr=1			-->	pin_addr=1				-->>>	pin_addr=2

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//	pin_addr=6			-->	pin_addr=7			-->>	pin_addr=0			-->	pin_addr=1			-->	pin_addr=2			-->	.....
//	col_addr=0			-->	col_addr=0			-->>	col_addr=1			-->	col_addr=1			-->	col_addr=1			-->	.....

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//	col_addr=206		-->	col_addr=207		-->	col_addr=208		-->>	col_addr=0			-->	col_addr=1			-->	.....
//	row_addr=0			-->	row_addr=0			-->	row_addr=0			-->>	row_addr=1			-->	row_addr==1			--> .....


//================================================================================================





//=============================================================================
//======================== 								=======================
//======================== 		sort data counter 		=======================
//======================== 								=======================
//=============================================================================
// sort counter control signal
always@(*)begin
	if(reset)begin
		en_pin_addr <= 'd0;
		en_ch_selector <= 'd0;
		en_ch_part <= 'd0;
		en_col_addr <= 'd0;
		en_row_addr <= 'd0; 
	end
	else begin
		if( en_sort_data == 1)begin
			if( cnt_addr_soz1 >= 2047 )begin  en_ch_selector <= 'd0; 	// cnt_addr_soz1 >= 2046  先關
			end else begin en_ch_selector <= 'd1;  end
			if( ch_selector >= 'd3) en_ch_part =1 ; else en_ch_part =0 ;
			if ( (ch_selector == 'd3) & (ch_part == 'd1) ) en_pin_addr =1 ; else en_pin_addr =0 ;
			if( ch_selector >= 'd3 && ch_part >= 'd1 && pin_addr >= 'd7 ) en_col_addr =1 ; else en_col_addr =0;
			if( ch_selector >= 'd3 && ch_part >= 'd1 && pin_addr >= 'd7 && col_addr >= 'd31) en_row_addr =1; else en_row_addr =0;
		end
		else begin
			en_pin_addr <= 'd0;
			en_ch_selector <= 'd0;
			en_ch_part <= 'd0;
			en_col_addr <= 'd0;
			en_row_addr <= 'd0;
		end
	end
end


//-------------------------------------------
// last signal use combinational logic
//-------------------------------------------
always@(*)begin
	if(reset)begin
		ch_selector_last =0;
		ch_part_last =0;
		pin_addr_last =0;
		col_addr_last =0;
	end else begin
		if( pin_addr == 7)begin
			pin_addr_last = 1;
		end else begin
			pin_addr_last =0;
		end
		if( ch_part == 1)begin
			ch_part_last =1;
		end else begin
			ch_part_last =0;
		end
		if( ch_selector == 3) ch_selector_last =1 ; else ch_selector_last = 0 ;
		if( col_addr == 31) col_addr_last =1 ; else col_addr_last =0 ;
		if( row_addr == 207 ) row_addr_last = 1 ; else row_addr_last =0 ;
	end
end



//--------------------------------
//always block : counter pin_addr 
//--------------------------------
always@( posedge clk or posedge cnt_rst )begin
    if ( cnt_rst )begin
        pin_addr<= 'd0; 
    end else begin

		if ( en_pin_addr )begin
			if( pin_addr >= 'd7 )begin
				pin_addr<= 'd0 ;
			end 
			else begin
				pin_addr<= pin_addr + 'd1 ;
			end
		end 
		else begin
			pin_addr <= pin_addr ;
			end

    end
end 

//--------------------------------
//always block : counter ch_selector 
//--------------------------------
always@( posedge clk or posedge cnt_rst )begin
    if ( cnt_rst )begin
        ch_selector<= 'd0; 
    end else begin

		if ( en_ch_selector )begin
			if( ch_selector >= 3 )begin
				ch_selector<= 'd0 ;
			end 
			else begin
				ch_selector<= ch_selector + 'd1 ;
			end
		end 
		else begin
			ch_selector <= ch_selector ;
			end

    end
end 
//--------------------------------
//always block : counter ch_part 
//--------------------------------
always@( posedge clk or negedge cnt_rst )begin
    if ( cnt_rst )begin
        ch_part<= 'd0; 
    end else begin

		if ( en_ch_part )begin
			if( ch_part >= 1 )begin
				ch_part<= 'd0 ;
			end 
			else begin
				ch_part <= ch_part + 'd1 ;
			end
		end 
		else begin
			ch_part <= ch_part ;
			end

    end
end 
//--------------------------------
//always block : counter col_addr 
//--------------------------------
always@( posedge clk or posedge cnt_rst )begin
    if ( cnt_rst )begin
        col_addr<= 'd0; 
    end else begin
		if ( en_col_addr )begin
			if( col_addr >= 31 )begin
				col_addr<= 'd0 ;
			end 
			else begin
				col_addr<= col_addr + 'd1 ;
			end
		end 
		else begin
			col_addr <= col_addr ;
			end

    end
end 
//--------------------------------
//always block : counter row_addr 
//--------------------------------
always@( posedge clk or posedge cnt_rst )begin
    if ( cnt_rst )begin
        row_addr<= 'd0; 
    end else begin
		if ( en_row_addr )begin
			if( row_addr >= 207 )begin
				row_addr<= 'd0 ;
			end 
			else begin
				row_addr<= row_addr + 'd1 ;
			end
		end 
		else begin
			row_addr <= row_addr ;
			end

    end
end 

//================================================================================================

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



