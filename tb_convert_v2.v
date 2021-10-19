//=========================================================
// File Name   : tb_v2.v
// Description : top testbench
// Designer    : Ching-Shun Wang
//=========================================================
`timescale 1ns / 100ps
`define CYCLE       50.0

//`define PAT_WEIGHTS        "C:\\PAT\\v2_verification_data\\weights.dat"
//`define PAT_BIAS           "C:\\Users\\john\\Desktop\\sim_3v3\cs_pat_bias41_0to1.dat"
//`define PAT_IFMAP          "D:\\PAT\\v2_verification_data\\L0_in.dat"
//`define PAT_OFMAP          "D:\\PAT\\v2_verification_data\\L0_out.dat"
//`define PAT_WEIGHTS        "C:\\Users\\JX\\Desktop\\YIYUAN\\yk_combine_0120\\total\\dat\\w_37.dat"
//`define PAT_BIAS           "C:\\Users\\JX\\Desktop\\YIYUAN\\yk_combine_0120\\total\\dat\\b_37.dat"
//`define PAT_IFMAP          "C:\\Users\\JX\\Desktop\\YIYUAN\\yk_combine_0120\\total\\dat\\if_37.dat"
//`define PAT_OFMAP          "C:\\Users\\JX\\Desktop\\YIYUAN\\yk_combine_0120\\total\\dat\\of_37.dat"


//2021_09_21 user: yi_yuan at home_PC
`define PAT_WEIGHTS         "E:/yuan/v_code/yi_20210929_convertor/Sorce/dat/layer4w_agile_1.dat"
`define PAT_IFMAP2           "E:/yuan/v_code/yi_20211012_ld_convertor/Sorce/dat/layer2_ofm_part2.dat"
`define PAT_IFMAP           "E:/yuan/v_code/yi_20211012_ld_convertor/Sorce/dat/layer2_ofm_size256_ch64_32once.dat"
`define PAT_OFMAP           "E:/yuan/v_code/yi_20211012_ld_convertor/Sorce/dat/lay3_ifm_bylay2_size208_ch64.dat"


module tb_convert_v2();
parameter TBITS = 64;
parameter TBYTE = 8;

parameter IFMAP_SIZE   = 524288;    // row*col*ch/8=256*256*32/8=262144       // 2021/06/08  IFMAP_37=8192
parameter OFMAP_SIZE   = 1664 * CV_REPEAT_TIMES ;         //	10/14:208*64/8 = 1664 		32*256*256/8=262144 
parameter ONCE_1_ROW_ENTRIES = 1024 ;	// layer2 output once 32ch = 256*32 =1024
parameter CV_REPEAT_TIMES = 208 ;

parameter BIAS_SIZE    = 32;                // 2021/06/08  bias_37=4
parameter WEIGHTS_SIZE = 1152;      //32*3*3*32/8= 72
parameter M  = 32'b0100_0000_0000_0010_0100_0101_1100_0101;     // changed at 0704
parameter ZW =  8'd138;
parameter ZO =  8'h0;
parameter index = 6'd7;

reg              S_AXIS_MM2S_TVALID = 0;
wire             S_AXIS_MM2S_TREADY;
reg  [TBITS-1:0] S_AXIS_MM2S_TDATA = 0;
reg  [TBYTE-1:0] S_AXIS_MM2S_TKEEP = 0;
reg  [1-1:0]     S_AXIS_MM2S_TLAST = 0;

wire             M_AXIS_S2MM_TVALID;
reg              M_AXIS_S2MM_TREADY = 0;
wire [TBITS-1:0] M_AXIS_S2MM_TDATA;
wire [TBYTE-1:0] M_AXIS_S2MM_TKEEP;
wire [1-1:0]     M_AXIS_S2MM_TLAST;

reg              aclk = 0;
reg              aresetn = 1;

reg              done = 0;
//=========================================================
// yolo_top
//---------------------------------------------------------
convert_top
#(
        .TBITS(TBITS),
        .TBYTE(TBYTE)
) c_top_instance (
        .S_AXIS_MM2S_TVALID(S_AXIS_MM2S_TVALID),
        .S_AXIS_MM2S_TREADY(S_AXIS_MM2S_TREADY),
        .S_AXIS_MM2S_TDATA(S_AXIS_MM2S_TDATA),
        .S_AXIS_MM2S_TKEEP(S_AXIS_MM2S_TKEEP),
        .S_AXIS_MM2S_TLAST(S_AXIS_MM2S_TLAST),
        
        .M_AXIS_S2MM_TVALID(M_AXIS_S2MM_TVALID),
        .M_AXIS_S2MM_TREADY(M_AXIS_S2MM_TREADY),
        .M_AXIS_S2MM_TDATA(M_AXIS_S2MM_TDATA),
        .M_AXIS_S2MM_TKEEP(M_AXIS_S2MM_TKEEP),
        .M_AXIS_S2MM_TLAST(M_AXIS_S2MM_TLAST),  // EOL      
        
        .S_AXIS_MM2S_ACLK(aclk),
        .M_AXIS_S2MM_ACLK(aclk),
        .aclk(aclk),
        .aresetn(aresetn)
); 
//---------------------------------------------------------

reg    [TBITS-1 : 0]      weights_t  [0:WEIGHTS_SIZE-1];
reg    [TBITS-1 : 0]      bias_t     [0:BIAS_SIZE-1];
reg    [TBITS-1 : 0]      ifmap_t    [0:IFMAP_SIZE-1];

reg    [TBITS-1 : 0]      weights_t2;
reg    [31:0]             bias_t2;
reg    [TBITS-1 : 0]      ifmap_t2;
reg    [TBITS-1 : 0]      ofmap_gold_t2;

reg    [TBITS-1 : 0]      ifmap2        [0:IFMAP_SIZE-1];
reg    [TBITS-1 : 0]      ifmap        [0:IFMAP_SIZE-1];
reg    [TBITS-1 : 0]      ofmap_gold   [0:OFMAP_SIZE-1];
reg    [TBITS-1 : 0]      ofmap_gold_t [0:OFMAP_SIZE-1];

reg    [TBITS-1 : 0]      ofmap        [0:OFMAP_SIZE-1];

reg    [19 : 0]           ofmap_cnt;

reg    [TBITS-1 : 0]      weights      [0:WEIGHTS_SIZE-1];
reg    [TBITS-1 : 0]      bias         [0:BIAS_SIZE-1];

//temp

//temp


integer i;
integer repeat_cnt ;
integer bias_num;
integer kernel_num;
integer ifmap_num;
integer ofmap_num;
integer error;

initial begin  
    $readmemh(`PAT_WEIGHTS   , weights_t);
    $readmemh(`PAT_IFMAP2     , ifmap2);
    
    $readmemh(`PAT_IFMAP     , ifmap);
    $readmemh(`PAT_OFMAP     , ofmap_gold_t);
end

// ifmap/weights reverse
initial begin
    for(i=0; i<WEIGHTS_SIZE; i=i+1)begin
        weights_t2 = weights_t[i];
        weights[i] = {      weights_t2[ 7 :  0], 
                            weights_t2[15 :  8],
                            weights_t2[23 : 16],
                            weights_t2[31 : 24],
                            weights_t2[39 : 32],
                            weights_t2[47 : 40],
                            weights_t2[55 : 48],
                            weights_t2[63 : 56]
                     };
    end  
    /*
    for(i=0; i<IFMAP_SIZE; i=i+1)begin
        ifmap[i]  = ifmap_t[i];
        
    end
	*/

    for(i=0; i<OFMAP_SIZE; i=i+1)begin
        ofmap_gold[i] = ofmap_gold_t[i];

    end
end

initial begin                      
    #(`CYCLE*2);
    aresetn = 0;
    #(`CYCLE*3);
    aresetn = 1;
  
    #(`CYCLE*10);
    M_AXIS_S2MM_TREADY = 1;
    S_AXIS_MM2S_TKEEP = 'hff;

    @(posedge aclk);
    S_AXIS_MM2S_TLAST  = 0;
    S_AXIS_MM2S_TVALID = 0;
    S_AXIS_MM2S_TDATA  = 'h0;
    #(`CYCLE*5);
    

	for( repeat_cnt=0 ; repeat_cnt<	CV_REPEAT_TIMES	; repeat_cnt=repeat_cnt+1 )begin
			// ifmap : lay2 output need to convert
		//	1 row with 32 ch = 256*32 = 1024
		// total need 2048 entries can be converted by convertor
		for(i=0; i<ONCE_1_ROW_ENTRIES; i=i+1)begin
			@(posedge aclk);
			S_AXIS_MM2S_TVALID = 1;
			S_AXIS_MM2S_TDATA  = ifmap	[i	+ 1024*repeat_cnt	];
			if(i==(ONCE_1_ROW_ENTRIES-1) )begin
				S_AXIS_MM2S_TLAST = 1;
			end
			#0.1;
			wait(S_AXIS_MM2S_TREADY);
		end

		@(posedge aclk);
		S_AXIS_MM2S_TLAST  = 0;
		S_AXIS_MM2S_TVALID = 0;
		S_AXIS_MM2S_TDATA  = 'h0;
		#(`CYCLE*5);

		for(i=0; i<ONCE_1_ROW_ENTRIES; i=i+1)begin
			@(posedge aclk);
			S_AXIS_MM2S_TVALID = 1;
			S_AXIS_MM2S_TDATA  = ifmap2	[	i	+ 1024*repeat_cnt	];	// next half ch map
			if(i==(ONCE_1_ROW_ENTRIES-1) )begin
				S_AXIS_MM2S_TLAST = 1;
			end
			#0.1;
			wait(S_AXIS_MM2S_TREADY);
		end
		
		@(posedge aclk);
		S_AXIS_MM2S_TLAST  = 0;
		S_AXIS_MM2S_TVALID = 0;
		S_AXIS_MM2S_TDATA  = 'h0;
		#(`CYCLE*5);

		wait(M_AXIS_S2MM_TLAST);		// wait for output transmit complete
		#(`CYCLE*100);					// delay for next convert

	end
    
	done = 1; // yi


			
	wait(M_AXIS_S2MM_TLAST);
		#(`CYCLE*5);
		done = 1;
   // $finish;
end



integer fp_w;

initial begin
    fp_w = $fopen("E:/yuan/v_code/yi_20211012_ld_convertor/Sorce/log/log_v2.log", "w");
    
    error = 0;
    wait(done);
    for(ofmap_num=0; ofmap_num<OFMAP_SIZE; ofmap_num=ofmap_num+1)begin
        if(ofmap[ofmap_num] !== ofmap_gold[ofmap_num])begin
            $display("error at %d, ofmap= %x  ofmap_gold= %x\n", ofmap_num, ofmap[ofmap_num], ofmap_gold[ofmap_num]);
            error = error + 1;
            $fwrite(fp_w, "error at %d, ofmap= %x  ofmap_gold= %x\n", ofmap_num, ofmap[ofmap_num], ofmap_gold[ofmap_num]);
        end
    end
    if(error > 0)begin
        $display("QQ, Total error = %d\n", error);
    end else begin
        $display("^__^");
    end
    $fclose(fp_w);
    $finish;
end

always @(posedge aclk)begin
    if(~aresetn)begin
        ofmap_cnt <= 'd0;
    end else begin
        if(M_AXIS_S2MM_TVALID)begin
            ofmap_cnt        <= ofmap_cnt + 1;
            ofmap[ofmap_cnt] <= M_AXIS_S2MM_TDATA;
        end else begin
            ofmap_cnt        <= ofmap_cnt;
        end
    end
end

always begin #(`CYCLE/2) aclk = ~aclk; end

endmodule
