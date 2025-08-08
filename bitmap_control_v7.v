`timescale 1ns / 1ps
//对bitmap的格式  
//打孔  {01,aack_valid,sack_valid,aack_shift,sack_shift}          no_return
//找洞  {10,15sack_high_shift,15hole_shift}                       return      10 new_hole_shift
//切换  {0000,15sack_high_shift，13'b0}                            return      00 fb_addr
//预读  {11,22'b0,fb_addr}                                        no_return

//aack_shift==0 无效 sack_shift
module bitmap_control_v7(
input clk,
input rst_n,

///////////FIFOTOBMAP
input [31:0] fifo_sacktobmap_dout,            //[14:0] shift 15 1sack 0aack
input fifo_sacktobmap_empty,
output  fifo_sacktobmaprd_en,
//////////FIFOTOSACK
input fifo_bmaptosack_full,       //暂未编写full时停止写入的逻辑
output reg [15:0] fifo_bmaptosack_din,
output reg fifo_bmaptosackwr_en

    );
//
reg [31:0]  wait_cnt;
reg         wr_newhole_flag;
reg [127:0] bitmap0;
reg [127:0] bitmap1;
//reg [127:0] bitmap2;
//reg [127:0] bitmap3;
//reg [127:0] bitmap4;
//reg [127:0] bitmap5;
reg [127:0] bank0;
reg [31:0]  bank1;
reg [31:0]  bank2;
reg [31:0]  bank3;
wire [3:0]  bank_sel;
reg         sack_valid;   
reg [13:0]  sack_shift;     
reg [13:0]  aack_shift;  
reg         aack_valid;  
reg [5:0]   hole1;
reg [14:0]  hole_shift;
reg [13:0]  hole_shift_return;

reg [14:0]  sack_high_shift;
wire [5:0]  block_cnt_needed;
reg  [5:0]  block_cnt_used;
wire        qieliu_finish_flag;
reg  [7:0]  fb_addr;
reg [7:0]   n_ptr;
///////////////////可用block地址fifo.....................
wire pop;
reg push;
wire [8:0] block_cnt; 
wire [7:0] block_addr_out;
reg [7:0] block_addr_in;
reg [7:0] block_cnt_out;
///////////////////bitmap_block_ram
reg             bitmap_ram_wea;
reg [7:0]       bitmap_ram_addra;      
reg [15:0]      bitmap_ram_dina;
wire [15:0]     bitmap_ram_douta;

wire [23:0] bitmap23_0;
////状态机
reg  [11:0]  state;
reg  [11:0]  next_state; 

parameter WAIT              =12'b0000_0000_0001 ;                 //001
parameter RD_FIFOTOBMAP     =12'b0000_0000_0010 ;                 //002 读取指令后判断下一状态
parameter SACK_DAKONG       =12'b0000_0000_0100 ;                 //004 打洞
parameter AACK_DAKONG       =12'b0000_0000_1000 ;                 //008 打洞

parameter ZHAODONG_PRE      =12'b0000_0001_0000 ;                 //010 读取block  
parameter ZHAODONG_ING      =12'b0000_0010_0000 ;                 //020 写回block
parameter ZHAODONG_END      =12'b0000_0100_0000 ;                 //040 RETURN  
///写回
//parameter QIELIU_0          =12'b0000_1000_0000 ;                 //080 返回fb_addr
parameter QIELIU_BODY       =12'b0001_0000_0000 ;                 //100 保存后续block
parameter QIELIU_HEAD       =12'b0010_0000_0000 ;                 //200 保存后续block
///读取
parameter YUDU              =12'b0100_0000_0000 ;                 //400 读取fb
parameter JUDGE             =12'b1000_0000_0000 ;                 //800 读取fb
//parameter RD_BLOCK          =10'b10_0000_0000 ;                 //200 读取block
//找洞
assign bitmap23_0 = bitmap0[23:0];
/////////////////状态机////////////////////////////
always @(posedge clk or negedge rst_n)begin
  if(~rst_n)
    state<=WAIT;
  else 
    state<=next_state;  
end

always @(*)begin
  case(state)
    WAIT          : begin
        if(~fifo_sacktobmap_empty)
          next_state = RD_FIFOTOBMAP ;
        else  
          next_state = WAIT ;
    end  
    RD_FIFOTOBMAP   :begin
      case(fifo_sacktobmap_dout[31:30])
        2'b00 :begin
          if(block_cnt_needed == 1)
            next_state = QIELIU_HEAD ; 
          else
            next_state = QIELIU_BODY;   
        end   
        2'b01 :begin
          if(fifo_sacktobmap_dout[28])
            next_state = SACK_DAKONG ;
          else  
            next_state = AACK_DAKONG ;
        end  
        2'b10 : next_state = ZHAODONG_PRE ;
        2'b11 : next_state = YUDU ;
      endcase
    end
    SACK_DAKONG     :begin
      if(aack_valid)
        next_state = AACK_DAKONG ;
      else
        next_state = WAIT ;  
    end
    AACK_DAKONG     : next_state = WAIT ;
    ZHAODONG_PRE    : next_state = ZHAODONG_ING ;
    ZHAODONG_ING    : next_state = ZHAODONG_END    ;
    ZHAODONG_END    : next_state = WAIT ; 
    QIELIU_BODY          : begin
      if(qieliu_finish_flag)
        next_state = QIELIU_HEAD ;
      else
        next_state = QIELIU_BODY;
    end
    QIELIU_HEAD:
      next_state = WAIT ;
    YUDU           : 
      if(wait_cnt ==1)
        next_state = JUDGE ;
      else
        next_state = YUDU ;   
    JUDGE          :begin
      if(bitmap_ram_douta[7:0]==0)
        next_state = WAIT ;
      else
        next_state = YUDU ;  
    end    
    default        : next_state = WAIT ;
  endcase
end 
assign block_cnt_needed = (fifo_sacktobmap_dout[15:0]-1)/8+1'b1;           //需要多少个
assign qieliu_finish_flag = block_cnt_needed== block_cnt_used+2'b10;//存的数量和需要的相等时结束 
// 因为存在一排延迟所以要提前一排借宿
//fifo_sacktobmaprd_en
assign fifo_sacktobmaprd_en = (state == WAIT && ~fifo_sacktobmap_empty);
always @(posedge clk or negedge rst_n) begin  
  if (~rst_n) 
    aack_valid <= 0;
  else if (state == RD_FIFOTOBMAP && fifo_sacktobmap_dout[31:30] == 2'b01) //shift目前只支持8位数据，但给输入留了14位
    aack_valid <= fifo_sacktobmap_dout[29];   
  else if (state == AACK_DAKONG && state!=next_state) 
    aack_valid <= 0;   
  else
    aack_valid <= aack_valid;
end
always @(posedge clk or negedge rst_n) begin  
  if (~rst_n) 
    aack_shift <= 0;
  else if (state == RD_FIFOTOBMAP && fifo_sacktobmap_dout[31:30] == 2'b01) //shift目前只支持8位数据，但给输入留了14位
    aack_shift <= fifo_sacktobmap_dout[27:14];   
  else if (state == AACK_DAKONG && state!=next_state) 
    aack_shift <= 0;   
  else
    aack_shift <= aack_shift;
end
//sack_valid 
always @(posedge clk or negedge rst_n) begin  
  if (~rst_n) 
    sack_valid <= 0;
  else if (state == RD_FIFOTOBMAP && fifo_sacktobmap_dout[31:30] == 2'b01) //shift目前只支持8位数据，但给输入留了14位
    sack_valid <= fifo_sacktobmap_dout[28];   
  else if (state == SACK_DAKONG && state!=next_state) 
    sack_valid <= 0;   
  else
    sack_valid <= sack_valid;
end
//sack_shift
always @(posedge clk or negedge rst_n) begin  
  if (~rst_n) 
    sack_shift <= 0;
  else if (state == RD_FIFOTOBMAP && fifo_sacktobmap_dout[31:30] == 2'b01) //shift目前只支持8位数据，但给输入留了14位
    sack_shift <= fifo_sacktobmap_dout[13:0];   
  else if (state == SACK_DAKONG && state!=next_state) 
    sack_shift <= 0;   
  else
    sack_shift <= sack_shift;
end
//bitmap0
always @(posedge clk or negedge rst_n) begin  
  if (~rst_n) 
    bitmap0 <= 0;
  else if (state == SACK_DAKONG  ) //shift目前只支持8位数据，但给输入留了14位
    bitmap0<=bitmap0 | (1 << sack_shift); 
  else if(state == AACK_DAKONG)
    bitmap0<=bitmap0 >> aack_shift;  
  else if( state == QIELIU_BODY|| state == QIELIU_HEAD)     //state == QIELIU_0 ||
    bitmap0<=bitmap0 >> 8;    
  else if(state == JUDGE  ) 
    bitmap0<={bitmap0[119:0],bitmap_ram_douta[15:8]}; 
  else if(state == SACK_DAKONG ||state ==AACK_DAKONG||state ==AACK_DAKONG ||state ==RD_FIFOTOBMAP||state ==WAIT) begin
    if(bitmap0[0] ==1'b1 )
      bitmap0<=bitmap0 >>1; 
    else if(bitmap0[1:0] == 2'b11 )
      bitmap0<=bitmap0 >>2; 
    else if(bitmap0[2:0] == 3'b111 )
      bitmap0<=bitmap0 >>3; 
    else if(bitmap0[3:0] == 4'b1111)
      bitmap0<=bitmap0 >>4; 
    else if(bitmap0[4:0] == 5'b11111)
      bitmap0<=bitmap0 >>5;
    else if(bitmap0[5:0] == 6'b111111)
      bitmap0<=bitmap0 >>6; 
    else if(bitmap0[6:0] == 7'b1111111)
      bitmap0<=bitmap0 >>7; 
    else if(bitmap0[7:0] == 8'b11111111)
      bitmap0<=bitmap0 >>8; 
    else
      bitmap0<=bitmap0;
  end    
  else
    bitmap0<=bitmap0;     
end

////////addr_cache
//always @(posedge clk or negedge rst_n) begin
//  if (~rst_n)
//    addr_cache <= 0;
//  else if (state == WR_BLOCK)
//    addr_cache <= block_addr_out;
//  else if (state == JUDGE || state == WAIT)
//    addr_cache <= 0;
//  else
//    addr_cache <= addr_cache;
//end
//block_cnt_used
always @(posedge clk or negedge rst_n) begin 
  if(~rst_n)
    block_cnt_used<=0;
  else if ( state ==  QIELIU_BODY|| state == QIELIU_HEAD)       //state == QIELIU_0 ||
    block_cnt_used <= block_cnt_used+1'b1;
  else
    block_cnt_used<= 0;
end
assign pop =   next_state ==  QIELIU_BODY || next_state == QIELIU_HEAD ;
//assign push =   next_state == YUDU ;
always @(posedge clk or negedge rst_n) begin 
  if(~rst_n)
    push<=0;
  else if(state== RD_FIFOTOBMAP && fifo_sacktobmap_dout[31:30] == 2'b11)  
    push <= 1'b1;
  else if (state == YUDU && state != next_state && bitmap_ram_douta[7:0]!=0)
    push <= 1'b1;
  else
    push <= 0;
end
//////block_addr_in
always @(posedge clk or negedge rst_n) begin    //由于延时过大和push一起改写为时序逻辑
  if (~rst_n)
    block_addr_in <= 0;
  else if (state== RD_FIFOTOBMAP && fifo_sacktobmap_dout[31:30] == 2'b11)
    block_addr_in <= fifo_sacktobmap_dout[7:0];
  else if(state == YUDU)  
    block_addr_in <= bitmap_ram_douta[7:0];
//  else if(state == RD_BLOCK)
//    block_addr_in <= fb_addr; //低七位存next指针
  else
    block_addr_in <= block_addr_in;
end
//output reg [15:0] fifo_bmaptosack_din,
//output reg fifo_bmaptosackwr_en
//wr_newhole_flag


////////////////////////////////////////////////////////////////////////
//找洞T
////////////////////////////////////////////////////////////////////////
//hole_shift;
always @(posedge clk or negedge rst_n) begin  
  if (~rst_n) 
    hole_shift <= 0;
  else if (state == RD_FIFOTOBMAP && fifo_sacktobmap_dout[31:30] == 2'b10) //shift目前只支持8位数据，但给输入留了14位
    hole_shift <= fifo_sacktobmap_dout[14:0];   
  else if (state == WAIT ) 
    hole_shift <= 0;   
  else
    hole_shift <= hole_shift;
end
//sack_high_shift
always @(posedge clk or negedge rst_n) begin  
  if (~rst_n) 
    sack_high_shift <= 0;
////qieliu yubei
  else if(state == RD_FIFOTOBMAP && fifo_sacktobmap_dout[31:30] == 2'b00 )
    sack_high_shift <= fifo_sacktobmap_dout[15:0]-1;      
//  else if (state == RD_FIFOTOBMAP && (fifo_sacktobmap_dout[31:30] == 2'b10 ||fifo_sacktobmap_dout[31:30] == 2'b00)) //shift目前只支持8位数据，但给输入留了14位
 //   sack_high_shift <= fifo_sacktobmap_dout[29:15];   
  else if (state == WAIT ) //&& state!=next_state
    sack_high_shift <= 0;   
  else
    sack_high_shift <= sack_high_shift;
end
//bank0
always@(posedge clk or negedge rst_n) begin
  if (~rst_n) 
    bank0 <= 0;
  else if(state == RD_FIFOTOBMAP && fifo_sacktobmap_dout[31:30] == 2'b10 )  
    bank0 <= bitmap0>>(fifo_sacktobmap_dout[13:0]+1);
  else if(state == WAIT)
    bank0 <= 0;
  else 
    bank0<=bank0;  
end
assign bank_sel[0] = bank0[31:0]   != 32'hffff_ffff;
assign bank_sel[1] = bank0[63:32]  != 32'hffff_ffff;
assign bank_sel[2] = bank0[95:64]  != 32'hffff_ffff;
assign bank_sel[3] = bank0[127:96] != 32'hffff_ffff;
//assign bank_sel[4] = bank0[31:0]   != 32'hfffffff;
//assign bank_sel[5] = bank0[63:32]  != 32'hfffffff;
//assign bank_sel[6] = bank0[95:64]  != 32'hfffffff;
//assign bank_sel[7] = bank0[127:96] != 32'hfffffff;
//bank1
always@(posedge clk or negedge rst_n) begin
  if (~rst_n) 
    bank1 <= 0;
  else if(state == ZHAODONG_PRE)  
    if(bank_sel[0])
      bank1 <= bank0[31:0];
    else if(bank_sel[1])
      bank1 <= bank0[63:32];
    else if(bank_sel[2])
      bank1 <= bank0[95:64];
    else
      bank1 <= bank0[127:96];    
  else 
    bank1<=bank1;  
end
//bank0
always@(posedge clk or negedge rst_n) begin
  if (~rst_n) 
    hole1 <= 0;
  else if(state == ZHAODONG_ING  )  
    if(~bank1[0])
      hole1 <= 6'd0;
    else if(~bank1[1])
      hole1 <= 6'd1;
    else if(~bank1[2])
      hole1 <= 6'd2;
    else if(~bank1[3])
      hole1 <= 6'd3;
    else if(~bank1[4])
      hole1 <= 6'd4;
    else if(~bank1[5])
      hole1 <= 6'd5;
    else if(~bank1[6])
      hole1 <= 6'd6;
    else if(~bank1[7])
      hole1 <= 6'd7;
    else if(~bank1[8])
      hole1 <= 6'd8;
    else if(~bank1[9])
      hole1 <= 6'd9;
    else if(~bank1[10])
      hole1 <= 6'd10;
    else if(~bank1[11])
      hole1 <= 6'd11;
    else if(~bank1[12])
      hole1 <= 6'd12;
    else if(~bank1[13])
      hole1 <= 6'd13;
    else if(~bank1[14])
      hole1 <= 6'd14;
    else if(~bank1[15])
      hole1 <= 6'd15;
    else if(~bank1[16])
      hole1 <= 6'd16;    
    else if(~bank1[17])
      hole1 <= 6'd17;
    else if(~bank1[18])
      hole1 <= 6'd18;
    else if(~bank1[19])
      hole1 <= 6'd19;
    else if(~bank1[20])
      hole1 <= 6'd20;
    else if(~bank1[21])
      hole1 <= 6'd21;
    else if(~bank1[22])
      hole1 <= 6'd22;
    else if(~bank1[23])
      hole1 <= 6'd23;
    else if(~bank1[24])
      hole1 <= 6'd24;
    else if(~bank1[25])
      hole1 <= 6'd25;
    else if(~bank1[26])
      hole1 <= 6'd26;
    else if(~bank1[27])
      hole1 <= 6'd27;
    else if(~bank1[28])
      hole1 <= 6'd28;
    else if(~bank1[29])
      hole1 <= 6'd29;
    else if(~bank1[30])
      hole1 <= 6'd30;
    else //if(bank1[31])
      hole1 <= 6'd31;
end
//hole_shift_return;
always @(posedge clk or negedge rst_n) begin  
  if (~rst_n) 
    hole_shift_return <= 0;
  else if (state == ZHAODONG_END) //shift目前只支持8位数据，但给输入留了14位
    if(bank_sel[0])
      hole_shift_return <= hole1+hole_shift+1'b1;
    else if(bank_sel[1])
      hole_shift_return <= hole1+hole_shift+10'd33;
    else if(bank_sel[2])
      hole_shift_return <= hole1+hole_shift+10'd65;
    else if(bank_sel[3])
      hole_shift_return <= hole1+hole_shift+10'd97;
    else 
      hole_shift_return <= hole_shift_return;
  else if(state == WAIT )  
    hole_shift_return<=0;
  else
    hole_shift_return <= hole_shift_return;
end
////////////////////////////////////////
////////////////////////////////////////
///////////////////bitmap_block_ram
//////fb_addr
always @(posedge clk or negedge rst_n) begin
  if (~rst_n)
    fb_addr <= 8'd1;
//  else if (state == RD_FIFOTOBMAP && fifo_sacktobmap_dout[31:30] == 2'b11)//读取
//    fb_addr <= fifo_sacktobmap_dout[7:0];
  else if (state == QIELIU_HEAD && state!=next_state)
    fb_addr <= block_addr_out;
  else
    fb_addr <= fb_addr;
end
//n_ptr
always @(posedge clk or negedge rst_n) begin
  if (~rst_n)
    n_ptr <= 0;
  else if(state == WAIT)  
    n_ptr <= 0;  
  else if (state == RD_FIFOTOBMAP && fifo_sacktobmap_dout[31:30] == 2'b11)//读取
    n_ptr <= fifo_sacktobmap_dout[7:0];
  else if (state == JUDGE)
    n_ptr <= block_addr_out;
  else if ( state == QIELIU_BODY)
    n_ptr <= block_addr_out;
  else
    n_ptr <= n_ptr;
end
//应该把nptr压回去，并给ram_addr_in,尚未修改，
////bitmap_ram_wea 链表写使能
always @(posedge clk or negedge rst_n) begin
  if (~rst_n)
    bitmap_ram_wea = 0;
  else if(state == QIELIU_HEAD|| state == QIELIU_BODY )    // state == QIELIU_0 || 
    bitmap_ram_wea = 1;
  else
    bitmap_ram_wea = 0;
end

////bitmap_ram_addra 链表地址
always @(posedge clk or negedge rst_n) begin     //////写
  if (~rst_n)   
    bitmap_ram_addra<=0;                               
  else if (state == QIELIU_HEAD   )                //写fb以外block的地址
    bitmap_ram_addra <= fb_addr;
  else if(state == QIELIU_BODY )  
    bitmap_ram_addra <= block_addr_out;
  else if(state ==RD_FIFOTOBMAP && fifo_sacktobmap_dout[31:30] == 2'b11)  
    bitmap_ram_addra <= fifo_sacktobmap_dout[7:0]; 
  else if (next_state == YUDU )                //写fb以外block的地址
    bitmap_ram_addra <= bitmap_ram_douta[7:0];     
  else                                                  
    bitmap_ram_addra = bitmap_ram_addra;
end

////bitmap_ram_dina  链表数据写入
always @(posedge clk or negedge rst_n) begin
  if (~rst_n) 
    bitmap_ram_dina<=0; 
  else if(state == QIELIU_BODY)
    bitmap_ram_dina<={bitmap0[7:0],n_ptr}; 
  else if(state == QIELIU_HEAD)
    if(block_cnt_needed==1)
      bitmap_ram_dina<={bitmap0[7:0],8'b0}; 
    else  
      bitmap_ram_dina<={bitmap0[7:0],n_ptr}; 
  else                                                  
    bitmap_ram_dina = 0;
end
//

always @(posedge clk or negedge rst_n) begin  
  if (~rst_n) 
    fifo_bmaptosack_din <= 0;
  else if (state == WAIT && hole_shift_return!=0) //shift目前只支持8位数据，但给输入留了14位
    fifo_bmaptosack_din <= {2'b10,hole_shift_return};
  else if(state == RD_FIFOTOBMAP && fifo_sacktobmap_dout[31:30]== 2'b00 )
    fifo_bmaptosack_din <= {2'b00,6'b0,fb_addr};
  else
    fifo_bmaptosack_din<=0;
end    
always @(posedge clk or negedge rst_n) begin  
  if (~rst_n) 
    fifo_bmaptosackwr_en <= 0;
  else if (state == WAIT && hole_shift_return!=0) //shift目前只支持8位数据，但给输入留了14位
    fifo_bmaptosackwr_en <= 1'b1;
  else if(state == RD_FIFOTOBMAP && fifo_sacktobmap_dout[31:30]== 2'b00 )
    fifo_bmaptosackwr_en <= 1'b1;
  else
    fifo_bmaptosackwr_en<=0;
end   

//////////////wait_cnt
always@(posedge clk or negedge rst_n)begin
  if(rst_n == 1'b0)
    wait_cnt <= 0 ;
  else if (state ==YUDU   && state != next_state)          //state == RD_STATE
    wait_cnt <= 0 ;
  else if ( state ==YUDU)                        //state == RD_STATE
    wait_cnt <= wait_cnt + 1'b1 ;
  else
    wait_cnt <= 0 ;
end
available_block available_block (
.clk(clk),
.rst_n(rst_n),
.pop(pop),
.push(push),
.addr_in(block_addr_in),
.block_cnt(block_cnt),
.addr_out(block_addr_out)
);

single_port_ram_16_256 block_ram_tx(
.clka(clk),            
.wea(bitmap_ram_wea),
.addra(bitmap_ram_addra),           //[7 : 0]          
.dina(bitmap_ram_dina),            //[31 : 0]
.douta(bitmap_ram_douta)            //[31 : 0]
);
endmodule
