`timescale 1ns / 1ps
//对bitmap的格式  
//打孔  {01,aack_valid,sack_valid,aack_shift,sack_shift}          no_return
//找洞  {10,14'b0,hole_shift}                                     return      10 new_hole_shift
//切换  {00,14'b0, 16sack_high_shift}                                                return      00 fb_addr
//预读  {11,22'b0,[7:0]fb_addr}                                        no_return

module sack_control_v41(
input               clk,
input               rst_n,
//重传FIFO
output reg          fiforetxwr_en,
output reg [63:0]   fiforetx_din,
input               fiforetx_full,
///////////////////////////////////////
//可发送fifo
input               fifotosendalmost_full,
output reg          fifotosendwr_en,
output reg [63:0]   fifotosend_din,
input               fifotosend_full,
input               fifotosend_empty,

//ack队列
input               fiforto_tosack_empty,                           
input [63:0]        fiforto_tosack_dout,
output              fiforto_tosackrd_en,

input               fifoack_tosack_empty,                           
input [63:0]        fifoack_tosack_dout,
output              fifoack_tosackrd_en,

output reg          fifo_ack_outwr_en,
output reg  [63:0]  fifo_ack_out_din,
input               fifo_ack_out_full,
input               fifo_ack_outalmost_full,
output  reg [31:0]      fifo1_din , 
output  reg [9:0]       fifo1_addr_din , 
output  reg             fifo1_addrwr_en,
output  reg             fifo1wr_en   ,
/////////////ram port////////////
//从ram里面读取状态，再往fifo里面赛要更新的地址和内容
output reg [13:0]   state_ram_addr     ,
input       [31:0]  state_ram_dout
    );
//
reg                 rd_ack_flag;
reg [3:0]           packet_type;
reg [3:0]           op_type;
///////fifotobmap
wire    [31:0]      fifo_sacktobmap_dout;
wire                fifo_sacktobmap_full;
wire                fifo_sacktobmap_empty;
reg                 fifo_sacktobmapwr_en;
wire                fifo_sacktobmaprd_en;
reg     [31:0]      fifo_sacktobmap_din;
////////////fifo bmap_to sack
wire                fifo_bmaptosackwr_en;
wire                fifo_bmaptosackrd_en;
wire    [15:0]      fifo_bmaptosack_dout;
wire    [15:0]      fifo_bmaptosack_din;
wire                fifo_bmaptosack_full;
wire                fifo_bmaptosack_empty;	

reg     [2:0]       sack_state;     ///gbn_flag , sack_flag , send_state
wire                loss_rec_flag0;
reg                 loss_rec_flag;
reg                 need_fb_flag;
wire                 need_retx_flag;          // 需要重传标志   
 //窗口状态
wire                ca_state;              
reg     [9:0]      cwnd;
reg     [9:0]      cwnd0;
reg     [9:0]      cwnd_cnt;  
reg     [9:0]      sstresh;
//丢包恢复状态
reg                 send_state=0;              //s
reg                 sack_flag=0;
reg                 gbn_flag=0;                    //是否回退到gbn
reg                 rto_flag;                //是否进入rto
reg                 qingpaidui;
reg     [1:0]       update_cwnd_cnt;        //往state_ram里面更新[15:0]cwnd [16]gbn_flag [27:17]dup_ack   同时往send_fifo里塞
reg     [1:0]       update_unack_cnt;        //往state_ram里面更新unack   同时往send_fifo里塞

reg     [2:0]       ack_tx_cnt;
reg     [2:0]       data_tx_cnt;
reg     [3:0]       retx_cnt;
reg     [7:0]       block_used_cnt;
reg     [7:0]       fb_addr;

wire    [14:0]      hole_shift;

wire    [15:0]      avail_sack_high;

reg     [15:0]      pre_flow;
reg     [15:0]      curr_flow;
reg     [23:0]      rx_ack;
reg     [23:0]      rx_sack;
reg     [13:0]      aack_shift;
wire     [13:0]     sack_shift;
wire                sack_valid;
wire                aack_valid;
wire    [13:0]      shift_temp;
reg     [23:0]      sack_high;
reg     [15:0]      sack_high_shift;
reg     [23:0]      unack;
reg     [12:0]      dupack;
reg     [23:0]      highest_retx_wait;
reg     [23:0]      qpid=0;

reg     [31:0]      wait_cnt;
reg     [31:0]      time_cnt0;


parameter WAIT              =    11'b000_0000_0001;         //001			
parameter RD_RTO_FLOW       =    11'b000_0000_0010;         //002
parameter RD_ACK_FLOW       =    11'b000_0000_0100;         //004
parameter SWAP_STATE        =    11'b000_0000_1000;         //008		
parameter SEND_STATE0       =    11'b000_0001_0000;         //010			
parameter SEND_STATE1       =    11'b000_0010_0000;         //020		
parameter UPDATE_STATE      =    11'b000_0100_0000;         //040		
parameter RE_TX             =    11'b000_1000_0000;         //080	
parameter WAIT_RETURN       =    11'b001_0000_0000;         //100
parameter RD_RETURN         =    11'b010_0000_0000;         //200		
//parameter JUDGE             =    11'b000_1000_0000;         //080	
parameter NORMAL            =    1'b1;         
parameter LOSS_REC          =    1'b0;  
reg [10:0]           state;
reg [10:0]           next_state;   
//
always@(posedge clk or negedge rst_n )begin                   
  if(rst_n==0) 
    state <= WAIT;
  else 
    state <= next_state;
end  
always@(*) begin
  case(state)
    WAIT        :begin
      if(~fiforto_tosack_empty && ~fifotosend_full)
        next_state = RD_RTO_FLOW;
      else if(~fifoack_tosack_empty&& ~fifotosend_full)  
        next_state = RD_ACK_FLOW ;
      else
      next_state = WAIT ;  
  end    
  RD_ACK_FLOW     :begin
    if(fifoack_tosack_dout[23:0] == qpid)   //不需要切换
      next_state = UPDATE_STATE;
    else 
      if(sack_flag)                         //前一调连接需要保存位图
        next_state = WAIT_RETURN ;
      else
        next_state = SWAP_STATE;         
end    
  RD_RTO_FLOW     :begin
      if(fiforto_tosack_dout[23:0] == qpid )   
        if( ~sack_flag)                 //不需要切换,不需要重传,直接初始化
          next_state = WAIT;
        else                            //不需要切换,需要重传
          next_state = WAIT_RETURN ;  
      else                              //xuyaoqie
        if(sack_flag)                         //前一调连接需要保存位图
          next_state = WAIT_RETURN ;
        else
          next_state = SWAP_STATE;                     
  end   
  SWAP_STATE        : begin
    if(wait_cnt==4)
      if(rto_flag )             
        if(sack_flag) //rto任务，且丢包了.开始选择重传
          next_state = WAIT_RETURN;
        else      //切过来超时了，未丢包，直接初始化
          next_state = WAIT;
      else           //ack 任务
        next_state = UPDATE_STATE ;
//        case(fifoack_tosack_dout[63:56])
//        8'b0000_0100:                       //远端来数据包
//        8'b0000_0001:                       //远端来ACK
//        8'b0000_0011:                       //远端来SACK
          //shujubao  
    else           
      next_state = SWAP_STATE;
  end    
  UPDATE_STATE  :next_state = SEND_STATE0;
  SEND_STATE0   : next_state = SEND_STATE1;
  SEND_STATE0   : next_state = WAIT;
  WAIT_RETURN  : begin
    if(fifo_bmaptosack_empty)
      next_state = WAIT_RETURN;
    else
      next_state = RD_RETURN;  
  end
  RD_RETURN     : begin
   if(fifo_bmaptosack_dout[15:14]==2'b00 ) 
     next_state = SWAP_STATE;              //返回fb_addr
   else if(fifo_bmaptosack_dout[15:14]==2'b10 )       
     next_state = RE_TX;                   //找洞
//       11           : next_state = WAIT;                    //可用 block数  不过和状态机没啥关系
   else
    next_state = WAIT;
  end    
  RE_TX         : begin
    if(need_retx_flag)
      next_state = WAIT_RETURN;
    else 
      next_state = WAIT;  
  end
  default: next_state = WAIT;
  endcase 
end    
/////*************标志位相关
assign ca_state = sstresh <= cwnd  ;
assign fifoack_tosackrd_en = next_state == RD_ACK_FLOW   ;
assign fiforto_tosackrd_en = next_state == RD_RTO_FLOW   ;
assign avail_sack_high =16'd1000;
//assign hole_shift =highest_retx_wait == unack ? 0:highest_retx_wait-unack-1; 
assign hole_shift =highest_retx_wait-unack;
assign aack_valid = aack_shift!=0;
assign sack_valid = sack_shift!=0;
//assign sack_shift = fifotosack_dout - rx_ack; 
assign need_retx_flag = state!=SWAP_STATE &&~(send_state == NORMAL || highest_retx_wait>=sack_high);
//assign loss_rec_flag0 = dupack >=3 || (state == RD_SACK && fiforto_tosack_dout> unack+2 );
//packet_type
always@(posedge clk or negedge rst_n) begin
  if(rst_n == 1'b0)
    packet_type <= 0 ;
  else if(state ==RD_RTO_FLOW  )
    packet_type<=fiforto_tosack_dout[59:56];
  else if(state ==RD_ACK_FLOW )
    packet_type<=fifoack_tosack_dout[59:56];
  else
    packet_type<=packet_type; 
end  
//
//reg [31:0]      dupack;
always@(posedge clk or negedge rst_n )begin     
  if(~rst_n) 
    dupack <= 0; 
  else
    dupack<=dupack;
end
/////cwnd;
always@(posedge clk or negedge rst_n )begin               
  if(~rst_n) begin
    cwnd_cnt <= 0;
    cwnd <= 16'd4;
  end  
  else begin
    cwnd_cnt<=cwnd_cnt;
    if(cwnd>=4)
      cwnd <= cwnd;
    else
      cwnd <= 4  ;
  end  
end
//reg [31:0]      unack;
always@(posedge clk or negedge rst_n )begin               
  if(~rst_n) 
    unack <=32'd1;
  else if( state ==SWAP_STATE && wait_cnt == 5 )
    unack <= state_ram_dout;
  else
    unack<=unack; 
end
//reg             rto_flag;
always@(posedge clk or negedge rst_n )begin               
  if(~rst_n) 
    rto_flag <= 0;
  else if(state == RD_RTO_FLOW && state!=next_state)
    rto_flag <= fiforto_tosack_dout[24] ;    
  else if(state == WAIT)
    rto_flag <= 0 ;        
  else
    rto_flag<=rto_flag; 
end
always@(posedge clk or negedge rst_n) begin
  if(~rst_n)            //返回ack
    ack_tx_cnt<=0;
  else if( state == UPDATE_STATE &&  (packet_type[0] == 1 ||packet_type[3] == 0))     // && rto_flag
    ack_tx_cnt<=2'b10;  
  else 
    if(ack_tx_cnt!=0)
      ack_tx_cnt<=ack_tx_cnt-1'b1;    
    else  
      ack_tx_cnt<=0; 
end 
//data_tx_cnt  正常更新unack cwnd dupack 的计数器
always@(posedge clk or negedge rst_n) begin
  if(~rst_n)
    data_tx_cnt<=0;
//  else if((state == RD_RTO_FLOW ) && qpid !=fiforto_tosack_dout[15:0] && (packet_type[0] == 1 ||packet_type[3] == 0))     // && rto_flag
  else if(state == RD_RTO_FLOW && fiforto_tosack_dout[23:0] == qpid && ~sack_flag)   //递交rto连接
    data_tx_cnt<=2'b10;  
  else 
    if(data_tx_cnt!=0)
      data_tx_cnt<=data_tx_cnt-1'b1;    
    else  
      data_tx_cnt<=0; 
end 
always@(posedge clk or negedge rst_n) begin
  if(rst_n == 1'b0)
    fifotosend_din <= 0 ;
  else if(data_tx_cnt ==2'b10 ) begin
    fifotosend_din[63:60] <= packet_type;
    fifotosend_din[59:39] <= 0;
    fifotosend_din[38:36] <= {rto_flag,gbn_flag,send_state};
    fifotosend_din[35:24] <= dupack;
    fifotosend_din[23:0] <= qpid; 
  end  
  else if(data_tx_cnt ==1 )
    fifotosend_din <= {30'D0,cwnd,unack}; 
  else
    fifotosend_din <=fifotosend_din;  //[31][30][29][28:13][12:0]
end 
//fifotosendwr_en
always@(posedge clk or negedge rst_n) begin
  if(rst_n == 1'b0)
    fifotosendwr_en <= 0 ;
  else if(data_tx_cnt==2 || data_tx_cnt==1)
    fifotosendwr_en <= 1'b1 ;
  else    
    fifotosendwr_en <= 0 ;
end
//rd_ack_flag
always@(posedge clk or negedge rst_n )begin                    //fifo读
  if(~rst_n) 
    rd_ack_flag <= 0;
  else if(next_state == RD_ACK_FLOW)             //当打不进去的时候，回退gbn
    rd_ack_flag<=1;
  else if(next_state == WAIT)  
    rd_ack_flag <= 0;
  else
    rd_ack_flag <= rd_ack_flag;  
end  
//////////////wait_cnt
always@(posedge clk or negedge rst_n)begin
  if(rst_n == 1'b0)
    wait_cnt <= 0 ;
  else if (state ==SWAP_STATE   && state != next_state)          //state == RD_STATE
    wait_cnt <= 0 ;
  else if ( state ==SWAP_STATE)                        //state == RD_STATE
    wait_cnt <= wait_cnt + 1'b1 ;
  else
    wait_cnt <= 0 ;
end














//*************************************下层
//////////////bitmap   部分///////////////////
fifo_32X32_32_CCDR_SF fifo_sacktobmap (
.clk(clk),
.wr_en(fifo_sacktobmapwr_en),
.rd_en(fifo_sacktobmaprd_en),
.srst(~rst_n),
.dout(fifo_sacktobmap_dout),
.din (fifo_sacktobmap_din),
.full(fifo_sacktobmap_full),
.empty(fifo_sacktobmap_empty)
  );

fifo_16X16_32_CCDR_SF fifo_bmaptosack (
.clk(clk),
.wr_en(fifo_bmaptosackwr_en),
.rd_en(fifo_bmaptosackrd_en),
.srst(~rst_n),
.dout(fifo_bmaptosack_dout),
.din (fifo_bmaptosack_din),
.full(fifo_bmaptosack_full),
.empty(fifo_bmaptosack_empty)
  );
  
//bitmap_control_tx_v4 bitmap_control_tx_v4(
bitmap_control_v7 bitmap_control_v7(
.clk(clk),              //input
.rst_n(rst_n),             //input

.fifo_sacktobmap_empty(fifo_sacktobmap_empty),
.fifo_sacktobmap_dout(fifo_sacktobmap_dout),
.fifo_sacktobmaprd_en(fifo_sacktobmaprd_en),

.fifo_bmaptosackwr_en(fifo_bmaptosackwr_en),
.fifo_bmaptosack_din (fifo_bmaptosack_din),
.fifo_bmaptosack_full(fifo_bmaptosack_full)
);
endmodule
