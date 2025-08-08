`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
//��·���л�����·��ʱ ��shift����ͳһ��¼��һ���أ�������ֻ�����¼�������������
//������SACK/�����¼��                -β����¼                                0001, state_addr[7:0] tail_addr[7:0], shift[7:0]
//                                                      -��ͷ��ʼ��¼                         0000, 0            , shift[7:0]
//���������ն��յ��ش�����ѯλͼ     -λͼ��ѯ-����RTO             0010, fb_addr, shift [7:0]    ---����FFFFFF��־����
//�������ͷ�λͼ                                -�����ͷ�                              0011, fb_addr,0[7:0]      -��ͷ�ͷ�����һ��0
//                                                                                                      0011, fb_addr,shift[7:0]    -��ͷ�ͷ�ָ������

//////////////////////////////////////////////////////////////////////////////////


module bitmap_ctrl_v30(
input                   clk,
input                   rst_n,

///////////FIFOTOBMAP
input       [31:0]    fifo_sacktobmap_dout,            //[14:0] shift 15 1sack 0aack
input                    fifo_sacktobmap_empty,
output                  fifo_sacktobmaprd_en,
//////////FIFOTOSACK
input                       fifo_bmaptosack_full,       //��δ��дfullʱֹͣд����߼�
output reg  [31:0]  fifo_bmaptosack_din,
output reg              fifo_bmaptosackwr_en
);

///״̬��
reg     [11:0]  state;
reg     [11:0]  next_state;

reg     [31:0]  search_shift;
reg     [31:0]  wait_cnt;

///////////////////����block��ַfifo.....................
wire               pop;
reg                 push;
wire [8:0]       block_cnt; 
wire [7:0]       block_addr_out;
reg   [7:0]       block_addr_in;
reg   [7:0]       block_cnt_out;
///////////////////bitmap_block_ram
reg                  bitmap_ram_wea;
reg  [7:0]         bitmap_ram_addra;      
reg  [15:0]       bitmap_ram_dina;
wire [15:0]      bitmap_ram_doutb;
reg   [7:0]       bitmap_ram_addrb;      
reg   [7:0]       bitmap_ram_addrb0;   
////////////////////////
reg     [15:0]      bmap_block_unit;
wire    [7:0]       block_need;
reg     [7:0]        record_cnt;
reg    [7:0]         shift_in;
reg      [7:0]       bmap_addr;
wire    [7:0]       bmap_offset;
wire    [7:0]       bmap_shift;
reg     [7:0]        rcd_left;
reg     [7:0]       pop_cnt;
reg                    overflow_flag;
reg     [7:0]       shift_return;
reg                    return_flag;
parameter WAIT                         =12'b0000_0000_0001 ;                 //001
parameter RD_FIFOTOBMAP     =12'b0000_0000_0010 ;                 //002 
parameter RCD_F0                     =12'b0000_0000_0100 ;                 //004         ��ͷ��¼��һ��
parameter RCD_FN                    =12'b0000_0000_1000 ;                 //008         ��ͷ��¼����
parameter RCD_T0                    =12'b0000_0001_0000 ;                 //010         β����¼��һ��
parameter RCD_TN                   =12'b0000_0010_0000 ;                 //020         β����¼����
parameter BMAP_SRC0            =12'b0000_0100_0000 ;                 //040          �ش�����ѯ
parameter BMAP_SRCN           =12'b0000_1000_0000 ;                 //080          �ش�����ѯ
parameter BMAP_RLS0              =12'b0001_0000_0000 ;                //100          λͼ�ͷ�
parameter BMAP_RLSN             =12'b0010_0000_0000 ;                //200          λͼ�ͷ�

/////////////////״̬��////////////////////////////
always @(posedge clk or negedge rst_n)begin
  if(~rst_n)
    state<=WAIT;
  else 
    state<=next_state;  
end

always @(*)begin
    case(state)
    WAIT                    :begin
        if(~fifo_sacktobmap_empty)
            next_state = RD_FIFOTOBMAP ;
        else  
          next_state = WAIT ;   
    end     
    RD_FIFOTOBMAP  :begin
        if(fifo_sacktobmap_dout[31:28] == 4'b0000)
            next_state =  RCD_F0 ; 
        else if (fifo_sacktobmap_dout[31:28] ==4'b0001)   
            next_state =  RCD_T0 ;      
        else if (fifo_sacktobmap_dout[31:28] ==4'b0010)   
            next_state =  BMAP_SRC0 ;        
        else if (fifo_sacktobmap_dout[31:28] ==4'b0011)  
            next_state = BMAP_RLS0 ; 
        else
            next_state = WAIT ; 
    end
    RCD_F0    :begin
        if(fifo_sacktobmap_dout[7:0]>7)
             next_state =  RCD_FN ; 
        else
            next_state = WAIT ;  
    end    
    RCD_FN    :begin
        if(rcd_left > 7)
             next_state =  RCD_FN ; 
        else
            next_state = WAIT ;  
    end        
    RCD_T0    :begin
        if(bmap_shift>8)
             next_state =  RCD_TN ; 
        else
            next_state = WAIT ;  
    end    
    RCD_TN    :begin
        if(rcd_left > 7)
             next_state =  RCD_TN ; 
        else
            next_state = WAIT ;  
    end        
    BMAP_SRC0    :begin
        if(bitmap_ram_doutb[7:0])
            next_state = BMAP_SRC0 ;  
       else
            next_state = BMAP_SRCN ;   
    end    
    BMAP_SRCN   :begin
        next_state = WAIT ;  
    end        
    BMAP_RLS0   :begin
        if(bitmap_ram_doutb[7:0]==0 )
            next_state = WAIT ;
        else if(bitmap_ram_doutb[15]&& bitmap_ram_doutb[14]&&bitmap_ram_doutb[13]&&bitmap_ram_doutb[12]&&bitmap_ram_doutb[11]&&bitmap_ram_doutb[10])    
             next_state = BMAP_RLSN ;  
        else    
            next_state = WAIT ;    
    end    
    BMAP_RLSN   :begin
        if(bitmap_ram_doutb[7:0]==0 )
            next_state = WAIT ;
        else if(bitmap_ram_doutb[15:8]==8'b1111_1111)    
           next_state = BMAP_RLSN ;  
        else    
            next_state = WAIT ;  
    end        
    default        : next_state = WAIT ;
    endcase
end   

assign fifo_sacktobmaprd_en = (state == WAIT && ~fifo_sacktobmap_empty);
assign block_need=fifo_sacktobmap_dout[7:3]+1;
assign pop =(next_state== RCD_F0 && state != next_state ) || (next_state== RCD_FN && pop_cnt)||( next_state== RCD_TN ) ;    //&& pop_cnt

assign bmap_offset = (bitmap_ram_doutb[15]) ? 4'd8 :(bitmap_ram_doutb[14]) ? 4'd7 :(bitmap_ram_doutb[13]) ? 4'd6 :(bitmap_ram_doutb[12]) ? 4'd5 :
                                     (bitmap_ram_doutb[11]) ? 4'd4 :(bitmap_ram_doutb[10]) ? 4'd3 :(bitmap_ram_doutb[9]) ? 4'd2 :(bitmap_ram_doutb[8]) ?4'd1:4'd0;         

//����ƫ����                            //����0����Ĭ�������һλ������-1
assign bmap_shift=bmap_offset+fifo_sacktobmap_dout[7:0];
////λͼ����������
//always @(posedge clk or negedge rst_n) begin  
//  if (~rst_n) 
//    record_cnt <= 0;
//  else if (state == RD_FIFOTOBMAP && next_state == RCD_F ) //shiftĿǰֻ֧��8λ���ݣ�������������14λ
//    record_cnt <= fifo_sacktobmap_dout[7:3]+1;   
//  else if (state == RCD_F  && record_cnt) 
//    record_cnt <= record_cnt-1;   
// else if (state == RCD_T  && wait_cnt ==0) 
//     record_cnt <=bmap_shift[7:3];
//  else
//    record_cnt <= record_cnt;
//end
//////////rcd_left
always @(posedge clk or negedge rst_n) begin  
    if (~rst_n) 
        rcd_left <= 0;
    else if (state == RD_FIFOTOBMAP&&  next_state == RCD_F0 ) //shiftĿǰֻ֧��8λ���ݣ�������������14λ
        rcd_left <= fifo_sacktobmap_dout[7:0];           
    else if(state == RD_FIFOTOBMAP&&   next_state == RCD_T0)    
         rcd_left <= bmap_offset+fifo_sacktobmap_dout[7:0]; 
    else if (rcd_left>7) //shiftĿǰֻ֧��8λ���ݣ�������������14λ
        rcd_left <=rcd_left-8;   
    else
        rcd_left <= 0;
end
////pop_cnt
always @(posedge clk or negedge rst_n) begin  
    if (~rst_n) 
        pop_cnt <= 0;
    else if (state == RD_FIFOTOBMAP && next_state == RCD_F0 ) //shiftĿǰֻ֧��8λ���ݣ�������������14λ
        pop_cnt <= fifo_sacktobmap_dout[7:3];               //��ʼ�׶��Ե���һ�����Բ������1
    else if(state == RCD_T0 && bmap_shift>8&& wait_cnt==0)    
      pop_cnt<=  bmap_shift[7:3];
    else if( pop_cnt) //shiftĿǰֻ֧��8λ���ݣ�������������14λ        (state == RCD_F0 || state == RCD_T0) &&
        pop_cnt <=pop_cnt-1;   
    else
        pop_cnt <= pop_cnt;
end
//push
always @(posedge clk or negedge rst_n) begin  
    if (~rst_n) 
        push <= 0;
    else if (state ==BMAP_RLS0 ) 
        if(bitmap_ram_doutb[7:0]==0  )//ֻ��һ��bmap
            case(bitmap_ram_doutb[15:9])   
                7'b0000_001: push<=1;
                7'b0000_011: push<=1;
                7'b0000_111: push<=1;
                7'b0001_111: push<=1;
                7'b0011_111: push<=1;
                7'b0111_111: push<=1;
                7'b1111_111: push<=1;
                default: push<=0;
            endcase
        else if(bitmap_ram_doutb[15:9]==7'b111_1111)          
            push <=1;       
        else
            push <=0;         
    else if(state ==BMAP_RLSN)  
         if(bitmap_ram_doutb[7:0]==0  )//ֻ��һ��bmap
            case(bitmap_ram_doutb[15:8])   
                8'b0000_0001: push<=1;
                8'b0000_0011: push<=1;
                8'b0000_0111: push<=1;
                8'b0001_1111: push<=1;
                8'b0011_1111: push<=1;
                8'b0111_1111: push<=1;
                8'b1111_1111: push<=1;
                default: push<=0;
            endcase                    
         else if(bitmap_ram_doutb[15:8]==8'b1111_1111)    
            push <=1;     
    else
        push <= 0;
end
/////block_addr_in
always @(posedge clk or negedge rst_n) begin  
    if (~rst_n) 
        block_addr_in <= 0;
    else if (state ==BMAP_RLS0 ) 
        if(bitmap_ram_doutb[7:0]==0  )//ֻ��һ��bmap
            case(bitmap_ram_doutb[15:9])   
                7'b0000_001: block_addr_in<=bitmap_ram_addrb0;
                7'b0000_011: block_addr_in<=bitmap_ram_addrb0;
                7'b0000_111: block_addr_in<=bitmap_ram_addrb0;
                7'b0001_111: block_addr_in<=bitmap_ram_addrb0;
                7'b0011_111: block_addr_in<=bitmap_ram_addrb0;
                7'b0111_111: block_addr_in<=bitmap_ram_addrb0;
                7'b1111_111: block_addr_in<=bitmap_ram_addrb0;
                default: block_addr_in<=block_addr_in;
            endcase
        else if(bitmap_ram_doutb[15:9]==7'b111_1111)          
            block_addr_in <=bitmap_ram_addrb0;       
        else
            block_addr_in <=block_addr_in;         
    else if(state ==BMAP_RLSN)  
         if(bitmap_ram_doutb[7:0]==0  )//ֻ��һ��bmap
            case(bitmap_ram_doutb[15:8])   
                8'b0000_0001: block_addr_in<=bitmap_ram_addrb0;
                8'b0000_0011: block_addr_in<=bitmap_ram_addrb0;
                8'b0000_0111: block_addr_in<=bitmap_ram_addrb0;
                8'b0001_1111: block_addr_in<=bitmap_ram_addrb0;
                8'b0011_1111: block_addr_in<=bitmap_ram_addrb0;
                8'b0111_1111: block_addr_in<=bitmap_ram_addrb0;
                8'b1111_1111: block_addr_in<=bitmap_ram_addrb0;
                default: block_addr_in<=block_addr_in;
            endcase                    
         else if(bitmap_ram_doutb[15:8]==8'b1111_1111)    
            block_addr_in <=bitmap_ram_addrb0;     
    else
        block_addr_in <= block_addr_in;
end
//bitmap_ram_addrb0
always @(posedge clk or negedge rst_n) begin     //////д
    if (~rst_n)   
        bitmap_ram_addrb0<=0;               
    else                                                  
        bitmap_ram_addrb0<=bitmap_ram_addrb;        
end
////shift_return
always @(posedge clk or negedge rst_n) begin  
    if (~rst_n) 
        shift_return <= 0;
    else if (state == BMAP_SRC0 &&  wait_cnt== 0) //shiftĿǰֻ֧��8λ���ݣ�������������14λ
        if(bitmap_ram_doutb[8]==0)
            shift_return <= shift_return+8;  
        else if(bitmap_ram_doutb[9]==0)
            shift_return <= shift_return+7;  
        else if(bitmap_ram_doutb[10]==0)
            shift_return <= shift_return+6;  
        else if(bitmap_ram_doutb[11]==0)
            shift_return <= shift_return+5; 
        else if(bitmap_ram_doutb[12]==0)
            shift_return <= shift_return+4; 
        else if(bitmap_ram_doutb[13]==0)
            shift_return <= shift_return+3; 
        else if(bitmap_ram_doutb[14]==0)
            shift_return <= shift_return+2; 
        else if(bitmap_ram_doutb[15]==0)
            shift_return <= shift_return+1; 
        else 
            shift_return <= shift_return;             
    else if (state == BMAP_SRC0 ) //shiftĿǰֻ֧��8λ���ݣ�������������14λ            
        shift_return <= shift_return+8;  
    else if(state ==BMAP_RLS0 )     
        if(bitmap_ram_doutb[8]==0)
            if(bitmap_ram_doutb[15:9]==7'b111_1111)
                shift_return <=8;
            else if(bitmap_ram_doutb[14:9]==6'b11_1111)
                shift_return <=7;
            else if(bitmap_ram_doutb[13:9]==5'b1_1111)
               shift_return <=6;
            else if(bitmap_ram_doutb[12:9]==4'b1111)
                shift_return <=5;
            else if(bitmap_ram_doutb[11:9]==3'b111)
                shift_return <=4;
            else if(bitmap_ram_doutb[10:9]==2'b11)
                shift_return <=3;               
            else if(bitmap_ram_doutb[9]==1'b1)
                shift_return <=2;                      
            else
                shift_return <=1;                      
        else if(bitmap_ram_doutb[9]==0)
            if(bitmap_ram_doutb[15:10]==6'b111111)
                shift_return <=7;
            else if(bitmap_ram_doutb[14:10]==5'b11111)
                shift_return <=6;
            else if(bitmap_ram_doutb[13:10]==4'b1111)
               shift_return <=5;
            else if(bitmap_ram_doutb[12:10]==3'b111)
                shift_return <=4;
            else if(bitmap_ram_doutb[11:10]==2'b11)
                shift_return <=3;
            else if(bitmap_ram_doutb[10]==1'b1)
                shift_return <=2;               
            else 
                shift_return <=1;                                
        else if(bitmap_ram_doutb[10]==0)
            if(bitmap_ram_doutb[15:11]==5'b11111)
                shift_return <=6;
            else if(bitmap_ram_doutb[14:11]==4'b1111)
                shift_return <=5;
            else if(bitmap_ram_doutb[13:11]==3'b111)
               shift_return <=4;
            else if(bitmap_ram_doutb[12:11]==2'b11)
                shift_return <=3;
            else if(bitmap_ram_doutb[11]==1'b1)
                shift_return <=2;        
            else 
                shift_return <=1;                           
        else if(bitmap_ram_doutb[11]==0)
            if(bitmap_ram_doutb[15:12]==4'b1111)
                shift_return <=5;
            else if(bitmap_ram_doutb[14:12]==3'b111)
                shift_return <=4;
            else if(bitmap_ram_doutb[13:12]==2'b11)
               shift_return <=3;
            else if(bitmap_ram_doutb[12]==1)
                shift_return <=2;
            else 
                shift_return <=1;                 
        else if(bitmap_ram_doutb[12]==0)
            if(bitmap_ram_doutb[15:13]==3'b111)
                shift_return <=4;
            else if(bitmap_ram_doutb[14:13]==2'b11)
                shift_return <=3;
            else if(bitmap_ram_doutb[13]==1'b1)
               shift_return <=2;
            else 
                shift_return <=1;              
        else if(bitmap_ram_doutb[13]==0)
            if(bitmap_ram_doutb[15:14]==2'b11)
                shift_return <=3;
            else if(bitmap_ram_doutb[14]==1'b1)
               shift_return <=2;
            else 
                shift_return <=1;            
        else if(bitmap_ram_doutb[14]==0)
            if(bitmap_ram_doutb[15]==1'b1)
                shift_return <=2;
            else 
                shift_return <=1;          
        else
              shift_return <=1;        
    else if(state ==BMAP_RLSN )
            if(bitmap_ram_doutb[15:8]==8'b1111_1111)         
                 shift_return <=shift_return+8;
            else if(bitmap_ram_doutb[14:8]==7'b111_1111)
                shift_return <=shift_return+1;   
            else if(bitmap_ram_doutb[13:8]==6'b11_1111)
                shift_return <=shift_return+1;   
            else if(bitmap_ram_doutb[12:8]==5'b1_1111)
               shift_return <=shift_return+1;   
            else if(bitmap_ram_doutb[11:8]==4'b1111)
                shift_return <=shift_return+1;   
            else if(bitmap_ram_doutb[10:8]==3'b111)
               shift_return <=shift_return+1;   
            else if(bitmap_ram_doutb[9:8]==2'b11)
                shift_return <=shift_return+1;                
            else if(bitmap_ram_doutb[8]==1'b1)
                shift_return <=shift_return+1;                      
            else
                shift_return <=shift_return;                   
    else
        shift_return <= 0;
end
//return_flag
always @(*) begin     //////д
    if (~rst_n)   
        return_flag=0;      
    else if (state==BMAP_SRC0&& shift_return && bitmap_ram_doutb[7:0]==0 )            //���һƬ && !bitmap_ram_doutb[15:9]
        if((bitmap_ram_doutb[8]==0 && bitmap_ram_doutb[15:9])||(bitmap_ram_doutb[9]==0 && bitmap_ram_doutb[15:10])
             ||(bitmap_ram_doutb[10]==0 && bitmap_ram_doutb[15:11]) ||(bitmap_ram_doutb[11]==0 && bitmap_ram_doutb[15:12])
             ||(bitmap_ram_doutb[12]==0 && bitmap_ram_doutb[15:13])||(bitmap_ram_doutb[13]==0 && bitmap_ram_doutb[15:14])
             ||(bitmap_ram_doutb[14]==0 && bitmap_ram_doutb[15]))
            return_flag=1;                  
        else 
        return_flag=0;                             
    else if (state==BMAP_SRC0 && shift_return  && !bitmap_ram_doutb[15:8])  
         return_flag=1;    
    else if (state==BMAP_SRC0 && shift_return && wait_cnt== 0 && !bitmap_ram_doutb[15:9])            
        return_flag=1;            
    else                                                  
        return_flag=0;        
end
// fifo_bmaptosack_din,     
always @(posedge clk or negedge rst_n) begin     //////д
    if (~rst_n)   
        fifo_bmaptosack_din<=0;               
   else if (return_flag && state ==  BMAP_SRC0 && state!=next_state)  
         fifo_bmaptosack_din<={16'hffff,shift_return,bitmap_ram_doutb[15:8]};                  
   else if (return_flag )  
         fifo_bmaptosack_din<={16'h0,shift_return,bitmap_ram_doutb[15:8]};  
    else                                                  
        fifo_bmaptosack_din<=0;        
end
// fifo_bmaptosackwr_en 
always @(posedge clk or negedge rst_n) begin     //////д
    if (~rst_n)   
        fifo_bmaptosackwr_en<=0;                               
   else if (return_flag )  
         fifo_bmaptosackwr_en<=1;  
    else                                                  
        fifo_bmaptosackwr_en<=0;        
end
////��ǰλͼƫ����
always @(posedge clk or negedge rst_n) begin     //////д
    if (~rst_n)   
        shift_in<=0;                               
    else if (state==RCD_T0 )                //дfb����block�ĵ�ַ
        shift_in <= fifo_sacktobmap_dout[7:0]+bmap_offset-8;
   else if(shift_in>8)
        shift_in<=shift_in - 8;     
    else                                                  
        shift_in<=0;        
end
////bitmap_ram_addra �����ַ
always @(posedge clk or negedge rst_n) begin     //////д
    if (~rst_n)   
        bitmap_ram_addra<=0;                               
    else if (state==RCD_F0 ||state==RCD_FN  ||state==RCD_TN )                //дfb����block�ĵ�ַ
        bitmap_ram_addra <= bmap_addr;
    else if(state == RD_FIFOTOBMAP && next_state == RCD_T0)
        bitmap_ram_addra<= fifo_sacktobmap_dout[15:8];
    else                                                  
    bitmap_ram_addra <= bitmap_ram_addra;
end
//////bitmap_ram_addrb �����ַ
always @(*) begin     //////д
    if (~rst_n)   
        bitmap_ram_addrb=0;                               
    else if (state == RD_FIFOTOBMAP ||next_state==RCD_F0 || state==RCD_FN || (state == RD_FIFOTOBMAP && next_state ==BMAP_SRC0))                //дfb����block�ĵ�ַ
        bitmap_ram_addrb<= fifo_sacktobmap_dout[15:8];
    else if(state == BMAP_SRC0 && state == next_state)    
        bitmap_ram_addrb<=  bitmap_ram_doutb[7:0];  
    else                                                  
    bitmap_ram_addrb <= bitmap_ram_doutb[7:0];
end
////bitmap_ram_dina  ��������д��
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) 
        bitmap_ram_dina[15:8]<=0; 
    else if (state == RCD_F0  && next_state ==  WAIT )                //ͷд�룬����һ��     
        case(fifo_sacktobmap_dout[2:0])
            3'd7:bitmap_ram_dina[15:8]<=8'b1111_1110;    
            3'd6:bitmap_ram_dina[15:8]<=8'b0111_1110;    
            3'd5:bitmap_ram_dina[15:8]<=8'b0011_1110;    
            3'd4:bitmap_ram_dina[15:8]<=8'b0001_1110;    
            3'd3:bitmap_ram_dina[15:8]<=8'b0000_1110;    
            3'd2:bitmap_ram_dina[15:8]<=8'b0000_0110;    
            3'd1:bitmap_ram_dina[15:8]<=8'b0000_0010;      
            default: bitmap_ram_dina[15:8]<=8'd0;   
        endcase          
    else if (state == RCD_F0  && next_state ==  RCD_FN )                //ͷд�룬������
        bitmap_ram_dina[15:8]<=8'b1111_1110;    
    else if( state==RCD_FN)      
        if(rcd_left>8)
             bitmap_ram_dina[15:8]<=8'b1111_1111;            
        else
            case(rcd_left)
                4'd8:bitmap_ram_dina[15:8]<=8'b1111_1111;    
                4'd7:bitmap_ram_dina[15:8]<=8'b0111_1111;    
                4'd6:bitmap_ram_dina[15:8]<=8'b0011_1111;    
                4'd5:bitmap_ram_dina[15:8]<=8'b0001_1111;    
                4'd4:bitmap_ram_dina[15:8]<=8'b0000_1111;    
                4'd3:bitmap_ram_dina[15:8]<=8'b0000_0111;    
                4'd2:bitmap_ram_dina[15:8]<=8'b0000_0011;    
                4'd1:bitmap_ram_dina[15:8]<=8'b0000_0001;      
                default: bitmap_ram_dina[15:8]<=8'd0;   
        endcase
    else if( state==RCD_T0 && next_state ==  WAIT)      // ����Ҫ��λͼ��
        case(bmap_shift)
            8'd8:bitmap_ram_dina[15:8]<= bitmap_ram_doutb[15:8] | 8'b1000_0000;
            8'd7:bitmap_ram_dina[15:8]<= bitmap_ram_doutb[15:8] | 8'b0100_0000;
            8'd6:bitmap_ram_dina[15:8]<= bitmap_ram_doutb[15:8] | 8'b0010_0000;
            8'd5:bitmap_ram_dina[15:8]<= bitmap_ram_doutb[15:8] | 8'b0001_0000;
            8'd4:bitmap_ram_dina[15:8]<= bitmap_ram_doutb[15:8] | 8'b0000_1000;
            8'd3:bitmap_ram_dina[15:8]<= bitmap_ram_doutb[15:8] | 8'b0000_0100;
            8'd2:bitmap_ram_dina[15:8]<= bitmap_ram_doutb[15:8] | 8'b0000_0010;
            8'd1:bitmap_ram_dina[15:8]<= bitmap_ram_doutb[15:8] | 8'b0000_0001;
            default: bitmap_ram_dina[15:8]<=bitmap_ram_doutb[15:8];   
        endcase        
     else if(state == RCD_TN &&  state == next_state)   // ��Ҫ��λͼ��
        bitmap_ram_dina[15:8]<= 0;
     else if(state == RCD_TN &&  state != next_state)       //������βλͼ
        case(shift_in)
            8'd8:bitmap_ram_dina[15:8]<= 8'b1000_0000;
            8'd7:bitmap_ram_dina[15:8]<=  8'b0100_0000;
            8'd6:bitmap_ram_dina[15:8]<=  8'b0010_0000;
            8'd5:bitmap_ram_dina[15:8]<=  8'b0001_0000;
            8'd4:bitmap_ram_dina[15:8]<=  8'b0000_1000;
            8'd3:bitmap_ram_dina[15:8]<=  8'b0000_0100;
            8'd2:bitmap_ram_dina[15:8]<=  8'b0000_0010;
            8'd1:bitmap_ram_dina[15:8]<=  8'b0000_0001;
            default: bitmap_ram_dina[15:8]<=bitmap_ram_doutb[15:8];   
        endcase                    
    else                                                  
     bitmap_ram_dina[15:8]<=bitmap_ram_dina[15:8]; 
end



always @(posedge clk or negedge rst_n) begin
    if (~rst_n) 
        bitmap_ram_dina[7:0]<=0; 
    else if (state==RCD_F0 && next_state ==  RCD_FN )            
        bitmap_ram_dina[7:0]<=block_addr_out;   
    else if(state==RCD_FN)
        if( rcd_left>7)
            bitmap_ram_dina[7:0]<=block_addr_out;         
        else
            bitmap_ram_dina[7:0]<=0;     
    else if (state==RCD_T0 && overflow_flag )         
          bitmap_ram_dina[7:0]<=block_addr_out;        
    else if(state == RCD_TN &&  state == next_state)   
          bitmap_ram_dina[7:0]<=block_addr_out;             
  else                                                  
    bitmap_ram_dina[7:0] = 0;
end
///////bitmap_ram_wea
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) 
        bitmap_ram_wea<=0; 
    else if (state==RCD_F0)                //дfb����block�ĵ�ַ
        bitmap_ram_wea<=1; 
    else if (state==RCD_FN && rcd_left )                //дfb����block�ĵ�ַ
        bitmap_ram_wea<=1;         
   else if( state==RCD_T0 || state ==RCD_TN )           
          bitmap_ram_wea<=1; 
  else                                                  
        bitmap_ram_wea<=0; 
end

////////////bmap_addr
always@(posedge clk or negedge rst_n)begin
    if(rst_n == 1'b0)
        bmap_addr <= 0 ;
    else
        bmap_addr <= block_addr_out ;
end


/////////////overflow_flag
always@(*)begin
    if(rst_n == 1'b0)
        overflow_flag = 0 ;
    else if(state == RCD_T0)
        if (bmap_shift>8)
            overflow_flag = 1 ;
        else
            overflow_flag = 0 ;    
   else
        overflow_flag = 0 ;         
end
//////////////wait_cnt
always@(posedge clk or negedge rst_n)begin
  if(rst_n == 1'b0)
    wait_cnt <= 0 ;
  else if ( state != next_state)          //state == RD_STATE
    wait_cnt <= 0 ;
  else
    wait_cnt <= wait_cnt+1 ;
end
////������ͨ�ĺ����ٸĻ���
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
.clka         (clk),
.wea        (bitmap_ram_wea),
.addra     (bitmap_ram_addra),
.dina       (bitmap_ram_dina),
.clkb       (clk),
.addrb    (bitmap_ram_addrb),
.doutb    (bitmap_ram_doutb)
);
endmodule
