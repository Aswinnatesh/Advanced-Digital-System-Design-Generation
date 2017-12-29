//-------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------//
// Project Title    : Hardware Generation Tool                                                   ESE 507 [Fall 2017] //
//                                                                                                                   //
// Project Member 1 : Aswin Natesh Venkatesh    [SBU ID: 111582677]                                                  //
// Project Member 2 : Gosakan Srinivasan        [SBU ID: 111579886]                                                  // 
//                                                                                                                   //
// Submission Date  : December 08, 2017                                                                              //
//-------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------//


//=================================================MAIN MODULE=====================================================
module layer_16_8_1_16(clk, reset, s_valid, m_ready, data_in, m_valid, s_ready, data_out);  

parameter N=8; parameter M=16; parameter T=16; parameter P=1;
parameter LOGM=4; parameter LOGW=7; parameter LOGN=3; parameter LOGB=3;

 input clk, reset, s_valid, m_ready;                       // Control Signals
  input signed [T-1:0] data_in;
  output logic signed [T-1:0] data_out; 
  output m_valid, s_ready;
  reg [LOGW-1:0] addr_m,counter;                                        // Memory Address pointer of Input matrix M
  reg [LOGN:0] addr_x;
  reg [LOGM:0] addr_y;                 // Memory Adress pointer of Input Vector and Output Values
  reg [LOGB:0] addr_b;
  logic wr_en_x, wr_en_y, clear_acc;  
  control_module1 c(clk, reset, s_valid,m_ready,addr_y, wr_en_y, addr_x, wr_en_x,addr_m,addr_b, m_valid,s_ready, clear_acc, counter);
  datapath_module1 d(clk, reset, counter, m_ready, clear_acc, data_in, addr_x, addr_m,addr_b, addr_y, wr_en_x, wr_en_y, data_out); 

endmodule

//==================================================CONTROL MODULE==================================================

 module control_module1 (clk, reset, s_valid,m_ready,addr_y, wr_en_y, addr_x, wr_en_x,addr_m,addr_b, m_valid,s_ready, clear_acc, counter);

parameter N=8; parameter M=16; parameter T=16; parameter P=1;
parameter LOGM=4; parameter LOGW=7; parameter LOGN=3; parameter LOGB=3;

  input clk, reset, s_valid, m_ready;
  output logic [LOGW-1:0] addr_m,counter;
  output logic [LOGN:0] addr_x;
  output logic [LOGM:0] addr_y;         
  output logic [LOGB:0] addr_b;
  output logic wr_en_x, wr_en_y, clear_acc, m_valid, s_ready;
  logic write_complete, mac_complete, hold_s2;
  logic [2:0] state, next_state;                                                      // FSM Transition Variables
  logic [2:0] data_displayed;
  logic [1:0] hold_s3;
  logic [LOGW-1:0] row, hold_disp;
  logic [LOGW-1:0] column;
  logic flag;

  always_ff @(posedge clk) begin
    if(reset==1)                                                                      // Reset Signal & Initial State Assignment
      state<=0;                                                               
    else begin
      state<=next_state;                                                              // State Transition Assignment 
    end
  end

  always_comb begin 
      if(state == 0) begin                          // State 0: Wait for Valid Data and Valid Data Signal
        if(s_valid == 1)
          next_state = 1;                  
        else 
          next_state = 0;
        end

      else if(state == 1)begin                      // State 1: Fetch Input Data (Matrix M and Vector X) 
        if(write_complete == 1) 
          next_state = 2;  
        else 
          next_state = 1;
      end

      else if(state == 2)begin                      // State 2: Matrix Vector Multiplication and Accumulation
        if(mac_complete ==1) 
          next_state = 3;    
        else 
          next_state = 2; 
      end

      else if(state == 3) begin                     // State 3: Wait for Master Ready Signal and Display Output  
        if(data_displayed == 1)
          next_state = 0; 
        else 
          next_state = 3;
       end

      else next_state = 0;                          // Wait for Instruction from Testbench
    end

  assign s_ready =  ((state==1) & (write_complete ==0));                              // Assert when system is ready for data input
  assign data_displayed = ((state ==3)&(column ==M/P));                                 // Assert once output is displayed
  assign write_complete = ((state ==1) & (addr_x==N));                                // Assert once input is taken in
  assign clear_acc = (((column==1)|(mac_complete==1))&(state==2));                    // Clear Accumulator Flag


  assign wr_en_x =((state ==1) & ((addr_x>=0) &(addr_x < N)));                         // Asserting Vector X Memory Write Enable
  assign wr_en_y=clear_acc;

  always_ff @(posedge clk) begin
      if(state==0) begin                                    // State 0 Operations: Initialize Address Pointers  
        addr_m <= 0; addr_x <= 0;addr_y <=0;addr_b <=0;               
        row <=0; column <=0; hold_s2 <=0;
        mac_complete <=0; hold_s3 <=0;
        hold_disp <=0; flag <=0; counter <=0;
      end  

      else if ((state == 1)) begin          // State 1 Operations: Shift Matrix and Vector Pointers
          if (s_valid ==1) begin
           if(addr_x < N) 
            addr_x <= addr_x + 1; 
          else
            addr_x <=0;
        end
      end
      else if (state == 2) begin                            // State 2 Operations: Matrix-Vector Multiply and Accumulate 
        if(hold_s2==0) begin
          addr_b <= row;
          hold_s2 <=1;
        end

        else if(row <=(M/P)-1)begin
        if((column<N) &(hold_s2 ==1)) begin
          addr_m<=((N*row)+column);
          addr_x<=column;
          column<=column+1;
      end  
        else begin
          column<=0;
          addr_y <= row; 
          row<=row+1;
          hold_s2 <=0;
          end
        end 
        else begin 
          mac_complete <=1;                                   // Assert when State 2 is complete
        end
      end

    else if (state == 3) begin                                // State 3 Operations: Send Multiplied & Accumulated Values
       if (hold_s3 == 0) begin
        if(flag==0) begin
           addr_y<= column;
           hold_disp <=hold_disp +1;
           if(hold_disp==P-1)flag<=1;  
           hold_s3 <=1;
         end
          else if(flag==1) begin
              flag <=0;
              column <= column +1;
              hold_disp<=0;
          end
        end 
      else if(hold_s3 ==1 & column!=(M/P)) begin
        m_valid <=1;
        hold_s3 <=2;
      end
      else if(m_ready ==0 & hold_s3 ==1)begin
        m_valid <=0; 
        hold_s3<=1;
      end 
      else if(m_ready ==0 & hold_s3 ==2) begin
           // $display(Waiting for m-ready to become 1)
      end 
      else  begin
        m_valid<=0;
        hold_s3<=0;       
        counter<=counter+1;       
      end
    end       
 end
endmodule

//==================================================DATAPATH MODULE==================================================
module datapath_module1 (clk, reset, counter, m_ready, clear_acc, data_in, addr_x, addr_m, addr_b, addr_y, wr_en_x, wr_en_y, data_out);    

parameter N=8; parameter M=16; parameter T=16; parameter P=1;
parameter LOGM=4; parameter LOGW=7; parameter LOGN=3; parameter LOGB=3;

 input m_ready, clk, reset, clear_acc, wr_en_x, wr_en_y;             // Input Signals from contol_module
 input [LOGW-1:0] addr_m,counter;
 input [LOGN:0] addr_x;
 input [LOGM:0] addr_y;         
 input [LOGB:0] addr_b;
 input signed [T-1:0] data_in;                                                  // Input Data 
 output logic signed [T-1:0] data_out;                                         // Output Data
 logic signed [T-1:0] data_x;
 logic signed [T-1:0] data_out0;

 logic signed [T-1:0] data_y1, data_y1_R;
 logic signed [T-1:0] data_m1, data_b1;
 logic signed [T-1:0] data_out1;

 assign data_y1_R= ((wr_en_y==1)&&(data_y1[T-1]==1))? 0:data_y1;

always_comb begin
 if(counter %P == 0) data_out = data_out1;
 else data_out = data_out0;
end

 memory #(T,N,LOGN+1) x(clk, data_in, data_x, addr_x, wr_en_x);                     // Vector Memory Instantiation
 memory #(T,M,LOGM+1) y1(clk, data_y1_R, data_out1, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer_16_8_1_16_B1_rom lb1 (clk, addr_b, data_b1);
 layer_16_8_1_16_W1_rom lw1 (clk, addr_m, data_m1);
 mac ma11(clk, reset, clear_acc,data_b1, data_m1, data_x, data_y1);               // MAC Module Instantiation


endmodule

//==================================================ROM  MODULE=====================================================
module layer_16_8_1_16_W1_rom(clk, addr, z);
   input clk;
   input [6:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd20);
        1: z <= $signed (-16'd58);
        2: z <= $signed (-16'd82);
        3: z <= $signed (-16'd74);
        4: z <= $signed (-16'd113);
        5: z <= $signed (16'd88);
        6: z <= $signed (-16'd98);
        7: z <= $signed (16'd15);
        8: z <= $signed (16'd36);
        9: z <= $signed (16'd96);
        10: z <= $signed (-16'd35);
        11: z <= $signed (-16'd36);
        12: z <= $signed (-16'd56);
        13: z <= $signed (16'd99);
        14: z <= $signed (-16'd90);
        15: z <= $signed (-16'd39);
        16: z <= $signed (16'd35);
        17: z <= $signed (16'd78);
        18: z <= $signed (16'd37);
        19: z <= $signed (-16'd31);
        20: z <= $signed (-16'd128);
        21: z <= $signed (-16'd28);
        22: z <= $signed (-16'd30);
        23: z <= $signed (16'd33);
        24: z <= $signed (-16'd40);
        25: z <= $signed (16'd42);
        26: z <= $signed (16'd16);
        27: z <= $signed (-16'd34);
        28: z <= $signed (-16'd97);
        29: z <= $signed (-16'd114);
        30: z <= $signed (-16'd16);
        31: z <= $signed (16'd12);
        32: z <= $signed (-16'd43);
        33: z <= $signed (16'd30);
        34: z <= $signed (16'd66);
        35: z <= $signed (-16'd28);
        36: z <= $signed (-16'd10);
        37: z <= $signed (16'd96);
        38: z <= $signed (16'd115);
        39: z <= $signed (-16'd102);
        40: z <= $signed (16'd65);
        41: z <= $signed (-16'd47);
        42: z <= $signed (-16'd10);
        43: z <= $signed (-16'd119);
        44: z <= $signed (-16'd76);
        45: z <= $signed (16'd28);
        46: z <= $signed (-16'd30);
        47: z <= $signed (16'd87);
        48: z <= $signed (-16'd21);
        49: z <= $signed (-16'd120);
        50: z <= $signed (-16'd72);
        51: z <= $signed (-16'd21);
        52: z <= $signed (-16'd20);
        53: z <= $signed (16'd26);
        54: z <= $signed (-16'd115);
        55: z <= $signed (16'd69);
        56: z <= $signed (-16'd60);
        57: z <= $signed (16'd29);
        58: z <= $signed (-16'd93);
        59: z <= $signed (-16'd28);
        60: z <= $signed (16'd44);
        61: z <= $signed (16'd19);
        62: z <= $signed (16'd112);
        63: z <= $signed (-16'd127);
        64: z <= $signed (-16'd79);
        65: z <= $signed (16'd50);
        66: z <= $signed (-16'd27);
        67: z <= $signed (16'd39);
        68: z <= $signed (16'd19);
        69: z <= $signed (-16'd40);
        70: z <= $signed (16'd65);
        71: z <= $signed (-16'd44);
        72: z <= $signed (16'd41);
        73: z <= $signed (-16'd73);
        74: z <= $signed (-16'd35);
        75: z <= $signed (16'd93);
        76: z <= $signed (16'd84);
        77: z <= $signed (16'd64);
        78: z <= $signed (16'd52);
        79: z <= $signed (-16'd65);
        80: z <= $signed (16'd72);
        81: z <= $signed (16'd108);
        82: z <= $signed (16'd42);
        83: z <= $signed (-16'd76);
        84: z <= $signed (16'd6);
        85: z <= $signed (16'd55);
        86: z <= $signed (16'd121);
        87: z <= $signed (16'd75);
        88: z <= $signed (-16'd43);
        89: z <= $signed (-16'd99);
        90: z <= $signed (-16'd81);
        91: z <= $signed (-16'd127);
        92: z <= $signed (16'd48);
        93: z <= $signed (-16'd97);
        94: z <= $signed (-16'd126);
        95: z <= $signed (16'd98);
        96: z <= $signed (16'd81);
        97: z <= $signed (-16'd25);
        98: z <= $signed (16'd9);
        99: z <= $signed (-16'd28);
        100: z <= $signed (16'd63);
        101: z <= $signed (-16'd53);
        102: z <= $signed (16'd56);
        103: z <= $signed (-16'd23);
        104: z <= $signed (16'd2);
        105: z <= $signed (-16'd106);
        106: z <= $signed (-16'd58);
        107: z <= $signed (-16'd42);
        108: z <= $signed (16'd86);
        109: z <= $signed (16'd123);
        110: z <= $signed (16'd21);
        111: z <= $signed (16'd30);
        112: z <= $signed (16'd103);
        113: z <= $signed (-16'd64);
        114: z <= $signed (16'd82);
        115: z <= $signed (-16'd18);
        116: z <= $signed (16'd119);
        117: z <= $signed (16'd76);
        118: z <= $signed (-16'd71);
        119: z <= $signed (-16'd52);
        120: z <= $signed (16'd105);
        121: z <= $signed (-16'd24);
        122: z <= $signed (-16'd51);
        123: z <= $signed (16'd25);
        124: z <= $signed (16'd7);
        125: z <= $signed (-16'd49);
        126: z <= $signed (-16'd5);
        127: z <= $signed (-16'd40);
      endcase
   end
endmodule

module layer_16_8_1_16_B1_rom(clk, addr, z);
   input clk;
   input [3:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd54);
        1: z <= $signed (-16'd123);
        2: z <= $signed (16'd61);
        3: z <= $signed (-16'd10);
        4: z <= $signed (-16'd48);
        5: z <= $signed (-16'd11);
        6: z <= $signed (16'd95);
        7: z <= $signed (16'd82);
        8: z <= $signed (16'd11);
        9: z <= $signed (-16'd91);
        10: z <= $signed (-16'd87);
        11: z <= $signed (-16'd31);
        12: z <= $signed (-16'd96);
        13: z <= $signed (16'd62);
        14: z <= $signed (16'd127);
        15: z <= $signed (-16'd120);
      endcase
   end
endmodule

//==================================================MAC  MODULE=====================================================

module mac(clk, reset, clear_acc,data_b, data_m, data_x, data_y);

parameter T=16;

input clk, reset, clear_acc;
input signed [T-1:0] data_m, data_x;
input signed [T-1:0] data_b;
output logic signed [T-1:0] data_y;
logic signed [T-1:0] product, sum, prod2;

  always_ff @(posedge clk) begin
    if(clear_acc == 1)   begin                  // Clearing Accumulated Value 
      data_y <=data_b;
     end
    else 
      data_y<=sum;
  end 


   always_ff @(posedge clk) begin
      if (reset == 1'b1) begin
      product<=0;
      end        
      else begin
        if(clear_acc ==1)
          product <=0;
        else
      product <= data_m * data_x;                     // Multiplication Operation   
    end
   end

   assign sum = product + data_y;

endmodule


//==================================================MEMORY  MODULE=====================================================

module memory(clk, data_in, data_out, addr, wr_en);
parameter WIDTH=16, SIZE=64, LOGSIZE=6; input [WIDTH-1:0] data_in;
output logic [WIDTH-1:0] data_out; input [LOGSIZE-1:0] addr;
input clk, wr_en;
logic [SIZE-1:0][WIDTH-1:0] mem;
always_ff @(posedge clk)
 begin 
data_out <= mem[addr];
if (wr_en)
mem[addr] <= data_in; end
endmodule

