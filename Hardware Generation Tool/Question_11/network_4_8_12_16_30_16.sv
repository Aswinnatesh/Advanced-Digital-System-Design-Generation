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


module network_4_8_12_16_30_16(clk, reset, s_valid, m_ready, data_in, m_valid, s_ready, data_out);

parameter T=16;

  input clk, reset, s_valid, m_ready; // Control Signals 
  input signed [T-1:0] data_in;
  output logic signed [T-1:0] data_out;
  output m_valid, s_ready;
  logic MS_ready_l2, MS_ready_l3;   // Interconnecting Layer parameters  
  logic MS_valid_l2, MS_valid_l3;
  logic signed [T-1:0] data_IO_l2, data_IO_l3;

layer1_8_4_8_16 Net1(clk, reset, s_valid, MS_ready_l2, data_in, MS_valid_l2, s_ready, data_IO_l2);   // Layer 1 GPIO
layer2_12_8_6_16 Net2(clk, reset, MS_valid_l2, MS_ready_l3, data_IO_l2, MS_valid_l3, MS_ready_l2, data_IO_l3);   // Layer 2 GPIO
layer3_16_12_16_16 Net3(clk, reset, MS_valid_l3, m_ready, data_IO_l3, m_valid, MS_ready_l3, data_out);   // Layer 3 GPIO

endmodule
//=================================================MAIN MODULE=====================================================
module layer1_8_4_8_16(clk, reset, s_valid, m_ready, data_in, m_valid, s_ready, data_out);  

parameter N=4; parameter M=8; parameter T=16; parameter P=8;
parameter LOGM=3; parameter LOGW=2; parameter LOGN=2; parameter LOGB=1;

 input clk, reset, s_valid, m_ready;                       // Control Signals
  input signed [T-1:0] data_in;
  output logic signed [T-1:0] data_out; 
  output m_valid, s_ready;
  reg [LOGW:0] addr_m,counter;                                        // Memory Address pointer of Input matrix M
  reg [LOGN:0] addr_x;
  reg [LOGM:0] addr_y;                 // Memory Adress pointer of Input Vector and Output Values
  reg [LOGB:0] addr_b;
  logic wr_en_x, wr_en_y, clear_acc;  
  control_module1 c(clk, reset, s_valid,m_ready,addr_y, wr_en_y, addr_x, wr_en_x,addr_m,addr_b, m_valid,s_ready, clear_acc, counter);
  datapath_module1 d(clk, reset, counter, m_ready, clear_acc, data_in, addr_x, addr_m,addr_b, addr_y, wr_en_x, wr_en_y, data_out); 

endmodule

//==================================================CONTROL MODULE==================================================

 module control_module1 (clk, reset, s_valid,m_ready,addr_y, wr_en_y, addr_x, wr_en_x,addr_m,addr_b, m_valid,s_ready, clear_acc, counter);

parameter N=4; parameter M=8; parameter T=16; parameter P=8;
parameter LOGM=3; parameter LOGW=2; parameter LOGN=2; parameter LOGB=1;

  input clk, reset, s_valid, m_ready;
  output logic [LOGW:0] addr_m,counter;
  output logic [LOGN:0] addr_x;
  output logic [LOGM:0] addr_y;         
  output logic [LOGB:0] addr_b;
  output logic wr_en_x, wr_en_y, clear_acc, m_valid, s_ready;
  logic write_complete, mac_complete, hold_s2;
  logic [2:0] state, next_state;                                                      // FSM Transition Variables
  logic [2:0] data_displayed;
  logic [1:0] hold_s3;
  logic [LOGW:0] row, hold_disp;
  logic [LOGW:0] column;
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

parameter N=4; parameter M=8; parameter T=16; parameter P=8;
parameter LOGM=3; parameter LOGW=2; parameter LOGN=2; parameter LOGB=1;

 input m_ready, clk, reset, clear_acc, wr_en_x, wr_en_y;             // Input Signals from contol_module
 input [LOGW:0] addr_m,counter;
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

 logic signed [T-1:0] data_y2, data_y2_R;
 logic signed [T-1:0] data_m2, data_b2;
 logic signed [T-1:0] data_out2;

 logic signed [T-1:0] data_y3, data_y3_R;
 logic signed [T-1:0] data_m3, data_b3;
 logic signed [T-1:0] data_out3;

 logic signed [T-1:0] data_y4, data_y4_R;
 logic signed [T-1:0] data_m4, data_b4;
 logic signed [T-1:0] data_out4;

 logic signed [T-1:0] data_y5, data_y5_R;
 logic signed [T-1:0] data_m5, data_b5;
 logic signed [T-1:0] data_out5;

 logic signed [T-1:0] data_y6, data_y6_R;
 logic signed [T-1:0] data_m6, data_b6;
 logic signed [T-1:0] data_out6;

 logic signed [T-1:0] data_y7, data_y7_R;
 logic signed [T-1:0] data_m7, data_b7;
 logic signed [T-1:0] data_out7;

 logic signed [T-1:0] data_y8, data_y8_R;
 logic signed [T-1:0] data_m8, data_b8;
 logic signed [T-1:0] data_out8;

 assign data_y1_R= ((wr_en_y==1)&&(data_y1[T-1]==1))? 0:data_y1;
 assign data_y2_R= ((wr_en_y==1)&&(data_y2[T-1]==1))? 0:data_y2;
 assign data_y3_R= ((wr_en_y==1)&&(data_y3[T-1]==1))? 0:data_y3;
 assign data_y4_R= ((wr_en_y==1)&&(data_y4[T-1]==1))? 0:data_y4;
 assign data_y5_R= ((wr_en_y==1)&&(data_y5[T-1]==1))? 0:data_y5;
 assign data_y6_R= ((wr_en_y==1)&&(data_y6[T-1]==1))? 0:data_y6;
 assign data_y7_R= ((wr_en_y==1)&&(data_y7[T-1]==1))? 0:data_y7;
 assign data_y8_R= ((wr_en_y==1)&&(data_y8[T-1]==1))? 0:data_y8;

always_comb begin
 if(counter %P == 0) data_out = data_out1;
 else if(counter % P == 1) data_out = data_out2;
 else if(counter % P == 2) data_out = data_out3;
 else if(counter % P == 3) data_out = data_out4;
 else if(counter % P == 4) data_out = data_out5;
 else if(counter % P == 5) data_out = data_out6;
 else if(counter % P == 6) data_out = data_out7;
 else if(counter % P == 7) data_out = data_out8;
 else data_out = data_out7;
end

 memory #(T,N,LOGN+1) x(clk, data_in, data_x, addr_x, wr_en_x);                     // Vector Memory Instantiation
 memory #(T,M,LOGM+1) y1(clk, data_y1_R, data_out1, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer1_8_4_8_16_B1_rom lb1 (clk, addr_b, data_b1);
 layer1_8_4_8_16_W1_rom lw1 (clk, addr_m, data_m1);
 mac ma11(clk, reset, clear_acc,data_b1, data_m1, data_x, data_y1);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y2(clk, data_y2_R, data_out2, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer1_8_4_8_16_B2_rom lb2 (clk, addr_b, data_b2);
 layer1_8_4_8_16_W2_rom lw2 (clk, addr_m, data_m2);
 mac ma12(clk, reset, clear_acc,data_b2, data_m2, data_x, data_y2);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y3(clk, data_y3_R, data_out3, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer1_8_4_8_16_B3_rom lb3 (clk, addr_b, data_b3);
 layer1_8_4_8_16_W3_rom lw3 (clk, addr_m, data_m3);
 mac ma13(clk, reset, clear_acc,data_b3, data_m3, data_x, data_y3);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y4(clk, data_y4_R, data_out4, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer1_8_4_8_16_B4_rom lb4 (clk, addr_b, data_b4);
 layer1_8_4_8_16_W4_rom lw4 (clk, addr_m, data_m4);
 mac ma14(clk, reset, clear_acc,data_b4, data_m4, data_x, data_y4);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y5(clk, data_y5_R, data_out5, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer1_8_4_8_16_B5_rom lb5 (clk, addr_b, data_b5);
 layer1_8_4_8_16_W5_rom lw5 (clk, addr_m, data_m5);
 mac ma15(clk, reset, clear_acc,data_b5, data_m5, data_x, data_y5);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y6(clk, data_y6_R, data_out6, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer1_8_4_8_16_B6_rom lb6 (clk, addr_b, data_b6);
 layer1_8_4_8_16_W6_rom lw6 (clk, addr_m, data_m6);
 mac ma16(clk, reset, clear_acc,data_b6, data_m6, data_x, data_y6);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y7(clk, data_y7_R, data_out7, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer1_8_4_8_16_B7_rom lb7 (clk, addr_b, data_b7);
 layer1_8_4_8_16_W7_rom lw7 (clk, addr_m, data_m7);
 mac ma17(clk, reset, clear_acc,data_b7, data_m7, data_x, data_y7);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y8(clk, data_y8_R, data_out8, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer1_8_4_8_16_B8_rom lb8 (clk, addr_b, data_b8);
 layer1_8_4_8_16_W8_rom lw8 (clk, addr_m, data_m8);
 mac ma18(clk, reset, clear_acc,data_b8, data_m8, data_x, data_y8);               // MAC Module Instantiation


endmodule

//==================================================ROM  MODULE=====================================================
module layer1_8_4_8_16_W1_rom(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd9);
        1: z <= $signed (16'd100);
        2: z <= $signed (-16'd55);
        3: z <= $signed (16'd88);
      endcase
   end
endmodule

module layer1_8_4_8_16_W2_rom(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd46);
        1: z <= $signed (-16'd8);
        2: z <= $signed (-16'd67);
        3: z <= $signed (-16'd105);
      endcase
   end
endmodule

module layer1_8_4_8_16_W3_rom(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd34);
        1: z <= $signed (16'd32);
        2: z <= $signed (16'd103);
        3: z <= $signed (16'd125);
      endcase
   end
endmodule

module layer1_8_4_8_16_W4_rom(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd86);
        1: z <= $signed (-16'd30);
        2: z <= $signed (-16'd51);
        3: z <= $signed (16'd42);
      endcase
   end
endmodule

module layer1_8_4_8_16_W5_rom(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd102);
        1: z <= $signed (16'd44);
        2: z <= $signed (16'd103);
        3: z <= $signed (16'd114);
      endcase
   end
endmodule

module layer1_8_4_8_16_W6_rom(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd89);
        1: z <= $signed (16'd20);
        2: z <= $signed (16'd35);
        3: z <= $signed (-16'd99);
      endcase
   end
endmodule

module layer1_8_4_8_16_W7_rom(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd59);
        1: z <= $signed (16'd79);
        2: z <= $signed (-16'd88);
        3: z <= $signed (-16'd112);
      endcase
   end
endmodule

module layer1_8_4_8_16_W8_rom(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd43);
        1: z <= $signed (-16'd30);
        2: z <= $signed (-16'd34);
        3: z <= $signed (-16'd76);
      endcase
   end
endmodule

module layer1_8_4_8_16_B1_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd58);
      endcase
   end
endmodule

module layer1_8_4_8_16_B2_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd39);
      endcase
   end
endmodule

module layer1_8_4_8_16_B3_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd116);
      endcase
   end
endmodule

module layer1_8_4_8_16_B4_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd25);
      endcase
   end
endmodule

module layer1_8_4_8_16_B5_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd97);
      endcase
   end
endmodule

module layer1_8_4_8_16_B6_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd55);
      endcase
   end
endmodule

module layer1_8_4_8_16_B7_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd48);
      endcase
   end
endmodule

module layer1_8_4_8_16_B8_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd66);
      endcase
   end
endmodule

//=================================================MAIN MODULE=====================================================
module layer2_12_8_6_16(clk, reset, s_valid, m_ready, data_in, m_valid, s_ready, data_out);  

parameter N=8; parameter M=12; parameter T=16; parameter P=6;
parameter LOGM=4; parameter LOGW=4; parameter LOGN=3; parameter LOGB=1;

 input clk, reset, s_valid, m_ready;                       // Control Signals
  input signed [T-1:0] data_in;
  output logic signed [T-1:0] data_out; 
  output m_valid, s_ready;
  reg [LOGW:0] addr_m,counter;                                        // Memory Address pointer of Input matrix M
  reg [LOGN:0] addr_x;
  reg [LOGM:0] addr_y;                 // Memory Adress pointer of Input Vector and Output Values
  reg [LOGB:0] addr_b;
  logic wr_en_x, wr_en_y, clear_acc;  
  control_module2 c(clk, reset, s_valid,m_ready,addr_y, wr_en_y, addr_x, wr_en_x,addr_m,addr_b, m_valid,s_ready, clear_acc, counter);
  datapath_module2 d(clk, reset, counter, m_ready, clear_acc, data_in, addr_x, addr_m,addr_b, addr_y, wr_en_x, wr_en_y, data_out); 

endmodule

//==================================================CONTROL MODULE==================================================

 module control_module2 (clk, reset, s_valid,m_ready,addr_y, wr_en_y, addr_x, wr_en_x,addr_m,addr_b, m_valid,s_ready, clear_acc, counter);

parameter N=8; parameter M=12; parameter T=16; parameter P=6;
parameter LOGM=4; parameter LOGW=4; parameter LOGN=3; parameter LOGB=1;

  input clk, reset, s_valid, m_ready;
  output logic [LOGW:0] addr_m,counter;
  output logic [LOGN:0] addr_x;
  output logic [LOGM:0] addr_y;         
  output logic [LOGB:0] addr_b;
  output logic wr_en_x, wr_en_y, clear_acc, m_valid, s_ready;
  logic write_complete, mac_complete, hold_s2;
  logic [2:0] state, next_state;                                                      // FSM Transition Variables
  logic [2:0] data_displayed;
  logic [1:0] hold_s3;
  logic [LOGW:0] row, hold_disp;
  logic [LOGW:0] column;
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
module datapath_module2 (clk, reset, counter, m_ready, clear_acc, data_in, addr_x, addr_m, addr_b, addr_y, wr_en_x, wr_en_y, data_out);    

parameter N=8; parameter M=12; parameter T=16; parameter P=6;
parameter LOGM=4; parameter LOGW=4; parameter LOGN=3; parameter LOGB=1;

 input m_ready, clk, reset, clear_acc, wr_en_x, wr_en_y;             // Input Signals from contol_module
 input [LOGW:0] addr_m,counter;
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

 logic signed [T-1:0] data_y2, data_y2_R;
 logic signed [T-1:0] data_m2, data_b2;
 logic signed [T-1:0] data_out2;

 logic signed [T-1:0] data_y3, data_y3_R;
 logic signed [T-1:0] data_m3, data_b3;
 logic signed [T-1:0] data_out3;

 logic signed [T-1:0] data_y4, data_y4_R;
 logic signed [T-1:0] data_m4, data_b4;
 logic signed [T-1:0] data_out4;

 logic signed [T-1:0] data_y5, data_y5_R;
 logic signed [T-1:0] data_m5, data_b5;
 logic signed [T-1:0] data_out5;

 logic signed [T-1:0] data_y6, data_y6_R;
 logic signed [T-1:0] data_m6, data_b6;
 logic signed [T-1:0] data_out6;

 assign data_y1_R= ((wr_en_y==1)&&(data_y1[T-1]==1))? 0:data_y1;
 assign data_y2_R= ((wr_en_y==1)&&(data_y2[T-1]==1))? 0:data_y2;
 assign data_y3_R= ((wr_en_y==1)&&(data_y3[T-1]==1))? 0:data_y3;
 assign data_y4_R= ((wr_en_y==1)&&(data_y4[T-1]==1))? 0:data_y4;
 assign data_y5_R= ((wr_en_y==1)&&(data_y5[T-1]==1))? 0:data_y5;
 assign data_y6_R= ((wr_en_y==1)&&(data_y6[T-1]==1))? 0:data_y6;

always_comb begin
 if(counter %P == 0) data_out = data_out1;
 else if(counter % P == 1) data_out = data_out2;
 else if(counter % P == 2) data_out = data_out3;
 else if(counter % P == 3) data_out = data_out4;
 else if(counter % P == 4) data_out = data_out5;
 else if(counter % P == 5) data_out = data_out6;
 else data_out = data_out5;
end

 memory #(T,N,LOGN+1) x(clk, data_in, data_x, addr_x, wr_en_x);                     // Vector Memory Instantiation
 memory #(T,M,LOGM+1) y1(clk, data_y1_R, data_out1, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer2_12_8_6_16_B1_rom lb1 (clk, addr_b, data_b1);
 layer2_12_8_6_16_W1_rom lw1 (clk, addr_m, data_m1);
 mac ma21(clk, reset, clear_acc,data_b1, data_m1, data_x, data_y1);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y2(clk, data_y2_R, data_out2, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer2_12_8_6_16_B2_rom lb2 (clk, addr_b, data_b2);
 layer2_12_8_6_16_W2_rom lw2 (clk, addr_m, data_m2);
 mac ma22(clk, reset, clear_acc,data_b2, data_m2, data_x, data_y2);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y3(clk, data_y3_R, data_out3, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer2_12_8_6_16_B3_rom lb3 (clk, addr_b, data_b3);
 layer2_12_8_6_16_W3_rom lw3 (clk, addr_m, data_m3);
 mac ma23(clk, reset, clear_acc,data_b3, data_m3, data_x, data_y3);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y4(clk, data_y4_R, data_out4, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer2_12_8_6_16_B4_rom lb4 (clk, addr_b, data_b4);
 layer2_12_8_6_16_W4_rom lw4 (clk, addr_m, data_m4);
 mac ma24(clk, reset, clear_acc,data_b4, data_m4, data_x, data_y4);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y5(clk, data_y5_R, data_out5, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer2_12_8_6_16_B5_rom lb5 (clk, addr_b, data_b5);
 layer2_12_8_6_16_W5_rom lw5 (clk, addr_m, data_m5);
 mac ma25(clk, reset, clear_acc,data_b5, data_m5, data_x, data_y5);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y6(clk, data_y6_R, data_out6, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer2_12_8_6_16_B6_rom lb6 (clk, addr_b, data_b6);
 layer2_12_8_6_16_W6_rom lw6 (clk, addr_m, data_m6);
 mac ma26(clk, reset, clear_acc,data_b6, data_m6, data_x, data_y6);               // MAC Module Instantiation


endmodule

//==================================================ROM  MODULE=====================================================
module layer2_12_8_6_16_W1_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd105);
        1: z <= $signed (16'd23);
        2: z <= $signed (16'd63);
        3: z <= $signed (-16'd108);
        4: z <= $signed (16'd121);
        5: z <= $signed (-16'd116);
        6: z <= $signed (16'd62);
        7: z <= $signed (16'd96);
        8: z <= $signed (-16'd110);
        9: z <= $signed (-16'd93);
        10: z <= $signed (16'd55);
        11: z <= $signed (-16'd52);
        12: z <= $signed (-16'd112);
        13: z <= $signed (16'd91);
        14: z <= $signed (16'd76);
        15: z <= $signed (16'd36);
      endcase
   end
endmodule

module layer2_12_8_6_16_W2_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd56);
        1: z <= $signed (16'd37);
        2: z <= $signed (16'd82);
        3: z <= $signed (16'd18);
        4: z <= $signed (-16'd71);
        5: z <= $signed (-16'd11);
        6: z <= $signed (16'd47);
        7: z <= $signed (-16'd1);
        8: z <= $signed (-16'd117);
        9: z <= $signed (16'd101);
        10: z <= $signed (-16'd40);
        11: z <= $signed (16'd5);
        12: z <= $signed (16'd46);
        13: z <= $signed (16'd78);
        14: z <= $signed (16'd105);
        15: z <= $signed (-16'd114);
      endcase
   end
endmodule

module layer2_12_8_6_16_W3_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd59);
        1: z <= $signed (16'd87);
        2: z <= $signed (16'd15);
        3: z <= $signed (16'd112);
        4: z <= $signed (-16'd71);
        5: z <= $signed (16'd109);
        6: z <= $signed (-16'd92);
        7: z <= $signed (16'd0);
        8: z <= $signed (-16'd125);
        9: z <= $signed (-16'd32);
        10: z <= $signed (-16'd24);
        11: z <= $signed (-16'd60);
        12: z <= $signed (16'd22);
        13: z <= $signed (16'd34);
        14: z <= $signed (-16'd66);
        15: z <= $signed (-16'd14);
      endcase
   end
endmodule

module layer2_12_8_6_16_W4_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd20);
        1: z <= $signed (-16'd80);
        2: z <= $signed (-16'd103);
        3: z <= $signed (16'd52);
        4: z <= $signed (-16'd6);
        5: z <= $signed (16'd73);
        6: z <= $signed (-16'd10);
        7: z <= $signed (-16'd29);
        8: z <= $signed (16'd47);
        9: z <= $signed (16'd73);
        10: z <= $signed (16'd7);
        11: z <= $signed (-16'd79);
        12: z <= $signed (-16'd124);
        13: z <= $signed (-16'd101);
        14: z <= $signed (16'd120);
        15: z <= $signed (-16'd105);
      endcase
   end
endmodule

module layer2_12_8_6_16_W5_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd32);
        1: z <= $signed (-16'd75);
        2: z <= $signed (-16'd9);
        3: z <= $signed (-16'd38);
        4: z <= $signed (-16'd63);
        5: z <= $signed (-16'd74);
        6: z <= $signed (-16'd70);
        7: z <= $signed (16'd121);
        8: z <= $signed (-16'd66);
        9: z <= $signed (16'd47);
        10: z <= $signed (-16'd29);
        11: z <= $signed (-16'd50);
        12: z <= $signed (16'd10);
        13: z <= $signed (-16'd81);
        14: z <= $signed (16'd114);
        15: z <= $signed (16'd21);
      endcase
   end
endmodule

module layer2_12_8_6_16_W6_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd91);
        1: z <= $signed (-16'd116);
        2: z <= $signed (16'd11);
        3: z <= $signed (-16'd107);
        4: z <= $signed (16'd2);
        5: z <= $signed (-16'd69);
        6: z <= $signed (16'd20);
        7: z <= $signed (16'd71);
        8: z <= $signed (-16'd108);
        9: z <= $signed (-16'd53);
        10: z <= $signed (-16'd101);
        11: z <= $signed (16'd66);
        12: z <= $signed (-16'd103);
        13: z <= $signed (-16'd124);
        14: z <= $signed (16'd80);
        15: z <= $signed (-16'd99);
      endcase
   end
endmodule

module layer2_12_8_6_16_B1_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd28);
        1: z <= $signed (-16'd19);
      endcase
   end
endmodule

module layer2_12_8_6_16_B2_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd71);
        1: z <= $signed (16'd10);
      endcase
   end
endmodule

module layer2_12_8_6_16_B3_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd31);
        1: z <= $signed (-16'd23);
      endcase
   end
endmodule

module layer2_12_8_6_16_B4_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd123);
        1: z <= $signed (16'd116);
      endcase
   end
endmodule

module layer2_12_8_6_16_B5_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd91);
        1: z <= $signed (16'd59);
      endcase
   end
endmodule

module layer2_12_8_6_16_B6_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd31);
        1: z <= $signed (-16'd19);
      endcase
   end
endmodule

//=================================================MAIN MODULE=====================================================
module layer3_16_12_16_16(clk, reset, s_valid, m_ready, data_in, m_valid, s_ready, data_out);  

parameter N=12; parameter M=16; parameter T=16; parameter P=16;
parameter LOGM=4; parameter LOGW=4; parameter LOGN=4; parameter LOGB=1;

 input clk, reset, s_valid, m_ready;                       // Control Signals
  input signed [T-1:0] data_in;
  output logic signed [T-1:0] data_out; 
  output m_valid, s_ready;
  reg [LOGW:0] addr_m,counter;                                        // Memory Address pointer of Input matrix M
  reg [LOGN:0] addr_x;
  reg [LOGM:0] addr_y;                 // Memory Adress pointer of Input Vector and Output Values
  reg [LOGB:0] addr_b;
  logic wr_en_x, wr_en_y, clear_acc;  
  control_module3 c(clk, reset, s_valid,m_ready,addr_y, wr_en_y, addr_x, wr_en_x,addr_m,addr_b, m_valid,s_ready, clear_acc, counter);
  datapath_module3 d(clk, reset, counter, m_ready, clear_acc, data_in, addr_x, addr_m,addr_b, addr_y, wr_en_x, wr_en_y, data_out); 

endmodule

//==================================================CONTROL MODULE==================================================

 module control_module3 (clk, reset, s_valid,m_ready,addr_y, wr_en_y, addr_x, wr_en_x,addr_m,addr_b, m_valid,s_ready, clear_acc, counter);

parameter N=12; parameter M=16; parameter T=16; parameter P=16;
parameter LOGM=4; parameter LOGW=4; parameter LOGN=4; parameter LOGB=1;

  input clk, reset, s_valid, m_ready;
  output logic [LOGW:0] addr_m,counter;
  output logic [LOGN:0] addr_x;
  output logic [LOGM:0] addr_y;         
  output logic [LOGB:0] addr_b;
  output logic wr_en_x, wr_en_y, clear_acc, m_valid, s_ready;
  logic write_complete, mac_complete, hold_s2;
  logic [2:0] state, next_state;                                                      // FSM Transition Variables
  logic [2:0] data_displayed;
  logic [1:0] hold_s3;
  logic [LOGW:0] row, hold_disp;
  logic [LOGW:0] column;
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
module datapath_module3 (clk, reset, counter, m_ready, clear_acc, data_in, addr_x, addr_m, addr_b, addr_y, wr_en_x, wr_en_y, data_out);    

parameter N=12; parameter M=16; parameter T=16; parameter P=16;
parameter LOGM=4; parameter LOGW=4; parameter LOGN=4; parameter LOGB=1;

 input m_ready, clk, reset, clear_acc, wr_en_x, wr_en_y;             // Input Signals from contol_module
 input [LOGW:0] addr_m,counter;
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

 logic signed [T-1:0] data_y2, data_y2_R;
 logic signed [T-1:0] data_m2, data_b2;
 logic signed [T-1:0] data_out2;

 logic signed [T-1:0] data_y3, data_y3_R;
 logic signed [T-1:0] data_m3, data_b3;
 logic signed [T-1:0] data_out3;

 logic signed [T-1:0] data_y4, data_y4_R;
 logic signed [T-1:0] data_m4, data_b4;
 logic signed [T-1:0] data_out4;

 logic signed [T-1:0] data_y5, data_y5_R;
 logic signed [T-1:0] data_m5, data_b5;
 logic signed [T-1:0] data_out5;

 logic signed [T-1:0] data_y6, data_y6_R;
 logic signed [T-1:0] data_m6, data_b6;
 logic signed [T-1:0] data_out6;

 logic signed [T-1:0] data_y7, data_y7_R;
 logic signed [T-1:0] data_m7, data_b7;
 logic signed [T-1:0] data_out7;

 logic signed [T-1:0] data_y8, data_y8_R;
 logic signed [T-1:0] data_m8, data_b8;
 logic signed [T-1:0] data_out8;

 logic signed [T-1:0] data_y9, data_y9_R;
 logic signed [T-1:0] data_m9, data_b9;
 logic signed [T-1:0] data_out9;

 logic signed [T-1:0] data_y10, data_y10_R;
 logic signed [T-1:0] data_m10, data_b10;
 logic signed [T-1:0] data_out10;

 logic signed [T-1:0] data_y11, data_y11_R;
 logic signed [T-1:0] data_m11, data_b11;
 logic signed [T-1:0] data_out11;

 logic signed [T-1:0] data_y12, data_y12_R;
 logic signed [T-1:0] data_m12, data_b12;
 logic signed [T-1:0] data_out12;

 logic signed [T-1:0] data_y13, data_y13_R;
 logic signed [T-1:0] data_m13, data_b13;
 logic signed [T-1:0] data_out13;

 logic signed [T-1:0] data_y14, data_y14_R;
 logic signed [T-1:0] data_m14, data_b14;
 logic signed [T-1:0] data_out14;

 logic signed [T-1:0] data_y15, data_y15_R;
 logic signed [T-1:0] data_m15, data_b15;
 logic signed [T-1:0] data_out15;

 logic signed [T-1:0] data_y16, data_y16_R;
 logic signed [T-1:0] data_m16, data_b16;
 logic signed [T-1:0] data_out16;

 assign data_y1_R= ((wr_en_y==1)&&(data_y1[T-1]==1))? 0:data_y1;
 assign data_y2_R= ((wr_en_y==1)&&(data_y2[T-1]==1))? 0:data_y2;
 assign data_y3_R= ((wr_en_y==1)&&(data_y3[T-1]==1))? 0:data_y3;
 assign data_y4_R= ((wr_en_y==1)&&(data_y4[T-1]==1))? 0:data_y4;
 assign data_y5_R= ((wr_en_y==1)&&(data_y5[T-1]==1))? 0:data_y5;
 assign data_y6_R= ((wr_en_y==1)&&(data_y6[T-1]==1))? 0:data_y6;
 assign data_y7_R= ((wr_en_y==1)&&(data_y7[T-1]==1))? 0:data_y7;
 assign data_y8_R= ((wr_en_y==1)&&(data_y8[T-1]==1))? 0:data_y8;
 assign data_y9_R= ((wr_en_y==1)&&(data_y9[T-1]==1))? 0:data_y9;
 assign data_y10_R= ((wr_en_y==1)&&(data_y10[T-1]==1))? 0:data_y10;
 assign data_y11_R= ((wr_en_y==1)&&(data_y11[T-1]==1))? 0:data_y11;
 assign data_y12_R= ((wr_en_y==1)&&(data_y12[T-1]==1))? 0:data_y12;
 assign data_y13_R= ((wr_en_y==1)&&(data_y13[T-1]==1))? 0:data_y13;
 assign data_y14_R= ((wr_en_y==1)&&(data_y14[T-1]==1))? 0:data_y14;
 assign data_y15_R= ((wr_en_y==1)&&(data_y15[T-1]==1))? 0:data_y15;
 assign data_y16_R= ((wr_en_y==1)&&(data_y16[T-1]==1))? 0:data_y16;

always_comb begin
 if(counter %P == 0) data_out = data_out1;
 else if(counter % P == 1) data_out = data_out2;
 else if(counter % P == 2) data_out = data_out3;
 else if(counter % P == 3) data_out = data_out4;
 else if(counter % P == 4) data_out = data_out5;
 else if(counter % P == 5) data_out = data_out6;
 else if(counter % P == 6) data_out = data_out7;
 else if(counter % P == 7) data_out = data_out8;
 else if(counter % P == 8) data_out = data_out9;
 else if(counter % P == 9) data_out = data_out10;
 else if(counter % P == 10) data_out = data_out11;
 else if(counter % P == 11) data_out = data_out12;
 else if(counter % P == 12) data_out = data_out13;
 else if(counter % P == 13) data_out = data_out14;
 else if(counter % P == 14) data_out = data_out15;
 else if(counter % P == 15) data_out = data_out16;
 else data_out = data_out15;
end

 memory #(T,N,LOGN+1) x(clk, data_in, data_x, addr_x, wr_en_x);                     // Vector Memory Instantiation
 memory #(T,M,LOGM+1) y1(clk, data_y1_R, data_out1, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer3_16_12_16_16_B1_rom lb1 (clk, addr_b, data_b1);
 layer3_16_12_16_16_W1_rom lw1 (clk, addr_m, data_m1);
 mac ma31(clk, reset, clear_acc,data_b1, data_m1, data_x, data_y1);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y2(clk, data_y2_R, data_out2, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer3_16_12_16_16_B2_rom lb2 (clk, addr_b, data_b2);
 layer3_16_12_16_16_W2_rom lw2 (clk, addr_m, data_m2);
 mac ma32(clk, reset, clear_acc,data_b2, data_m2, data_x, data_y2);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y3(clk, data_y3_R, data_out3, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer3_16_12_16_16_B3_rom lb3 (clk, addr_b, data_b3);
 layer3_16_12_16_16_W3_rom lw3 (clk, addr_m, data_m3);
 mac ma33(clk, reset, clear_acc,data_b3, data_m3, data_x, data_y3);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y4(clk, data_y4_R, data_out4, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer3_16_12_16_16_B4_rom lb4 (clk, addr_b, data_b4);
 layer3_16_12_16_16_W4_rom lw4 (clk, addr_m, data_m4);
 mac ma34(clk, reset, clear_acc,data_b4, data_m4, data_x, data_y4);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y5(clk, data_y5_R, data_out5, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer3_16_12_16_16_B5_rom lb5 (clk, addr_b, data_b5);
 layer3_16_12_16_16_W5_rom lw5 (clk, addr_m, data_m5);
 mac ma35(clk, reset, clear_acc,data_b5, data_m5, data_x, data_y5);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y6(clk, data_y6_R, data_out6, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer3_16_12_16_16_B6_rom lb6 (clk, addr_b, data_b6);
 layer3_16_12_16_16_W6_rom lw6 (clk, addr_m, data_m6);
 mac ma36(clk, reset, clear_acc,data_b6, data_m6, data_x, data_y6);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y7(clk, data_y7_R, data_out7, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer3_16_12_16_16_B7_rom lb7 (clk, addr_b, data_b7);
 layer3_16_12_16_16_W7_rom lw7 (clk, addr_m, data_m7);
 mac ma37(clk, reset, clear_acc,data_b7, data_m7, data_x, data_y7);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y8(clk, data_y8_R, data_out8, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer3_16_12_16_16_B8_rom lb8 (clk, addr_b, data_b8);
 layer3_16_12_16_16_W8_rom lw8 (clk, addr_m, data_m8);
 mac ma38(clk, reset, clear_acc,data_b8, data_m8, data_x, data_y8);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y9(clk, data_y9_R, data_out9, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer3_16_12_16_16_B9_rom lb9 (clk, addr_b, data_b9);
 layer3_16_12_16_16_W9_rom lw9 (clk, addr_m, data_m9);
 mac ma39(clk, reset, clear_acc,data_b9, data_m9, data_x, data_y9);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y10(clk, data_y10_R, data_out10, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer3_16_12_16_16_B10_rom lb10 (clk, addr_b, data_b10);
 layer3_16_12_16_16_W10_rom lw10 (clk, addr_m, data_m10);
 mac ma310(clk, reset, clear_acc,data_b10, data_m10, data_x, data_y10);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y11(clk, data_y11_R, data_out11, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer3_16_12_16_16_B11_rom lb11 (clk, addr_b, data_b11);
 layer3_16_12_16_16_W11_rom lw11 (clk, addr_m, data_m11);
 mac ma311(clk, reset, clear_acc,data_b11, data_m11, data_x, data_y11);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y12(clk, data_y12_R, data_out12, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer3_16_12_16_16_B12_rom lb12 (clk, addr_b, data_b12);
 layer3_16_12_16_16_W12_rom lw12 (clk, addr_m, data_m12);
 mac ma312(clk, reset, clear_acc,data_b12, data_m12, data_x, data_y12);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y13(clk, data_y13_R, data_out13, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer3_16_12_16_16_B13_rom lb13 (clk, addr_b, data_b13);
 layer3_16_12_16_16_W13_rom lw13 (clk, addr_m, data_m13);
 mac ma313(clk, reset, clear_acc,data_b13, data_m13, data_x, data_y13);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y14(clk, data_y14_R, data_out14, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer3_16_12_16_16_B14_rom lb14 (clk, addr_b, data_b14);
 layer3_16_12_16_16_W14_rom lw14 (clk, addr_m, data_m14);
 mac ma314(clk, reset, clear_acc,data_b14, data_m14, data_x, data_y14);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y15(clk, data_y15_R, data_out15, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer3_16_12_16_16_B15_rom lb15 (clk, addr_b, data_b15);
 layer3_16_12_16_16_W15_rom lw15 (clk, addr_m, data_m15);
 mac ma315(clk, reset, clear_acc,data_b15, data_m15, data_x, data_y15);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y16(clk, data_y16_R, data_out16, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer3_16_12_16_16_B16_rom lb16 (clk, addr_b, data_b16);
 layer3_16_12_16_16_W16_rom lw16 (clk, addr_m, data_m16);
 mac ma316(clk, reset, clear_acc,data_b16, data_m16, data_x, data_y16);               // MAC Module Instantiation


endmodule

//==================================================ROM  MODULE=====================================================
module layer3_16_12_16_16_W1_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd113);
        1: z <= $signed (16'd51);
        2: z <= $signed (16'd4);
        3: z <= $signed (-16'd51);
        4: z <= $signed (-16'd30);
        5: z <= $signed (16'd103);
        6: z <= $signed (16'd27);
        7: z <= $signed (16'd108);
        8: z <= $signed (-16'd106);
        9: z <= $signed (16'd13);
        10: z <= $signed (16'd2);
        11: z <= $signed (-16'd86);
      endcase
   end
endmodule

module layer3_16_12_16_16_W2_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd88);
        1: z <= $signed (16'd29);
        2: z <= $signed (16'd108);
        3: z <= $signed (16'd114);
        4: z <= $signed (16'd33);
        5: z <= $signed (16'd61);
        6: z <= $signed (-16'd113);
        7: z <= $signed (-16'd123);
        8: z <= $signed (16'd118);
        9: z <= $signed (-16'd16);
        10: z <= $signed (-16'd128);
        11: z <= $signed (16'd81);
      endcase
   end
endmodule

module layer3_16_12_16_16_W3_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd112);
        1: z <= $signed (-16'd19);
        2: z <= $signed (-16'd36);
        3: z <= $signed (-16'd7);
        4: z <= $signed (-16'd31);
        5: z <= $signed (-16'd105);
        6: z <= $signed (16'd102);
        7: z <= $signed (-16'd16);
        8: z <= $signed (16'd75);
        9: z <= $signed (-16'd21);
        10: z <= $signed (16'd61);
        11: z <= $signed (-16'd83);
      endcase
   end
endmodule

module layer3_16_12_16_16_W4_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd46);
        1: z <= $signed (-16'd40);
        2: z <= $signed (-16'd102);
        3: z <= $signed (-16'd23);
        4: z <= $signed (16'd102);
        5: z <= $signed (16'd28);
        6: z <= $signed (16'd19);
        7: z <= $signed (16'd62);
        8: z <= $signed (-16'd71);
        9: z <= $signed (16'd0);
        10: z <= $signed (16'd48);
        11: z <= $signed (16'd90);
      endcase
   end
endmodule

module layer3_16_12_16_16_W5_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd67);
        1: z <= $signed (16'd63);
        2: z <= $signed (16'd95);
        3: z <= $signed (-16'd77);
        4: z <= $signed (-16'd80);
        5: z <= $signed (16'd96);
        6: z <= $signed (-16'd124);
        7: z <= $signed (-16'd64);
        8: z <= $signed (-16'd51);
        9: z <= $signed (-16'd32);
        10: z <= $signed (16'd57);
        11: z <= $signed (16'd47);
      endcase
   end
endmodule

module layer3_16_12_16_16_W6_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd8);
        1: z <= $signed (16'd31);
        2: z <= $signed (-16'd97);
        3: z <= $signed (-16'd61);
        4: z <= $signed (-16'd118);
        5: z <= $signed (16'd93);
        6: z <= $signed (-16'd16);
        7: z <= $signed (-16'd35);
        8: z <= $signed (-16'd75);
        9: z <= $signed (16'd10);
        10: z <= $signed (16'd70);
        11: z <= $signed (-16'd101);
      endcase
   end
endmodule

module layer3_16_12_16_16_W7_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd90);
        1: z <= $signed (-16'd39);
        2: z <= $signed (16'd90);
        3: z <= $signed (-16'd33);
        4: z <= $signed (16'd89);
        5: z <= $signed (16'd10);
        6: z <= $signed (-16'd71);
        7: z <= $signed (-16'd106);
        8: z <= $signed (-16'd54);
        9: z <= $signed (-16'd103);
        10: z <= $signed (-16'd55);
        11: z <= $signed (-16'd6);
      endcase
   end
endmodule

module layer3_16_12_16_16_W8_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd121);
        1: z <= $signed (-16'd50);
        2: z <= $signed (16'd58);
        3: z <= $signed (-16'd58);
        4: z <= $signed (16'd46);
        5: z <= $signed (-16'd13);
        6: z <= $signed (16'd117);
        7: z <= $signed (-16'd90);
        8: z <= $signed (-16'd110);
        9: z <= $signed (-16'd107);
        10: z <= $signed (-16'd23);
        11: z <= $signed (-16'd99);
      endcase
   end
endmodule

module layer3_16_12_16_16_W9_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd114);
        1: z <= $signed (16'd90);
        2: z <= $signed (-16'd6);
        3: z <= $signed (-16'd89);
        4: z <= $signed (-16'd28);
        5: z <= $signed (-16'd64);
        6: z <= $signed (-16'd61);
        7: z <= $signed (16'd11);
        8: z <= $signed (16'd25);
        9: z <= $signed (-16'd99);
        10: z <= $signed (16'd106);
        11: z <= $signed (-16'd13);
      endcase
   end
endmodule

module layer3_16_12_16_16_W10_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd39);
        1: z <= $signed (-16'd92);
        2: z <= $signed (16'd9);
        3: z <= $signed (16'd113);
        4: z <= $signed (-16'd67);
        5: z <= $signed (16'd83);
        6: z <= $signed (-16'd21);
        7: z <= $signed (-16'd74);
        8: z <= $signed (-16'd95);
        9: z <= $signed (-16'd91);
        10: z <= $signed (-16'd4);
        11: z <= $signed (16'd79);
      endcase
   end
endmodule

module layer3_16_12_16_16_W11_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd24);
        1: z <= $signed (-16'd14);
        2: z <= $signed (16'd118);
        3: z <= $signed (16'd43);
        4: z <= $signed (16'd7);
        5: z <= $signed (-16'd33);
        6: z <= $signed (16'd72);
        7: z <= $signed (-16'd7);
        8: z <= $signed (-16'd71);
        9: z <= $signed (-16'd62);
        10: z <= $signed (16'd32);
        11: z <= $signed (16'd30);
      endcase
   end
endmodule

module layer3_16_12_16_16_W12_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd2);
        1: z <= $signed (16'd99);
        2: z <= $signed (-16'd87);
        3: z <= $signed (-16'd101);
        4: z <= $signed (-16'd128);
        5: z <= $signed (-16'd109);
        6: z <= $signed (16'd14);
        7: z <= $signed (16'd40);
        8: z <= $signed (-16'd73);
        9: z <= $signed (-16'd104);
        10: z <= $signed (16'd25);
        11: z <= $signed (-16'd12);
      endcase
   end
endmodule

module layer3_16_12_16_16_W13_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd107);
        1: z <= $signed (-16'd123);
        2: z <= $signed (16'd42);
        3: z <= $signed (-16'd116);
        4: z <= $signed (-16'd86);
        5: z <= $signed (-16'd89);
        6: z <= $signed (16'd91);
        7: z <= $signed (16'd67);
        8: z <= $signed (16'd25);
        9: z <= $signed (16'd81);
        10: z <= $signed (-16'd18);
        11: z <= $signed (-16'd96);
      endcase
   end
endmodule

module layer3_16_12_16_16_W14_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd79);
        1: z <= $signed (-16'd74);
        2: z <= $signed (16'd25);
        3: z <= $signed (-16'd22);
        4: z <= $signed (-16'd8);
        5: z <= $signed (-16'd71);
        6: z <= $signed (-16'd120);
        7: z <= $signed (16'd122);
        8: z <= $signed (-16'd99);
        9: z <= $signed (-16'd79);
        10: z <= $signed (-16'd107);
        11: z <= $signed (-16'd99);
      endcase
   end
endmodule

module layer3_16_12_16_16_W15_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd59);
        1: z <= $signed (16'd36);
        2: z <= $signed (16'd69);
        3: z <= $signed (-16'd4);
        4: z <= $signed (16'd60);
        5: z <= $signed (-16'd33);
        6: z <= $signed (16'd113);
        7: z <= $signed (16'd39);
        8: z <= $signed (-16'd28);
        9: z <= $signed (16'd27);
        10: z <= $signed (16'd51);
        11: z <= $signed (16'd14);
      endcase
   end
endmodule

module layer3_16_12_16_16_W16_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd66);
        1: z <= $signed (16'd14);
        2: z <= $signed (-16'd47);
        3: z <= $signed (-16'd37);
        4: z <= $signed (-16'd32);
        5: z <= $signed (16'd63);
        6: z <= $signed (-16'd5);
        7: z <= $signed (16'd17);
        8: z <= $signed (16'd117);
        9: z <= $signed (-16'd108);
        10: z <= $signed (16'd123);
        11: z <= $signed (-16'd19);
      endcase
   end
endmodule

module layer3_16_12_16_16_B1_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd50);
      endcase
   end
endmodule

module layer3_16_12_16_16_B2_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd124);
      endcase
   end
endmodule

module layer3_16_12_16_16_B3_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd25);
      endcase
   end
endmodule

module layer3_16_12_16_16_B4_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd21);
      endcase
   end
endmodule

module layer3_16_12_16_16_B5_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd75);
      endcase
   end
endmodule

module layer3_16_12_16_16_B6_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd3);
      endcase
   end
endmodule

module layer3_16_12_16_16_B7_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd8);
      endcase
   end
endmodule

module layer3_16_12_16_16_B8_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd6);
      endcase
   end
endmodule

module layer3_16_12_16_16_B9_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd95);
      endcase
   end
endmodule

module layer3_16_12_16_16_B10_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd50);
      endcase
   end
endmodule

module layer3_16_12_16_16_B11_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd119);
      endcase
   end
endmodule

module layer3_16_12_16_16_B12_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd93);
      endcase
   end
endmodule

module layer3_16_12_16_16_B13_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd45);
      endcase
   end
endmodule

module layer3_16_12_16_16_B14_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd104);
      endcase
   end
endmodule

module layer3_16_12_16_16_B15_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd4);
      endcase
   end
endmodule

module layer3_16_12_16_16_B16_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd111);
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

