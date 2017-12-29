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


module network_4_8_12_16_10_16(clk, reset, s_valid, m_ready, data_in, m_valid, s_ready, data_out);

parameter T=16;

  input clk, reset, s_valid, m_ready; // Control Signals 
  input signed [T-1:0] data_in;
  output logic signed [T-1:0] data_out;
  output m_valid, s_ready;
  logic MS_ready_l2, MS_ready_l3;   // Interconnecting Layer parameters  
  logic MS_valid_l2, MS_valid_l3;
  logic signed [T-1:0] data_IO_l2, data_IO_l3;

layer1_8_4_2_16 Net1(clk, reset, s_valid, MS_ready_l2, data_in, MS_valid_l2, s_ready, data_IO_l2);   // Layer 1 GPIO
layer2_12_8_4_16 Net2(clk, reset, MS_valid_l2, MS_ready_l3, data_IO_l2, MS_valid_l3, MS_ready_l2, data_IO_l3);   // Layer 2 GPIO
layer3_16_12_4_16 Net3(clk, reset, MS_valid_l3, m_ready, data_IO_l3, m_valid, MS_ready_l3, data_out);   // Layer 3 GPIO

endmodule
//=================================================MAIN MODULE=====================================================
module layer1_8_4_2_16(clk, reset, s_valid, m_ready, data_in, m_valid, s_ready, data_out);  

parameter N=4; parameter M=8; parameter T=16; parameter P=2;
parameter LOGM=3; parameter LOGW=4; parameter LOGN=2; parameter LOGB=1;

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

parameter N=4; parameter M=8; parameter T=16; parameter P=2;
parameter LOGM=3; parameter LOGW=4; parameter LOGN=2; parameter LOGB=1;

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

parameter N=4; parameter M=8; parameter T=16; parameter P=2;
parameter LOGM=3; parameter LOGW=4; parameter LOGN=2; parameter LOGB=1;

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

 assign data_y1_R= ((wr_en_y==1)&&(data_y1[T-1]==1))? 0:data_y1;
 assign data_y2_R= ((wr_en_y==1)&&(data_y2[T-1]==1))? 0:data_y2;

always_comb begin
 if(counter %P == 0) data_out = data_out1;
 else if(counter % P == 1) data_out = data_out2;
 else data_out = data_out1;
end

 memory #(T,N,LOGN+1) x(clk, data_in, data_x, addr_x, wr_en_x);                     // Vector Memory Instantiation
 memory #(T,M,LOGM+1) y1(clk, data_y1_R, data_out1, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer1_8_4_2_16_B1_rom lb1 (clk, addr_b, data_b1);
 layer1_8_4_2_16_W1_rom lw1 (clk, addr_m, data_m1);
 mac ma11(clk, reset, clear_acc,data_b1, data_m1, data_x, data_y1);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y2(clk, data_y2_R, data_out2, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer1_8_4_2_16_B2_rom lb2 (clk, addr_b, data_b2);
 layer1_8_4_2_16_W2_rom lw2 (clk, addr_m, data_m2);
 mac ma12(clk, reset, clear_acc,data_b2, data_m2, data_x, data_y2);               // MAC Module Instantiation


endmodule

//==================================================ROM  MODULE=====================================================
module layer1_8_4_2_16_W1_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd48);
        1: z <= $signed (16'd115);
        2: z <= $signed (16'd21);
        3: z <= $signed (-16'd87);
        4: z <= $signed (-16'd18);
        5: z <= $signed (16'd65);
        6: z <= $signed (-16'd128);
        7: z <= $signed (-16'd25);
        8: z <= $signed (-16'd80);
        9: z <= $signed (-16'd100);
        10: z <= $signed (16'd57);
        11: z <= $signed (16'd58);
        12: z <= $signed (16'd53);
        13: z <= $signed (-16'd4);
        14: z <= $signed (-16'd105);
        15: z <= $signed (-16'd117);
      endcase
   end
endmodule

module layer1_8_4_2_16_W2_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd90);
        1: z <= $signed (16'd68);
        2: z <= $signed (-16'd73);
        3: z <= $signed (16'd92);
        4: z <= $signed (-16'd120);
        5: z <= $signed (-16'd51);
        6: z <= $signed (16'd21);
        7: z <= $signed (-16'd64);
        8: z <= $signed (-16'd74);
        9: z <= $signed (-16'd92);
        10: z <= $signed (-16'd4);
        11: z <= $signed (-16'd34);
        12: z <= $signed (16'd85);
        13: z <= $signed (16'd93);
        14: z <= $signed (-16'd109);
        15: z <= $signed (16'd5);
      endcase
   end
endmodule

module layer1_8_4_2_16_B1_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd80);
        1: z <= $signed (16'd46);
        2: z <= $signed (-16'd19);
        3: z <= $signed (16'd83);
      endcase
   end
endmodule

module layer1_8_4_2_16_B2_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd40);
        1: z <= $signed (16'd119);
        2: z <= $signed (16'd102);
        3: z <= $signed (16'd91);
      endcase
   end
endmodule

//=================================================MAIN MODULE=====================================================
module layer2_12_8_4_16(clk, reset, s_valid, m_ready, data_in, m_valid, s_ready, data_out);  

parameter N=8; parameter M=12; parameter T=16; parameter P=4;
parameter LOGM=4; parameter LOGW=5; parameter LOGN=3; parameter LOGB=1;

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

parameter N=8; parameter M=12; parameter T=16; parameter P=4;
parameter LOGM=4; parameter LOGW=5; parameter LOGN=3; parameter LOGB=1;

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

parameter N=8; parameter M=12; parameter T=16; parameter P=4;
parameter LOGM=4; parameter LOGW=5; parameter LOGN=3; parameter LOGB=1;

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

 assign data_y1_R= ((wr_en_y==1)&&(data_y1[T-1]==1))? 0:data_y1;
 assign data_y2_R= ((wr_en_y==1)&&(data_y2[T-1]==1))? 0:data_y2;
 assign data_y3_R= ((wr_en_y==1)&&(data_y3[T-1]==1))? 0:data_y3;
 assign data_y4_R= ((wr_en_y==1)&&(data_y4[T-1]==1))? 0:data_y4;

always_comb begin
 if(counter %P == 0) data_out = data_out1;
 else if(counter % P == 1) data_out = data_out2;
 else if(counter % P == 2) data_out = data_out3;
 else if(counter % P == 3) data_out = data_out4;
 else data_out = data_out3;
end

 memory #(T,N,LOGN+1) x(clk, data_in, data_x, addr_x, wr_en_x);                     // Vector Memory Instantiation
 memory #(T,M,LOGM+1) y1(clk, data_y1_R, data_out1, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer2_12_8_4_16_B1_rom lb1 (clk, addr_b, data_b1);
 layer2_12_8_4_16_W1_rom lw1 (clk, addr_m, data_m1);
 mac ma21(clk, reset, clear_acc,data_b1, data_m1, data_x, data_y1);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y2(clk, data_y2_R, data_out2, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer2_12_8_4_16_B2_rom lb2 (clk, addr_b, data_b2);
 layer2_12_8_4_16_W2_rom lw2 (clk, addr_m, data_m2);
 mac ma22(clk, reset, clear_acc,data_b2, data_m2, data_x, data_y2);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y3(clk, data_y3_R, data_out3, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer2_12_8_4_16_B3_rom lb3 (clk, addr_b, data_b3);
 layer2_12_8_4_16_W3_rom lw3 (clk, addr_m, data_m3);
 mac ma23(clk, reset, clear_acc,data_b3, data_m3, data_x, data_y3);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y4(clk, data_y4_R, data_out4, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer2_12_8_4_16_B4_rom lb4 (clk, addr_b, data_b4);
 layer2_12_8_4_16_W4_rom lw4 (clk, addr_m, data_m4);
 mac ma24(clk, reset, clear_acc,data_b4, data_m4, data_x, data_y4);               // MAC Module Instantiation


endmodule

//==================================================ROM  MODULE=====================================================
module layer2_12_8_4_16_W1_rom(clk, addr, z);
   input clk;
   input [5:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd39);
        1: z <= $signed (16'd83);
        2: z <= $signed (-16'd62);
        3: z <= $signed (16'd47);
        4: z <= $signed (-16'd96);
        5: z <= $signed (16'd88);
        6: z <= $signed (16'd112);
        7: z <= $signed (-16'd48);
        8: z <= $signed (16'd108);
        9: z <= $signed (16'd84);
        10: z <= $signed (-16'd55);
        11: z <= $signed (-16'd116);
        12: z <= $signed (16'd44);
        13: z <= $signed (-16'd71);
        14: z <= $signed (-16'd35);
        15: z <= $signed (16'd33);
        16: z <= $signed (16'd74);
        17: z <= $signed (-16'd118);
        18: z <= $signed (-16'd103);
        19: z <= $signed (-16'd9);
        20: z <= $signed (-16'd61);
        21: z <= $signed (-16'd10);
        22: z <= $signed (-16'd104);
        23: z <= $signed (-16'd91);
      endcase
   end
endmodule

module layer2_12_8_4_16_W2_rom(clk, addr, z);
   input clk;
   input [5:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd116);
        1: z <= $signed (16'd41);
        2: z <= $signed (-16'd117);
        3: z <= $signed (-16'd85);
        4: z <= $signed (16'd77);
        5: z <= $signed (16'd7);
        6: z <= $signed (16'd9);
        7: z <= $signed (16'd3);
        8: z <= $signed (16'd98);
        9: z <= $signed (-16'd24);
        10: z <= $signed (16'd76);
        11: z <= $signed (16'd48);
        12: z <= $signed (16'd111);
        13: z <= $signed (-16'd43);
        14: z <= $signed (-16'd77);
        15: z <= $signed (16'd114);
        16: z <= $signed (16'd94);
        17: z <= $signed (16'd100);
        18: z <= $signed (16'd85);
        19: z <= $signed (16'd77);
        20: z <= $signed (-16'd71);
        21: z <= $signed (-16'd120);
        22: z <= $signed (16'd63);
        23: z <= $signed (-16'd81);
      endcase
   end
endmodule

module layer2_12_8_4_16_W3_rom(clk, addr, z);
   input clk;
   input [5:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd125);
        1: z <= $signed (16'd32);
        2: z <= $signed (16'd14);
        3: z <= $signed (16'd88);
        4: z <= $signed (-16'd2);
        5: z <= $signed (16'd33);
        6: z <= $signed (-16'd34);
        7: z <= $signed (-16'd50);
        8: z <= $signed (16'd118);
        9: z <= $signed (16'd65);
        10: z <= $signed (16'd74);
        11: z <= $signed (-16'd12);
        12: z <= $signed (-16'd29);
        13: z <= $signed (-16'd88);
        14: z <= $signed (16'd66);
        15: z <= $signed (16'd45);
        16: z <= $signed (16'd74);
        17: z <= $signed (16'd9);
        18: z <= $signed (16'd35);
        19: z <= $signed (-16'd83);
        20: z <= $signed (16'd50);
        21: z <= $signed (-16'd26);
        22: z <= $signed (16'd90);
        23: z <= $signed (16'd103);
      endcase
   end
endmodule

module layer2_12_8_4_16_W4_rom(clk, addr, z);
   input clk;
   input [5:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd54);
        1: z <= $signed (-16'd116);
        2: z <= $signed (-16'd59);
        3: z <= $signed (16'd55);
        4: z <= $signed (16'd114);
        5: z <= $signed (-16'd104);
        6: z <= $signed (16'd18);
        7: z <= $signed (16'd25);
        8: z <= $signed (-16'd75);
        9: z <= $signed (-16'd120);
        10: z <= $signed (-16'd28);
        11: z <= $signed (-16'd89);
        12: z <= $signed (-16'd96);
        13: z <= $signed (16'd118);
        14: z <= $signed (16'd65);
        15: z <= $signed (-16'd116);
        16: z <= $signed (-16'd18);
        17: z <= $signed (-16'd66);
        18: z <= $signed (-16'd114);
        19: z <= $signed (16'd14);
        20: z <= $signed (-16'd76);
        21: z <= $signed (16'd79);
        22: z <= $signed (16'd27);
        23: z <= $signed (16'd126);
      endcase
   end
endmodule

module layer2_12_8_4_16_B1_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd89);
        1: z <= $signed (-16'd86);
        2: z <= $signed (-16'd15);
      endcase
   end
endmodule

module layer2_12_8_4_16_B2_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd52);
        1: z <= $signed (16'd13);
        2: z <= $signed (-16'd105);
      endcase
   end
endmodule

module layer2_12_8_4_16_B3_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd11);
        1: z <= $signed (-16'd62);
        2: z <= $signed (16'd85);
      endcase
   end
endmodule

module layer2_12_8_4_16_B4_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd100);
        1: z <= $signed (-16'd120);
        2: z <= $signed (16'd43);
      endcase
   end
endmodule

//=================================================MAIN MODULE=====================================================
module layer3_16_12_4_16(clk, reset, s_valid, m_ready, data_in, m_valid, s_ready, data_out);  

parameter N=12; parameter M=16; parameter T=16; parameter P=4;
parameter LOGM=4; parameter LOGW=6; parameter LOGN=4; parameter LOGB=1;

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

parameter N=12; parameter M=16; parameter T=16; parameter P=4;
parameter LOGM=4; parameter LOGW=6; parameter LOGN=4; parameter LOGB=1;

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

parameter N=12; parameter M=16; parameter T=16; parameter P=4;
parameter LOGM=4; parameter LOGW=6; parameter LOGN=4; parameter LOGB=1;

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

 assign data_y1_R= ((wr_en_y==1)&&(data_y1[T-1]==1))? 0:data_y1;
 assign data_y2_R= ((wr_en_y==1)&&(data_y2[T-1]==1))? 0:data_y2;
 assign data_y3_R= ((wr_en_y==1)&&(data_y3[T-1]==1))? 0:data_y3;
 assign data_y4_R= ((wr_en_y==1)&&(data_y4[T-1]==1))? 0:data_y4;

always_comb begin
 if(counter %P == 0) data_out = data_out1;
 else if(counter % P == 1) data_out = data_out2;
 else if(counter % P == 2) data_out = data_out3;
 else if(counter % P == 3) data_out = data_out4;
 else data_out = data_out3;
end

 memory #(T,N,LOGN+1) x(clk, data_in, data_x, addr_x, wr_en_x);                     // Vector Memory Instantiation
 memory #(T,M,LOGM+1) y1(clk, data_y1_R, data_out1, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer3_16_12_4_16_B1_rom lb1 (clk, addr_b, data_b1);
 layer3_16_12_4_16_W1_rom lw1 (clk, addr_m, data_m1);
 mac ma31(clk, reset, clear_acc,data_b1, data_m1, data_x, data_y1);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y2(clk, data_y2_R, data_out2, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer3_16_12_4_16_B2_rom lb2 (clk, addr_b, data_b2);
 layer3_16_12_4_16_W2_rom lw2 (clk, addr_m, data_m2);
 mac ma32(clk, reset, clear_acc,data_b2, data_m2, data_x, data_y2);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y3(clk, data_y3_R, data_out3, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer3_16_12_4_16_B3_rom lb3 (clk, addr_b, data_b3);
 layer3_16_12_4_16_W3_rom lw3 (clk, addr_m, data_m3);
 mac ma33(clk, reset, clear_acc,data_b3, data_m3, data_x, data_y3);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y4(clk, data_y4_R, data_out4, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer3_16_12_4_16_B4_rom lb4 (clk, addr_b, data_b4);
 layer3_16_12_4_16_W4_rom lw4 (clk, addr_m, data_m4);
 mac ma34(clk, reset, clear_acc,data_b4, data_m4, data_x, data_y4);               // MAC Module Instantiation


endmodule

//==================================================ROM  MODULE=====================================================
module layer3_16_12_4_16_W1_rom(clk, addr, z);
   input clk;
   input [6:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd96);
        1: z <= $signed (16'd20);
        2: z <= $signed (16'd90);
        3: z <= $signed (16'd106);
        4: z <= $signed (-16'd99);
        5: z <= $signed (-16'd2);
        6: z <= $signed (-16'd105);
        7: z <= $signed (16'd79);
        8: z <= $signed (16'd100);
        9: z <= $signed (16'd113);
        10: z <= $signed (16'd54);
        11: z <= $signed (-16'd46);
        12: z <= $signed (-16'd21);
        13: z <= $signed (16'd105);
        14: z <= $signed (-16'd75);
        15: z <= $signed (16'd27);
        16: z <= $signed (16'd64);
        17: z <= $signed (16'd64);
        18: z <= $signed (16'd116);
        19: z <= $signed (-16'd91);
        20: z <= $signed (16'd12);
        21: z <= $signed (-16'd42);
        22: z <= $signed (16'd123);
        23: z <= $signed (-16'd16);
        24: z <= $signed (-16'd88);
        25: z <= $signed (16'd97);
        26: z <= $signed (16'd89);
        27: z <= $signed (16'd16);
        28: z <= $signed (16'd88);
        29: z <= $signed (-16'd57);
        30: z <= $signed (16'd109);
        31: z <= $signed (-16'd68);
        32: z <= $signed (16'd69);
        33: z <= $signed (-16'd16);
        34: z <= $signed (16'd84);
        35: z <= $signed (-16'd77);
        36: z <= $signed (16'd113);
        37: z <= $signed (-16'd74);
        38: z <= $signed (-16'd8);
        39: z <= $signed (16'd92);
        40: z <= $signed (-16'd77);
        41: z <= $signed (-16'd11);
        42: z <= $signed (16'd5);
        43: z <= $signed (-16'd124);
        44: z <= $signed (16'd67);
        45: z <= $signed (-16'd6);
        46: z <= $signed (-16'd91);
        47: z <= $signed (16'd88);
      endcase
   end
endmodule

module layer3_16_12_4_16_W2_rom(clk, addr, z);
   input clk;
   input [6:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd81);
        1: z <= $signed (16'd69);
        2: z <= $signed (16'd96);
        3: z <= $signed (-16'd29);
        4: z <= $signed (16'd20);
        5: z <= $signed (-16'd5);
        6: z <= $signed (-16'd31);
        7: z <= $signed (-16'd18);
        8: z <= $signed (-16'd81);
        9: z <= $signed (16'd87);
        10: z <= $signed (16'd10);
        11: z <= $signed (-16'd39);
        12: z <= $signed (16'd12);
        13: z <= $signed (-16'd4);
        14: z <= $signed (-16'd12);
        15: z <= $signed (-16'd41);
        16: z <= $signed (16'd87);
        17: z <= $signed (-16'd30);
        18: z <= $signed (-16'd65);
        19: z <= $signed (16'd49);
        20: z <= $signed (-16'd25);
        21: z <= $signed (16'd118);
        22: z <= $signed (-16'd18);
        23: z <= $signed (-16'd35);
        24: z <= $signed (-16'd6);
        25: z <= $signed (-16'd77);
        26: z <= $signed (-16'd115);
        27: z <= $signed (-16'd18);
        28: z <= $signed (16'd71);
        29: z <= $signed (16'd1);
        30: z <= $signed (-16'd93);
        31: z <= $signed (-16'd100);
        32: z <= $signed (16'd106);
        33: z <= $signed (16'd124);
        34: z <= $signed (16'd125);
        35: z <= $signed (16'd41);
        36: z <= $signed (-16'd110);
        37: z <= $signed (-16'd17);
        38: z <= $signed (-16'd107);
        39: z <= $signed (16'd11);
        40: z <= $signed (16'd17);
        41: z <= $signed (16'd98);
        42: z <= $signed (16'd92);
        43: z <= $signed (16'd123);
        44: z <= $signed (16'd28);
        45: z <= $signed (-16'd23);
        46: z <= $signed (-16'd85);
        47: z <= $signed (16'd71);
      endcase
   end
endmodule

module layer3_16_12_4_16_W3_rom(clk, addr, z);
   input clk;
   input [6:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd28);
        1: z <= $signed (16'd76);
        2: z <= $signed (-16'd31);
        3: z <= $signed (16'd86);
        4: z <= $signed (16'd100);
        5: z <= $signed (-16'd74);
        6: z <= $signed (16'd1);
        7: z <= $signed (-16'd124);
        8: z <= $signed (16'd74);
        9: z <= $signed (-16'd37);
        10: z <= $signed (16'd110);
        11: z <= $signed (16'd104);
        12: z <= $signed (-16'd28);
        13: z <= $signed (-16'd2);
        14: z <= $signed (16'd2);
        15: z <= $signed (16'd23);
        16: z <= $signed (-16'd18);
        17: z <= $signed (-16'd118);
        18: z <= $signed (-16'd33);
        19: z <= $signed (16'd89);
        20: z <= $signed (16'd116);
        21: z <= $signed (16'd20);
        22: z <= $signed (-16'd12);
        23: z <= $signed (16'd52);
        24: z <= $signed (16'd81);
        25: z <= $signed (-16'd50);
        26: z <= $signed (16'd117);
        27: z <= $signed (-16'd95);
        28: z <= $signed (-16'd108);
        29: z <= $signed (16'd23);
        30: z <= $signed (-16'd55);
        31: z <= $signed (-16'd67);
        32: z <= $signed (-16'd7);
        33: z <= $signed (-16'd94);
        34: z <= $signed (16'd77);
        35: z <= $signed (-16'd47);
        36: z <= $signed (16'd75);
        37: z <= $signed (16'd14);
        38: z <= $signed (-16'd20);
        39: z <= $signed (-16'd33);
        40: z <= $signed (16'd126);
        41: z <= $signed (-16'd1);
        42: z <= $signed (16'd59);
        43: z <= $signed (16'd111);
        44: z <= $signed (16'd54);
        45: z <= $signed (-16'd77);
        46: z <= $signed (16'd75);
        47: z <= $signed (16'd105);
      endcase
   end
endmodule

module layer3_16_12_4_16_W4_rom(clk, addr, z);
   input clk;
   input [6:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd89);
        1: z <= $signed (-16'd123);
        2: z <= $signed (16'd55);
        3: z <= $signed (16'd61);
        4: z <= $signed (16'd118);
        5: z <= $signed (-16'd18);
        6: z <= $signed (-16'd113);
        7: z <= $signed (-16'd91);
        8: z <= $signed (-16'd77);
        9: z <= $signed (16'd112);
        10: z <= $signed (16'd8);
        11: z <= $signed (16'd71);
        12: z <= $signed (-16'd44);
        13: z <= $signed (-16'd23);
        14: z <= $signed (16'd89);
        15: z <= $signed (16'd97);
        16: z <= $signed (16'd63);
        17: z <= $signed (16'd84);
        18: z <= $signed (-16'd47);
        19: z <= $signed (-16'd53);
        20: z <= $signed (-16'd48);
        21: z <= $signed (16'd70);
        22: z <= $signed (16'd34);
        23: z <= $signed (-16'd88);
        24: z <= $signed (-16'd22);
        25: z <= $signed (16'd58);
        26: z <= $signed (16'd13);
        27: z <= $signed (-16'd81);
        28: z <= $signed (-16'd86);
        29: z <= $signed (-16'd31);
        30: z <= $signed (-16'd29);
        31: z <= $signed (16'd37);
        32: z <= $signed (16'd20);
        33: z <= $signed (-16'd16);
        34: z <= $signed (-16'd109);
        35: z <= $signed (-16'd36);
        36: z <= $signed (16'd40);
        37: z <= $signed (-16'd47);
        38: z <= $signed (16'd109);
        39: z <= $signed (-16'd20);
        40: z <= $signed (16'd75);
        41: z <= $signed (-16'd110);
        42: z <= $signed (-16'd60);
        43: z <= $signed (16'd93);
        44: z <= $signed (16'd1);
        45: z <= $signed (-16'd39);
        46: z <= $signed (-16'd24);
        47: z <= $signed (-16'd109);
      endcase
   end
endmodule

module layer3_16_12_4_16_B1_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd69);
        1: z <= $signed (16'd46);
        2: z <= $signed (16'd71);
        3: z <= $signed (16'd10);
      endcase
   end
endmodule

module layer3_16_12_4_16_B2_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd60);
        1: z <= $signed (-16'd71);
        2: z <= $signed (-16'd118);
        3: z <= $signed (16'd20);
      endcase
   end
endmodule

module layer3_16_12_4_16_B3_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd114);
        1: z <= $signed (16'd30);
        2: z <= $signed (16'd88);
        3: z <= $signed (16'd53);
      endcase
   end
endmodule

module layer3_16_12_4_16_B4_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd87);
        1: z <= $signed (-16'd7);
        2: z <= $signed (16'd69);
        3: z <= $signed (-16'd64);
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

