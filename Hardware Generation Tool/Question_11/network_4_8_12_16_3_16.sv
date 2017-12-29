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


module network_4_8_12_16_3_16(clk, reset, s_valid, m_ready, data_in, m_valid, s_ready, data_out);

parameter T=16;

  input clk, reset, s_valid, m_ready; // Control Signals 
  input signed [T-1:0] data_in;
  output logic signed [T-1:0] data_out;
  output m_valid, s_ready;
  logic MS_ready_l2, MS_ready_l3;   // Interconnecting Layer parameters  
  logic MS_valid_l2, MS_valid_l3;
  logic signed [T-1:0] data_IO_l2, data_IO_l3;

layer1_8_4_1_16 Net1(clk, reset, s_valid, MS_ready_l2, data_in, MS_valid_l2, s_ready, data_IO_l2);   // Layer 1 GPIO
layer2_12_8_1_16 Net2(clk, reset, MS_valid_l2, MS_ready_l3, data_IO_l2, MS_valid_l3, MS_ready_l2, data_IO_l3);   // Layer 2 GPIO
layer3_16_12_1_16 Net3(clk, reset, MS_valid_l3, m_ready, data_IO_l3, m_valid, MS_ready_l3, data_out);   // Layer 3 GPIO

endmodule
//=================================================MAIN MODULE=====================================================
module layer1_8_4_1_16(clk, reset, s_valid, m_ready, data_in, m_valid, s_ready, data_out);  

parameter N=4; parameter M=8; parameter T=16; parameter P=1;
parameter LOGM=3; parameter LOGW=5; parameter LOGN=2; parameter LOGB=2;

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

parameter N=4; parameter M=8; parameter T=16; parameter P=1;
parameter LOGM=3; parameter LOGW=5; parameter LOGN=2; parameter LOGB=2;

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

parameter N=4; parameter M=8; parameter T=16; parameter P=1;
parameter LOGM=3; parameter LOGW=5; parameter LOGN=2; parameter LOGB=2;

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

 assign data_y1_R= ((wr_en_y==1)&&(data_y1[T-1]==1))? 0:data_y1;

always_comb begin
 if(counter %P == 0) data_out = data_out1;
 else data_out = data_out0;
end

 memory #(T,N,LOGN+1) x(clk, data_in, data_x, addr_x, wr_en_x);                     // Vector Memory Instantiation
 memory #(T,M,LOGM+1) y1(clk, data_y1_R, data_out1, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer1_8_4_1_16_B1_rom lb1 (clk, addr_b, data_b1);
 layer1_8_4_1_16_W1_rom lw1 (clk, addr_m, data_m1);
 mac ma11(clk, reset, clear_acc,data_b1, data_m1, data_x, data_y1);               // MAC Module Instantiation


endmodule

//==================================================ROM  MODULE=====================================================
module layer1_8_4_1_16_W1_rom(clk, addr, z);
   input clk;
   input [5:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd22);
        1: z <= $signed (16'd71);
        2: z <= $signed (16'd57);
        3: z <= $signed (-16'd76);
        4: z <= $signed (-16'd84);
        5: z <= $signed (-16'd123);
        6: z <= $signed (16'd38);
        7: z <= $signed (-16'd82);
        8: z <= $signed (-16'd49);
        9: z <= $signed (16'd6);
        10: z <= $signed (16'd91);
        11: z <= $signed (-16'd74);
        12: z <= $signed (-16'd111);
        13: z <= $signed (-16'd39);
        14: z <= $signed (16'd9);
        15: z <= $signed (16'd123);
        16: z <= $signed (16'd74);
        17: z <= $signed (16'd107);
        18: z <= $signed (-16'd62);
        19: z <= $signed (-16'd3);
        20: z <= $signed (16'd108);
        21: z <= $signed (16'd114);
        22: z <= $signed (16'd32);
        23: z <= $signed (-16'd114);
        24: z <= $signed (-16'd48);
        25: z <= $signed (-16'd72);
        26: z <= $signed (-16'd75);
        27: z <= $signed (-16'd63);
        28: z <= $signed (16'd53);
        29: z <= $signed (16'd4);
        30: z <= $signed (-16'd117);
        31: z <= $signed (-16'd52);
      endcase
   end
endmodule

module layer1_8_4_1_16_B1_rom(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd53);
        1: z <= $signed (16'd68);
        2: z <= $signed (16'd0);
        3: z <= $signed (-16'd9);
        4: z <= $signed (16'd74);
        5: z <= $signed (-16'd89);
        6: z <= $signed (16'd37);
        7: z <= $signed (-16'd103);
      endcase
   end
endmodule

//=================================================MAIN MODULE=====================================================
module layer2_12_8_1_16(clk, reset, s_valid, m_ready, data_in, m_valid, s_ready, data_out);  

parameter N=8; parameter M=12; parameter T=16; parameter P=1;
parameter LOGM=4; parameter LOGW=7; parameter LOGN=3; parameter LOGB=3;

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

parameter N=8; parameter M=12; parameter T=16; parameter P=1;
parameter LOGM=4; parameter LOGW=7; parameter LOGN=3; parameter LOGB=3;

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

parameter N=8; parameter M=12; parameter T=16; parameter P=1;
parameter LOGM=4; parameter LOGW=7; parameter LOGN=3; parameter LOGB=3;

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

 assign data_y1_R= ((wr_en_y==1)&&(data_y1[T-1]==1))? 0:data_y1;

always_comb begin
 if(counter %P == 0) data_out = data_out1;
 else data_out = data_out0;
end

 memory #(T,N,LOGN+1) x(clk, data_in, data_x, addr_x, wr_en_x);                     // Vector Memory Instantiation
 memory #(T,M,LOGM+1) y1(clk, data_y1_R, data_out1, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer2_12_8_1_16_B1_rom lb1 (clk, addr_b, data_b1);
 layer2_12_8_1_16_W1_rom lw1 (clk, addr_m, data_m1);
 mac ma21(clk, reset, clear_acc,data_b1, data_m1, data_x, data_y1);               // MAC Module Instantiation


endmodule

//==================================================ROM  MODULE=====================================================
module layer2_12_8_1_16_W1_rom(clk, addr, z);
   input clk;
   input [7:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd45);
        1: z <= $signed (16'd0);
        2: z <= $signed (-16'd48);
        3: z <= $signed (16'd62);
        4: z <= $signed (16'd89);
        5: z <= $signed (16'd89);
        6: z <= $signed (16'd57);
        7: z <= $signed (16'd35);
        8: z <= $signed (16'd68);
        9: z <= $signed (16'd123);
        10: z <= $signed (-16'd96);
        11: z <= $signed (16'd48);
        12: z <= $signed (16'd109);
        13: z <= $signed (16'd64);
        14: z <= $signed (16'd62);
        15: z <= $signed (-16'd66);
        16: z <= $signed (16'd120);
        17: z <= $signed (16'd115);
        18: z <= $signed (-16'd1);
        19: z <= $signed (16'd46);
        20: z <= $signed (-16'd9);
        21: z <= $signed (16'd10);
        22: z <= $signed (16'd122);
        23: z <= $signed (16'd66);
        24: z <= $signed (-16'd49);
        25: z <= $signed (-16'd6);
        26: z <= $signed (-16'd71);
        27: z <= $signed (-16'd103);
        28: z <= $signed (16'd33);
        29: z <= $signed (16'd94);
        30: z <= $signed (-16'd78);
        31: z <= $signed (-16'd50);
        32: z <= $signed (-16'd34);
        33: z <= $signed (16'd2);
        34: z <= $signed (-16'd116);
        35: z <= $signed (-16'd73);
        36: z <= $signed (-16'd37);
        37: z <= $signed (16'd69);
        38: z <= $signed (16'd90);
        39: z <= $signed (-16'd97);
        40: z <= $signed (16'd64);
        41: z <= $signed (16'd122);
        42: z <= $signed (16'd79);
        43: z <= $signed (16'd46);
        44: z <= $signed (16'd58);
        45: z <= $signed (16'd13);
        46: z <= $signed (16'd108);
        47: z <= $signed (16'd50);
        48: z <= $signed (16'd0);
        49: z <= $signed (-16'd21);
        50: z <= $signed (-16'd32);
        51: z <= $signed (16'd119);
        52: z <= $signed (16'd118);
        53: z <= $signed (-16'd38);
        54: z <= $signed (16'd57);
        55: z <= $signed (-16'd59);
        56: z <= $signed (16'd85);
        57: z <= $signed (16'd114);
        58: z <= $signed (-16'd34);
        59: z <= $signed (-16'd10);
        60: z <= $signed (16'd80);
        61: z <= $signed (16'd16);
        62: z <= $signed (16'd69);
        63: z <= $signed (-16'd82);
        64: z <= $signed (-16'd109);
        65: z <= $signed (16'd81);
        66: z <= $signed (-16'd27);
        67: z <= $signed (-16'd18);
        68: z <= $signed (16'd23);
        69: z <= $signed (-16'd65);
        70: z <= $signed (16'd14);
        71: z <= $signed (-16'd41);
        72: z <= $signed (-16'd71);
        73: z <= $signed (-16'd35);
        74: z <= $signed (-16'd123);
        75: z <= $signed (16'd115);
        76: z <= $signed (16'd107);
        77: z <= $signed (16'd113);
        78: z <= $signed (16'd38);
        79: z <= $signed (-16'd21);
        80: z <= $signed (-16'd35);
        81: z <= $signed (-16'd122);
        82: z <= $signed (-16'd29);
        83: z <= $signed (-16'd45);
        84: z <= $signed (-16'd31);
        85: z <= $signed (-16'd100);
        86: z <= $signed (16'd24);
        87: z <= $signed (-16'd74);
        88: z <= $signed (-16'd113);
        89: z <= $signed (16'd118);
        90: z <= $signed (16'd44);
        91: z <= $signed (16'd95);
        92: z <= $signed (16'd6);
        93: z <= $signed (-16'd15);
        94: z <= $signed (-16'd114);
        95: z <= $signed (16'd25);
      endcase
   end
endmodule

module layer2_12_8_1_16_B1_rom(clk, addr, z);
   input clk;
   input [3:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd61);
        1: z <= $signed (-16'd13);
        2: z <= $signed (-16'd120);
        3: z <= $signed (16'd90);
        4: z <= $signed (16'd51);
        5: z <= $signed (16'd22);
        6: z <= $signed (-16'd79);
        7: z <= $signed (16'd108);
        8: z <= $signed (16'd115);
        9: z <= $signed (-16'd73);
        10: z <= $signed (16'd96);
        11: z <= $signed (16'd94);
      endcase
   end
endmodule

//=================================================MAIN MODULE=====================================================
module layer3_16_12_1_16(clk, reset, s_valid, m_ready, data_in, m_valid, s_ready, data_out);  

parameter N=12; parameter M=16; parameter T=16; parameter P=1;
parameter LOGM=4; parameter LOGW=8; parameter LOGN=4; parameter LOGB=3;

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

parameter N=12; parameter M=16; parameter T=16; parameter P=1;
parameter LOGM=4; parameter LOGW=8; parameter LOGN=4; parameter LOGB=3;

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

parameter N=12; parameter M=16; parameter T=16; parameter P=1;
parameter LOGM=4; parameter LOGW=8; parameter LOGN=4; parameter LOGB=3;

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

 assign data_y1_R= ((wr_en_y==1)&&(data_y1[T-1]==1))? 0:data_y1;

always_comb begin
 if(counter %P == 0) data_out = data_out1;
 else data_out = data_out0;
end

 memory #(T,N,LOGN+1) x(clk, data_in, data_x, addr_x, wr_en_x);                     // Vector Memory Instantiation
 memory #(T,M,LOGM+1) y1(clk, data_y1_R, data_out1, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer3_16_12_1_16_B1_rom lb1 (clk, addr_b, data_b1);
 layer3_16_12_1_16_W1_rom lw1 (clk, addr_m, data_m1);
 mac ma31(clk, reset, clear_acc,data_b1, data_m1, data_x, data_y1);               // MAC Module Instantiation


endmodule

//==================================================ROM  MODULE=====================================================
module layer3_16_12_1_16_W1_rom(clk, addr, z);
   input clk;
   input [8:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd88);
        1: z <= $signed (16'd6);
        2: z <= $signed (-16'd54);
        3: z <= $signed (16'd5);
        4: z <= $signed (16'd12);
        5: z <= $signed (16'd45);
        6: z <= $signed (16'd88);
        7: z <= $signed (16'd109);
        8: z <= $signed (16'd73);
        9: z <= $signed (-16'd16);
        10: z <= $signed (-16'd93);
        11: z <= $signed (16'd88);
        12: z <= $signed (-16'd26);
        13: z <= $signed (16'd80);
        14: z <= $signed (16'd56);
        15: z <= $signed (16'd109);
        16: z <= $signed (-16'd63);
        17: z <= $signed (16'd70);
        18: z <= $signed (16'd6);
        19: z <= $signed (16'd4);
        20: z <= $signed (-16'd71);
        21: z <= $signed (16'd14);
        22: z <= $signed (-16'd34);
        23: z <= $signed (16'd108);
        24: z <= $signed (-16'd92);
        25: z <= $signed (16'd16);
        26: z <= $signed (16'd89);
        27: z <= $signed (-16'd104);
        28: z <= $signed (16'd71);
        29: z <= $signed (16'd57);
        30: z <= $signed (16'd118);
        31: z <= $signed (16'd111);
        32: z <= $signed (-16'd65);
        33: z <= $signed (-16'd64);
        34: z <= $signed (-16'd11);
        35: z <= $signed (16'd75);
        36: z <= $signed (16'd109);
        37: z <= $signed (-16'd51);
        38: z <= $signed (16'd57);
        39: z <= $signed (16'd55);
        40: z <= $signed (16'd62);
        41: z <= $signed (16'd92);
        42: z <= $signed (16'd15);
        43: z <= $signed (-16'd92);
        44: z <= $signed (16'd44);
        45: z <= $signed (-16'd57);
        46: z <= $signed (-16'd111);
        47: z <= $signed (16'd110);
        48: z <= $signed (-16'd115);
        49: z <= $signed (16'd24);
        50: z <= $signed (-16'd14);
        51: z <= $signed (-16'd57);
        52: z <= $signed (-16'd90);
        53: z <= $signed (16'd81);
        54: z <= $signed (-16'd77);
        55: z <= $signed (-16'd53);
        56: z <= $signed (-16'd31);
        57: z <= $signed (-16'd116);
        58: z <= $signed (-16'd29);
        59: z <= $signed (-16'd88);
        60: z <= $signed (16'd69);
        61: z <= $signed (-16'd39);
        62: z <= $signed (-16'd105);
        63: z <= $signed (-16'd124);
        64: z <= $signed (16'd26);
        65: z <= $signed (16'd12);
        66: z <= $signed (16'd80);
        67: z <= $signed (16'd7);
        68: z <= $signed (16'd90);
        69: z <= $signed (16'd9);
        70: z <= $signed (-16'd66);
        71: z <= $signed (16'd24);
        72: z <= $signed (-16'd27);
        73: z <= $signed (16'd78);
        74: z <= $signed (16'd60);
        75: z <= $signed (-16'd110);
        76: z <= $signed (-16'd107);
        77: z <= $signed (16'd78);
        78: z <= $signed (-16'd128);
        79: z <= $signed (-16'd93);
        80: z <= $signed (-16'd26);
        81: z <= $signed (-16'd14);
        82: z <= $signed (-16'd22);
        83: z <= $signed (16'd12);
        84: z <= $signed (-16'd61);
        85: z <= $signed (16'd29);
        86: z <= $signed (16'd87);
        87: z <= $signed (16'd36);
        88: z <= $signed (16'd42);
        89: z <= $signed (-16'd70);
        90: z <= $signed (16'd76);
        91: z <= $signed (-16'd17);
        92: z <= $signed (16'd20);
        93: z <= $signed (16'd100);
        94: z <= $signed (-16'd12);
        95: z <= $signed (-16'd82);
        96: z <= $signed (-16'd16);
        97: z <= $signed (-16'd60);
        98: z <= $signed (16'd53);
        99: z <= $signed (-16'd54);
        100: z <= $signed (16'd77);
        101: z <= $signed (16'd116);
        102: z <= $signed (16'd98);
        103: z <= $signed (-16'd78);
        104: z <= $signed (16'd66);
        105: z <= $signed (16'd31);
        106: z <= $signed (-16'd60);
        107: z <= $signed (16'd87);
        108: z <= $signed (-16'd19);
        109: z <= $signed (-16'd60);
        110: z <= $signed (16'd122);
        111: z <= $signed (16'd83);
        112: z <= $signed (16'd55);
        113: z <= $signed (-16'd28);
        114: z <= $signed (-16'd33);
        115: z <= $signed (16'd122);
        116: z <= $signed (-16'd126);
        117: z <= $signed (-16'd73);
        118: z <= $signed (16'd31);
        119: z <= $signed (16'd44);
        120: z <= $signed (-16'd15);
        121: z <= $signed (-16'd21);
        122: z <= $signed (-16'd101);
        123: z <= $signed (-16'd123);
        124: z <= $signed (-16'd49);
        125: z <= $signed (16'd15);
        126: z <= $signed (-16'd77);
        127: z <= $signed (16'd64);
        128: z <= $signed (16'd83);
        129: z <= $signed (16'd105);
        130: z <= $signed (-16'd118);
        131: z <= $signed (16'd32);
        132: z <= $signed (16'd93);
        133: z <= $signed (16'd109);
        134: z <= $signed (16'd83);
        135: z <= $signed (16'd31);
        136: z <= $signed (16'd12);
        137: z <= $signed (-16'd105);
        138: z <= $signed (-16'd10);
        139: z <= $signed (16'd121);
        140: z <= $signed (-16'd36);
        141: z <= $signed (-16'd15);
        142: z <= $signed (16'd76);
        143: z <= $signed (-16'd109);
        144: z <= $signed (16'd85);
        145: z <= $signed (-16'd85);
        146: z <= $signed (-16'd115);
        147: z <= $signed (16'd87);
        148: z <= $signed (-16'd30);
        149: z <= $signed (16'd44);
        150: z <= $signed (16'd3);
        151: z <= $signed (16'd84);
        152: z <= $signed (-16'd104);
        153: z <= $signed (16'd31);
        154: z <= $signed (16'd89);
        155: z <= $signed (-16'd25);
        156: z <= $signed (-16'd82);
        157: z <= $signed (-16'd115);
        158: z <= $signed (-16'd89);
        159: z <= $signed (-16'd126);
        160: z <= $signed (16'd118);
        161: z <= $signed (-16'd78);
        162: z <= $signed (16'd34);
        163: z <= $signed (16'd83);
        164: z <= $signed (-16'd97);
        165: z <= $signed (-16'd11);
        166: z <= $signed (-16'd14);
        167: z <= $signed (16'd43);
        168: z <= $signed (16'd13);
        169: z <= $signed (16'd104);
        170: z <= $signed (16'd36);
        171: z <= $signed (16'd105);
        172: z <= $signed (-16'd39);
        173: z <= $signed (-16'd16);
        174: z <= $signed (16'd124);
        175: z <= $signed (-16'd81);
        176: z <= $signed (16'd27);
        177: z <= $signed (-16'd119);
        178: z <= $signed (-16'd122);
        179: z <= $signed (16'd126);
        180: z <= $signed (16'd54);
        181: z <= $signed (16'd10);
        182: z <= $signed (16'd82);
        183: z <= $signed (16'd78);
        184: z <= $signed (-16'd87);
        185: z <= $signed (16'd43);
        186: z <= $signed (-16'd75);
        187: z <= $signed (-16'd41);
        188: z <= $signed (16'd56);
        189: z <= $signed (-16'd35);
        190: z <= $signed (-16'd39);
        191: z <= $signed (16'd46);
      endcase
   end
endmodule

module layer3_16_12_1_16_B1_rom(clk, addr, z);
   input clk;
   input [3:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd15);
        1: z <= $signed (16'd124);
        2: z <= $signed (16'd1);
        3: z <= $signed (16'd46);
        4: z <= $signed (-16'd15);
        5: z <= $signed (16'd115);
        6: z <= $signed (-16'd39);
        7: z <= $signed (16'd126);
        8: z <= $signed (16'd92);
        9: z <= $signed (16'd125);
        10: z <= $signed (16'd103);
        11: z <= $signed (-16'd75);
        12: z <= $signed (-16'd19);
        13: z <= $signed (16'd99);
        14: z <= $signed (-16'd28);
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

