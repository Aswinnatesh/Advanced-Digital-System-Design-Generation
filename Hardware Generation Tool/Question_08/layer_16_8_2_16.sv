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
module layer_16_8_2_16(clk, reset, s_valid, m_ready, data_in, m_valid, s_ready, data_out);  

parameter N=8; parameter M=16; parameter T=16; parameter P=2;
parameter LOGM=4; parameter LOGW=6; parameter LOGN=3; parameter LOGB=2;

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

parameter N=8; parameter M=16; parameter T=16; parameter P=2;
parameter LOGM=4; parameter LOGW=6; parameter LOGN=3; parameter LOGB=2;

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

parameter N=8; parameter M=16; parameter T=16; parameter P=2;
parameter LOGM=4; parameter LOGW=6; parameter LOGN=3; parameter LOGB=2;

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
 layer_16_8_2_16_B1_rom lb1 (clk, addr_b, data_b1);
 layer_16_8_2_16_W1_rom lw1 (clk, addr_m, data_m1);
 mac ma11(clk, reset, clear_acc,data_b1, data_m1, data_x, data_y1);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y2(clk, data_y2_R, data_out2, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer_16_8_2_16_B2_rom lb2 (clk, addr_b, data_b2);
 layer_16_8_2_16_W2_rom lw2 (clk, addr_m, data_m2);
 mac ma12(clk, reset, clear_acc,data_b2, data_m2, data_x, data_y2);               // MAC Module Instantiation


endmodule

//==================================================ROM  MODULE=====================================================
module layer_16_8_2_16_W1_rom(clk, addr, z);
   input clk;
   input [5:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd49);
        1: z <= $signed (16'd7);
        2: z <= $signed (-16'd31);
        3: z <= $signed (16'd7);
        4: z <= $signed (16'd46);
        5: z <= $signed (-16'd34);
        6: z <= $signed (-16'd78);
        7: z <= $signed (-16'd45);
        8: z <= $signed (16'd26);
        9: z <= $signed (-16'd49);
        10: z <= $signed (16'd75);
        11: z <= $signed (16'd65);
        12: z <= $signed (-16'd128);
        13: z <= $signed (16'd100);
        14: z <= $signed (16'd8);
        15: z <= $signed (-16'd93);
        16: z <= $signed (16'd84);
        17: z <= $signed (16'd0);
        18: z <= $signed (16'd60);
        19: z <= $signed (16'd2);
        20: z <= $signed (16'd94);
        21: z <= $signed (16'd110);
        22: z <= $signed (16'd85);
        23: z <= $signed (16'd63);
        24: z <= $signed (-16'd43);
        25: z <= $signed (-16'd114);
        26: z <= $signed (16'd62);
        27: z <= $signed (-16'd42);
        28: z <= $signed (16'd115);
        29: z <= $signed (-16'd58);
        30: z <= $signed (-16'd7);
        31: z <= $signed (16'd31);
        32: z <= $signed (-16'd65);
        33: z <= $signed (-16'd102);
        34: z <= $signed (16'd126);
        35: z <= $signed (-16'd98);
        36: z <= $signed (-16'd119);
        37: z <= $signed (16'd83);
        38: z <= $signed (16'd93);
        39: z <= $signed (16'd77);
        40: z <= $signed (16'd15);
        41: z <= $signed (-16'd127);
        42: z <= $signed (16'd76);
        43: z <= $signed (16'd2);
        44: z <= $signed (-16'd57);
        45: z <= $signed (-16'd59);
        46: z <= $signed (-16'd95);
        47: z <= $signed (16'd10);
        48: z <= $signed (-16'd85);
        49: z <= $signed (16'd66);
        50: z <= $signed (16'd95);
        51: z <= $signed (-16'd76);
        52: z <= $signed (16'd22);
        53: z <= $signed (16'd60);
        54: z <= $signed (-16'd127);
        55: z <= $signed (-16'd8);
        56: z <= $signed (-16'd128);
        57: z <= $signed (-16'd71);
        58: z <= $signed (16'd82);
        59: z <= $signed (-16'd56);
        60: z <= $signed (-16'd2);
        61: z <= $signed (16'd115);
        62: z <= $signed (16'd82);
        63: z <= $signed (-16'd98);
      endcase
   end
endmodule

module layer_16_8_2_16_W2_rom(clk, addr, z);
   input clk;
   input [5:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd96);
        1: z <= $signed (16'd86);
        2: z <= $signed (-16'd71);
        3: z <= $signed (-16'd2);
        4: z <= $signed (16'd43);
        5: z <= $signed (-16'd44);
        6: z <= $signed (16'd73);
        7: z <= $signed (16'd83);
        8: z <= $signed (16'd44);
        9: z <= $signed (16'd124);
        10: z <= $signed (16'd98);
        11: z <= $signed (-16'd126);
        12: z <= $signed (16'd102);
        13: z <= $signed (-16'd52);
        14: z <= $signed (-16'd98);
        15: z <= $signed (-16'd75);
        16: z <= $signed (16'd68);
        17: z <= $signed (-16'd114);
        18: z <= $signed (-16'd67);
        19: z <= $signed (-16'd16);
        20: z <= $signed (-16'd29);
        21: z <= $signed (-16'd122);
        22: z <= $signed (-16'd61);
        23: z <= $signed (16'd125);
        24: z <= $signed (-16'd62);
        25: z <= $signed (-16'd37);
        26: z <= $signed (16'd33);
        27: z <= $signed (-16'd88);
        28: z <= $signed (16'd40);
        29: z <= $signed (16'd63);
        30: z <= $signed (-16'd34);
        31: z <= $signed (-16'd4);
        32: z <= $signed (16'd98);
        33: z <= $signed (-16'd102);
        34: z <= $signed (-16'd67);
        35: z <= $signed (-16'd59);
        36: z <= $signed (-16'd96);
        37: z <= $signed (16'd1);
        38: z <= $signed (-16'd62);
        39: z <= $signed (-16'd10);
        40: z <= $signed (16'd32);
        41: z <= $signed (16'd66);
        42: z <= $signed (16'd50);
        43: z <= $signed (-16'd56);
        44: z <= $signed (16'd2);
        45: z <= $signed (-16'd112);
        46: z <= $signed (16'd68);
        47: z <= $signed (16'd65);
        48: z <= $signed (16'd86);
        49: z <= $signed (-16'd65);
        50: z <= $signed (16'd61);
        51: z <= $signed (16'd119);
        52: z <= $signed (16'd64);
        53: z <= $signed (16'd127);
        54: z <= $signed (-16'd19);
        55: z <= $signed (-16'd49);
        56: z <= $signed (16'd54);
        57: z <= $signed (16'd4);
        58: z <= $signed (-16'd25);
        59: z <= $signed (-16'd72);
        60: z <= $signed (16'd21);
        61: z <= $signed (-16'd85);
        62: z <= $signed (16'd121);
        63: z <= $signed (16'd64);
      endcase
   end
endmodule

module layer_16_8_2_16_B1_rom(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd110);
        1: z <= $signed (16'd116);
        2: z <= $signed (16'd21);
        3: z <= $signed (16'd124);
        4: z <= $signed (-16'd76);
        5: z <= $signed (-16'd29);
        6: z <= $signed (16'd56);
        7: z <= $signed (-16'd60);
      endcase
   end
endmodule

module layer_16_8_2_16_B2_rom(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd89);
        1: z <= $signed (16'd4);
        2: z <= $signed (16'd117);
        3: z <= $signed (-16'd20);
        4: z <= $signed (16'd57);
        5: z <= $signed (16'd116);
        6: z <= $signed (16'd80);
        7: z <= $signed (16'd57);
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

