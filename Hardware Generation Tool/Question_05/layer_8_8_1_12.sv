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
module layer_8_8_1_12(clk, reset, s_valid, m_ready, data_in, m_valid, s_ready, data_out);  

parameter N=8; parameter M=8; parameter T=12; parameter P=1;
parameter LOGM=3; parameter LOGW=6; parameter LOGN=3; parameter LOGB=2;

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

parameter N=8; parameter M=8; parameter T=12; parameter P=1;
parameter LOGM=3; parameter LOGW=6; parameter LOGN=3; parameter LOGB=2;

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

parameter N=8; parameter M=8; parameter T=12; parameter P=1;
parameter LOGM=3; parameter LOGW=6; parameter LOGN=3; parameter LOGB=2;

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
 layer_8_8_1_12_B1_rom lb1 (clk, addr_b, data_b1);
 layer_8_8_1_12_W1_rom lw1 (clk, addr_m, data_m1);
 mac ma11(clk, reset, clear_acc,data_b1, data_m1, data_x, data_y1);               // MAC Module Instantiation


endmodule

//==================================================ROM  MODULE=====================================================
module layer_8_8_1_12_W1_rom(clk, addr, z);
   input clk;
   input [5:0] addr;
   output logic signed [11:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (12'd24);
        1: z <= $signed (12'd1);
        2: z <= $signed (12'd19);
        3: z <= $signed (-12'd24);
        4: z <= $signed (12'd12);
        5: z <= $signed (-12'd21);
        6: z <= $signed (-12'd15);
        7: z <= $signed (-12'd4);
        8: z <= $signed (12'd12);
        9: z <= $signed (-12'd11);
        10: z <= $signed (12'd13);
        11: z <= $signed (-12'd16);
        12: z <= $signed (-12'd16);
        13: z <= $signed (-12'd3);
        14: z <= $signed (12'd0);
        15: z <= $signed (-12'd15);
        16: z <= $signed (-12'd26);
        17: z <= $signed (-12'd24);
        18: z <= $signed (-12'd2);
        19: z <= $signed (12'd25);
        20: z <= $signed (-12'd26);
        21: z <= $signed (-12'd22);
        22: z <= $signed (12'd25);
        23: z <= $signed (-12'd17);
        24: z <= $signed (-12'd13);
        25: z <= $signed (12'd16);
        26: z <= $signed (-12'd6);
        27: z <= $signed (-12'd26);
        28: z <= $signed (-12'd9);
        29: z <= $signed (-12'd9);
        30: z <= $signed (-12'd6);
        31: z <= $signed (-12'd17);
        32: z <= $signed (12'd24);
        33: z <= $signed (-12'd19);
        34: z <= $signed (-12'd8);
        35: z <= $signed (12'd4);
        36: z <= $signed (-12'd8);
        37: z <= $signed (12'd9);
        38: z <= $signed (-12'd32);
        39: z <= $signed (-12'd28);
        40: z <= $signed (12'd30);
        41: z <= $signed (12'd14);
        42: z <= $signed (-12'd11);
        43: z <= $signed (-12'd18);
        44: z <= $signed (-12'd21);
        45: z <= $signed (12'd21);
        46: z <= $signed (-12'd1);
        47: z <= $signed (-12'd15);
        48: z <= $signed (12'd29);
        49: z <= $signed (12'd30);
        50: z <= $signed (-12'd22);
        51: z <= $signed (-12'd29);
        52: z <= $signed (-12'd24);
        53: z <= $signed (-12'd28);
        54: z <= $signed (-12'd14);
        55: z <= $signed (-12'd5);
        56: z <= $signed (12'd20);
        57: z <= $signed (12'd13);
        58: z <= $signed (12'd2);
        59: z <= $signed (-12'd21);
        60: z <= $signed (-12'd28);
        61: z <= $signed (12'd28);
        62: z <= $signed (-12'd5);
        63: z <= $signed (12'd28);
      endcase
   end
endmodule

module layer_8_8_1_12_B1_rom(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [11:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-12'd23);
        1: z <= $signed (12'd19);
        2: z <= $signed (12'd1);
        3: z <= $signed (12'd2);
        4: z <= $signed (-12'd4);
        5: z <= $signed (12'd1);
        6: z <= $signed (12'd6);
        7: z <= $signed (-12'd5);
      endcase
   end
endmodule

//==================================================MAC  MODULE=====================================================

module mac(clk, reset, clear_acc,data_b, data_m, data_x, data_y);

parameter T=12;

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

