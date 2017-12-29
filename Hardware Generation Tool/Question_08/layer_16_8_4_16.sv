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
module layer_16_8_4_16(clk, reset, s_valid, m_ready, data_in, m_valid, s_ready, data_out);  

parameter N=8; parameter M=16; parameter T=16; parameter P=4;
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
  control_module1 c(clk, reset, s_valid,m_ready,addr_y, wr_en_y, addr_x, wr_en_x,addr_m,addr_b, m_valid,s_ready, clear_acc, counter);
  datapath_module1 d(clk, reset, counter, m_ready, clear_acc, data_in, addr_x, addr_m,addr_b, addr_y, wr_en_x, wr_en_y, data_out); 

endmodule

//==================================================CONTROL MODULE==================================================

 module control_module1 (clk, reset, s_valid,m_ready,addr_y, wr_en_y, addr_x, wr_en_x,addr_m,addr_b, m_valid,s_ready, clear_acc, counter);

parameter N=8; parameter M=16; parameter T=16; parameter P=4;
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
module datapath_module1 (clk, reset, counter, m_ready, clear_acc, data_in, addr_x, addr_m, addr_b, addr_y, wr_en_x, wr_en_y, data_out);    

parameter N=8; parameter M=16; parameter T=16; parameter P=4;
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
 layer_16_8_4_16_B1_rom lb1 (clk, addr_b, data_b1);
 layer_16_8_4_16_W1_rom lw1 (clk, addr_m, data_m1);
 mac ma11(clk, reset, clear_acc,data_b1, data_m1, data_x, data_y1);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y2(clk, data_y2_R, data_out2, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer_16_8_4_16_B2_rom lb2 (clk, addr_b, data_b2);
 layer_16_8_4_16_W2_rom lw2 (clk, addr_m, data_m2);
 mac ma12(clk, reset, clear_acc,data_b2, data_m2, data_x, data_y2);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y3(clk, data_y3_R, data_out3, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer_16_8_4_16_B3_rom lb3 (clk, addr_b, data_b3);
 layer_16_8_4_16_W3_rom lw3 (clk, addr_m, data_m3);
 mac ma13(clk, reset, clear_acc,data_b3, data_m3, data_x, data_y3);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y4(clk, data_y4_R, data_out4, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer_16_8_4_16_B4_rom lb4 (clk, addr_b, data_b4);
 layer_16_8_4_16_W4_rom lw4 (clk, addr_m, data_m4);
 mac ma14(clk, reset, clear_acc,data_b4, data_m4, data_x, data_y4);               // MAC Module Instantiation


endmodule

//==================================================ROM  MODULE=====================================================
module layer_16_8_4_16_W1_rom(clk, addr, z);
   input clk;
   input [5:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd38);
        1: z <= $signed (16'd33);
        2: z <= $signed (16'd109);
        3: z <= $signed (16'd24);
        4: z <= $signed (-16'd3);
        5: z <= $signed (-16'd45);
        6: z <= $signed (-16'd19);
        7: z <= $signed (16'd48);
        8: z <= $signed (16'd103);
        9: z <= $signed (16'd18);
        10: z <= $signed (16'd82);
        11: z <= $signed (-16'd28);
        12: z <= $signed (16'd101);
        13: z <= $signed (-16'd64);
        14: z <= $signed (-16'd108);
        15: z <= $signed (16'd3);
        16: z <= $signed (-16'd18);
        17: z <= $signed (16'd107);
        18: z <= $signed (16'd2);
        19: z <= $signed (-16'd45);
        20: z <= $signed (-16'd85);
        21: z <= $signed (16'd22);
        22: z <= $signed (16'd86);
        23: z <= $signed (16'd32);
        24: z <= $signed (-16'd1);
        25: z <= $signed (-16'd105);
        26: z <= $signed (16'd81);
        27: z <= $signed (16'd42);
        28: z <= $signed (16'd45);
        29: z <= $signed (16'd39);
        30: z <= $signed (-16'd54);
        31: z <= $signed (16'd63);
      endcase
   end
endmodule

module layer_16_8_4_16_W2_rom(clk, addr, z);
   input clk;
   input [5:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd29);
        1: z <= $signed (-16'd75);
        2: z <= $signed (-16'd24);
        3: z <= $signed (16'd89);
        4: z <= $signed (-16'd12);
        5: z <= $signed (-16'd72);
        6: z <= $signed (16'd114);
        7: z <= $signed (-16'd15);
        8: z <= $signed (-16'd11);
        9: z <= $signed (-16'd4);
        10: z <= $signed (-16'd36);
        11: z <= $signed (16'd105);
        12: z <= $signed (16'd53);
        13: z <= $signed (-16'd50);
        14: z <= $signed (-16'd37);
        15: z <= $signed (16'd72);
        16: z <= $signed (-16'd110);
        17: z <= $signed (-16'd77);
        18: z <= $signed (16'd9);
        19: z <= $signed (16'd71);
        20: z <= $signed (16'd1);
        21: z <= $signed (16'd100);
        22: z <= $signed (16'd15);
        23: z <= $signed (-16'd72);
        24: z <= $signed (16'd90);
        25: z <= $signed (16'd84);
        26: z <= $signed (16'd7);
        27: z <= $signed (-16'd36);
        28: z <= $signed (16'd56);
        29: z <= $signed (-16'd106);
        30: z <= $signed (16'd20);
        31: z <= $signed (16'd22);
      endcase
   end
endmodule

module layer_16_8_4_16_W3_rom(clk, addr, z);
   input clk;
   input [5:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd109);
        1: z <= $signed (-16'd24);
        2: z <= $signed (16'd30);
        3: z <= $signed (16'd68);
        4: z <= $signed (-16'd123);
        5: z <= $signed (-16'd41);
        6: z <= $signed (16'd119);
        7: z <= $signed (16'd31);
        8: z <= $signed (16'd54);
        9: z <= $signed (16'd121);
        10: z <= $signed (16'd12);
        11: z <= $signed (16'd60);
        12: z <= $signed (-16'd48);
        13: z <= $signed (16'd3);
        14: z <= $signed (-16'd37);
        15: z <= $signed (16'd78);
        16: z <= $signed (16'd93);
        17: z <= $signed (-16'd101);
        18: z <= $signed (16'd116);
        19: z <= $signed (-16'd83);
        20: z <= $signed (16'd31);
        21: z <= $signed (-16'd49);
        22: z <= $signed (16'd124);
        23: z <= $signed (16'd29);
        24: z <= $signed (-16'd78);
        25: z <= $signed (16'd8);
        26: z <= $signed (16'd67);
        27: z <= $signed (16'd81);
        28: z <= $signed (16'd87);
        29: z <= $signed (16'd63);
        30: z <= $signed (-16'd18);
        31: z <= $signed (-16'd105);
      endcase
   end
endmodule

module layer_16_8_4_16_W4_rom(clk, addr, z);
   input clk;
   input [5:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd2);
        1: z <= $signed (-16'd5);
        2: z <= $signed (16'd22);
        3: z <= $signed (-16'd23);
        4: z <= $signed (16'd96);
        5: z <= $signed (-16'd58);
        6: z <= $signed (16'd36);
        7: z <= $signed (-16'd70);
        8: z <= $signed (16'd126);
        9: z <= $signed (16'd113);
        10: z <= $signed (-16'd73);
        11: z <= $signed (16'd94);
        12: z <= $signed (-16'd73);
        13: z <= $signed (16'd92);
        14: z <= $signed (-16'd104);
        15: z <= $signed (-16'd98);
        16: z <= $signed (-16'd64);
        17: z <= $signed (-16'd77);
        18: z <= $signed (-16'd4);
        19: z <= $signed (-16'd9);
        20: z <= $signed (-16'd113);
        21: z <= $signed (16'd20);
        22: z <= $signed (16'd21);
        23: z <= $signed (-16'd3);
        24: z <= $signed (16'd115);
        25: z <= $signed (16'd106);
        26: z <= $signed (16'd14);
        27: z <= $signed (-16'd126);
        28: z <= $signed (-16'd1);
        29: z <= $signed (-16'd93);
        30: z <= $signed (16'd0);
        31: z <= $signed (16'd126);
      endcase
   end
endmodule

module layer_16_8_4_16_B1_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd70);
        1: z <= $signed (16'd120);
        2: z <= $signed (16'd71);
        3: z <= $signed (-16'd60);
      endcase
   end
endmodule

module layer_16_8_4_16_B2_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd47);
        1: z <= $signed (16'd115);
        2: z <= $signed (-16'd83);
        3: z <= $signed (16'd67);
      endcase
   end
endmodule

module layer_16_8_4_16_B3_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd41);
        1: z <= $signed (16'd38);
        2: z <= $signed (-16'd81);
        3: z <= $signed (-16'd106);
      endcase
   end
endmodule

module layer_16_8_4_16_B4_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd103);
        1: z <= $signed (16'd83);
        2: z <= $signed (16'd0);
        3: z <= $signed (-16'd10);
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

