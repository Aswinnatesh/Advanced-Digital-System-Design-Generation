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
module layer_16_8_8_16(clk, reset, s_valid, m_ready, data_in, m_valid, s_ready, data_out);  

parameter N=8; parameter M=16; parameter T=16; parameter P=8;
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
  control_module1 c(clk, reset, s_valid,m_ready,addr_y, wr_en_y, addr_x, wr_en_x,addr_m,addr_b, m_valid,s_ready, clear_acc, counter);
  datapath_module1 d(clk, reset, counter, m_ready, clear_acc, data_in, addr_x, addr_m,addr_b, addr_y, wr_en_x, wr_en_y, data_out); 

endmodule

//==================================================CONTROL MODULE==================================================

 module control_module1 (clk, reset, s_valid,m_ready,addr_y, wr_en_y, addr_x, wr_en_x,addr_m,addr_b, m_valid,s_ready, clear_acc, counter);

parameter N=8; parameter M=16; parameter T=16; parameter P=8;
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
module datapath_module1 (clk, reset, counter, m_ready, clear_acc, data_in, addr_x, addr_m, addr_b, addr_y, wr_en_x, wr_en_y, data_out);    

parameter N=8; parameter M=16; parameter T=16; parameter P=8;
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
 layer_16_8_8_16_B1_rom lb1 (clk, addr_b, data_b1);
 layer_16_8_8_16_W1_rom lw1 (clk, addr_m, data_m1);
 mac ma11(clk, reset, clear_acc,data_b1, data_m1, data_x, data_y1);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y2(clk, data_y2_R, data_out2, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer_16_8_8_16_B2_rom lb2 (clk, addr_b, data_b2);
 layer_16_8_8_16_W2_rom lw2 (clk, addr_m, data_m2);
 mac ma12(clk, reset, clear_acc,data_b2, data_m2, data_x, data_y2);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y3(clk, data_y3_R, data_out3, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer_16_8_8_16_B3_rom lb3 (clk, addr_b, data_b3);
 layer_16_8_8_16_W3_rom lw3 (clk, addr_m, data_m3);
 mac ma13(clk, reset, clear_acc,data_b3, data_m3, data_x, data_y3);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y4(clk, data_y4_R, data_out4, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer_16_8_8_16_B4_rom lb4 (clk, addr_b, data_b4);
 layer_16_8_8_16_W4_rom lw4 (clk, addr_m, data_m4);
 mac ma14(clk, reset, clear_acc,data_b4, data_m4, data_x, data_y4);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y5(clk, data_y5_R, data_out5, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer_16_8_8_16_B5_rom lb5 (clk, addr_b, data_b5);
 layer_16_8_8_16_W5_rom lw5 (clk, addr_m, data_m5);
 mac ma15(clk, reset, clear_acc,data_b5, data_m5, data_x, data_y5);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y6(clk, data_y6_R, data_out6, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer_16_8_8_16_B6_rom lb6 (clk, addr_b, data_b6);
 layer_16_8_8_16_W6_rom lw6 (clk, addr_m, data_m6);
 mac ma16(clk, reset, clear_acc,data_b6, data_m6, data_x, data_y6);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y7(clk, data_y7_R, data_out7, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer_16_8_8_16_B7_rom lb7 (clk, addr_b, data_b7);
 layer_16_8_8_16_W7_rom lw7 (clk, addr_m, data_m7);
 mac ma17(clk, reset, clear_acc,data_b7, data_m7, data_x, data_y7);               // MAC Module Instantiation

 memory #(T,M,LOGM+1) y8(clk, data_y8_R, data_out8, addr_y, wr_en_y);           // Output Data Memory Instantiation
 layer_16_8_8_16_B8_rom lb8 (clk, addr_b, data_b8);
 layer_16_8_8_16_W8_rom lw8 (clk, addr_m, data_m8);
 mac ma18(clk, reset, clear_acc,data_b8, data_m8, data_x, data_y8);               // MAC Module Instantiation


endmodule

//==================================================ROM  MODULE=====================================================
module layer_16_8_8_16_W1_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd49);
        1: z <= $signed (-16'd32);
        2: z <= $signed (-16'd126);
        3: z <= $signed (-16'd41);
        4: z <= $signed (-16'd81);
        5: z <= $signed (-16'd91);
        6: z <= $signed (16'd14);
        7: z <= $signed (-16'd88);
        8: z <= $signed (16'd3);
        9: z <= $signed (-16'd89);
        10: z <= $signed (-16'd62);
        11: z <= $signed (16'd121);
        12: z <= $signed (16'd69);
        13: z <= $signed (-16'd1);
        14: z <= $signed (16'd62);
        15: z <= $signed (16'd81);
      endcase
   end
endmodule

module layer_16_8_8_16_W2_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd48);
        1: z <= $signed (-16'd18);
        2: z <= $signed (-16'd61);
        3: z <= $signed (-16'd76);
        4: z <= $signed (-16'd14);
        5: z <= $signed (-16'd93);
        6: z <= $signed (16'd107);
        7: z <= $signed (-16'd100);
        8: z <= $signed (16'd127);
        9: z <= $signed (16'd56);
        10: z <= $signed (-16'd49);
        11: z <= $signed (16'd34);
        12: z <= $signed (16'd28);
        13: z <= $signed (16'd106);
        14: z <= $signed (16'd33);
        15: z <= $signed (-16'd37);
      endcase
   end
endmodule

module layer_16_8_8_16_W3_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd36);
        1: z <= $signed (16'd90);
        2: z <= $signed (16'd3);
        3: z <= $signed (-16'd109);
        4: z <= $signed (16'd4);
        5: z <= $signed (-16'd9);
        6: z <= $signed (16'd24);
        7: z <= $signed (-16'd55);
        8: z <= $signed (-16'd119);
        9: z <= $signed (16'd51);
        10: z <= $signed (16'd29);
        11: z <= $signed (16'd31);
        12: z <= $signed (-16'd34);
        13: z <= $signed (-16'd87);
        14: z <= $signed (16'd56);
        15: z <= $signed (-16'd43);
      endcase
   end
endmodule

module layer_16_8_8_16_W4_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd3);
        1: z <= $signed (-16'd51);
        2: z <= $signed (-16'd70);
        3: z <= $signed (16'd75);
        4: z <= $signed (-16'd24);
        5: z <= $signed (16'd4);
        6: z <= $signed (-16'd50);
        7: z <= $signed (16'd56);
        8: z <= $signed (16'd111);
        9: z <= $signed (16'd29);
        10: z <= $signed (16'd53);
        11: z <= $signed (-16'd71);
        12: z <= $signed (16'd80);
        13: z <= $signed (16'd77);
        14: z <= $signed (-16'd24);
        15: z <= $signed (-16'd44);
      endcase
   end
endmodule

module layer_16_8_8_16_W5_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd100);
        1: z <= $signed (-16'd48);
        2: z <= $signed (-16'd113);
        3: z <= $signed (-16'd108);
        4: z <= $signed (-16'd11);
        5: z <= $signed (16'd29);
        6: z <= $signed (-16'd68);
        7: z <= $signed (16'd69);
        8: z <= $signed (16'd117);
        9: z <= $signed (16'd42);
        10: z <= $signed (-16'd51);
        11: z <= $signed (16'd58);
        12: z <= $signed (-16'd87);
        13: z <= $signed (-16'd117);
        14: z <= $signed (16'd11);
        15: z <= $signed (-16'd88);
      endcase
   end
endmodule

module layer_16_8_8_16_W6_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd116);
        1: z <= $signed (16'd0);
        2: z <= $signed (16'd121);
        3: z <= $signed (-16'd2);
        4: z <= $signed (16'd35);
        5: z <= $signed (16'd100);
        6: z <= $signed (16'd27);
        7: z <= $signed (16'd127);
        8: z <= $signed (16'd67);
        9: z <= $signed (16'd90);
        10: z <= $signed (16'd74);
        11: z <= $signed (-16'd32);
        12: z <= $signed (16'd69);
        13: z <= $signed (-16'd21);
        14: z <= $signed (16'd59);
        15: z <= $signed (16'd78);
      endcase
   end
endmodule

module layer_16_8_8_16_W7_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd62);
        1: z <= $signed (-16'd98);
        2: z <= $signed (-16'd110);
        3: z <= $signed (-16'd62);
        4: z <= $signed (16'd22);
        5: z <= $signed (16'd42);
        6: z <= $signed (16'd12);
        7: z <= $signed (-16'd103);
        8: z <= $signed (-16'd97);
        9: z <= $signed (-16'd40);
        10: z <= $signed (-16'd19);
        11: z <= $signed (-16'd3);
        12: z <= $signed (16'd2);
        13: z <= $signed (-16'd91);
        14: z <= $signed (16'd82);
        15: z <= $signed (-16'd15);
      endcase
   end
endmodule

module layer_16_8_8_16_W8_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd119);
        1: z <= $signed (16'd70);
        2: z <= $signed (16'd101);
        3: z <= $signed (-16'd32);
        4: z <= $signed (-16'd54);
        5: z <= $signed (-16'd77);
        6: z <= $signed (-16'd104);
        7: z <= $signed (-16'd82);
        8: z <= $signed (16'd67);
        9: z <= $signed (16'd8);
        10: z <= $signed (16'd43);
        11: z <= $signed (16'd19);
        12: z <= $signed (-16'd43);
        13: z <= $signed (-16'd109);
        14: z <= $signed (16'd103);
        15: z <= $signed (-16'd54);
      endcase
   end
endmodule

module layer_16_8_8_16_B1_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd61);
        1: z <= $signed (-16'd22);
      endcase
   end
endmodule

module layer_16_8_8_16_B2_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd76);
        1: z <= $signed (16'd90);
      endcase
   end
endmodule

module layer_16_8_8_16_B3_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd124);
        1: z <= $signed (-16'd29);
      endcase
   end
endmodule

module layer_16_8_8_16_B4_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd103);
        1: z <= $signed (-16'd81);
      endcase
   end
endmodule

module layer_16_8_8_16_B5_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd64);
        1: z <= $signed (-16'd59);
      endcase
   end
endmodule

module layer_16_8_8_16_B6_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (16'd15);
        1: z <= $signed (-16'd98);
      endcase
   end
endmodule

module layer_16_8_8_16_B7_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd113);
        1: z <= $signed (16'd125);
      endcase
   end
endmodule

module layer_16_8_8_16_B8_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= $signed (-16'd125);
        1: z <= $signed (-16'd28);
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

