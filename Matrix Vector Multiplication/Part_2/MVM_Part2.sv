//-------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------//
// Project Title    : Matrix Vector Multiplication  || Part: 02                                  ESE 507 [Fall 2017] //
//                                                                                                                   //
// Project Member 1 : Aswin Natesh Venkatesh    [SBU ID: 111582677]                                                  //
// Project Member 2 : Gosakan Srinivasan        [SBU ID: 111579886]                                                  // 
//                                                                                                                   //
// Submission Date  : November 6, 2017                                                                               //
//-------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------//
// Code Sections                                                                                                     //
//    [1] TOP MODULE        :   mvma3_part2                                                                          //
//    [2] CONTROL MODULE    :   control_module                                                                       //
//    [3] DATAPATH MODULE   :   datapath_module                                                                      //
//    [4] MAC MODULE        :   mac                                                                                  //
//    [5] MEMORY MODULE     :   memory                                                                               //
//                                                                                                                   //
// Testbench                                                                                                         //
//    [1] Random Value Testbench      : Line No:  260                                                                //
//    [2] 3 Input Testbench - Given   : Line No:  353     [commented]                                                //
//    [3] 1 Input Testbench - Given   : Line No:  488     [commented]                                                //
//                                                                                                                   //
// Instructions for Running  :  Readme.txt                                                                           //
// Variable Legend           :  Legend.txt                                                                           //
//                                                                                                                   //
//-------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------//

//==================================================MAIN MODULE=======================================================

module mvma3_part2(clk, reset, s_valid, m_ready, data_in, m_valid, s_ready, data_out, overflow);  

  input clk, reset, s_valid, m_ready;               // Control Signals
  input signed [7:0] data_in;
  output logic signed [15:0] data_out; 
  output m_valid, s_ready, overflow;
  reg [3:0] addr_m;                                // Memory Address pointer of Input matrix M
  reg [1:0] addr_x, addr_y, addr_o, addr_b;        // Memory Adress pointer of Input Vector and Output Values
  logic wr_en_m, wr_en_x, wr_en_y, wr_en_b, wr_en_o, clear_acc;  

  control_module c(clk, reset, s_valid,m_ready,addr_y, addr_o, wr_en_y, wr_en_o, addr_x, wr_en_x, addr_m, addr_b, wr_en_m, wr_en_b, m_valid,s_ready, clear_acc);
  datapath_module d(clk, m_ready, clear_acc, data_in, addr_x, addr_m, addr_b, addr_y, addr_o, wr_en_x, wr_en_o, wr_en_m, wr_en_b, wr_en_y, data_out, overflow); 
  
endmodule

//==================================================CONTROL MODULE==================================================

 module control_module(clk, reset, s_valid,m_ready,addr_y, addr_o, wr_en_y, wr_en_o, addr_x, wr_en_x, addr_m, addr_b, wr_en_m, wr_en_b, m_valid,s_ready, clear_acc);

  input clk, reset, s_valid, m_ready;
  output logic [3:0] addr_m;
  output logic [1:0] addr_x, addr_y, addr_o, addr_b;
  output logic wr_en_m, wr_en_x, wr_en_y, wr_en_b, wr_en_o, clear_acc, m_valid, s_ready;
  logic write_complete, hold_s2,mac_complete ;
  logic [2:0] state, next_state;                                              // FSM Transition Variables
  logic [2:0] row,switch, column, data_displayed;
  logic [1:0]  hold_s3;
 
  always_ff @(posedge clk) begin
    if(reset==1)                                                              // Reset Signal & Initial State Assignment
      state<=0;                                                               
    else begin
      state<=next_state;                                                      // State Transition Assignment 
    end
  end
 
  always_comb begin 
      if(state == 0) begin                 // State 0: Wait for Valid Data and Valid Data Signal
        if(s_valid == 1)
          next_state = 1;                  
        else 
          next_state = 0;
        end

      else if(state == 1) begin             // State 1: Fetch Input Data (Matrix M and Vector X)
        if(write_complete == 1)
          next_state = 2;
        else 
          next_state = 1;
      end
       
      else if(state == 2) begin             // State 2: Matrix Vector Multiplication and Accumulation
        if(mac_complete ==1)  
          next_state = 3;   
        else 
          next_state = 2; 
      end
      
      else if(state == 3) begin             // State 3: Wait for Master Ready Signal and Display Output  
        if(data_displayed == 1) 
          next_state = 0; 
        else 
          next_state = 3;
      end
      
      else next_state = 0;                 // Wait for Instruction from Testbench
    end

  assign s_ready =  ((state==1)&(write_complete==0));                          // Assert when system is ready for data input 
  assign data_displayed = ((state ==3)&(column ==4));                          // Assert once output is displayed
  assign write_complete = ((state ==1) & (addr_x ==3));                        // Assert once input is taken in
  assign clear_acc = (((column==1)|(mac_complete==1))&(state==2));             // Clear Accumulator Flag
 
  assign wr_en_m=((state==1) & ((addr_m >=0) & (addr_m <9)));                  // Asserting Matrix Memory Write Enable 
  assign wr_en_b=((state==1) & ((addr_m == 9) & (addr_b<3)));                  // Asserting Vector B Memory Write Enable
  assign wr_en_x=((state ==1) & ((addr_b==3) &(addr_x < 3)));                  // Asserting Vector X Memory Write Enable 
  assign wr_en_y=((state==2)&(hold_s2==1)&(column==0)&(mac_complete==0));      // Asserting Output Memory Write Enable 
  assign wr_en_o=((state==2)&(hold_s2==1)&(column==0)&(mac_complete==0));      // Asserting Overflow Memory Write Enable

  always_ff @(posedge clk) begin
      if(state==0) begin                                      // State 0 Operations: Initialize Address Pointers  
        addr_m <= 0; addr_x <= 0; addr_b <=0; addr_y<=0;              
        row <=0; column <=0; hold_s3 <=0; 
        hold_s2 <=0; mac_complete <=0;  
      end  

      else if ((state == 1) & (s_valid == 1)) begin           // State 1 Operations: Shift Matrix and Vector Pointers
            if(addr_m < 9) 
              addr_m <= addr_m + 1; 
            else if((addr_m == 9) & (addr_b < 3))
              addr_b <= addr_b + 1;
            else if ((addr_b == 3) &(addr_x < 3))
              addr_x <= addr_x + 1;
            else 
              begin
                addr_m <=0;                                   
                addr_x <=0;
                addr_b <=0;
            end 
      end
      
      else if (state == 2) begin                              // State 2 Operations: Matrix-Vector Multiply and Accumulate 
        if(hold_s2 == 0)begin
            addr_b <= row; 
            hold_s2 <= 1;
          end
        else if(row <=2)begin
          if((column<3) & (hold_s2==1)) begin
            addr_m<=((3*row)+column);
            addr_x<=column;
            column<=column+1;
          end
          else begin
            column<=0;
            row <= row+1;
            addr_y <= row; 
            addr_o <= row;
            hold_s2 <=0;
          end
        end 
        else begin 
          mac_complete <=1;                                 // Assert when State 2 is complete
        end
    end

    else if (state == 3) begin                              // State 3 Operations: Send Multiplied & Accumulated Values
      if (hold_s3 == 0) begin
          addr_y <= column;
          addr_o <= column;
          column<=column+1;
          hold_s3 <=1;
        end 
      else if(m_ready ==1 & hold_s3 ==1 & column !=4) begin
          m_valid <=1;
          hold_s3 <=2;
        end
      else if(m_ready ==0 & hold_s3 ==1) begin
          m_valid <=0; 
          hold_s3<=1;
        end 
      else if(m_ready ==0 & hold_s3 ==2) begin
        // $display("Waiting for m-ready to become 1");
        end 
      else  begin
        m_valid<=0;
        hold_s3<=0;       
        end
    end       
 end
endmodule

//==================================================DATAPATH MODULE==================================================
          
module datapath_module(clk, m_ready, clear_acc, data_in, addr_x, addr_m, addr_b, addr_y, addr_o, wr_en_x, wr_en_o, wr_en_m, wr_en_b, wr_en_y, data_out, overflow_out);    

 input m_ready, clk, clear_acc, wr_en_m, wr_en_b, wr_en_x, wr_en_y, wr_en_o;     // Input Signals from contol_module
 input [3:0] addr_m;
 input [1:0] addr_x, addr_y, addr_b, addr_o;
 input signed [7:0] data_in;                                          // Input Data 
 output logic signed [15:0] data_out;                                 // Output Data
 output overflow_out;
 logic overflow;
 logic signed [7:0] data_b;
 logic signed [7:0] data_m, data_x;                                   
 logic signed [15:0] data_y;

 memory #(8,9,4) m(clk, data_in, data_m, addr_m, wr_en_m);                   // Matrix Memory Instantiation
 memory #(8,3,2) b(clk, data_in, data_b, addr_b, wr_en_b);                   // Matrix Memory Instantiation
 memory #(8,3,2) x(clk, data_in, data_x, addr_x, wr_en_x);                   // Vector Memory Instantiation
 memory #(16,3,2) y(clk, data_y, data_out, addr_y, wr_en_y);                 // Output Data Memory Instantiation
 memory #(1,3,2) o(clk, overflow, overflow_out, addr_o, wr_en_o);            // Matrix Memory Instantiation
 mac ma(clk, clear_acc, data_b, data_m, data_x, data_y, overflow);           // MAC Module Instantiation

endmodule

//==================================================MAC  MODULE=====================================================

module mac(clk, clear_acc, data_b, data_m, data_x, data_y, overflow);

input clk, clear_acc;
input signed [7:0] data_m, data_x;
input signed [7:0] data_b;
output logic signed [15:0] data_y;
output logic overflow;
logic signed [15:0] product, sum;
logic overflow_internal;

  always_ff @(posedge clk) begin
    if(clear_acc == 1) begin                // Clearing Accumulated Value 
      data_y <=data_b;                      // Initialising B Vetor for Accumulation
      overflow <=0;
    end
    else begin
      data_y<=sum;
      overflow <= overflow_internal;
    end
  end

  always_comb begin
      product = data_m * data_x;           // Multiplication Operation   
      sum = product + data_y;              // Accumulation Operation
    end

  always_comb begin                       // Combinational overflow detection
    if (overflow)
      overflow_internal  = 1;

    if ((product[15] == data_y[15]) && (sum[15] != data_y[15]))
      overflow_internal  = 1;
    else
      overflow_internal  = 0;      
  end

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

//========================================== Random Value Testbench ===================================================

module check_timing();

parameter ITERATION=5;        // Number of Iterations
parameter XSIZE =3;           // Enter 3 if (3 x 3 Matrix)
parameter BSIZE=XSIZE *2;     // Ignore - Internal Parameter 

logic clk, s_valid, s_ready, m_valid, m_ready, reset, overflow;
logic signed [7:0] data_in;
logic signed [15:0] data_out;
   
initial clk=0;
always #5 clk = ~clk;   

mvma3_part2 dut (.clk(clk), .reset(reset), .s_valid(s_valid), .m_ready(m_ready), 
       .data_in(data_in), .m_valid(m_valid), .s_ready(s_ready), .data_out(data_out),
       .overflow(overflow));

logic rb, rb2;
 
   always begin
      @(posedge clk);
      #1;
      std::randomize(rb, rb2); // randomize rb
   end

   logic [7:0] invals[0:(ITERATION*((XSIZE*XSIZE)+BSIZE))-1];
   initial $readmemh("C_Input", invals);
   
   logic [15:0] j;

   always @* begin
      if (s_valid == 1)
         data_in = invals[j];
      else
         data_in = 'x;
   end

   always @* begin // random bit (rb1) to determine if we can assert s_valid.
      if ((j>=0) && (j<(ITERATION*((XSIZE*XSIZE)+BSIZE))) && (rb==1'b1)) begin
         s_valid=1;
      end
      else
         s_valid=0;
   end

   always @(posedge clk) begin
      if (s_valid && s_ready)
         j <= #1 j+1;
   end

   logic [15:0] i;   // random bit (rb2) to determine if we can assert m_ready.
   always @* begin
      if ((i>=0) && (i<(ITERATION*XSIZE)) && (rb2==1'b1))
         m_ready = 1;
      else
         m_ready = 0;
   end

   integer filehandle=$fopen("Sv_Output");
   always @(posedge clk) begin
      if (m_ready && m_valid) begin
      if(overflow) begin
           $fdisplay(filehandle, "OVERFLOW");
           $display("%d  |  %d", data_out, overflow); end

      else begin $fdisplay(filehandle, "%d\n%d", data_out, overflow);
      $display("%d  |  %d", data_out, overflow); end
      i=i+1;
      end 
   end

   initial begin
      j=0; i=0;
      m_ready = 0; 
      reset = 0;
      @(posedge clk); #1; reset = 1; 
      @(posedge clk); #1; reset = 0; 

      wait(i==ITERATION*XSIZE); begin // wait until (ITERATION x 3) outputs have come out, then finish.
      $fclose(filehandle);
      $finish; end
   end

   initial begin       // Watchdog Timer
      repeat(1000) begin
         @(posedge clk);
      end
      $display("Warning: Output not produced within 1000 clock cycles; stopping simulation so it doens't run forever");
      $stop;
   end

endmodule

//========================================== 3 Input Testbench ========================================================

// ESE-507 Project 2, Fall 2017

/*
 # SUCCESS:          y[    0] =    186; overflow = 0
 # SUCCESS:          y[    1] =    152; overflow = 0
 # SUCCESS:          y[    2] =   -210; overflow = 0
 # SUCCESS:          y[    3] =   4191; overflow = 0
 # SUCCESS:          y[    4] = -17149; overflow = 1
 # SUCCESS:          y[    5] =    762; overflow = 0
 # SUCCESS:          y[    6] =   -494; overflow = 0
 # SUCCESS:          y[    7] =   1012; overflow = 0
 # SUCCESS:          y[    8] =    808; overflow = 0
 */

/*module check_timing();

   logic clk, s_valid, s_ready, m_valid, m_ready, reset, overflow;
   logic signed [7:0] data_in;
   logic signed [15:0] data_out;
   
   initial clk=0;
   always #5 clk = ~clk;
   

   mvma3_part2 dut (.clk(clk), .reset(reset), .s_valid(s_valid), .m_ready(m_ready), 
       .data_in(data_in), .m_valid(m_valid), .s_ready(s_ready), .data_out(data_out),
       .overflow(overflow));


   //////////////////////////////////////////////////////////////////////////////////////////////////
   // code to feed some test inputs

   // rb and rb2 represent random bits. Each clock cycle, we will randomize the value of these bits.
   // When rb is 0, we will not let our testbench send new data to the DUT.
   // When rb is 1, we can send data.
   logic rb, rb2;
   always begin
      @(posedge clk);
      #1;
      std::randomize(rb, rb2); // randomize rb
   end

   // Put our test data into this array. These are the values we will feed as input into the system.
   logic [7:0] invals[0:44] = '{1, -8, 3, 9, -5, 11, -7, 8, -9, 1, 2, 3, 1, -22, 3, 
        10, 11, 12, 127, 127, 127, 1,  2, 3, 4, 5, 6, 127, 127, 127,
        19, 18, -17,  16,  -15,  14, 13, -12, 11, 7, 8, 9, 19, -22, 27 };

   
   logic signed [15:0] expVals[0:8]  = {187, 154, -207, 4195, -17144, 768, -487, 1020, 817};
   logic  expOverflow[0:8] = {0, 0, 0, 0, 1, 0, 0, 0, 0};
   
   logic [15:0] j;

   // If s_valid is set to 1, we will put data on data_in.
   // If s_valid is 0, we will put an X on the data_in to test that your system does not 
   // process the invalid input.
   always @* begin
      if (s_valid == 1)
         data_in = invals[j];
      else
         data_in = 'x;
   end

   // If our random bit rb is set to 1, and if j is within the range of our test vector (invals),
   // we will set s_valid to 1.
   always @* begin
      if ((j>=0) && (j<45) && (rb==1'b1)) begin
         s_valid=1;
      end
      else
         s_valid=0;
   end

   // If we set s_valid and s_ready on this clock edge, we will increment j just after
   // this clock edge.
   always @(posedge clk) begin
      if (s_valid && s_ready)
         j <= #1 j+1;
   end
   ////////////////////////////////////////////////////////////////////////////////////////
   // code to receive the output values

   // we will use another random bit (rb2) to determine if we can assert m_ready.
   logic [15:0] i;
   always @* begin
      if ((i>=0) && (i<45) && (rb2==1'b1))
         m_ready = 1;
      else
         m_ready = 0;
   end

   always @(posedge clk) begin
      if (m_ready &&m_valid) begin
   if ((data_out == expVals[i]) && (overflow == expOverflow[i]))
           $display("SUCCESS:          y[%d] = %d; overflow = %b" , i, data_out, overflow);
   else
     $display("ERROR:   Expected y[%d] = %d; overflow = %b.   Instead your system produced: y[%d] = %d; overflow = %b" , i, expVals[i], expOverflow[i], i, data_out, overflow);
         i=i+1; 
      end 
   end

   ////////////////////////////////////////////////////////////////////////////////

   initial begin
      j=0; i=0;

      // Before first clock edge, initialize
      m_ready = 0; 
      reset = 0;
   
      // reset
      @(posedge clk); #1; reset = 1; 
      @(posedge clk); #1; reset = 0; 

      // wait until 3 outputs have come out, then finish.
      wait(i==9);
      $finish;
   end


   // This is just here to keep the testbench from running forever in case of error.
   // In other words, if your system never produces three outputs, this code will stop 
   // the simulation after 1000 clock cycles.
   initial begin
      repeat(1000) begin
         @(posedge clk);
      end
      $display("Warning: Output not produced within 1000 clock cycles; stopping simulation so it doens't run forever");
      $stop;
   end

endmodule*/

//========================================== 1 Input Testbench ========================================================

/*module check_timing();

   logic clk, s_valid, s_ready, m_valid, m_ready, reset, overflow;
   logic signed [7:0] data_in;
   logic signed [15:0] data_out;
   
   initial clk=0;
   always #5 clk = ~clk;
   

   mvma3_part2 dut (.clk(clk), .reset(reset), .s_valid(s_valid), .m_ready(m_ready), 
       .data_in(data_in), .m_valid(m_valid), .s_ready(s_ready), .data_out(data_out),
       .overflow(overflow));


   //////////////////////////////////////////////////////////////////////////////////////////////////
   // code to feed some test inputs

   // rb and rb2 represent random bits. Each clock cycle, we will randomize the value of these bits.
   // When rb is 0, we will not let our testbench send new data to the DUT.
   // When rb is 1, we can send data.
   logic rb, rb2;
   always begin
      @(posedge clk);
      #1;
      std::randomize(rb, rb2); // randomize rb
   end

   // Put our test data into this array. These are the values we will feed as input into the system.
   logic [7:0] invals[0:14] = '{1, 2, 3, 4, 5, 6, 7, 8, 9, 2, 3, 4, 1, 2, 3};
   logic [15:0] j;

   // If s_valid is set to 1, we will put data on data_in.
   // If s_valid is 0, we will put an X on the data_in to test that your system does not 
   // process the invalid input.
   always @* begin
      if (s_valid == 1) begin
         data_in = invals[j];
      end else
         data_in = 'x;
   end

   // If our random bit rb is set to 1, and if j is within the range of our test vector (invals),
   // we will set s_valid to 1.
   always @* begin
      if ((j>=0) && (j<15) && (rb==1'b1)) begin
         s_valid=1;

      end
      else
         s_valid=0;
   end

   // If we set s_valid and s_ready on this clock edge, we will increment j just after
   // this clock edge.
   always @(posedge clk) begin
      if (s_valid && s_ready)begin
         j <= #1 j+1;  
   end
end
   ////////////////////////////////////////////////////////////////////////////////////////
   // code to receive the output values

   // we will use another random bit (rb2) to determine if we can assert m_ready.
   logic [15:0] i;
   always @* begin
      if ((i>=0) && (i<3) && (rb2==1'b1))
         m_ready = 1;
      else
         m_ready = 0;
   end

   always @(posedge clk) begin
      if (m_ready && m_valid) begin
         $display("Overflow= %d \t y[%d] = %d", overflow, i, data_out); 
         i=i+1; 
      end 
   end

   ////////////////////////////////////////////////////////////////////////////////

   initial begin
      j=0; i=0;
      $display("Small example: correct output is 16, 35, 54");

      // Before first clock edge, initialize
      m_ready = 0; 
      reset = 0;
   
      // reset
      @(posedge clk); #1; reset = 1; 
      @(posedge clk); #1; reset = 0; 

      // wait until 3 outputs have come out, then finish.
      wait(i==3);
      $finish;
   end


   // This is just here to keep the testbench from running forever in case of error.
   // In other words, if your system never produces three outputs, this code will stop 
   // the simulation after 1000 clock cycles.
   initial begin
      repeat(1000) begin
         @(posedge clk);
      end
      $display("Warning: Output not produced within 1000 clock cycles; stopping simulation so it doens't run forever");
      $stop;
   end

endmodule*/
