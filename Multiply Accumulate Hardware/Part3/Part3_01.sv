//********************************************Main Code***************************************************//

module part3_mac(clk, reset, a, b, valid_in, f, overflow, valid_out);         

   input clk, reset, valid_in;                // Inputs (1 Bits)               
   input signed [7:0] a, b;                   // Inputs A & B (8 Bits)
   
   output logic signed [15:0] f;              // Output F (16 Bits)
   output logic signed valid_out=0, overflow=0;

   logic signed [7:0] areg, breg;
   logic signed [15:0] product, productreg, sum;
   logic voflag=0, voflag1=0, offlag=0;
   logic  signed [15:0] sumprev;              // Added on Saturday
 
   always_ff @(posedge clk) begin            // Control Unit 1 | Reg A & Reg B
      
      if(reset == 1) begin                   // Check Reset Signal
         areg <= 0;                          // If Yes -> Pass 0 for areg & breg 
         breg <= 0;
         voflag <= 0;
      end    

      else if (valid_in == 1) begin
       areg <= a;                          // Else -> Pass Inputs for areg & breg
         breg <= b;
         voflag <= 1;
     end

      else begin 
      areg <= 0;                           // If Yes -> Pass 0 for areg & breg 
      breg <= 0;    
      voflag <= 0; 
     end
   end   
   
   always_ff @(posedge clk) begin            // Control Unit 2 | Reg C 

      if(reset == 1)                         // Check Reset Signal
         f <= 0;
      else  begin 
          f <= sum;      
          voflag1 <= voflag;
          offlag <= overflow;
          sumprev <= sum;
    end
   end

      always_ff @(posedge clk) begin            // Control Unit 3 | Reg D
         if(reset == 1)                         // Check Reset Signal
            product <= 0;
         else  begin
         product <= productreg;
         valid_out <= voflag1;
      end
      end 

   always_comb begin
      productreg = areg * breg;              // Multiplication Operation   
      sum = product + f;                     // Accumulation Operation
      if(areg[7] == breg[7] && f[15] != breg[7] || f[15] != sumprev[15]) overflow <= 1;
      else if(reset != 1 && offlag == 1) overflow <= 1;
      else overflow <= 0;
   end

endmodule 


//********************************************Test Bench 1 ***************************************************//

module tb_part3_mac();

   logic clk, reset,valid_in, valid_out;
   //logic clk, reset;
   logic signed [7:0] a, b;
   logic signed [15:0] f;

   //part2_mac dut(clk, reset, a, b, f);
  part3_mac dut(clk, reset, a, b, valid_in, f, overflow, valid_out);

   initial clk = 0;
   always #5 clk = ~clk;

   initial begin

      // Before first clock edge, initialize
      
      reset = 1;
      {a, b} = {8'b0,8'b0};
       valid_in = 0;
      
      //valid_out = ;

      @(posedge clk);
      #1; // After 1 posedge
      reset = 0; a = 1; b = 1; valid_in = 0;
      @(posedge clk);
      #1; // After 2 posedges
      a = 2; b = 2; valid_in = 1;
      @(posedge clk);
      #1; // After 3 posedges
      a = 3; b = 3; valid_in = 1;
      @(posedge clk);
      #1; // After 4 posedges
      a = 4; b = 4; valid_in = 0;
      @(posedge clk);
      #1; // After 5 posedges
      a = 5; b = 5; valid_in = 0;
      @(posedge clk);
      #1; // After 6 posedges
      a = 6; b = 6; valid_in = 1;
   end // initial begin

 initial begin
      @(posedge clk);
      #1; // After 1 posedge
      $display("Valid_out = %b | Overflow = %b | Output = %d", valid_out, overflow, f);
      @(posedge clk);

      #1; // After 2 posedges
      $display("Valid_out = %b | Overflow = %b | Output = %d", valid_out, overflow, f);
      @(posedge clk);

      #1; // After 3 posedges
      $display("Valid_out = %b | Overflow = %b | Output = %d", valid_out, overflow, f);
      
      @(posedge clk);
      #1; // After 4 posedges
      $display("Valid_out = %b | Overflow = %b | Output = %d", valid_out, overflow, f);
      
      @(posedge clk);
      #1; // After 5 posedges
      $display("Valid_out = %b | Overflow = %b | Output = %d", valid_out, overflow, f);
      
      @(posedge clk);
      #1; // After 6 posedges
      $display("Valid_out = %b | Overflow = %b | Output = %d", valid_out, overflow, f);
      
      @(posedge clk);
      #1; // After 7 posedges
      $display("Valid_out = %b | Overflow = %b | Output = %d", valid_out, overflow, f);
      
      @(posedge clk);
      #1; // After 8 posedges
      $display("Valid_out = %b | Overflow = %b | Output = %d", valid_out, overflow, f);

      @(posedge clk);
      #1; // After 9 posedges
      $display("Valid_out = %b | Overflow = %b | Output = %d", valid_out, overflow, f);
      
      #20;
      $finish;
   end

 endmodule // tb_part2_mac



