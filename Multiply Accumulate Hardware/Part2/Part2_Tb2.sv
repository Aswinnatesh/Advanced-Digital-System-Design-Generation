//********************************************Main Code***************************************************//

module part2_mac(clk, reset, a, b, valid_in, f, overflow, valid_out);         

   input clk, reset, valid_in;                // Inputs (1 Bits)               
   input signed [7:0] a, b;                   // Inputs A & B (8 Bits)
   
   output logic signed [15:0] f;              // Output F (16 Bits)
   output logic signed valid_out=0, overflow=0;

   logic signed [7:0] areg, breg;
   logic signed [15:0] product, sum;
   logic voflag=0, offlag=0;
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
          valid_out <= voflag;
          offlag <= overflow;
          sumprev <= sum;
    end
   end

   always_comb begin
      product = areg * breg;                 // Multiplication Operation   
      sum = product + f;                     // Accumulation Operation
      if(areg[7] == breg[7] && f[15] != breg[7] || f[15] != sumprev[15]) overflow <= 1;
      else if(reset != 1 && offlag == 1) overflow <= 1;
      else overflow <= 0;
   end

endmodule 

//********************************************Test Bench 2 ***************************************************//

module tb_part2_mac();                                                           // --- 

   logic signed [7:0] a, b; 
   logic signed [15:0] f;
   logic clk, reset,valid_in, valid_out; 
   initial clk = 0;
   always #5 clk = ~clk;

   part2_mac dut(clk, reset, a, b, valid_in, f, overflow, valid_out);
  // part2_mac dut(clk, reset, a, b, f);
   logic [7:0] testData[2400:0];
   initial $readmemh("inputData", testData);
   // initial $monitor($time,, "clk=%b, a=%d, b=%d, f=%d, valid_in=%d", clk, a, b, f, valid_in);  

   integer i;
   initial begin
      reset = 1;
      {a, b} = {8'b0,8'b0};
      valid_in = 0;

      @(posedge clk)
      reset = 0;
      
      for(i=0; i<800;i=i+1) begin
      @(posedge clk)
      #1;
      valid_in=testData[3*i][7:0]; 
      a=testData[3*i+1][7:0];
      b=testData[3*i+2][7:0];
      end

      @(posedge clk);
      @(posedge clk);
      @(posedge clk);

      $fclose(filehandle);
      $finish;
   end
   
   integer filehandle=$fopen("outValues");
   always @(posedge clk) 
       $fdisplay(filehandle, "%d \n%d ", valid_out, f);
       // $fdisplay(filehandle, "%d", f);
endmodule



