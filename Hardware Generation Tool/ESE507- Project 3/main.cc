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

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <cstring>
#include <assert.h>
#include <math.h>
using namespace std;

void printUsage();
void genLayer(int M, int N, int P, int bits, vector<int>& constvector, string modName, ofstream &os);
void genAllLayers(int N, int M1, int M2, int M3, int mult_budget, int bits, vector<int>& constVector, string modName, ofstream &os);
void readConstants(ifstream &constStream, vector<int>& constvector);
void genROM(vector<int>& constVector, int bits, string modName, ofstream &os,  int addrBits);
void Optimal_P(int N, int M1, int M2, int M3, int mult_budget);
void memmac(int bits, ofstream &os);
void nameblock(ofstream &os);
int iter=1, P1, P2, P3;  int addrBits;

int main(int argc, char* argv[]) {

   // If the user runs the program without enough parameters, print a helpful message
   // and quit.
   if (argc < 7) {
      printUsage();
      return 1;
   }

   int mode = atoi(argv[1]);

   ifstream const_file;
   ofstream os;
   vector<int> constVector;

   //----------------------------------------------------------------------
   // Look here for Part 1 and 2
   if ((mode == 1) && (argc == 7)) {
      // Mode 1: Generate one layer with given dimensions and one testbench

      // --------------- read parameters, etc. ---------------
      int M = atoi(argv[2]);
      int N = atoi(argv[3]);
      int P = atoi(argv[4]);
      int bits = atoi(argv[5]);
      const_file.open(argv[6]);
      if (const_file.is_open() != true) {
         cout << "ERROR reading constant file " << argv[6] << endl;
         return 1;
      }

      // Read the constants out of the provided file and place them in the constVector vector
      readConstants(const_file, constVector);

      string out_file = "layer_" + to_string(M) + "_" + to_string(N) + "_" + to_string(P) + "_" + to_string(bits) + ".sv";

      os.open(out_file);
      if (os.is_open() != true) {
         cout << "ERROR opening " << out_file << " for write." << endl;
         return 1;
      }
      // -------------------------------------------------------------

      // call the genLayer function you will write to generate this layer
      string modName = "layer_" + to_string(M) + "_" + to_string(N) + "_" + to_string(P) + "_" + to_string(bits);
      nameblock(os);
      genLayer(M, N, P, bits, constVector, modName, os); 
      memmac(bits,os);

   }
   //--------------------------------------------------------------------


   // ----------------------------------------------------------------
   // Look here for Part 3
   else if ((mode == 2) && (argc == 9)) {
      // Mode 2: Generate three layer with given dimensions and interconnect them

      // --------------- read parameters, etc. ---------------
      int N  = atoi(argv[2]);
      int M1 = atoi(argv[3]);
      int M2 = atoi(argv[4]);
      int M3 = atoi(argv[5]);
      int mult_budget = atoi(argv[6]);
      int bits = atoi(argv[7]);
      const_file.open(argv[8]);
      if (const_file.is_open() != true) {
         cout << "ERROR reading constant file " << argv[8] << endl;
         return 1;
      }
      readConstants(const_file, constVector);

      string out_file = "network_" + to_string(N) + "_" + to_string(M1) + "_" + to_string(M2) + "_" + to_string(M3) + "_" + to_string(mult_budget) + "_" + to_string(bits) + ".sv";


      os.open(out_file);
      if (os.is_open() != true) {
         cout << "ERROR opening " << out_file << " for write." << endl;
         return 1;
      }
      // -------------------------------------------------------------

      string mod_name = "network_" + to_string(N) + "_" + to_string(M1) + "_" + to_string(M2) + "_" + to_string(M3) + "_" + to_string(mult_budget) + "_" + to_string(bits);

      // call the genAllLayers function
      nameblock(os);
      genAllLayers(N, M1, M2, M3, mult_budget, bits, constVector, mod_name, os);
      memmac(bits,os);

   }
   //-------------------------------------------------------

   else {
      printUsage();
      return 1;
   }

   // close the output stream
   os.close();

}

// Read values from the constant file into the vector
void readConstants(ifstream &constStream, vector<int>& constvector) {
   string constLineString;
   while(getline(constStream, constLineString)) {
      int val = atoi(constLineString.c_str());
      constvector.push_back(val);
   }
}

// Generate a ROM based on values constVector.
// Values should each be "bits" number of bits.
void genROM(vector<int>& constVector, int bits, string modName, ofstream &os, int addrBits) {

      int numWords = constVector.size();

      os << "module " << modName << "(clk, addr, z);" << endl;
      os << "   input clk;" << endl;
      os << "   input [" << addrBits << ":0] addr;" << endl;
      os << "   output logic signed [" << bits-1 << ":0] z;" << endl;
      os << "   always_ff @(posedge clk) begin" << endl;
      os << "      case(addr)" << endl;
      int i=0;
      for (vector<int>::iterator it = constVector.begin(); it < constVector.end(); it++, i++) {
         if (*it < 0)
            os << "        " << i << ": z <= $signed (-" << bits << "'d" << abs(*it) << ");" << endl;
         else
            os << "        " << i << ": z <= $signed ("  << bits << "'d" << *it      << ");" << endl;

      }
      os << "      endcase" << endl << "   end" << endl << "endmodule" << endl << endl;
}

// Parts 1 and 2
// Here is where you add your code to produce a neural network layer.
void genLayer(int M, int N, int P, int bits, vector<int>& constVector, string modName, ofstream &os) {

   // Make your module name: layer_M_N_P_bits, where these parameters are replaced with the
   // actual numbers
   
int LOGM= ceil(log2(M));
int LOGN= ceil(log2(N));
int LOGB, LOGW;

if(P<2)
{  LOGB= ceil(log2(M)-1);
   LOGW= ceil(log2(M*N));
}
else 
{
   LOGB= ceil((log2(M/P)-1));
   LOGW= ceil(log2((M*N)/P));
}
if(LOGN<=2) { LOGN = 2;}
if(LOGN<2) { LOGM = 2;}
if(LOGB<1) {LOGB = 1;}




os <<"//=================================================MAIN MODULE====================================================="<< endl;
os <<"module "<< modName <<"(clk, reset, s_valid, m_ready, data_in, m_valid, s_ready, data_out);  " << endl;
os <<"" << endl;
os <<"parameter N="<< N <<"; parameter M="<< M <<"; parameter T="<< bits <<"; parameter P="<< P <<";"<< endl;
os <<"parameter LOGM="<< LOGM <<"; parameter LOGW="<< LOGW <<"; parameter LOGN="<< LOGN <<"; parameter LOGB="<< LOGB <<";"<< endl;
os <<"" << endl;
os <<" input clk, reset, s_valid, m_ready;               " << "        // Control Signals"<< endl;
os <<"  input signed [T-1:0] data_in;" << endl;
os <<"  output logic signed [T-1:0] data_out; " << endl;
os <<"  output m_valid, s_ready;" << endl;
os <<"  reg [LOGW:0] addr_m,counter;                                " << "        // Memory Address pointer of Input matrix M"<< endl;
os <<"  reg [LOGN:0] addr_x;" << endl;
os <<"  reg [LOGM:0] addr_y;         " << "        // Memory Adress pointer of Input Vector and Output Values"<< endl;
os <<"  reg [LOGB:0] addr_b;" << endl;

os <<"  logic wr_en_x, wr_en_y, clear_acc;  " << endl;
os <<"  control_module"<<iter<<" c(clk, reset, s_valid,m_ready,addr_y, wr_en_y, addr_x, wr_en_x,addr_m,addr_b, m_valid,s_ready, clear_acc, counter);" << endl;
os <<"  datapath_module"<<iter<<" d(clk, reset, counter, m_ready, clear_acc, data_in, addr_x, addr_m,addr_b, addr_y, wr_en_x, wr_en_y, data_out); " << endl;
os <<"" << endl;
os <<"endmodule" << endl;
os <<"" << endl;
os <<"//==================================================CONTROL MODULE=================================================="<< endl;
os <<"" << endl;
os <<" module control_module"<<iter<<" (clk, reset, s_valid,m_ready,addr_y, wr_en_y, addr_x, wr_en_x,addr_m,addr_b, m_valid,s_ready, clear_acc, counter);" << endl;
os <<"" << endl;
os <<"parameter N="<< N <<"; parameter M="<< M <<"; parameter T="<< bits <<"; parameter P="<< P <<";"<< endl;
os <<"parameter LOGM="<< LOGM <<"; parameter LOGW="<< LOGW <<"; parameter LOGN="<< LOGN <<"; parameter LOGB="<< LOGB <<";"<< endl;
os <<"" << endl;
os <<"  input clk, reset, s_valid, m_ready;" << endl;
os <<"  output logic [LOGW:0] addr_m,counter;" << endl;
os <<"  output logic [LOGN:0] addr_x;" << endl;
os <<"  output logic [LOGM:0] addr_y;         " << endl;
os <<"  output logic [LOGB:0] addr_b;" << endl;
os <<"  output logic wr_en_x, wr_en_y, clear_acc, m_valid, s_ready;" << endl;
os <<"  logic write_complete, mac_complete, hold_s2;" << endl;
os <<"  logic [2:0] state, next_state;                                              " << "        // FSM Transition Variables"<< endl;
os <<"  logic [2:0] data_displayed;" << endl;
os <<"  logic [1:0] hold_s3;" << endl;
os <<"  logic [LOGW:0] row, hold_disp;" << endl;
os <<"  logic [LOGW:0] column;" << endl;
os <<"  logic flag;" << endl;
os <<"" << endl;
os <<"  always_ff @(posedge clk) begin" << endl;
os <<"    if(reset==1)                                                              " << "        // Reset Signal & Initial State Assignment"<< endl;
os <<"      state<=0;                                                               " << endl;
os <<"    else begin" << endl;
os <<"      state<=next_state;                                                      " << "        // State Transition Assignment "<< endl;
os <<"    end" << endl;
os <<"  end" << endl;
os <<"" << endl;
os <<"  always_comb begin " << endl;
os <<"      if(state == 0) begin                  " << "        // State 0: Wait for Valid Data and Valid Data Signal"<< endl;
os <<"        if(s_valid == 1)" << endl;
os <<"          next_state = 1;                  " << endl;
os <<"        else " << endl;
os <<"          next_state = 0;" << endl;
os <<"        end" << endl;
os <<"" << endl;
os <<"      else if(state == 1)begin              " << "        // State 1: Fetch Input Data (Matrix M and Vector X) "<< endl;
os <<"        if(write_complete == 1) " << endl;
os <<"          next_state = 2;  " << endl;
os <<"        else " << endl;
os <<"          next_state = 1;" << endl;
os <<"      end" << endl;
os <<"" << endl;
os <<"      else if(state == 2)begin              " << "        // State 2: Matrix Vector Multiplication and Accumulation"<< endl;
os <<"        if(mac_complete ==1) " << endl;
os <<"          next_state = 3;    " << endl;
os <<"        else " << endl;
os <<"          next_state = 2; " << endl;
os <<"      end" << endl;
os <<"" << endl;
os <<"      else if(state == 3) begin             " << "        // State 3: Wait for Master Ready Signal and Display Output  "<< endl;
os <<"        if(data_displayed == 1)" << endl;
os <<"          next_state = 0; " << endl;
os <<"        else " << endl;
os <<"          next_state = 3;" << endl;
os <<"       end" << endl;
os <<"" << endl;
os <<"      else next_state = 0;                  " << "        // Wait for Instruction from Testbench"<< endl;
os <<"    end" << endl;
os <<"" << endl;
os <<"  assign s_ready =  ((state==1) & (write_complete ==0));                      " << "        // Assert when system is ready for data input"<< endl;
os <<"  assign data_displayed = ((state ==3)&(column ==M/P));                         " << "        // Assert once output is displayed"<< endl;
os <<"  assign write_complete = ((state ==1) & (addr_x==N));                        " << "        // Assert once input is taken in"<< endl;
os <<"  assign clear_acc = (((column==1)|(mac_complete==1))&(state==2));            " << "        // Clear Accumulator Flag"<< endl;
os <<"" << endl;
os <<"" << endl;
os <<"  assign wr_en_x =((state ==1) & ((addr_x>=0) &(addr_x < N)));                 " << "        // Asserting Vector X Memory Write Enable"<< endl;
os <<"  assign wr_en_y=clear_acc;" << endl;
os <<"" << endl;
os <<"  always_ff @(posedge clk) begin" << endl;
os <<"      if(state==0) begin                            " << "        // State 0 Operations: Initialize Address Pointers  "<< endl;
os <<"        addr_m <= 0; addr_x <= 0;addr_y <=0;addr_b <=0;               " << endl;
os <<"        row <=0; column <=0; hold_s2 <=0;" << endl;
os <<"        mac_complete <=0; hold_s3 <=0;" << endl;
os <<"        hold_disp <=0; flag <=0; counter <=0;" << endl;
os <<"      end  " << endl;
os <<"" << endl;
os <<"      else if ((state == 1)) begin  " << "        // State 1 Operations: Shift Matrix and Vector Pointers"<< endl;
os <<"          if (s_valid ==1) begin" << endl;
os <<"           if(addr_x < N) " << endl;
os <<"            addr_x <= addr_x + 1; " << endl;
os <<"          else" << endl;
os <<"            addr_x <=0;" << endl;
os <<"        end" << endl;
os <<"      end" << endl;
os <<"      else if (state == 2) begin                    " << "        // State 2 Operations: Matrix-Vector Multiply and Accumulate "<< endl;
os <<"        if(hold_s2==0) begin" << endl;
os <<"          addr_b <= row;" << endl;
os <<"          hold_s2 <=1;" << endl;
os <<"        end" << endl;
os <<"" << endl;
os <<"        else if(row <=(M/P)-1)begin" << endl;
os <<"        if((column<N) &(hold_s2 ==1)) begin" << endl;
os <<"          addr_m<=((N*row)+column);" << endl;
os <<"          addr_x<=column;" << endl;
os <<"          column<=column+1;" << endl;
os <<"      end  " << endl;
os <<"        else begin" << endl;
os <<"          column<=0;" << endl;
os <<"          addr_y <= row; " << endl;
os <<"          row<=row+1;" << endl;
os <<"          hold_s2 <=0;" << endl;
os <<"          end" << endl;
os <<"        end " << endl;
os <<"        else begin " << endl;
os <<"          mac_complete <=1;                           " << "        // Assert when State 2 is complete"<< endl;
os <<"        end" << endl;
os <<"      end" << endl;
os <<"" << endl;
os <<"    else if (state == 3) begin                        " << "        // State 3 Operations: Send Multiplied & Accumulated Values"<< endl;
os <<"       if (hold_s3 == 0) begin"<< endl;
os<<"        if(flag==0) begin" << endl;
os<<"           addr_y<= column;" << endl;
os<<"           hold_disp <=hold_disp +1;" << endl;
os<<"           if(hold_disp==P-1)flag<=1;  " << endl;
os<<"           hold_s3 <=1;" << endl;
os<<"         end" << endl;
os<<"          else if(flag==1) begin" << endl;
os<<"              flag <=0;" << endl;
os<<"              column <= column +1;" << endl;
os<<"              hold_disp<=0;" << endl;
os<<"          end" << endl;
os<<"        end " << endl;
os <<"      else if(hold_s3 ==1 & column!=(M/P)) begin" << endl; // m_ready ==1 & 
os <<"        m_valid <=1;" << endl;
os <<"        hold_s3 <=2;" << endl;
os <<"      end" << endl;
os <<"      else if(m_ready ==0 & hold_s3 ==1)begin" << endl;
os <<"        m_valid <=0; " << endl;
os <<"        hold_s3<=1;" << endl;
os <<"      end " << endl;
os <<"      else if(m_ready ==0 & hold_s3 ==2) begin" << endl;
os <<"           // $display(Waiting for m-ready to become 1)"<< endl;
os <<"      end " << endl;
os <<"      else  begin" << endl;
os <<"        m_valid<=0;" << endl;
os <<"        hold_s3<=0;       " << endl;
os <<"        counter<=counter+1;       " << endl;
os <<"      end" << endl;
os <<"    end       " << endl;
os <<" end" << endl;
os <<"endmodule" << endl;
os <<"" << endl;
os <<"//==================================================DATAPATH MODULE=================================================="<< endl;
os <<"module datapath_module"<<iter<<" (clk, reset, counter, m_ready, clear_acc, data_in, addr_x, addr_m, addr_b, addr_y, wr_en_x, wr_en_y, data_out);    " << endl;
os <<"" << endl;
os <<"parameter N="<< N <<"; parameter M="<< M <<"; parameter T="<< bits <<"; parameter P="<< P <<";"<< endl;
os <<"parameter LOGM="<< LOGM <<"; parameter LOGW="<< LOGW <<"; parameter LOGN="<< LOGN <<"; parameter LOGB="<< LOGB <<";"<< endl;
os <<"" << endl;

os <<" input m_ready, clk, reset, clear_acc, wr_en_x, wr_en_y;     " << "        // Input Signals from contol_module"<< endl;
os <<" input [LOGW:0] addr_m,counter;" << endl;
os <<" input [LOGN:0] addr_x;" << endl;
os <<" input [LOGM:0] addr_y;         " << endl;
os <<" input [LOGB:0] addr_b;" << endl;
os <<" input signed [T-1:0] data_in;                                          " << "        // Input Data "<< endl;
os <<" output logic signed [T-1:0] data_out;                                 " << "        // Output Data"<< endl;
os <<" logic signed [T-1:0] data_x;" << endl;
os <<" logic signed [T-1:0] data_out0;" << endl;

os <<"" << endl;

for(int i=1; i<=P;i++)
  { 
    os<<" logic signed [T-1:0] data_y"<< i <<", data_y"<< i <<"_R;" << endl;
    os<<" logic signed [T-1:0] data_m"<< i <<", data_b"<< i <<";" << endl;  
    os<<" logic signed [T-1:0] data_out"<< i <<";" << endl; 
    os <<"" << endl;
  }
for(int i=1; i<=P;i++)
  {
  os<<" assign data_y"<< i <<"_R= ((wr_en_y==1)&&(data_y"<< i <<"[T-1]==1))? 0:data_y"<< i <<";"<<endl; 
  }

os <<"" << endl;

os <<"always_comb begin" << endl;
os<<" if(counter %P == 0) data_out = data_out1;"<<endl;
for(int i =1; i<P;i++)
{  
  os<<" else if(counter % P == "<< i <<") data_out = data_out"<< i+1 <<";"<<endl;
}
os<<" else data_out = data_out"<< P-1 <<";"<<endl;
os <<"end" << endl;
os <<"" << endl;
os <<" memory #(T,N,LOGN+1) x(clk, data_in, data_x, addr_x, wr_en_x);             " << "        // Vector Memory Instantiation"<< endl;
for(int i=1; i<=P;i++)
{
os <<" memory #(T,M,LOGM+1) y"<< i <<"(clk, data_y"<< i <<"_R, data_out"<< i <<", addr_y, wr_en_y);           // Output Data Memory Instantiation"<< endl;
os <<" "<< modName <<"_B"<< i <<"_rom lb"<< i <<" (clk, addr_b, data_b"<< i <<");" << endl;
os <<" "<< modName <<"_W"<< i <<"_rom lw"<< i <<" (clk, addr_m, data_m"<< i <<");" << endl;
os <<" mac ma"<<iter<< i <<"(clk, reset, clear_acc,data_b"<< i <<", data_m"<< i <<", data_x, data_y"<< i <<");       " << "        // MAC Module Instantiation"<< endl;
os <<"" << endl;
}
os <<"" << endl;
os <<"endmodule" << endl;
os <<"" << endl;
os <<"//==================================================ROM  MODULE====================================================="<< endl;

   // At some point you will want to generate a ROM with values from the pre-stored constant values.
   // Here is code that demonstrates how to do this for the simple case where you want to put all of
   // the matrix values W in one ROM, and all of the bias values B into another ROM. (This is probably)
   // what you will need for P=1, but you will want to change this for P>1.


   // Check there are enough values in the constant file.
   if (M*N+M > constVector.size()) {
      cout << "ERROR: constVector does not contain enough data for the requested design" << endl;
      cout << "The design parameters requested require " << M*N+M << " numbers, but the provided data only have " << constVector.size() << " constants" << endl;
      assert(false);
   }

// Generate a ROM (for W) with constants 0 through M*N-1, with "bits" number of bits
  
//--------------------------------------------------------------------


   if (P==1)
   {
      string romModName = modName + "_W1_rom";
      vector<int> wVector(&constVector[0], &constVector[M*N]);
      genROM(wVector, bits, romModName, os, LOGW);

      romModName = modName + "_B1_rom";
      vector<int> bVector(&constVector[M*N], &constVector[M*N+M]);
      genROM(bVector, bits, romModName, os, LOGB);
   }

   else if (P<=M)
    {
      int Iter1=0, Iter2=1, Cond1=0, Cond2=1;
      for(int k=1; k<=P; k++)
        {
          string romModName = modName + "_W" + to_string(k) + "_rom";
          vector<int> wVector(&constVector[N*Iter1], &constVector[N*Iter2]);
        if(P<M)
          {
            for(int i=0; i<(M/P)-1; i++)
              { 
                Iter1+=P; Iter2+=P; 
                for(int j=(N*Iter1); j<(N*Iter2); j++)
                  {
                    wVector.push_back (constVector[j]); //constVector[M*Iter2])
                  } 
              }
            Iter1=++Cond1; Iter2=++Cond2;
          } 
        else {Iter1++; Iter2++;}
        genROM(wVector, bits, romModName, os, LOGW);
      }
   
       int Cond3=(M*N), Cond4=((M*N)+1);
       int Iter_3=(M*N), Iter_4=(M*N)+1; // To Generalise
       for(int l=1; l<=P; l++)
          {
            string romModName = modName + "_B" + to_string(l) + "_rom";
            vector<int> bVector(&constVector[Iter_3], &constVector[Iter_4]);
              if(P<M)
                {
                  for(int n=0; n<(M/P)-1; n++)
                    {
                      Iter_3+=P; Iter_4+=P;
                      for(int m=Iter_3; m<Iter_4; m++)
                          {
                            bVector.push_back (constVector[m]); //constVector[M*Iter_2])
                          }
                    }
          Iter_3=++Cond3; Iter_4=++Cond4;     
        }
          else {Iter_3++; Iter_4++;}
          genROM(bVector, bits, romModName, os, LOGB);
       }

   }
}

// Part 3: Generate a hardware system with three layers interconnected.
// Layer 1: Input length: N, output length: M1
// Layer 2: Input length: M1, output length: M2
// Layer 3: Input length: M2, output length: M3
// mult_budget is the number of multipliers your overall design may use.
// Your goal is to build the fastest design that uses mult_budget or fewer multipliers
// constVector holds all the constants for your system (all three layers, in order)
void genAllLayers(int N, int M1, int M2, int M3, int mult_budget, int bits, vector<int>& constVector, string modName, ofstream &os) {

   // Here you will write code to figure out the best values to use for P1, P2, and P3, given
   // mult_budget. 

   Optimal_P(N, M1, M2, M3, mult_budget);
   // int P1 = P_L1; // replace this with your optimized value
   // int P2 = P_L2; // replace this with your optimized value
   // int P3 = P_L3; // replace this with your optimized value

   // output top-level module
   // set your top-level name to "network_top"

os << "module "<<modName<<"(clk, reset, s_valid, m_ready, data_in, m_valid, s_ready, data_out);"<<endl;
os<<endl;
os <<"parameter T="<< bits <<";"<<  endl;
os<<endl;
os << "  input clk, reset, s_valid, m_ready; // Control Signals "<<endl;
os << "  input signed [T-1:0] data_in;"<<endl;
os << "  output logic signed [T-1:0] data_out;"<<endl; 
os << "  output m_valid, s_ready;"<<endl;

os << "  logic MS_ready_l2, MS_ready_l3;   // Interconnecting Layer parameters  "<<endl;
os << "  logic MS_valid_l2, MS_valid_l3;"<<endl;
os << "  logic signed [T-1:0] data_IO_l2, data_IO_l3;"<<endl;
os<<endl;

string subModName = "layer1_" + to_string(M1) + "_" + to_string(N) + "_" + to_string(P1) + "_" + to_string(bits);
os << subModName <<" Net1(clk, reset, s_valid, MS_ready_l2, data_in, MS_valid_l2, s_ready, data_IO_l2);   // Layer 1 GPIO"<<endl;
string subModName_layer1 = subModName;

subModName = "layer2_" + to_string(M2) + "_" + to_string(M1) + "_" + to_string(P2) + "_" + to_string(bits);
os << subModName <<" Net2(clk, reset, MS_valid_l2, MS_ready_l3, data_IO_l2, MS_valid_l3, MS_ready_l2, data_IO_l3);   // Layer 2 GPIO"<<endl;
string subModName_layer2 = subModName;

subModName = "layer3_" + to_string(M3) + "_" + to_string(M2) + "_" + to_string(P3) + "_" + to_string(bits);
os << subModName <<" Net3(clk, reset, MS_valid_l3, m_ready, data_IO_l3, m_valid, MS_ready_l3, data_out);   // Layer 3 GPIO"<<endl;
string subModName_layer3 = subModName;
os<<endl;
os << "endmodule" << endl;
   
   // -------------------------------------------------------------------------
   // Split up constVector for the three layers
   // layer 1's W matrix is M1 x N and its B vector has size M1
   int start = 0;
   int stop = M1*N+M1;
   vector<int> constVector1(&constVector[start], &constVector[stop]);

   // layer 2's W matrix is M2 x M1 and its B vector has size M2
   start = stop;
   stop = start+M2*M1+M2;
   vector<int> constVector2(&constVector[start], &constVector[stop]);

   // layer 3's W matrix is M3 x M2 and its B vector has size M3
   start = stop;
   stop = start+M3*M2+M3;
   vector<int> constVector3(&constVector[start], &constVector[stop]);

   if (stop > constVector.size()) {
      cout << "ERROR: constVector does not contain enough data for the requested design" << endl;
      cout << "The design parameters requested require " << stop << " numbers, but the provided data only have " << constVector.size() << " constants" << endl;
      assert(false);
   }
   // --------------------------------------------------------------------------


   // generate the three layer modules
   genLayer(M1, N, P1, bits, constVector1, subModName_layer1, os);
   iter++;
   genLayer(M2, M1, P2, bits, constVector2, subModName_layer2, os);
   iter++;
   genLayer(M3, M2, P3, bits, constVector3, subModName_layer3, os);

   // You will need to add code in the module at the top of this function to stitch together insantiations of these three modules

}

void nameblock (ofstream &os){
os <<"//-------------------------------------------------------------------------------------------------------------------//"<<endl;
os <<"//-------------------------------------------------------------------------------------------------------------------//"<<endl;
os <<"// Project Title    : Hardware Generation Tool                                                   ESE 507 [Fall 2017] //"<<endl;
os <<"//                                                                                                                   //"<<endl;
os <<"// Project Member 1 : Aswin Natesh Venkatesh    [SBU ID: 111582677]                                                  //"<<endl;
os <<"// Project Member 2 : Gosakan Srinivasan        [SBU ID: 111579886]                                                  // "<<endl;
os <<"//                                                                                                                   //"<<endl;
os <<"// Submission Date  : December 08, 2017                                                                              //"<<endl;
os <<"//-------------------------------------------------------------------------------------------------------------------//"<<endl;
os <<"//-------------------------------------------------------------------------------------------------------------------//"<<endl;
os <<"" << endl;
os <<"" << endl;
}

void Optimal_P(int N, int M1, int M2, int M3, int mult_budget) {

     long MIN_K, temp, sol=1e9;

   for(int i=1;i<=M1;i++) {
      if(M1%i!=0) continue;
      
      for(int j=1;j<=M2;j++) {
         if(M2%j!=0) continue;
         MIN_K = min(M3,mult_budget-i-j);
      
            for(int k=1;k<=MIN_K;k++){
               if(M3%k!=0) continue;
               temp = (M1/i)*N + (M2/j)*M1 + (M3/k)*M2;
      
               if(temp<sol) {  
                  sol = temp;
                  P1 = i; P2 = j; P3 = k;
               }       
            }
      }
   }
   cout<<"Optimal Pipeline Values"<<P1<<" "<<P2<<" "<<P3<<endl;
}

void printUsage() {
  cout << "Usage: ./gen MODE ARGS" << endl << endl;

  cout << "   Mode 1: Produce one neural network layer and testbench (Part 1 and Part 2)" << endl;
  cout << "      ./gen 1 M N P bits const_file" << endl;
  cout << "      Example: produce a neural network layer with a 4 by 5 matrix, with parallelism 1" << endl;
  cout << "               and 16 bit words, with constants stored in file const.txt" << endl;
  cout << "                   ./gen 1 4 5 1 16 const.txt" << endl << endl;

  cout << "   Mode 2: Produce a system with three interconnected layers with four testbenches (Part 3)" << endl;
  cout << "      Arguments: N, M1, M2, M3, mult_budget, bits, const_file" << endl;
  cout << "         Layer 1: M1 x N matrix" << endl;
  cout << "         Layer 2: M2 x M1 matrix" << endl;
  cout << "         Layer 3: M3 x M2 matrix" << endl;
  cout << "              e.g.: ./gen 2 4 5 6 7 15 16 const.txt" << endl << endl;
}

void memmac (int bits, ofstream &os) {
os <<"//==================================================MAC  MODULE====================================================="<< endl;
os <<"" << endl;
os <<"module mac(clk, reset, clear_acc,data_b, data_m, data_x, data_y);" << endl;
os <<"" << endl;
os <<"parameter T="<< bits <<";"<<  endl;
os <<"" << endl;
os <<"input clk, reset, clear_acc;" << endl;
os <<"input signed [T-1:0] data_m, data_x;" << endl;
os <<"input signed [T-1:0] data_b;" << endl;
os <<"output logic signed [T-1:0] data_y;" << endl;
os <<"logic signed [T-1:0] product, sum, prod2;" << endl;
os <<"" << endl;
os <<"  always_ff @(posedge clk) begin" << endl;
os <<"    if(clear_acc == 1)   begin          " << "        // Clearing Accumulated Value "<< endl;
os <<"      data_y <=data_b;" << endl;
os <<"     end" << endl;
os <<"    else " << endl;
os <<"      data_y<=sum;" << endl;
os <<"  end " << endl;
os <<"" << endl;
os <<"" << endl;
os <<"   always_ff @(posedge clk) begin" << endl;
os <<"      if (reset == 1'b1) begin" << endl;
os <<"      product<=0;" << endl;
os <<"      end        " << endl;
os <<"      else begin" << endl;
os <<"        if(clear_acc ==1)" << endl;
os <<"          product <=0;" << endl;
os <<"        else" << endl;
os <<"      product <= data_m * data_x;             " << "        // Multiplication Operation   "<< endl;
os <<"    end" << endl;
os <<"   end" << endl;
os <<"" << endl;
os <<"   assign sum = product + data_y;" << endl;
os <<"" << endl;
os <<"endmodule" << endl;
os <<"" << endl;
os <<"" << endl;
os <<"//==================================================MEMORY  MODULE====================================================="<< endl;
os <<"" << endl;
os <<"module memory(clk, data_in, data_out, addr, wr_en);" << endl;
os <<"parameter WIDTH=16, SIZE=64, LOGSIZE=6; input [WIDTH-1:0] data_in;" << endl;
os <<"output logic [WIDTH-1:0] data_out; input [LOGSIZE-1:0] addr;" << endl;
os <<"input clk, wr_en;" << endl;
os <<"logic [SIZE-1:0][WIDTH-1:0] mem;" << endl;
os <<"always_ff @(posedge clk)" << endl;
os <<" begin " << endl;
os <<"data_out <= mem[addr];" << endl;
os <<"if (wr_en)" << endl;
os <<"mem[addr] <= data_in; end" << endl;
os <<"endmodule" << endl;
os <<"" << endl;
}
