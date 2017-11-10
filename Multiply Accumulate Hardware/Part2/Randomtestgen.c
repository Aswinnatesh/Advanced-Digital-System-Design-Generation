// Final With 3 inputs
#include <stdio.h>
#include <time.h>

signed int  a, b, areg, breg;
short int f;
int i, vin=0, vout=0;

int main()
{
        int desiredInputs = 800;   // No of Input values
        srand(time(NULL));   // Set random seed based on current time
        FILE *inputData, *expectedOutput;
        inputData = fopen("inputData","w");
        expectedOutput = fopen("expectedOutput", "w");
        f=0;
        fprintf(expectedOutput,"0\nx\n0\n0\n0\n0\n0\n0\n");
        for(i=0;i<=800;i++)
        {
                a = (rand() % 256) - 128;
                b = (rand() % 256) - 128;
                vin = abs((rand() % 2) - 1);
                if(vin==1)
                {
                        areg = a;
                        breg = b;
                        vout = 1;
			                }
                else
                {
                        areg = 0;
                        breg = 0;
                        vout = 0;
                }

                f= f+(areg*breg);


                printf("%d %d %d %d %d \n",vin,a,b,f,vout);
                fprintf(inputData,"%x\n",(vin>>0)&0xff);
                fprintf(inputData,"%x\n%x\n",((a>>0)&0xff),((b>>0)&0xff));
        //      fprintf(inputData,"%x\n%x\n%x\n",((vin>>0)&0xff),(a>>0)&0xff),((b>>0)&0xff);
		fprintf(expectedOutput, "%d\n%hd\n",vout,f);
        }
        printf("%d",sizeof(f));
        fclose(inputData);
        fclose(expectedOutput);
        return 0;
}
