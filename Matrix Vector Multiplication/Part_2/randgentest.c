#include <stdio.h>
#include <time.h>
//#include <cstdlib>

int main()
{
	srand(time(NULL)); 
	FILE *inputvalues, *CcodeOutput;
	inputvalues = fopen("C_Input","w");
	CcodeOutput = fopen("C_Output", "w");
	int i,j,k,a;
	char m[3][3],x[3][1],b[3][1],input[5][15];
	short int y[3][1];
	
	
	for ( a = 0; a < 5 ; ++a)
	{
		k=0;
		for ( i = 0; i < 3; ++i)
		     for (j = 0; j < 3; ++j)
		       {
			     m[i][j]=((rand() % 256)-128) ;
			     input[a][k]=m[i][j];
			     k++;
		       }

		for (i = 0; i < 3; ++i)
		     for (j = 0; j < 1; ++j)
		        {
			     b[i][j]=((rand() % 256)-128);
			     input[a][k]=b[i][j];
			     k++;
                }
        for (i = 0; i < 3; ++i)
		     for (j = 0; j < 1; ++j)
		        {
			     x[i][j]=((rand() % 256)-128);
			     input[a][k]=x[i][j];
			     k++;
                }

		for (i = 0; i < 15; ++i)
		     {
			   fprintf(inputvalues,"%x\n",(input[a][i]&0xff));
		     }


	    for (i = 0; i < 3; ++i)                   
	        {
		      y[i][0]=b[i][0];
		       for (j = 0; j < 3; ++j)
		          {
			        y[i][0] += m[i][j] * x[j][0];
		          }
	        }

	          for (i = 0; i < 3; ++i)
	              {
		            for (j = 0; j < 3; ++j)
			        printf("%d ",m[i][j] );
		            printf("\n");
	              }
	          for (i = 0; i < 3; ++i)
	             {
		           for (j = 0; j < 1; ++j)
			       printf("%d ",b[i][j]);
		           printf("\n");
	             }   

	         for (i = 0; i < 3; ++i)
	             {
		           for (j = 0; j < 1; ++j)
			       printf("%d ",x[i][j] );
		           printf("\n");
	             }

	         for ( i = 0; i < 3; ++i)
	             {
		         for ( j = 0; j < 1; ++j)
    			         // Over here : the code has to check if (y[i][j])is greater than (2^15), if yes, 
			          if(y[i][j] >32678)
			          	 {
			         // Printf(CcodeOutput, "OVERFLOW");
			          	 	 fprintf(CcodeOutput,"OVERFLOW");
			         // else below part
			             }
			             else
			             {  
			         fprintf(CcodeOutput,"%hd\n",(y[i][j]&0xffff));
			         fprintf(CcodeOutput,"0 \n");
		                 }
	    }
	}


	fclose(inputvalues);
	fclose(CcodeOutput);
	
}

					 
