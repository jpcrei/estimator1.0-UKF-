#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>


#define PI 3.1415926536

double gaussianNoise()
{// White Gaussian Noise with zero mean and a standard deviation of 1
 
  double temp1;
  double temp2;
  double result;
  int p;

  p = 1;

  while( p > 0 )
  {
	temp2 = ( rand() / ( (double)RAND_MAX ) ); 

    if ( temp2 == 0 )
    {// temp2 is >= (RAND_MAX / 2)
      p = 1;
    }// end if
    else
    {// temp2 is < (RAND_MAX / 2)
       p = -1;
    }// end else

  }// end while()

  temp1 = cos( ( 2.0 * (double)PI ) * rand() / ( (double)RAND_MAX ) );
  result = sqrt( -2.0 * log( temp2 ) ) * temp1;

  return result;	// return the generated random sample to the caller

}
