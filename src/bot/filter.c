#include <stdio.h>
#include <string.h>
#include "filter.h"

//#define DEBUG

//#define WINDOW_MED 5  //Choose odd number
//#define WINDOW_MVAG 30

//med_filter_t x_filt;
//mvag_filter_t x_filt_mvag;

double median_filter(double input,med_filter_t *filt){

    int j;
    
    double y_sort[500];


    for(j=0;j<filt->window;j++){
	if(j<filt->window-1)
	    filt->values[j] = filt->values[j+1];
	else
	    filt->values[j] = input;
	
    }
        
#ifdef DEBUG
    printf("WINDOW_MED DATA:\n");
    
    int l;

    for(l=0;l<filt->window;l++){
      printf("%2.1f,",filt->values[l]);
    }
    
    printf("\n");
#endif
    
    memcpy(y_sort,filt->values,sizeof(double)*filt->window);
    
    quick_sort(y_sort,filt->window);
    
    
    
#ifdef DEBUG
    printf("Sorted DATA:\n");
    
    l = 0;

    for(l=0;l<filt->window;l++){
      printf("%2.1f,",y_sort[l]);
    }
    
    printf("\n");
#endif

    filt->output = median(y_sort,filt->window);

    
    return filt->output;

}

double mv_avg_filter(double input,mvag_filter_t *filt){

    int j;
    

    for(j=0;j<filt->window;j++){
	if(j<filt->window-1)
	    filt->values[j] = filt->values[j+1];
	else
	    filt->values[j] = input;
	
    }

#ifdef DEBUG
    
    
    printf("WINDOW_MVAG DATA:\n");
    
    int l;

    for(l=0;l<filt->window;l++){
      printf("%2.1f,",filt->values[l]);
    }
    
    printf("\n");
#endif


    double output = mean(filt->values,filt->window);

    return output;

}


void quick_sort (double *a, int n) {
    if (n < 2)
        return;
    int p = a[n / 2];
    double *l = a;
    double *r = a + n - 1;
    while (l <= r) {
        if (*l < p) {
            l++;
        }
        else if (*r > p) {
            r--;
        }
        else {
            double t = *l;
            *l = *r;
            *r = t;
            l++;
            r--;
        }
    }
    quick_sort(a, r - a + 1);
    quick_sort(l, a + n - l);
}

double median(double *values,int window){

    if(window%2 == 0){ 
	return ((values[window/2-1] + values[window/2])/2);
    }
    else{
	return values[window/2] ;
    }

}

double mean(double *values,int window){

    int i;

    double mean = 0;

    for(i=0;i<window;i++){
	mean += values[i];
    }

    mean = mean/window;

    return mean;
}

/*
#define DATA 20

int main(){

    double y[DATA]= {1,2,3,6,2,3,5,2,7,8,2,4,6,6,1,3,6,2,3,8};

    memset(&x_filt,0,sizeof(med_filter_t));
    x_filt.window = 3 ;
    
    memset(&x_filt_mvag,0,sizeof(mvag_filter_t));
    x_filt_mvag.window = 2 ;
    

    printf("DATA:\n");
    
    int i,k;

    for(k=0;k<DATA;k++){
	printf("%2.1f, ",y[k]);
    }

    printf("\n");

    double output;

    for(i=0;i<DATA;i++){

	output = median_filter(y[i],&x_filt);

	output = mv_avg_filter(output,&x_filt_mvag);

	printf("%2.1f, ",output);

    }
   
    

    printf("\n");


    if(12 == 014)
	printf("octal conversion is true\n");
    
}

*/
