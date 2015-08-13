#include <stdio.h>
#include <string.h>

typedef struct med_filter med_filter_t;
struct med_filter{
    double values[50];
    double output;
    int window;
    
};

typedef struct mvag_filter mvag_filter_t;
struct mvag_filter{
    double values[50];
    double output;
    int window;
    
};

void quick_sort(double *values,int n);

double median(double *values,int window);

double mean(double *values,int window);

double median_filter(double input,med_filter_t *filt);

double mv_avg_filter(double input,mvag_filter_t *filt);
