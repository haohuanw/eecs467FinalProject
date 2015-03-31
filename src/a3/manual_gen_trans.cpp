#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string>
#include "math/point.hpp"
#include <gsl/gsl_linalg.h>
#include <fstream>
#include "common/getopt.h"
#include "math/matd.h"

using namespace std;

int main(int argc, char** argv){
    char *filename;
    getopt_t *gopt = getopt_create();
    getopt_add_string (gopt, 'o', "filename","","File URL");
    if (!getopt_parse (gopt, argc, argv, 1)) {
        printf ("Usage: %s [--filename=FILEURL]\n\n", argv[0]);
        getopt_do_usage (gopt);
        exit (EXIT_FAILURE);
    }
    if(!strncmp (getopt_get_string (gopt, "filename"),"",1)){
        printf("no file name found\n");
        exit (EXIT_FAILURE);
    }
    else{
        char str[100] = "../calibration/";
        char *fn = strdup(getopt_get_string (gopt, "filename"));
        filename = strncat(str,fn,15);
    }

    //pixel based on the image you see on the window
    //(0,0) at bottom left corner
    double a_data[36];
    double b_data[6];
    /*b_data[0] = -1.75/2.0;
    b_data[1] = 1.5;
    b_data[2] = 1.75/2.0;
    b_data[3] = 1.5;
    b_data[4] = 1.75/2.0;
    b_data[5] = -1.5;
    */
    for(int i=0;i<3;++i){
        cout << "Please input the first set of (x_a,y_a) - (x_b,y_b)" << endl;
        cin >> a_data[i*12 +0] >> a_data[i*12 +1] >> b_data[i*2+0] >> b_data[i*2+1];
        cout << a_data[i*12 +0] << ' ' <<  a_data[i*12 +1] << ' ' << b_data[i*2 +0] << ' ' << b_data[i*2 +1] << endl;
        //cout << "File scanned: " << i << endl;
        a_data[i*12 +2] = 1;
        a_data[i*12 +3] = 0;
        a_data[i*12 +4] = 0;
        a_data[i*12 +5] = 0;
        a_data[i*12 +6] = 0;
        a_data[i*12 +7] = 0;
        a_data[i*12 +8] = 0;
        a_data[i*12 +9] = a_data[i*12 +0];
        a_data[i*12 +10] = a_data[i*12 +1];
        a_data[i*12 +11] = 1;
    }
    gsl_matrix_view m = gsl_matrix_view_array(a_data, 6, 6);
    gsl_vector_view b = gsl_vector_view_array(b_data, 6);
    gsl_vector *x = gsl_vector_alloc(6);
    int s;
    gsl_permutation *p = gsl_permutation_alloc(6);
    gsl_linalg_LU_decomp(&m.matrix, p, &s);
    gsl_linalg_LU_solve(&m.matrix, p, &b.vector, x);
    printf("x = \n");
    FILE *fp = fopen(filename,"w");
    gsl_vector_fprintf(fp, x, "%g");
    gsl_vector_fprintf(stdout, x, "%g");
    gsl_permutation_free(p);
    gsl_vector_free(x);
    getopt_destroy(gopt);
}
