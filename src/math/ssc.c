#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>
#include <math.h>
#include <gsl/gsl_errno.h>

#include "math_util.h"
#include "gsl_util.h"
#include "so3.h"
#include "dm.h"
#include "ssc.h"

// helper macros for easy indexing
#define X_ij(i) (X_ij[i])
#define X_jk(i) (X_jk[i])
#define X_ji(i) (X_ji[i])
#define X_ik(i) (X_ik[i])
#define R_ij(i,j) (R_ij.data[i*3+j])
#define R_jk(i,j) (R_jk.data[i*3+j])
#define R_ik(i,j) (R_ik.data[i*3+j])

void
ssc_pose_printf (const double X[6], const char *name, bool degree, bool rowvec)
{
    GSLU_VECTOR_VIEW (x_ij, 6, {X[0], X[1], X[2], X[3], X[4], X[5]});
    char varname[256];
    if (degree) {
        x_ij.data[3] *= RTOD;
        x_ij.data[4] *= RTOD;
        x_ij.data[5] *= RTOD;
        snprintf (varname, sizeof varname, "%s (deg)", name);
    }
    else
        snprintf (varname, sizeof varname, "%s", name);

    gslu_vector_printfc (&x_ij.vector, varname, NULL, rowvec ? CblasTrans : CblasNoTrans);
}


// Smith, Self, and Cheesemen 6-DOF coordinate frame relationships
int
ssc_inverse_gsl (gsl_vector *X_ji, gsl_matrix *J_minus, const gsl_vector *X_ij)
{
    assert (X_ji->size == 6 && X_ji->stride == 1);
    assert (X_ij->size == 6 && X_ij->stride == 1);

    if (J_minus) {
        assert (J_minus->size1 == 6 && J_minus->size2 == 6 && J_minus->tda == 6);
        return ssc_inverse (X_ji->data, J_minus->data, X_ij->data);
    }
    else
        return ssc_inverse (X_ji->data, NULL, X_ij->data);
}

int
ssc_inverse (double X_ji[6], double J_minus[6*6], const double X_ij[6])
{
    GSLU_MATRIX_VIEW (R_ij, 3, 3);
    so3_rotxyz (R_ij.data, SSC_RPH (X_ij));

    // X_ji
    const double h_ji = atan2 (R_ij(0,1), R_ij(0,0));
#ifdef _GNU_SOURCE
    double sh_ji, ch_ji;
    sincos (h_ji, &sh_ji, &ch_ji);
#else
    double sh_ji = sin (h_ji), ch_ji = cos(h_ji);
#endif

    const double x_ji = -(R_ij(0,0)*X_ij(0) + R_ij(1,0)*X_ij(1) + R_ij(2,0)*X_ij(2));
    const double y_ji = -(R_ij(0,1)*X_ij(0) + R_ij(1,1)*X_ij(1) + R_ij(2,1)*X_ij(2));
    const double z_ji = -(R_ij(0,2)*X_ij(0) + R_ij(1,2)*X_ij(1) + R_ij(2,2)*X_ij(2));
    const double r_ji = atan2 (R_ij(2,0)*sh_ji - R_ij(2,1)*ch_ji, -R_ij(1,0)*sh_ji + R_ij(1,1)*ch_ji);
    const double p_ji = atan2 (-R_ij(0,2), R_ij(0,0)*ch_ji + R_ij(0,1)*sh_ji);

    X_ji(SSC_DOF_X) = x_ji;
    X_ji(SSC_DOF_Y) = y_ji;
    X_ji(SSC_DOF_Z) = z_ji;
    X_ji(SSC_DOF_R) = r_ji;
    X_ji(SSC_DOF_P) = p_ji;
    X_ji(SSC_DOF_H) = h_ji;

    if (J_minus != NULL) {
        const double x_ij=X_ij(SSC_DOF_X), y_ij=X_ij(SSC_DOF_Y), z_ij=X_ij(SSC_DOF_Z);
        const double r_ij=X_ij(SSC_DOF_R), p_ij=X_ij(SSC_DOF_P), h_ij=X_ij(SSC_DOF_H);

#ifdef _GNU_SOURCE
        double sr_ij, cr_ij, sp_ij, cp_ij, sh_ij, ch_ij;
        sincos (r_ij, &sr_ij, &cr_ij);
        sincos (p_ij, &sp_ij, &cp_ij);
        sincos (h_ij, &sh_ij, &ch_ij);
#else
        double sr_ij = sin (r_ij), cr_ij = cos (r_ij);
        double sp_ij = sin (p_ij), cp_ij = cos (p_ij);
        double sh_ij = sin (h_ij), ch_ij = cos (h_ij);
#endif

        // N
        double tmp = x_ij*ch_ij + y_ij*sh_ij;
        const double N_data[] = {
             0,    -R_ij(2,0)*tmp + z_ij*cp_ij,       R_ij(1,0)*x_ij - R_ij(0,0)*y_ij,
             z_ji, -R_ij(2,1)*tmp + z_ij*sp_ij*sr_ij, R_ij(1,1)*x_ij - R_ij(0,1)*y_ij,
            -y_ji, -R_ij(2,2)*tmp + z_ij*sp_ij*cr_ij, R_ij(1,2)*x_ij - R_ij(0,2)*y_ij
        };
        gsl_matrix_const_view N = gsl_matrix_const_view_array (N_data, 3, 3);

        // Q
        tmp = sqrt (1-R_ij(0,2)*R_ij(0,2));
        double Q_data[] = {
            -R_ij(0,0),           -R_ij(0,1)*cr_ij,      R_ij(0,2)*R_ij(2,2),
             R_ij(0,1)*tmp,       -R_ij(2,2)*ch_ij*tmp,  R_ij(1,2)*tmp,
             R_ij(0,2)*R_ij(0,0), -R_ij(1,2)*ch_ij,     -R_ij(2,2)
        };
        gsl_matrix_view Q = gsl_matrix_view_array (Q_data, 3, 3);
        gsl_matrix_scale (&Q.matrix, 1./(tmp*tmp));

        // J_minus
        gsl_matrix_view J = gsl_matrix_view_array (J_minus, 6, 6);
        gsl_matrix_set_zero (&J.matrix);
        gsl_matrix_transpose (&R_ij.matrix);
        gsl_matrix_scale (&R_ij.matrix, -1.0);
        gslu_matrix_set_submatrix (&J.matrix, 0, 0, &R_ij.matrix); // -R_ij'
        gslu_matrix_set_submatrix (&J.matrix, 0, 3, &N.matrix);
        gslu_matrix_set_submatrix (&J.matrix, 3, 3, &Q.matrix);
    }
    return GSL_SUCCESS;
}

int
ssc_head2tail_gsl (gsl_vector *X_ik, gsl_matrix *J_plus, const gsl_vector *X_ij, const gsl_vector *X_jk)
{
    assert (X_ik->size == 6 && X_ik->stride == 1);
    assert (X_ij->size == 6 && X_ij->stride == 1);
    assert (X_jk->size == 6 && X_jk->stride == 1);

    if (J_plus) {
        assert (J_plus->size1 == 6 && J_plus->size2 == 12 && J_plus->tda == 12);
        return ssc_head2tail (X_ik->data, J_plus->data, X_ij->data, X_jk->data);
    }
    else
        return ssc_head2tail (X_ik->data, NULL, X_ij->data, X_jk->data);
}

int
ssc_head2tail (double X_ik[6], double J_plus[6*12], const double X_ij[6], const double X_jk[6])
{
    // R_ij, R_jk, R_ik
    GSLU_MATRIX_VIEW (R_ij, 3, 3);
    GSLU_MATRIX_VIEW (R_jk, 3, 3);
    GSLU_MATRIX_VIEW (R_ik, 3, 3);
    so3_rotxyz (R_ij.data, SSC_RPH (X_ij));
    so3_rotxyz (R_jk.data, SSC_RPH (X_jk));
    gslu_blas_mm (&R_ik.matrix, &R_ij.matrix, &R_jk.matrix);

    // X_ij
    const double x_ij = SSC_X(X_ij), y_ij = SSC_Y(X_ij), z_ij = SSC_Z(X_ij);
    const double r_ij = SSC_R(X_ij), p_ij = SSC_P(X_ij), h_ij = SSC_H(X_ij);

    // X_jk
    const double x_jk = SSC_X(X_jk), y_jk = SSC_Y(X_jk), z_jk = SSC_Z(X_jk);
    const double r_jk = SSC_R(X_jk), p_jk = SSC_P(X_jk); //, h_jk = SSC_H(X_jk);

    // X_ik
    const double x_ik = R_ij(0,0)*x_jk + R_ij(0,1)*y_jk + R_ij(0,2)*z_jk + x_ij;
    const double y_ik = R_ij(1,0)*x_jk + R_ij(1,1)*y_jk + R_ij(1,2)*z_jk + y_ij;
    const double z_ik = R_ij(2,0)*x_jk + R_ij(2,1)*y_jk + R_ij(2,2)*z_jk + z_ij;
    const double h_ik = atan2 (R_ik(1,0), R_ik(0,0));
#ifdef _GNU_SOURCE
    double sh_ik, ch_ik;
    sincos (h_ik, &sh_ik, &ch_ik);
#else
    double sh_ik = sin (h_ik), ch_ik = cos (h_ik);
#endif
    const double r_ik = atan2 (R_ik(0,2)*sh_ik - R_ik(1,2)*ch_ik, -R_ik(0,1)*sh_ik + R_ik(1,1)*ch_ik);
    const double p_ik = atan2 (-R_ik(2,0), R_ik(0,0)*ch_ik + R_ik(1,0)*sh_ik);

    X_ik(SSC_DOF_X) = x_ik;
    X_ik(SSC_DOF_Y) = y_ik;
    X_ik(SSC_DOF_Z) = z_ik;
    X_ik(SSC_DOF_R) = r_ik;
    X_ik(SSC_DOF_P) = p_ik;
    X_ik(SSC_DOF_H) = h_ik;

    if (J_plus != NULL) {
#ifdef _GNU_SOURCE
        double sr_ij, cr_ij;
        sincos (r_ij, &sr_ij, &cr_ij);
        double sp_ij, cp_ij;
        sincos (p_ij, &sp_ij, &cp_ij);
        double sh_ij, ch_ij;
        sincos (h_ij, &sh_ij, &ch_ij);

        double sp_jk, cp_jk;
        sincos (p_jk, &sp_jk, &cp_jk);

        double sr_ik, cr_ik;
        sincos (r_ik, &sr_ik, &cr_ik);
        double sp_ik, cp_ik;
        sincos (p_ik, &sp_ik, &cp_ik);

        double sh_dif, ch_dif;
        sincos (h_ik-h_ij, &sh_dif, &ch_dif);
        double sr_dif, cr_dif;
        sincos (r_ik-r_jk, &sr_dif, &cr_dif);
#else
        double sr_ij = sin (r_ij), cr_ij = cos (r_ij);
        double sp_ij = sin (p_ij), cp_ij = cos (p_ij);
        double sh_ij = sin (h_ij), ch_ij = cos (h_ij);

        double sp_jk = sin (p_jk), cp_jk = cos (p_jk);

        double sr_ik = sin (r_ik), cr_ik = cos (r_ik);
        double sp_ik = sin (p_ik), cp_ik = cos (p_ik);

        double sh_dif = sin (h_ik-h_ij), ch_dif = cos (h_ik-h_ij);
        double sr_dif = sin (r_ik-r_jk), cr_dif = cos (r_ik-r_jk);
#endif
        const double tp_ik = tan (p_ik);

        const double x_dif = x_ik - x_ij, y_dif = y_ik - y_ij, z_dif = z_ik - z_ij;

        if (fabs (cp_ik) < 1e-10)
            GSL_ERROR ("divide by zero, cp_ik", GSL_EZERODIV);
        double secp_ik = 1/cp_ik;

        // M
        const double M_data[9] =
            { R_ij(0,2)*y_jk - R_ij(0,1)*z_jk,  z_dif*ch_ij,                                  -y_dif,
              R_ij(1,2)*y_jk - R_ij(1,1)*z_jk,  z_dif*sh_ij,                                   x_dif,
              R_ij(2,2)*y_jk - R_ij(2,1)*z_jk, -x_jk*cp_ij - (y_jk*sr_ij + z_jk*cr_ij)*sp_ij,  0       };
        gsl_matrix_const_view M = gsl_matrix_const_view_array (M_data, 3, 3);

        // K1
        const double K1_data[9] =
            { cp_ij*ch_dif*secp_ik,                         sh_dif*secp_ik,  0,
             -cp_ij*sh_dif,                                 ch_dif,          0,
              (R_jk(0,1)*sr_ik + R_jk(0,2)*cr_ik)*secp_ik,  sh_dif*tp_ik,    1  };
        gsl_matrix_const_view K1 = gsl_matrix_const_view_array (K1_data, 3, 3);

        // K2
        const double K2_data[9] =
            { 1,  sr_dif*tp_ik,    (R_ij(0,2)*ch_ik + R_ij(1,2)*sh_ik)*secp_ik,
              0,  cr_dif,         -cp_jk*sr_dif,
              0,  sr_dif*secp_ik,  cp_jk*cr_dif*secp_ik };
        gsl_matrix_const_view K2 = gsl_matrix_const_view_array (K2_data, 3, 3);


        // I_3x3
        double I_data[9] = { 1, 0, 0,
                             0, 1, 0,
                             0, 0, 1  };
        gsl_matrix_view I = gsl_matrix_view_array (I_data, 3, 3);

        // J_plus
        gsl_matrix_view J = gsl_matrix_view_array (J_plus, 6, 12);
        gsl_matrix_set_zero (&J.matrix);
        gslu_matrix_set_submatrix (&J.matrix, 0, 0, &I.matrix);
        gslu_matrix_set_submatrix (&J.matrix, 0, 3, &M.matrix);
        gslu_matrix_set_submatrix (&J.matrix, 0, 6, &R_ij.matrix);
        gslu_matrix_set_submatrix (&J.matrix, 3, 3, &K1.matrix);
        gslu_matrix_set_submatrix (&J.matrix, 3, 9, &K2.matrix);
    }
    return GSL_SUCCESS;
}

int
ssc_tail2tail_gsl (gsl_vector *X_jk, gsl_matrix *J_tail, const gsl_vector *X_ij, const gsl_vector *X_ik)
{
    assert (X_jk->size == 6 && X_jk->stride == 1);
    assert (X_ij->size == 6 && X_ij->stride == 1);
    assert (X_ik->size == 6 && X_ik->stride == 1);

    if (J_tail) {
        assert (J_tail->size1 == 6 && J_tail->size2 == 12 && J_tail->tda == 12);
        return ssc_tail2tail (X_jk->data, J_tail->data, X_ij->data, X_ik->data);
    }
    else
        return ssc_tail2tail (X_jk->data, NULL, X_ij->data, X_ik->data);
}

int
ssc_tail2tail (double X_jk[6], double J_tail[6*12], const double X_ij[6], const double X_ik[6])
{
    GSLU_VECTOR_VIEW (X_ji, 6);

    if (J_tail == NULL) {
        // x_ji = ominus x_ij
        ssc_inverse (X_ji.data, NULL, X_ij);

        // x_jk = x_ji oplus x_ik
        ssc_head2tail (X_jk, NULL, X_ji.data, X_ik);
    }
    else {
        // x_ji = ominus x_ij
        GSLU_MATRIX_VIEW (J_minus, 6, 6);
        ssc_inverse (X_ji.data, J_minus.data, X_ij);

        // x_jk = x_ji oplus x_ik
        GSLU_MATRIX_VIEW (J_plus, 6, 12);
        ssc_head2tail (X_jk, J_plus.data, X_ji.data, X_ik);
        gsl_matrix_view J_1plus = gsl_matrix_submatrix (&J_plus.matrix, 0, 0, 6, 6);
        gsl_matrix_view J_2plus = gsl_matrix_submatrix (&J_plus.matrix, 0, 6, 6, 6);

        // Jacobian
        gsl_matrix_view J = gsl_matrix_view_array (J_tail, 6, 12);
        GSLU_MATRIX_VIEW (J_1pm, 6, 6);
        gslu_blas_mm (&J_1pm.matrix, &J_1plus.matrix, &J_minus.matrix);
        gslu_matrix_set_submatrix (&J.matrix, 0, 0, &J_1pm.matrix);
        gslu_matrix_set_submatrix (&J.matrix, 0, 6, &J_2plus.matrix);
    }
    return GSL_SUCCESS;
}

int
ssc_homo4x4_gsl (gsl_matrix *H_ij, const gsl_vector *X_ij)
{
    assert (H_ij->size1 == 4 && H_ij->size2 && H_ij->tda == 4);
    assert (X_ij->size == 6 && X_ij->stride == 1);

    return ssc_homo4x4 (H_ij->data, X_ij->data);
}

int
ssc_homo4x4 (double H_ij[4*4], const double X_ij[6])
{
    GSLU_MATRIX_VIEW (R_ij, 3, 3);
    so3_rotxyz (R_ij.data, SSC_RPH (X_ij));

    const double H_ij_data[4*4] = {
        R_ij(0,0), R_ij(0,1), R_ij(0,2), X_ij(0),
        R_ij(1,0), R_ij(1,1), R_ij(1,2), X_ij(1),
        R_ij(2,0), R_ij(2,1), R_ij(2,2), X_ij(2),
        0,         0,         0,         1
    };
    memcpy (H_ij, H_ij_data, sizeof H_ij_data);

    return GSL_SUCCESS;
}
