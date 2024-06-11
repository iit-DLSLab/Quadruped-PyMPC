/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

// standard
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
// acados
// #include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

// example specific
#include "centroidal_model_model/centroidal_model_model.h"
#include "centroidal_model_constraints/centroidal_model_constraints.h"



#include "acados_solver_centroidal_model.h"

#define NX     CENTROIDAL_MODEL_NX
#define NZ     CENTROIDAL_MODEL_NZ
#define NU     CENTROIDAL_MODEL_NU
#define NP     CENTROIDAL_MODEL_NP
#define NY0    CENTROIDAL_MODEL_NY0
#define NY     CENTROIDAL_MODEL_NY
#define NYN    CENTROIDAL_MODEL_NYN

#define NBX    CENTROIDAL_MODEL_NBX
#define NBX0   CENTROIDAL_MODEL_NBX0
#define NBU    CENTROIDAL_MODEL_NBU
#define NG     CENTROIDAL_MODEL_NG
#define NBXN   CENTROIDAL_MODEL_NBXN
#define NGN    CENTROIDAL_MODEL_NGN

#define NH     CENTROIDAL_MODEL_NH
#define NHN    CENTROIDAL_MODEL_NHN
#define NH0    CENTROIDAL_MODEL_NH0
#define NPHI   CENTROIDAL_MODEL_NPHI
#define NPHIN  CENTROIDAL_MODEL_NPHIN
#define NPHI0  CENTROIDAL_MODEL_NPHI0
#define NR     CENTROIDAL_MODEL_NR

#define NS     CENTROIDAL_MODEL_NS
#define NS0    CENTROIDAL_MODEL_NS0
#define NSN    CENTROIDAL_MODEL_NSN

#define NSBX   CENTROIDAL_MODEL_NSBX
#define NSBU   CENTROIDAL_MODEL_NSBU
#define NSH0   CENTROIDAL_MODEL_NSH0
#define NSH    CENTROIDAL_MODEL_NSH
#define NSHN   CENTROIDAL_MODEL_NSHN
#define NSG    CENTROIDAL_MODEL_NSG
#define NSPHI0 CENTROIDAL_MODEL_NSPHI0
#define NSPHI  CENTROIDAL_MODEL_NSPHI
#define NSPHIN CENTROIDAL_MODEL_NSPHIN
#define NSGN   CENTROIDAL_MODEL_NSGN
#define NSBXN  CENTROIDAL_MODEL_NSBXN



// ** solver data **

centroidal_model_solver_capsule * centroidal_model_acados_create_capsule(void)
{
    void* capsule_mem = malloc(sizeof(centroidal_model_solver_capsule));
    centroidal_model_solver_capsule *capsule = (centroidal_model_solver_capsule *) capsule_mem;

    return capsule;
}


int centroidal_model_acados_free_capsule(centroidal_model_solver_capsule *capsule)
{
    free(capsule);
    return 0;
}


int centroidal_model_acados_create(centroidal_model_solver_capsule* capsule)
{
    int N_shooting_intervals = CENTROIDAL_MODEL_N;
    double* new_time_steps = NULL; // NULL -> don't alter the code generated time-steps
    return centroidal_model_acados_create_with_discretization(capsule, N_shooting_intervals, new_time_steps);
}


int centroidal_model_acados_update_time_steps(centroidal_model_solver_capsule* capsule, int N, double* new_time_steps)
{
    if (N != capsule->nlp_solver_plan->N) {
        fprintf(stderr, "centroidal_model_acados_update_time_steps: given number of time steps (= %d) " \
            "differs from the currently allocated number of " \
            "time steps (= %d)!\n" \
            "Please recreate with new discretization and provide a new vector of time_stamps!\n",
            N, capsule->nlp_solver_plan->N);
        return 1;
    }

    ocp_nlp_config * nlp_config = capsule->nlp_config;
    ocp_nlp_dims * nlp_dims = capsule->nlp_dims;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &new_time_steps[i]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &new_time_steps[i]);
    }
    return 0;
}

/**
 * Internal function for centroidal_model_acados_create: step 1
 */
void centroidal_model_acados_create_1_set_plan(ocp_nlp_plan_t* nlp_solver_plan, const int N)
{
    assert(N == nlp_solver_plan->N);

    /************************************************
    *  plan
    ************************************************/

    nlp_solver_plan->nlp_solver = SQP;

    nlp_solver_plan->ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;

    nlp_solver_plan->nlp_cost[0] = LINEAR_LS;
    for (int i = 1; i < N; i++)
        nlp_solver_plan->nlp_cost[i] = LINEAR_LS;

    nlp_solver_plan->nlp_cost[N] = LINEAR_LS;

    for (int i = 0; i < N; i++)
    {
        nlp_solver_plan->nlp_dynamics[i] = CONTINUOUS_MODEL;
        nlp_solver_plan->sim_solver_plan[i].sim_solver = ERK;
    }

    nlp_solver_plan->nlp_constraints[0] = BGH;

    for (int i = 1; i < N; i++)
    {
        nlp_solver_plan->nlp_constraints[i] = BGH;
    }
    nlp_solver_plan->nlp_constraints[N] = BGH;

    nlp_solver_plan->regularization = NO_REGULARIZE;
}


/**
 * Internal function for centroidal_model_acados_create: step 2
 */
ocp_nlp_dims* centroidal_model_acados_create_2_create_and_set_dimensions(centroidal_model_solver_capsule* capsule)
{
    ocp_nlp_plan_t* nlp_solver_plan = capsule->nlp_solver_plan;
    const int N = nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;

    /************************************************
    *  dimensions
    ************************************************/
    #define NINTNP1MEMS 18
    int* intNp1mem = (int*)malloc( (N+1)*sizeof(int)*NINTNP1MEMS );

    int* nx    = intNp1mem + (N+1)*0;
    int* nu    = intNp1mem + (N+1)*1;
    int* nbx   = intNp1mem + (N+1)*2;
    int* nbu   = intNp1mem + (N+1)*3;
    int* nsbx  = intNp1mem + (N+1)*4;
    int* nsbu  = intNp1mem + (N+1)*5;
    int* nsg   = intNp1mem + (N+1)*6;
    int* nsh   = intNp1mem + (N+1)*7;
    int* nsphi = intNp1mem + (N+1)*8;
    int* ns    = intNp1mem + (N+1)*9;
    int* ng    = intNp1mem + (N+1)*10;
    int* nh    = intNp1mem + (N+1)*11;
    int* nphi  = intNp1mem + (N+1)*12;
    int* nz    = intNp1mem + (N+1)*13;
    int* ny    = intNp1mem + (N+1)*14;
    int* nr    = intNp1mem + (N+1)*15;
    int* nbxe  = intNp1mem + (N+1)*16;
    int* np  = intNp1mem + (N+1)*17;

    for (int i = 0; i < N+1; i++)
    {
        // common
        nx[i]     = NX;
        nu[i]     = NU;
        nz[i]     = NZ;
        ns[i]     = NS;
        // cost
        ny[i]     = NY;
        // constraints
        nbx[i]    = NBX;
        nbu[i]    = NBU;
        nsbx[i]   = NSBX;
        nsbu[i]   = NSBU;
        nsg[i]    = NSG;
        nsh[i]    = NSH;
        nsphi[i]  = NSPHI;
        ng[i]     = NG;
        nh[i]     = NH;
        nphi[i]   = NPHI;
        nr[i]     = NR;
        nbxe[i]   = 0;
        np[i]     = NP;
    }

    // for initial state
    nbx[0] = NBX0;
    nsbx[0] = 0;
    ns[0] = NS0;
    nbxe[0] = 30;
    ny[0] = NY0;
    nh[0] = NH0;
    nsh[0] = NSH0;
    nsphi[0] = NSPHI0;
    nphi[0] = NPHI0;


    // terminal - common
    nu[N]   = 0;
    nz[N]   = 0;
    ns[N]   = NSN;
    // cost
    ny[N]   = NYN;
    // constraint
    nbx[N]   = NBXN;
    nbu[N]   = 0;
    ng[N]    = NGN;
    nh[N]    = NHN;
    nphi[N]  = NPHIN;
    nr[N]    = 0;

    nsbx[N]  = NSBXN;
    nsbu[N]  = 0;
    nsg[N]   = NSGN;
    nsh[N]   = NSHN;
    nsphi[N] = NSPHIN;

    /* create and set ocp_nlp_dims */
    ocp_nlp_dims * nlp_dims = ocp_nlp_dims_create(nlp_config);

    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nx", nx);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nu", nu);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nz", nz);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "ns", ns);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "np", np);

    for (int i = 0; i <= N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbx", &nbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbu", &nbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbx", &nsbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbu", &nsbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "ng", &ng[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsg", &nsg[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbxe", &nbxe[i]);
    }
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, 0, "ny", &ny[0]);
    for (int i = 1; i < N; i++)
        ocp_nlp_dims_set_cost(nlp_config, nlp_dims, i, "ny", &ny[i]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, 0, "nh", &nh[0]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, 0, "nsh", &nsh[0]);

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nh", &nh[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsh", &nsh[i]);
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nh", &nh[N]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nsh", &nsh[N]);
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, N, "ny", &ny[N]);

    free(intNp1mem);

    return nlp_dims;
}


/**
 * Internal function for centroidal_model_acados_create: step 3
 */
void centroidal_model_acados_create_3_create_and_set_functions(centroidal_model_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;


    /************************************************
    *  external functions
    ************************************************/

#define MAP_CASADI_FNC(__CAPSULE_FNC__, __MODEL_BASE_FNC__) do{ \
        capsule->__CAPSULE_FNC__.casadi_fun = & __MODEL_BASE_FNC__ ;\
        capsule->__CAPSULE_FNC__.casadi_n_in = & __MODEL_BASE_FNC__ ## _n_in; \
        capsule->__CAPSULE_FNC__.casadi_n_out = & __MODEL_BASE_FNC__ ## _n_out; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_in = & __MODEL_BASE_FNC__ ## _sparsity_in; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_out = & __MODEL_BASE_FNC__ ## _sparsity_out; \
        capsule->__CAPSULE_FNC__.casadi_work = & __MODEL_BASE_FNC__ ## _work; \
        external_function_param_casadi_create(&capsule->__CAPSULE_FNC__ , 29); \
    } while(false)
    MAP_CASADI_FNC(nl_constr_h_0_fun_jac, centroidal_model_constr_h_0_fun_jac_uxt_zt);
    MAP_CASADI_FNC(nl_constr_h_0_fun, centroidal_model_constr_h_0_fun);
    // constraints.constr_type == "BGH" and dims.nh > 0
    capsule->nl_constr_h_fun_jac = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*(N-1));
    for (int i = 0; i < N-1; i++) {
        MAP_CASADI_FNC(nl_constr_h_fun_jac[i], centroidal_model_constr_h_fun_jac_uxt_zt);
    }
    capsule->nl_constr_h_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*(N-1));
    for (int i = 0; i < N-1; i++) {
        MAP_CASADI_FNC(nl_constr_h_fun[i], centroidal_model_constr_h_fun);
    }
    



    // explicit ode
    capsule->forw_vde_casadi = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(forw_vde_casadi[i], centroidal_model_expl_vde_forw);
    }

    capsule->expl_ode_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(expl_ode_fun[i], centroidal_model_expl_ode_fun);
    }



#undef MAP_CASADI_FNC
}


/**
 * Internal function for centroidal_model_acados_create: step 4
 */
void centroidal_model_acados_create_4_set_default_parameters(centroidal_model_solver_capsule* capsule) {
    const int N = capsule->nlp_solver_plan->N;
    // initialize parameters to nominal value
    double* p = calloc(NP, sizeof(double));
    p[0] = 1;
    p[1] = 1;
    p[2] = 1;
    p[3] = 1;
    p[4] = 0.5;
    p[19] = 0.2310941359705289;
    p[20] = -0.0014987128245817424;
    p[21] = -0.021400468992761768;
    p[22] = -0.0014987128245817424;
    p[23] = 1.4485084687476608;
    p[24] = 0.0004641447134275615;
    p[25] = -0.021400468992761768;
    p[26] = 0.0004641447134275615;
    p[27] = 1.503217877350808;
    p[28] = 24.637;

    for (int i = 0; i <= N; i++) {
        centroidal_model_acados_update_params(capsule, i, p, NP);
    }
    free(p);
}


/**
 * Internal function for centroidal_model_acados_create: step 5
 */
void centroidal_model_acados_create_5_set_nlp_in(centroidal_model_solver_capsule* capsule, const int N, double* new_time_steps)
{
    assert(N == capsule->nlp_solver_plan->N);
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;

    int tmp_int = 0;

    /************************************************
    *  nlp_in
    ************************************************/
//    ocp_nlp_in * nlp_in = ocp_nlp_in_create(nlp_config, nlp_dims);
//    capsule->nlp_in = nlp_in;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    // set up time_steps

    if (new_time_steps)
    {
        centroidal_model_acados_update_time_steps(capsule, N, new_time_steps);
    }
    else
    {double time_step = 0.02;
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &time_step);
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &time_step);
        }
    }

    /**** Dynamics ****/
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "expl_vde_forw", &capsule->forw_vde_casadi[i]);
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "expl_ode_fun", &capsule->expl_ode_fun[i]);
    }

    /**** Cost ****/
    double* yref_0 = calloc(NY0, sizeof(double));
    // change only the non-zero elements:
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "yref", yref_0);
    free(yref_0);

   double* W_0 = calloc(NY0*NY0, sizeof(double));
    // change only the non-zero elements:
    W_0[2+(NY0) * 2] = 1500;
    W_0[3+(NY0) * 3] = 200;
    W_0[4+(NY0) * 4] = 200;
    W_0[5+(NY0) * 5] = 200;
    W_0[6+(NY0) * 6] = 500;
    W_0[7+(NY0) * 7] = 500;
    W_0[9+(NY0) * 9] = 20;
    W_0[10+(NY0) * 10] = 20;
    W_0[11+(NY0) * 11] = 50;
    W_0[12+(NY0) * 12] = 300;
    W_0[13+(NY0) * 13] = 300;
    W_0[14+(NY0) * 14] = 300;
    W_0[15+(NY0) * 15] = 300;
    W_0[16+(NY0) * 16] = 300;
    W_0[17+(NY0) * 17] = 300;
    W_0[18+(NY0) * 18] = 300;
    W_0[19+(NY0) * 19] = 300;
    W_0[20+(NY0) * 20] = 300;
    W_0[21+(NY0) * 21] = 300;
    W_0[22+(NY0) * 22] = 300;
    W_0[23+(NY0) * 23] = 300;
    W_0[24+(NY0) * 24] = 50;
    W_0[25+(NY0) * 25] = 10;
    W_0[26+(NY0) * 26] = 10;
    W_0[27+(NY0) * 27] = 10;
    W_0[28+(NY0) * 28] = 10;
    W_0[29+(NY0) * 29] = 10;
    W_0[30+(NY0) * 30] = 0.0001;
    W_0[31+(NY0) * 31] = 0.0001;
    W_0[32+(NY0) * 32] = 0.00001;
    W_0[33+(NY0) * 33] = 0.0001;
    W_0[34+(NY0) * 34] = 0.0001;
    W_0[35+(NY0) * 35] = 0.00001;
    W_0[36+(NY0) * 36] = 0.0001;
    W_0[37+(NY0) * 37] = 0.0001;
    W_0[38+(NY0) * 38] = 0.00001;
    W_0[39+(NY0) * 39] = 0.0001;
    W_0[40+(NY0) * 40] = 0.0001;
    W_0[41+(NY0) * 41] = 0.00001;
    W_0[42+(NY0) * 42] = 0.001;
    W_0[43+(NY0) * 43] = 0.001;
    W_0[44+(NY0) * 44] = 0.001;
    W_0[45+(NY0) * 45] = 0.001;
    W_0[46+(NY0) * 46] = 0.001;
    W_0[47+(NY0) * 47] = 0.001;
    W_0[48+(NY0) * 48] = 0.001;
    W_0[49+(NY0) * 49] = 0.001;
    W_0[50+(NY0) * 50] = 0.001;
    W_0[51+(NY0) * 51] = 0.001;
    W_0[52+(NY0) * 52] = 0.001;
    W_0[53+(NY0) * 53] = 0.001;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "W", W_0);
    free(W_0);
    double* Vx_0 = calloc(NY0*NX, sizeof(double));
    // change only the non-zero elements:
    Vx_0[0+(NY0) * 0] = 1;
    Vx_0[1+(NY0) * 1] = 1;
    Vx_0[2+(NY0) * 2] = 1;
    Vx_0[3+(NY0) * 3] = 1;
    Vx_0[4+(NY0) * 4] = 1;
    Vx_0[5+(NY0) * 5] = 1;
    Vx_0[6+(NY0) * 6] = 1;
    Vx_0[7+(NY0) * 7] = 1;
    Vx_0[8+(NY0) * 8] = 1;
    Vx_0[9+(NY0) * 9] = 1;
    Vx_0[10+(NY0) * 10] = 1;
    Vx_0[11+(NY0) * 11] = 1;
    Vx_0[12+(NY0) * 12] = 1;
    Vx_0[13+(NY0) * 13] = 1;
    Vx_0[14+(NY0) * 14] = 1;
    Vx_0[15+(NY0) * 15] = 1;
    Vx_0[16+(NY0) * 16] = 1;
    Vx_0[17+(NY0) * 17] = 1;
    Vx_0[18+(NY0) * 18] = 1;
    Vx_0[19+(NY0) * 19] = 1;
    Vx_0[20+(NY0) * 20] = 1;
    Vx_0[21+(NY0) * 21] = 1;
    Vx_0[22+(NY0) * 22] = 1;
    Vx_0[23+(NY0) * 23] = 1;
    Vx_0[24+(NY0) * 24] = 1;
    Vx_0[25+(NY0) * 25] = 1;
    Vx_0[26+(NY0) * 26] = 1;
    Vx_0[27+(NY0) * 27] = 1;
    Vx_0[28+(NY0) * 28] = 1;
    Vx_0[29+(NY0) * 29] = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "Vx", Vx_0);
    free(Vx_0);
    double* Vu_0 = calloc(NY0*NU, sizeof(double));
    // change only the non-zero elements:
    Vu_0[30+(NY0) * 0] = 1;
    Vu_0[31+(NY0) * 1] = 1;
    Vu_0[32+(NY0) * 2] = 1;
    Vu_0[33+(NY0) * 3] = 1;
    Vu_0[34+(NY0) * 4] = 1;
    Vu_0[35+(NY0) * 5] = 1;
    Vu_0[36+(NY0) * 6] = 1;
    Vu_0[37+(NY0) * 7] = 1;
    Vu_0[38+(NY0) * 8] = 1;
    Vu_0[39+(NY0) * 9] = 1;
    Vu_0[40+(NY0) * 10] = 1;
    Vu_0[41+(NY0) * 11] = 1;
    Vu_0[42+(NY0) * 12] = 1;
    Vu_0[43+(NY0) * 13] = 1;
    Vu_0[44+(NY0) * 14] = 1;
    Vu_0[45+(NY0) * 15] = 1;
    Vu_0[46+(NY0) * 16] = 1;
    Vu_0[47+(NY0) * 17] = 1;
    Vu_0[48+(NY0) * 18] = 1;
    Vu_0[49+(NY0) * 19] = 1;
    Vu_0[50+(NY0) * 20] = 1;
    Vu_0[51+(NY0) * 21] = 1;
    Vu_0[52+(NY0) * 22] = 1;
    Vu_0[53+(NY0) * 23] = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "Vu", Vu_0);
    free(Vu_0);
    double* yref = calloc(NY, sizeof(double));
    // change only the non-zero elements:

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
    }
    free(yref);
    double* W = calloc(NY*NY, sizeof(double));
    // change only the non-zero elements:
    W[2+(NY) * 2] = 1500;
    W[3+(NY) * 3] = 200;
    W[4+(NY) * 4] = 200;
    W[5+(NY) * 5] = 200;
    W[6+(NY) * 6] = 500;
    W[7+(NY) * 7] = 500;
    W[9+(NY) * 9] = 20;
    W[10+(NY) * 10] = 20;
    W[11+(NY) * 11] = 50;
    W[12+(NY) * 12] = 300;
    W[13+(NY) * 13] = 300;
    W[14+(NY) * 14] = 300;
    W[15+(NY) * 15] = 300;
    W[16+(NY) * 16] = 300;
    W[17+(NY) * 17] = 300;
    W[18+(NY) * 18] = 300;
    W[19+(NY) * 19] = 300;
    W[20+(NY) * 20] = 300;
    W[21+(NY) * 21] = 300;
    W[22+(NY) * 22] = 300;
    W[23+(NY) * 23] = 300;
    W[24+(NY) * 24] = 50;
    W[25+(NY) * 25] = 10;
    W[26+(NY) * 26] = 10;
    W[27+(NY) * 27] = 10;
    W[28+(NY) * 28] = 10;
    W[29+(NY) * 29] = 10;
    W[30+(NY) * 30] = 0.0001;
    W[31+(NY) * 31] = 0.0001;
    W[32+(NY) * 32] = 0.00001;
    W[33+(NY) * 33] = 0.0001;
    W[34+(NY) * 34] = 0.0001;
    W[35+(NY) * 35] = 0.00001;
    W[36+(NY) * 36] = 0.0001;
    W[37+(NY) * 37] = 0.0001;
    W[38+(NY) * 38] = 0.00001;
    W[39+(NY) * 39] = 0.0001;
    W[40+(NY) * 40] = 0.0001;
    W[41+(NY) * 41] = 0.00001;
    W[42+(NY) * 42] = 0.001;
    W[43+(NY) * 43] = 0.001;
    W[44+(NY) * 44] = 0.001;
    W[45+(NY) * 45] = 0.001;
    W[46+(NY) * 46] = 0.001;
    W[47+(NY) * 47] = 0.001;
    W[48+(NY) * 48] = 0.001;
    W[49+(NY) * 49] = 0.001;
    W[50+(NY) * 50] = 0.001;
    W[51+(NY) * 51] = 0.001;
    W[52+(NY) * 52] = 0.001;
    W[53+(NY) * 53] = 0.001;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W);
    }
    free(W);
    double* Vx = calloc(NY*NX, sizeof(double));
    // change only the non-zero elements:
    Vx[0+(NY) * 0] = 1;
    Vx[1+(NY) * 1] = 1;
    Vx[2+(NY) * 2] = 1;
    Vx[3+(NY) * 3] = 1;
    Vx[4+(NY) * 4] = 1;
    Vx[5+(NY) * 5] = 1;
    Vx[6+(NY) * 6] = 1;
    Vx[7+(NY) * 7] = 1;
    Vx[8+(NY) * 8] = 1;
    Vx[9+(NY) * 9] = 1;
    Vx[10+(NY) * 10] = 1;
    Vx[11+(NY) * 11] = 1;
    Vx[12+(NY) * 12] = 1;
    Vx[13+(NY) * 13] = 1;
    Vx[14+(NY) * 14] = 1;
    Vx[15+(NY) * 15] = 1;
    Vx[16+(NY) * 16] = 1;
    Vx[17+(NY) * 17] = 1;
    Vx[18+(NY) * 18] = 1;
    Vx[19+(NY) * 19] = 1;
    Vx[20+(NY) * 20] = 1;
    Vx[21+(NY) * 21] = 1;
    Vx[22+(NY) * 22] = 1;
    Vx[23+(NY) * 23] = 1;
    Vx[24+(NY) * 24] = 1;
    Vx[25+(NY) * 25] = 1;
    Vx[26+(NY) * 26] = 1;
    Vx[27+(NY) * 27] = 1;
    Vx[28+(NY) * 28] = 1;
    Vx[29+(NY) * 29] = 1;
    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Vx", Vx);
    }
    free(Vx);

    
    double* Vu = calloc(NY*NU, sizeof(double));
    // change only the non-zero elements:
    
    Vu[30+(NY) * 0] = 1;
    Vu[31+(NY) * 1] = 1;
    Vu[32+(NY) * 2] = 1;
    Vu[33+(NY) * 3] = 1;
    Vu[34+(NY) * 4] = 1;
    Vu[35+(NY) * 5] = 1;
    Vu[36+(NY) * 6] = 1;
    Vu[37+(NY) * 7] = 1;
    Vu[38+(NY) * 8] = 1;
    Vu[39+(NY) * 9] = 1;
    Vu[40+(NY) * 10] = 1;
    Vu[41+(NY) * 11] = 1;
    Vu[42+(NY) * 12] = 1;
    Vu[43+(NY) * 13] = 1;
    Vu[44+(NY) * 14] = 1;
    Vu[45+(NY) * 15] = 1;
    Vu[46+(NY) * 16] = 1;
    Vu[47+(NY) * 17] = 1;
    Vu[48+(NY) * 18] = 1;
    Vu[49+(NY) * 19] = 1;
    Vu[50+(NY) * 20] = 1;
    Vu[51+(NY) * 21] = 1;
    Vu[52+(NY) * 22] = 1;
    Vu[53+(NY) * 23] = 1;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Vu", Vu);
    }
    free(Vu);
    double* yref_e = calloc(NYN, sizeof(double));
    // change only the non-zero elements:
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", yref_e);
    free(yref_e);

    double* W_e = calloc(NYN*NYN, sizeof(double));
    // change only the non-zero elements:
    W_e[2+(NYN) * 2] = 1500;
    W_e[3+(NYN) * 3] = 200;
    W_e[4+(NYN) * 4] = 200;
    W_e[5+(NYN) * 5] = 200;
    W_e[6+(NYN) * 6] = 500;
    W_e[7+(NYN) * 7] = 500;
    W_e[9+(NYN) * 9] = 20;
    W_e[10+(NYN) * 10] = 20;
    W_e[11+(NYN) * 11] = 50;
    W_e[12+(NYN) * 12] = 300;
    W_e[13+(NYN) * 13] = 300;
    W_e[14+(NYN) * 14] = 300;
    W_e[15+(NYN) * 15] = 300;
    W_e[16+(NYN) * 16] = 300;
    W_e[17+(NYN) * 17] = 300;
    W_e[18+(NYN) * 18] = 300;
    W_e[19+(NYN) * 19] = 300;
    W_e[20+(NYN) * 20] = 300;
    W_e[21+(NYN) * 21] = 300;
    W_e[22+(NYN) * 22] = 300;
    W_e[23+(NYN) * 23] = 300;
    W_e[24+(NYN) * 24] = 50;
    W_e[25+(NYN) * 25] = 10;
    W_e[26+(NYN) * 26] = 10;
    W_e[27+(NYN) * 27] = 10;
    W_e[28+(NYN) * 28] = 10;
    W_e[29+(NYN) * 29] = 10;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", W_e);
    free(W_e);
    double* Vx_e = calloc(NYN*NX, sizeof(double));
    // change only the non-zero elements:
    
    Vx_e[0+(NYN) * 0] = 1;
    Vx_e[1+(NYN) * 1] = 1;
    Vx_e[2+(NYN) * 2] = 1;
    Vx_e[3+(NYN) * 3] = 1;
    Vx_e[4+(NYN) * 4] = 1;
    Vx_e[5+(NYN) * 5] = 1;
    Vx_e[6+(NYN) * 6] = 1;
    Vx_e[7+(NYN) * 7] = 1;
    Vx_e[8+(NYN) * 8] = 1;
    Vx_e[9+(NYN) * 9] = 1;
    Vx_e[10+(NYN) * 10] = 1;
    Vx_e[11+(NYN) * 11] = 1;
    Vx_e[12+(NYN) * 12] = 1;
    Vx_e[13+(NYN) * 13] = 1;
    Vx_e[14+(NYN) * 14] = 1;
    Vx_e[15+(NYN) * 15] = 1;
    Vx_e[16+(NYN) * 16] = 1;
    Vx_e[17+(NYN) * 17] = 1;
    Vx_e[18+(NYN) * 18] = 1;
    Vx_e[19+(NYN) * 19] = 1;
    Vx_e[20+(NYN) * 20] = 1;
    Vx_e[21+(NYN) * 21] = 1;
    Vx_e[22+(NYN) * 22] = 1;
    Vx_e[23+(NYN) * 23] = 1;
    Vx_e[24+(NYN) * 24] = 1;
    Vx_e[25+(NYN) * 25] = 1;
    Vx_e[26+(NYN) * 26] = 1;
    Vx_e[27+(NYN) * 27] = 1;
    Vx_e[28+(NYN) * 28] = 1;
    Vx_e[29+(NYN) * 29] = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "Vx", Vx_e);
    free(Vx_e);






    /**** Constraints ****/

    // bounds for initial stage
    // x0
    int* idxbx0 = malloc(NBX0 * sizeof(int));
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;
    idxbx0[4] = 4;
    idxbx0[5] = 5;
    idxbx0[6] = 6;
    idxbx0[7] = 7;
    idxbx0[8] = 8;
    idxbx0[9] = 9;
    idxbx0[10] = 10;
    idxbx0[11] = 11;
    idxbx0[12] = 12;
    idxbx0[13] = 13;
    idxbx0[14] = 14;
    idxbx0[15] = 15;
    idxbx0[16] = 16;
    idxbx0[17] = 17;
    idxbx0[18] = 18;
    idxbx0[19] = 19;
    idxbx0[20] = 20;
    idxbx0[21] = 21;
    idxbx0[22] = 22;
    idxbx0[23] = 23;
    idxbx0[24] = 24;
    idxbx0[25] = 25;
    idxbx0[26] = 26;
    idxbx0[27] = 27;
    idxbx0[28] = 28;
    idxbx0[29] = 29;

    double* lubx0 = calloc(2*NBX0, sizeof(double));
    double* lbx0 = lubx0;
    double* ubx0 = lubx0 + NBX0;
    // change only the non-zero elements:

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);
    free(idxbx0);
    free(lubx0);
    // idxbxe_0
    int* idxbxe_0 = malloc(30 * sizeof(int));
    
    idxbxe_0[0] = 0;
    idxbxe_0[1] = 1;
    idxbxe_0[2] = 2;
    idxbxe_0[3] = 3;
    idxbxe_0[4] = 4;
    idxbxe_0[5] = 5;
    idxbxe_0[6] = 6;
    idxbxe_0[7] = 7;
    idxbxe_0[8] = 8;
    idxbxe_0[9] = 9;
    idxbxe_0[10] = 10;
    idxbxe_0[11] = 11;
    idxbxe_0[12] = 12;
    idxbxe_0[13] = 13;
    idxbxe_0[14] = 14;
    idxbxe_0[15] = 15;
    idxbxe_0[16] = 16;
    idxbxe_0[17] = 17;
    idxbxe_0[18] = 18;
    idxbxe_0[19] = 19;
    idxbxe_0[20] = 20;
    idxbxe_0[21] = 21;
    idxbxe_0[22] = 22;
    idxbxe_0[23] = 23;
    idxbxe_0[24] = 24;
    idxbxe_0[25] = 25;
    idxbxe_0[26] = 26;
    idxbxe_0[27] = 27;
    idxbxe_0[28] = 28;
    idxbxe_0[29] = 29;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbxe", idxbxe_0);
    free(idxbxe_0);



    // set up nonlinear constraints for last stage
    double* luh_0 = calloc(2*NH0, sizeof(double));
    double* lh_0 = luh_0;
    double* uh_0 = luh_0 + NH0;
    
    lh_0[0] = -10000000;
    lh_0[1] = -10000000;
    lh_0[5] = -10000000;
    lh_0[6] = -10000000;
    lh_0[10] = -10000000;
    lh_0[11] = -10000000;
    lh_0[15] = -10000000;
    lh_0[16] = -10000000;

    
    uh_0[2] = 10000000;
    uh_0[3] = 10000000;
    uh_0[4] = 241.68897;
    uh_0[7] = 10000000;
    uh_0[8] = 10000000;
    uh_0[9] = 241.68897;
    uh_0[12] = 10000000;
    uh_0[13] = 10000000;
    uh_0[14] = 241.68897;
    uh_0[17] = 10000000;
    uh_0[18] = 10000000;
    uh_0[19] = 241.68897;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "nl_constr_h_fun_jac", &capsule->nl_constr_h_0_fun_jac);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "nl_constr_h_fun", &capsule->nl_constr_h_0_fun);
    
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lh", lh_0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "uh", uh_0);
    free(luh_0);





    /* constraints that are the same for initial and intermediate */












    // set up nonlinear constraints for stage 1 to N-1
    double* luh = calloc(2*NH, sizeof(double));
    double* lh = luh;
    double* uh = luh + NH;

    
    lh[0] = -10000000;
    lh[1] = -10000000;
    lh[5] = -10000000;
    lh[6] = -10000000;
    lh[10] = -10000000;
    lh[11] = -10000000;
    lh[15] = -10000000;
    lh[16] = -10000000;

    
    uh[2] = 10000000;
    uh[3] = 10000000;
    uh[4] = 241.68897;
    uh[7] = 10000000;
    uh[8] = 10000000;
    uh[9] = 241.68897;
    uh[12] = 10000000;
    uh[13] = 10000000;
    uh[14] = 241.68897;
    uh[17] = 10000000;
    uh[18] = 10000000;
    uh[19] = 241.68897;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "nl_constr_h_fun_jac",
                                      &capsule->nl_constr_h_fun_jac[i-1]);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "nl_constr_h_fun",
                                      &capsule->nl_constr_h_fun[i-1]);
        
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lh", lh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "uh", uh);
    }
    free(luh);



    /* terminal constraints */













}


/**
 * Internal function for centroidal_model_acados_create: step 6
 */
void centroidal_model_acados_create_6_set_opts(centroidal_model_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    void *nlp_opts = capsule->nlp_opts;

    /************************************************
    *  opts
    ************************************************/

int fixed_hess = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "fixed_hess", &fixed_hess);
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "globalization", "fixed_step");int with_solution_sens_wrt_params = false;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_solution_sens_wrt_params", &with_solution_sens_wrt_params);

    int with_value_sens_wrt_params = false;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_value_sens_wrt_params", &with_value_sens_wrt_params);

    int full_step_dual = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "full_step_dual", &full_step_dual);

    // set collocation type (relevant for implicit integrators)
    sim_collocation_type collocation_type = GAUSS_LEGENDRE;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_collocation_type", &collocation_type);

    // set up sim_method_num_steps
    // all sim_method_num_steps are identical
    int sim_method_num_steps = 1;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_num_steps", &sim_method_num_steps);

    // set up sim_method_num_stages
    // all sim_method_num_stages are identical
    int sim_method_num_stages = 4;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_num_stages", &sim_method_num_stages);

    int newton_iter_val = 3;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_newton_iter", &newton_iter_val);

    // set up sim_method_jac_reuse
    bool tmp_bool = (bool) 0;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_jac_reuse", &tmp_bool);

    double nlp_solver_step_length = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "step_length", &nlp_solver_step_length);

    double levenberg_marquardt = 0.001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "levenberg_marquardt", &levenberg_marquardt);

    /* options QP solver */
    int qp_solver_cond_N;const int qp_solver_cond_N_ori = 12;
    qp_solver_cond_N = N < qp_solver_cond_N_ori ? N : qp_solver_cond_N_ori; // use the minimum value here
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_N", &qp_solver_cond_N);

    int nlp_solver_ext_qp_res = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "ext_qp_res", &nlp_solver_ext_qp_res);
    int log_primal_step_norm = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "log_primal_step_norm", &log_primal_step_norm);
    // set HPIPM mode: should be done before setting other QP solver options
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_hpipm_mode", "BALANCE");


    // set SQP specific options
    double nlp_solver_tol_stat = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_stat", &nlp_solver_tol_stat);

    double nlp_solver_tol_eq = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_eq", &nlp_solver_tol_eq);

    double nlp_solver_tol_ineq = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_ineq", &nlp_solver_tol_ineq);

    double nlp_solver_tol_comp = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_comp", &nlp_solver_tol_comp);

    int nlp_solver_max_iter = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "max_iter", &nlp_solver_max_iter);

    int initialize_t_slacks = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "initialize_t_slacks", &initialize_t_slacks);

    int qp_solver_iter_max = 50;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_iter_max", &qp_solver_iter_max);



    int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "print_level", &print_level);
    int qp_solver_cond_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_ric_alg", &qp_solver_cond_ric_alg);

    int qp_solver_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_ric_alg", &qp_solver_ric_alg);


    int ext_cost_num_hess = 0;



}


/**
 * Internal function for centroidal_model_acados_create: step 7
 */
void centroidal_model_acados_create_7_set_nlp_out(centroidal_model_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;

    // initialize primal solution
    double* xu0 = calloc(NX+NU, sizeof(double));
    double* x0 = xu0;

    // initialize with x0
    


    double* u0 = xu0 + NX;

    for (int i = 0; i < N; i++)
    {
        // x0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x0);
        // u0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x0);
    free(xu0);
}


/**
 * Internal function for centroidal_model_acados_create: step 8
 */
//void centroidal_model_acados_create_8_create_solver(centroidal_model_solver_capsule* capsule)
//{
//    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts);
//}

/**
 * Internal function for centroidal_model_acados_create: step 9
 */
int centroidal_model_acados_create_9_precompute(centroidal_model_solver_capsule* capsule) {
    int status = ocp_nlp_precompute(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    if (status != ACADOS_SUCCESS) {
        printf("\nocp_nlp_precompute failed!\n\n");
        exit(1);
    }

    return status;
}


int centroidal_model_acados_create_with_discretization(centroidal_model_solver_capsule* capsule, int N, double* new_time_steps)
{
    // If N does not match the number of shooting intervals used for code generation, new_time_steps must be given.
    if (N != CENTROIDAL_MODEL_N && !new_time_steps) {
        fprintf(stderr, "centroidal_model_acados_create_with_discretization: new_time_steps is NULL " \
            "but the number of shooting intervals (= %d) differs from the number of " \
            "shooting intervals (= %d) during code generation! Please provide a new vector of time_stamps!\n", \
             N, CENTROIDAL_MODEL_N);
        return 1;
    }

    // number of expected runtime parameters
    capsule->nlp_np = NP;

    // 1) create and set nlp_solver_plan; create nlp_config
    capsule->nlp_solver_plan = ocp_nlp_plan_create(N);
    centroidal_model_acados_create_1_set_plan(capsule->nlp_solver_plan, N);
    capsule->nlp_config = ocp_nlp_config_create(*capsule->nlp_solver_plan);

    // 3) create and set dimensions
    capsule->nlp_dims = centroidal_model_acados_create_2_create_and_set_dimensions(capsule);
    centroidal_model_acados_create_3_create_and_set_functions(capsule);

    // 4) set default parameters in functions
    centroidal_model_acados_create_4_set_default_parameters(capsule);

    // 5) create and set nlp_in
    capsule->nlp_in = ocp_nlp_in_create(capsule->nlp_config, capsule->nlp_dims);
    centroidal_model_acados_create_5_set_nlp_in(capsule, N, new_time_steps);

    // 6) create and set nlp_opts
    capsule->nlp_opts = ocp_nlp_solver_opts_create(capsule->nlp_config, capsule->nlp_dims);
    centroidal_model_acados_create_6_set_opts(capsule);

    // 7) create and set nlp_out
    // 7.1) nlp_out
    capsule->nlp_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    // 7.2) sens_out
    capsule->sens_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    centroidal_model_acados_create_7_set_nlp_out(capsule);

    // 8) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts);
    //centroidal_model_acados_create_8_create_solver(capsule);

    // 9) do precomputations
    int status = centroidal_model_acados_create_9_precompute(capsule);

    return status;
}

/**
 * This function is for updating an already initialized solver with a different number of qp_cond_N. It is useful for code reuse after code export.
 */
int centroidal_model_acados_update_qp_solver_cond_N(centroidal_model_solver_capsule* capsule, int qp_solver_cond_N)
{
    // 1) destroy solver
    ocp_nlp_solver_destroy(capsule->nlp_solver);

    // 2) set new value for "qp_cond_N"
    const int N = capsule->nlp_solver_plan->N;
    if(qp_solver_cond_N > N)
        printf("Warning: qp_solver_cond_N = %d > N = %d\n", qp_solver_cond_N, N);
    ocp_nlp_solver_opts_set(capsule->nlp_config, capsule->nlp_opts, "qp_cond_N", &qp_solver_cond_N);

    // 3) continue with the remaining steps from centroidal_model_acados_create_with_discretization(...):
    // -> 8) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts);

    // -> 9) do precomputations
    int status = centroidal_model_acados_create_9_precompute(capsule);
    return status;
}


int centroidal_model_acados_reset(centroidal_model_solver_capsule* capsule, int reset_qp_solver_mem)
{

    // set initialization to all zeros

    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;
    ocp_nlp_in* nlp_in = capsule->nlp_in;
    ocp_nlp_solver* nlp_solver = capsule->nlp_solver;

    double* buffer = calloc(NX+NU+NZ+2*NS+2*NSN+2*NS0+NBX+NBU+NG+NH+NPHI+NBX0+NBXN+NHN+NH0+NPHIN+NGN, sizeof(double));

    for(int i=0; i<N+1; i++)
    {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "sl", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "su", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "lam", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "t", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "z", buffer);
        if (i<N)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "pi", buffer);
        }
    }
    // get qp_status: if NaN -> reset memory
    int qp_status;
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "qp_status", &qp_status);
    if (reset_qp_solver_mem || (qp_status == 3))
    {
        // printf("\nin reset qp_status %d -> resetting QP memory\n", qp_status);
        ocp_nlp_solver_reset_qp_memory(nlp_solver, nlp_in, nlp_out);
    }

    free(buffer);
    return 0;
}




int centroidal_model_acados_update_params(centroidal_model_solver_capsule* capsule, int stage, double *p, int np)
{
    int solver_status = 0;

    int casadi_np = 29;
    if (casadi_np != np) {
        printf("acados_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }

    const int N = capsule->nlp_solver_plan->N;
    if (stage < N && stage >= 0)
    {
        capsule->forw_vde_casadi[stage].set_param(capsule->forw_vde_casadi+stage, p);
        capsule->expl_ode_fun[stage].set_param(capsule->expl_ode_fun+stage, p);

        // constraints
        if (stage == 0)
        {
            capsule->nl_constr_h_0_fun_jac.set_param(&capsule->nl_constr_h_0_fun_jac, p);
            capsule->nl_constr_h_0_fun.set_param(&capsule->nl_constr_h_0_fun, p);
        }
        else
        {
            capsule->nl_constr_h_fun_jac[stage-1].set_param(capsule->nl_constr_h_fun_jac+stage-1, p);
            capsule->nl_constr_h_fun[stage-1].set_param(capsule->nl_constr_h_fun+stage-1, p);
        }

        // cost
        if (stage == 0)
        {
        }
        else // 0 < stage < N
        {
        }
    }

    else // stage == N
    {
        // terminal shooting node has no dynamics
        // cost
        // constraints
    }

    return solver_status;
}


int centroidal_model_acados_update_params_sparse(centroidal_model_solver_capsule * capsule, int stage, int *idx, double *p, int n_update)
{
    int solver_status = 0;

    int casadi_np = 29;
    if (casadi_np < n_update) {
        printf("centroidal_model_acados_update_params_sparse: trying to set %d parameters for external functions."
            " External function has %d parameters. Exiting.\n", n_update, casadi_np);
        exit(1);
    }
    // for (int i = 0; i < n_update; i++)
    // {
    //     if (idx[i] > casadi_np) {
    //         printf("centroidal_model_acados_update_params_sparse: attempt to set parameters with index %d, while"
    //             " external functions only has %d parameters. Exiting.\n", idx[i], casadi_np);
    //         exit(1);
    //     }
    //     printf("param %d value %e\n", idx[i], p[i]);
    // }
    const int N = capsule->nlp_solver_plan->N;
    if (stage < N && stage >= 0)
    {
        capsule->forw_vde_casadi[stage].set_param_sparse(capsule->forw_vde_casadi+stage, n_update, idx, p);
        capsule->expl_ode_fun[stage].set_param_sparse(capsule->expl_ode_fun+stage, n_update, idx, p);

        // constraints
        if (stage == 0)
        {
            capsule->nl_constr_h_0_fun_jac.set_param_sparse(&capsule->nl_constr_h_0_fun_jac, n_update, idx, p);
            capsule->nl_constr_h_0_fun.set_param_sparse(&capsule->nl_constr_h_0_fun, n_update, idx, p);
        }
        else
        {
            capsule->nl_constr_h_fun_jac[stage-1].set_param_sparse(capsule->nl_constr_h_fun_jac+stage-1, n_update, idx, p);
            capsule->nl_constr_h_fun[stage-1].set_param_sparse(capsule->nl_constr_h_fun+stage-1, n_update, idx, p);
        }

        // cost
        if (stage == 0)
        {
        }
        else // 0 < stage < N
        {
        }
    }

    else // stage == N
    {
        // terminal shooting node has no dynamics
        // cost
        // constraints
    }


    return solver_status;
}

int centroidal_model_acados_solve(centroidal_model_solver_capsule* capsule)
{
    // solve NLP
    int solver_status = ocp_nlp_solve(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}


void centroidal_model_acados_batch_solve(centroidal_model_solver_capsule ** capsules, int N_batch)
{

    int num_threads_bkp = omp_get_num_threads();
    omp_set_num_threads(3);

    #pragma omp parallel for
    for (int i = 0; i < N_batch; i++)
    {
        ocp_nlp_solve(capsules[i]->nlp_solver, capsules[i]->nlp_in, capsules[i]->nlp_out);
    }


    omp_set_num_threads( num_threads_bkp );
    return;
}


int centroidal_model_acados_free(centroidal_model_solver_capsule* capsule)
{
    // before destroying, keep some info
    const int N = capsule->nlp_solver_plan->N;
    // free memory
    ocp_nlp_solver_opts_destroy(capsule->nlp_opts);
    ocp_nlp_in_destroy(capsule->nlp_in);
    ocp_nlp_out_destroy(capsule->nlp_out);
    ocp_nlp_out_destroy(capsule->sens_out);
    ocp_nlp_solver_destroy(capsule->nlp_solver);
    ocp_nlp_dims_destroy(capsule->nlp_dims);
    ocp_nlp_config_destroy(capsule->nlp_config);
    ocp_nlp_plan_destroy(capsule->nlp_solver_plan);

    /* free external function */
    // dynamics
    for (int i = 0; i < N; i++)
    {
        external_function_param_casadi_free(&capsule->forw_vde_casadi[i]);
        external_function_param_casadi_free(&capsule->expl_ode_fun[i]);
    }
    free(capsule->forw_vde_casadi);
    free(capsule->expl_ode_fun);

    // cost

    // constraints
    for (int i = 0; i < N-1; i++)
    {
        external_function_param_casadi_free(&capsule->nl_constr_h_fun_jac[i]);
        external_function_param_casadi_free(&capsule->nl_constr_h_fun[i]);
    }
    free(capsule->nl_constr_h_fun_jac);
    free(capsule->nl_constr_h_fun);
    external_function_param_casadi_free(&capsule->nl_constr_h_0_fun_jac);
    external_function_param_casadi_free(&capsule->nl_constr_h_0_fun);

    return 0;
}


void centroidal_model_acados_print_stats(centroidal_model_solver_capsule* capsule)
{
    int nlp_iter, stat_m, stat_n, tmp_int;
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "nlp_iter", &nlp_iter);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_n", &stat_n);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_m", &stat_m);

    
    double stat[12];
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "statistics", stat);

    int nrow = nlp_iter+1 < stat_m ? nlp_iter+1 : stat_m;


    printf("iter\tres_stat\tres_eq\t\tres_ineq\tres_comp\tqp_stat\tqp_iter\talpha");
    if (stat_n > 8)
        printf("\t\tqp_res_stat\tqp_res_eq\tqp_res_ineq\tqp_res_comp");
    printf("\n");
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < stat_n + 1; j++)
        {
            if (j == 0 || j == 5 || j == 6)
            {
                tmp_int = (int) stat[i + j * nrow];
                printf("%d\t", tmp_int);
            }
            else
            {
                printf("%e\t", stat[i + j * nrow]);
            }
        }
        printf("\n");
    }
}

int centroidal_model_acados_custom_update(centroidal_model_solver_capsule* capsule, double* data, int data_len)
{
    (void)capsule;
    (void)data;
    (void)data_len;
    printf("\ndummy function that can be called in between solver calls to update parameters or numerical data efficiently in C.\n");
    printf("nothing set yet..\n");
    return 1;

}



ocp_nlp_in *centroidal_model_acados_get_nlp_in(centroidal_model_solver_capsule* capsule) { return capsule->nlp_in; }
ocp_nlp_out *centroidal_model_acados_get_nlp_out(centroidal_model_solver_capsule* capsule) { return capsule->nlp_out; }
ocp_nlp_out *centroidal_model_acados_get_sens_out(centroidal_model_solver_capsule* capsule) { return capsule->sens_out; }
ocp_nlp_solver *centroidal_model_acados_get_nlp_solver(centroidal_model_solver_capsule* capsule) { return capsule->nlp_solver; }
ocp_nlp_config *centroidal_model_acados_get_nlp_config(centroidal_model_solver_capsule* capsule) { return capsule->nlp_config; }
void *centroidal_model_acados_get_nlp_opts(centroidal_model_solver_capsule* capsule) { return capsule->nlp_opts; }
ocp_nlp_dims *centroidal_model_acados_get_nlp_dims(centroidal_model_solver_capsule* capsule) { return capsule->nlp_dims; }
ocp_nlp_plan_t *centroidal_model_acados_get_nlp_plan(centroidal_model_solver_capsule* capsule) { return capsule->nlp_solver_plan; }
