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
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/sim_interface.h"
#include "acados_sim_solver_centroidal_model.h"

#define NX     CENTROIDAL_MODEL_NX
#define NZ     CENTROIDAL_MODEL_NZ
#define NU     CENTROIDAL_MODEL_NU
#define NP     CENTROIDAL_MODEL_NP


int main()
{
    int status = 0;
    centroidal_model_sim_solver_capsule *capsule = centroidal_model_acados_sim_solver_create_capsule();
    status = centroidal_model_acados_sim_create(capsule);

    if (status)
    {
        printf("acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    sim_config *acados_sim_config = centroidal_model_acados_get_sim_config(capsule);
    sim_in *acados_sim_in = centroidal_model_acados_get_sim_in(capsule);
    sim_out *acados_sim_out = centroidal_model_acados_get_sim_out(capsule);
    void *acados_sim_dims = centroidal_model_acados_get_sim_dims(capsule);

    // initial condition
    double x_current[NX];
    x_current[0] = 0.0;
    x_current[1] = 0.0;
    x_current[2] = 0.0;
    x_current[3] = 0.0;
    x_current[4] = 0.0;
    x_current[5] = 0.0;
    x_current[6] = 0.0;
    x_current[7] = 0.0;
    x_current[8] = 0.0;
    x_current[9] = 0.0;
    x_current[10] = 0.0;
    x_current[11] = 0.0;
    x_current[12] = 0.0;
    x_current[13] = 0.0;
    x_current[14] = 0.0;
    x_current[15] = 0.0;
    x_current[16] = 0.0;
    x_current[17] = 0.0;
    x_current[18] = 0.0;
    x_current[19] = 0.0;
    x_current[20] = 0.0;
    x_current[21] = 0.0;
    x_current[22] = 0.0;
    x_current[23] = 0.0;
    x_current[24] = 0.0;
    x_current[25] = 0.0;
    x_current[26] = 0.0;
    x_current[27] = 0.0;
    x_current[28] = 0.0;
    x_current[29] = 0.0;

  
    x_current[0] = 0;
    x_current[1] = 0;
    x_current[2] = 0;
    x_current[3] = 0;
    x_current[4] = 0;
    x_current[5] = 0;
    x_current[6] = 0;
    x_current[7] = 0;
    x_current[8] = 0;
    x_current[9] = 0;
    x_current[10] = 0;
    x_current[11] = 0;
    x_current[12] = 0;
    x_current[13] = 0;
    x_current[14] = 0;
    x_current[15] = 0;
    x_current[16] = 0;
    x_current[17] = 0;
    x_current[18] = 0;
    x_current[19] = 0;
    x_current[20] = 0;
    x_current[21] = 0;
    x_current[22] = 0;
    x_current[23] = 0;
    x_current[24] = 0;
    x_current[25] = 0;
    x_current[26] = 0;
    x_current[27] = 0;
    x_current[28] = 0;
    x_current[29] = 0;
    
  


    // initial value for control input
    double u0[NU];
    u0[0] = 0.0;
    u0[1] = 0.0;
    u0[2] = 0.0;
    u0[3] = 0.0;
    u0[4] = 0.0;
    u0[5] = 0.0;
    u0[6] = 0.0;
    u0[7] = 0.0;
    u0[8] = 0.0;
    u0[9] = 0.0;
    u0[10] = 0.0;
    u0[11] = 0.0;
    u0[12] = 0.0;
    u0[13] = 0.0;
    u0[14] = 0.0;
    u0[15] = 0.0;
    u0[16] = 0.0;
    u0[17] = 0.0;
    u0[18] = 0.0;
    u0[19] = 0.0;
    u0[20] = 0.0;
    u0[21] = 0.0;
    u0[22] = 0.0;
    u0[23] = 0.0;
    // set parameters
    double p[NP];
    p[0] = 1;
    p[1] = 1;
    p[2] = 1;
    p[3] = 1;
    p[4] = 0.5;
    p[5] = 0;
    p[6] = 0;
    p[7] = 0;
    p[8] = 0;
    p[9] = 0;
    p[10] = 0;
    p[11] = 0;
    p[12] = 0;
    p[13] = 0;
    p[14] = 0;
    p[15] = 0;
    p[16] = 0;
    p[17] = 0;
    p[18] = 0;
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

    centroidal_model_acados_sim_update_params(capsule, p, NP);
  

  


    int n_sim_steps = 3;
    // solve ocp in loop
    for (int ii = 0; ii < n_sim_steps; ii++)
    {
        // set inputs
        sim_in_set(acados_sim_config, acados_sim_dims,
            acados_sim_in, "x", x_current);
        sim_in_set(acados_sim_config, acados_sim_dims,
            acados_sim_in, "u", u0);

        // solve
        status = centroidal_model_acados_sim_solve(capsule);
        if (status != ACADOS_SUCCESS)
        {
            printf("acados_solve() failed with status %d.\n", status);
        }

        // get outputs
        sim_out_get(acados_sim_config, acados_sim_dims,
               acados_sim_out, "x", x_current);

    

        // print solution
        printf("\nx_current, %d\n", ii);
        for (int jj = 0; jj < NX; jj++)
        {
            printf("%e\n", x_current[jj]);
        }
    }

    printf("\nPerformed %d simulation steps with acados integrator successfully.\n\n", n_sim_steps);

    // free solver
    status = centroidal_model_acados_sim_free(capsule);
    if (status) {
        printf("centroidal_model_acados_sim_free() returned status %d. \n", status);
    }

    centroidal_model_acados_sim_solver_free_capsule(capsule);

    return status;
}
