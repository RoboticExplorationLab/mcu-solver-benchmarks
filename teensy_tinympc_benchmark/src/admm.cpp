#include <iostream>

#include "admm.hpp"

#include <admm.hpp>
#include "Arduino.h"
// #include "problem_data/quadrotor_50hz_params.hpp"
// #include "problem_data/quadrotor_50hz_params_3.hpp"
// #include "problem_data/quadrotor_50hz_params_unconstrained.hpp"
// #include "problem_data/quadrotor_50hz_params_constrained.hpp"
// #include "trajectory_data/quadrotor_50hz_line_5s.hpp"
// #include "trajectory_data/quadrotor_50hz_line_9s.hpp"
using Eigen::Matrix;

#define DEBUG_MODULE "TINYALG"

extern "C" {

// #include "debug.h"

static uint64_t startTimestamp;

void solve_lqr(struct tiny_problem *problem, const struct tiny_params *params) {
    problem->u.col(0) = -params->cache.Kinf[problem->cache_level] * (problem->x.col(0) - params->Xref.col(0));
}


void solve_admm(struct tiny_problem *problem, const struct tiny_params *params) {

    // for (int i=NHORIZON-2; i>=0; i--) {
    //     // problem->Qu.noalias() = params->cache.Bdyn.transpose().lazyProduct(problem->p.col(i+1));
    //     // problem->Qu += problem->r.col(i);
    //     // (problem->d.col(i)).noalias() = params->cache.Quu_inv.lazyProduct(problem->Qu);
        
    //     // (problem->d.col(i)).noalias() = params->cache.Quu_inv[problem->cache_level] * (params->cache.Bdyn[problem->cache_level].transpose() * problem->p.col(i+1) + problem->r.col(i));
    //     // (problem->p.col(i)).noalias() = problem->q.col(i) + params->cache.AmBKt[problem->cache_level].lazyProduct(problem->p.col(i+1)) - (params->cache.Kinf[problem->cache_level].transpose()).lazyProduct(problem->r.col(i)) + params->cache.coeff_d2p[problem->cache_level] * problem->d.col(i);
    //     std::cout << "Quu: " << params->cache.Quu_inv[problem->cache_level] << std::endl;
    //     std::cout << "p: " << problem->p.col(i+1) << std::endl;
    //     std::cout << "r: " << problem->r.col(i) << std:: endl;
    //     // (problem->d.col(i)).noalias() = params->cache.Quu_inv[problem->cache_level] * (params->cache.Bdyn[problem->cache_level].transpose() * problem->p.col(i+1) + problem->r.col(i));
    //     // (problem->p.col(i)).noalias() = problem->q.col(i) + params->cache.AmBKt[problem->cache_level].lazyProduct(problem->p.col(i+1)) - (params->cache.Kinf[problem->cache_level].transpose()).lazyProduct(problem->r.col(i)); // + params->cache.coeff_d2p * problem->d.col(i); // coeff_d2p always appears to be zeros
    // }
    

    problem->status = 0;
    problem->iter = 0;

    forward_pass(problem, params);
    update_slack(problem, params);
    update_dual(problem, params);
    update_linear_cost(problem, params);
    // std::cout << problem->r << std::endl;
    // std::cout << problem->q << std::endl;
    // std::cout << problem->p << std::endl;
    for (int i=0; i<problem->max_iter; i++) {
    // for (int i=0; i<1; i++) {
    // while (usecTimestamp() + 300 < maxTime) {

        // Solve linear system with Riccati and roll out to get new trajectory
        update_primal(problem, params);
        // backward_pass_grad(problem, params);
        // std::cout << problem->d << std::endl;
        // std::cout << problem->p << std::endl;
        // forward_pass(problem, params);
        // std::cout << problem->u << std::endl;
        // std::cout << problem->x << std::endl;

        // Project slack variables into feasible domain
        update_slack(problem, params);

        // Compute next iteration of dual variables
        update_dual(problem, params);

        // Update linear control cost terms using reference trajectory, duals, and slack variables
        update_linear_cost(problem, params);

        problem->primal_residual_state = (problem->x - problem->vnew).cwiseAbs().maxCoeff();
        problem->dual_residual_state = ((problem->v - problem->vnew).cwiseAbs().maxCoeff()) * params->cache.rho[problem->cache_level];
        problem->primal_residual_input = (problem->u - problem->znew).cwiseAbs().maxCoeff();
        problem->dual_residual_input = ((problem->z - problem->znew).cwiseAbs().maxCoeff()) * params->cache.rho[problem->cache_level];

        // TODO: convert arrays of Eigen vectors into one Eigen matrix
        // Save previous slack variables
        problem->v = problem->vnew;
        problem->z = problem->znew;

        problem->iter += 1;

        // TODO: remove convergence check and just return when allotted runtime is up
        // Check for convergence
        if (problem->primal_residual_state < problem->abs_tol &&
            problem->primal_residual_input < problem->abs_tol &&
            problem->dual_residual_state < problem->abs_tol &&
            problem->dual_residual_input < problem->abs_tol)
        {
            problem->status = 1;
            break;
        }

        // TODO: add rho scaling


        // std::cout << problem->primal_residual_state << std::endl;
        // std::cout << problem->dual_residual_state << std::endl;
        // std::cout << problem->primal_residual_input << std::endl;
        // std::cout << problem->dual_residual_input << "\n" << std::endl;
    }
}

/**
 * Do backward Riccati pass then forward roll out
*/
void update_primal(struct tiny_problem *problem, const struct tiny_params *params) {
    backward_pass_grad(problem, params);
    forward_pass(problem, params);
}

/**
 * Update linear terms from Riccati backward pass
*/
void backward_pass_grad(struct tiny_problem *problem, const struct tiny_params *params) {
    for (int i=NHORIZON-2; i>=0; i--) {
        // problem->Qu.noalias() = params->cache.Bdyn.transpose().lazyProduct(problem->p.col(i+1));
        // problem->Qu += problem->r.col(i);
        // (problem->d.col(i)).noalias() = params->cache.Quu_inv.lazyProduct(problem->Qu);
        
        // (problem->d.col(i)).noalias() = params->cache.Quu_inv[problem->cache_level] * (params->cache.Bdyn[problem->cache_level].transpose() * problem->p.col(i+1) + problem->r.col(i));
        // (problem->p.col(i)).noalias() = problem->q.col(i) + params->cache.AmBKt[problem->cache_level].lazyProduct(problem->p.col(i+1)) - (params->cache.Kinf[problem->cache_level].transpose()).lazyProduct(problem->r.col(i)) + params->cache.coeff_d2p[problem->cache_level] * problem->d.col(i);
        
        (problem->d.col(i)).noalias() = params->cache.Quu_inv[problem->cache_level] * (params->cache.Bdyn[problem->cache_level].transpose() * problem->p.col(i+1) + problem->r.col(i));
        (problem->p.col(i)).noalias() = problem->q.col(i) + params->cache.AmBKt[problem->cache_level].lazyProduct(problem->p.col(i+1)) - (params->cache.Kinf[problem->cache_level].transpose()).lazyProduct(problem->r.col(i)); // + params->cache.coeff_d2p * problem->d.col(i); // coeff_d2p always appears to be zeros
    }
}

/**
 * Use LQR feedback policy to roll out trajectory
*/
void forward_pass(struct tiny_problem *problem, const struct tiny_params *params) {
    for (int i=0; i<NHORIZON-1; i++) {
        (problem->u.col(i)).noalias() = -params->cache.Kinf[problem->cache_level].lazyProduct(problem->x.col(i)) - problem->d.col(i);
        // problem->u.col(i) << .001, .02, .3, 4;
        // DEBUG_PRINT("u(0): %f\n", problem->u.col(0)(0));
        // multAdyn(problem->Ax, params->cache.Adyn[problem->cache_level], problem->x.col(i));
        (problem->x.col(i+1)).noalias() = params->cache.Adyn[problem->cache_level].lazyProduct(problem->x.col(i)) + params->cache.Bdyn[problem->cache_level].lazyProduct(problem->u.col(i));
        // (problem->x.col(i+1)).noalias() = params->cache.Adyn.lazyProduct(problem->x.col(i)) + params->cache.Bdyn.lazyProduct(problem->u.col(i));
    }
}

/**
 * Project slack (auxiliary) variables into their feasible domain, defined by
 * projection functions related to each constraint
 * TODO: pass in meta information with each constraint assigning it to a
 * projection function
*/
void update_slack(struct tiny_problem *problem, const struct tiny_params *params) {
    // Box constraints on input
    // Get current time

    problem->znew = params->u_max.cwiseMin(params->u_min.cwiseMax(problem->u + problem->y));
    problem->vnew = params->x_max.cwiseMin(params->x_min.cwiseMax(problem->x + problem->g));

    // Half space constraints on state
    // TODO: support multiple half plane constraints per knot point
    //      currently this only works for one constraint per knot point
    // TODO: can potentially take advantage of the fact that A_constraints[3:end] is zero and just do
    //      v.col(i) = x.col(i) - dist*A_constraints[i] since we have to copy x[3:end] into v anyway
    //      downside is it's not clear this is happening externally and so values of A_constraints
    //      not set to zero (other than the first three) can cause the algorithm to fail
    // TODO: the only state values changing here are the first three (x, y, z) so it doesn't make sense
    //      to do operations on the remaining 9 when projecting (or doing anything related to the dual
    //      or auxiliary variables). v and g could be of size (3) and everything would work the same.
    //      The only reason this doesn't break is because in the update_linear_cost function subtracts
    //      g from v and so the last nine entries are always zero.
    // problem->xg = problem->x + problem->g;
    // // problem->dists = (params->A_constraints.transpose().cwiseProduct(problem->xg)).colwise().sum();
    // // problem->dists -= params->x_max;
    // // startTimestamp = usecTimestamp();
    // problem->cache_level = 0;
    // for (int i=0; i<NHORIZON; i++) {
    //     problem->dist = (params->A_constraints[i].head(3)).lazyProduct(problem->xg.col(i).head(3)); // Distances can be computed in one step outside the for loop
    //     problem->dist -= params->x_max[i](0);
    //     problem->xyz_news.col(i) = problem->xg.col(i).head(3) - problem->dist*params->A_constraints[i].head(3).transpose();
    //     // DEBUG_PRINT("dist: %f\n", dist);
    //     if (problem->dist <= 0) {
    //         problem->vnew.col(i) = problem->xg.col(i);
    //     }
    //     else {
    //         problem->cache_level = 1; // Constraint violated, use second cache level
    //         problem->xyz_new = problem->xg.col(i).head(3) - problem->dist*params->A_constraints[i].head(3).transpose();
    //         problem->vnew.col(i) << problem->xyz_new, problem->xg.col(i).tail(NSTATES-3);
    //     }
    // }
    // problem->vnew = problem->xg;
    // DEBUG_PRINT("s: %d\n", usecTimestamp() - startTimestamp);
}

/**
 * Update next iteration of dual variables by performing the augmented
 * lagrangian multiplier update
*/
void update_dual(struct tiny_problem *problem, const struct tiny_params *params) {
    problem->y = problem->y + problem->u - problem->znew;
    problem->g = problem->g + problem->x - problem->vnew;
}

/**
 * Update linear control cost terms in the Riccati feedback using the changing
 * slack and dual variables from ADMM
*/
void update_linear_cost(struct tiny_problem *problem, const struct tiny_params *params) {
    // problem->r = -(params->Uref.array().colwise() * params->R[problem->cache_level].array()); // Uref = 0 so commented out for speed up. Need to uncomment if using Uref
    problem->r = -params->cache.rho[problem->cache_level] * (problem->znew - problem->y);
    problem->q = -(params->Xref.array().colwise() * params->Q[problem->cache_level].array());
    problem->q -= params->cache.rho[problem->cache_level] * (problem->vnew - problem->g);
    problem->p.col(NHORIZON-1) = -(params->Xref.col(NHORIZON-1).transpose().lazyProduct(params->cache.Pinf[problem->cache_level]));
    problem->p.col(NHORIZON-1) -= params->cache.rho[problem->cache_level] * (problem->vnew.col(NHORIZON-1) - problem->g.col(NHORIZON-1));
}


// void solve_admm(struct tiny_problem *problem, const struct tiny_params *params) {

//     problem->status = 0;
//     problem->iter = 1;

//     forward_pass(problem, params);
//     update_slack(problem, params);
//     update_dual(problem, params);
//     update_linear_cost(problem, params);
//     for (int i=0; i<problem->max_iter; i++) {

//         // Solve linear system with Riccati and roll out to get new trajectory
//         update_primal(problem, params);

//         // Project slack variables into feasible domain
//         update_slack(problem, params);

//         // Compute next iteration of dual variables
//         update_dual(problem, params);

//         // Update linear control cost terms using reference trajectory, duals, and slack variables
//         update_linear_cost(problem, params);

//         problem->primal_residual_state = (problem->x - problem->vnew).cwiseAbs().maxCoeff();
//         problem->dual_residual_state = ((problem->v - problem->vnew).cwiseAbs().maxCoeff()) * params->cache.rho;
//         problem->primal_residual_input = (problem->u - problem->znew).cwiseAbs().maxCoeff();
//         problem->dual_residual_input = ((problem->z - problem->znew).cwiseAbs().maxCoeff()) * params->cache.rho;

//         // TODO: convert arrays of Eigen vectors into one Eigen matrix
//         // Save previous slack variables
//         problem->v = problem->vnew;
//         problem->z = problem->znew;

//         // TODO: remove convergence check and just return when allotted runtime is up
//         // Check for convergence
//         if (problem->primal_residual_state < problem->abs_tol &&
//             problem->primal_residual_input < problem->abs_tol &&
//             problem->dual_residual_state < problem->abs_tol &&
//             problem->dual_residual_input < problem->abs_tol)
//         {
//             problem->status = 1;
//             break;
//         }

//         // TODO: add rho scaling

//         problem->iter += 1;

//         // std::cout << problem->primal_residual_state << std::endl;
//         // std::cout << problem->dual_residual_state << std::endl;
//         // std::cout << problem->primal_residual_input << std::endl;
//         // std::cout << problem->dual_residual_input << "\n" << std::endl;
//     }
// }

// /**
//  * Do backward Riccati pass then forward roll out
// */
// void update_primal(struct tiny_problem *problem, const struct tiny_params *params) {
//     backward_pass_grad(problem, params);
//     forward_pass(problem, params);
// }

// /**
//  * Update linear terms from Riccati backward pass
// */
// void backward_pass_grad(struct tiny_problem *problem, const struct tiny_params *params) {
//     for (int i=NHORIZON-2; i>=0; i--) {
//         // problem->Qu.noalias() = params->cache.Bdyn.transpose().lazyProduct(problem->p.col(i+1));
//         // problem->Qu += problem->r.col(i);
//         // (problem->d.col(i)).noalias() = params->cache.Quu_inv.lazyProduct(problem->Qu);
//         (problem->d.col(i)).noalias() = params->cache.Quu_inv * (params->cache.Bdyn.transpose() * problem->p.col(i+1) + problem->r.col(i));
//         (problem->p.col(i)).noalias() = problem->q.col(i) + params->cache.AmBKt.lazyProduct(problem->p.col(i+1)) - (params->cache.Kinf.transpose()).lazyProduct(problem->r.col(i)); // + params->cache.coeff_d2p * problem->d.col(i); // coeff_d2p always appears to be zeros
//     }
// }

// /**
//  * Use LQR feedback policy to roll out trajectory
// */
// void forward_pass(struct tiny_problem *problem, const struct tiny_params *params) {
//     for (int i=0; i<NHORIZON-1; i++) {
//         (problem->u.col(i)).noalias() = -params->cache.Kinf.lazyProduct(problem->x.col(i)) - problem->d.col(i);
//         // problem->u.col(i) << .001, .02, .3, 4;
//         // DEBUG_PRINT("u(0): %f\n", problem->u.col(0)(0));
//         multAdyn(problem->Ax, params->cache.Adyn, problem->x.col(i));
//         (problem->x.col(i+1)).noalias() = problem->Ax + params->cache.Bdyn.lazyProduct(problem->u.col(i));
//     }
// }

// /**
//  * Project slack (auxiliary) variables into their feasible domain, defined by
//  * projection functions related to each constraint
//  * TODO: pass in meta information with each constraint assigning it to a
//  * projection function
// */
// void update_slack(struct tiny_problem *problem, const struct tiny_params *params) {
//     // Box constraints on input
//     // Get current time

//     problem->znew = params->u_max.cwiseMin(params->u_min.cwiseMax(problem->u));

//     // Half space constraints on state
//     // TODO: support multiple half plane constraints per knot point
//     //      currently this only works for one constraint per knot point
//     // TODO: can potentially take advantage of the fact that A_constraints[3:end] is zero and just do
//     //      v.col(i) = x.col(i) - dist*A_constraints[i] since we have to copy x[3:end] into v anyway
//     //      downside is it's not clear this is happening externally and so values of A_constraints
//     //      not set to zero (other than the first three) can cause the algorithm to fail
//     // TODO: the only state values changing here are the first three (x, y, z) so it doesn't make sense
//     //      to do operations on the remaining 9 when projecting (or doing anything related to the dual
//     //      or auxiliary variables). v and g could be of size (3) and everything would work the same.
//     //      The only reason this doesn't break is because in the update_linear_cost function subtracts
//     //      g from v and so the last nine entries are always zero.
//     // startTimestamp = usecTimestamp();
//     problem->xg = problem->x + problem->g;
//     // problem->dists = (params->A_constraints.transpose().cwiseProduct(problem->xg)).colwise().sum();
//     // problem->dists -= params->x_max;
//     for (int i=0; i<NHORIZON; i++) {
//         problem->dist = (params->A_constraints[i].head(3)).lazyProduct(problem->xg.col(i).head(3)); // Distances can be computed in one step outside the for loop
//         problem->dist -= params->x_max[i](0);
//         // DEBUG_PRINT("dist: %f\n", dist);
//         if (problem->dist <= 0) {
//             problem->vnew.col(i) = problem->xg.col(i);
//         }
//         else {
//             problem->xyz_new = problem->xg.col(i).head(3) - problem->dist*params->A_constraints[i].head(3).transpose();
//             problem->vnew.col(i) << problem->xyz_new, problem->xg.col(i).tail(NSTATES-3);

//             // if (i == NHORIZON-1) {
//             // if (i == 0) {
//             //     std::cout << "prev xyz: "   << problem->x.col(i).head(3)   << std::endl;
//             //     std::cout << "vnew: "       << problem->vnew.col(i).head(3) << std::endl;
//             // }
//         }
        
//         problem->xyz_news.col(i) = problem->xg.col(i).head(3) - problem->dist*params->A_constraints[i].head(3).transpose();
//     }
//     // DEBUG_PRINT("slack: %d\n", usecTimestamp() - startTimestamp);
// }

// /**
//  * Update next iteration of dual variables by performing the augmented
//  * lagrangian multiplier update
// */
// void update_dual(struct tiny_problem *problem, const struct tiny_params *params) {
//     problem->y = problem->y + problem->u - problem->znew;
//     problem->g = problem->g + problem->x - problem->vnew;
// }

// /**
//  * Update linear control cost terms in the Riccati feedback using the changing
//  * slack and dual variables from ADMM
// */
// void update_linear_cost(struct tiny_problem *problem, const struct tiny_params *params) {
//     // problem->r = -(params->Uref.array().colwise() * params->R.array()); // Uref = 0 so commented out for speed up. Need to uncomment if using Uref
//     problem->r = -params->cache.rho * (problem->znew - problem->y);
//     problem->q = -(params->Xref.array().colwise() * params->Q.array());
//     problem->q -= params->cache.rho * (problem->vnew - problem->g);
//     // problem->p.col(NHORIZON-1) = -(params->Xref.col(NHORIZON-1).array().colwise() * params->Qf.array());
//     problem->p.col(NHORIZON-1) = -(params->Xref.col(NHORIZON-1).transpose().lazyProduct(params->cache.Pinf));
//     problem->p.col(NHORIZON-1) -= params->cache.rho * (problem->vnew.col(NHORIZON-1) - problem->g.col(NHORIZON-1));

//     // for (int i=0; i<NHORIZON-1; i++) {
//     //     problem->r.col(i) = -params->cache.rho * (problem->znew.col(i) - problem->y.col(i)) - params->R * params->Uref.col(i);
//     //     problem->q.col(i) = -params->cache.rho * (problem->vnew.col(i) - problem->g.col(i)) - params->Q * params->Xref.col(i);
//     // }
//     // problem->p.col(NHORIZON-1) = -params->cache.rho * (problem->vnew.col(NHORIZON-1) - problem->g.col(NHORIZON-1)) - params->Qf * params->Xref.col(NHORIZON-1);
// }



} /* extern "C" */