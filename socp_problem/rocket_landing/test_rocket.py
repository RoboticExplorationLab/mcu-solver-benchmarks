import numpy as np
import cvxpy as cp
from cvxpygen import cpg
import scipy

def stage_cost_expansion(p, k):
    dx = -np.array(p['Xref'][k])
    du = -np.array(p['Uref'][k])
    Q = p['Q']
    R = p['R']
    return Q, Q @ dx, R, R @ du


def term_cost_expansion(p):
    dx = -np.array(p['Xref'][p['NHORIZON']-1])
    Qf = p['Qf']
    # print('Qf', Qf)
    # print('dx', dx)
    return Qf, Qf @  dx

def mpc_cvxpy(params, X, U, A, B, f):
    NHORIZON = params['NHORIZON']
    NSTATES = params['NSTATES']
    NINPUTS = params['NINPUTS']
    alpha_max = params['c_cone'][2]  # Thrust angle constraint
    NN = NHORIZON*NSTATES + (NHORIZON-1)*NINPUTS  # NINPUTSmber of decision variables
    x0 = X[0].copy()  # initial state

    z = cp.Variable(NN)
    inds = np.reshape(np.arange(0, NN + NINPUTS), (NSTATES + NINPUTS, NHORIZON), order='F')
    # print('inds', inds.shape)
    xinds = [inds[:NSTATES, i] for i in range(NHORIZON)]
    # print('xinds', (xinds))
    uinds = [inds[NSTATES:, i] for i in range(NHORIZON - 1)]
    # print('uinds', (uinds))

    # Define parameters
    P_param = cp.Parameter((NN, NN),symmetric=True, value=np.zeros((NN, NN)) )
    q_param = cp.Parameter((NN, 1), value =np.zeros((NN, 1)) )

    # Objective function
    P = np.zeros((NN, NN))
    q = np.zeros((NN, 1))
    for j in range(NHORIZON - 1):
        Q, dQ, R, dR = stage_cost_expansion(params, j)
        Pj = np.block([[Q, np.zeros((NSTATES, NINPUTS))],
                      [np.zeros((NINPUTS, NSTATES)), R]])
        qj = np.concatenate((dQ, dR))
        P[j * (NSTATES + NINPUTS): (j + 1) * (NSTATES + NINPUTS), j * (NSTATES + NINPUTS): (j + 1) * (NSTATES + NINPUTS)] = Pj
        q[j * (NSTATES + NINPUTS): (j + 1) * (NSTATES + NINPUTS)] = qj.reshape(-1, 1)
    
    # Terminal cost
    Qf, dQf = term_cost_expansion(params)
    P[-NSTATES:, -NSTATES:] = Qf  
    q[-NSTATES:] = dQf.reshape(-1, 1)    
    # print('P', P[0:10, 0:10])
    # print('q', q[0:10])
    P_param.value = scipy.linalg.sqrtm(P)
    #P_param.value = P
    q_param.value = q
    # print('z',z.shape)
    if q.shape != z.shape:
        q = q.reshape(z.shape)
    # print('P',P.shape)
    # print('P_param',P_param.shape)
    # print('q',q.shape)
    # print('q_param',q_param.shape)  

    objective = cp.Minimize(0.5 * cp.sum_squares(P_param @z) + q_param.T @ z)
    #objective = cp.Minimize(0.5 * cp.quad_form(z, P_param) + q_param.T @ z)
    constraints = []

    # Dynamics Constraints
    for k in range(NHORIZON - 2):
        constraints.append(A @ z[xinds[k]] + B @ z[uinds[k]] + f == z[xinds[k+1]])

    constraints.append(z[xinds[0]] == x0)

    # Thrust angle constraint (SOC): norm([u1,u2]) <= alpha_max * u3
    if params["ncu_cone"] > 0:
        for k in range(NHORIZON-1):
            u1, u0 = z[uinds[k]][0:2], z[uinds[k]][2]
            constraints.append(cp.norm(u1) <= alpha_max * u0)
    """
    if params['ncu_cone'] > 0:
        for k in range(NHORIZON - 1):

            u1, u2, u3 = z[uinds[k]]


            soc_constraint_vector = cp.vstack([u1, u2])

            constraints.append(cp.SOC(alpha_max * u3, soc_constraint_vector))
    """
    # State constraints
    # if params['ncx'] > 0:
    #     for k in range(NHORIZON):
    #         constraints.append(z[xinds[k]] <= params['x_max'])
    #         constraints.append(z[xinds[k]] >= params['x_min'])

    # Input constraints
    if params['ncu'] > 0:
        for k in range(NHORIZON-1):
            constraints.append(z[uinds[k]] <= params['u_max'])
            constraints.append(z[uinds[k]] >= params['u_min'])
    
    # Goal constraint
    # if params['ncg'] > 0:
    #     constraints.append(z[xinds[NHORIZON-1]] == np.zeros(NSTATES))
    
    # Solve
    problem = cp.Problem(objective,constraints)
    #problem.solve(verbose=True, feastol = 1e-4,abstol=1e-4, reltol =1e-4)
    try:
        problem.solve(verbose=False, solver="ECOS", abstol=1e-2)
    except cp.error.DCPError as e:
        print("DCPError:", e)
        return None
    problem.solve(verbose=False, solver="ECOS", abstol=1e-2)

    # generate code
    # cpg.generate_code(problem, code_dir='SOCP_rocket_landing', solver='ECOS')

    # Extract results
    for j in range(NHORIZON-1):
        X[j] = z[xinds[j]].value
        U[j] = z[uinds[j]].value
    X[NHORIZON-1] = z[xinds[NHORIZON-1]].value

    return U[0]

def setup_solver(params, X, U, A, B, f):
    NHORIZON = params['NHORIZON']
    NSTATES = params['NSTATES']
    NINPUTS = params['NINPUTS']
    alpha_max = params['c_cone'][2]  # Thrust angle constraint
    NN = NHORIZON*NSTATES + (NHORIZON-1)*NINPUTS  # NINPUTSmber of decision variables
    x0 = X[0].copy()  # initial state

    z = cp.Variable(NN)
    inds = np.reshape(np.arange(0, NN + NINPUTS), (NSTATES + NINPUTS, NHORIZON), order='F')
    # print('inds', inds.shape)
    xinds = [inds[:NSTATES, i] for i in range(NHORIZON)]
    # print('xinds', (xinds))
    uinds = [inds[NSTATES:, i] for i in range(NHORIZON - 1)]
    # print('uinds', (uinds))

    # Define parameters
    P_param = cp.Parameter((NN, NN),symmetric=True, value=np.zeros((NN, NN)) )
    q_param = cp.Parameter((NN, 1), value =np.zeros((NN, 1)) )

    # Objective function
    P = np.zeros((NN, NN))
    q = np.zeros((NN, 1))
    for j in range(NHORIZON - 1):
        Q, dQ, R, dR = stage_cost_expansion(params, j)
        Pj = np.block([[Q, np.zeros((NSTATES, NINPUTS))],
                      [np.zeros((NINPUTS, NSTATES)), R]])
        qj = np.concatenate((dQ, dR))
        P[j * (NSTATES + NINPUTS): (j + 1) * (NSTATES + NINPUTS), j * (NSTATES + NINPUTS): (j + 1) * (NSTATES + NINPUTS)] = Pj
        q[j * (NSTATES + NINPUTS): (j + 1) * (NSTATES + NINPUTS)] = qj.reshape(-1, 1)
    
    # Terminal cost
    Qf, dQf = term_cost_expansion(params)
    P[-NSTATES:, -NSTATES:] = Qf  
    q[-NSTATES:] = dQf.reshape(-1, 1)    
    # print('P', P[0:10, 0:10])
    # print('q', q[0:10])
    P_param.value = scipy.linalg.sqrtm(P)
    #P_param.value = P
    q_param.value = q
    # print('z',z.shape)
    if q.shape != z.shape:
        q = q.reshape(z.shape)
    # print('P',P.shape)
    # print('P_param',P_param.shape)
    # print('q',q.shape)
    # print('q_param',q_param.shape)  

    objective = cp.Minimize(0.5 * cp.sum_squares(P_param @z) + q_param.T @ z)
    #objective = cp.Minimize(0.5 * cp.quad_form(z, P_param) + q_param.T @ z)
    constraints = []

    # Dynamics Constraints
    for k in range(NHORIZON - 2):
        constraints.append(A @ z[xinds[k]] + B @ z[uinds[k]] + f == z[xinds[k+1]])

    constraints.append(z[xinds[0]] == x0)

    # Thrust angle constraint (SOC): norm([u1,u2]) <= alpha_max * u3
    if params["ncu_cone"] > 0:
        for k in range(NHORIZON-1):
            u1, u0 = z[uinds[k]][0:2], z[uinds[k]][2]
            constraints.append(cp.norm(u1) <= alpha_max * u0)
    """
    if params['ncu_cone'] > 0:
        for k in range(NHORIZON - 1):

            u1, u2, u3 = z[uinds[k]]


            soc_constraint_vector = cp.vstack([u1, u2])

            constraints.append(cp.SOC(alpha_max * u3, soc_constraint_vector))
    """
    # State constraints
    # if params['ncx'] > 0:
    #     for k in range(NHORIZON):
    #         constraints.append(z[xinds[k]] <= params['x_max'])
    #         constraints.append(z[xinds[k]] >= params['x_min'])

    # Input constraints
    if params['ncu'] > 0:
        for k in range(NHORIZON-1):
            constraints.append(z[uinds[k]] <= params['u_max'])
            constraints.append(z[uinds[k]] >= params['u_min'])
    
    # Goal constraint
    # if params['ncg'] > 0:
    #     constraints.append(z[xinds[NHORIZON-1]] == np.zeros(NSTATES))
    
    # Solve
    problem = cp.Problem(objective,constraints)
    return problem

if __name__ == "__main__":
    # Define problem parameters
    A = np.array([[1.0, 0.0, 0.0, 0.05, 0.0, 0.0],
                  [0.0, 1.0, 0.0, 0.0, 0.05, 0.0],
                  [0.0, 0.0, 1.0, 0.0, 0.0, 0.05],
                  [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
    B = np.array([[0.000125, 0.0, 0.0],
                  [0.0, 0.000125, 0.0],
                  [0.0, 0.0, 0.000125],
                  [0.005, 0.0, 0.0],
                  [0.0, 0.005, 0.0],
                  [0.0, 0.0, 0.005]])
    f = np.array([0.0, 0.0, -0.0122625, 0.0, 0.0, -0.4905])*0

    NSTATES = 6
    NINPUTS = 3
    NHORIZON = 20 # horizon length, short for MPC
    NTOTAL = 301
    dt = 0.05
    t_vec = dt * np.arange(NTOTAL)
    # print('t_vec', t_vec.shape)

    x0 = np.array([4, 2, 20, -1, 2, -4.0])
    xg = np.array([0, 0, 0, 0, 0, 0.0])
    Xref = [x0 + (xg - x0) * k / (NTOTAL-1) for k in range(NTOTAL)]
    Uref = [[0, 0, 10.] for _ in range(NTOTAL - 1)]
    Xref_hrz = Xref[:NHORIZON]*1
    Uref_hrz = Uref[:NHORIZON-1]*1

    # print('Xref', Xref)
    # print('Uref', len(Uref))
    Q = 1e2 * np.eye(NSTATES)
    R = 1e0 * np.eye(NINPUTS)
    Qf = Q*1

    # gravity = np.array([0, 0, -9.81])
    # mass = 10.0
    # perWeightMax = 2.0
    # θ_thrust_max = 5.0  # deg

    A_cone = np.array([[1, 0, 0], [0, 1.0, 0]])
    # c_cone = np.array([0.0, 0.0, np.tan(np.radians(θ_thrust_max))])
    c_cone = np.array([0.0, 0.0, 0.4])
    # u_bnd = mass * np.abs(gravity[2]) * perWeightMax
    # print('u_bnd', u_bnd.shape)

    # Sloppy bounds to test
    u_min = -20 * np.ones(NINPUTS)
    u_max = 105.0 * np.ones(NINPUTS)
    x_min = [-5, -5, 0, -10, -10, -10.0]
    x_max = [5, 5, 20, 10, 10, 10.0]

    # Enable constraints
    ncx = 0
    ncu = 1
    ncg = 0
    ncu_cone = 1

    params = {
        'NSTATES': NSTATES,
        'NINPUTS': NINPUTS,
        'ncx': ncx,
        'ncu': ncu,
        'ncg': ncg,
        'ncu_cone': ncu_cone,
        'A_cone': A_cone,
        'c_cone': c_cone,
        'NHORIZON': NHORIZON,
        'Q': Q,
        'R': R,
        'Qf': Qf,
        'u_min': u_min,
        'u_max': u_max,
        'x_min': x_min,
        'x_max': x_max,
        'Xref': Xref_hrz,
        'Uref': Uref_hrz,
        'dt': dt,
    }

    X = [np.copy(x0) for _ in range(NHORIZON)]
    # print('X', len(X))
    U = [np.copy(Uref[0]) for _ in range(NHORIZON-1)]
    # print('U', len(U))

    # MPC loop
    np.random.seed(1234)
    NRUNS = NTOTAL - NHORIZON - 1
    Xhist = np.zeros((NSTATES, NRUNS))
    Xhist[:, 0] = x0*1
    Uhist = np.zeros((NINPUTS, NRUNS-1))
    
    for k in range(NRUNS-1):
        # Get measurements
        X[0] = Xhist[:, k]*1

        # Update references
        params["Xref"] = Xref[k:k+NHORIZON]*1
        params["Uref"] = Uref[k:k+NHORIZON-1]*1

        # Solve MPC problem
        Uhist[:, k] = mpc_cvxpy(params, X, U, A, B, f)

        # Simulate system
        Xhist[:, k+1] = A @ Xhist[:, k] + B @ Uhist[:, k] + f

    # plot results
    import matplotlib.pyplot as plt
    plt.figure()
    plt.plot(Xhist[:3,:].T)
    plt.title('States')
    plt.show()


    