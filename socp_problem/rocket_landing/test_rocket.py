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


def update_linear_term(params):
    # Define cost
    P = np.zeros((NN, NN))
    q = np.zeros((NN, 1))
    for j in range(NHORIZON - 1):
        Q, dQ, R, dR = stage_cost_expansion(params, j)
        Pj = np.block([[Q, np.zeros((NSTATES, NINPUTS))],
                      [np.zeros((NINPUTS, NSTATES)), R]])
        qj = np.concatenate((dQ, dR))
        # import pdb; pdb.set_trace()
        P[j * (NSTATES + NINPUTS): (j + 1) * (NSTATES + NINPUTS), j * (NSTATES + NINPUTS): (j + 1) * (NSTATES + NINPUTS)] = Pj
        q[j * (NSTATES + NINPUTS): (j + 1) * (NSTATES + NINPUTS)] = qj.reshape(-1, 1)
    
    # Terminal cost
    Qf, dQf = term_cost_expansion(params)
    P[-NSTATES:, -NSTATES:] = Qf  
    q[-NSTATES:] = dQf.reshape(-1, 1)    
    return scipy.linalg.sqrtm(P), q

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
f = np.array([0.0, 0.0, -0.0122625, 0.0, 0.0, -0.4905])

NSTATES = 6
NINPUTS = 3
NHORIZON = 21 # horizon length, short for MPC
NTOTAL = 301
dt = 0.05
t_vec = dt * np.arange(NTOTAL)
# print('t_vec', t_vec.shape)

x0 = np.array([4, 2, 20, -3, 2, -4.5])
xg = np.array([0, 0, 0, 0, 0, 0.0])
Xref = [x0 + (xg - x0) * k / (NTOTAL-1) for k in range(NTOTAL)]
Uref = [[0, 0, 10.] for _ in range(NTOTAL - 1)]
Xref_hrz = Xref[:NHORIZON]*1
Uref_hrz = Uref[:NHORIZON-1]*1

# print('Xref', Xref)
# print('Uref', len(Uref))
Q = 1e3 * np.eye(NSTATES)
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
u_min = -10 * np.ones(NINPUTS)
u_max = 105.0 * np.ones(NINPUTS)
x_min = [-5, -5, 0, -10, -10, -10.0]
x_max = [5, 5, 20, 10, 10, 10.0]

params = {
    'NSTATES': NSTATES,
    'NINPUTS': NINPUTS,
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

NN = NHORIZON*NSTATES + (NHORIZON-1)*NINPUTS  # NINPUTSmber of decision variables
x0_param = cp.Parameter(NSTATES)  # initial state

z = cp.Variable(NN)
inds = np.reshape(np.arange(0, NN + NINPUTS), (NSTATES + NINPUTS, NHORIZON), order='F')
# print('inds', inds.shape)
xinds = [inds[:NSTATES, i] for i in range(NHORIZON)]
# print('xinds', (xinds))
uinds = [inds[NSTATES:, i] for i in range(NHORIZON - 1)]
# print('uinds', (uinds))

# import pdb; pdb.set_trace()

P_param = cp.Parameter((NN, NN),symmetric=True, value=np.zeros((NN, NN)) )
q_param = cp.Parameter((NN, 1), value =np.zeros((NN, 1)))
P_param.value, q_param.value = update_linear_term(params)
# print(P_param.value, q_param.value)

objective = cp.Minimize(0.5 * cp.sum_squares(P_param @z) + q_param.T @ z)
#objective = cp.Minimize(0.5 * cp.quad_form(z, P_param) + q_param.T @ z)
constraints = []

# Dynamics Constraints
for k in range(NHORIZON - 2):
    constraints.append(A @ z[xinds[k]] + B @ z[uinds[k]] + f == z[xinds[k+1]])

# Initial states
constraints.append(z[xinds[0]] == x0_param)

# Thrust angle constraint (SOC): norm([u1,u2]) <= alpha_max * u3
for k in range(NHORIZON-1):
    u1, u0 = z[uinds[k]][0:2], z[uinds[k]][2]
    constraints.append(cp.norm(u1) <= 0.25 * u0)

# State constraints
# for k in range(NHORIZON):
#     constraints.append(z[xinds[k]] <= params['x_max'])
#     constraints.append(z[xinds[k]] >= params['x_min'])

# Input constraints
for k in range(NHORIZON-1):
    constraints.append(z[uinds[k]] <= params['u_max'])
    constraints.append(z[uinds[k]] >= params['u_min'])

# Goal constraint
# constraints.append(z[xinds[NHORIZON-1]] == np.zeros(NSTATES))

problem = cp.Problem(objective,constraints)
opts = {"verbose": False, "solver": "ECOS", "abstol": 1e-2, "max_iters": 100}

# MPC loop
np.random.seed(1234)
NRUNS = NTOTAL - NHORIZON - 1
Xhist = np.zeros((NSTATES, NTOTAL))
Xhist[:, 0] = x0*1.1
Uhist = np.zeros((NINPUTS, NTOTAL-1))
x0_param.value = Xhist[:, 0]*1
params["Xref"] = Xref[0:NHORIZON]*1
params["Uref"] = Uref[0:NHORIZON-1]*1
P_param.value, q_param.value = update_linear_term(params)

# GENERATE CODE (uncomment to generate code)
GEN_CODE = 1

if GEN_CODE:
    cpg.generate_code(problem, code_dir='SOCP_rocket_landing', solver='ECOS', solver_opts=opts)

    problem.solve(verbose=False, solver="ECOS", abstol=1e-2, max_iters=100)
    print(z.value)
else:
    for k in range(NRUNS):
        # Get measurements
        x0_param.value = Xhist[:, k]*1

        # Update references
        params["Xref"] = Xref[k:k+NHORIZON]*1
        params["Uref"] = Uref[k:k+NHORIZON-1]*1
        P_param.value, q_param.value = update_linear_term(params)
        # print(P_param.value, q_param.value,x0_param.value)
        
        # Solve MPC problem
        problem.solve(verbose=False, solver="ECOS", abstol=1e-2, max_iters=100)
        # Extract results
        for j in range(NHORIZON-1):
            X[j] = z[xinds[j]].value
            U[j] = z[uinds[j]].value
        X[NHORIZON-1] = z[xinds[NHORIZON-1]].value
        Uhist[:, k] = U[0]*1

        # Simulate system
        Xhist[:, k+1] = A @ Xhist[:, k] + B @ Uhist[:, k] + f

    # print(U)
    # print(X)
    # plot results
    import matplotlib.pyplot as plt
    plt.figure()
    plt.plot(Xhist[:3,:NRUNS-1].T)
    # plt.plot(np.array(X))
    plt.title('States')
    plt.show()

    plt.figure()
    plt.plot(Uhist[:,:NRUNS-1].T)
    # plt.plot(np.array(X))
    plt.title('Controls')
    plt.show()

