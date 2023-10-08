import osqp
import numpy as np
import scipy as sp
from scipy import sparse

# Discrete time model of a quadcopter
# Ad = sparse.csc_matrix([
#   [1.,      0.,     0., 0.],#, 0., 0., 0.1,     0.,     0.,  0.,     0.,     0.    ],
#   [0.,      1.,     0., 0.],#, 0., 0., 0.,      0.1,    0.,  0.,     0.,     0.    ],
#   [0.,      0.,     1., 0.],#, 0., 0., 0.,      0.,     0.1, 0.,     0.,     0.    ],
#   [0.0488,  0.,     0., 1.]])#, 0., 0., 0.0016,  0.,     0.,  0.0992, 0.,     0.    ],
#   [0.,     -0.0488, 0., 0., 1., 0., 0.,     -0.0016, 0.,  0.,     0.0992, 0.    ],
#   [0.,      0.,     0., 0., 0., 1., 0.,      0.,     0.,  0.,     0.,     0.0992],
#   [0.,      0.,     0., 0., 0., 0., 1.,      0.,     0.,  0.,     0.,     0.    ],
#   [0.,      0.,     0., 0., 0., 0., 0.,      1.,     0.,  0.,     0.,     0.    ],
#   [0.,      0.,     0., 0., 0., 0., 0.,      0.,     1.,  0.,     0.,     0.    ],
#   [0.9734,  0.,     0., 0., 0., 0., 0.0488,  0.,     0.,  0.9846, 0.,     0.    ],
#   [0.,     -0.9734, 0., 0., 0., 0., 0.,     -0.0488, 0.,  0.,     0.9846, 0.    ],
#   [0.,      0.,     0., 0., 0., 0., 0.,      0.,     0.,  0.,     0.,     0.9846]
# ])
# Bd = sparse.csc_matrix([
#   [0.,      -0.0726,  0.,     0.0726],
#   [-0.0726,  0.,      0.0726, 0.    ],
#   [-0.0152,  0.0152, -0.0152, 0.0152],
#   [-0.,     -0.0006, -0.,     0.0006],
#   [0.0006,   0.,     -0.0006, 0.0000],
#   [0.0106,   0.0106,  0.0106, 0.0106],
#   [0,       -1.4512,  0.,     1.4512],
#   [-1.4512,  0.,      1.4512, 0.    ],
#   [-0.3049,  0.3049, -0.3049, 0.3049],
#   [-0.,     -0.0236,  0.,     0.0236],
#   [0.0236,   0.,     -0.0236, 0.    ],
#   [0.2107,   0.2107,  0.2107, 0.2107]])
# [nx, nu] = Bd.shape

# # Constraints
# u0 = 10.5916
# umin = np.array([9.6, 9.6, 9.6, 9.6]) - u0
# umax = np.array([13., 13., 13., 13.]) - u0
# xmin = np.array([-np.pi/6,-np.pi/6,-np.inf,-np.inf,-np.inf,-1.,
#                  -np.inf,-np.inf,-np.inf,-np.inf,-np.inf,-np.inf])
# xmax = np.array([ np.pi/6, np.pi/6, np.inf, np.inf, np.inf, np.inf,
#                   np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])

# # Objective function
# Q = sparse.diags([0., 0., 10., 10., 10., 10., 0., 0., 0., 5., 5., 5.])
# QN = Q
# R = 0.1*sparse.eye(4)

# # Initial and reference states
# x0 = np.zeros(12)
# xr = np.array([0.,0.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.])

# # Prediction horizon
# N = 10
data = np.load('rand_prob_osqp_params.npz')
nx = data['nx']
nu = data['nu']
Nh = data['Nh']
Nsim = data['Nsim']
Q = data['Q']*1000
R = data['R']
A = data['A']
B = data['B']
QN = data['Qf']
xbar_full = data['x_bar'].T
umin = data['umin'][:,0] 
umax = data['umax'][:,0] 
 
Ad = sparse.csc_matrix(A)
Bd = sparse.csc_matrix(B)

xbar = xbar_full[:,:Nh]
xmin = -np.Inf*np.ones(nx)
xmax = np.Inf*np.ones(nx)
x0 = np.zeros(nx)
xr = np.array([0.,0.,1.,0.])

# Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
# - quadratic objective
P = sparse.block_diag([sparse.kron(sparse.eye(Nh), Q), QN,
                       sparse.kron(sparse.eye(Nh), R)], format='csc')
# - linear objective
q = np.hstack([np.hstack([-Q@xbar_full[:,i] for i in range(Nh)]), -QN@xbar_full[:,Nh], np.zeros(Nh*nu)])
# q = np.hstack([np.hstack([-Q@xr for i in range(Nh)]), -QN@xr, np.zeros(Nh*nu)])
# q = np.hstack([np.kron(np.ones(Nh), -Q@xr), -QN@xr, np.zeros(Nh*nu)])
# - linear dynamics
Ax = sparse.kron(sparse.eye(Nh+1),-sparse.eye(nx)) + sparse.kron(sparse.eye(Nh+1, k=-1), Ad)
Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, Nh)), sparse.eye(Nh)]), Bd)
Aeq = sparse.hstack([Ax, Bu])
leq = np.hstack([-x0, np.zeros(Nh*nx)])
ueq = leq

# - input and state constraints
Aineq = sparse.eye((Nh+1)*nx + Nh*nu)
lineq = np.hstack([np.kron(np.ones(Nh+1), xmin), np.kron(np.ones(Nh), umin)])
uineq = np.hstack([np.kron(np.ones(Nh+1), xmax), np.kron(np.ones(Nh), umax)])
# - OSQP constraints
A = sparse.vstack([Aeq, Aineq], format='csc')
l = np.hstack([leq, lineq])
u = np.hstack([ueq, uineq])

# Create an OSQP object
prob = osqp.OSQP()

# Setup workspace
prob.setup(P, q, A, l, u, warm_starting=True, verbose=False)
# prob.setup(P, q, A, l, u, alpha=1.0, scaling=0, check_termination=1, eps_abs=1e-5, eps_rel=1e-5, eps_prim_inf=1e-5, eps_dual_inf=1e-5, max_iter=10000, polish=False, rho=0.1, adaptive_rho=False, warm_start=True)

# import pdb; pdb.set_trace()
# Simulate in closed loop
for i in range(Nsim-Nh-1):
    # Solve
    res = prob.solve()
    # print("Res.x: ",res.x)
    # print(np.linalg.norm(x0-xbar_full[:,i]))
    print("x0: ",x0,"\nxbar: ",xbar_full[:,i],"\n")
    print("diff: ", np.linalg.norm(x0-xbar_full[:,i]))

    # Check solver status
    if res.info.status != 'solved':
        raise ValueError('OSQP did not solve the problem!')

    # Apply first control input to the plant
    # ctrl = res.x[-Nh*nu:-(Nh-1)*nu]
    ctrl = res.x[(Nh +1) * nx:]
    x0 = Ad@x0 + Bd@ctrl

    # Update initial state
    l[:nx] = -x0
    u[:nx] = -x0
    q = np.hstack([np.hstack([-Q@xbar_full[:,i+1+k] for k in range(Nh)]), -QN@xbar_full[:,i+1+Nh], np.zeros(Nh*nu)])
    prob.update(q=q,l=l, u=u)

# if i == Nsim-nx-1:
print(res.info.obj_val)