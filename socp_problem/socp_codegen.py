import cvxpy as cp
import numpy as np
from cvxpygen import cpg


m = 3
n = 10
p = 5
n_i = 5
# Seed random number generator
np.random.seed(2)

# Define parameters with random values
f = cp.Parameter(n, value=np.random.randn(n))
A = [cp.Parameter((n_i, n), value=np.random.randn(n_i, n)) for _ in range(m)]
b = [cp.Parameter(n_i, value=np.random.randn(n_i)) for _ in range(m)]
c = [cp.Parameter(n, value=np.random.randn(n)) for _ in range(m)]
F = cp.Parameter((p, n), value=np.random.randn(p, n))
x0 = np.random.randn(n)
g = cp.Parameter(p)
g.value = F.value @ x0
d = [cp.Parameter() for _ in range(m)]
# Define decision variables
x = cp.Variable(n)


for i in range(m):
    n_i, n = A[i].value.shape


    if len(A[i].value.shape) != 2:
        raise ValueError("A[i].value is not a 2D array.")

    d[i] = cp.sqrt(cp.square(A[i].value @ x0 + b[i].value).sum()) - c[i].T @ x0


# Define constraints
soc_constraints = [
    cp.SOC(c[i].T @ x + d[i], A[i] @ x + b[i]) for i in range(m)
]
constraints = soc_constraints + [F @ x == g]

# Define objective function
obj = cp.Minimize(f.T @ x)

# Define and solve the problem
prob = cp.Problem(obj, constraints)
prob.solve(solver='ECOS')

# Print result
print("The optimal value is", prob.value)
print("A solution x is")
print(x.value)
for i in range(m):
    print("SOC constraint %i dual variable solution" % i)
    print(soc_constraints[i].dual_value)

# generate code
cpg.generate_code(prob, code_dir='SOCP_main', solver='ECOS')
