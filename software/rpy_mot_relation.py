import numpy as np

B = np.array([
    [+1, +1, -1, -1],
    [-1, +1, +1, -1],
    [-1, +1, -1, +1]
])

C = B.T @ np.linalg.inv(B @ B.T)
print(C)
