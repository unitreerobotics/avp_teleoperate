import numpy as np

def mat_update(prev_mat, mat):
    if np.linalg.det(mat) == 0:
        return prev_mat, False # Return previous matrix and False flag if the new matrix is non-singular (determinant ≠ 0).
    else:
        return mat, True


def fast_mat_inv(mat):
    ret = np.eye(4)
    ret[:3, :3] = mat[:3, :3].T
    ret[:3, 3] = -mat[:3, :3].T @ mat[:3, 3]
    return ret