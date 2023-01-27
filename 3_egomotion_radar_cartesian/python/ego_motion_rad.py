import numpy as np


def cts_meas(z, mount_x, mount_y, mount_yaw):
    """ Coordinate Transformation of the radar measurements from the sensor frame to vehicle frame
    Input: z - radar measurements (px, py, vx, vy) of shape (n, d) in the ego vehicle frame
         : mount_x, mount_y, mount_yaw - radar mounting parameters
    Output: z -  coordinate transformed radar measurements
    """
    mount_rot = np.array([[np.cos(mount_yaw), -np.sin(mount_yaw)],
                          [np.sin(mount_yaw),  np.cos(mount_yaw)]])
    T = np.array([mount_x, mount_y])
    z[:, 0:2] = z[:, 0:2] @ mount_rot.transpose() + T
    z[:, 2:4] = z[:, 2:4] @ mount_rot.transpose()
    return z


def meas_model(z):
    """ create the measurement model (vx, yaw_rate) -> z
    Input: z - radar measurements (px, py, vx, vy) of shape (n, d) in the ego vehicle frame
    Output : A - measurement model of shape (2n x 2)
    """
    A1 = np.stack([-z[:, 1],   z[:, 0]],  axis=0)                   # 2 x n
    A1 = np.expand_dims(np.reshape(A1.transpose(), -1), axis=-1)    # 2n x 1

    A2 = np.eye(2)                                         # 2 x 2
    A2 = np.expand_dims(A2, axis=0)                        # 1 x 2 x 2
    A2 = np.repeat(A2, z.shape[0], axis=0)                 # n x 2 x 2
    A2 = np.reshape(A2, (A2.shape[0]*A2.shape[1], -1))     # 2n x 2

    A = np.concatenate([A2, A1], axis=-1)
    return A


def gated_meas_model(A, gated_meas_idx):
    """ select a sub matrix from A corrosponding to the gated measurement index
    Input: A - measurement model of shape (2n x 3)
         : gated_meas_idx - gated measurement index of shape (k, )
    Output: A - measurement model of shape (2k x 2)
    """
    n = A.shape[0] // 2
    A = A.reshape((n, 2, 3))
    A = A[gated_meas_idx, :, :]
    A = A.reshape((A.shape[0]*A.shape[1], -1))
    return A


def compute_ego_motion_lse(z, A, z_rms, mode):
    """ Estimate ego motion from radar measurements
    Inputs: z - measurements of shape (n, d), has attribute ( px, py, vx, vy )
          : z_rms - measurements errors of shape (n, d), having attributes ( px_rms, py_rms, vx_rms, vy_rms )
          : A - measurement model
          : mode - 2dof or 3dof based estimation
    Outputs: ego_motion - (vx, yaw_rate) or (vx, vy, yaw_rate) depending on which dof measurement model is used
             sigma - covariance of the ego motion estimation 
    """
    num_meas = z.shape[0]
    y = np.stack((-z[:, 2], -z[:, 3]), axis=0)
    y = np.expand_dims(np.reshape(y.transpose(), -1), axis=1)

    R = np.eye(2*num_meas)
    z_rms = (z_rms[:, [2, 3]]).flatten()
    np.fill_diagonal(R, z_rms)

    # solve the linear equations
    if mode == '2dof': A = A[:, [0, 2]]
    A = np.linalg.inv( A.transpose() @ A ) @ A.transpose()
    x = A @ y
    x = np.reshape(x, -1)

    # estimation covariance
    sigma = A @ R @ A.transpose()

    return x, sigma


def predict_meas(ego_motion, A):
    """ predict measurements given the predicted ego motion
    Input : ego_motion - predicted ego motion - (vx, yaw_rate) / (vx, vy, yaw_rate)
          : A - measurement model of shape (2n x 2) / (2n x 3)
    Output : z_pred - predicted measurement of shape (n x 2)
    """
    num_meas = A.shape[0] // 2
    z_pred = A @ ego_motion
    z_pred = z_pred.reshape((num_meas, -1))
    return -z_pred


def compute_errors(z, z_pred):
    """ compute errror between meas and pred meas velocities
    Input : z - meas of shape (n x 4) , (px, py, vx, vy)
          : z_pred - pred meas of shape (n x 2), (px, py)
    Output : err - error of shape (n x 2)
           : mu -  mean of the error of shape (2, )
           : sigma - covariance of the error of shape (2 x 2)
    """
    z = np.stack((z[:, 2], z[:, 3]), axis=1)
    err = z - z_pred
    mu = np.sum(err, axis=0) / err.shape[0]
    delta = np.expand_dims(err - mu, axis=-1)
    delta = np.sum(delta @ delta.transpose(0,2,1), axis=0)
    sigma = delta / (err.shape[0] - 1)
    return err, mu, sigma


def gate_meas(mu, sigma, err, chi_sq):
    """ if the error (z - z_pred) within a certain threshold then gate the measurement
    Input : mu -  mean of the error of shape (2, )
          : sigma - covariance of the error of shape (2 x 2)
          : chi_sq - threshold
    Output : gated_meas - gated meas index
           : ungated_meas - un-gated meas index 
    """
    err = np.expand_dims(err - mu, axis=-1)
    m_dist = err.transpose(0,2,1) @ np.linalg.inv( sigma ) @ err
    gated_meas = (m_dist <= chi_sq).nonzero()[0]
    ungated_meas = (m_dist > chi_sq).nonzero()[0]
    return gated_meas, ungated_meas