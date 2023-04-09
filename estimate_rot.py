import numpy as np
from scipy import io
from quaternion import Quaternion
import math
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
#data files are numbered on the server.
#for exmaple imuRaw1.mat, imuRaw2.mat and so on.
#write a function that takes in an input number (1 through 3)
#reads in the corresponding imu Data, and estimates
#roll pitch and yaw using an unscented kalman filter

def quat_multiply(q1, q2):
    t0 = q1[0] * q2[0] - \
         q1[1] * q2[1] - \
         q1[2] * q2[2] - \
         q1[3] * q2[3]
    t1 = q1[0] * q2[1] + \
         q1[1] * q2[0] + \
         q1[2] * q2[3] - \
         q1[3] * q2[2]
    t2 = q1[0] * q2[2] - \
         q1[1] * q2[3] + \
         q1[2] * q2[0] + \
         q1[3] * q2[1]
    t3 = q1[0] * q2[3] + \
         q1[1] * q2[2] - \
         q1[2] * q2[1] + \
         q1[3] * q2[0]
    retval = Quaternion(t0, [t1, t2, t3])
    return retval.q

def _get_sigma_points(P_prev, Q, qw, n):

    S = np.linalg.cholesky(P_prev + Q)
    sigma_points = np.hstack((S * np.sqrt(n[0]), -1 * S * np.sqrt(n[0])))
    W = np.zeros((7, sigma_points.shape[1]))
    qw_quat = Quaternion(qw[0], qw[1:4])
    for i in range(sigma_points.shape[1]):
        obj = Quaternion()
        obj.from_axis_angle(sigma_points[:3, i])
        W[0:4, i] = (qw_quat * obj).q
        del obj
        W[4:, i] = sigma_points[3:, i] + qw[4:]
    return W

# def transform_to_y():
def q_from_axis_angle(a):

    angle = np.linalg.norm(a)
    if angle != 0:
        axis = a/angle
    else:
        axis = np.array([1,0,0])
    q = np.array([math.cos(angle/2)])
    q_axis = axis*math.sin(angle/2)
    q = np.concatenate((q, q_axis), axis = 0)
    return q


def find_quaternion_mean(Y, q_init):

    error_vectors = np.zeros((3, Y.shape[1]))
    q_t = q_init
    error_mean_mag = 100
    mean_matrix = np.zeros((Y.shape[0], 1))
    count = 0
    while error_mean_mag > 0.0004 and count<26:
        count +=1
        q_inv = q_t.inv()
        # q_inv = q_t
        for each_row in range(Y.shape[1]):
            q_i = Y[:4, each_row]
            q_i = Quaternion(q_i[0], q_i[1:4])
            error_quat = (q_i * q_inv)
            error_quat.normalize()
            error_vect = Quaternion.axis_angle(error_quat)
            error_vectors[:, each_row] = error_vect

        error_mean = np.mean(error_vectors, axis = 1)
        error_mean_mag = np.linalg.norm(error_mean)
        error_mean_obj = Quaternion()
        error_mean_obj.from_axis_angle(error_mean)
        q_t = error_mean_obj*q_t

    mean_matrix[0:4, 0] = q_t.q
    mean_matrix[4:, 0] = np.mean(Y[4:, :], axis=1)
    # mean_matrix = np.concatenate((q_t.q, np.mean(Y[4:, :], axis = 1))).reshape(-1, 1)
    omega_error = Y[4:, :] - mean_matrix[4:]
    error_matrix = np.concatenate((error_vectors, omega_error))


    return mean_matrix, error_matrix


def estimate_rot(data_num=1):
    #load data
    imu = io.loadmat('source/imu/imuRaw'+str(data_num)+'.mat')
    # vicon = io.loadmat('vicon/viconRot'+str(data_num)+'.mat')
    accel = imu['vals'][0:3,:]
    gyro = imu['vals'][3:6,:]
    T = np.shape(imu['ts'])[1]
    roll = np.zeros(T)
    pitch = np.zeros(T)
    yaw = np.zeros(T)

    # your code goes here

    # Calibration
    n = imu["vals"].shape
    # accel_bias = -1 * np.array([[511], [501], [503]])
    accel_bias = np.array([[511], [501], [503]])

    accel_sensitivity = 33.86
    gyro_bias = np.array([[369.5], [371.5], [377]])
    gyro_sensitivity = 193.55
    # accel = np.vstack((-1 * accel[:2, :], accel[2, :]))

    accel_calibrated = (accel - accel_bias) * (3300/(1023 * accel_sensitivity))
    gyro_calibrated = (gyro - gyro_bias) * (3300/(1023 * gyro_sensitivity))
    accel_calibrated = np.vstack((-1 * accel_calibrated[:2, :], accel_calibrated[2, :]))
    gyro_calibrated = np.vstack((gyro_calibrated[1:3, :], gyro_calibrated[0, :]))
    g_quat = Quaternion(0, [0, 0, 9.81])

    # Finding P and S
    P = np.zeros((T, 6, 6))
    P[0] = 0.1 * np.identity(6)
    P_prev = P[0]
    Q = 0.1 * np.identity(6)
    R = 0.1 * np.identity(6)
    q_init = Quaternion().q
    w_init = np.array([0.4, 0.4, 0.4])

    w_k = np.zeros((7, T))
    w_k[:, 0] = np.hstack((q_init, w_init))
    state_new = Quaternion()

    for i in range(1, T):
        # print("i", i)
        delta_T = imu["ts"][0][i] - imu["ts"][0][i-1]
        qw = w_k[:, i-1]

        W = _get_sigma_points(P_prev, Q, qw, n)

        # Estimate Transformed point Yi MU k+1/k : using q_delta and W_delta
        q_delta = q_from_axis_angle(qw[4:] * delta_T)
        Y = np.zeros((W.shape[0], W.shape[1]))

        for each_sigma in range(W.shape[1]):
            Y[0:4, each_sigma] = quat_multiply(W[0:4, each_sigma], q_delta)
            Y[4:, each_sigma] = W[4:, each_sigma]

        mean_matrix, error_matrix = find_quaternion_mean(Y, state_new)

        # H PART
        Z = np.zeros((Y.shape[0]-1, Y.shape[1]))
        for each_row in range(Y.shape[1]):
            q_k = Quaternion(Y[0, each_row], Y[1:4, each_row])
            g_prime = q_k.inv() * g_quat * q_k
            Z[:, each_row] = np.concatenate((g_prime.q[1:], Y[4:, each_row]), axis = 0)

        # Calculate P_xz
        Z_mean = np.mean(Z, axis = 1)
        # print("Z_mean", Z_mean)
        Z_bary = Z - Z_mean.reshape(-1, 1)
        P_xz = (error_matrix @ Z_bary.T) / Z.shape[1]
        # print("P_xz", P_xz)

        P_zz = (Z_bary @ Z_bary.T) / Z.shape[1]
        P_vv = P_zz + R
        K_K = P_xz @ np.linalg.inv(P_vv)

        P_k_inv = (error_matrix @ error_matrix.T)/error_matrix.shape[1]

        P[i] = P_k_inv - (K_K @ P_vv @ K_K.T)
        # print("P_k", P_k)
        # print("a")
        # P[k] = covariance_k - (K_K @ P_vv @ K_K.T)
        P_prev = P[i]

        z_k = np.concatenate((accel_calibrated[:, i-1], gyro_calibrated[:, i-1]), axis =0)
        # print(z_k.shape)
        v_k = (z_k - Z_mean)
        # print("v_k", v_k)


        k_update = K_K @ v_k

        q_new = Quaternion()
        q_new.from_axis_angle(k_update[:3])

        mu_new = mean_matrix[4:].T + k_update[3:]
        state_new = Quaternion(mean_matrix[0], mean_matrix[1:4].reshape(1, -1)[0])
        state_new = q_new * state_new

        w_k[:, i] = np.concatenate((state_new.q, mu_new[0]))
        quatToEul = Quaternion(w_k[0, i], w_k[1:4, i]).euler_angles()
        roll[i] = quatToEul[0]
        pitch[i] = quatToEul[1]
        yaw[i] = quatToEul[2]

    quatToEul = Quaternion(w_k[0, 0], w_k[1:4, 0]).euler_angles()
    roll[0] = quatToEul[0]
    pitch[0] = quatToEul[1]
    yaw[0] = quatToEul[2]


        # Find mean of transformed points


    # roll, pitch, yaw are numpy arrays of length T
    return roll,pitch,yaw
estimate_rot()



