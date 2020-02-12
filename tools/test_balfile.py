import numpy as np
from scipy.spatial.transform import Rotation

# https://github.com/kashif/ceres-solver/blob/master/include/ceres/rotation.h#L457
def angle_axis_rotate_point(angle_axis, point3d):
    theta2 = np.dot(angle_axis,angle_axis)
    w = np.zeros(3)
    result = np.zeros(3)
    if theta2  > 0:
        theta = np.sqrt(theta2)
        w[0] = angle_axis[0] / theta
        w[1] = angle_axis[1] / theta
        w[2] = angle_axis[2] / theta
        costheta = np.cos(theta)
        sintheta = np.sin(theta)
        w_cross_pt = np.cross(w,point3d)
        w_dot_pt = np.dot(w,point3d)
        for i in range(3):
            result[i] = point3d[i] * costheta + w_cross_pt[i] * sintheta + w[i] * (1.0 -costheta) * w_dot_pt
    else:
        w_cross_pt = np.cross(angle_axis,point3d)
        for i in range(3):
            result[i] = point3d[i] + w_cross_pt[i]
    return result

def get_observe(bal_file = 'temple-bal-with-distrot.txt',observe_index = 0):
    camera_params = []
    point3d = []
    with open(bal_file,'r') as f:
        headline = f.readline()
        camera_number, point_number, observation_number = headline.strip().split(' ')
        camera_number = int(camera_number)
        point_number = int(point_number)
        observation_number = int(observation_number)
        # read first line
        for i in range(observe_index + 1):
            observe_line = f.readline()
        observe_line = observe_line.strip().split(' ')
        camera, point, observation_x, observation_y = observe_line
        camera = int(camera)
        point = int(point)
        # skip unuse data
        for i in range(observation_number - 1 - observe_index):
            _ = f.readline()
        # seek for camera
        for i in range(camera):
            for j in range(9):
                _ = f.readline()
        # save camera data
        camera_params = []
        for i in range(9):
            d = f.readline().strip()
            camera_params.append(float(d))
        # seek to point
        for i in range(camera_number - camera - 1):
            for j in range(9):
                _ = f.readline()
        # seek for point
        for i in range(point):
            for j in range(3):
                _ = f.readline()
        # get point
        for i in range(3):
            d = f.readline().strip()
            point3d.append(float(d))
    observe_point = (float(observation_x),float(observation_y))
    return camera_params, point3d, observe_point

def ceres_projection(camera_params, point3d):
    camera_params = np.asarray(camera_params)
    point3d = np.asarray(point3d)
    angle_axis = camera_params[:3]
    p = angle_axis_rotate_point(angle_axis,point3d) #1.98909 0.609712 1.26013

    p[0] = p[0] + camera_params[3]
    p[1] = p[1] + camera_params[4]
    p[2] = p[2] + camera_params[5]
    print(p)
    xp = p[0] / p[2]
    yp = p[1] / p[2]
    l1 = camera_params[7]
    l2 = camera_params[8]
    r2 = xp*xp + yp*yp
    distortion = 1.0 + r2  * (l1 + l2  * r2)
    focal = camera_params[6]
    predicted_x = focal * distortion * xp + 320
    predicted_y = focal * distortion * yp + 240
    return predicted_x, predicted_y

def putin_projection(camera_params, point3d, px=320, py=240):
    camera_params = np.asarray(camera_params)
    point3d = np.asarray(point3d)
    rotvec = camera_params[:3]
    tvec = camera_params[3:6]
    focal = camera_params[6]
    rotate = Rotation.from_rotvec(rotvec)
    rotation_matrix = rotate.as_dcm()
    intrinsic = np.array([
        [           focal,                 0,        px],
        [               0,             focal,        py],
        [               0,                 0,          1]
    ])
    extrinsic = np.zeros((4,4))
    extrinsic[:3, :3] = rotation_matrix
    print(rotation_matrix)
    extrinsic[:3, 3] =  tvec
    extrinsic[3, 3] = 1
    position = np.ones((4,1))
    position[:3,0] = point3d
    ppp = np.matmul(extrinsic, position)[:3,:]
    projected = np.matmul(intrinsic,ppp)
    print(ppp)
    projected = (projected / projected[2,:]).T
    return projected[0,0], projected[0,1]



if __name__ == '__main__':
    camera, point3d, point2d = get_observe(observe_index=0)
    #predicted_x, predicted_y = putin_projection(camera, point3d)
    predicted_x, predicted_y  = ceres_projection(camera, point3d)
    print("observe: %f, %f" % (point2d[0],point2d[1]))
    print("predict: %f, %f" % (predicted_x,predicted_y))

#เอาลบออก แก้ principle point