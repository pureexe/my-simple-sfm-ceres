"""
Convert colmap sparse resconstruction into BAL format
"""
import argparse, os
import numpy as np
from read_write_model import read_model
from scipy.spatial.transform import Rotation

def main(args):
    cameras, images, points3d = read_model(args.input,'.bin')
    observation_info = [] #format: [camera_id, point3d_id, x, y]
    camera_info = [] # format: [c1, ... c9]
    point_info = [] # format: [x,y,z]
    point3d_lookup = {}
    focal_lookup = {}
    # build point data
    point_len = 0
    for point_id in points3d:
        pid, xyz, rgb, err, img_ids, kpt_ids = points3d[point_id]
        point3d_lookup[pid] = point_len
        point_info.append(xyz)
        point_len = point_len + 1
    #build camera lookup
    for cam_id in cameras:
        cid, model, w, h, params = cameras[cam_id]
        focal_lookup[cid] = params[0] 
    # build observe data
    for image_id in images:
        img_id, qvec, tvec, camera_id, name, xys, point3d_ids = images[image_id]
        for i in range(len(xys)):
            if point3d_ids[i] != -1:
                x,y = xys[i]
                observation_info.append([
                    camera_id-1,
                    point3d_lookup[point3d_ids[i]],
                    x,
                    y
                ])
        rotation =  Rotation.from_quat([qvec[1],qvec[2],qvec[3],qvec[0]])
        rotvec = rotation.as_rotvec()
        camera_feature = [
            rotvec[0],
            rotvec[1],
            rotvec[2],
            tvec[0],
            tvec[1],
            tvec[2],
            focal_lookup[camera_id],
            0, # assume no distrotion
            0, # assume no distrotion
        ]
        camera_info.append(camera_feature)
    total_camera = len(cameras)
    total_point = len(points3d)
    total_observe = len(observation_info)
    # sort by point3d and camera
    observation_info = sorted(observation_info, key = lambda x: (x[1], x[0]))
    #write output
    with open(args.output,'w') as f:
        f.write("{:d} {:d} {:d}\n".format(total_camera, total_point, total_observe))
        for o in observation_info:
            f.write('{} {} {} {}\n'.format(o[0],o[1],o[2],o[3]))
        for cam in camera_info:
            for p in cam:
                f.write('{}\n'.format(p))
        for point in point_info:
            for p in point:
                f.write('{}\n'.format(p))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='colmap2bal.py - convert colmap into bal format')
    parser.add_argument('-i', '--input', type=str, help='spare reconstriction directory', default='D:\\Datasets\\temple\\colmap') #\\colmap_undistrotion\\sparse
    parser.add_argument('-o', '--output', type=str, help='output bal file', default='test.txt')
    main(parser.parse_args())
