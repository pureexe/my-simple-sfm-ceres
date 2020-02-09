"""
Convert colmap sparse resconstruction into BAL format
"""
import argparse, os
import numpy as np
from read_write_model import read_model
from database import COLMAPDatabase, blob_to_array, pair_id_to_image_ids

point3ds_id = []
pint2ds_data = []

def keypoint_decode(kpt):
    img_id, r, c, data = kpt
    data = blob_to_array(data, np.float32, (r,c) )
    data = {i: data[i] for i in range(len(data))}
    return (img_id, data)

def get_keypoint_from_db(db_path):
    db = COLMAPDatabase.connect(db_path)
    c = db.cursor()
    c.execute("SELECT image_id,rows,cols,data FROM keypoints")
    keypoints_data = list(map(keypoint_decode,c.fetchall()))
    db.close()
    keypoint_lookup = {}
    for img_id, kpt_data in keypoints_data:
        keypoint_lookup[img_id] = kpt_data
    return keypoint_lookup 



def main(args):
    point2d_lookup = get_keypoint_from_db(args.database)
    cameras, images, points3d = read_model(args.sparse,'.bin')
    point_len = 0
    # build 9 camera parameter
    print(images)
    exit()
    # build point data
    for point_id in points3d:
        pid, xyz, rgb, err, img_ids, kpt_ids = points3d[point_id]
        point3d_ids.append(pid)
        for i in range(len(img_ids)):
            pint2ds_data.append({
                'id': point_len,
                'point2d':point2d_lookup[img_ids[i]][kpt_ids[i]]
            })
        point_len = point_len + 1



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='colmap2bal.py - convert colmap into bal format')
    parser.add_argument('-d', '--database', type=str, help='colmap database', default='D:\\Datasets\\temple\\colmap\\temple_matched.db')
    parser.add_argument('-s', '--sparse', type=str, help='spare reconstriction directory', default='D:\\Datasets\\temple\\colmap')
    parser.add_argument('-o', '--output', type=str, help='output bal file', default='temple-bal.txt')
    main(parser.parse_args())
