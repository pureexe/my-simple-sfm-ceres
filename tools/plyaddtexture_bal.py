"""
Convert point3d in numpy format into ply
"""
from database import COLMAPDatabase
import numpy as np
import argparse

def main(args):
    point3d = []
    image_id_index = []
    with open(args.input,'r') as f:
        # remove header
        while f.readline().strip() != 'end_header':
            pass
        line = f.readline().strip()
        while line != '':
            point = line.split(' ')
            point3d.append([float(point[0]),float(point[1]),float(point[2])])
            line = f.readline().strip()

    with open(args.bal, 'r'):
        point_count = 0
        line = f.readline().strip()
        print(line)
        exit()
        

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='point3dnpy2ply.py - Convert point3d in numpy format into ply')
    parser.add_argument('-i', '--input', type=str, help='input ply', required=True)
    parser.add_argument('-d', '--directory', type=str, help='image directory', required=True)
    parser.add_argument('-c', '--colmap', type=str, help='colmap database for lookup', required=True)
    parser.add_argument('-b', '--bal', type=str, help='bal format file for lookup', required=True)
    parser.add_argument('-o', '--output', type=str, help='output ply file', required=True)
    main(parser.parse_args())

# python .\plyaddtexture_bal.py -i ../penguin_ceres.ply -o ../penguin_ceres_color.ply -c penguinguy_cam004_matched.db -d 'D:\\Datasets\\penguinguy_cam004' -b .\penguin_feature_matching.txt