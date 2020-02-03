"""
Convert point3d in numpy format into ply
"""
import numpy as np
import argparse

def write_heade(number):
    """
    ply
    format ascii 1.0
    element vertex 
    property float x
    property float y
    property float z
    property uchar red
    property uchar green
    property uchar blue
    end_header
    """

def main(args):
    point3d = np.load(args.input)
    with open(args.output,'w') as f:
        # Write header
        f.write('ply\n')
        f.write('format ascii 1.0\n')
        f.write('element vertex {:d}\n'.format(point3d.shape[0]))
        f.write('property float x\n')
        f.write('property float y\n')
        f.write('property float z\n')
        f.write('property uchar red\n')
        f.write('property uchar green\n')
        f.write('property uchar blue\n')
        f.write('end_header\n')
        # write point
        for point in point3d:
            f.write('{} {} {} 255 255 255\n'.format(point[0],point[1],point[2]))
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='point3dnpy2ply.py - Convert point3d in numpy format into ply')
    parser.add_argument('-i', '--input', type=str, help='input numpy file', required=True)
    parser.add_argument('-o', '--output', type=str, help='output ply file', required=True)
    main(parser.parse_args())
