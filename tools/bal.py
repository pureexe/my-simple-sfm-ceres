"""
Convert colmap into BAL format
"""
from database import COLMAPDatabase,blob_to_array, pair_id_to_image_ids
import argparse
import numpy as np
from operator import attrgetter

def main(args):
    db = COLMAPDatabase.connect(args.input)
    c = db.cursor()
    c.execute("SELECT pair_id,rows,cols,data FROM matches")
    matches_data = c.fetchall()
    c.execute("SELECT image_id,rows,cols,data FROM keypoints")
    keypoints_data = c.fetchall()
    db.close()
    
    total_image = len(keypoints_data)
    output_track = []
    match_from = {}
    match_search = {}
    keypoint_search = {}
    total_image_list = []
    keypoint_counter = 0
    for keypoint_image in keypoints_data:
        img_id, r, c, data = keypoint_image
        kpt = blob_to_array(data,np.float32,(r,c))
        image_kpt = {}
        for i in range(r):
            u,v =  kpt[i,:2]
            image_kpt[i] = [u,v]
        keypoint_search[img_id] = image_kpt

    for match_record in matches_data:
        image_from , image_to = pair_id_to_image_ids(match_record[0])
        image_from = int(image_from)
        total_image_list.append(image_from)
        if match_record[1] != 0:
            if not image_from in match_from:
                match_from[image_from] = []
            match_from[image_from].append(match_record[0])
            lookup = blob_to_array(match_record[3],np.int32, (match_record[1],match_record[2]))
            match_search[match_record[0]] = dict(lookup)

    for image_from, records in match_from.items():
        for kpt_id, [x,y] in keypoint_search[image_from].items():
            buffer = []
            buffer.append({
                'image': image_from,
                'x': x,
                'y': y,
                'kpt_id': keypoint_counter
            })
            for record in records:
                if kpt_id in match_search[record]:
                    _, image_to = pair_id_to_image_ids(record)
                    kpt_on_dest = match_search[record][kpt_id]
                    x,y = keypoint_search[image_to][kpt_on_dest]
                    buffer.append({
                        'image': image_to,
                        'x': x,
                        'y': y,
                        'kpt_id': keypoint_counter
                    })
            if len(buffer) != 1:
                output_track = output_track + buffer
                keypoint_counter = keypoint_counter + 1
                    
    # write output
    with open('feature_matching.txt','w') as f:
        f.write('%d %d %d\n' %( total_image,keypoint_counter, len(output_track)))
        for o in output_track:
            f.write('%d %d     %lf %lf\n' % (o['image']-1,o['kpt_id'], o['x'], o['y']) )
        if args.zero_polyfill:
            # zero polyfill for backward compatibility
            for i in range(3 * keypoint_counter + total_image * 9):
                f.write('0\n')
        else:
            random_value = np.random.normal(0,1,3 * keypoint_counter + total_image * 9)
            for value in random_value:
                f.write('{}\n'.format(value))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='bal.py - convert colmap input to bal style input')
    parser.add_argument('-i', '--input', default='penguinguy_cam004_matched.db', type=str,
        help='database file that already do feature matching')
    parser.add_argument('-0', '--zero_polyfill', action='store_true')
    main(parser.parse_args())
