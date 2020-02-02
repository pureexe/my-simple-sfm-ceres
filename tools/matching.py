from database import COLMAPDatabase,blob_to_array, pair_id_to_image_ids
import argparse

def main(args):
    db = COLMAPDatabase.connect(args.input)
    c = db.cursor()
    c.execute("SELECT pair_id,rows,cols,data FROM matches LIMIT 1")
    data = c.fetchall()
    print(data)
    db.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='matching.py - convert colmap input to sfm ready input')
    parser.add_argument('-i', '--input', default='penguinguy_cam004_matched.db', type=str,
        help='database file that already do feature matching')
    main(parser.parse_args())
