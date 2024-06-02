import sqlite3
import argparse


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--database_path", required=True) # path to database.db
    parser.add_argument("--match_list_path", required=True) # filename of the output match list, eg. match_pairs.txt
    parser.add_argument("--min_num_matches", type=int, default=15) # minimum value deciding whether two images are matched
    args = parser.parse_args()
    return args


def pair_id_to_image_ids(pair_id):
    image_id2 = pair_id % 2147483647
    image_id1 = (pair_id - image_id2) / 2147483647
    return image_id1, image_id2


def main():
    args = parse_args()

    connection = sqlite3.connect(args.database_path)
    cursor = connection.cursor()

    # Get a mapping between image ids and image names
    image_id_to_name = dict()
    cursor.execute("SELECT image_id, name FROM images;")
    for row in cursor:
        image_id = row[0]
        name = row[1]
        image_id_to_name[image_id] = name

    # Iterate over entries in the two_view_geometries table
    output_id_txt = open(args.match_list_path+'_id.txt', "w")
    output_name_txt = open(args.match_list_path+'_name.txt', "w")
    cursor.execute("SELECT pair_id, rows FROM two_view_geometries;")
    
    match_static = [];
    for row_static in cursor:
        rows = row_static[1]
        match_static.append(rows)
    everage_match = int(0.1 * sum(match_static) / len(match_static))
    #print(everage_match)
    
    cursor.execute("SELECT pair_id, rows FROM two_view_geometries;")
    for row in cursor:
        pair_id = row[0]
        rows = row[1]
        
        #print(rows)
        if rows < max(everage_match, args.min_num_matches):
            continue

        image_id1, image_id2 = pair_id_to_image_ids(pair_id)
        image_name1 = image_id_to_name[image_id1]
        image_name2 = image_id_to_name[image_id2]

        output_id_txt.write("%d %d %d\n" % (image_id1, image_id2, rows))
        output_name_txt.write("%s %s %d\n" % (image_name1, image_name2, rows))

    output_id_txt.close()
    output_name_txt.close()
    cursor.close()
    connection.close()


if __name__ == "__main__":
    main()
    # command line example: python export_match_relationship.py --database_path database.db --match_list_path match_pair --min_num_matches 20