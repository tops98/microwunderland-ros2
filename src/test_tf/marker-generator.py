### Generate markers
import cv2
from cv2 import aruco
import os
import argparse
from pathlib import Path
import json
import datetime

dict_choices = [
    	"DICT_4X4_50",
		"DICT_4X4_100",
		"DICT_4X4_250",
		"DICT_4X4_1000",
		"DICT_5X5_50",
		"DICT_5X5_100",
		"DICT_5X5_250",
		"DICT_5X5_1000",
		"DICT_6X6_50",
		"DICT_6X6_100",
		"DICT_6X6_250",
		"DICT_6X6_1000",
		"DICT_7X7_50",
		"DICT_7X7_100",
		"DICT_7X7_250",
		"DICT_7X7_1000",
		"DICT_ARUCO_ORIGINAL",
		"DICT_APRILTAG_16h5",
		"DICT_APRILTAG_25h9",
		"DICT_APRILTAG_36h10",
		"DICT_APRILTAG_36h11",
	]

def check_positive(value):
    ivalue = int(value)
    if ivalue < 0:
        raise argparse.ArgumentTypeError("%s is an invalid positive int value" % value)
    return ivalue

parser = argparse.ArgumentParser(
    prog='Arucu-Marker Generator',
    description="CLI Tool for generating arucu-markers using open cv's default implementation",
    epilog="example usage:\
    marker-generator.py /path/to/export/dir -d DICT_4X4_50 -c 10")

parser.add_argument('-f','--export_directory',type=str,default="./markers")
parser.add_argument('-d', '--dictionary_type', type=str, default="DICT_4X4_100", choices=dict_choices)
parser.add_argument('-c', '--count', type=check_positive, default=1)
parser.add_argument('-i', '--start_id', type=check_positive, default=0)
parser.add_argument('-s', '--image_size', type=check_positive, default=128)

args = parser.parse_args()

print(f"{args.start_id + args.count} >{int(str(args.dictionary_type).split('_')[-1])}")
if args.start_id + args.count > int(str(args.dictionary_type).split("_")[-1]):
    print("your start index in conjunction with the number of desired markers, exceeds the range of the selected dictionary!")
    quit()

dict_aruco = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
export_path = os.path.join(args.export_directory,args.dictionary_type)
Path(export_path).mkdir(parents=True, exist_ok=True)

# create markers
for count in range(args.start_id,args.count) :

    id_mark = count
    img_mark = aruco.generateImageMarker(dict_aruco,id_mark,args.image_size)

    if count < 10 :
        img_name_mark = 'mark_id_0' + str(count) + '.jpg'
    else :
        img_name_mark = 'mark_id_' + str(count) + '.jpg'
    path_mark = os.path.join(export_path, img_name_mark)

    cv2.imwrite(path_mark, img_mark)

# create json file with configuzration details of the generated markers
config_path = os.path.join(export_path, "marker_config.json")
config_dict = vars(args)
config_dict["created"] = str(datetime.datetime.now())
config_dict.pop("export_directory")
with open(config_path, '+w') as file:
    json.dump(vars(args),file)
    