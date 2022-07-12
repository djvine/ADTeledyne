#!/usr/bin/env python3
"""
Script to convert the Teledyne format defect correction map to the json format expected by area detector.
"""

import json
import sys

def parse_defect_map(fname):
    with open(fname, 'r') as f:
        lines = f.readlines()

    begin = False
    pixels = []
    columns = []
    for line in lines:
        line=line.strip()
        if line.find('$begin')>-1:
            begin = True
        elif begin:
            if line[0]=='P': # Pixel
                pixels.append([int(val)-1 for val in line[1:].split(',')])
            elif line[0]=='C': # Column
                columns.append([
                    int(line[1:].split(' ')[0])-1,
                    int(line[1:].split(' ')[1].split('-')[0])-1,
                    int(line[1:].split(' ')[1].split('-')[1])-1,
                ])
    return pixels, columns

if __name__=='__main__':
    defect_map_filename = sys.argv[1]

    pixels, columns = parse_defect_map(defect_map_filename)

    replace_mode = 'Median'
    # Median kernel is [2NX+1, 2NY+1]
    kernel_size1 = [1, 1]
    kernel_size2 = [1, 0]

    json_content1 = {}
    json_content2 = {}
    json_content1['Bad pixels'] = []
    json_content2['Bad pixels'] = []
    for pixel in pixels:
        json_content1['Bad pixels'].append(
            {
                'Pixel': [pixel[0], pixel[1]],
                replace_mode: kernel_size1,
            }
        )
        json_content2['Bad pixels'].append(
            {
                'Pixel': [pixel[0], pixel[1]],
                'Set' : 65000,
            }
        )
    for column in columns:
        for i in range(column[1], column[2]+1):
            json_content1['Bad pixels'].append(
                {
                    'Pixel': [column[0], i],
                    replace_mode: kernel_size2,
                }
            )
            json_content2['Bad pixels'].append(
                {
                    'Pixel': [column[0], i],
                    'Set': 65000,
                }
            )

    with open(defect_map_filename+'.json', 'w') as f:
        f.writelines(json.dumps(json_content1, indent=4, sort_keys=True))
    with open(defect_map_filename+'_ID.json', 'w') as f:
        f.writelines(json.dumps(json_content2, indent=4, sort_keys=True))
