#!/usr/bin/env python
# coding=utf-8

import xml.etree.ElementTree as et
import argparse
import linecache

# xmlメモ
# <タグ 属性1="value1" 属性2="value2">     ⇠親
#     <子タグ2 属性3="value3">text</タグ2>  ⇠子
# </タグ>


def main():
    print "Hello World!"
    p = argparse.ArgumentParser(description="vicon用オブジェクトの編集（平行移動のみ）")
    p.add_argument("filename")
    p.add_argument("out_file")
    p.add_argument("x")
    p.add_argument("y")
    p.add_argument("z")

    args = p.parse_args()
    offset = {}
    offset["x"] = float(args.x)
    offset["y"] = float(args.y)
    offset["z"] = float(args.z)

    tree = et.parse(args.filename)
    root = tree.getroot()
    # params = root[0]
    # for child in params:
    #     print child.attrib["PRIOR"]

    # markerset = root[2]
    # markers = markerset[0]
    # for child in markers:
    #     print child.attrib["POSITION"]

    for marker in root.iter("Marker"):
        pos = map(float, marker.attrib["POSITION"].split(" "))
        print pos
        pos[0] += offset["x"]
        pos[1] += offset["y"]
        pos[2] += offset["z"]
        marker.attrib["POSITION"] = "{0} {1} {2}".format(pos[0], pos[1], pos[2])

    for parameter in root.iter("Parameter"):
        axis = parameter.attrib["NAME"][-1]
        old_val = float(parameter.attrib["VALUE"])
        parameter.attrib["PRIOR"] = str(old_val + offset[axis])
        parameter.attrib["VALUE"] = str(old_val + offset[axis])

    for parameter in root.iter("StaticParameter"):
        axis = parameter.attrib["NAME"][-1]
        old_val = float(parameter.attrib["VALUE"])
        parameter.attrib["PRIOR"] = str(old_val + offset[axis])
        parameter.attrib["VALUE"] = str(old_val + offset[axis])
        # print parameter.attrib["NAME"]

    # xmlファイル生成
    tree.write(args.out_file, xml_declaration=None)

    # 特殊なヘッダーしてるせいでライブラリから書けなかった．手動で書き足す．
    with open(args.out_file, "r+") as f:
        old = f.read()
        f.seek(0)
        f.write('<?xml version="1.0" encoding="UTF-8" standalone="no" ?>\n\n')
        f.write(old)


if __name__ == '__main__':
    main()
