#!/usr/bin/python
# -*- coding: utf-8 -*-
import pymongo
from collections import OrderedDict
import time

client = pymongo.MongoClient('localhost',27017)
db = client.rostmsdb
co = db.default
default_list = list(co.find().sort('id'))
print default_list
default_list_new=[]
for old in default_list:
    new = OrderedDict()
    new['id'] = old['id']
    new['name'] = old['name']
    new['type'] = old['type']
    new['offset_x'] = old['offset_x']
    new['offset_y'] = old['offset_y']
    new['offset_z'] = old['offset_z']
    new['weight'] = old['weight']
    new['rfid'] = old['rfid']
    new['etcdata'] = old['etcdata']
    new['place'] = old['place']
    new['note'] = old['note']
    new['tag'] = old['tag']
    if 'announce' in old:
        new['announce'] = old['announce']
    print new
    default_list_new.append(new)
co.remove()
co.insert(default_list_new)

co = db.now
now_list = list(co.find().sort('id'))
print now_list
now_list_new=[]
for old in now_list:
    new = OrderedDict()
    new['time'] = old['time']
    new['id'] = old['id']
    new['name'] = old['name']
    new['type'] = old['type']
    new['x'] = old['x']
    new['y'] = old['y']
    new['z'] = old['z']
    new['rr'] = old['rr']
    new['rp'] = old['rp']
    new['ry'] = old['ry']
    new['offset_x'] = old['offset_x']
    new['offset_y'] = old['offset_y']
    new['offset_z'] = old['offset_z']
    new['joint'] = old['joint']
    new['weight'] = old['weight']
    new['rfid'] = old['rfid']
    new['etcdata'] = old['etcdata']
    new['place'] = old['place']
    new['extfile'] = old['extfile']
    new['sensor'] = old['sensor']
    new['probability'] = old['probability']
    new['state'] = old['state']
    new['task'] = old['task']
    new['note'] = old['note']
    new['tag'] = old['tag']
    print new
    now_list_new.append(new)
co.remove()
co.insert(now_list_new)
