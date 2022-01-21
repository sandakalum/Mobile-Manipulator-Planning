#!/usr/bin/env python
# coding: utf-8
#
# Author:   Kazuto Nakashima
# URL:      https://github.com/kazuto1011
# Created:  2016-06-07

import socket
import datetime
import time
import xml.etree.ElementTree as ET

from config import config

#-------------------------------------------------------------------------
# A class to monitor user's sleep state continuously
class NemuriScanner():

    def __init__(self, config):
        self.xml = config['xml_file']
        self.address = (config['server_IP'], config['PORT'])
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect(self.address)
        print 'NemuriScanner launched!'

    def run(self):
        while True:
            self.sleep_state = self.parse()
            self.result = self.call(self.sleep_state)
            print datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f:"), self.result
            time.sleep(0.5)

    def parse(self):
        root = ET.parse(self.xml).getroot()
        return root.find('.//State').text

    def call(self, msg):
        self.client.send(msg)
        return self.client.recv(4096)

#-------------------------------------------------------------------------
node = NemuriScanner(config)
node.run()
