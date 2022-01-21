#!/usr/bin/env python
# -*- coding:utf-8 -*-

from os import path
import xml

import rospy
from tms_msg_ss.msg import vicon_data


class OffsetManager:

    def __init__(self, index):
        self.ip_index = index
        self.xml_file = path.normpath(
            path.join(path.dirname(path.abspath(__file__)),
                      '../data/offset' + str(self.ip_index + 1) + '.xml')
        )
        self.translation = [0.0, 0.0, 0.0]  # x,y,z
        self.rotation = [0.0, 0.0, 0.0, 1.0]  # x,y,z,w

    def __getFromVicon(self, msg):
        if msg.subjectName == 'checker_board':
            print('Get offset from vicon_stream: \
                  Name: {0}'.format(msg.subjectName))
            self.sub_offset.unregister()

            # Store offset to XML file
            offset_data = xml.etree.ElementTree.Element('extrinsic_parameter')
            translation = xml.etree.ElementTree.SubElement(offset_data,
                                                           'translation')
            rotation = xml.etree.ElementTree.SubElement(offset_data,
                                                        'rotation')
            translation_names = ['tx', 'ty', 'tz']
            rotation_names = ['qx', 'qy', 'qz', 'qw']
            param_translation = [
                xml.etree.ElementTree.Element('param', name=i)
                for i in translation_names
            ]
            param_rotation = [
                xml.etree.ElementTree.Element('param', name=i)
                for i in rotation_names
            ]
            param_translation[0].text = str(msg.translation.x / 1000.0)
            param_translation[1].text = str(msg.translation.y / 1000.0)
            param_translation[2].text = str(msg.translation.z / 1000.0)
            param_rotation[0].text = str(msg.rotation.x)
            param_rotation[1].text = str(msg.rotation.y)
            param_rotation[2].text = str(msg.rotation.z)
            param_rotation[3].text = str(msg.rotation.w)
            translation.extend(param_translation)
            rotation.extend(param_rotation)
            file_handler = open(self.xml_file, 'w')
            file_handler.write(
                xml.dom.minidom.parseString(
                    xml.etree.ElementTree.tostring(offset_data, 'utf-8')
                ).toprettyxml(indent='  ')
            )
            file_handler.close()

    def run(self):
        rospy.init_node('get_offset' + str(self.ip_index + 1))
        self.sub_offset = rospy.Subscriber('vicon_stream/output',
                                           vicon_data, self.__getFromVicon)
        rospy.spin()

    def read(self):
        read_xml = xml.dom.minidom.parse(self.xml_file)
        translation = read_xml.getElementsByTagName('translation')[0]
        rotation = read_xml.getElementsByTagName('rotation')[0]
        for i in translation.getElementsByTagName('param'):
            if i.getAttribute('name') == 'tx':
                self.translation[0] = float(i.childNodes[0].data)
            if i.getAttribute('name') == 'ty':
                self.translation[1] = float(i.childNodes[0].data)
            if i.getAttribute('name') == 'tz':
                self.translation[2] = float(i.childNodes[0].data)
        for i in rotation.getElementsByTagName('param'):
            if i.getAttribute('name') == 'qx':
                self.rotation[0] = float(i.childNodes[0].data)
            if i.getAttribute('name') == 'qy':
                self.rotation[1] = float(i.childNodes[0].data)
            if i.getAttribute('name') == 'qz':
                self.rotation[2] = float(i.childNodes[0].data)
            if i.getAttribute('name') == 'qw':
                self.rotation[3] = float(i.childNodes[0].data)
        return (self.translation, self.rotation)
