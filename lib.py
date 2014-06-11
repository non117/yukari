# -*- coding: utf-8 -*-
import csv

class Vector():
    def __init__(self, t, x, y, z):
        self.t = float(t)
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

JOINT_NAMES = {
    0:'Head', 1:'Neck', 2:'Torso',3:'LeftShoulder', 4:'LeftElbow', 5:'LeftHand',
    6:'RightShoulder', 7:'RightElbow', 8:'RightHand',
    9:'LeftHip', 10:'LeftKnee', 11:'LeftFoot',
    12:'RightHip', 13:'RightKnee', 14:'RightFoot',
}

class Joint():
    def __init__(self, joint_name):
        self.joint_name = joint_name
        self.vector_sequence = []
    def add_data(self, t, x, y, z):
        vector = Vector(t, x, y, z)
        self.vector_sequence.append(vector)

def csv_to_joint(filename):
    reader = csv.reader(open(filename, 'rb'))
    joints = []
    for i in range(15):
        joints.append(Joint())
    for n, row in enumerate(reader):
        if n == 0: continue
        time = row[0]
        data = row[1:]
        for i in range(15): #15個の関節
            x, y, z = data[i*3:(i+1)*3] #x,y,zを3つずつ切り出し
            joints[i].add_data(time, x, y, z)
    return joints

