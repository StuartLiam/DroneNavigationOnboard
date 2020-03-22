
#imports
import logging
import math
import sys

import numpy as np
from vispy import scene
from vispy.scene import visuals
from vispy.scene.cameras import TurntableCamera

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

import matplotlib.pyplot as plt
import pandas as pd
from scipy.signal import lfilter
from scipy.signal import find_peaks
import tensorflow as tf

from tensorflow import keras
from tensorflow.keras import layers
import datetime as dt
from sklearn import preprocessing


time = dt.datetime.now()
timeStamp = time.strftime("_%H_%M_%S")


#some start coords

#some end coords

#safe takeoff

#while not at goal

#do dikstras
class Point:
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z

    def print(self):
        print("X: {} Y: {} Z: {}".format(self.x,self.y,self.z))

class Block2D:
    def __init__(self,maxX,minX,maxY,minY):
        self.maxX = maxX
        self.minX = minX
        self.maxY = maxY
        self.minY = minY
        self.isFilled = False


    def print(self):
        print("{} long by {} tall".format(self.maxX-self.minX,self.maxY-self.minY))



#ZA WOURLDO
class World2D(Block2D):
    def __init__(self,maxX,minX,maxY,minY):
        super(World2D,self).__init__(maxX,minX,maxY,minY)
        self.blocks = [Block2D(maxX,minX,maxY,minY)]
    


class DNode:
    def __init__(self,points,loc):
        super().__init__()
        self.points = points
        self.conNodes = []
        self.loc = loc

    def addNode(self,node):
        self.conNodes.append(node)
        print(self.conNodes)

    def rmNode(self,Node):
        self.conNodes.remove(Node)

    def print(self):
        print("Location: ")
        self.loc.print()

class DroneActor:
    def __init__(self):
        super().__init__()
    



NodeOne = Point(1,1,1)
nodeTest = DNode(None,NodeOne)
nodeTwo = DNode(None,NodeOne)
nodeTest.addNode(nodeTwo)

print("test")
nodeTest.print()

print("Creating world block!")

World = World2D(10,0,10,5)
World.print()


time = dt.datetime.now()
timeStamp = time.strftime("_%H_%M_%S")

#all of our files

class_names = ['back', 'enter', 'forward', 'left', 'right']


CHUNK_SIZE = 8
#A smoothing function that makes finding peaks easier





#======================
#
#   I could not write a clean way to process our data in to a single file
#   thus the following 5 loops compile the data the long way
#
#======================





def preprocess(x, y):
  x = tf.cast(x, tf.float32)
  y = tf.cast(y, tf.int64)

  return x, y

def create_dataset(xs, ys, n_classes=5):
  ys = tf.one_hot(ys, depth=n_classes)
  return tf.data.Dataset.from_tensor_slices((xs, ys)) \
    .map(preprocess) \
    .shuffle(len(ys)) \
    .batch(8)





model = keras.Sequential([
    keras.layers.Dense(units=1, input_shape=(272,)),
    keras.layers.Dense(units=64, activation='relu'),
    keras.layers.Dense(units=32, activation='relu'),
    keras.layers.Dense(units=32, activation='relu'),
    keras.layers.Dense(units=16, activation='relu'),
    keras.layers.Dense(units=5, activation='softmax')
])


# Train the model, iterating on the data in batches of 32 samples
model.compile(optimizer='adamax', 
            loss=tf.losses.CategoricalCrossentropy(from_logits=True),
            metrics=['mean_absolute_error','accuracy'])

# history = model.fit(
#     train_dataset.repeat(), 
#     epochs=1600, 
#     steps_per_epoch=200,
#     validation_data=val_dataset.repeat(), 
#     validation_steps=20,
#     verbose = 1
# )
# predictions = model.predict(val_dataset)

# print(test_res)
# for pred in predictions:
#     print(class_names[np.argmax(pred)])

model.summary()