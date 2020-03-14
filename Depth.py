import numpy as np
import cv2 as cv

class Depth_Map:
       def __init__(self, point_cloud):

        print("Constructor called")
        self.Map_ = np.zeros((121,161,121))         #121 layers, 161 rows, 121 cols
        self.vox_res = 5                            #voxel resolution is 5 cm
        #self.point_cloud = point_cloud
 
        if  point_cloud.shape[0] == 0:
            print("No Point Cloud")

        elif point_cloud.shape[0]>0:
 
           for i in range(point_cloud.shape[0]):
              x = int(60 - round(point_cloud[i][0]*100/self.vox_res))  #side  or column
              y = int(60 - round(point_cloud[i][1]*100/self.vox_res))  #layer 
              z = int(160-round(point_cloud[i][2]*100/self.vox_res))  #depth or row             
              self.Map_[y][z][x] = 1


                     
       def clearMap(self):
            self.Map_ = np.zeros((121,161,121))

