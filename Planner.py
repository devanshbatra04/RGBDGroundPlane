#read the point cloud
#fill the depth map
#fill the angle histograms
#get the available sectors with the steering angles
#pass the data to the controller

import numpy as np
import math

class Planner:

	def __init__(self):
          self.Map_        = np.zeros((121,161,121))
          self.vox_res     = 5.0
          self.angle_res   = 10.0

          self.H_left  = np.zeros(10)
          self.H_right = np.zeros(10)
          self.V_up    = np.zeros(10)
          self.V_down  = np.zeros(10)
          self.sectors = []
          self.grid    = np.zeros((850,640))
  

	def fill_Map(self, point_cloud):
            for i in range(point_cloud.shape[0]):
                x = int(round(point_cloud[i][0]*100/self.vox_res))  #side  or column
                y = int(round(point_cloud[i][1]*100/self.vox_res))  #layer 
                z = int(round(point_cloud[i][2]*100/self.vox_res))  #depth or row             
                self.Map_[60 + y][160 - z][60 + x] = 1
                if y == 0:
                   self.grid[(160-z)][(60+x)] = 1
                #now corresponding to these points, fill the angle histogram
                h_angle = math.degrees(math.atan((x)/z))   #horizontal angle
                #print(h_angle)
                if abs(h_angle)<=45.0:
                 if h_angle < 0:
                   self.H_left[int(abs(h_angle)/self.angle_res)] =  1

                 elif h_angle >= 0:
                   self.H_right[int(h_angle/self.angle_res)]      =  1  


                v_angle = math.degrees(math.atan((y)/z))   #vertical angle
               
                if abs(v_angle)<=45.0:      
                   if v_angle < 0:
                      self.V_down[int(abs(v_angle)/self.angle_res)]  =  1
               
                   elif v_angle >= 0:                 
                      self.V_up[int(v_angle/self.angle_res)]        =  1 


	def getSectors(self, hist,s):    
            point = [0,0]   #size of the sector and the steering angle for that    
            count = 0
            prev_index = 0    
            curr_index = 0
            for i in range(hist.shape[0]):
                if hist[i] == 1:
                   count = 0
                   continue

                elif hist[i] == 0:
                     prev_index = i
                     count = count + 1
                     #now check the next indices
                     for j in range(i,hist.shape[0]):

                         if hist[j] != 0:
                            point[0] = count
                         
                            if s>0:
                               point[1] = prev_index + count/2.0
                            elif s<0:
                               point[1] = -(prev_index + count/2.0)
                            self.sectors.append(point)
                            count = 0
                            i = j + 1
                            point = [0,0]
                            break 
                    
                         elif hist[j] == 0:
                              #keep increasing the count until you reach the end
                              count = count + 1
                              if j == hist.shape[0] -1 and hist[j] == 0:
                                 point[0] = count                             
                                 if s > 0:
                                    point[1] =  prev_index + count/2.0
                                 if s < 0:
                                    point[1] = -(prev_index + count/2.0)
                                 self.sectors.append(point)                                 
                                 count = 0
                                 point = [0,0]
                                 i = j
                                 break
            return self.sectors


	def clearMap(self):
            self.Map_ = np.zeros((121,161,121))
            self.H_left  = np.zeros(9)
            self.H_right = np.zeros(9)
            self.V_up    = np.zeros(9)
            self.V_down  = np.zeros(9)
            self.sectors.clear()
            self.grid = np.zeros((161,121))
