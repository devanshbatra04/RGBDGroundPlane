#path planner

import Planner
import Controller
import Drone

import pyrealsense2 as rs
import cv2 as cv
import math
import pcl
import sys
import pcl.pcl_visualization
import numpy as np
import pygame 


pygame.init()

Path_Planner = Planner.Planner()           #planner object created
Controller   = Controller.Controller()
FlyBy        = Drone.Drone()

#pcl visualizer
viewer = pcl.pcl_visualization.PCLVisualizering()

#initialize the realsense pipeline for the rgb and depth frames
pipeline = rs.pipeline()
config   = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8,30)

# Start streaming
pipeline.start(config)

# Processing blocks
pc = rs.pointcloud()
colorizer = rs.colorizer()

BLACK = (0,0,0)
WHITE = (255,255,255)

##-----------------------------------___________ START THE MAIN LOOP ___________---------------------------------------------##


#set the height and width of each grid location
WIDTH = 12
HEIGHT = 12

#This sets the margin between each cell
MARGIN =  3

clock = pygame.time.Clock()

WINDOW_SIZE = [960,660]

screen = pygame.display.set_mode(WINDOW_SIZE)

#set the title of the screen
pygame.display.set_caption("GRID")



def Collision_Avoidance():
    while True:
          frames = pipeline.wait_for_frames()
          depth_frame = frames.get_depth_frame()
          color_frame = frames.get_color_frame()
          
          depth_image = np.asanyarray(depth_frame.get_data())
          color_image = np.asanyarray(color_frame.get_data())
         
          points = pc.calculate(depth_frame)
 
          #get the point cloud
          v,t = points.get_vertices(), points.get_texture_coordinates()
          verts = np.asanyarray(v).view(np.float32).reshape(-1,3)     #xyz
          texcoordinates = np.asanyarray(t).view(np.float32).reshape(-1,2) #uv
   
          p = pcl.PointCloud()
   
          p.from_array(verts)
 


                             #-------------------------Point Cloud Preprocessing--------------------------------------#

          #apply range filter
          pass_filter = p.make_passthrough_filter()
          pass_filter.set_filter_field_name('z')
          pass_filter.set_filter_limits(0.2 , 4)
          vp_new = pass_filter.filter()
 
          #apply range filter on y
          pass_filter = vp_new.make_passthrough_filter()
          pass_filter.set_filter_field_name('y')
          pass_filter.set_filter_limits(-3,3)
          vp_new = pass_filter.filter()

          #apply range filter on x
          pass_filter = vp_new.make_passthrough_filter()
          pass_filter.set_filter_field_name('x')
          pass_filter.set_filter_limits(-3,3)
          vp_new = pass_filter.filter()
           

          #now apply voxel filter
          voxel_filter = vp_new.make_voxel_grid_filter()
          voxel_filter.set_leaf_size(0.05,0.05,0.05)
          vp_new2 = voxel_filter.filter()
          vp_new2_filtered = np.asanyarray(vp_new2)     #this point cloud will be used to fill the 3D depth map and the histogram
          #print(vp_new2)
 
          #visualization
          vp_new2_filtered = np.asanyarray(vp_new2)
          viewer.AddPointCloud(vp_new2, b'scene_cloud', 0)
          viewer.SpinOnce()
     
                                 #--------------------------Preprocessing Ends-----------------------------------#





             #######----------------------------------Planner Module-------------------------------------------#########

          Path_Planner.fill_Map(vp_new2_filtered)

          print(Path_Planner.H_left)
          print(Path_Planner.H_right)

          HR_sectors = Path_Planner.getSectors(Path_Planner.H_right, 1)
          HL_sectors = Path_Planner.getSectors(Path_Planner.H_left, -1)
          VT_sectors = Path_Planner.getSectors(Path_Planner.V_up,   1)
          VD_sectors = Path_Planner.getSectors(Path_Planner.V_down, -1)

          screen.fill(BLACK)

          print(Path_Planner.grid)
          cv.imshow("GRID",Path_Planner.grid*255)

          screen.fill(BLACK)

          grid  = Path_Planner.grid

          for row in range(161):
              for column in range(121):
                  if grid[row][column] == 1:
                     print("inside the grid")
                     color = (255,0,0)
                     pygame.draw.rect(screen, color,[(MARGIN + WIDTH) * column + MARGIN,
                              (MARGIN + HEIGHT) * row + MARGIN,
                              WIDTH,
                              HEIGHT])                    
 

          clock.tick(60)

          pygame.display.flip()
 
#########-----------------------------------------------------Controller Module----------------------------------------##########

         #get the current drone velocity as the drone state.
          
          control_signal = Controller.getControlSignal(HR_sectors + HL_sectors, VT_sectors + VD_sectors)
          print("control signal is "  + str(control_signal))
   
          #this control signal is passed to the drone using the python-Mavsdk API

          viewer.RemovePointCloud(b'scene_cloud', 0)
          cv.imshow("image", color_image)
          cv.waitKey(1)
          Path_Planner.clearMap()
        
          
if __name__=="__main__":
   Collision_Avoidance()
