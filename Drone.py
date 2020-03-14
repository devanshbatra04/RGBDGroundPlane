#this contains the drone class which is used to update the drone state at each iteration

class Drone:
   
      def __init__(self):
          self.lin_vel = [0,0,0]    #vx,vy,vz
          self.ang_vel = [0,0,0]    #wx, wy, wz
          self.orientation = [0,0,0] #roll, pitch and yaw
      
      def setLinVelocity(self, v):   #v = [vx, vy, vz]
          self.lin_vel = v
  

      def setAngVelocity(self, w):  
          self.ang_vel = w
 
      def setOrientation(self, theta):
          self.orientation = theta
