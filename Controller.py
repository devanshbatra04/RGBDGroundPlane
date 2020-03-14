#controller takes the path planner output and generates the control signal
#it takes the drone state into consideration as well
#it takes the velocity of the drone and keeps the speed same and changes the steering angle according to the cost calculated below 

import numpy as np
import Planner 

class Controller:

      def __init__(self):
          self.cost_H = 0
          self.cost_V = 0
          self.A =      0.8
          self.B =      0.14

      def getControlSignal(self, H, V):
          min_cost = []
          cost     = 0

          if len(H) != 0:
             for i in range(len(H)):
                 cost = self.A*abs(H[i][0]) + self.B*abs(H[i][1])
                 H[i].append(cost)
                 min_cost.append(cost)             
          return H[min_cost.index(min(min_cost))][1]    


          if len(H) == 0:
              for i in range(len(V)):
                 cost = self.A*abs(V[i][0]) + self.B*abs(V[i][1])
                 V[i].append(cost)
                 min_cost.append(cost)
          return V[min_cost.index(min(min_cost))][1]               
               
       
        
