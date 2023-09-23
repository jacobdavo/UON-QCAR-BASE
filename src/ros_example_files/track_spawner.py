#!/usr/bin/env python3
from gazebo_msgs.srv import DeleteModel, SpawnModel
from gazebo_msgs.msg import ModelState
import rospy
import os

file_path = os.path.dirname(os.path.abspath(__file__))
file_path += "/../qcar_gazebo/models/cones/"
file_path_blue = file_path +"bluecone.sdf" 
file_path_yellow = file_path +"yellowcone.sdf" 
file_path_orange = file_path +"orangecone.sdf" 
# BLUE_CONE_SDF = open(file_path)
BLUE_CONE_SDF = open(file_path_blue).read()
YELLOW_CONE_SDF = open(file_path_yellow).read()
ORANGE_CONE_SDF = open(file_path_orange).read()

def load_points_from_txt(file_path):
  x_data = []
  y_data = []
  with open(file_path, 'r') as file:
    for line in file:
      values = line.strip().split()
      # print(values)
      # print(type(values))
      if values == []:
        continue
      x_data.append(values[0])
      y_data.append(values[1])
  return x_data, y_data

class ConeSpawner:
  def __init__(self, x_points, y_points, blue_sdf_cone, yellow_sdf_cone, orange_sdf_cone):
    self.x_points = x_points
    self.y_points = y_points
    self.sdf_cone = blue_sdf_cone
    self.blue_sdf_cone = blue_sdf_cone
    self.yellow_sdf_cone = yellow_sdf_cone
    self.orange_sdf_cone = orange_sdf_cone
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    rospy.sleep(10)
    self.spawn_cones()    
  def spawn_cones(self):
    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel) 
    blue_count = 0
    yellow_count = 0
    orange_count = 0

    for i in range(1,len(self.x_points)):
      if self.x_points[i] == 'Blue':
        self.sdf_cone = self.blue_sdf_cone
      elif self.x_points[i] == 'Yellow':
        self.sdf_cone = self.yellow_sdf_cone
      elif self.x_points[i] == 'Orange':
        self.sdf_cone = self.orange_sdf_cone
      else:
        if self.sdf_cone == self.blue_sdf_cone:
          model_name = f'blue_cone_{blue_count}'
          blue_count = blue_count + 1
        elif self.sdf_cone == self.yellow_sdf_cone:
          model_name = f'yellow_cone_{yellow_count}'
          yellow_count = yellow_count + 1
        elif self.sdf_cone == self.orange_sdf_cone:
          model_name = f'orange_cone_{orange_count}'
          orange_count = orange_count + 1
        modelstate = ModelState()
        modelstate.model_name = model_name
        print("x:",float(self.x_points[i]),"y:", float(self.y_points[i]))
        modelstate.pose.position.x = float(self.x_points[i])
        modelstate.pose.position.y = float(self.y_points[i])
        modelstate.pose.position.z = 0
        modelstate.pose.orientation.x = 0
        modelstate.pose.orientation.y = 0
        modelstate.pose.orientation.z = 0
        modelstate.pose.orientation.w = 1 
                   
        try:
          resp = spawn_model_prox(modelstate.model_name, self.sdf_cone, "/", modelstate.pose, "world")
          if resp.success:
            print(f"Successfully spawned model {model_name}")
          else:
            print(f"Failed to spawn model {model_name}. Error message: {resp.status_message}")
        except rospy.ServiceException as e:
          print("Service call failed: %s"%e)   

if __name__ == '__main__':
  # Initialize the node
  rospy.init_node('cone_spawn_node', anonymous=True)    # Load points from text files
  print('Absolute directoryname: ',
      os.path.dirname(os.path.abspath(__file__)))
  file_path = os.path.dirname(os.path.abspath(__file__))
  file_path += "/track_example.txt"
  # file_path = rospy.get_param('file_path', '/ros_example_files/track_example.txt')
  x_points, y_points = load_points_from_txt(file_path)   
  if len(x_points) == 0:
    print("No points loaded. Please check the input files.")
  else:
    print(f"Loaded {len(x_points)} points from file {file_path}")
    spawner = ConeSpawner(x_points, y_points, BLUE_CONE_SDF, YELLOW_CONE_SDF, ORANGE_CONE_SDF)    # Keep the node running
  rospy.spin()