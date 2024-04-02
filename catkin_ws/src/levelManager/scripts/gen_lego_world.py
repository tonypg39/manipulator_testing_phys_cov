#!/usr/bin/python3
from pandas import array
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import *
from gazebo_msgs.msg import ModelStates
from tf.transformations import quaternion_from_euler
import rospy, rospkg, rosservice
import sys
import time
import random
import numpy as np
import json
import xml.etree.ElementTree as ET

lego = [] 	#lego = [[name, type, pose, radius], ...]
# World Configuration parameters
spawn_name = '_spawn'
spawn_pos = (-0.35, -0.42, 0.74)  		#center of spawn area
spawn_dim = (0.32, 0.23)    			#spawning area
package_name = "levelManager"

brickDict = { \
		'X1-Y1-Z2': (0,(0.031,0.031,0.057)), \
		'X1-Y2-Z1': (1,(0.031,0.063,0.038)), \
		'X1-Y2-Z2': (2,(0.031,0.063,0.057)), \
		'X1-Y2-Z2-CHAMFER': (3,(0.031,0.063,0.057)), \
		'X1-Y2-Z2-TWINFILLET': (4,(0.031,0.063,0.057)), \
		'X1-Y3-Z2': (5,(0.031,0.095,0.057)), \
		'X1-Y3-Z2-FILLET': (6,(0.031,0.095,0.057)), \
		'X1-Y4-Z1': (7,(0.031,0.127,0.038)), \
		'X1-Y4-Z2': (8,(0.031,0.127,0.057)), \
		'X2-Y2-Z2': (9,(0.063,0.063,0.057)), \
		'X2-Y2-Z2-FILLET': (10,(0.063,0.063,0.057)) \
		}

brickOrientations = { \
		'X1-Y2-Z1': (((1,1),(1,3)),-1.715224,0.031098), \
		'X1-Y2-Z2-CHAMFER': (((1,1),(1,2),(0,2)),2.359515,0.015460), \
		'X1-Y2-Z2-TWINFILLET': (((1,1),(1,3)),2.145295,0.024437), \
		'X1-Y3-Z2-FILLET': (((1,1),(1,2),(0,2)),2.645291,0.014227), \
		'X1-Y4-Z1': (((1,1),(1,3)),3.14,0.019), \
		'X2-Y2-Z2-FILLET': (((1,1),(1,2),(0,2)),2.496793,0.018718) \
		} #brickOrientations = (((side, roll), ...), rotX, height)


#color bricks
colorList = ['Gazebo/Indigo', 'Gazebo/Gray', 'Gazebo/Orange', \
		'Gazebo/Red', 'Gazebo/Purple', 'Gazebo/SkyBlue', \
		'Gazebo/DarkYellow', 'Gazebo/White', 'Gazebo/Green']

brickList = list(brickDict.keys())

task_params = {
	"task_id": "a01",
	"brick_type": 2, # index 0 -> 10 defining which type from the brickDict
	"posX": -0.5, # value from -1 to 1, px is calculated as px = posX * (total_x/2)
	"posY": 1, # value from -1 to 1, py is calculated as py = posy * (total_y/2)
	"rotationZ": 45, #in degrees (-180 to 180)
	"color": 2 # index of ColorList (0 -> 9  = len(colorList))
}


# TOD: implement a read function from a position
def getPose(params):
	_, dim, = brickDict[brickList[params['brick_type']]]
	spawnX = spawn_dim[0]
	spawnY = spawn_dim[1]
	rotX = 0
	rotY = 0
	rotZ = params['rotationZ']
	pointX = params["posX"] * spawnX
	pointY = params["posY"] * spawnY
	pointZ = dim[2]/2
	rot = Quaternion(*quaternion_from_euler(rotX, rotY, rotZ))
	point = Point(pointX, pointY, pointZ)
	return Pose(point, rot)

#get model path
def getModelPath(model):
	pkgPath = rospkg.RosPack().get_path(package_name)
	return f'{pkgPath}/lego_models/{model}/model.sdf'


def model_exists(model_name):
	models = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=1.0)
	for m in models.name:
		if model_name in m:
			return True
	return False

def changeModelColor(model_xml, color):
	root = ET.XML(model_xml)
	root.find('.//material/script/name').text = color
	return ET.tostring(root, encoding='unicode')	

#function to spawn model
def spawn_model(model, pos, name=None, ref_frame='world', color=None):
	
	if name is None:
		name = model
	
	model_xml = open(getModelPath(model), 'r').read()
	if color is not None:
		model_xml = changeModelColor(model_xml, color)

	spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
	return spawn_model_client(model_name=name, 
	    model_xml=model_xml,
	    robot_namespace='/foo',
	    initial_pose=pos,
	    reference_frame=ref_frame)


#support functon spawn bricks
def spawnLego(params):
	brickType = brickList[params['brick_type']]
	brickIndex = brickDict[brickType][0]
	task_id = params['task_id']
	name = f'{brickType}_0'
	pos = getPose(params)
	color = colorList[params['color']]
	spawn_model(brickType, pos, name, spawn_name, color)
	lego.append((name, brickType, pos))

def read_task_params_json():
	# FIXCONFIG: replace with the path from settings
	file_path = "/root/UR5-Pick-and-Place-Simulation/ml/dev/task_params.json"
	with open(file_path, 'r') as file:
		data = json.load(file)
	return data



#main function setup area and level manager
def setUpArea(): 	
	#Read the parameters (1.0 -> from a json) and send them to spawnaLego 
	params = read_task_params_json()
	# params = task_params
	brick_type = brickList[params['brick_type']]
	model_name = f"{brick_type}_0"
	
	if model_exists(model_name):
		delete_model(model_name)
	rospy.sleep(1.5)
	#creating spawn area
	if not model_exists("_spawn"):
		spawn_model(spawn_name, Pose(Point(*spawn_pos),None) )
	spawnLego(params)
	print(f"Added bricks")

#support function delete bricks on table
def delete_model(name):
	try:
		delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
		return delete_model_client(model_name=name)
	except:
		rospy.logwarn(f"Model {name} did not exist, so skipping deletion")
	

if __name__ == "__main__":
	try:
		rospy.init_node("GenWorld")
		if '/gazebo/spawn_sdf_model' not in rosservice.get_service_list():
			print("Waining gazebo service..")
			rospy.wait_for_service('/gazebo/spawn_sdf_model')
		#starting position bricks
		setUpArea()
		print("All done. Ready to start.")


	
	except rosservice.ROSServiceIOException as err:
		print("No ROS master execution")
	except rospy.ROSInterruptException as err:
		print(err)
	except rospy.service.ServiceException:
		print("No Gazebo services in execution")
	except rospkg.common.ResourceNotFound:
		print(f"Package not found: '{package_name}'")
	except FileNotFoundError as err:
		print(f"Model not found: \n{err}")
		print(f"Check model in folder'{package_name}/lego_models'")