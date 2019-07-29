
import math
import numpy as np

robot_length = 0.354/2
clearance = 0.5

d = robot_length + clearance
def obstacle_space(x, y):
    
    c=0
    

    if x<=-5.55+d or x >=5.55-d or y<=-5.05+d or y>=5.05-d:
        c=1

    
    elif ((x+1.6)**2 + (y+4.6)**2 ) <= (0.405 + d)**2 :
        c=1
    elif ((x+1.17)**2 + (y+2.31)**2 ) <= (0.405 + d)**2 :
        c=1
    elif ((x+1.17)**2 + (y-2.31)**2 ) <= (0.405 + d)**2 :
        c=1
    elif ((x+1.65)**2 + (y-4.6)**2 ) <= (0.405 + d)**2 :
        c=1
        

    
    elif ((x+4.05)**2 + (y-3.25)**2 ) <= (0.7995 + d)**2 or ((x+2.46)**2 + (y-3.25)**2 ) <= (0.7995 + d)**2:
        c=1
    elif -4.05 - d <= x and x <= -2.45 + d and 2.45 - d <= y and y <= 4.05 + d:
        c=1
        

    
    elif 1.3 - d <= x and x <= 5.55 + d and -5.05 - d <= y and y <= -4.7 + d:        
        c=1
    
    elif -0.81 - d <= x and x <= 1.93 + d and -4.7 - d <= y and y <= -3.18 + d:        
        c=1

    elif 2.24 - d <= x and x <= 3.41 + d and -4.7 - d <= y and y <= -4.12 + d:        
        c=1

    elif 3.72 - d <= x and x <= 5.55 + d and -4.7 - d <= y and y <= -3.94 + d:        
        c=1


    
    elif -1.17 - d <= x and x <= -0.26 + d and -1.9 - d <= y and y <= -0.08 + d:        
        c=1
        
    elif -0.26 - d <= x and x <= 1.57 + d and -2.4 - d <= y and y <= -1.64 + d:        
        c=1
        
    elif 2.29 - d <= x and x <= 3.8 + d and -2.38 - d <= y and y <= -1.21 + d:        
        c=1
        
    elif 4.97 - d <= x and x <= 5.55 + d and -3.27 - d <= y and y <= -2.1 + d:        
        c=1
        
    elif 4.64 - d <= x and x <= 5.55 + d and -1.42 - d <= y and y <= -0.57 + d:        
        c=1
        
    elif 4.97 - d <= x and x <= 5.55 + d and -0.57 - d <= y and y <= 0.6 + d:        
        c=1
        
    elif 1.89 - d <= x and x <= 5.55 + d and 1.16 - d <= y and y <= 1.92 + d:        
        c=1
    
    elif 2.77 - d <= x and x <= 3.63 + d and 3.22 - d <= y and y <= 5.05 + d:        
        c=1
        
    elif 4.28 - d <= x and x <= 4.71 + d and 4.14 - d <= y and y <= 5.05 + d:        
        c=1
    
    return c



def correct_start(first_node):
    q=obstacle_space(first_node[0],first_node[1])
    if q == 1:
        exit("Start point inside obstacle space or outside boundary")
    
def correct_end(goal):
    q=obstacle_space(goal[0],goal[1])
    if q == 1:
        #print("Goal inside obstacle space")
        exit("Goal inside obstacle space or outside boundary")





import math
import matplotlib.pyplot as plt
from sys import exit

x_i=input("Start Point: X-Coordinate")
y_i=input("Start Point: Y-oordinate")
theta = float(input("Give the orientation of the robot"))
x_f=input("Goal Point: X-Coordinate")
y_f=input("Goal Point: Y-Coordinate")

first_node = [float(x_i), float(y_i)]
correct_start(first_node)


goal = [float(x_f), float(y_f)]
correct_end(goal)




initial_node = first_node
final_node = goal


rpm1 = 50
rpm2 = 100
radius = (0.038)
#v1 = (rpm1*((2*math.pi)/60)) #left wheel velocity in metre == 5.23 rad/sec
#v2 = (rpm2*((2*math.pi)/60)) #right wheel velocity in metre == 10.46 rad/sec
v1 = 5 #Defining in radians/sec
v2 = 10 #Defining in radians/sec
visited = []


dt = 0
dtime = 1.5 #in seconds
Length_car = 0.230
child_updated = [initial_node]

child_list = [initial_node]
parent_list = [initial_node]
theta_list = [theta]

cost_min = 0

cost = []
h_cost = []
cost.append(float('inf'))
h_cost.append(float('inf'))
visited_parent = []
speed=[]
speed_new=[]
index_least_heu = 0
speed.append([0,0])
theta_list=[]
theta_list.append(theta)

def checkPoint(node, points):
    flag = 0
    for point in points:
        if (((node[0] - point[0])**2 + (node[1] - point[1])**2) - 0.1**2 < 0):
            return True
    return False

def check_child(child_updated, initial_node,ul,ur,dt):
    h2=heu(initial_node,child_updated)
    h1 = heu(child_updated, final_node)
    cost_to_come = h2 +cost_min 
    costh = cost_to_come + h1
    #child_updated = [round(child_updated[0],2),round(child_updated[1],2) ]
    child_updated = [ ((math.floor(child_updated[0] * 100)) / 100.0), ((math.floor(child_updated[1] * 100)) / 100.0) ]
    #print(child_updated,q)
    q=obstacle_space(child_updated[0],child_updated[1])
    f = checkPoint(child_updated, visited)
    if child_updated not in visited and q == 0 and f== False : #to avoid overlapping
        if child_updated in child_list: #if already present 
            index3 = child_list.index(child_updated)    #find index of child from child list
            check_cost = h_cost[index3]
            if check_cost <= costh:
                pass
            else:
                child_list.pop(index3)
                cost.pop(index3)
                parent_list.pop(index3)
                h_cost.pop(index3)
                speed.pop(index3)
                theta_list.pop(index3)
                theta_list.append(dt)
                cost.append(cost_to_come)
                child_list.append(child_updated)
                parent_list.append(initial_node)
                h_cost.append(costh)
                speed.append([ul,ur])
        else:
            theta_list.append(dt)
            cost.append(cost_to_come)
            child_list.append(child_updated)
            parent_list.append(initial_node)
            h_cost.append(costh)
            speed.append([ul,ur])
    else:
        pass
        
def travel(ul, ur, initial_node,theta_list):
   
    xd = initial_node[0]
    yd = initial_node[1]
    dt = theta_list
    
    for i in range(0,100):
        xd = xd + (radius/2)*(ul + ur)*math.cos(dt)*dtime*(1/100)
        yd = yd + (radius/2)*(ul + ur)*math.sin(dt)*dtime*(1/100)
        dt = dt + (radius/Length_car)*(ur-ul)*dtime*(1/100)
        q=obstacle_space(xd,yd)
        if q==1:
            break
    child_updated = (xd, yd)
    
    

    check_child(child_updated, initial_node,ul,ur,dt)
    
def heu(child_updated, final_node):
    h1 = math.sqrt((child_updated[0] - final_node[0])**2 + (child_updated[1] - final_node[1])**2) 
    h1 = round(h1, 2)
    return h1

def move(v1, v2, initial_node,theta_list):
    travel(v1, v1, initial_node,theta_list)
    travel(v2, v2, initial_node,theta_list)
    travel(0, v1, initial_node,theta_list)
    travel(0, v2, initial_node,theta_list)
    travel(v1, 0, initial_node,theta_list)
    travel(v2, 0, initial_node,theta_list)
    travel(v1, v2, initial_node,theta_list)
    travel(v2, v1, initial_node,theta_list) 

        
def goal_achieved(node):
    c=0

    if ((node[0]-final_node[0])**2+(node[1]-final_node[1])**2<(0.1)**2):
        c=1

    return c

c = 0
count = 0

while c!=1:
    
    count = count +1
    print(count)
    if child_list == []:
        exit("The goal cannot be reached, reduce clearance-radius")
    move(v1, v2, child_list[index_least_heu],theta_list[index_least_heu])
    visited.append(child_list[index_least_heu])

    visited_parent.append(parent_list[index_least_heu])
    speed_new.append(speed[index_least_heu])
    # once it is visited, we have to remove it from everywhere or it would be considered as
    # minimum cost, everytime
    child_list.pop(index_least_heu)
    cost.pop(index_least_heu)
    parent_list.pop(index_least_heu)
    h_cost.pop(index_least_heu)
    speed.pop(index_least_heu)
    theta_list.pop(index_least_heu)

    if h_cost != []:
        min_cost = min(h_cost)
        index_least_heu = h_cost.index(min_cost)
        cost_min = cost[index_least_heu]

    c = goal_achieved(child_list[index_least_heu])

    

papa = []
speed_final=[]

goal = visited[-1]
papa.append(goal)
speed_final.append(speed_new[-1])


while True:
        index3 = visited.index(goal)
        goal = visited_parent[index3] 
        papa.append(goal)
        speed_final.append(speed_new[index3])
        if goal == initial_node:
            break

print(speed_final[::-1])

print("Generating obstacle space. Takes about 90 seconds")

def generateMap(v, p):

    xcoor = np.linspace(-5.5,5.5,110)
    ycoor = np.linspace(-5,5,100)
    
    
    
    for i in visited:
        plt.scatter(i[0], i[1], s=1,color = 'r')
    
    for i in p:
        plt.scatter(i[0], i[1], s=2,color = 'b')
    
    for x1 in xcoor:
        for y1 in ycoor:
            if obstacle_space(x1, y1) == True:
                plt.scatter(x1, y1, color = 'k')
        
    plt.show()
    return 0


generateMap(visited,papa)

try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    #res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking)
    #if res==vrep.simx_return_ok:
    #    print ('Number of objects in the scene: ',len(objs))
    #else:
    #    print ('Remote API function call returned with error code: ',res)
    time = 0
#retrieve motor  handles
    errorCode,left_motor_handle=vrep.simxGetObjectHandle(clientID,'wheel_left_joint',vrep.simx_opmode_blocking)
    errorCode,right_motor_handle=vrep.simxGetObjectHandle(clientID,'wheel_right_joint',vrep.simx_opmode_blocking)
    r, signalValue = vrep.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', vrep.simx_opmode_streaming)
    ul_ur = speed_final[::-1]
    
    for k in ul_ur:
        time = 0
        err_code1 = 1
        err_code2 = 2

        while(err_code1 != 0 and err_code2 != 0):
            err_code1 = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, k[0], vrep.simx_opmode_streaming)
            #print(err_code1)

            err_code2 = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, k[1], vrep.simx_opmode_streaming)
            #print(err_code2)

        r, signalValue = vrep.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', vrep.simx_opmode_buffer)

        while(time<=1.5):

            r, signalValue2 = vrep.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', vrep.simx_opmode_buffer)

            time = signalValue2 - signalValue

        errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0, vrep.simx_opmode_streaming)
        errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0, vrep.simx_opmode_streaming)

    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')

