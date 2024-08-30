#BEAST VERSION!!!...

#Author : Pranup Chhetri

import math
from controller import Robot
from keyboardreader import KeyboardReader
from goalchecker import get_goals_in_range
import shapely
import pyperplan
import os
from pyperplan.planner import find_domain, HEURISTICS, search_plan, SEARCHES


# Initialize the Robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

#Initialising global state variables and memory

command_list = [] #Stores the list of valid commands given through keyboard input
busy = False #Flag to set the state of the robot, is busy is false, then next command is considered
goal_reached = False 
last_command = ""
current_room = "unknown"
current_zone = []
# Initialize devices for the robot
l_motor = robot.getDevice("wheel_left_joint")
r_motor = robot.getDevice("wheel_right_joint")
lidar = robot.getDevice("lidar")
lidar.enable(timestep)
lidar.enablePointCloud()
compass = robot.getDevice("compass")
compass.enable(timestep)
gps = robot.getDevice('gps')
gps.enable(timestep)
l_motor.setPosition(math.inf)
r_motor.setPosition(math.inf)
l_motor.setVelocity(0)
r_motor.setVelocity(0)

keyboard = KeyboardReader(timestep)



# Defining coordinates for "Rooms" and goals

red = shapely.Polygon([
      (3.18, 2.12),
      (4.19, 2.12),
      (4.19, 3.90),
      (3.18, 3.90)])
green = shapely.Polygon([
      (3.18, -4.86),
      (4.19, -4.86),
      (4.19, -3.02),
      (3.18, -3.02)])
ducks = shapely.Polygon([
      (-2.59, 3.90),
      (-3.07, 3.08),
      (-3.55, 3.37),
      (-3.09, 4.19)])
balls = shapely.Polygon([
      (-2.55, -3.67),
      (-2.25, -4.20),
      (-3.09, -4.67),
      (-3.34, -4.15)])
redroom_shape = shapely.Polygon([
      (0.4, 2.3),
      (1.3, 2.3),
      (1.3, 3.2),
      (0.4, 3.2)])
      
redroom = shapely.centroid(redroom_shape)

greenroom_shape = shapely.Polygon([
      (0.4, -3.33),
      (1.3, -3.33),
      (1.3, -2.43),
      (0.4, -2.34)])
      
greenroom = shapely.centroid(greenroom_shape)  
  
duckroom_shape = shapely.Polygon([
      (-1.1, 2.3),
      (-0.4, 2.3),
      (-0.4, 3.2),
      (-1.1, 3.2)])
      
duckroom = shapely.centroid(duckroom_shape)

ballroom = shapely.Point(-0.37, -3.50) 
ballroom2 = shapely.Point(-0.41, -2.3)
startroom = shapely.Point(-1.96, -1.95)   
midroom = shapely.Point(-0.6, -0.02)
redroom = shapely.centroid(redroom_shape)
#Dictionary that stores are the 
coordinates = { "red" : shapely.centroid(red), "green": shapely.centroid(green),
"ducks" : shapely.centroid(ducks), "balls" : shapely.centroid(balls),
"redroom" : redroom, "greenroom" : greenroom, "duckroom": duckroom, "ballroom": ballroom, "ballroom2": ballroom2, "midroom" : midroom}


def write_task(where, to):
    """
        
    description : Writes a "task.pddl" file that defines the objects, problems,
    rules and initial state of the environment
    
    param where : str, Current room/zone of the robot
    param to : str, Target destination for the robot

    return : None
    """
    output = "task.pddl"
    fd = open(output, 'w')
    fd.write(f"""(define (problem go-to) (:domain example)
	(:objects
		bot - robot
		redroom greenroom duckroom ballroom ballroom2 startroom midroom red green ducks balls - room
	)
	(:init
		(in bot {where})
		
		(cango startroom midroom)
		(cango startroom ducks)
		(cango midroom ballroom)
		(cango midroom duckroom)
		(cango greenroom redroom)
		(cango ballroom ballroom2)
		(cango ballroom2 duckroom)
		(cango ballroom2 ducks)
		(cango duckroom ducks)
		(cango ducks duckroom)
		(cango redroom greenroom)
		(cango duckroom ballroom)
		(cango greenroom green)
		(cango green greenroom)
		(cango red green)
		(cango red greenroom)
		(cango greenroom red)
		(cango green red)
		(cango green redroom)
		(cango redroom green)
		(cango duckroom redroom)
		(cango ballroom greenroom)
		(cango redroom red)
		(cango red redroom)
		(cango redroom duckroom)
		(cango greenroom ballroom)
		(cango ballroom balls)
		(cango balls ballroom)
	)
	(:goal
		(in bot {to})
	)
    )""")
    fd.close()

# Helper function to get the robot's current position
def get_robot_position():
    """
    param : None
    
    return : list, x, y and z coordinates of the current position of the robot
    """
    return gps.getValues()

def get_robot_room(current_position):
    """
    Description : This function checks the current gps position of the robot
    and searches for a corresponding "zone" or "room" near it. It returns the
    zone as the current zone if the distance between the zone and current 
    position is below a threshold.
    
    param current_position : list, gps coordinates of the robot's position
    
    returns : str, the name of the area that the robot is currently in
    """
    current_zone = []
     
    zones = {"startroom" : startroom, "midroom":midroom, "redroom" : redroom,
     "duckroom" : duckroom, "greenroom" : greenroom, "ballroom" : ballroom, "ballroom2" : ballroom2}
    goal_zones = {"red" : red, "green" : green, "ducks" : ducks, "balls" : balls}
    current_room = "unknown"
    for zone in zones.keys():
        distance = zones[zone].distance(shapely.Point(current_position[0], current_position[1]))
        if distance < 0.2:
            current_zone.append(zone)
            current_room = current_zone[0]
            # print(current_zone)
    #Using a different threshold for "goal-zones" since 0.3 led to crashing into the goals
    for zone in goal_zones.keys():
        distance = goal_zones[zone].distance(shapely.Point(current_position[0], current_position[1]))
        if distance < 0.6:
            current_zone.append(zone)
            current_room = current_zone[0]
            # print(current_zone)
    
    return current_room
     

def get_robot_orientation():
    """
    param : None
    
    returns : float, current orientation of the robot in radians
    """
    compass_values = compass.getValues()
    angle = math.atan2(compass_values[0], -compass_values[2])
    return angle

def get_angle_diff(target_x, target_y, current_x, current_y ):
    """
    Description : This function calculates the target angle and finds the 
    difference between the two.
    
    param target_x : float, x-coordinate of the target destination
    param target_y : float, y-coordinate of the target destination
    param current_x : float, x-coordinate of current position
    param current_y : float, y_coordinate of current position
    
    returns : float, The difference between the target orientation and current 
    orientation in radians
    """
    target_angle = math.atan2(target_y - current_y, target_x - current_x)
    current_angle = get_robot_orientation()
    angle_diff = target_angle - current_angle
    # print(target_angle, current_angle, angle_diff)
    return(angle_diff)
        
def obstacle_coeff():
    """
    Description : 
        This function uses LiDAR values to check obstacles in front.
        
    parmeters : None
    
    returns : int, int; Velocity coeffecients to either stop the robot or not
    """
    # Read LiDAR values
    lidar_values = lidar.getRangeImage()
    # Check transition into rotating
    for d in lidar_values[len(lidar_values)//3 : -len(lidar_values)//3]:
        if d < 0.45:
            print("SOMETHING's IN THE WAY")
            return 0, 0 #Stop and wait till the object in front moves away
    return 1, 1  # No obstacle detected within the frontal 60-degree arc 

    
# Function to make the robot go to a specified coordinate
def go_to_coordinate(target_x, target_y):
    """
    Description : 
        This function uses the the above functions to find the
    current position, orientation and target orientation.
    Using the normalised difference in orientation, it calculates an angular
    and linear velocity.
    
    param target_x : float, X-coordinate of target destination
    param target_y : float, Y-coordinate of target destination
    
    returns : l_velocity - float, velocity for the left wheel
              r_velocity - float, velocity for the right wheel
    """
    current_position = get_robot_position()
    current_x, current_y = current_position[0], current_position[1]
    distance = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)
    current_room = get_robot_room(current_position)
    # print(current_room)
    angle_diff = get_angle_diff(target_x, target_y, current_x, current_y)
    # Normalise the angle difference to the range [-pi, pi]
    if angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    elif angle_diff < -math.pi:
        angle_diff += 2 * math.pi
    # print(angle_diff)
    # Proportional controller for rotation
    angular_velocity = 2.0 * angle_diff
    max_angular_velocity = 2.0
    angular_velocity = max(-max_angular_velocity, min(max_angular_velocity, angular_velocity))
    
    if abs(angle_diff) < 0.2:  # If the robot is facing the target
        linear_velocity = 5.0 
    else:
        linear_velocity = 0
     
    # Set motor velocities
    l_velocity = linear_velocity - angular_velocity
    r_velocity = linear_velocity + angular_velocity
    return l_velocity, r_velocity


def call_planner(domain, problem):
    """
    Description:
        It processes the domain and problem files, selects the search 
        algorithm and heuristic, and calls a planning function to generate 
        a solution.
        
    param domain: str, Name of the domain pddl file
    param problem: str, Name of the problem pddl file
    
    Returns : list, A list of operations in string, i.e. the solution.
    """
    def get_callable_names(callables, omit_string):
        names = [c.__name__ for c in callables]
        names = [n.replace(omit_string, "").replace("_", " ") for n in names]
        return ", ".join(names)

    search_names = get_callable_names(SEARCHES.values(), "_search")
    heuristic = "hff"
    search="bfs"
    problem = os.path.abspath(problem)
    domain = os.path.abspath(domain)
    search = SEARCHES[search]
    heuristic = None
    use_preferred_ops = heuristic == "hffpo"
    solution = search_plan(
        domain,
        problem,
        search,
        heuristic,
        use_preferred_ops=use_preferred_ops,
    )
    return solution

while (robot.step(timestep) != -1):
    #getting current coordinates
    current_x, current_y = gps.getValues()[0], gps.getValues()[1]
    #list of nearby goals
    goal_nearby = get_goals_in_range(current_x, current_y)
    #current room
    room = get_robot_room(gps.getValues())
    #initilising velocity coeffecients
    l_coeff = 1
    r_coeff = 1
    command = keyboard.get_command()
    if command is not None:
        print(f'Got command: {command}')
        #Making sure only valid command are appended to the list
        if command in ['red', 'green', 'balls', 'ducks']:
            command_list.append(command)
            print("Updated queue :", command_list)
        else :
            print("Invalid command")
    # print(len(command_list))
    if len(command_list)>0:   
        # print(command_list)
        #First command is the current command
        cur_command = command_list[0]
        #If current destination is nearby, pop the list and move to next command if exists
        if cur_command in goal_nearby:
            print("GOAAAAAAAAAAL! Reached ", cur_command)
            last_command = cur_command
            command_list.pop(0)
            if len(command_list)>0:
                cur_command = command_list[0]
                print("Next destination :", cur_command)
            else:
                print("Waiting for next command ...")
            l_velocity = 0
            r_velocity = 0
            #set busy to false, i.e. find solution for next task
            busy = False
            goal_reached = True
        #If robot in unknown room, use the last known command as a starting point
        #Since the threshold for goal is more than for room identification, this was necessary
        if room == "unknown":
            write_task(last_command, cur_command)
        else: 
            write_task(room, cur_command)
        #Check for busy flag and existing commands
        if not busy and len(command_list)>0:
            solution = call_planner("domain.pddl", "task.pddl")
            busy = True
            goal_reached = False
            print("Plan :")
            print(solution)
        #If there exists a solution select the first "local" target
        if len(solution)>0:
            cur_task = solution[0].name[1:].split()[3]
        #If goal is not reached yet get motor velocities
        if not goal_reached:
            l_velocity, r_velocity = go_to_coordinate(coordinates[cur_task[:-1]].x,coordinates[cur_task[:-1]].y)
        #If the "local" target is reached, switch to the next target in the solution
        if room == cur_task[:-1]:
            print("reached", room)
            solution.pop(0)
            print("now moving to:", solution[0].name[1:].split()[3][:-1])
        #Check for onstacles ONLY if the robot is moving, and not turning
        if l_velocity > 0 and r_velocity > 0 :
            l_coeff, r_coeff = obstacle_coeff()
        #Update the velocity values
        l_velocity = l_coeff * l_velocity
        r_velocity = r_coeff * r_velocity
        #Set motor velocities
        l_motor.setVelocity(l_velocity)
        r_motor.setVelocity(r_velocity)
"""
RATIONALE:
    Architecture of the robot:
        The robot usees a hierarchial state machine as its architecture.
        Some states are defined explicitly by flags, e.g. busy and 
        goal_reached while some states are implicit, e.g. line 440 -> moving 
        forward checked using velocities.
        The hierarcial bit is when it checks for obstacles when moving 
        forward and reacts to it.
    Mapping :
        The map used in this solution is a graph-based hybrid map because:
            Topological bit - the graph edges are defined in the task.pddl 
            file using a "cango" relationship.
            Metric bit - the position of the nodes are based on coordinates.
    Navigation:
       Global Navigation : 
           The robot uses PDDL(inspired by the tutorial 
       example) for navigation and uses a breadth first search algorithm
       to determine the best solution.
       
       Local Navigation : 
           The local navigation is a straightforward target-based 
           navigation which uses a reactive architecture to avoid dynamic obstacles.

CONCLUSION : 
    The robot successfully finds a path for all valid target destinations
    from its current position wherever it may be.
    It uses deliberation using pddl for global navigation and a reactive
    architecture for local navigation.
    The path planning is done using a breadth first search of the accessible
    nodes in the map.
    Limitations:
        The path planner does not consider the length of the path while 
        planning, i.e. it just finds the optimal path based on number of nodes.
        The obstacle avoidance for dynamic abstacles are inconsistent, edgecases may fail.
"""