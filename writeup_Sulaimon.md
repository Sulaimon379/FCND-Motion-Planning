## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation that includes...

#### The differences between the backyard_flyer_solution.py file and the motion_planning.py file are listed below:

The backyard_flyer_solution.py was written to control a drone to arm, takeoff, transition a drone in through a drone in a square motion, return back to its position, land and disarm. While motion_planning.py was written to control the drone to arm, takeoff, transition following a zig-zag path land and disarm.

The backyard_flyer_solution.py has calculate_box method which is used to produce a fixed square path while
motion_planning.py has plan_path method which is used to produce the more zig-zag or any path on demand.

The backyard_flyer_solution.py completes its goal by returning to the home position therefore the arming_transition method sets home_position as the target location while the motion_planning.py does not return to the home position therefore the arming_transition method does not set home_position as target location.

The state_callback methods in backyard_flyer_solution.py checks if the drone is armed, then change to take off state if true while state_callback method in motion_planning.py checks if the drone is armed, change to  planning state if true then if it is planning state changes to taking off state.

The motion_planning.py contains a send_waypoints() method, which is used to send waypoints information to the simulator environment for visualization of waypoints while the backyard_flyer_solution.py does not have send_waypoints() method.

The motion_planning.py has an additional State value, PLANNING, which was not implemented in backyard_flyer_solution.py

All State values in motion_planning.py are set by the ENUM auto() method (auto numbering) while all states in backyard_flyer_solution.py are manually numbered (1,2,3,4,5,6).

The takeoff_transition method in the backyard_flyer_solution.py has a fixed target_altitude while the takeoff_transition method does not have fixed target_altitude in the motion_planning.py.


### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here I read the first line of the "colliders.csv" file using the numpy loadtxt function to extract only 1st row and 1st and 2nd column.
I next extracted lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home.


#### 2. Set your current local position
Here, I retrieved the global_position as a numpy array with longitude, latitude and altitude using self._longitude, self._latitude,  self._altitude.
I next converted the global position to current local position using global_to_local() function with the global position and self.global_home


#### 3. Set grid start position from local position
Here, I used the create_grid() function to create a grid.  The create_grid() function returns a grid representation of a 2D configuration space based on given obstacle data, a drone altitude of 5 and safety margin of 5 around obstacles; and north_offset, east_offset.
Next, I extracted the grid start position from the current position by substracting north_offset, east_offset from self._north, self._east.

#### 4. Set grid goal position from geodetic coords
Here, I set the grid goal as some arbitrary position on the grid and substracted the north_offset, east_offset from the longitude and latitude of the position.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Here, modified the A* implementation provided in the planning_utils.py to include diagonal motion by creating conditions for actions Action.NW, Action.NE, Action.SW, Action.SE. This is shown below:


	if x - 1 < 0 or y - 1 < 0 or grid[x-1,y-1] == 1:
			valid_actions.remove(Action.NW)
		if x - 1 < 0 or y + 1 > m or grid[x-1,y+1] == 1:
			valid_actions.remove(Action.NE)
		if x + 1 > n or y - 1 < 0 or grid[x+1,y-1] == 1:
			valid_actions.remove(Action.SW)
		if x + 1 > n or y + 1 > m or grid[x+1,y+1] == 1:
			valid_actions.remove(Action.SE)
		

#### 6. Cull waypoints 
In this step, I used a collinearity test to check for collinearity of the points in the path. 
creating a matrix that includes the coordinates of these three points as rows, then creating a determiant of these points. The determinant of the points must be equal to epsilon(1e-6) for the points to be collinear.
The idea is simply to prune the path of unnecessary waypoints.
The code for the collinearity and path pruning is shown below:

	def collinearity_check(p1, p2, p3, epsilon=1e-6): 
		collinear = False
		#Create the matrix out of three points
		mat = np.vstack((point(p1), point(p2), point(p3)))    
		det = np.linalg.det(mat)
		# Set collinear to True if the determinant is less than epsilon
		if det < epsilon:
			collinear = True
			
		return collinear

	
	def prune_path(path):
		pruned_path = [p for p in path]
		i = 0
		while i < len(pruned_path) - 2:
			p1 = pruned_path[i]
			p2 = pruned_path[i+1]
			p3 = pruned_path[i+2]

			if collinearity_check(p1, p2, p3):
				pruned_path.remove(pruned_path[i+1])
			else:
				i += 1
		return pruned_path

Next, I converted the paths to waypoints, set the waypoints as the value of self.waypoints and finally send the waypoints to the similautor for visualisation. 


### Execute the flight
#### 1. Does it work?
Yes it works!

