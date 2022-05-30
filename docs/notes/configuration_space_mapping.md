# Intro
This document will explain all the developments and ideas regarding configuration space mapping.

In order to understand the ideas employed, I will split the development process into two classes:
- Basic Components
- Configuration Mapping-associated algorithms

# Basic Components

In general, in order to get the configuration mapping, two components must be available:
1. The normal vectors (to polygon's faces)
2. The polygons' corners positions in global reference frame

where "polygon" can be the obstacles themselves or bounding boxes in general.

So it was necessary to build code to calculate such values. These calculations can be found in `utils.py` and `configuration_space_mapping.py`.

## Utils

Here it will be explained all necessary functions to get the two basic components. They were developed so each one need the previous one to work.

All the code needed to get the information regarding the normal vectors and corner positions **from the simulator** can be found here, even though the result will be incomplete. Note that we basically need to:
1. get one object handler
2. get all scene objects' handlers (this function is based on the previous one)
3. get bounding box corners positions (this is expressed in local frame)
4. get robot configuration (position and orientation) such that we'll be able to map the corners to global reference frame.
5. get the scene's objects' handlers and configuration (based on the previous functions)
6. have an specific variable containing the robot's info (handler and configuration) and obstacles' info (handlers and configuration for each one). 

These functions will be a detailed (a lil more) right now.

### get_object_handle 

This method returns the handler of a given object.

### get_handlers_scene

This method returns the handlers of all the objects whose names are provided.

### get_bounding_box_corners_positions
The bounding box corners' positions are actually easily available. The only issue on it is that the coordinates are expressed in a local reference frame (i.e.,the robot frame or the obstacle frame), instead of the global reference frame.

Moreover, there are not "coordinates" themselves. What we have is the x_min, y_min, z_min, x_max, y_max, and z_max values. They can fully define the coordinates, since they are the "positions" of the bounding box faces.

The function retrieve these values. The only confusing parameter is the *parameter_id_type*. I don't know the difference, but there are two *ways* of retrieving the bounding boxes faces' position:  
1. using the `parameterID` whose root is `sim.sim_objfloatparam_modelbbox_`
2. using the `parameterID` whose root is `sim.sim_objfloatparam_objbbox_`
When we choose one instead of the other the result is different. 

### get_configuration 

This method returns the position and orientation of a given object.

### get_scene_objects_info 

This function return compilled information regarding the scene's objects. That is, for each object: 
- the name (even though we should know it beforehand) of the object
- the object's handler
- the object's position
- the object's orientation 


### split_robot_from_info_list

The `get_scene_objects_info()` function returns both obstacles and robot info. This function separates them and returns each (the robot's info and the obstacles' info).

## Configuration Space Mapping

Configuration Space was supposed to compute both the "high level parameters" (like normal vectors and corners coordinate in global frame) and the mapping algorithm itself, but it ended having only the high level parameters (they will be explained in this sub-section). The ideas related to the mapping algorithm will be on the next section.

The functions found here follow the following idea:
1. get the normal vectors' orientations. Since we just need to check if a given normal is between two others, we don't need to have the coordinates of them, only their orientation (angle with respect to the global X axis) 
2. limit the normal vectors' orientation to [0, 2*pi]. Since we'll need to compare stuff, and angle is a cyclic value, we should limit orientation to 2pi so we will not have problems 
3. At this point of the development we already have a set of values indicating the bounding box's corners (i.e., x_min, y_min, z_min, x_max, y_max, and z_max). Here they build the points themselves (local frame coordinates)
4. The corners' coordinates have to be expressed in global frame. That's why we need a rotation matrix. Luckly, it is possible to compute a [rotation matrix from euler angles](http://eecs.qmul.ac.uk/~gslabaugh/publications/euler.pdf).
5. In order to compute the corners' global coordinates is necessary to apply the rotation matrix and add the local coordinate frame origin (in global coordinates) to the result.
6. Once we know how to map a single point to global coordinates, we just need to take a bunch of local-frame points and return their global-frame counterparts.

### get_normals_orientation

To calculate the normals orientation is not a big deal. The orientation angle around Z is calculated between the global X axis and the local X axis. This is the first normal orientation. 

Since the bounding boxes are cubes, the other normal orientation is computed adding pi/2 rad (90º) to the previous orientation.

To compute the last two normals we add pi rad (90º) to the two previous normals' orientations.

Notice that the order of the normals is anti-clockwise from the right face.

### convert_angle_to_0_2pi_interval

In order to check if a given angle is between other two, is important to guarantee the angles are limited (we don't need an angle equal to 47.1238898038 rad - that's 15pi- if we can have it equal to 3.14 rad).

The funtion down there inside the code wasn't made by me. I just happened to find it elsewhere in a StackOverFlow question.

### get_bounding_box_corners_local_coordinates

What we retrieved using the utils code were the bounding box "extreme positions". That is:  x_min, y_min, z_min, x_max, y_max, and z_max.

In order to get the corner coordinates (still in local frame), we just need to combine those values. Below we have the values and their position, considering the local X axis goes to the right and the local Y axis goes upward.
- [x_max, y_max, z_min]: top right corner
- [x_min, y_max, z_min]: top left corner
- [x_min, y_min, z_min]: bottom left corner
- [x_max, y_min, z_min]: bottom right corner

Notice they are the bottom layer corners. The top layer corners are not returned, only if otherwise commanded. The top layer corners are almost the same as the bottom's ones. The difference is the Z value.

### rotation_matrix_from_euler_angles

From the simulator we have the euler angles. That is, the individual rotations around the global X, Y and Z axis. If we want to map from the local frame to the global one, we'll gonna need a rotation matrix. This function computes it. 

### get_global_position

If we have a rotation matrix, a local-frame-described point, and we know the origin of the local frame, we have everything needed to map the point to global frame.

First, apply rotation to the point. At the end we add the local frame origin.

### map_local_coordinates_to_global_coordinates

Ok, right now we can map any point already. If we can map one with the previous function, this one maps a whole bunch of them. 

Notice: the points returned follow the same order as the points provided. 

# Configuration Mapping Ideas

The functions explained so far only get us the basic stuff we need to compute the configuration mapping itself. That is:
1. The object corners in global frame
2. Some form of indicating where the normal vectors are (I'm using the orientation angle)

Now I'll explain how I figured an implementation of the mapping algorithm. It can be found in the file `sketching_configuration_mapping.ipynb`. Go to the section "Creating Configuration Map". Before it we have the connection to the simulator and some function testing.

What I do there is the following:
- Get the robot's normal vector's orientations and mirror it (adding pi radians to the values). I also compute its corners' global coordinates
- Choose `Cuboid_0` information to compute its normals and corner coordinates (global) 

The following cells just display the values. Don't mind them. The fifth cell bottom-up has the algorithm sketch. Let's dive in its inner workings.
1. First, I define some lists containing the normal vectors orientations (for both object - here the `Cuboid_0`, since it was planned to be inside a loop for all the obstacles - and the robot).
2. Two loops are prepared. I chose two loops (one nested) because I didn't know how to scan the normal vector circle ([slide number 7](https://arquivos.info.ufrn.br/arquivos/2020226137847580014240828bbdd969d/Espao_de_Configurao_-_2a_parte.pdf)). This way each robot normal vector will compared to other two obstacle's normal vectors (and vice versa). It is a rather silly approach, but it was the one I figured.
3. Inside the loops two values are compared: `answer_a` and `answer_b`. They are boolean values answering the question: is a vector between other two? They follow the notation `APL_ij ...` [Pablo showed in slide 5](https://arquivos.info.ufrn.br/arquivos/2020226137847580014240828bbdd969d/Espao_de_Configurao_-_2a_parte.pdf) 
4. If a given vector is inside other two, we can create a vertice in configuration space. I'm not sure if the index of the values inside the print are correct.
5. The last step is to append the computed vector in a list containing all configuration space vertices.

Here I got some trouble dealing with the indexes. I couldn't figure out what went wrong, and I was pooped enough to decide writing this doc instead of keep trying, so you all would know what, how and why I did what I did.

## When the checks goes False

There is a small theoretical behaviour on how we check the angles between. Notice that is possible that two angles are coincidental, because we're dealing with cubes. In this case, we shouldn't compute a configuration space vertice, according to Pablo's slides. The way the angles are compared will return False, but I don't know if this is the best behaviour ever, nor I have any idea of the consequences.

## is_angle_between

In order to check if a given angle is between other two, I literally check if it is between. That is it. Naturally, a problem arises here:
- an angle of 0 rad wouldn't be account as "in between" the angles 1.57 rad and 6 rad, for example.

**What to do next? I really recommend looking the algorithm's corner indexes. I'm unsure if they make sense.**

**Do the corner indexes I'm dealing with inside the loop match the points to be manipulated? I'm not sure**