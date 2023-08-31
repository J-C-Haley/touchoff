
'''
A simple touchoff probe to produce base calibrations

A ros service is provided to touch a single axis at a time, listening for a 'contact' message

A new frame (param:'~touchoff_frame') will be published when positions are contacted,
and when the axes are sufficiently constrained (XYZ) one can write the frame to a static launch file.

TODO:
- more graceful handling of no contact found
- get ctrl+c to kill motion, or another escape. Right now needs to be estopped and restarted
- USE CALIBRATION MANAGER
- publish a box visual on touchoffs? 
- permit calling as service
- make a rqt plugin for buttons
- make a ros-mobile integration

TODO: new touchoff macros...
- internal corner
- centerpoint of hole
- centerpoint of rod
- TCP 4 point??
- many point angle
- many point plane
- Edge 
- corner + edge
'''
# Python imports
import copy
from math import pi, tau, dist, fabs, cos, atan, degrees, radians
import numpy as np
import os
import pathlib
import sys

# ROS imports
import moveit_commander
import rospy
from tf import transformations as tfs
import tf2_ros

# Message imports
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Pose, TransformStamped
import moveit_msgs.msg
from std_msgs.msg import Bool

class touchoff:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("touchoff")
        self.touchoff_frame = rospy.get_param("~touchoff_frame", default="touchoff")
        contact_topic = rospy.get_param("~contact_topic", default="/contact")
        self.velocity_scaling = float(rospy.get_param("~velocity_scaling", default="0.002"))
        self.acceleration_scaling = float(rospy.get_param("~acceleration_scaling", default="0.01"))
        self.debounce = bool(rospy.get_param("~debounce", default="True"))
        self.debounce_count = int(rospy.get_param("~debounce_count", default=3))

        self.cal_dir = pathlib.Path(os.path.expanduser(rospy.get_param("~calibration_folder", default="~/.ros/touchoff/")))
        self.cal_dir.mkdir(parents=True,exist_ok=True)

        self.tool_radius_x = float(rospy.get_param("~tool_radius_x", default="0.002"))
        self.tool_radius_y = float(rospy.get_param("~tool_radius_y", default="0.002"))
        self.tool_radius_z = float(rospy.get_param("~tool_radius_z", default="0.0"))

        self.probe_search_distance = float(rospy.get_param("~probe_search_distance", default=0.01))

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = rospy.get_param("~move_group_name", default="arm")
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.move_group.set_planner_id("RRTConnectkConfigDefault")
        self.move_group.set_num_planning_attempts(20)
        self.move_group.allow_replanning(True)

        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()

        self.contact = False
        self.realigned = False
        self.alignment_rotations = {'x':0,'y':0,'z':0}
        rospy.Subscriber(contact_topic,Bool,self.contact_cb,queue_size=50)

        robot_ns = rospy.get_param("~robot_ns", default="my_robot_ns")
        rospy.Subscriber(f'/{robot_ns}/execute_trajectory/status/',GoalStatusArray,self.watch_status,queue_size=5)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        rospy.on_shutdown(self.shutdown)

        # rospy.spin()

    def go(self,pose,vel=None,stop_on_contact=True):
        '''Move to a pose in the robot base frame.

        By default stops motion if contact is detected,
        returns True if contact was detected during the move
        '''
        if vel == None:
            vel = self.velocity_scaling
        (plan, fraction) = self.move_group.compute_cartesian_path([pose], 0.01, 0.0)
        replan = self.move_group.retime_trajectory(
            self.robot.get_current_state(),
            plan,
            velocity_scaling_factor=vel,
            acceleration_scaling_factor=self.acceleration_scaling,
            algorithm = "iterative_time_parameterization",
            )
        
        if not stop_on_contact:
            success = self.move_group.execute(replan, wait=True)
            self.move_group.clear_pose_targets()
            return False
        success = self.move_group.execute(replan, wait=False)

        # wait for move to start
        t0 = rospy.Time.now()
        while not 1 in self.move_status:
            rospy.sleep(0.001)
            elapsed = rospy.Time.now() - t0
            if elapsed.to_sec() > 5:
                print('failed to move')
                self.move_group.stop()
                return False
        
        # wait while moving for contact 
        while True: 
            if self.contact:
                print('\a')
                print('contact!')
                contacted = True
                break
            if not 1 in self.move_status:
                print('Move finished without contact')
                contacted = False
                break
            rospy.sleep(0.001)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return contacted
    
    def jog_axis(self,axis,distance,vel=0.001,stop_on_contact=True):
        '''Jog along a tool frame axis by distance in meters (x,-x,y,-y,z,-z)
        
        Returns True if contact was detected during the move
        '''
        relative_pose = Pose()
        if axis == 'x':
            relative_pose.position.x += distance
        elif axis == '-x':
            relative_pose.position.x -= distance
        elif axis == 'y':
            relative_pose.position.y += distance
        elif axis == '-y':
            relative_pose.position.y -= distance
        elif axis == 'z':
            relative_pose.position.z += distance
        elif axis == '-z':
            relative_pose.position.z -= distance        
        new_pose = self.tool_pose_to_world_pose(relative_pose)

        # TODO: ensure contact message is recent
        contacted = self.go(new_pose,vel=vel,stop_on_contact=stop_on_contact)
        return contacted

    def rotate(self,axis,angle_deg,vel=None):
        if vel == None:
            vel = self.velocity_scaling
        relative_pose = Pose()
        rot = radians(float(angle_deg))
        relative_pose.orientation.w = 1.0
        if axis == 'z':
            paxis1, paxis2 = 'x', 'y'
        elif axis == 'y':
            paxis1, paxis2 = 'z', 'x'
        elif axis == 'x':
            paxis1, paxis2 = 'y', 'z'

        start_pose = self.move_group.get_current_pose().pose
        world2current_rotmat = tfs.quaternion_matrix([start_pose.orientation.x,start_pose.orientation.y,start_pose.orientation.z,start_pose.orientation.w])
        alignrotmat = tfs.euler_matrix(rot,0,0,axes=f'r{axis}{paxis1}{paxis2}')
        world2aligned_rotmat = tfs.concatenate_matrices(world2current_rotmat,alignrotmat)
        world2aligned_quat = tfs.quaternion_from_matrix(world2aligned_rotmat)

        start_pose_rotated = copy.deepcopy(start_pose)
        start_pose_rotated.orientation.x = world2aligned_quat[0]
        start_pose_rotated.orientation.y = world2aligned_quat[1]
        start_pose_rotated.orientation.z = world2aligned_quat[2] 
        start_pose_rotated.orientation.w = world2aligned_quat[3]
        self.go(start_pose_rotated,vel=float(vel))
            
    def axis(self, axis, search_dist=None):
        '''Search along axis (x,-x,y,-y,z,-z) for contact
        
        Returns:
        - if XYZ are sufficiently constrained,
        - the pose of the base corner, in the robot planning frame,
        - pose of the contact point, offset from TCP by tool radius'''
        if search_dist == None:
            search_dist = self.probe_search_distance
        # Initialize storage pose for touchoffs - relative to starting position during first axis call. Resets on align
        if not hasattr(self,'touchoff_start_pose') or self.realigned:

            self.touchoff_start_pose = self.move_group.get_current_pose().pose
            self.x_touched, self.y_touched, self.z_touched = False, False, False
            self.x_offest_tool, self.y_offest_tool, self.z_offest_tool = 0.0, 0.0, 0.0
            self.constrained = False
            self.realigned = False

        # jog into contact
        contacted = self.jog_axis(axis,search_dist,vel=0.0002)

        # Again, slower
        self.jog_axis(axis,-0.001,vel=0.0002,stop_on_contact=False)
        contacted = self.jog_axis(axis,search_dist,vel=0.00005)
        if not contacted:
            print('no contact detected, touchoff aborted')
            return False

        # log position internally
        touched_pose = copy.deepcopy(self.move_group.get_current_pose().pose)
        self.jog_axis(axis,-0.001,vel=0.0002,stop_on_contact=False)

        # calculate touched point in the touchoff_start_pose frame
        xyz_offset_world = np.array([
            touched_pose.position.x - self.touchoff_start_pose.position.x,
            touched_pose.position.y - self.touchoff_start_pose.position.y,
            touched_pose.position.z - self.touchoff_start_pose.position.z,
            1.0,
        ])

        world2tool_rotmat = tfs.quaternion_matrix([
            self.touchoff_start_pose.orientation.x,
            self.touchoff_start_pose.orientation.y,
            self.touchoff_start_pose.orientation.z,
            self.touchoff_start_pose.orientation.w,
            ])
        # NOTE: when transforming points in a frame to another frame, 
        # you use the INVERSE of the transformation for the coordinate frame. 
        # Aka, the point doesn't move, the frame moves, so the apparent position of the point moves the other direction
        xyz1_offset_tool = np.matmul(tfs.inverse_matrix(world2tool_rotmat),xyz_offset_world.T).T

        # calculate axis offset from touchoff_start_pose frame to the object edge and store
        if '-' in axis:
            sign = -1.0
        else:
            sign = 1.0
        if 'x' in axis:
            self.x_offest_tool = xyz1_offset_tool[0] + (sign * self.tool_radius_x)
            print(f'new x relative position: {self.x_offest_tool}')
            self.x_touched = True
        elif 'y' in axis:
            self.y_offest_tool =  xyz1_offset_tool[1] + (sign * self.tool_radius_y)
            print(f'new y relative position: {self.y_offest_tool}')
            self.y_touched = True
        elif 'z' in axis:
            self.z_offest_tool = xyz1_offset_tool[2] + (sign * self.tool_radius_z)
            print(f'new z relative position: {self.z_offest_tool}')
            self.z_touched = True

        # Calculate object corner's world position and publish
        if hasattr(self,'x_offest_tool') and \
            hasattr(self,'y_offest_tool') and \
            hasattr(self,'z_offest_tool'):
            self.constrained = True
        tool2world_rotmat = tfs.inverse_matrix(world2tool_rotmat)
        xyz1_corner = np.array([
            self.x_offest_tool, # These can be accessed directly by angle() to get touchoff differences in tool frame
            self.y_offest_tool,
            self.z_offest_tool,
            1.0,
        ])
        xyz1_corner_world = tfs.concatenate_matrices(tfs.inverse_matrix(tool2world_rotmat),xyz1_corner)
        self.corner_pose = copy.deepcopy(self.touchoff_start_pose)
        self.corner_pose.position.x = self.touchoff_start_pose.position.x + xyz1_corner_world[0]
        self.corner_pose.position.y = self.touchoff_start_pose.position.y + xyz1_corner_world[1]
        self.corner_pose.position.z = self.touchoff_start_pose.position.z + xyz1_corner_world[2]

        # calculate the touch point pose (offset from tcp, used for angle() )
        # unnecessary? angle uses *difference*, offset drops out of equation
        touch_point_pose = copy.deepcopy(touched_pose)
        offset_tool = np.array([
            ('x' in axis) * sign * self.tool_radius_x,
            ('y' in axis) * sign * self.tool_radius_y,
            ('z' in axis) * sign * self.tool_radius_z,
            1.0,
        ])
        offset_world = np.matmul(tfs.inverse_matrix(tool2world_rotmat),offset_tool.T).T
        touch_point_pose.position.x += offset_world[0]
        touch_point_pose.position.y += offset_world[1]
        touch_point_pose.position.z += offset_world[2]
        
        # Publish frame for visualization/use
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = self.planning_frame
        static_transformStamped.child_frame_id = self.touchoff_frame
        static_transformStamped.transform.translation.x = self.corner_pose.position.x
        static_transformStamped.transform.translation.y = self.corner_pose.position.y
        static_transformStamped.transform.translation.z = self.corner_pose.position.z
        static_transformStamped.transform.rotation = self.corner_pose.orientation
        self.tf_broadcaster.sendTransform(static_transformStamped)

        return self.constrained, self.corner_pose, touch_point_pose

        # centered_pose = copy.deepcopy(touched_pose)
        # if '-' in axis:
        #     sign = -1.0
        # else:
        #     sign = 1.0
        # # TODO: AAAAAAAAAAAAAAGGGGGGGGGGGGGGGGHHHHHHHHHHHHHH
        # # The centered pose is in GLOBAL right now, not tool. Need to constrain locally, then transform when fully constrained
        # # on first axis, create a local frame at start position, then move it when xyz updated locally, then when all fixed publish
        # if 'x' in axis:
        #     centered_pose.position.x = touched_pose.position.x + (sign * self.tool_radius_x)
        #     self.x = centered_pose.position.x
        #     print(f'new x position: {self.x}')
        # elif 'y' in axis:
        #     centered_pose.position.y = touched_pose.position.y + (sign * self.tool_radius_y)
        #     self.y = centered_pose.position.y
        #     print(f'new y position: {self.y}')
        # elif 'z' in axis:
        #     centered_pose.position.z = touched_pose.position.z + (sign * self.tool_radius_z)
        #     self.z = centered_pose.position.z
        #     print(f'new z position: {self.z}')

        # # if all axes touched, publish frame (aligned with tool)
        # if hasattr(self,'x') and hasattr(self,'y')  and hasattr(self,'z'):
        #     static_transformStamped = TransformStamped()
        #     static_transformStamped.header.stamp = rospy.Time.now()
        #     static_transformStamped.header.frame_id = self.planning_frame
        #     static_transformStamped.child_frame_id = 'touchoff_origin'

        #     static_transformStamped.transform.translation.x = self.x
        #     static_transformStamped.transform.translation.y = self.y
        #     static_transformStamped.transform.translation.z = self.z

        #     if hasattr(self,'aligned_orientation'):
        #         static_transformStamped.transform.rotation = self.aligned_orientation
        #     else:
        #         static_transformStamped.transform.rotation = touched_pose.orientation

        #     self.tf_broadcaster.sendTransform(static_transformStamped)
        # return centered_pose
    
    def corner(self, axis1, axis2, axis3):
        '''Search along axis (x,-x,y,-y,z,-z) for contact'''
        # get current tip position
        start_pose = self.move_group.get_current_pose().pose

        search_dist = 0.02
        depth_dist = 0.001 # Distance below axis1 to stick nozzle 

        # first axis
        self.axis(axis1)
        clear_pose = self.move_group.get_current_pose().pose
        
        # second axis
        self.jog_axis(axis2,-1.0*search_dist)
        self.jog_axis(axis1,0.001+depth_dist)
        self.axis(axis2,search_dist)
        self.jog_axis(axis1,-0.001-depth_dist)
        self.go(clear_pose)

        # third axis
        self.jog_axis(axis3,-1.0*search_dist)
        self.jog_axis(axis1,0.001+depth_dist)
        constrained, corner_pose, touch_point_pose = self.axis(axis3,search_dist)
        self.jog_axis(axis1,-0.001-depth_dist)
        self.go(clear_pose)

        return True

    def angle(self,axis1,axis2, spacing='0.040'):
        '''Set a rotation angle by touching off in axis1 at two points along axis2'''
        spacing = float(spacing)
        axis3 = [axis for axis in ['x','y','z'] if axis not in axis1+axis2]
        axis3 = axis3[0]
        
        sign1, sign2 = 1.0, 1.0
        if '-' in axis1:
            sign1 = -1.0
            axis1base = axis1[1]
        else:
            axis1base = axis1
        if '-' in axis2:
            sign2 = -1.0
            axis2base = axis2[1]
        else:
            axis2base = axis2

        # get axes ordered by plane normal to rotation vector
        if 'z' in axis3:
            paxis1, paxis2 = 'x', 'y'
        elif 'y' in axis3:
            paxis1, paxis2 = 'z', 'x'
        elif 'x' in axis3:
            paxis1, paxis2 = 'y', 'z'

        # get current pose
        start_pose = self.move_group.get_current_pose().pose

        # touchoff at current position
        constrained, corner_pose, touch1 = self.axis(axis1)
        self.go(start_pose)
        axis1_offset1 = getattr(self,axis1base+'_offest_tool')
        axis2_offset1 = getattr(self,axis2base+'_offest_tool')

        # move to second point and touchoff
        self.jog_axis(axis2,distance=spacing)
        start_pose_2 = self.move_group.get_current_pose().pose
        constrained, corner_pose, touch2 = self.axis(axis1)
        axis1_offset2 = getattr(self,axis1base+'_offest_tool')
        axis2_offset2 = getattr(self,axis2base+'_offest_tool')
        self.go(start_pose_2)
        self.go(start_pose)       

        # Flip rotation direction if axes left handed
        if paxis1 + paxis2 == axis1base + axis2base:
            mirror = 1.0
        else:
            mirror = -1.0

        # calculate angle difference
        touchdiff = axis1_offset2 - axis1_offset1
        print(f'difference in {axis1} touchoffs: {touchdiff}')
        rot = atan(touchdiff/spacing) * mirror * -1.0 # Rotation needed to align tool to base
        print(f'rotation, degrees: {degrees(rot)}')
        
        # if rotation > 45 deg, throw a warning
        if abs(degrees(rot)) > 15.0:
            print("WARNING, rotation over 15 degrees detected, aborting angle touchoff")
            return
        
        self.alignment_rotations[axis3] = rot

        # Find new aligned pose with world->tool + align rotation
        world2current_rotmat = tfs.quaternion_matrix([start_pose.orientation.x,start_pose.orientation.y,start_pose.orientation.z,start_pose.orientation.w])
        alignrotmat = tfs.euler_matrix(rot,0,0,axes=f'r{axis3}{paxis1}{paxis2}')
        world2aligned_rotmat = tfs.concatenate_matrices(world2current_rotmat,alignrotmat)
        world2aligned_quat = tfs.quaternion_from_matrix(world2aligned_rotmat)

        start_pose_rotated = copy.deepcopy(start_pose)
        start_pose_rotated.orientation.x = world2aligned_quat[0]
        start_pose_rotated.orientation.y = world2aligned_quat[1]
        start_pose_rotated.orientation.z = world2aligned_quat[2] 
        start_pose_rotated.orientation.w = world2aligned_quat[3]

        self.aligned_orientation = start_pose_rotated.orientation            
        
        self.realigned = True
        return rot

    def plane(self,axis1):
        '''FUTURE Set a tool rotation by touching off along axis1 at three points on a plane'''
        pass
    
    def align(self):
        '''Rotate a toolhead to align with touched off frame'''
        if not hasattr(self,'aligned_orientation'):
            print('moving to alignment requires an angle touchoff first')
            return
        # aligned_pose = copy.deepcopy(self.move_group.get_current_pose().pose)
        # aligned_pose.orientation = self.aligned_orientation
        # self.go(aligned_pose)
        
        for axis, rot in self.alignment_rotations.items():
            self.rotate(axis,degrees(rot))
        
        self.realigned = True
    
    def go_to_corner(self):
        '''Go to touchoff origin and rotation'''
        if self.constrained:
            contacted = self.go(self.corner_pose)
    
    def report(self):
        '''Print out the current base offset'''
        if not self.constrained:
            print('error, touchoff not complete, ensure that the tool frame is aligned and all axes have been probed')
            return
        print('WARNING: this assumes the robot is configured in the wall mount mode,')
        print('and ROS is configured in the floor mount mode')
        
        # Kuka uses a rotation -> translation from world to base
        # Inverting, then reversing the transformation
        world2corner_rotmat = self.pose2mat(self.corner_pose)
        floor2wall = tfs.euler_matrix(radians(-90),0,0,axes=f'ryxz')
        world2cornerrot = tfs.concatenate_matrices(world2corner_rotmat,floor2wall)
        corner2world_rotmat = tfs.inverse_matrix(world2cornerrot)
        corner2world_pose = self.mat2pose(corner2world_rotmat)

        quat = corner2world_pose.orientation
        trans = corner2world_pose.position
        euler = tfs.euler_from_quaternion(np.array([
            quat.x,
            quat.y,
            quat.z,
            quat.w,
        ]))
        print(f'Base calibration in XYZ ABC, millimeters:')
        print(f'''
X: {trans.x*1000.0*-1.0} mm
Y: {trans.y*1000.0*-1.0} mm
Z: {trans.z*1000.0*-1.0} mm
A: {degrees(euler[0])} deg
B: {degrees(euler[1])} deg
C: {degrees(euler[2])} deg
''')
    
    def accept(self,frame_name=None):
        '''Write the touched pose to a roslaunch xml'''
        if not self.constrained:
            print('error, touchoff not complete, ensure that the tool frame is aligned and all axes have been probed')
            return
        if frame_name == None:
            frame_name = self.touchoff_frame
        quat = self.corner_pose.orientation
        trans = self.corner_pose.position

        output_file = f"tf_{frame_name}.launch"
        output_path = self.cal_dir / output_file

        with open(output_path, "w") as f:
            f.write("<launch>\n")
            f.write("""<!-- {} -->\n""".format(frame_name))

            node = """<node pkg="tf2_ros" type="static_transform_publisher" name="{}" args="{} {} {} {} {} {} {} {} {}" />\n""".format(
                frame_name + "_tf_publisher",
                f"{trans.x:.12f}",
                f"{trans.y:.12f}",
                f"{trans.z:.12f}",
                f"{quat.x:.12f}",
                f"{quat.y:.12f}",
                f"{quat.z:.12f}",
                f"{quat.w:.12f}",
                self.planning_frame,
                frame_name,
            )
            f.write(node)
            f.write("</launch>\n")
        print(f'saved touchoff launch to {str(output_path)}')

    ########## CALBACKS ##########
    def contact_cb(self, msg):
        '''callback to monitor electrical contact'''
        if not self.debounce:
            self.contact = msg.data
            return
        if not hasattr(self,'contacts'):
            self.contacts = []
        self.contacts.append(msg.data)
        if len(self.contacts) > self.debounce_count:
            self.contacts.pop(0)
        if self.contacts.count(True) > self.contacts.count(False):
            self.contact = True
        else:
            self.contact = False
        
    def watch_status(self,msg):
        '''Callback to monitor movegroup motion'''
        self.move_status = []
        for status in msg.status_list:
            self.move_status.append(status.status)

    def shutdown(self):
        self.move_group.stop()
    
    ########## UTILITIES ##########
    def tool_pose_to_world_pose(self,tool_jog_pose):
        
        # combine world -> tool, tool -> tool_new_position, return world -> tool_new_position
        current_tool_pose = self.move_group.get_current_pose().pose
        world2tool = self.pose2mat(current_tool_pose)
        tool2new = self.pose2mat(tool_jog_pose)
        world2new = tfs.concatenate_matrices(world2tool,tool2new)
        new_pose = self.mat2pose(world2new)

        return new_pose
    
    def pose2mat(self,pose):
        '''turn a geometry_msgs.msg Pose object into a 4x4 transformation matrix'''
        trans = tfs.translation_matrix(np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z,
        ]))
        rot = tfs.quaternion_matrix(np.array([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]))
        tfmat = tfs.concatenate_matrices(trans,rot)
        return tfmat

    def mat2pose(self,mat):
        '''turn a 4x4 transformation matrix into a geometry_msgs.msg Pose object'''
        pose = Pose()
        trans = tfs.translation_from_matrix(mat)
        quat = tfs.quaternion_from_matrix(mat)
        pose.position.x = trans[0]
        pose.position.y = trans[1]
        pose.position.z = trans[2]
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose

    def test(self):

        rots = []
        angles = [0,2,2,2,2,2,2,2,2,2,2]
        for angle in angles:
            self.rotate('z',angle)
            rot = self.angle('-x','-y')
            rots.append(rot)
        print(rots)

if __name__ == "__main__":
    touch = touchoff()
    while not rospy.is_shutdown():
        print('''Enter touchoff type and arguments, or 'help':''')
        help = '''
Usage: 
(axes are x,-x,y,-y,z,-z, distance is in meters, rotation is in degrees, bracketed are optional)
jog AXIS DISTANCE - move along AXIS in the toolhead coordinate frame by DISTANCE
rotate ROTATION_AXIS ANGLE_DEGREES - rotate about the ROTATION_AXIS by ANGLE_DEGREES
axis TOUCHOFF_AXIS [search distance] - slowly move along TOUCHOFF_AXIS until contact is detected, and save measurement
angle TOUCHOFF_AXIS TRAVERSE_AXIS [distance_between_points_m] - measure rotation angle by touching along TOUCHOFF_AXIS at two points along the TRAVERSE AXIS
(future) plane [x,-x,y,-y,z,-z] [distance_between_points_m] - touch a grid of points to get a plane rotation
align - after measuring an angle, rotate the toolhead to align with the base
accept [frame_name] - write a launch.xml file to store touchoff as a static transform
report - print out touchoff frame transformation from robot base (XYZ ABC)

corner AXIS1 AXIS2 AXIS3 [search_distance] - touch off three axes about an outer-facing corner, see diagram
(future) edge 
(future) corneredge

Quit with ctrl+c or q'''
        resp = input()
        resp = resp.split()
        if len(resp) < 1:
            print(help)
        # manual motions:
        if resp[0] == 'jog':
            touch.jog_axis(resp[1],float(resp[2]))
        elif resp[0] == 'rotate':
            touch.rotate(resp[1],resp[2])
        # touchoffs:
        elif resp[0] == 'axis' and resp[1] in ['x','-x','y','-y','z','-z']:
            touch.axis(resp[1])
        elif resp[0] == 'angle' and resp[1] in ['x','-x','y','-y','z','-z'] and resp[2] in ['x','-x','y','-y','z','-z']:
            touch.angle(resp[1],resp[2])
        elif resp[0] == 'plane' and resp[1] in ['x','-x','y','-y','z','-z']:
            touch.plane(resp[1])
        elif resp[0] == 'align':
            touch.align()
        # macro touchoffs:
        elif resp[0] == 'go_to_corner':
            touch.go_to_corner()
        elif resp[0] == 'corner':
            touch.corner(resp[1],resp[2],resp[3])
        # finish:
        elif resp[0] == 'accept':
            touch.accept()
        elif resp[0] == 'report':
            touch.report()
        elif resp[0] == 'q':
            break
        elif resp[0] == 'test':
            touch.test()
        else:
            print(help)
