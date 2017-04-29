# Wang, Yongxin
# Nguyen, Anh Duong

#!/usr/bin/env python3

''' Get a raw frame from camera and display in OpenCV
By press space, save the image from 001.bmp to ...
'''

import cv2
import cozmo
import numpy as np
import time
import random
from numpy.linalg import inv
import threading

from ar_markers.hamming.detect import detect_markers

from grid import CozGrid
from gui import GUIWindow
from particle import Particle, Robot
from setting import *
from particle_filter import *
from utils import *


# camera params
camK = np.matrix([[295, 0, 160], [0, 295, 120], [0, 0, 1]], dtype='float32')

#marker size in inches
marker_size = 3.0

# tmp cache
last_pose = cozmo.util.Pose(0,0,0,angle_z=cozmo.util.Angle(degrees=0))
flag_odom_init = False

# goal location for the robot to drive to, (x, y, theta)
goal = (6,10,0)

# map
Map_filename = "map_arena.json"
grid = CozGrid(Map_filename)
gui = GUIWindow(grid)

async def image_processing(robot):

    global camK, marker_size

    event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

    # convert camera image to opencv format
    opencv_image = np.asarray(event.image)
    
    # detect markers
    markers = detect_markers(opencv_image, marker_size, camK)
    
    # show markers
    for marker in markers:
        marker.highlite_marker(opencv_image, draw_frame=True, camK=camK)
        #print("ID =", marker.id);
        #print(marker.contours);
    cv2.imshow("Markers", opencv_image)

    return markers

#calculate marker pose
def cvt_2Dmarker_measurements(ar_markers):
    
    marker2d_list = [];
    
    for m in ar_markers:
        R_1_2, J = cv2.Rodrigues(m.rvec)
        R_1_1p = np.matrix([[0,0,1], [0,-1,0], [1,0,0]])
        R_2_2p = np.matrix([[0,-1,0], [0,0,-1], [1,0,0]])
        R_2p_1p = np.matmul(np.matmul(inv(R_2_2p), inv(R_1_2)), R_1_1p)
        yaw = -math.atan2(R_2p_1p[2,0], R_2p_1p[0,0])
        
        x, y = m.tvec[2][0] + 0.5, -m.tvec[0][0]
        
        
        # remove any duplate markers
        dup_thresh = 2.0
        find_dup = False
        for m2d in marker2d_list:
            if grid_distance(m2d[0], m2d[1], x, y) < dup_thresh:
                find_dup = True
                break
        if not find_dup:
            marker2d_list.append((x,y,math.degrees(yaw)))

    for tup in marker2d_list:
        print('x =', tup[0], 'y =', tup[1],'theta =', tup[2])

    return marker2d_list


#compute robot odometry based on past and current pose
def compute_odometry(curr_pose, cvt_inch=True):
    global last_pose, flag_odom_init
    last_x, last_y, last_h = last_pose.position.x, last_pose.position.y, \
        last_pose.rotation.angle_z.degrees
    curr_x, curr_y, curr_h = curr_pose.position.x, curr_pose.position.y, \
        curr_pose.rotation.angle_z.degrees
    
    dx, dy = rotate_point(curr_x-last_x, curr_y-last_y, -last_h)
    if cvt_inch:
        dx, dy = dx / 25.6, dy / 25.6

    return (dx, dy, diff_heading_deg(curr_h, last_h))

#particle filter functionality
class ParticleFilter:

    def __init__(self, grid):
        self.particles = Particle.create_random(PARTICLE_COUNT, grid)
        self.grid = grid

    def update(self, odom, r_marker_list):

        # ---------- Motion model update ----------
        self.particles = motion_update(self.particles, odom)

        # ---------- Sensor (markers) model update ----------
        self.particles = measurement_update(self.particles, r_marker_list, self.grid)

        # ---------- Show current state ----------
        # Try to find current best estimate for display
        m_x, m_y, m_h, m_confident = compute_mean_pose(self.particles)
        return (m_x, m_y, m_h, m_confident)


def converge(particles):
    m_x, m_y, m_h, m_c = compute_mean_pose(particles)
    count = 0
    for p in particles:
        d = grid_distance(p.x, p.y, m_x, m_y)
        # print ("particle is: ", p.x, p.y)
        diff_angle = diff_heading_deg(p.h, m_h)
        # break
        if d <= 0.6 and diff_angle < 10:
            count += 1

    # print (count * 1.0 / (PARTICLE_COUNT))

    return count >= 0.97 * PARTICLE_COUNT

async def run(robot: cozmo.robot.Robot):

    global flag_odom_init, last_pose
    global grid, gui

    # start streaming
    robot.camera.image_stream_enabled = True

    #start particle filter
    pf = ParticleFilter(grid)

    ###################

    ############YOUR CODE HERE#################

    # await robot.play_anim_trigger(cozmo.anim.Triggers.AcknowledgeObject).wait_for_completed()
    # return

    picked_up = 0
    goal_reached = False
    while True:
        if not converge(pf.particles):
            if robot.is_picked_up:
                picked_up = 1
                print("Being picked up")
                await robot.play_anim_trigger(cozmo.anim.Triggers.KnockOverFailure, in_parallel=True).wait_for_completed()
                pf = ParticleFilter(grid)

                time.sleep(4)
            else:
                await robot.set_head_angle(cozmo.util.degrees(10)).wait_for_completed()
                # 1. Move around the robot
                
                # 2. Get Robot position (x, y, angle) using robot.pose()
                currPose = robot.pose
                # print (currPose)
                # 3. Get the Odometry using compute_odometry()
                odom = compute_odometry(currPose)

                # 4. Get the Robot's new reading of the markers using ImageProcessing() and cvt_2Dmarker_measurements()
                markers = await image_processing(robot)
                measurement = cvt_2Dmarker_measurements(markers)

                # 5. Call pf.update(odom, r_marker_list)
                estm = pf.update(odom, measurement)
                gui.show_particles(pf.particles)
                gui.show_mean(estm[0], estm[1], estm[2], estm[3])
                gui.updated.set()

                # 6. Count the # of particles around the mean pose of particles

                last_pose = currPose
                # await robot.drive_wheels(10, 18)
                # time.sleep(0.1)
                if robot.is_picked_up:
                    pf = ParticleFilter(grid)
                    continue

                if len(markers) != 0 and measurement[0][0] > 2.0:
                    await robot.drive_straight(cozmo.util.distance_mm(40), cozmo.util.speed_mmps(40)).wait_for_completed()
                    # robot.driv
                # deg = int((random.random()-0.5) * 180)
                else:
                    await robot.turn_in_place(cozmo.util.degrees(-30)).wait_for_completed()
        else:
            if not goal_reached:
                m_x, m_y, m_h, m_c = compute_mean_pose(pf.particles)
                # print (m_x, m_y, m_h, m_c)
                goal_in_inch = (goal[0]*25/25.6, goal[1]*25/25.6, goal[2])
                dif_y = goal_in_inch[1] - m_y
                dif_x = goal_in_inch[0] - m_x
                arc = math.degrees(math.atan2(dif_y, dif_x))
                dist = math.sqrt(dif_y**2 + dif_x**2) * 25.6
                a_to_turn = diff_heading_deg(arc, m_h)
                if not robot.is_picked_up:
                    await robot.turn_in_place(cozmo.util.degrees(a_to_turn)).wait_for_completed()
                else:
                    pf = ParticleFilter(grid)
                    continue

                d = 0

                pickup_while_drive = False
                while d < dist:
                    if robot.is_picked_up:
                        pf = ParticleFilter(grid)
                        pickup_while_drive = True
                        break
                    await robot.drive_straight(cozmo.util.distance_mm(min(40, dist-d)), cozmo.util.speed_mmps(40)).wait_for_completed()
                    d += 40
                
                if pickup_while_drive:

                    continue

                if robot.is_picked_up:
                    pf = ParticleFilter(grid)
                    continue
                else:
                    await robot.turn_in_place(cozmo.util.degrees(-arc)).wait_for_completed()

                if robot.is_picked_up:
                    pf = ParticleFilter(grid)
                    continue
                else:
                    await robot.play_anim_trigger(cozmo.anim.Triggers.AcknowledgeObject).wait_for_completed()
                    goal_reached = True
            else:
                # print ("reached goal")
                time.sleep(1)
                await robot.drive_straight(cozmo.util.distance_mm(0), cozmo.util.speed_mmps(40)).wait_for_completed()
                if robot.is_picked_up:
                    print ("picked up")
                    goal_reached = False
                    pf = ParticleFilter(grid)
                    continue


class CozmoThread(threading.Thread):
    
    def __init__(self):
        threading.Thread.__init__(self, daemon=False)

    def run(self):
        cozmo.robot.Robot.drive_off_charger_on_connect = False  # Cozmo can stay on his charger
        cozmo.run_program(run, use_viewer=False)

class ParticleFilterThread(threading.Thread):
    
    def __init__(self, particle_filter, gui):
        threading.Thread.__init__(self, daemon=True)
        self.filter = particle_filter
        self.gui = gui

    def run(self):
        while True:
            self.gui.show_particles(self.filter.particles)
            self.gui.updated.set()

if __name__ == '__main__':
    grid = CozGrid(Map_filename)
    gui = GUIWindow(grid)
    cozmo_thread = CozmoThread()
    cozmo_thread.start()    
    gui.start()

