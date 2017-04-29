# Wang, Yongxin
# Nguyen, AnhDuong

from grid import *
from particle import Particle
from utils import *
from setting import *
from time import sleep
import numpy as np


def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments: 
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    motion_particles = []

    for p in particles:
        local_x = odom[0]
        local_y = odom[1]
        x, y = rotate_point(local_x, local_y, p.h)
        p.x += x
        p.y += y
        p.x = add_gaussian_noise(p.x, ODOM_TRANS_SIGMA)
        p.y = add_gaussian_noise(p.y, ODOM_TRANS_SIGMA)

        p.h += odom[2]
        p.h = add_gaussian_noise(p.h, ODOM_HEAD_SIGMA)
        
        motion_particles.append(p)

    return motion_particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments: 
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information, 
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    weight = []
    cnt = 0

    # no new sensor info
    if len(measured_marker_list) == 0:
        s = 1
        for p in particles:
            weight.append((p, 1/len(particles)))
    else:
        for p in particles:
            markers_visible_to_p = p.read_markers(grid)

            if p.x < 0 or p.x >= grid.width or p.y < 0 or p.y >= grid.height:
                weight.append((p, 0))
                continue
            if (p.x, p.y) in grid.occupied:
                weight.append((p, 0))
                continue

            match = []
            diff = int(math.fabs(len(measured_marker_list)-len(markers_visible_to_p)))

            for cm in measured_marker_list:
                if len(markers_visible_to_p) == 0:
                    break
                cmx, cmy, cmh = add_marker_measurement_noise(cm, MARKER_TRANS_SIGMA, MARKER_ROT_SIGMA)

                # find minp, the closest marker out of markers_visible_to_particle
                minp = markers_visible_to_p[0]
                mind = grid_distance(cmx, cmy, minp[0], minp[1])

                for mvp in markers_visible_to_p:
                    mvpx, mvpy, mvph = mvp[0], mvp[1], mvp[2]
                    dist = grid_distance(cmx, cmy, mvpx, mvpy)
                    if dist < mind:
                        mind = dist
                        minp = mvp

                # store the pairing [cm, m] for later calculations
                match.append((minp, cm))
                markers_visible_to_p.remove(minp)

            # use match to calculate weight of p
            prob = 1

            maxc1 = 0
            maxc2 = (45 ** 2) / (2*(MARKER_ROT_SIGMA ** 2))
            c1 = 2*(MARKER_TRANS_SIGMA ** 2)
            c2 = 2*(MARKER_ROT_SIGMA ** 2)

            for i, j in match:
                distBetweenMarkers = grid_distance(i[0], i[1], j[0], j[1])
                angleBetweenMarkers = diff_heading_deg(i[2], j[2])
                const1 = (distBetweenMarkers ** 2) / c1
                const2 = (angleBetweenMarkers ** 2) / c2
                maxc1 = max(maxc1, const1)
                prob *= np.exp(-const1-const2)

            for _ in range(diff):
                prob *= np.exp(-maxc1-maxc2)

            weight.append((p, prob))

        #normalize weight
        s = 0
        weight.sort(key=lambda x: x[1])
        delete = int(PARTICLE_COUNT/100)
        weight = weight[delete:]
        for i, j in weight:
            if j == 0:
                cnt+=1
            else:
                s += j
        weight = weight[cnt:]
        cnt += delete

    plist = []
    wlist = []

    for i, j in weight:
        newi = Particle(i.x, i.y, i.h)
        wlist.append(j/s)
        plist.append(newi)

    newplist = []

    if plist != []:
        newplist = np.random.choice(plist, size=len(plist), replace = True, p=wlist)

    measured_particles = Particle.create_random(cnt, grid)[:]

    for p in newplist:
        ph = add_gaussian_noise(p.h, ODOM_HEAD_SIGMA)
        px = add_gaussian_noise(p.x, ODOM_TRANS_SIGMA)
        py = add_gaussian_noise(p.y, ODOM_TRANS_SIGMA)
        newp = Particle(px, py, ph)
        measured_particles.append(newp)

    return measured_particles