"""

plot a simulation of steering
adjustable parameters

"""
# from scipy import signal
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import time
import transforms3d


def run():
    #global scene
    scene = Scene()


    x = [1,2,3,4]

    # go out and measure these things or
    # if not built yet, or you want to try a change, put in a number you want to try

    scene.track_width = 24 # inches
    scene.wheel_base = 24 # inches # ICT: 35
    #max_steering_angle = 45 # degrees
    scene.axle_length = 3.25 # inches
    control_arm_length = 3.5 # inches behind axle of control arm connection point
    control_arm_offset = 0 # inches length of center of axle to pivot point
    scene.control_arm_angle = deg(30)

    #steering_position = np.array([0,-3.25,0]) # inches from centerline of front axle 1.94 at 39.43deg
    #steering_position = np.array([0, -2.25, 0])  # inches from centerline of front axle 6.49 at 39.51deg
    #steering_position = np.array([0, -4.25, 0])  # inches from centerline of front axle -1.84 at 39.31deg
    #steering_position = np.array([0, -3.75, 0])  # inches from centerline of front axle -0.04 at 39.38deg
    #steering_position = np.array([0, -2.80, 0])  # inches from centerline of front axle
    steering_position = np.array([0, -3.25, 0])  # inches from centerline of front axle
    # steering_position = np.array([0, -3.25, 0])  # inches from centerline of front axle ICT measurement
    steering_arm_length = 2 # inches
    steering_offset = 1.063 # inches
    scene.wheel_width = 5
    wheel_size = [scene.wheel_width,10,0] # inches

    y=  np.square(x)

    #plt.plot(x, y, '-', label=x_trace, color='red')

    # draw wheels
    # wheel_x = [-wheel_size[0]/2, wheel_size[0]/2, wheel_size[0]/2, -wheel_size[0]/2, -wheel_size[0]/2]
    # wheel_y = [-wheel_size[1]/2, -wheel_size[1]/2, wheel_size[1]/2, wheel_size[1]/2, -wheel_size[1]/2]
    #
    # left_wheel_x = np.add(wheel_x, - track_width/2)
    # left_wheel_y = wheel_y
    # x = left_wheel_x
    # y = left_wheel_y
    # plt.plot(x, y, '-', label='wheel_base', color='blue')
    #
    # right_wheel_x = np.add(left_wheel_x, track_width)
    # right_wheel_y = wheel_y
    # x = right_wheel_x
    # y = right_wheel_y
    # plt.plot(x, y, '-', label='wheel_base', color='green')

    # try it with an object

    #scene.box.plot(plt)

    scene.add({'right_wheel': Box(pos=np.array([scene.track_width/2-wheel_size[0]/2,0,0]),
              size=wheel_size, color='black')})
    scene.add({'left_wheel': Box(pos=np.array([-scene.track_width/2+wheel_size[0]/2,0,0]),
              size=wheel_size, color='black')})

    scene.add({'rear_right_wheel': Box(pos=np.array([scene.track_width/2-wheel_size[0]/2,-scene.wheel_base,0]),
              size=wheel_size, color='black')})
    scene.add({'rear_left_wheel': Box(pos=np.array([-scene.track_width/2+wheel_size[0]/2,-scene.wheel_base,0]),
              size=wheel_size, color='black')})


    #scene.right_wheel.plot(plt)

    scene.add({'right_axle': Box(pos=scene.objects['right_wheel'].pos-np.array([scene.axle_length/2,0,0]),
              size=[scene.axle_length,1,1])})
    scene.add({'right_axle_pivot': Box(pos=scene.objects['right_axle'].pos-np.array([scene.axle_length/2,0,0]),
              size=[.1,.1,.1], color='red')})

    scene.add({'left_axle': Box(pos=scene.objects['left_wheel'].pos-np.array([-scene.axle_length/2,0,0]),
              size=[scene.axle_length,1,1])})
    scene.add({'left_axle_pivot': Box(pos=scene.objects['left_axle'].pos+np.array([scene.axle_length/2,0,0]),
              size=[.1,.1,.1], color='red')})

    #scene.right_axle.plot(plt)

    scene.add({'right_control_arm': Box(pos=scene.objects['right_axle'].pos-np.array([control_arm_offset+scene.axle_length/2,control_arm_length/2,0]),
              size=[1,control_arm_length,1])})

    scene.add({'right_control_arm_pivot': Box(pos=scene.objects['right_control_arm'].pos+np.array([0,-control_arm_length/2,0]),
              size=[.1,.1,.1])})

    scene.add({'left_control_arm': Box(pos=scene.objects['left_axle'].pos+np.array([control_arm_offset+scene.axle_length/2,-control_arm_length/2,0]),
              size=[1,control_arm_length,1])})

    scene.add({'left_control_arm_pivot': Box(pos=scene.objects['left_control_arm'].pos+np.array([0,-control_arm_length/2,0]),
              size=[.1,.1,.1])})

    #scene.right_control_arm.plot(plt)

    scene.add({'steering_arm': Box(pos=np.array([steering_position[0],steering_position[1]-steering_arm_length/2,0]),
              size=[2,steering_arm_length,1])})
    parent_pos = scene.objects['steering_arm'].pos
    scene.add({'steering_arm_pivot': Box(pos= parent_pos + np.array([0,steering_arm_length/2,0]),
              size=[.1,.1,.1])})

    scene.add({'steering_arm_right_pivot': Box(pos= parent_pos + np.array([steering_offset/2,-steering_arm_length/2,0]),
              size=[.1,.1,.1])})

    scene.add({'steering_arm_left_pivot': Box(pos= parent_pos + np.array([-steering_offset/2,-steering_arm_length/2,0]),
              size=[.1,.1,.1])})

    tie_rod_length = np.linalg.norm(scene.objects['steering_arm_right_pivot'].pos-scene.objects['right_control_arm_pivot'].pos)

    scene.add({'steering_circle_right': Sphere(pos=scene.objects['steering_arm_right_pivot'].pos,
                                         size=[tie_rod_length,tie_rod_length,tie_rod_length])})

    scene.add({'steering_circle_left': Sphere(pos=scene.objects['steering_arm_left_pivot'].pos,
                                         size=[tie_rod_length,tie_rod_length,tie_rod_length])})

    # calculat control arm circle
    wheel_circle = np.linalg.norm(scene.objects['right_control_arm_pivot'].pos-scene.objects['right_axle_pivot'].pos)

    scene.add(
        {'wheel_circle_right': Sphere(pos=scene.objects['right_axle'].pos-
                                          np.array([scene.axle_length/2,0,0]),
                                         size=[wheel_circle,wheel_circle,wheel_circle])})
    scene.add({'wheel_circle_right_pivot': Box(pos=scene.objects['wheel_circle_right'].pos,
              size=[.1,.1,.1])})

    scene.add(
        {'wheel_circle_left': Sphere(pos=scene.objects['left_axle'].pos+
                                          np.array([scene.axle_length/2,0,0]),
                                         size=[wheel_circle,wheel_circle,wheel_circle],color='purple')})
    scene.add({'wheel_circle_left_pivot': Box(pos=scene.objects['wheel_circle_left'].pos,
              size=[.1,.1,.1],color='green')})


    #scene.steering_arm.plot(plt)

    # create vector for tie rod direction
    tie_rod_right_v = scene.objects['right_control_arm_pivot'].pos - scene.objects['steering_arm_right_pivot'].pos
    #tie_rod_right_v = tie_rod_right_v / np.linalg.norm(tie_rod_right_v)
    tie_rod_left_v = scene.objects['left_control_arm_pivot'].pos - scene.objects['steering_arm_left_pivot'].pos
    scene.add({'tie_rod_right': Box(pos=scene.objects['steering_arm_right_pivot'].pos+np.array([tie_rod_length/2,0,0]),
                      size=np.array([tie_rod_length,.375,.375]), color='yellow')})




    # rotate to axis
    axis = tie_rod_right_v
    if not np.allclose(axis, np.array([1, 0, 0])):
        R = RU(np.array([1, 0, 0]), axis)
        scene.objects['tie_rod_right'].rotate_translate(R)

    #scene.add({'tie_rod_left': Box(pos=scene.objects['steering_arm_left_pivot'].pos-np.array([tie_rod_length/2,0,0]),
    #                  size=np.array([tie_rod_length,.375,.375]))})

    #scene.add({'line': Line(pos=np.array([0, 0, 0]), end_pos=np.array([1, 2, 0]))})


    # ax1.set_xlim((mm.min_x, mm.max_x))
    # ax1.set_ylim((-270, 270))

    # scene.ax1.set_title('steer')

    # plt.xlabel('x')
    # plt.ylabel('y')



    # http://stackoverflow.com/questions/30482727/pyplot-setting-grid-line-spacing-for-plot
    # intervals = 1 # 1 dB ticks
    # loc = matplotlib.ticker.MultipleLocator(base=intervals)
    # self.ax1.yaxis.set_major_locator(loc)
    # scene.ax1.grid(b=True, which='major')



    plt.ion()
    #plt.show()
    for i in range(1):
        for a in [70,60,50,40,30,20,10,0]:
        #for a in [60]:
        #for a in [0]:
            print(a)
            plt.figure(figsize=(8,8))
            plt.gcf().clear()

            #steering_angle = \
            update_scene(scene,a)
            #if a != 0:
            #    print("gain %0.2f"%(steering_angle/a))

            #scene.steering_arm.plot(plt)
            plt.draw()
            #time.sleep(1.0)
            plt.pause(.005)
            #input('test')

            # put it back to starting location
            # scene.objects['steering_arm'].rotate(angle=rad(-a), axis=np.array([0, 0, 1]), origin=origin)
            # scene.objects['steering_arm_right_pivot'].rotate(angle=rad(-a), axis=np.array([0, 0, 1]), origin=origin)
            # scene.objects['steering_circle_right'].rotate(angle=rad(-a), axis=np.array([0, 0, 1]), origin=origin)

            # time.sleep(0.1)


    # plt.show()
    plt.ioff()
    plt.show()

def unit_v(v):
    return v/np.linalg.norm(v)

# get rotation matrix from two vectors
# https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
def ssc(v):
    return np.array([[0, -v[2], v[1]],
    [v[2], 0, -v[0]],
    [-v[1], v[0], 0]])


def RU(A,B):
    return (np.eye(3) + ssc(np.cross(A,B)) +
        ssc(np.cross(A,B))**2*(1-np.dot(A,B))/(np.linalg.norm(np.cross(A,B))**2))


# import docopt

# https://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python/13849249#13849249

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    # return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
    return np.arctan2(v1_u[1],v1_u[0]) - np.arctan2(v2_u[1],v2_u[0])


def rad(deg):
    return deg*(np.pi/180)

def deg(rad):
    return rad*(180/np.pi)

class Box():
    def __init__(self,pos, size, axis=np.array([1,0,0]),color = 'red',name=''):
        self.color = color
        self.pos = pos
        self.axis = unit_v(axis)
        self.size = size
        self.pts = np.array([[-self.size[0] / 2,-self.size[1] / 2, 0],
                    [self.size[0] / 2, -self.size[1] / 2, 0],
                    [self.size[0] / 2, self.size[1] / 2, 0],
                    [-self.size[0] / 2, self.size[1] / 2, 0],
                    [-self.size[0] / 2, -self.size[1] / 2, 0]])
        # self.x = [-self.size[0] / 2, self.size[0] / 2, self.size[0] / 2, -self.size[0] / 2, -self.size[0] / 2]
        # self.y = [-self.size[1] / 2, -self.size[1] / 2, self.size[1] / 2, self.size[1] / 2, -self.size[1] / 2]
        for i, s in enumerate(self.pts):
            self.pts[i] = np.add(s, self.pos)
            # self.y = np.add(self.y, self.pos)
        # rotate to axis
        if not np.allclose(self.axis,np.array([1,0,0])):
            R = RU(np.array([1,0,0]),self.axis)
            self.rotate_translate(R)

    def plot(self,plt,label):
        x,y,z = self.pts.T
        plt.plot(x,y,color=self.color, label=label)

    def rotate(self,angle,axis,origin):
        # http://www.glowscript.org/docs/VPythonDocs/rotation.html
        T = [0,0,0] # translations
        T = origin
        ai, aj, ak = transforms3d.euler.axangle2euler(axis, angle)
        # R = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]  # rotation matrix
        R = transforms3d.euler.euler2mat(ai, aj, ak, 'syxz')
        Z = [0.0, 0.0, 0.0]  # zooms
        A = transforms3d.affines.compose(T, R, Z)

        for i, s in enumerate(self.pts):
            # https://cseweb.ucsd.edu/classes/wi18/cse167-a/lec3.pdf
            # self.pts[i] = np.delete(np.dot(np.append(s,1),A),-1)

            # https://www.mathworks.com/matlabcentral/answers/93554-how-can-i-rotate-a-set-of-points-in-a-plane-by-a-certain-angle-about-an-arbitrary-point
            s1 = s - T # first translate
            s2 = np.dot(s1, R[:3,:3].T) # then rotate
            s3 = s2 + T  # then restore
            self.pts[i] = s3

        # also rotate defined location of object
        self.pos = self.pos - T  # first translate
        self.pos = np.dot(self.pos,R[:3,:3].T)
        self.pos = self.pos + T  # then restore

    def rotate_translate(self, R, T = np.array([0, 0, 0])):
        # http://www.glowscript.org/docs/VPythonDocs/rotation.html

        for i, s in enumerate(self.pts):
            # https://cseweb.ucsd.edu/classes/wi18/cse167-a/lec3.pdf
            # self.pts[i] = np.delete(np.dot(np.append(s,1),A),-1)

            # https://www.mathworks.com/matlabcentral/answers/93554-how-can-i-rotate-a-set-of-points-in-a-plane-by-a-certain-angle-about-an-arbitrary-point
            s1 = s - T  # first translate
            s2 = np.dot(s1, R[:3, :3].T)  # then rotate
            s3 = s2 + T  # then restore
            self.pts[i] = s3

        # also rotate defined location of object
        self.pos = self.pos - T  # first translate
        self.pos = np.dot(self.pos, R[:3, :3].T)
        self.pos = self.pos + T  # then restore


class Sphere(Box):
    def __init__(self, pos, size, color='blue'):
        self.color = color
        self.pos = pos
        # self.axis = axis
        self.size = size
        self.pts = np.empty((0,3),float)
        for i in range(0,360,1):
            self.pts = np.append(self.pts,np.array([[self.size[0]*np.sin(rad(i)),self.size[1]*np.cos(rad(i)),0]]),axis=0)
        #print(self.pts)
        for i, s in enumerate(self.pts):
            self.pts[i] = np.add(s, self.pos)
            # self.y = np.add(self.y, self.pos)

class Line(Box):
    def __init__(self, pos, end_pos, color='green'):
        self.color = color
        self.pos = pos
        # self.axis = axis
        self.size = np.linalg.norm(pos - end_pos)
        self.pts = np.empty((0, 3), float)
        self.pts = np.array([self.pos,
                             end_pos])
        #print(self.pts)


# https://gist.github.com/mcleonard/5351452
class Vector():
    def __init__(self,*v):
        self.values = np.array(v)

    def __sub__(self, other):
        """ Returns the vector difference of self and other """
        subbed = tuple( a - b for a, b in zip(self, other) )
        return Vector(*subbed)

    def __getitem__(self, key):
        return self.values[key]

class Scene():
    def __init__(self):
        self.objects = {}

    def add(self, dict):
        self.objects.update(dict)

def update_scene(scene,steering_angle, calc=True):
    # update all related objects
    # scene.box.plot(plt)
    # scene.right_wheel.plot(plt)
    # scene.right_axle.plot(plt)
    # scene.right_control_arm.plot(plt)
    # scene.steering_arm.plot(plt)
    #
    #
    # scene.steering_arm_right_pivot.plot(plt)
    # scene.steering_circle_right.plot(plt)
    #
    # scene.wheel_circle_right.plot(plt)
    # scene.wheel_circle_right_pivot.plot(plt)
    #
    # scene.control_arm.plot(plt)
    # scene.line.plot(plt)




    plt.xlabel('x')
    plt.ylabel('y')

    #plt.axes().set_aspect('equal', 'datalim')
    #plt.xlim((-20, 20))
    plt.axis('equal')
    plt.axis([-15, 15,-50,10])


    angle = steering_angle
    if calc:
        # left side
        origin = scene.objects['steering_arm_pivot'].pos
        scene.objects['steering_arm'].rotate(angle=rad(-steering_angle), axis=np.array([0, 0, 1]), origin=origin)
        scene.objects['steering_arm_left_pivot'].rotate(angle=rad(-steering_angle), axis=np.array([0, 0, 1]), origin=origin)
        scene.objects['steering_circle_left'].rotate(angle=rad(-steering_angle), axis=np.array([0, 0, 1]), origin=origin)

        r1 = scene.objects['steering_circle_left'].size[0]
        r2 = scene.objects['wheel_circle_left'].size[0]
        pos1 = scene.objects['steering_arm_left_pivot'].pos
        # pos2 =np.array([10,0,0])
        pos2 = scene.objects['wheel_circle_left_pivot'].pos
        pos = solve_positions(r1, pos1, r2, pos2)
        x = pos[0]
        a = pos[1]
        # create rotation matrix
        R = np.array([[pos[0], pos[1], 0],
                      [pos1[1], pos1[0], 0],
                      [0, 0, 1]])
        # v = np.array([pos[0],pos[1],0]) + pos1
        v = np.dot(pos, R[:3, :3].T)
        # plt.plot([pos1[0],v[0]],[pos1[1],v[1]])

        # draw a line from center to center
        #scene.add({'line_center_to_center_left': Line(pos=pos1, end_pos=pos2)})
        # https://math.stackexchange.com/questions/83404/finding-a-point-along-a-line-in-three-dimensions-given-two-points
        # draw a line in that direction but only as long as x
        # create vector
        v = pos2 - pos1
        # normalize
        v = v / np.linalg.norm(v)
        # point of distance x
        v_x = pos1 + x * v
        #scene.add({'line_center_to_intersection_left': Line(pos=pos1, end_pos=v_x, color='cyan')})
        # create vector for a perpendicular to d

        v_a = np.cross(v, np.array([0, 0, -1]))
        # normalize
        v_a = v_a / np.linalg.norm(v)
        # scale
        v_a_end = v_x + a * v_a
        #scene.add({'line_to_intersection_point_left': Line(pos=v_x, end_pos=v_a_end)})

        # draw a line for tie rod
        scene.add({'tie_rod_left': Line(pos=pos1, end_pos=v_a_end)})

        # update position of wheels

        # calculate angle between current position and intersection point
        # around wheel origin

        wheel_origin_left = wheel_origin = scene.objects['wheel_circle_left_pivot'].pos

        current_pos = scene.objects['left_control_arm_pivot'].pos
        intersect_point_left = intersect_point = v_a_end

        angle = angle_between(intersect_point - wheel_origin,current_pos - wheel_origin )


        print("left wheel angle: %0.2f" %deg(angle))
        scene.objects['left_axle'].rotate(angle=angle, axis=np.array([0, 0, 1]), origin=wheel_origin)
        scene.objects['left_control_arm'].rotate(angle=angle, axis=np.array([0, 0, 1]), origin=wheel_origin)
        scene.objects['left_control_arm_pivot'].rotate(angle=angle, axis=np.array([0, 0, 1]), origin=wheel_origin)
        scene.objects['left_wheel'].rotate(angle=angle, axis=np.array([0, 0, 1]), origin=wheel_origin)

        #scene.add({'debug': Line(pos=current_pos, end_pos=current_pos, color='red')})
        # right side
        scene.objects['steering_arm_right_pivot'].rotate(angle=rad(-steering_angle), axis=np.array([0, 0, 1]), origin=origin)
        scene.objects['steering_circle_right'].rotate(angle=rad(-steering_angle), axis=np.array([0, 0, 1]), origin=origin)

        r1 = scene.objects['steering_circle_right'].size[0]
        r2 = scene.objects['wheel_circle_right'].size[0]
        pos1 = scene.objects['steering_arm_right_pivot'].pos
        # pos2 =np.array([10,0,0])
        pos2 = scene.objects['wheel_circle_right_pivot'].pos
        pos = solve_positions(r1, pos1, r2, pos2)
        x = pos[0]
        a = pos[1]
        # create rotation matrix
        R = np.array([[pos[0], pos[1], 0],
                      [pos1[1], pos1[0], 0],
                      [0, 0, 1]])
        # v = np.array([pos[0],pos[1],0]) + pos1
        v = np.dot(pos, R[:3, :3].T)
        # plt.plot([pos1[0],v[0]],[pos1[1],v[1]])

        # draw a line from center to center
        #scene.add({'line_center_to_center': Line(pos=pos1, end_pos=pos2)})
        # https://math.stackexchange.com/questions/83404/finding-a-point-along-a-line-in-three-dimensions-given-two-points
        # draw a line in that direction but only as long as x
        # create vector
        v = pos2 - pos1
        # normalize
        v = v / np.linalg.norm(v)
        # point of distance x
        v_x = pos1 + x * v
        #scene.add({'line_center_to_intersection': Line(pos=pos1, end_pos=v_x, color='cyan')})
        # create vector for a perpendicular to d

        v_a = np.cross(v, np.array([0, 0, 1]))
        # normalize
        v_a = v_a / np.linalg.norm(v)
        # scale
        v_a_end = v_x + a * v_a
        #scene.add({'line_to_intersection_point': Line(pos=v_x, end_pos=v_a_end)})

        # draw a line for tie rod
        scene.add({'tie_rod_right': Line(pos=pos1, end_pos=v_a_end)})

        # update position of wheels

        # calculate angle between current position and intersection point
        # around wheel origin


        wheel_origin = scene.objects['wheel_circle_right_pivot'].pos

        current_pos = scene.objects['right_control_arm_pivot'].pos
        intersect_point = v_a_end

        angle = angle_between(intersect_point - wheel_origin,current_pos - wheel_origin )

        #scene.add({'debug': Line(pos=current_pos, end_pos=intersect_point)})

        print(deg(angle))
        scene.objects['right_axle'].rotate(angle=angle, axis=np.array([0, 0, 1]), origin=wheel_origin)
        scene.objects['right_control_arm'].rotate(angle=angle, axis=np.array([0, 0, 1]), origin=wheel_origin)
        scene.objects['right_control_arm_pivot'].rotate(angle=angle, axis=np.array([0, 0, 1]), origin=wheel_origin)
        scene.objects['right_wheel'].rotate(angle=angle, axis=np.array([0, 0, 1]), origin=wheel_origin)



    # calculate wheel angle
    wheel_angle_left = 10
    wheel_angle_right = 10
    if calc:
        wheel_angle_left = angle_between(np.array([0, -1, 0]), intersect_point_left - wheel_origin_left)
        wheel_angle_right = angle_between(np.array([0, -1, 0]), intersect_point - wheel_origin)
        # print(wheel_angle_left)
        # print(wheel_angle_right)

    # adjust for wheel offset from kingpin
    if wheel_angle_left < 0:
        offset = - scene.axle_length - scene.wheel_width / 2
    else:
        offset = + scene.axle_length + scene.wheel_width / 2

    # calculate turning radius
    # for outside front wheel
    # http://www.davdata.nl/math/turning_radius.html
    alpha = abs(wheel_angle_right)
    R_right = scene.wheel_base/(np.sin(alpha)) - offset
    #D_feet = np.floor(R_right*2/12)
    #D_inches = (R_right*2/12-D_feet)*12

    # https://www.quora.com/How-can-a-cars-turning-radius-be-reduced-1
    # calculate turning radius
    # for inside front wheel

    alpha = abs(wheel_angle_left)
    R_left = scene.wheel_base/np.sin(alpha) + offset
    #D_l_feet = np.floor(R_left*2/12)
    #D_l_inches = (R_left*2/12-D_l_feet)*12

    # calculate ideal outside angle
    if wheel_angle_left < 0:
        # turning left
        alpha_in = abs(wheel_angle_left)
    else:
        alpha_in = abs(wheel_angle_right)
    # R_inside is square with front inside kingpin
    R_inside = scene.wheel_base / np.tan(alpha_in)
    # R_outside is square with front outside kingpin
    kingpin_offsets = scene.axle_length*2 + scene.wheel_width
    R_outside = scene.track_width-kingpin_offsets+R_inside
    alpha_out = np.arctan(scene.wheel_base / R_outside)
    #alpha_out = np.arctan(scene.wheel_base / ((scene.wheel_base / np.tan(alpha_in) + scene.track_width)))
    # alpha_out = np.arctan(scene.wheel_base / ((scene.wheel_base / np.tan(alpha_in) + scene.track_width)))

    def feet_inches(inches):
        if inches == np.inf:
            s = "inf"
        else:
            feet = np.floor(inches / 12)
            inches = (inches / 12 - feet) * 12
            s = "%0.2f\", %0.2f'"%(feet, inches)
        return s

    R_out_ackerman = scene.wheel_base/np.sin(alpha_out)+scene.axle_length + scene.wheel_width / 2
    #R_out_ackerman = R_outside
    #R_out_ackerman = R_inside
    #(D_out_ackerman_feet, D_out_ackerman_inches) = feet_inches(R_out_ackerman*2)

    # calculate turning center

    R_rear_left = scene.wheel_base / np.tan(abs(wheel_angle_left)) + offset
    R_rear_right = scene.wheel_base / np.tan(abs(wheel_angle_right)) - offset
    print("R_rear_right %0.2f"%R_rear_right)
    # adjust for wheel offset from kingpin
    if wheel_angle_left < 0:
        turning_circle_left_pos = np.array([-R_rear_left - scene.track_width / 2, -scene.wheel_base, 0])
        turning_circle_right_pos = np.array([-R_rear_right + scene.track_width / 2, -scene.wheel_base, 0])
        turning_circle_inside_pos = turning_circle_left_pos
        # what is the difference between actual and ideal ackerman in radians
        ackerman_delta_angle = alpha_out - abs(wheel_angle_right)
    else:
        turning_circle_left_pos = np.array([R_rear_left - scene.track_width / 2, -scene.wheel_base, 0])
        turning_circle_right_pos = np.array([R_rear_right + scene.track_width / 2, -scene.wheel_base, 0])
        turning_circle_inside_pos = turning_circle_right_pos
        # what is the difference between actual and ideal ackerman in radians
        ackerman_delta_angle = alpha_out - wheel_angle_left

    # adjust for wheel offset from kingpin
    R_turning_circle = R_rear_left

    print(turning_circle_left_pos)
    print(R_left)
    scene.add({'turning_circle_rear_left': Sphere(pos=turning_circle_left_pos,
                                         size=np.ones([3,1])*R_turning_circle,color='red')})

    scene.add({'turning_circle_rear_left_pos': Sphere(pos=turning_circle_left_pos,
                                         size=np.ones([3,1])*.01,color='red')})

    R_turning_circle = R_left

    scene.add({'turning_circle_front_left': Sphere(pos=turning_circle_left_pos,
                                         size=np.ones([3,1])*R_turning_circle)})
    scene.add({'turning_circle_front_left_pos': Sphere(pos=turning_circle_left_pos,
                                         size=np.ones([3,1])*.02)})

    R_turning_circle = R_right

    scene.add({'turning_circle_front_right': Sphere(pos=turning_circle_right_pos,
                                         size=np.ones([3,1])*R_turning_circle,color='green')})
    scene.add({'turning_circle_front_right_pos': Sphere(pos=turning_circle_right_pos,
                                         size=np.ones([3,1])*.03,color='green')})

    print('right:')
    print(turning_circle_right_pos)
    print(R_right)

    R_turning_circle = R_rear_right

    scene.add({'turning_circle_rear_right': Sphere(pos=turning_circle_right_pos,
                                         size=np.ones([3,1])*R_turning_circle,color='cyan')})
    scene.add({'turning_circle_rear_right_pos': Sphere(pos=turning_circle_right_pos,
                                         size=np.ones([3,1])*.04,color='cyan')})


    R_turning_circle = R_out_ackerman

    scene.add({'turning_circle_out_ackerman': Sphere(pos=turning_circle_inside_pos,
                                         size=np.ones([3,1])*R_turning_circle,color='magenta')})
    scene.add({'turning_circle_out_ackerman_pos': Sphere(pos=turning_circle_inside_pos,
                                         size=np.ones([3,1])*.05,color='magenta')})


    plt.title('steering angle %0.2f,angle left %0.2f, angle right %0.2f,\n'
              'left turning circle: %s, right turning circle: %s\n' 
              'ideal ackerman outer turning circle: %s, ackerman delta %0.2f deg'%
              (steering_angle,deg(wheel_angle_left),deg(wheel_angle_right),
               feet_inches(R_left * 2),
               feet_inches(R_right * 2),
               feet_inches(R_out_ackerman * 2),
               deg(ackerman_delta_angle),
               ),fontsize=8)

    plt.grid(b=True, which='major')
    # actually update plot
    for key, value in scene.objects.items():
        value.plot(plt, label=key)


    # scene.ax1.legend(loc='center left')
    # Shrink current axis by 20%
    #box = scene.ax1.get_position()
    plt.subplots_adjust(left=0.2, right=0.7, top =.9, bottom = 0.1)
    #scene.ax1.set_position([box.x0, box.y0, box.width * 0.8, box.height])
    plt.legend(loc='upper left', bbox_to_anchor=(1,1),prop={'size':6})


    # undo
    scene.objects['steering_arm_left_pivot'].rotate(angle=rad(steering_angle), axis=np.array([0, 0, 1]), origin=origin)
    scene.objects['steering_circle_left'].rotate(angle=rad(steering_angle), axis=np.array([0, 0, 1]), origin=origin)

    origin = scene.objects['steering_arm_pivot'].pos
    scene.objects['steering_arm'].rotate(angle=rad(steering_angle), axis=np.array([0, 0, 1]), origin=origin)
    scene.objects['steering_arm_right_pivot'].rotate(angle=rad(steering_angle), axis=np.array([0, 0, 1]), origin=origin)
    scene.objects['steering_circle_right'].rotate(angle=rad(steering_angle), axis=np.array([0, 0, 1]), origin=origin)

    return



def solve_positions(r1, pos1, r2, pos2):
    # pos1 is pivot point of control arm near steering
    # pos2 is pivot point of wheel
    # calculates position of end of control arm
    # http://mathworld.wolfram.com/Circle-CircleIntersection.html
    # inputs will be in 3d, but presuming z = 0

    R = r1
    r = r2

    # find distance between circles
    d = np.linalg.norm(pos1 - pos2)
    x = (d**2-r**2+R**2)/(2*d) # distance along center line to intersection
    a = 1/d*np.sqrt(4*d**2*R**2-(d**2-r**2+R**2)**2) # distance between two intersection points perpendicular to centerline

    pos = np.array([x,a/2,0])

    return pos


if __name__ == '__main__':
    run()
