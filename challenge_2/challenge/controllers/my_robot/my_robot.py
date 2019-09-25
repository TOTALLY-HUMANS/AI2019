"""EDIT THIS FILE

InvaderBot is a simple helper class which has methods to control the robot.
It offers an interface over Webots' default motor controller.
You can use it, modify it or you can also use Webots' own commands.
Using the helper class is recommended because it will later help you to
transfer your code into a physical robot, which is running on Raspberry Pi."""

from invader_bot import InvaderBot
from cv_connector import OpenCVConnector
import time
import math

bot = InvaderBot()
bot.setup()

kb = bot.robot.getKeyboard()
kb.enable(bot.timestep)

print("Starting OpenCVConnector...")
connector = OpenCVConnector()
connector.initConnection()
print("Started.")

DEG2RAD = math.pi/180.0
RAD2DEG = 180.0/math.pi

        

def limit_controls(x_control, y_control):

    while abs(x_control) > 1.0 or abs(y_control) > 1.0:
        y_control = y_control/2.0
        x_control = x_control/2.0
    return x_control, y_control
    
def controls(vel, omega):
    R = 0.05
    L = 0.2
    V_r = (2.0*vel + omega*L)/(2.0*R)
    V_l = (2.0*vel - omega*L)/(2.0*R)
    print("left_control: " +str(V_l) +" right control: " +str(V_r))
    
    min_v = min(V_r, V_l)
    max_v = max(V_r, V_l)
     
    mean = (min_v + max_v)/2.0
        

    V_r, V_l = limit_controls(V_r, V_l)
    print("scaled left_control: " +str(V_l) +" scaled right control: " +str(V_r))

    bot.set_motors(V_r , V_l)
    bot.step()
   
# main loop
print("Starting main loop...")


#pi to pi
def normalize_angle(angle):
    if  angle < -2.0*math.pi or angle > 2*2.0*math.pi:
        n   = math.floor(angle/(2.0*math.pi))
        angle = angle - n*(2.0*math.pi)
    

    if angle > math.pi:
        angle = angle - (2.0*math.pi)

    if angle < -math.pi:
        angle = angle + (2.0*math.pi)

    return angle


class PID:
    def __init__(self, kp, kd, ki):
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0
        self.prev_error = 0.0
        
        self.k_p = kp
        self.k_d = kd
        self.k_i = ki
        
    def pid(self, error):
    
        self.p_error = error
        self.i_error += error
        self.d_error = error-self.prev_error
        self.prev_error = error
        
        total_error = -(self.k_p*self.p_error + self.k_i*self.i_error + self.k_d*self.d_error)
        return total_error






def generate_trajectory():
    X=[]
    Y=[]
    pose = bot.get_position()

    for y in range(0,30,1):
        y = y/10.0
        x=0.2*y**3 
        X.append(x + pose[0])
        Y.append(y + pose[1])
    return X,Y
        





def simple_follow_waypoints(wp_x, wp_y):

    wp_idx = 0
    speed = 0.1
    yaw_pid = PID(0.8,0.0, 0.0)

    while wp_idx < len(wp_x):
        pose = bot.get_position()
        curr_x = wp_x[wp_idx]
        curr_y = wp_y[wp_idx]
        d = math.sqrt((curr_x - pose[0])**2 + (curr_y - pose[1])**2)
        goal_theta = math.atan2(curr_y-pose[1], curr_x-pose[0])
        theta_error =   normalize_angle((pose[2] -90.0)*DEG2RAD  -goal_theta  )
    
        if d < 0.20:
            wp_idx = wp_idx + 1
            controls(0.0, 0.0)
            continue
        yaw_control = yaw_pid.pid(theta_error)
        controls(speed, yaw_control)

        

        print("goal " +str(curr_x) +", " +str(curr_y))
        print("d " +str(d))
        print("theta_error " +str(theta_error) )
        print("curr_theta " +str((pose[2] -90.0)*DEG2RAD) )
        print("goal_theta " +str(goal_theta) )

        print("curr " +str(pose[0]) + ", " +str(pose[1]))
        

        print("wp_idx: "+str(wp_idx))

    controls(0.0, 0.0)
 



print(generate_trajectory())
#X,Y = generate_trajectory()
X = [1.0, 1.0, -1.0, -1.0, -1.0, 0.0, 1.0]
Y = [1.0, -1.0, -1.0, 1.0, 1.0, 0.0, 1.0]
simple_follow_waypoints(X, Y)


print("my_robot is kill")

