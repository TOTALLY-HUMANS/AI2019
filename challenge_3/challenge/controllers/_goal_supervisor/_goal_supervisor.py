import math
import os

from controller import Supervisor

supervisor = Supervisor()

timestep = int(supervisor.getBasicTimeStep())
print("Suvervisor starting with timestep of %i" % timestep)

blue_score = 0
red_score = 0
blue_greens = 0
red_greens = 0

last_scorer = 0

game_done = 0

blue_goal = supervisor.getFromDef("_blue_goal")
red_goal = supervisor.getFromDef("_red_goal")

def updateScore():
    print("Setting labels.")
    supervisor.setLabel(0, "RED " + str(red_score), 0, 0, 0.1, 0xff0000, 0, "Impact")
    supervisor.setLabel(1, "BLUE " + str(blue_score), 0, 0.05, 0.1, 0x0000ff, 0, "Impact")

# Ends the game: result 1 means blue won, result -1 means red won,
# result 0 means the game was a draw
def endGame(result):
    global game_done
    if(game_done):
        return
    game_done = 1
    ## Echos the result into file
    # echo %i > RESULT would put it in the _goal_supervisor folder, using home for now
    os.system("echo %i > ~/RESULT" % result)


def check_goals():

    def check_goal(ball, goal):
        ball_pos = ball.getPosition()
        goal_pos = goal.getPosition()
        goal_angle = goal.getField("rotation").getSFRotation()[
            3]  # [0, -1, 0, angle]
        goal_width = 1  # meter
        goal_precision = 0.05  # meter

        # pole vector
        pole_x = math.cos(goal_angle)*goal_width/2
        pole_z = math.sin(goal_angle)*goal_width/2

        # distance between a point (a ball) to a line segment (a goal)
        dot_product = (ball_pos[0]-goal_pos[0]) * \
            pole_x + (ball_pos[2]-goal_pos[2]) * pole_z
        scale = max(-1, min(1, dot_product / (pole_z**2 + pole_x**2)))
        proj_x = goal_pos[0] + scale * pole_x
        proj_z = goal_pos[2] + scale * pole_z
        distance2 = (proj_x - ball_pos[0])**2 + (proj_z - ball_pos[2])**2

        is_goal = distance2 < goal_precision**2
        return is_goal

    def loop_balls(group_name, color):
        global red_score, blue_score, red_greens, blue_greens, last_scorer
        root = supervisor.getFromDef(group_name)
        root = root.getField("children")
        ballCount = root.getCount()
        for i in range(ballCount - 1, -1, -1):
            ball = root.getMFNode(i)

            if check_goal(ball, blue_goal):
                blue_score += 1 if color == "yellow" else -1
                last_scorer = "blue" if color == "yellow" else "red"
                if color == "green":
                    blue_greens += 1
                updateScore()
                root.removeMF(i)
            elif check_goal(ball, red_goal):
                red_score += 1 if color == "yellow" else -1
                last_scorer = "red" if color == "yellow" else "blue"
                if color == "green":
                    red_greens += 1
                updateScore()
                root.removeMF(i)

    loop_balls("_yellow_balls", "yellow")
    loop_balls("_green_balls", "green")

def check_winning_conditions(t):
    global blue_score, red_score, red_greens, blue_greens, last_scorer
    if red_greens >= 3:
        supervisor.setLabel(1, "BLUE\nWINS", 0.02, 0.1, 0.1, 0xffffff, 0, "Impact")
        endGame(1)
    if blue_greens >= 3:
        supervisor.setLabel(1, "RED\nWINS", 0.02, 0.1, 0.1, 0xffffff, 0, "Impact")
        endGame(-1)

    # Check time limit
    if t > 120:
        if blue_score > red_score:
            endGame(1)
        elif red_score > blue_score:
            endGame(-1)
        elif last_scorer == "red":
            endGame(1)
        elif last_scorer == "blue":
            endGame(-1)
        else:
            endGame(0)
 
updateScore()
t = 0.
while supervisor.step(timestep) != -1:
    check_goals()
    check_winning_conditions(t)
    t += timestep * 0.001
    

