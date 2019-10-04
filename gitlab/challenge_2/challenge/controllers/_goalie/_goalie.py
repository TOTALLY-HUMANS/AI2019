import math

from invader_bot import InvaderBot

bot = InvaderBot()
bot.setup()

patrolTargets = [[0.5, 1.7], [1.7, 0.5]]
idx = 0
speed = 0.2 # robot speed

# main loop
while bot.step():
    target = patrolTargets[idx%len(patrolTargets)] # get the next target

    pos = bot.get_position()

    dx = target[0] - pos[0] # difference in x
    dz = target[1] - pos[1] # difference in z

    da = math.degrees(math.atan2(dz, dx)) # difference in angle
    da += 90

    ra = pos[2] - da # relative angle between robot orientation and the goal

    if ra > 180:
        ra -= 360
    if ra < -180:
        ra += 360

    """
    turn the face towards the goal; about 10 degrees of precision is sufficient. Then march!
    """
    if ra < - 2:
        bot.set_motors(speed, -speed)
    elif ra > 2:
        bot.set_motors(-speed, speed)
    else:
        bot.set_motors(speed,speed)

    if (abs(dx) + abs(dz) <=0.2): # switch to the next target if close enough
        idx += 1
