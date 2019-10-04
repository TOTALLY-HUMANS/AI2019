### Do not touch this file. ###
# This file will be replaced with the originial version when
# your solution is checked. So, modifing this file does not
# help you to solve the challenge.

from controller import Supervisor
import math

# supervision can access to other objects
supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

player = supervisor.getFromDef("my_robot")
coin_root = supervisor.getFromDef("_coins").getField("children")

# distance when the coin is concerned as collected
ACCEPT_DISTANCE = 0.5


def update():
    count = coin_root.getCount()

    for idx in reversed(range(count)):
        coin = coin_root.getMFNode(idx)

        pos = coin.getPosition()
        player_pos = player.getPosition()

        dist = math.sqrt(sum([(a - b) ** 2 for a, b in zip(pos, player_pos)]))

        if dist < ACCEPT_DISTANCE:
            coin_root.removeMF(idx)  # remove the collected coin
            return

        # rotate the coin, just for viusal effect
        rotationField = coin.getField("rotation")
        angle = rotationField.getSFRotation()
        angle[3] = angle[3] + 0.1
        rotationField.setSFRotation(angle)

    # inform the user about the coins
    supervisor.setLabel(0, "COINS LEFT: %d" %
                        (count), 0.02, 0.02, 0.1, 0xffffff, 0, "Impact")
    if count == 0:
        supervisor.setLabel(1, "MISSION\nCOMPLETED",
                            0.02, 0.1, 0.1, 0xffffff, 0, "Impact")


def reset():

    # remove previous coins
    count = coin_root.getCount()
    for idx in reversed(range(count)):
        coin_root.removeMF(idx)

    # reset the player pos
    translation = player.getField("translation")
    translation.setSFVec3f([0, 0, 1.5])
    rotation = player.getField("rotation")
    rotation.setSFRotation([0, -1, 0, 1.5*math.pi])

    # create the coins
    def make_coin(pos):
        coin_root.importMFNode(0, "../../protos/_coin.wbo")
        coin = coin_root.getMFNode(0)
        translation = coin.getField("translation")
        translation.setSFVec3f(pos)

    make_coin([0, 0, -1])
    make_coin([1.5, 0, -1.5])
    make_coin([-1.5, 0, -1.5])


# Main loop
reset()
while supervisor.step(timestep) != -1:
    update()
