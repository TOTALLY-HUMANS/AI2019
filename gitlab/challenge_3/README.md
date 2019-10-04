# Artificial Invaders

![Challenge 3](img/webots_3.png "Webots Challenge 3")


## Game on!

Time for integration and logic. The last released challenge is a full on 
battle arena of action where teams put their mastered skills into use. This 
challenge is a simulation of the physical battle arena where teams will 
eventually be playing against each other in October. The task is simple: create
an AI that beats one that an another team has submitted.

When a solution is submitted it will be run against a solution submitted by 
another applicant team. Be prepared to create solutions for both the red and 
blue side of the arena. Submitted solutions are run two times a week; every 
Wednesday and Sunday night. Teams can keep track of their progress on the 
leaderboard and check the recorded matches at the leaderboard (coming soon).

## Game rules

The game is played between two teams battling each other in short 3 round 
showdowns. Each team has two autonomous robots playing for their side at a 
time. All teams start with base of 0 points that they can either try to 
increase or protect to win the tournament. The points are increased by 
collecting positive energy cores and taking them to the mothership. Points are 
lost by receiving negative energy cores or by receiving other penalties (e.x. 
robot break down during a physical tournament).

If the mothership receives 3 negative energy cores, the mothership is destroyed
and the opponent wins. If both motherships are alive at the end of a 
tournament, the mothership holding greater energy status will overpower their 
opponent and win. In case of a tie, the team that collected their points 
fastest wins.

*Round time is set to **two minutes**. Webots only plays one one round.*

## Points

Energy cores are presented in the challenge as light and hollow objects in the 
area - each worth a different amounts of points. Energy cores are round 
objects that the robots are able to push and also lift if additional arms are 
attached to the robot. Robots are allowed to captivate only one energy core at 
a time. Free pushing of multiple energy cores at the same time is allowed. 
Different energy cores values are:

- Yellow +1
- Green -1

## Submitting Your Solution

When you feel you're ready to submit,
  1. Copy all files you want to submit into the folder `challenge_n/submission`
  2. Commit & Push: the CI will do the rest

Submissions will be evaluated, and scores will be published on the leaderboard (coming soon).
    (publishing can take a few days as some of the challenges need to be verified by hand)

### Submitting to challenge 3

You'll need to provide solutions to both sides: the red and the blue. The code can be same, but it doesn't need to be. Place these solutions into `challenge_3/submission/blue` and `challenge_3/submission/red`.

## Aruco
You'll (probably) need to detect both your robots and the opponents using the Aruco markers on top of them. Use the provided OpenCV Aruco example or make your own.

## Dummy bot
We've provided you a dummy training bot to test your solutions with before submitting.
