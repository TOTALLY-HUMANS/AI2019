# Artificial Invaders

![Challenge 3](img/webots final.png "Webots Final Challenge (Challenge 3)")


[Artificial Invaders](https://robotuprising.fi/hackathon/artificial-invaders/) is a story-driven, strategic robotics game designed for hacker teams experienced in Artificial Intelligence and Robotics.

Artificial Invaders consists of two phases: ``pre-challenges`` and ``a physical robot battle tournament``. The pre-challenges are part of our qualification process of the tournament. The best 10 teams will be selected to fight for supremacy of the Robot Uprising universe. 

Schedule:

- Pre-challenges (1st phase) 19.06.-1.9.19
- Tournament (2nd phase) 11.-13.10.19

## Pre-Challenges

Pre-challenges test the skills of your team and also help you learn some of the technologies involved. The pre-challenge phase will culminate into a full battle against other competitors on the simulation platform.

The simulation platform is [``Webots``](https://cyberbotics.com/), which is an open source robotics simulation software. It is available for ``Windows``, ``Linux`` and ``macOS``.

In Webots, you can easily model custom robots and control them by using ``Python``. The code that is written for the simulation can be reused easily with the real physical robot running on for instance Arduino or Rasberry Pi.

For the pre-challenges, we will provide you a premade robot model and the game arena. So, you can focus on the crucial robotic skills, e.g. coding and AI. In other words, there is no need to spend time learning Webots thoroughly (unless you want to!).

## How to Start

Fork this repo to start working on the challenges! 

The challenge grading is done fully automatically, just push your code to the repo and the CI will do the rest.

### Setup process

1. Fork
2. Go to "settings" -> "CI / CD" -> "Runners" and enable shared runners
3. Update your team info into the file `team_info.txt`
    - Add a public team name
    - Add a secret name for your team: this will be used to verify that no one else have submitted stuff by your team name
    - Add a contact email for your team
    - Add team member names: these can be nicknames or real names
4. To receive the new challenges and other updates to this repo, set up [**Repository Mirroring**](https://about.gitlab.com/2016/12/01/how-to-keep-your-fork-up-to-date-with-its-origin/)
on **your fork of this repository**.
    - Tip: use development branches for each challenge to prevent merge conflicts in the master branch

## Submitting Your Solution

When you feel you're ready to submit,
  1. Copy all files you want to submit into the folder `challenge_n/submission`
  2. Commit & Push: the CI will do the rest

Submissions will be evaluated, and scores will be published [here](http://lab.robotuprising.fi:8080/scoreboard)
    (publishing can take a few days as some of the challenges need to be verified by hand)

## New Challenges

The four pre-challenges will be released bi-weekly.
You'll be notified of updates to the email address you used to register to Artificial Invaders.
All of the challenges are open for submissions until 01.09.2019.
The first three challenges are basic learning tasks and the last, 4th challenge is a full game simulation where you get to train against competing solutions created by opponent teams.

## Contacts

- To point out problems with the prechallenges or to propose a change, [feel free to open an issue.](https://gitlab.com/artificial-invaders-2019/artificialinvaders2019/issues)
- For Questions and discussion about the prechallenges or the event in general, join the official [Robot Uprising community telegram channel](https://t.me/joinchat/ClZIsRcqLWiksObFremapw).
- If you have more personal questions about the application process etc, you can email us at info@robotuprising.fi.
