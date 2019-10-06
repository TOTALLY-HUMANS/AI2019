import pygame
import sys
from pygame.locals import *


FPS=30
WINDOWWIDTH = 500
WINDOWHEIGHT = 500
COLOR = { "white": (255, 255, 255),
          "black": (0, 0, 0),
          "green": (0, 255, 0),
          "blue": (0, 0, 255),
          "red": (255, 0, 0),
          "purple": (128, 0, 128)
        }
        
        

  
class Visualisation():

  def __init__(self):
    self.pygame = pygame
    self.fpsClock = self.pygame.time.Clock()
    self.main_clock = self.pygame.time.Clock()
    self.window = self.pygame.display.set_mode((WINDOWWIDTH, WINDOWHEIGHT), 0, 32)
    self.pygame.display.set_caption("AI2019 Viz")

    
  def draw_prior_map(self):
    self.pygame.draw.line(self.window, COLOR["green"], (50,50), (450,50))
    self.pygame.draw.line(self.window, COLOR["green"], (450,50), (450,450))
    self.pygame.draw.line(self.window, COLOR["green"], (450,450), (50,450))
    
    self.pygame.draw.line(self.window, COLOR["green"], (50,450),
   (50,50))
    self.pygame.draw.line(self.window, COLOR["green"], (350,50), (450,150))
        
    self.pygame.draw.line(self.window, COLOR["green"], (50,350), (150,450))
  
  #converts the real coordinate to pixel coordinates
  def convert_coordinates(self, point):
    return (int(point[0], int(point[1])))
  
  def viz(self):
    #pygame_.draw.circle(window, COLOR["green"], (2,2), 3)
    self.draw_prior_map()
    self.fpsClock.tick(FPS)
    self.pygame.display.update()
    
  def test_end(self, event):
    if event.type == QUIT:
        self.pygame.quit()
        sys.exit()
  def clear(self):
    """Fill the background white"""
    self.window.fill(COLOR["white"])
  
viz = Visualisation()


while 1:
  for event in viz.pygame.event.get():
    viz.test_end(event)
  viz.clear()
  viz.viz()
