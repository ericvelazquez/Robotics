import pygame
from pygame.locals import *
import time

class FrontEnd(object):
    """ Maintains the display and clock.
        Press escape key to quit.
        
        Sub-class this and add custom mousedown(), mouseup(), keydown(), keyup(), update() and draw() routines as needed.
    """
    def __init__(self,width,height,scale=2):
        self.width = width
        self.height = height
        self.scale = scale

        # init pygame
        pygame.init()

        # create window
        self.screen = pygame.display.set_mode((self.width*self.scale,self.height*self.scale))
        pygame.display.set_caption('Robot')
        
        # create surface for drawing
        self.surface = pygame.Surface((self.width,self.height))
        self.surface = self.surface.convert()
        self.surface.fill((255,255,255))
        
        # display surface
        self.screen.blit(self.surface,(0,0))
        pygame.display.flip()
        
        # create update timer
        pygame.time.set_timer(USEREVENT+1, 50)
        self.last_update_time = 0

    def run(self):
        # run until quit signal
        should_stop = False
        while not should_stop:
            # check events in queue
            for event in pygame.event.get():
                if event.type == USEREVENT+1:
                    # custom update routine in sub-class
                    current_time = time.time()
                    if self.last_update_time == 0:
                        self.last_update_time = current_time
                    update_period = current_time - self.last_update_time
                    self.last_update_time = current_time
                    self.update(update_period)
                elif event.type == QUIT:
                    should_stop = True
                elif event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        should_stop = True
                    else:
                        self.keydown(event.key)
                elif event.type == KEYUP:
                    self.keyup(event.key)
                elif event.type == MOUSEBUTTONUP:
                    x = event.pos[0]/float(self.scale)
                    y = self.height-event.pos[1]/float(self.scale)
                    self.mouseup(x,y,event.button)
                elif event.type == MOUSEBUTTONDOWN:
                    x = event.pos[0]/float(self.scale)
                    y = self.height-event.pos[1]/float(self.scale)
                    self.mousedown(x,y,event.button)

            # clear surface to white
            self.surface.fill((255,255,255))
            
            # call custom draw subroutine
            self.draw(self.surface)
            
            # flip vertically so that origin is at bottom left
            flipped = pygame.transform.flip(self.surface,False,True)
            
            # scale surface
            scaled = pygame.transform.smoothscale(flipped,(self.width*self.scale,self.height*self.scale))
            
            # draw surface to screen
            self.screen.blit(scaled,(0,0))
            
            # flip display
            pygame.display.flip()
    
    def mouseup(self,x,y,button):
        """ Mouse up event: override this in your sub-class
            Arguments:
                x: mouse x position
                y: mouse y position
                button: mouse button
        """
        pass

    def mousedown(self,x,y,button):
        """ Mouse down event: override this in your sub-class
            Arguments:
                x: mouse x position
                y: mouse y position
                button: mouse button
        """
        pass

    def keyup(self,key):
        """ Key up event: override this in your sub-class
            Arguments:
                key: pygame key
        """
        pass

    def keydown(self,key):
        """ Key down event: override this in your sub-class
            Arguments:
                key: pygame key
        """
        pass
        
    def update(self,time_delta):
        """ Update routine: override this in your sub-class
            Arguments:
                time_delta: time in seconds since last update
        """
        pass
    
    def draw(self,surface):
        """ Draw routine: override this in your sub-class
            Arguments:
                surface: pygame surface to draw on
        """
        pass

