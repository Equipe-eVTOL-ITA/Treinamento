import pygame
from math import sin, cos
from constants import SCREEN_WIDTH, SCREEN_HEIGHT, PIX2M, M2PIX


class Simulation(object):
    """
    Represents the simulation.
    """
    def __init__(self, roomba):
        """
        Creates the simulation.

        :param roomba: the roomba robot used in this simulation.
        :type roomba: Roomba
        """
        self.point_list = []
        self.roomba = roomba

    def check_collision(self):
        """
        Checks collision between the robot and the walls.

        :return: the bumper state (if a collision has been detected).
        :rtype: bool
        """
        # Converting screen limits from pixels to meters
        width = SCREEN_WIDTH * PIX2M
        height = SCREEN_HEIGHT * PIX2M
        bumper_state = False
        # Computing the limits of the roomba's bounding box
        left = self.roomba.pose.position.x - self.roomba.radius
        right = self.roomba.pose.position.x + self.roomba.radius
        top = self.roomba.pose.position.y - self.roomba.radius
        bottom = self.roomba.pose.position.y + self.roomba.radius
        # Testing if the bounding box has hit a wall
        if left <= 0.0:
            self.roomba.pose.position.x = self.roomba.radius
            bumper_state = True
        if right >= width:
            self.roomba.pose.position.x = width - self.roomba.radius
            bumper_state = True
        if top <= 0.0:
            self.roomba.pose.position.y = self.roomba.radius
            bumper_state = True
        if bottom >= height:
            self.roomba.pose.position.y = height - self.roomba.radius
            bumper_state = True
        return bumper_state

    def update(self):
        """
        Updates the simulation.
        """
        # Adding roomba's current position to the movement history
        self.point_list.append((round(M2PIX * self.roomba.pose.position.x), round(M2PIX * self.roomba.pose.position.y)))
        if len(self.point_list) > 2000:
            self.point_list.pop(0)
        # Verifying collision
        bumper_state = self.check_collision()
        self.roomba.set_bumper_state(bumper_state)
        # Updating the robot's behavior and movement
        self.roomba.update()

    def draw(self, window):
        """
        Draws the roomba and its movement history.

        :param window: pygame's window where the drawing will occur.
        """
        # If we have less than 2 points, we are unable to plot the movement history
        if len(self.point_list) >= 2:
            pygame.draw.lines(window, (255, 0, 0), False, self.point_list, 4)
        # Computing roomba's relevant points and radius in pixels
        sx = round(M2PIX * self.roomba.pose.position.x)
        sy = round(M2PIX * self.roomba.pose.position.y)
        ex = round(M2PIX * (self.roomba.pose.position.x + self.roomba.radius * cos(self.roomba.pose.rotation)))
        ey = round(M2PIX * (self.roomba.pose.position.y + self.roomba.radius * sin(self.roomba.pose.rotation)))
        r = round(M2PIX * self.roomba.radius)
        # Drawing roomba's inner circle
        pygame.draw.circle(window, (200, 200, 200), (sx, sy), r, 0)
        # Drawing roomba's outer circle
        pygame.draw.circle(window, (100, 100, 100), (sx, sy), r, 4)
        # Drawing roomba's orientation
        pygame.draw.line(window, (50, 50, 50), (sx, sy), (ex, ey), 3)


def draw(simulation, window):
    """
    Redraws the pygame's window.

    :param simulation: the simulation object.
    :param window: pygame's window where the drawing will occur.
    """
    window.fill((224, 255, 255))
    simulation.draw(window)
    pygame.display.update()

