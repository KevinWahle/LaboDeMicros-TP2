# https://stackoverflow.com/questions/21019471/how-can-i-draw-a-3d-shape-using-pygame-no-other-modules

import pygame
import serial
import numpy as np
from numpy import array
from math import cos, sin, pi

COLORS= {"BLACK": (0, 0, 0),
         "RED": (255, 0, 0),
         "BLUE": (0, 0, 255),
         "YELLOW": (255, 255, 51),
         "PINK": (255, 0, 255),
         "GREEN": (0, 255, 0),}
 
GROUPS_COUNT = 5
AXIS_COUNT = 3
SCREEN_SIZE = 1080, 720

######################
#                    #
#    math section    #
#                    #
######################

X, Y, Z = 0, 1, 2
def rotation_matrix(α, β, γ):
    """
    rotation matrix of α, β, γ radians around x, y, z axes (respectively)
    """
    sα, cα = sin(α), cos(α)
    sβ, cβ = sin(β), cos(β)
    sγ, cγ = sin(γ), cos(γ)
    return (
        (cβ*cγ, -cβ*sγ, sβ),
        (cα*sγ + sα*sβ*cγ, cα*cγ - sγ*sα*sβ, -cβ*sα),
        (sγ*sα - cα*sβ*cγ, cα*sγ*sβ + sα*cγ, cα*cβ)
    )

class Physical:
    def __init__(self, vertices = ((1, 1, 1), (1, 1, -1), (1, -1, 1), (1, -1, -1), (-1, 1, 1), (-1, 1, -1), (-1, -1, 1), (-1, -1, -1)), 
                        edges = ({0, 1}, {0, 2}, {2, 3}, {1, 3}, {4, 5}, {4, 6}, {6, 7}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}), 
                        offset = (0,0,0),
                        color = COLORS["RED"]):
        """
        a 3D object that can rotate around the three axes
        :param vertices: a tuple of points (each has 3 coordinates)
        :param edges: a tuple of pairs (each pair is a set containing 2 vertices' indexes)
        """
        self.__vertices = array(vertices)
        self.__edges = tuple(edges)
        self.__rotation = [0, 0, 0]  # radians around each axis
        self.__offset = offset
        self.color = color

    def rotate(self, axis, θ):
        self.__rotation[axis] = θ

    def traslate(self):
        for i in range(len(self.__vertices)):
            self.__vertices[i]= (self.__vertices[i][0], self.__vertices[i][1], self.__vertices[i][2])
            

    @property
    def lines(self):
        location = self.__vertices.dot(rotation_matrix(*self.__rotation))+self.__offset  # an index->location mapping
        
        return ((location[v1], location[v2]) for v1, v2 in self.__edges)

######################
#                    #
#    gui section     #
#                    #
######################

def handle_events():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            exit()

def fit(vec, size_screen):
    """
    ignore the z-element (creating a very cheap projection), and scale x, y to the coordinates of the screen
    """
    # notice that len(self.__size) is 2, hence zip(vec, self.__size) ignores the vector's last coordinate
    return [round(70 * coordinate + frame / 2) for coordinate, frame in zip(vec, size_screen)]


def draw_cube(cube, __screen, size_screen, thickness=4):
    for start, end in cube.lines:
        pygame.draw.line(__screen, cube.color, fit(start, size_screen), fit(end, size_screen), thickness)

def updateScreen(screen):
    screen.fill(COLORS["BLACK"])
    for cube in cubes:
        draw_cube(cube, screen, SCREEN_SIZE)             # Una vez que se llama Paint se deja de correr el main
    pygame.display.flip()

######################
#                    #
#     main start     #
#                    #
######################

cubes = [   
            Physical(offset= (0,0,0), color=COLORS["RED"]), 
            Physical(offset= (3,3,0), color=COLORS["BLUE"]), 
            Physical(offset= (-3,3,0), color=COLORS["YELLOW"]),
            Physical(offset= (3,-3,3), color=COLORS["PINK"]),
            Physical(offset= (-3,-3,3), color=COLORS["GREEN"]),
        ]

def main():
    ser = serial.Serial('COM8', 115200)
    boards = np.empty((GROUPS_COUNT, AXIS_COUNT))

    pygame.init()
    screen = pygame.display.set_mode(SCREEN_SIZE)
    clock = pygame.time.Clock()
    clock.tick()

    while 1:
        line = str(ser.readline(), 'UTF-8')
        index=int(line[4])

        #if index<GROUPS_COUNT:
        if index==5:
            boards[:] = line[7:].removesuffix('\r\n').split('\t')

        for grupo in range(GROUPS_COUNT):
            cubes[grupo].rotate(axis=X, θ=float(boards[grupo][0])*pi/180)    # ROLL
            cubes[grupo].rotate(axis=Y, θ=float(boards[grupo][1])*pi/180)    # CABECEO
            cubes[grupo].rotate(axis=Z, θ=float(boards[grupo][2])*pi/180)    # ORIENTACION

        print(boards)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                exit()

        updateScreen(screen)

if __name__ == '__main__':
    main()
