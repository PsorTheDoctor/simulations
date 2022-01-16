import pygame
import math
import random

width = 224
height = 224
r = 60
angle1 = 0
angle2 = 0

pygame.init()
screen = pygame.display.set_mode((width, height))
clock = pygame.time.Clock()


def createDataset(samples):
    with open('planarManipulator/report.txt', 'w') as f:
        f.truncate(0)  # clears a file!

    for i in range(samples):
        clock.tick(24)
        screen.fill((0, 0, 0))

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        angle1 = 2 * math.pi * random.random() * random.choice((-1, 1))
        angle2 = 2 * math.pi * random.random() * random.choice((-1, 1))

        startX = width / 2
        startY = height / 2
        midX = int(startX + r * math.sin(angle1))
        midY = int(startY + r * math.cos(angle1))
        endX = int(midX + r * math.sin(angle2))
        endY = int(midY + r * math.cos(angle2))

        pygame.draw.line(screen, (255, 255, 255), (startX, startY), (midX, midY), width=10)
        pygame.draw.line(screen, (255, 255, 255), (midX, midY), (endX, endY), width=10)
        pygame.display.flip()

        pygame.image.save(screen, 'planarManipulator/images/{}.jpg'.format(i))
        decimals = 4
        with open('planarManipulator/report.txt', 'a') as f:
            line = 'id: {} joints: {} {}\n'.format(i, round(angle1, decimals),
                                                   round(angle2, decimals))
            f.write(line)


createDataset(10000)
