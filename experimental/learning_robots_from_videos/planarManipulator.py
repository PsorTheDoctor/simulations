import pygame
import math
import random

width = 140
height = 140
r = 35
angle1 = 0
angle2 = 0

pygame.init()
screen = pygame.display.set_mode((width, height))
clock = pygame.time.Clock()


def createDataset(samples):
    with open('planarManipulator2/report.txt', 'w') as f:
        f.truncate(0)  # clears a file!

    for i in range(samples):
        clock.tick(24)
        screen.fill((0, 0, 0))

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        # Angle values in range (-6.28, 6.28) for actual control
        # Joint values in range (-1, 1) for dataset labeling
        joint1 = random.random() * random.choice((-1, 1))
        angle1 = 2 * math.pi * joint1
        joint2 = random.random() * random.choice((-1, 1))
        angle2 = 2 * math.pi * joint2

        startX = width / 2
        startY = height / 2
        midX = int(startX + r * math.sin(angle1))
        midY = int(startY + r * math.cos(angle1))
        endX = int(midX + r * math.sin(angle2))
        endY = int(midY + r * math.cos(angle2))

        pygame.draw.line(screen, (255, 255, 255), (startX, startY), (midX, midY), width=5)
        pygame.draw.line(screen, (255, 255, 255), (midX, midY), (endX, endY), width=5)
        pygame.display.flip()

        pygame.image.save(screen, 'planarManipulator2/images/{}.jpg'.format(i))
        decimals = 4
        with open('planarManipulator2/report.txt', 'a') as f:
            line = 'id: {} joints: {} {}\n'.format(i, round(joint1, decimals),
                                                   round(joint2, decimals))
            f.write(line)


createDataset(10000)
