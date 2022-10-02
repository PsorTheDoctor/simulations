import cv2

width = 1024
height = 1024
fourcc = cv2.VideoWriter_fourcc(*'XVID')
writer = cv2.VideoWriter('videos/biped.mp4', fourcc, 48, (width, height))

cap1 = cv2.VideoCapture('videos/walk.mp4')
cap2 = cv2.VideoCapture('videos/squat.mp4')
cap3 = cv2.VideoCapture('videos/torso_twist.mp4')
cap4 = cv2.VideoCapture('videos/jump_with_torso_twist.mp4')

while cap1.isOpened():
    _, frame1 = cap1.read()
    _, frame2 = cap2.read()
    _, frame3 = cap3.read()
    _, frame4 = cap4.read()

    upper = cv2.hconcat([frame1, frame2])
    lower = cv2.hconcat([frame3, frame4])
    frame = cv2.vconcat([upper, lower])
    writer.write(frame)

    cv2.imshow('', frame)
    if cv2.waitKey(10) == 27:
        break

writer.release()
cap1.release()
cap2.release()
cap3.release()
cap4.release()

print('Done')
