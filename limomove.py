#!/usr/bin/env python3
# coding=UTF-8
from pylimo import limo
import time
limo=limo.LIMO()
limo.EnableCommand()         # Steuerung


# while num>0:
#         limo.SetMotionCommand(linear_vel=-1,angular_vel=0)
#         linear_vel = limo.GetLinearVelocity()
#         print(linear_vel)
#         time.sleep(0.3)
#         num-=1


# try:
#     # limo.SetMotionCommand(linear_vel=1,angular_vel=0) # vorwärts
#     # time.sleep(0.5)
#     limo.SetMotionCommand(linear_vel=0.1,angular_vel=500) # vorwärts
#     # time.sleep(10)

# except KeyboardInterrupt:
#     limo.SetMotionCommand(linear_vel=0)