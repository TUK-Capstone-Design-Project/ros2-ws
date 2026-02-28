#!/usr/bin/python3
# -*- coding: UTF-8 -*-
from Raspbot_Lib import Raspbot
import time,math

# 初始化机器人对象
bot = Raspbot()
"""
 q w e
a--丨--d
 z x c
"""
def move_forward(speed):
    l1,l2,r1,r2=set_deflection(speed,90)
    print(f"L1:{l1:>4}| w |R1:{r1:<4}")
    print(f"L2:{l2:>4}|   |R2:{r2:<4}\n")
    bot.Ctrl_Muto(0, l1+ 0)
    bot.Ctrl_Muto(1, l2+ 0)
    bot.Ctrl_Muto(2, r1+ 0)
    bot.Ctrl_Muto(3, r2+ 0)

def move_backward(speed):
    l1,l2,r1,r2=set_deflection(speed,270)
    print(f"L1:{l1:>4}| x |R1:{r1:<4}")
    print(f"L2:{l2:>4}|   |R2:{r2:<4}\n")
    bot.Ctrl_Muto(0, l1+ 0)
    bot.Ctrl_Muto(1, l2+ 0)
    bot.Ctrl_Muto(2, r1+ 0)
    bot.Ctrl_Muto(3, r2+ 0)

def move_left(speed):
    l1,l2,r1,r2=set_deflection(speed,180)
    print(f"L1:{l1:>4}| a |R1:{r1:<4}")
    print(f"L2:{l2:>4}|   |R2:{r2:<4}\n")
    bot.Ctrl_Muto(0, l1+ 0)
    bot.Ctrl_Muto(1, l2+ 0)
    bot.Ctrl_Muto(2, r1+ 0)
    bot.Ctrl_Muto(3, r2+ 0)

def move_right(speed):
    l1,l2,r1,r2=set_deflection(speed,0)
    print(f"L1:{l1:>4}| d |R1:{r1:<4}")
    print(f"L2:{l2:>4}|   |R2:{r2:<4}\n")
    bot.Ctrl_Muto(0, l1+ 0)
    bot.Ctrl_Muto(1, l2+ 0)
    bot.Ctrl_Muto(2, r1+ 0)
    bot.Ctrl_Muto(3, r2+ 0)

def rotate_left(speed):
    l1,l2,r1,r2=set_deflection(speed,180)
    print(f"L1:{l1:>4}| q |R1:{r1:<4}")
    print(f"L2:{-l2:>4}|   |R2:{abs(r2):<4}\n")
    bot.Ctrl_Muto(0, l1+ 0)
    bot.Ctrl_Muto(1,-l2+ 0)
    bot.Ctrl_Muto(2, r1+ 0)
    bot.Ctrl_Muto(3, abs(r2)+ 0)

def rotate_right(speed):
    l1,l2,r1,r2=set_deflection(speed,0)
    print(f"L1:{l1:>4}| e |R1:{r1:<4}")
    print(f"L2:{abs(l2):>4}|   |R2:{-r2:<4}\n")
    bot.Ctrl_Muto(0, l1+ 0)
    bot.Ctrl_Muto(1, abs(l2)+ 0)
    bot.Ctrl_Muto(2, r1+ 0)
    bot.Ctrl_Muto(3,-r2+ 0)

def move_diagonal_left_front(speed):
    l1,l2,r1,r2=set_deflection(speed,135)
    print(f"L1:{l1:>4}| q |R1:{r1:<4}")
    print(f"L2:{l2:>4}|   |R2:{r2:<4}\n")
    bot.Ctrl_Muto(0, l1+ 0)
    bot.Ctrl_Muto(1, l2+ 0)
    bot.Ctrl_Muto(2, r1+ 0)
    bot.Ctrl_Muto(3, r2+ 0)

def move_diagonal_left_back(speed):
    l1,l2,r1,r2=set_deflection(speed,225)
    print(f"L1:{l1:>4}| z |R1:{r1:<4}")
    print(f"L2:{l2:>4}|   |R2:{r2:<4}\n")
    bot.Ctrl_Muto(0, l1+ 0)
    bot.Ctrl_Muto(1, l2+ 0)
    bot.Ctrl_Muto(2, r1+ 0)
    bot.Ctrl_Muto(3, r2+ 0)

def move_diagonal_right_front(speed):
    l1,l2,r1,r2=set_deflection(speed,45)
    print(f"L1:{l1:>4}| e |R1:{r1:<4}")
    print(f"L2:{l2:>4}|   |R2:{r2:<4}\n")
    bot.Ctrl_Muto(0, l1+ 0)
    bot.Ctrl_Muto(1, l2+ 0)
    bot.Ctrl_Muto(2, r1+ 0)
    bot.Ctrl_Muto(3, r2+ 0)

def move_diagonal_right_back(speed):
    l1,l2,r1,r2=set_deflection(speed,315)
    print(f"L1:={l1:>4}| c |R1:={r1:<4}")
    print(f"L2:={l2:>4}|   |R2:={r2:<4}\n")
    bot.Ctrl_Muto(0, l1+ 0)
    bot.Ctrl_Muto(1, l2+ 0)
    bot.Ctrl_Muto(2, r1+ 0)
    bot.Ctrl_Muto(3, r2+ 0)

def stop_robot():
        bot.Ctrl_Car(0, 0, 0)
        bot.Ctrl_Car(1, 0, 0)
        bot.Ctrl_Car(2, 0, 0)
        bot.Ctrl_Car(3, 0, 0)

def stop():
    for i in range(4):
        #print(i)
        time.sleep(0.25)
        bot.Ctrl_Car(0, 0, 0)
        bot.Ctrl_Car(1, 0, 0)
        bot.Ctrl_Car(2, 0, 0)
        bot.Ctrl_Car(3, 0, 0)

def set_deflection(speed, deflection):
    """
        90
    180--丨--0
        270
    """
    if(speed>255):speed=255
    if(speed<0):speed=0
    rad2deg = math.pi / 180
    vx = speed * math.cos(deflection * rad2deg)
    vy = speed * math.sin(deflection * rad2deg)
    l1 = int(vy + vx) 
    l2 = int(vy - vx)
    r1 = int(vy - vx)
    r2 = int(vy + vx)
    return l1,l2,r1,r2
