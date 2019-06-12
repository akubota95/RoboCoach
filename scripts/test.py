#!/usr/bin/env python3
"""
import speech_recognition as sr

# get audio from the microphone
r = sr.Recognizer()
with sr.Microphone() as source:
    print("Speak:")
    audio = r.listen(source)

try:
    print("You said " + r.recognize_google(audio))
except sr.UnknownValueError:
    print("Could not understand audio")
except sr.RequestError as e:
    print("Could not request results; {0}".format(e))
"""
"""
import pygame
import sys
import pygame.sprite as sprite

theClock = pygame.time.Clock()

bg = pygame.image.load('colored_land.png')
bg = pygame.transform.scale(bg,(800,600))
bgX = 0
bgX2 = bg.get_width()
w,h = bg.get_size()
DISPLAYSURF = pygame.display.set_mode((w,h))

running = True

while running:
    #screen.blit(background,background_rect)
    #pygame.display.update()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    theClock.tick(30)
    bgX -= 1.4
    bgX2 -= 1.4
    if bgX < bg.get_width() *(-1):
        bgX = bg.get_width()
    if bgX2 < bg.get_width()*(-1):
        bgX2 = bg.get_width()
    DISPLAYSURF.blit(bg,(bgX,0))
    DISPLAYSURF.blit(bg,(bgX2,0))
    pygame.display.update()
"""

import socket
import time

HOST = '100.80.229.211'  # Standard loopback interface address (localhost)
PORT = 65435       # Port to listen on (non-privileged ports are > 1023)
"""
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()3
    with conn:
        print('Connected by', addr)
        while True:
            data = conn.recv(1024)
            if not data:
                continue
            else:
                print(data)
                if data == b'1':
                    print("kitchen!")
"""
with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as s:
    s.connect((HOST,PORT))
    start = time.time()
    c = 5
    #while c > 0:
    '''
    while True:
        s.sendall(b'1')
        end = time.time()
        data = s.recv(1024)
        print(end-start)
        if (end - start) >= 5:
            while c > 0 : 
            	s.sendall(b'4')
            	print(data)
            	c = c - 1
            	#start = time.time()
            break
    #s.sendall(b'-1')
    '''
    while True:
        s.sendall(b'1')
        end = time.time()
        data = s.recv(1024)
        print(end-start)
        if (end - start) >= 60:
            s.sendall(b'4')
            print(data)
            break

	


