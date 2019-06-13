##########################
#RoboCoach interface, written by Yi Peng, with the 
#input of Alyssa Kubota, Maryam Pourebadi



import pygame, sys
from pygame.locals import *
import cv2
import numpy as np
import pygame_textinput
import speech_recognition as sr
import socket
import pyttsx3


#setup sockets to communicate with ROS
HOST = '100.80.229.211'
#PORT_WALK = 65430
PORT_EXER = 65435

#s_walk = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
#s_exer = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
#s_walk.connect((HOST,PORT_WALK))
#s_exer.connect((HOST,PORT_EXER))
def sendMsg_walk(msg):
	s_walk.sendall(msg)
	return
def sendMsg_exer(msg):
	s_exer.sendall(msg)
	return


FPS = 30
WINDOWWIDTH = 800
WINDOWHEIGHT = 600


#RGB
WHITE = (255,255,255)
BLACK = (0,0,0)
RED = (200,0,0)
GREEN = (0,200,0)
BRIGHTRED = (255,0,0)
BRIGHTGREEN = (0,255,0)
BLUE = (0,128,255)
BRIGHTBLUE = (0,255,255)
PURPLE = (127,0,255)
BRIGHTPURPLE = (255,0,255)

def terminate():
	pygame.quit()
	sys.exit()
#used to play animation
class spritesheet(object):
	def __init__(self, filename):
		try:
			self.sheet = pygame.image.load(filename).convert()
		except pygame.error:
			print ('Unable to load spritesheet image:', filename)
			raise SystemExit
	# Load a specific image from a specific rectangle
	def image_at(self, rectangle, colorkey = None):
		"Loads image from x,y,x+offset,y+offset"
		rect = pygame.Rect(rectangle)
		image = pygame.Surface(rect.size).convert()
		image.blit(self.sheet, (0, 0), rect)
		if colorkey is not None:
			if colorkey is -1:
				colorkey = image.get_at((0,0))
			image.set_colorkey(colorkey, pygame.RLEACCEL)
		return image
	# Load a whole bunch of images and return them as a list
	def images_at(self, rects, colorkey = None):
		"Loads multiple images, supply a list of coordinates"
		return [self.image_at(rect, colorkey) for rect in rects]
	# Load a whole strip of images
	def load_strip(self, rect, image_count, colorkey = None):
		"Loads a strip of images and returns them as a list"
		tups = [(rect[0]+rect[2]*x, rect[1], rect[2], rect[3])
				for x in range(image_count)]
		return self.images_at(tups, colorkey)

class SpriteStripAnim(object):
	
	def __init__(self, filename, rect, count, colorkey=None, loop=False, frames=1):
		self.filename = filename
		ss = spritesheet(filename)
		self.images = ss.load_strip(rect, count, colorkey)
		self.i = 0
		self.loop = loop
		self.frames = frames
		self.f = frames
	def iter(self):
		self.i = 0
		self.f = self.frames
		return self
	def next(self):
		if self.i >= len(self.images):
			if not self.loop:
				raise StopIteration
			else:
				self.i = 0
		image = self.images[self.i]
		self.f -= 1
		if self.f == 0:
			self.i += 1
			self.f = self.frames
		return image
	def __add__(self, ss):
		self.images.extend(ss.images)
		return self
#walking companion
def walkingCompanion():
	state = 0
	frames = 10
	strips = SpriteStripAnim('p1_walk.png',(0,0,66,90),3,4,True,frames)

	strips.iter()

	image = strips.next()
	smallText = pygame.font.Font('freesansbold.ttf',20)
	DISPLAYSURF.fill(WHITE)
	instructionText = "Where would you like to go?"
	instSurf = smallText.render(instructionText,1,BLACK)
	instRect = instSurf.get_rect()
	instRect.top = 50
	instRect.centerx = 400
	DISPLAYSURF.blit(instSurf,instRect)
	destID = None
	while True:
		if state == 0:
			#state 0 destination selection
			for event in pygame.event.get():
				if event.type == QUIT:
					terminate()
			smallText = pygame.font.Font('freesansbold.ttf',20)
			DISPLAYSURF.fill(WHITE)
			instructionText = "Where would you like to go?"
			instSurf = smallText.render(instructionText,1,BLACK)
			instRect = instSurf.get_rect()
			instRect.top = 50
			instRect.centerx = 400
			DISPLAYSURF.blit(instSurf,instRect)
			"""
			with sr.Microphone() as source:
				print("Speak:")
				audio = SPEECHREC.listen(source)
			try:
				print("You said "+SPEECHREC.recognize_google(audio))
			except sr.UnknownValueError:
				print("Could not understand audio")
			"""
			mouse = pygame.mouse.get_pos()
			click = pygame.mouse.get_pressed()
			#print(mouse)
			if 200+150 > mouse[0] > 200 and 300+50 > mouse[1] > 300:
				pygame.draw.rect(DISPLAYSURF,BRIGHTGREEN,(200,300,150,50))
				#print("inside kitchen")
				if click[0] == 1:
					destID = 1
					#sendMsg_walk(b'1')
					state = 1
					DISPLAYSURF.fill(WHITE)
					bg = pygame.image.load('colored_land.png')
					bg = pygame.transform.scale(bg,(800,600))
					bgX = 0
					bgX2 = bg.get_width()
					w,h = bg.get_size()
					continue


			else:
				#print("in here")
				pygame.draw.rect(DISPLAYSURF,GREEN,(200,300,150,50))
			
			if 500+150 > mouse[0] > 500 and 300 + 50 > mouse[1] > 300:
				pygame.draw.rect(DISPLAYSURF,BRIGHTRED,(500,300,150,50))
				if click[0] == 1:
					destID = 2
					#sendMsg_walk(b'2')
					state = 1
					DISPLAYSURF.fill(WHITE)
					bg = pygame.image.load('colored_land.png')
					bg = pygame.transform.scale(bg,(800,600))
					bgX = 0
					bgX2 = bg.get_width()
					w,h = bg.get_size()
					continue
			else:
				pygame.draw.rect(DISPLAYSURF,RED,(500,300,150,50))
			
			
			if 200+150 > mouse[0] > 200 and 400 + 50 > mouse[1] > 400:
				pygame.draw.rect(DISPLAYSURF,BRIGHTBLUE,(200,400,150,50))
				if click[0] == 1:
					destID = 3
					#sendMsg_walk(b'3')
					state = 1
					DISPLAYSURF.fill(WHITE)
					bg = pygame.image.load('colored_land.png')
					bg = pygame.transform.scale(bg,(800,600))
					bgX = 0
					bgX2 = bg.get_width()
					w,h = bg.get_size()
					continue
			else:
				pygame.draw.rect(DISPLAYSURF,BLUE,(200,400,150,50))
			if 500+150 > mouse[0] > 500 and 400 + 50 > mouse[1] > 400:
				pygame.draw.rect(DISPLAYSURF,BRIGHTPURPLE,(500,400,150,50))
				if click[0] == 1:
					destID = 4
					#sendMsg_walk(b'4')
					state = 1
					DISPLAYSURF.fill(WHITE)
					bg = pygame.image.load('colored_land.png')
					bg = pygame.transform.scale(bg,(800,600))
					bgX = 0
					bgX2 = bg.get_width()
					w,h = bg.get_size()
					continue
			else:
				pygame.draw.rect(DISPLAYSURF,PURPLE,(500,400,150,50))
			
				
			textSurfg = smallText.render('Kitchen',1,BLACK)
			textRectg = textSurfg.get_rect()
			textRectg.center = ((200+(150/2)),(300+(50/2)))
			DISPLAYSURF.blit(textSurfg,textRectg)
			
			textSurfr = smallText.render('Restroom',1,BLACK)
			textRectr = textSurfr.get_rect()
			textRectr.center = ((500 + (150/2)),(300+(50/2)))
			DISPLAYSURF.blit(textSurfr,textRectr)
			
			
			textSurfb = smallText.render('Lounge',1,BLACK)
			textRectb = textSurfb.get_rect()
			textRectb.center = ((200 + (150/2)),(400+(50/2)))
			DISPLAYSURF.blit(textSurfb,textRectb)
			
			textSurfp = smallText.render('Desk',1,BLACK)
			textRectp = textSurfp.get_rect()
			textRectp.center = ((500 + (150/2)),(400+(50/2)))
			DISPLAYSURF.blit(textSurfp,textRectp)
			
			pygame.display.update()
			FPSCLOCK.tick()
		elif state == 1:
			#state 1 play animation, send signal to ros, wait for feedback from ros
			while destID != None:
				bgX -= 1.4
				bgX2 -= 1.4
				if bgX < bg.get_width() * -1:
					bgX = bg.get_width()
				if bgX2 < bg.get_width() * -1:
					bgX2 = bg.get_width()
				DISPLAYSURF.blit(bg,(bgX,0))
				DISPLAYSURF.blit(bg,(bgX2,0))
				instructionText = "Good idea! Let's head there now!"
				instSurf = smallText.render(instructionText,1,BLACK)
				instRect = instSurf.get_rect()
				instRect.top = 200
				instRect.centerx = 400
				DISPLAYSURF.blit(instSurf,instRect)
				for event in pygame.event.get():
					if event.type == QUIT:
						terminate()
					strips.iter()
				DISPLAYSURF.blit(image,(370,350))
				pygame.display.update()
				image = strips.next()
				#data = s_walk.recv(1024)
				"""
				print (data)
				if data == b'1':
					state = 2
					break
				"""
				FPSCLOCK.tick()
		elif state == 2:
			#destination reached, ask for another destination
			DISPLAYSURF.fill(WHITE)
			newText = "Is there anywhere else you want to go?"
			newTextSurf = smallText.render(newText,1,BLACK)
			newTextRect = newTextSurf.get_rect()
			newTextRect.top = 300
			newTextRect.centerx = 400
			DISPLAYSURF.blit(newTextSurf,newTextRect)

			textinput = pygame_textinput.TextInput()
			while True:
				events = pygame.event.get()
				for event in events:
					if event.type == pygame.QUIT:
						if begin == 0:
							begin = 1
							continue
						terminate()
				ret = textinput.update(events)
				if ret:
					if textinput.get_text() == 'yes':
						state = 0
						#textinput = ""
						break
					elif textinput.get_text() == 'no':
						#textinput = ""
						print("here!")
						state = 3
						break
				DISPLAYSURF.blit(textinput.get_surface(),(400,350))
				pygame.display.update()
		elif state == 3:
			#print("now here!")
			DISPLAYSURF.fill(WHITE)
			newText = "Ok! Let's head back to work."
			newTextSurf = smallText.render(newText,1,BLACK)
			newTextRect = newTextSurf.get_rect()
			newTextRect.top = 300
			newTextRect.centerx = 400
			DISPLAYSURF.blit(newTextSurf,newTextRect)

			pygame.display.update()
			print("updated")
			FPSCLOCK.tick()
			
			sendMsg_walk(b'4')
			while True:
				data = s_walk.recv(1024)
				if data == b'1':
					return
			


#play video
def playvideo(movie):
	cap = cv2.VideoCapture(movie)
	if(cap.isOpened() == False):
		print("Error opening video file")
	while(cap.isOpened()):
		ret,frame = cap.read()
		if ret == True:
			cv2.imshow('Exercise',frame)
			if cv2.waitKey(25) & 0xFF == ord('q'):
				break
		else:
			break
	cap.release()
	cv2.destroyWindow('Exercise') 
#interactive exercise
def interactiveExer():
	DISPLAYSURF.fill(WHITE)
	instructionText = "Choose your exercise!"
	smallText = pygame.font.Font("freesansbold.ttf",18)
	instSurf = smallText.render(instructionText,1,BLACK)
	instRect = instSurf.get_rect()
	instRect.top = 250
	instRect.centerx = 400
	DISPLAYSURF.blit(instSurf,instRect)
	pygame.display.update()

	SPEECHENGINE.say(instructionText)
	SPEECHENGINE.runAndWait()
	begin = 0
	state2 = False
	state4 = False

	vidID = -1
	state = 0
	sent = False
	sent1 = False
	sent2 = False

	while True:
		print('vidID=',vidID)
		print('state=',state)
		#print(begin)
		for event in pygame.event.get():
			if event.type == QUIT:
				#print(begin)
				if begin == 0:
					continue
				terminate()
		if state == 0:#choose exercise screen
			DISPLAYSURF.fill(WHITE)
			instructionText = "Choose your exercise!"
			smallText = pygame.font.Font("freesansbold.ttf",18)
			instSurf = smallText.render(instructionText,1,BLACK)
			instRect = instSurf.get_rect()
			instRect.top = 250
			instRect.centerx = 400
			DISPLAYSURF.blit(instSurf,instRect)
			#pygame.display.update()
			
			mouse = pygame.mouse.get_pos()
			click = pygame.mouse.get_pressed()
			#print(mouse)
			if 200+150 > mouse[0] > 200 and 300+50 > mouse[1] > 300:
				pygame.draw.rect(DISPLAYSURF,BRIGHTGREEN,(200,300,150,50))
				#print("inside kitchen")
				if click[0] == 1:
					vidID = 1
					state = 1
					
			else:
				#print("in here")
				pygame.draw.rect(DISPLAYSURF,GREEN,(200,300,150,50))
			
			if 500+150 > mouse[0] > 500 and 300 + 50 > mouse[1] > 300:
				pygame.draw.rect(DISPLAYSURF,BRIGHTRED,(500,300,150,50))
				if click[0] == 1:
					vidID = 2
					state = 1
					
			else:
				pygame.draw.rect(DISPLAYSURF,RED,(500,300,150,50))
			
			
			if 350+150 > mouse[0] > 350 and 400 + 50 > mouse[1] > 400:
				pygame.draw.rect(DISPLAYSURF,BRIGHTBLUE,(350,400,150,50))
				if click[0] == 1:
					vidID = 3
					state = 1
					
			else:
				pygame.draw.rect(DISPLAYSURF,BLUE,(350,400,150,50))

			textSurfg = smallText.render('Exercise 1',1,BLACK)
			textRectg = textSurfg.get_rect()
			textRectg.center = ((200+(150/2)),(300+(50/2)))
			DISPLAYSURF.blit(textSurfg,textRectg)
			
			textSurfr = smallText.render('Exercise 2',1,BLACK)
			textRectr = textSurfr.get_rect()
			textRectr.center = ((500 + (150/2)),(300+(50/2)))
			DISPLAYSURF.blit(textSurfr,textRectr)
			
			
			textSurfb = smallText.render('Exercise 3',1,BLACK)
			textRectb = textSurfb.get_rect()
			textRectb.center = ((350 + (150/2)),(400+(50/2)))
			DISPLAYSURF.blit(textSurfb,textRectb)

			pygame.display.update()
			FPSCLOCK.tick()



		
		if state == 1:#play videos
			if vidID == -1:
				state = 0
				continue
			pygame.display.update()
			if vidID == 1:
				playvideo('exer1.mp4')
			elif vidID == 2:
				playvideo('exer2.mp4')
			elif vidID == 3:
				playvideo('exer3.mp4')
			
			state = 2
		
		elif state == 2:
			#ask if user needs to see the video again
			DISPLAYSURF.fill(WHITE)
			newText = "Do you need to see the video again?"
			newTextSurf = smallText.render(newText,1,BLACK)
			newTextRect = newTextSurf.get_rect()
			newTextRect.top = 300
			newTextRect.centerx = 400
			DISPLAYSURF.blit(newTextSurf,newTextRect)
			
			textinput = pygame_textinput.TextInput()
			while True:
				events = pygame.event.get()
				for event in events:
					if event.type == pygame.QUIT:
						if begin == 0:
							begin = 1
							continue
						terminate()
				ret = textinput.update(events)
				if ret:
					if textinput.get_text() == 'yes':
						state = 1
						#textinput = ""
						break
					elif textinput.get_text() == 'no':
						#textinput = ""
						state = 3
						break
				DISPLAYSURF.blit(textinput.get_surface(),(400,350))
				pygame.display.update()
				if state2 == False:
					SPEECHENGINE.say(newText)
					SPEECHENGINE.runAndWait()
					begin = 0
					#print(begin)
					state2 = True
				FPSCLOCK.tick()
		elif state == 3:
			#prepare player for exercise
			DISPLAYSURF.fill(WHITE)
			newText = "Now it's your turn!"
			newTextSurf = smallText.render(newText,1,BLACK)
			newTextRect = newTextSurf.get_rect()
			newTextRect.top = 300
			newTextRect.centerx = 400
			DISPLAYSURF.blit(newTextSurf,newTextRect)
			pygame.display.update()
			SPEECHENGINE.say(newText)
			SPEECHENGINE.runAndWait()
			begin = 0
			nextText2 = "Get ready! Your exercise will start in 3 seconds!"
			nextText2Surf = smallText.render(nextText2,1,BLACK)
			nextText2Rect = nextText2Surf.get_rect()
			nextText2Rect.top = 350
			nextText2Rect.centerx = 400
			DISPLAYSURF.blit(nextText2Surf,nextText2Rect)
			state = 4
			
			pygame.display.update()
			SPEECHENGINE.say(nextText2)
			SPEECHENGINE.runAndWait()
			begin = 0
			FPSCLOCK.tick()
			start_ticks = pygame.time.get_ticks()
			
			
		elif state == 4:#countdown
			seconds = 6-int((pygame.time.get_ticks()-start_ticks)/1000)
			if seconds<=0:
				DISPLAYSURF.fill(WHITE)
				newText = "Start!"
				state = 5
				state4 = True
				start_ticks = pygame.time.get_ticks()
			elif 3>=seconds>0:
				DISPLAYSURF.fill(WHITE)
				newText = str(seconds)
			else:
				newText = ""
	
			newTextSurf = smallText.render(newText,1,BLACK)
			newTextRect = newTextSurf.get_rect()
			newTextRect.top = 300
			newTextRect.centerx = 400
			DISPLAYSURF.blit(newTextSurf,newTextRect)
			pygame.display.update()
			if seconds <= 0:
				SPEECHENGINE.say(newText)
				SPEECHENGINE.runAndWait()
			begin = 0
			
		elif state == 5:
			#send signal to ros to run gesture analysis
			seconds = int((pygame.time.get_ticks()-start_ticks)/1000)
			print(seconds)
			if seconds < 60:
				if sent == False:
					if vidID == 1:
						sendMsg_exer(b'1')
					elif vidID == 2:
						sendMsg_exer(b'2')
					elif vidID == 3:
						sendMsg_exer(b'3')
					sent = True
				
			
			#skeleton analysis
			else:
				if sent1 == False:
					sendMsg_exer(b'4')
					sent1 = True
				data = s_exer.recv(1024)
				#data = str(data, 'utf-8')
				print("data=",data)
				if data != b'0':
					#sendMsg_exer(b'-1')
					state = 6
					start_ticks = pygame.time.get_ticks()
					continue
				print(data)
		elif state == 6:
			#exercise finished, display feedback
			if sent2 == False:
				sendMsg_exer(b'-1')
				sent2 = True
			DISPLAYSURF.fill(WHITE)
			text = data
			textSurf = smallText.render(text,1,BLACK)
			textRect = textSurf.get_rect()
			textRect.top = 300
			textRect.centerx = 400
			DISPLAYSURF.blit(textSurf,textRect)
			pygame.display.update()
			seconds = int((pygame.time.get_ticks()-start_ticks)/1000)
			if seconds >= 5:
				vidID = -1
				state = 0
				continue
			
				
			
			#skeletonanalysis()
			
			

	

def showDefaultpage():
	
	
	while True:
		#default page for user to choose type of activity
		instructionText = 'Welcome to RoboCoach!'
	
		DISPLAYSURF.fill(WHITE)
		
		instSurf = BASICFONT.render(instructionText,1,BLACK)
		instRect = instSurf.get_rect()
		instRect.top = 50
		instRect.centerx = 400
		DISPLAYSURF.blit(instSurf,instRect)
		
		for event in pygame.event.get():
			if event.type == QUIT:
				terminate()
		#pygame.draw.rect(DISPLAYSURF,RED,(150,450,100,50))
		#pygame.draw.rect(DISPLAYSURF,DARKBLUE,(550,450,100,50))
		
		mouse = pygame.mouse.get_pos()
		click = pygame.mouse.get_pressed()
		
		if 50+200 > mouse[0] > 100 and 450+50 > mouse[1] > 450:
			pygame.draw.rect(DISPLAYSURF,BRIGHTGREEN,(50,450,200,50))
			if click[0] == 1:
				walkingCompanion()
				continue
		else:
			pygame.draw.rect(DISPLAYSURF,GREEN,(50,450,200,50))
		if 550+200 > mouse[0] > 550 and 450 + 50 > mouse[1] > 450:
			pygame.draw.rect(DISPLAYSURF,BRIGHTRED,(550,450,200,50))
			if click[0] == 1:
				interactiveExer()
				continue
		else:
			pygame.draw.rect(DISPLAYSURF,RED,(550,450,200,50))
		
		smallText = pygame.font.Font("freesansbold.ttf",18)
		textSurfg = smallText.render('walking companion',1,BLACK)
		textRectg = textSurfg.get_rect()
		textRectg.center = ((50+(200/2)),(450+(50/2)))
		DISPLAYSURF.blit(textSurfg,textRectg)
		
		textSurfr = smallText.render('interactive Exercise',1,BLACK)
		textRectr = textSurfr.get_rect()
		textRectr.center = ((550 + (200/2)),(450+(50/2)))
		DISPLAYSURF.blit(textSurfr,textRectr)
		
		pygame.display.update()
		FPSCLOCK.tick()
	
	

def main():
	#main loop
	global FPSCLOCK, DISPLAYSURF, SETTIME,BASICFONT,SPEECHREC,SPEECHENGINE
	pygame.init()
	FPSCLOCK = pygame.time.Clock()
	DISPLAYSURF = pygame.display.set_mode((WINDOWWIDTH,WINDOWHEIGHT))
	SETTIME = False
	BASICFONT = pygame.font.Font('freesansbold.ttf',50)
	
	SPEECHREC = sr.Recognizer()
	SPEECHENGINE = pyttsx3.init()
	
	pygame.display.set_caption('RoboCoach')
	while True:
		
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				terminate()
		showDefaultpage()

if __name__ == '__main__':
	main()
	
