import cv2
import numpy as np
import time
import VL53L1X
import math
import motoron
import board
import neopixel_spi as neopixel
import RPi.GPIO as GPIO

#============================================================================
# 車輪初速度
vL_pre=0
vR_pre=0

vL=vL_pre
vR=vR_pre

mc = motoron.MotoronI2C()
mc.reinitialize()  # Bytes: 0x96 0x74
mc.disable_crc()   # Bytes: 0x8B 0x04 0x7B 0x43
mc.clear_reset_flag()  # Bytes: 0xA9 0x00 0x04

# Configure motor 1
mc.set_max_acceleration(1, 140)
mc.set_max_deceleration(1, 100)

# Configure motor 2
mc.set_max_acceleration(2, 140)
mc.set_max_deceleration(2, 100)


#=========================================================
# エンコーダのピン設定
# motor1
encoder1_pin_A = 26
encoder1_pin_B = 16

# motor2
encoder2_pin_A = 6
encoder2_pin_B = 5

# 初期値と移動量
encoder_value1 = 0
movement_distance1 = 0
dig1 = 0

encoder_value2 = 0
movement_distance2 = 0
dig2 = 0

# GPIOピンの初期化
GPIO.setmode(GPIO.BCM)
GPIO.setup(encoder1_pin_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(encoder1_pin_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(encoder2_pin_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(encoder2_pin_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

#Migi
def handle_encoder1(channel):
	global encoder_value1, movement_distance1, dig1
	
	if GPIO.input(encoder1_pin_A) == GPIO.input(encoder1_pin_B):
	    encoder_value1 += 1
	    dig1 = (encoder_value1*3.14159265358979*2)/(50*800)
	    movement_distance1 += dig1*35 # エンコーダの値が増加した場合、移動量も増加
	
	else:
	    encoder_value1 -= 1
	    dig1 = (encoder_value1*3.14159265358979*2)/(50*800)
	    movement_distance1 -= dig1*35  # エンコーダの値が減少した場合、移動量も減少
	#print(f"Encoder Value: {encoder_value1}, Movement Distance: {movement_distance1}")
	
GPIO.add_event_detect(encoder1_pin_A, GPIO.BOTH, callback=handle_encoder1)

#Hidari
def handle_encoder2(channel):
   global encoder_value2, movement_distance2, dig2
    
   if GPIO.input(encoder2_pin_A) == GPIO.input(encoder2_pin_B):
       encoder_value2 += 1
       dig2 = (encoder_value2*3.14159265358979*2)/(50*800)
       movement_distance2 += dig2*35 # エンコーダの値が増加した場合、移動量も増加
    
   else:
       encoder_value2 -= 1
       dig2 = (encoder_value2*3.14159265358979*2)/(50*800)
       movement_distance2 -= dig2*35  # エンコーダの値が減少した場合、移動量も減少
	#print(f"Encoder Value: {encoder_value2}, Movement Distance: {movement_distance2}")
	
GPIO.add_event_detect(encoder2_pin_A, GPIO.BOTH, callback=handle_encoder2)


WIDTH = 1000          # 横サイズ
HEIGHT = 700       # 縦サイズ
#DEVICE = -1             # カメラデバイスのインデックス（通常は0）

CAMERA_FILE = "camera_AF170.csv" # 出力ファイル
DIST_FILE = "dist_AF170.csv"     # 出力ファイル名

# カメラの歪み係数を読み込む
def loadCalibrationFile():
	try:
		camera = np.loadtxt(CAMERA_FILE, delimiter=',')
		dist = np.loadtxt(DIST_FILE, delimiter=',')
	except Exception as e:
		raise e
	return camera, dist

#=======================================================================
# 赤色は２つの領域に
# np.array([色彩H, 彩度S, 明度V])
# 各値は適宜設定！

#red
R_LOW_COLOR1 = np.array([0,80,80]) # 各最小値を指定
R_HIGH_COLOR1 = np.array([5,255,255]) # 各最大値を指定
R_LOW_COLOR2 = np.array([170,80,50])
R_HIGH_COLOR2 = np.array([180,255,255])

#blue
B_LOW_COLOR1 = np.array([105,80,80])
B_HIGH_COLOR1 = np.array([125,255,255])

#Orange
O_LOW_COLOR1 = np.array([8,120,132])
O_HIGH_COLOR1 = np.array([20,220,255])

#green
G_LOW_COLOR1 = np.array([61,145,145])
G_HIGH_COLOR1 = np.array([69,255,245])

#yellow
#Y_LOW_COLOR1 = np.array([30,50,50])
#Y_HIGH_COLOR1 = np.array([40,255,255])

#LightGreen
L_LOW_COLOR1 = np.array([24,100,100])
L_HIGH_COLOR1 = np.array([39,255,255])

#Purple
P_LOW_COLOR1 = np.array([130,80,80])
P_HIGH_COLOR1 = np.array([158,255,255])


def ColorPixel(img_name):
	global redarea, bluearea, orangearea, greenarea, yellowarea, purplearea,lightgreenarea ,area_list_max
	
	img = cv2.imread(img_name) # 画像を読み込む

	img_yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV) # RGB => YUV(YCbCr)
	clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(100,100)) # claheオブジェクトを生成
	img_yuv[:,:,0] = clahe.apply(img_yuv[:,:,0]) # 輝度にのみヒストグラム平坦化
	img = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR) # YUV => RGB
	
	img_blur = cv2.blur(img, (15, 15)) # 平滑化フィルタを適用
	
	hsv = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV) # BGRからHSVに変換
#==============================================================================================
	#red
	bin_img1 = cv2.inRange(hsv, R_LOW_COLOR1, R_HIGH_COLOR1) # マスクを作成
	bin_img2 = cv2.inRange(hsv, R_LOW_COLOR2, R_HIGH_COLOR2)
	mask_R = bin_img1 + bin_img2 # 必要ならマスクを足し合わせる
	redarea = cv2.countNonZero(mask_R)
	print("redarea=",redarea)
	masked_img_R = cv2.bitwise_and(img_blur, img_blur, mask= mask_R) # 元画像から特定の色を抽出
	
	#blue
	#mask_B = cv2.inRange(hsv, B_LOW_COLOR1, B_HIGH_COLOR1)
	#bluearea = cv2.countNonZero(mask_B)
	#print("bluearea=",bluearea)
	#masked_img_B = cv2.bitwise_and(img_blur, img_blur, mask= mask_B)
	
	#Orange
	mask_O = cv2.inRange(hsv, O_LOW_COLOR1, O_HIGH_COLOR1)
	orangearea = cv2.countNonZero(mask_O)
	print("orangearea=",orangearea)
	masked_img_O = cv2.bitwise_and(img_blur, img_blur, mask= mask_O)
	
	#green
	mask_G = cv2.inRange(hsv, G_LOW_COLOR1, G_HIGH_COLOR1)
	greenarea = cv2.countNonZero(mask_G)
	print("greenarea=",greenarea)
	masked_img_G = cv2.bitwise_and(img_blur, img_blur, mask= mask_G)
	
	#yellow
	#mask_Y = cv2.inRange(hsv, Y_LOW_COLOR1, Y_HIGH_COLOR1)
	#yellowarea = cv2.countNonZero(mask_Y)
	#print("yellowarea=",yellowarea)
	#masked_img_Y = cv2.bitwise_and(img_blur, img_blur, mask= mask_Y)
	
	#lightgreen
	mask_L = cv2.inRange(hsv, L_LOW_COLOR1, L_HIGH_COLOR1)
	lightgreenarea = cv2.countNonZero(mask_L)
	print("lightgreenarea=",lightgreenarea)
	masked_img_L = cv2.bitwise_and(img_blur, img_blur, mask= mask_L)
	
	#purple
	mask_P = cv2.inRange(hsv, P_LOW_COLOR1, P_HIGH_COLOR1)
	purplearea = cv2.countNonZero(mask_P)
	print("purplearea=",purplearea)
	masked_img_P = cv2.bitwise_and(img_blur, img_blur, mask= mask_P)
	
#============================================================================
	area_list = [redarea, orangearea, lightgreenarea, greenarea, purplearea]
	
	area_list_max = max(area_list)
	print(area_list_max)
#==========================================================================	
	if (redarea == area_list_max):
		print("redarea")
		cv2.imwrite("out_img_R.jpg", masked_img_R) # 書き出す
	
	#elif (bluearea  == area_list_max):
		#print("bluearea")
		#cv2.imwrite("out_img_B.jpg", masked_img_B)
	
	elif (orangearea  == area_list_max):
		print("orangearea")
		cv2.imwrite("out_img_O.jpg", masked_img_O)
		
	elif (greenarea  == area_list_max):
		print("greenarea")
		cv2.imwrite("out_img_G.jpg", masked_img_G) 
	
	#elif (yellowarea  == area_list_max):
		#print("yellowarea")
		#cv2.imwrite("out_img_Y.jpg", masked_img_Y) 
	
	elif (lightgreenarea  == area_list_max):
		print("lightgreenarea")
		cv2.imwrite("out_img_L.jpg", masked_img_L) 
	
	elif (purplearea  == area_list_max):
		print("purplearea")
		cv2.imwrite("out_img_P.jpg", masked_img_P) 



		


num = 0
redarea1 = redarea2 = 0
orangearea1 = orangearea2 = 0
greenarea1 = greenarea2 = 0
lightgreenarea1 = lightgreenarea2 = 0
purplearea1 = purplearea2 = 0		
		
		
while True:
	# 歪み係数ファイルを開き、マップを計算
	
	t1=time.time()
	camera, dist = loadCalibrationFile()
	mapX, mapY = cv2.initUndistortRectifyMap(camera, dist, None, None, (WIDTH, HEIGHT), cv2.CV_32FC1)
	video1 = cv2.VideoCapture(0)
	video1.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
	video1.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
	_1, frame1 = video1.read()
	ret_img1 = cv2.remap(frame1, mapX, mapY, cv2.INTER_LINEAR)
	
	cv2.imwrite("image1.jpg",ret_img1)
	ColorPixel("image1.jpg")
	video1.release()
	
	redarea = redarea1
	orangearea = orangearea1
	greenarea = greenarea1
	lightgreenarea = lightgreenarea1
	purplearea = purplearea1
	
	camera, dist = loadCalibrationFile()
	video2 = cv2.VideoCapture(2)
	video2.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
	video2.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
	_2, frame2 = video2.read()
	ret_img2 = cv2.remap(frame2, mapX, mapY, cv2.INTER_LINEAR)
	
	cv2.imwrite("image2.jpg",ret_img2)
	ColorPixel("image2.jpg")
	video2.release()
	
	#redarea = redarea2
	#orangearea = orangearea2
	#greenarea = greenarea2
	#lightgreenarea = lightgreenarea2
	#purplearea = purplearea2
	
	Redarea = redarea1 #+ redarea2
	Orangearea = orangearea1 + orangearea
	Greenarea = greenarea + greenarea
	Lightgreenarea = lightgreenarea1 + lightgreenarea
	
	t2=time.time()
	print(t2-t1)
	
	if (Redarea == area_list_max) and Redarea > 3500:
		print("redarea")
		#Migi
		movement_distance1=0
		encoder_value1=0
		dig1=0
		
		#Hidari
		movement_distance2=0
		encoder_value2=0
		dig2=0
		while True:
			#黄緑から赤になる場合-左旋回
			if num == 4:
				while True:
					handle_encoder1(1)
					mc.set_speed(1, int(-70))
					mc.set_speed(2, int(-70))
					print("Hidari")
					
					if (abs(movement_distance1) > 10):
						num = 0 
						break
				
			num = 0
			mc.set_speed(1, int(-250))
			mc.set_speed(2, int(233))
			camera, dist = loadCalibrationFile()
			mapX, mapY = cv2.initUndistortRectifyMap(camera, dist, None, None, (WIDTH, HEIGHT), cv2.CV_32FC1)
			video = cv2.VideoCapture(-1)
			video.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
			video.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
			_, frame = video.read()
			ret_img = cv2.remap(frame, mapX, mapY, cv2.INTER_LINEAR)
			cv2.imwrite("image_red.jpg",ret_img)
			ColorPixel("image_red.jpg")
			video.release()
			
			
			if redarea != area_list_max or redarea < 3000:
				num = 1
				break
				
	#elif (bluearea  == area_list_max) and bluearea > 5000:
	    #print("bluearea")
	    #while True:
			#camera, dist = loadCalibrationFile()
			#mapX, mapY = cv2.initUndistortRectifyMap(camera, dist, None, None, (WIDTH, HEIGHT), cv2.CV_32FC1)
			#video = cv2.VideoCapture(-1)
			#video.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
			#video.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
			#_, frame = video.read()
			#ret_img = cv2.remap(frame, mapX, mapY, cv2.INTER_LINEAR)
			#cv2.imwrite("image2.jpg",ret_img)
			#ColorPixel("image2.jpg")
			#video.release()
			
			#mc.set_speed(1, int(-201))
			#mc.set_speed(2, int(226))
			
			#if bluearea != area_list_max or bluearea < 3000:
				#break
				
	
	
	elif (orangearea  == area_list_max) and orangearea > 3500:
		print("orangearea")
		#Migi
		movement_distance1=0
		encoder_value1=0
		dig1=0
		
		#Hidari
		movement_distance2=0
		encoder_value2=0
		dig2=0
		
		while True:
			#緑から橙になる場合-右旋回
			if num == 3:
				while True:
					handle_encoder1(1)
					mc.set_speed(1, int(70))
					mc.set_speed(2, int(70))
					print("Migi")
					
					if (abs(movement_distance1) > 10):
						num = 0 
						break
			
			num = 0
			mc.set_speed(1, int(-233))
			mc.set_speed(2, int(250))
			camera, dist = loadCalibrationFile()
			mapX, mapY = cv2.initUndistortRectifyMap(camera, dist, None, None, (WIDTH, HEIGHT), cv2.CV_32FC1)
			video = cv2.VideoCapture(-1)
			video.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
			video.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
			_, frame = video.read()
			ret_img = cv2.remap(frame, mapX, mapY, cv2.INTER_LINEAR)
			cv2.imwrite("image_orange.jpg",ret_img)
			ColorPixel("image_orange.jpg")
			video.release()
			
			
			if orangearea != area_list_max or orangearea < 3000:
				num = 2
				break
		

		
	elif (greenarea  == area_list_max) and greenarea > 3500:
		print("greenarea")
		#Migi
		movement_distance1=0
		encoder_value1=0
		dig1=0
		
		#Hidari
		movement_distance2=0
		encoder_value2=0
		dig2=0
		while True:
			#橙から緑になる場合-左旋回
			if num == 2:
				while True:
					handle_encoder1(1)
					mc.set_speed(1, int(-70))
					mc.set_speed(2, int(-70))
					print("Hidari")
					
					if (abs(movement_distance1) > 10):
						num = 0 
						break
				
			num = 0
			mc.set_speed(1, int(-130))
			mc.set_speed(2, int(113.3))
			camera, dist = loadCalibrationFile()
			mapX, mapY = cv2.initUndistortRectifyMap(camera, dist, None, None, (WIDTH, HEIGHT), cv2.CV_32FC1)
			video = cv2.VideoCapture(-1)
			video.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
			video.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
			_, frame = video.read()
			ret_img = cv2.remap(frame, mapX, mapY, cv2.INTER_LINEAR)
			cv2.imwrite("image_green.jpg",ret_img)
			ColorPixel("image_green.jpg")
			video.release()
			
			
			if greenarea != area_list_max or greenarea < 3000:
				num = 3
				break
	
	#elif (yellowarea  == area_list_max) and yellowarea > 5000:
		#print("yellowarea")
		#while True:
			#camera, dist = loadCalibrationFile()
			#mapX, mapY = cv2.initUndistortRectifyMap(camera, dist, None, None, (WIDTH, HEIGHT), cv2.CV_32FC1)
			#video = cv2.VideoCapture(-1)
			#video.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
			#video.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
			#_, frame = video.read()
			#ret_img = cv2.remap(frame, mapX, mapY, cv2.INTER_LINEAR)
			#cv2.imwrite("image4.jpg",ret_img)
			#ColorPixel("image4.jpg")
			#video.release()
			
			#mc.set_speed(1, int(-22))
			#mc.set_speed(2, int(47))
			
			#if yellowarea != area_list_max or yellowarea < 3000:
				#break
	
	elif (lightgreenarea  == area_list_max) and lightgreenarea > 3500:
		print("lightgreenarea")
		#Migi
		movement_distance1=0
		encoder_value1=0
		dig1=0
		
		#Hidari
		movement_distance2=0
		encoder_value2=0
		dig2=0
		while True:
			#赤から黄緑になる場合-右旋回
			if num == 1:
				while True:
					handle_encoder1(1)
					mc.set_speed(1, int(70))
					mc.set_speed(2, int(70))
					print("Migi")
					
					if (abs(movement_distance1) > 10):
						num = 0 
						break
			
			num = 0
			mc.set_speed(1, int(-113.3))
			mc.set_speed(2, int(130))
			camera, dist = loadCalibrationFile()
			mapX, mapY = cv2.initUndistortRectifyMap(camera, dist, None, None, (WIDTH, HEIGHT), cv2.CV_32FC1)
			video = cv2.VideoCapture(-1)
			video.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
			video.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
			_, frame = video.read()
			ret_img = cv2.remap(frame, mapX, mapY, cv2.INTER_LINEAR)
			cv2.imwrite("image_lightgreen.jpg",ret_img)
			ColorPixel("image_lightgreen.jpg")
			video.release()
			
			
			if lightgreenarea != area_list_max or lightgreenarea < 3000:
				num = 4
				break
		
	
	elif (purplearea  == area_list_max) and purplearea > 3500:
		print("purplearea")
		#Migi
		movement_distance1=0
		encoder_value1=0
		dig1=0
		
		#Hidari
		movement_distance2=0
		encoder_value2=0
		dig2=0
		while True:
			#赤から紫になる場合-右旋回
			if num == 1:
				while True:
					handle_encoder1(1)
					mc.set_speed(1, int(70))
					mc.set_speed(2, int(70))
					print("Migi")
					
					if (abs(movement_distance1) > 10):
						num = 0 
						break
			
			#橙から紫になる場合-左旋回
			if num == 2:
				while True:
					handle_encoder1(1)
					mc.set_speed(1, int(-70))
					mc.set_speed(2, int(-70))
					print("Hidari")
					
					if (abs(movement_distance1) > 10):
						num = 0 
						break
			
			#緑から紫になる場合-右旋回
			if num == 3:
				while True:
					handle_encoder1(1)
					mc.set_speed(1, int(70))
					mc.set_speed(2, int(70))
					print("Migi")
					
					if (abs(movement_distance1) > 10):
						num = 0 
						break
			
			#黄緑から紫になる場合-左旋回
			if num == 4:
				while True:
					handle_encoder1(1)
					mc.set_speed(1, int(-70))
					mc.set_speed(2, int(-70))
					print("Hidari")
					
					if (abs(movement_distance1) > 10):
						num = 0 
						break
			
			
			num = 0
			mc.set_speed(1, int(-145))
			mc.set_speed(2, int(150))
			camera, dist = loadCalibrationFile()
			mapX, mapY = cv2.initUndistortRectifyMap(camera, dist, None, None, (WIDTH, HEIGHT), cv2.CV_32FC1)
			video = cv2.VideoCapture(-1)
			video.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
			video.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
			_, frame = video.read()
			ret_img = cv2.remap(frame, mapX, mapY, cv2.INTER_LINEAR)
			cv2.imwrite("image_purple.jpg",ret_img)
			ColorPixel("image_purple.jpg")
			video.release()
			
			
			if purplearea != area_list_max or purplearea < 3000:
				num = 5
				break
		
		
	else:
		print("else")
