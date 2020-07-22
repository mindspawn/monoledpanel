from led import *
import time

ledClearFB(0)
ledClearFB(1)
ledSwitchFB(0)

def scanCols():
	fb = 1
	for col in range(LED_NUM_COLS):
		for row in range(LED_NUM_ROWS):
			ledSetPixel(1, row, col)
		ledSwitchFB(1)
		time.sleep(0.1)
		# clear screen
		ledSwitchFB(0)
		for row in range(LED_NUM_ROWS):
			ledClearPixel(1, row, col)

def scanRows():
	fb = 1
	for row in range(LED_NUM_ROWS):
		for col in range(LED_NUM_COLS):
			ledSetPixel(1, row, col)
		ledSwitchFB(1)
		time.sleep(0.1)
		# clear screen
		ledSwitchFB(0)
		for col in range(LED_NUM_COLS):
			ledClearPixel(1, row, col)
def runPixel():
	for row in range(LED_NUM_ROWS):
		for col in range(LED_NUM_COLS):
			ledSetPixel(1, row, col)
			ledSwitchFB(1)
			time.sleep(0.001)
			ledSwitchFB(0)
			time.sleep(0.001)
			ledClearPixel(1, row, col)
			

#for col in range(LED_NUM_COLS):
#	ledSetPixel(1, 15, col)
#ledSetPixel(1, 15, 0)
#ledSwitchFB(1)
while(1):
	#scanCols()
	#scanRows()
	runPixel()

