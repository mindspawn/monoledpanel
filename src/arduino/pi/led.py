import smbus
import time
bus = smbus.SMBus(1)

SLAVE_ADDRESS = 4

OP_CODE_SET_PIXEL  = 0xF0
OP_CODE_CLEAR_PIXEL = 0xF1
OP_CODE_RESET      = 0xF2
OP_CODE_CLEAR_ALL_FB= 0xF3
OP_CODE_CLEAR_FB = 0xF4
OP_CODE_SWITCH_FB = 0xF5
OP_CODE_COPY_FB = 0xF6
OP_CODE_BARS = 0xFF

LED_NUM_ROWS = 32
LED_NUM_COLS = 120

def ledSetPixel(fb, row, col):
	bus.write_byte(SLAVE_ADDRESS, OP_CODE_SET_PIXEL)
	bus.write_byte(SLAVE_ADDRESS, fb)
	bus.write_byte(SLAVE_ADDRESS, row)
	bus.write_byte(SLAVE_ADDRESS, col)

def ledClearPixel(fb, row, col):
	bus.write_byte(SLAVE_ADDRESS, OP_CODE_CLEAR_PIXEL)
	bus.write_byte(SLAVE_ADDRESS, fb)
	bus.write_byte(SLAVE_ADDRESS, row)
	bus.write_byte(SLAVE_ADDRESS, col)

def ledClearFB(fb):
	bus.write_byte(SLAVE_ADDRESS, OP_CODE_CLEAR_FB)
	bus.write_byte(SLAVE_ADDRESS, fb)
	bus.write_byte(SLAVE_ADDRESS, 0)
	bus.write_byte(SLAVE_ADDRESS, 0)

def ledSwitchFB(fb):
	bus.write_byte(SLAVE_ADDRESS, OP_CODE_SWITCH_FB)
	bus.write_byte(SLAVE_ADDRESS, fb)
	bus.write_byte(SLAVE_ADDRESS, 0)
	bus.write_byte(SLAVE_ADDRESS, 0)

def ledDrawBars(barHeights):
	if len(barHeights) <= LED_NUM_COLS:
		bus.write_byte(SLAVE_ADDRESS, OP_CODE_BARS)
		for i in range(LED_NUM_COLS):
			try:
				bus.write_byte(SLAVE_ADDRESS, barHeights[i])
			except IndexError:
				bus.write_byte(SLAVE_ADDRESS, 0)


	
