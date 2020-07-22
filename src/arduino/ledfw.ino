#include <SPI.h>
#include <Wire.h>

#define DRAWING_MODE_NORMAL    0
#define DRAWING_MODE_BARS      1

byte currDrawingMode;

#define OP_CODE_SET_PIXEL       0xF0
#define OP_CODE_CLEAR_PIXEL     0xF1
#define OP_CODE_RESET           0xF2
#define OP_CODE_CLEAR_ALL_FB    0xF3
#define OP_CODE_CLEAR_FB        0xF4
#define OP_CODE_SWITCH_FB       0xF5
#define OP_CODE_COPY_FB         0xF6
#define OP_CODE_ENTER_NORMAL_MODE 0xF7

// Enable use of SPI HW for accelerated data transfers
#define USE_SPI_HW    1
// TODO: Try turning using analogWrite(255, ) to turn off OE when transitioning to avoid prev line showing?
#define USE_PWM_HW    0 // Currently buggy. Line above last lit row seems to be faintly on

// SPI HW Config
#define HW_SPI_CLK_DIV  SPI_CLOCK_DIV2
#define HW_SPI_MODE     SPI_MODE0

// I2C HW Config
#define HW_I2C_SLAVE_ADDRESS   4
#define HW_I2C_FREQ            400000L  // Fast Mode


// PWM HW Config
#define HW_PWM_DUTY_CYCLE_LED_ON    0
#define HW_PWM_DUTY_CYCLE_LED_OFF   255
// Current DUTY CYCLE
#define HW_PWM_DUTY_CYCLE   (3*HW_PWM_DUTY_CYCLE_LED_OFF/4)  // 1 is brightest 255 is dimmest

// Pin Configuraion

//      Name             Arduino     Wire     LED Connector pin
// ROW Selection
// ROW_SELECT() macro hard assumes the following 4 pin allocations !!!!!
#define PIN_ROW_SEL_A       4      // red         1
#define PIN_ROW_SEL_B       5      // brown       2
#define PIN_ROW_SEL_C       6      // yellow      3
#define PIN_ROW_SEL_D       7      // orange      4

// Column Selection (62705CP)
// NOTE: SDA/CLK must use 11/13 for SPI hardware
#define PIN_COL_LATCH_BAR_A 8      // purple      13 (Active Low)
#define PIN_COL_EN_BAR_A    9      // grey        15 (Active Low)
#define PIN_COL_SDA         11     // green       5
#define PIN_COL_CLK         13     // blue        11

#define PIN_COL_LATCH_BAR_B 3      // purple      13 (Active Low)
#define PIN_COL_EN_BAR_B    10     // grey        15 (Active Low)
// Power
//  NA                     GND     // white       12



// Display Geometry
#define NUM_LED_ROWS_PER_GRID    8
#define NUM_LED_COLS_PER_GRID    8

// Panel dimensions
#define NUM_LED_GRIDS_PER_ROW    15
#define NUM_LED_GRIDS_PER_COL    4

#define NUM_LED_ROWS             (NUM_LED_GRIDS_PER_COL * NUM_LED_ROWS_PER_GRID)
#define NUM_LED_COLS             (NUM_LED_GRIDS_PER_ROW * NUM_LED_COLS_PER_GRID) // 3 x 5 x 8

#define NUM_BARS                 NUM_LED_COLS
#define MIN_BAR_HEIGHT           0
#define MAX_BAR_HEIGHT           (NUM_LED_ROWS)


#define I2C_CMD_SIZE_BYTES    4
byte i2cCmds[I2C_CMD_SIZE_BYTES];
byte cmdIndex;

// Frame buffer bitmap
#define NUM_FRAME_BUFFERS        2
#define FB_NUM_LED_COL_BYTES    (NUM_LED_COLS/8)
#define FB_SIZE_BYTES           (FB_NUM_LED_COL_BYTES * NUM_LED_ROWS)

byte frameBuffers[NUM_FRAME_BUFFERS][FB_SIZE_BYTES];
byte currFrameBufferIndex;
byte nextFrameBufferIndex;

void clearFrameBuffer(byte fbIndex) {
  if (fbIndex < NUM_FRAME_BUFFERS) {
    memset(&frameBuffers[fbIndex][0], 0, FB_SIZE_BYTES);
  }
}

void clearAllFrameBuffers() {
  for(unsigned int i=0; i<NUM_FRAME_BUFFERS; i++)
    clearFrameBuffer(i);
}

void resetFrameBuffers() {
  currFrameBufferIndex = 0;
  nextFrameBufferIndex = 0;
  clearAllFrameBuffers();
}

void switchFrameBufferOnVSync(byte fbIndex) {
  if (fbIndex < NUM_FRAME_BUFFERS) {
    nextFrameBufferIndex = fbIndex;
  }
}

void copyFrameBuffer(byte dstIndx, byte srcIndx) {
  if (dstIndx < NUM_FRAME_BUFFERS && srcIndx < NUM_FRAME_BUFFERS) {
    memcpy(&frameBuffers[dstIndx][0], &frameBuffers[srcIndx][0], FB_SIZE_BYTES);
  }
}

void setPixel(byte fbIndex, byte r, byte c) {
  if (fbIndex < NUM_FRAME_BUFFERS && r < NUM_LED_ROWS && c < NUM_LED_COLS) {
    const unsigned int byteOffset = ((unsigned int)(r) * FB_NUM_LED_COL_BYTES) + (c >> 3);
    frameBuffers[fbIndex][byteOffset] |= (1 << (c&0x7));
  }
}

void clearPixel(byte fbIndex, byte r, byte c) {
  if (fbIndex < NUM_FRAME_BUFFERS && r < NUM_LED_ROWS && c < NUM_LED_COLS) {
    const unsigned int byteOffset = ((unsigned int)(r) * FB_NUM_LED_COL_BYTES) + (c >> 3);
    frameBuffers[fbIndex][byteOffset] &= ~(1 << (c&0x7));
  }
}


// Hardware macros

// Latch control macros
#define COL_LATCH_A_ASSERT()    digitalWrite(PIN_COL_LATCH_BAR_A, LOW)
#define COL_LATCH_A_DEASSERT()  digitalWrite(PIN_COL_LATCH_BAR_A, HIGH)
#define COL_LATCH_A()           COL_LATCH_A_DEASSERT(); COL_LATCH_A_ASSERT();

#define COL_LATCH_B_ASSERT()    digitalWrite(PIN_COL_LATCH_BAR_B, LOW)
#define COL_LATCH_B_DEASSERT()  digitalWrite(PIN_COL_LATCH_BAR_B, HIGH)
#define COL_LATCH_B()           COL_LATCH_B_DEASSERT(); COL_LATCH_B_ASSERT();

// OE control macros
#define COL_EN_A_ASSERT()       digitalWrite(PIN_COL_EN_BAR_A,    LOW)
#define COL_EN_A_DEASSERT()     digitalWrite(PIN_COL_EN_BAR_A,    HIGH)
#define COL_EN_B_ASSERT()       digitalWrite(PIN_COL_EN_BAR_B,    LOW)
#define COL_EN_B_DEASSERT()     digitalWrite(PIN_COL_EN_BAR_B,    HIGH)

// Bit banging support via software and hardware
#if USE_SPI_HW
#define COL_SDA_BANG(byteVal) SPI.transfer(byteVal)
#else
#define COL_SDA_BANG(byteVal) shiftOut(PIN_COL_SDA, PIN_COL_CLK, MSBFIRST, byteVal)
#endif

// Force clock to zero
#define RESET_CLK()           digitalWrite(PIN_COL_CLK, LOW)
// Select required rows with a single register write.S
#define ROW_SELECT(rowSel)    PORTD = ((rowSel & 0x0F) << 4)

// NOTE: OE is inverted logic
// TODO: cleanup hardcoded constant 16
inline void enablePWM(byte row) {
  if (row < 16) {
    analogWrite(PIN_COL_EN_BAR_A, HW_PWM_DUTY_CYCLE);
  }
  else {
    analogWrite(PIN_COL_EN_BAR_B, HW_PWM_DUTY_CYCLE);
  }
}

inline void disablePWM(byte row) {
  if (row < 16) {
    analogWrite(PIN_COL_EN_BAR_A, HW_PWM_DUTY_CYCLE_LED_OFF);
  }
  else {
    analogWrite(PIN_COL_EN_BAR_B, HW_PWM_DUTY_CYCLE_LED_OFF);
  }
}

inline void colLatch(byte row) {
   if (row < 16) {
      COL_LATCH_A();
   }
   else {
      COL_LATCH_B();
   }
}

inline void colEnAssert(byte row) {
   if (row < 16) {
      COL_EN_A_ASSERT();
   }
   else {
      COL_EN_B_ASSERT();
   }
}

inline void colEnDeassert(byte row) {
   if (row < 16) {
      COL_EN_A_DEASSERT();
   }
   else {
      COL_EN_B_DEASSERT();
   }
}

// I2C event handler
void i2cRecvEvent(int numBytesRcv);

void setup_leds() {
  // Initialize clk to low
  RESET_CLK();
  COL_EN_A_DEASSERT();  // high
  COL_EN_B_DEASSERT();  // high
  COL_LATCH_A_ASSERT(); // low
  COL_LATCH_B_ASSERT(); // low

#if USE_PWM_HW
  analogWrite(PIN_COL_EN_BAR_A, HW_PWM_DUTY_CYCLE);
  analogWrite(PIN_COL_EN_BAR_B, HW_PWM_DUTY_CYCLE);
#endif
}

// Global array containing bar heights
// 0                : no bar
// 1-MAX_BAR_HEIGHT : Bar Height
byte *barHeights;


// N-1 slots will be usable
#define NUM_BAR_FIFO_SLOTS     3

byte barsFifo[NUM_BAR_FIFO_SLOTS][NUM_BARS];
byte barsFifoPI;
byte barsFifoCI;
byte piByteIndex;

void resetBarsFifo() {
  barsFifoPI = 0;
  barsFifoCI = 0;
  piByteIndex = 0;
  barHeights = &barsFifo[0][0];
}

#define START_OF_FRAME     0xFF


inline bool isBarsFifoEmpty() {
  return (barsFifoPI == barsFifoCI);
}

inline bool isBarsFifoFull() {
  byte nextPI = barsFifoPI + 1;

  if (nextPI == NUM_BAR_FIFO_SLOTS)
    nextPI = 0;

  return (nextPI == barsFifoCI);
}

inline void fillBarsFifo(byte data) {
  // If end of frame then check if new op code is a mode switch
  if (piByteIndex == 0) {
    if (data == OP_CODE_ENTER_NORMAL_MODE) {
      currDrawingMode = DRAWING_MODE_NORMAL;
      i2cCmds[0] = data;
      cmdIndex = 1;
      return;
    }
  }
  // For additional sync check for SOF
  // If SOF is received in middle of a frame restart filling frame in same slot
  if (data == START_OF_FRAME) {
    if (piByteIndex != 0) {
      // Out of sync SOF received
    }
    
    piByteIndex = 0;
    return;
  }

  // Save incoming data to buffer slot
  barsFifo[barsFifoPI][piByteIndex++] = data;

  // If current slot has been filled
  if (piByteIndex == NUM_BARS) {
    piByteIndex = 0;

    // Current slot has been filled

    // If FIFO is not full move to the next slot
    if (! isBarsFifoFull()) {
      barsFifoPI++;

      if (barsFifoPI == NUM_BAR_FIFO_SLOTS)
        barsFifoPI = 0;
    }
    // Otherwise, reuse the last slot
  }
}

inline void getNextFrame() {
  // If there are unprocessed frames move to the next one
  if (! isBarsFifoEmpty() ) {
    const byte prevCI = barsFifoCI;

    // Consume the frame
    barsFifoCI++;
    
    if (barsFifoCI == NUM_BAR_FIFO_SLOTS)
      barsFifoCI = 0;

    // Keep pointing to the latest frame location (new CI will be unwritten)
    barHeights = &barsFifo[prevCI][0];
  }

  // If no more unprocessed frames exist contiue displaying last one
}

void setup_sa()
{
}

#if USE_SPI_HW
void setup_spi()
{
  SPI.setClockDivider(HW_SPI_CLK_DIV);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(HW_SPI_MODE);
  SPI.begin();
}
#endif

void setup_i2c() {
  Wire.setClock(HW_I2C_FREQ);
  Wire.begin(HW_I2C_SLAVE_ADDRESS);
  Wire.onReceive(i2cRecvEvent);
}

void setup() {
  // Set all pins to output direction
  pinMode(PIN_ROW_SEL_A,       OUTPUT);
  pinMode(PIN_ROW_SEL_B,       OUTPUT);
  pinMode(PIN_ROW_SEL_C,       OUTPUT);
  pinMode(PIN_ROW_SEL_D,       OUTPUT);
  pinMode(PIN_COL_SDA,         OUTPUT);
  pinMode(PIN_COL_CLK,         OUTPUT);
  pinMode(PIN_COL_LATCH_BAR_A, OUTPUT);
  pinMode(PIN_COL_LATCH_BAR_B, OUTPUT);
  pinMode(PIN_COL_EN_BAR_A,    OUTPUT);
  pinMode(PIN_COL_EN_BAR_B,    OUTPUT);

  resetBarsFifo();
  
#if USE_SPI_HW
  setup_spi();
#endif
  setup_i2c();

  setup_leds();
  setup_sa();

  resetFrameBuffers();

  currDrawingMode = DRAWING_MODE_NORMAL;
}

// This was initial design where bar heights were transfered
void transferRowDataBars(byte currRow) {
  const byte currRowHeight = NUM_LED_ROWS - currRow;
  
  // Loop all columns of grids
  for(byte currGridCol=NUM_LED_GRIDS_PER_ROW; currGridCol-->0; ) {
    byte opByte = 0;
    
    // Loop all bits in a column grid
    for(byte bitPos=NUM_LED_COLS_PER_GRID; bitPos-->0; ) {
      const byte barIndex = (currGridCol * NUM_LED_COLS_PER_GRID) + bitPos;
      // Only set bits when below the bar heights array
      if (currRowHeight <= barHeights[barIndex]) {
        opByte |= (1 << bitPos);
      }      
    }
    COL_SDA_BANG(opByte);
  }
}

void transferRowDataNormal(byte currRow) {
  // Reverse order due to shift registers
  const unsigned int endOffset = (((unsigned int)(currRow) + 1) * FB_NUM_LED_COL_BYTES) - 1;
  byte *ptr = &frameBuffers[currFrameBufferIndex][endOffset];
  unsigned int ctr = 0;

  while (ctr++ < NUM_LED_GRIDS_PER_ROW) {
    COL_SDA_BANG(*ptr--);
  }
}


// Called everytime a frame has been completely displayed
inline void onVsync() {
  if (currDrawingMode == DRAWING_MODE_BARS) {
    getNextFrame();
  }
  else {
    currFrameBufferIndex = nextFrameBufferIndex;
  }
}

#if USE_PWM_HW
void loop_sa() {
  static byte currRow = 0;
  const byte nextRow = (currRow + 1) % NUM_LED_ROWS;

  // Select column
  ROW_SELECT(currRow);

  // Turn on PWM
  enablePWM(currRow);

  // Complete frame processed (note: need to take into account pre-fetch into shift registers)
  if (currRow == (NUM_LED_ROWS - 1)) {
    onVsync();
  }

  (currDrawingMode == DRAWING_MODE_BARS) ? transferRowDataBars(nextRow) : transferRowDataNormal(nextRow);

  // Turn off PWM
  disablePWM(currRow);

  colLatch(nextRow);
  
  // Move to next row
  currRow = (currRow + 1) % NUM_LED_ROWS;
}

#else

void loop_sa() {
  static byte currRow = 0;
  const byte nextRow = (currRow + 1) % NUM_LED_ROWS;

  // Select column
  ROW_SELECT(currRow);

  // Overlap compute and transfer of row data with assertion of OE signal
  colEnAssert(currRow); // low

  // Complete frame processed (note: need to take into account pre-fetch into shift registers)
  if (currRow == (NUM_LED_ROWS - 1)) {
    onVsync();
  }

  (currDrawingMode == DRAWING_MODE_BARS) ? transferRowDataBars(nextRow) : transferRowDataNormal(nextRow);

  colEnDeassert(currRow); // high

  // Latch row
  colLatch(nextRow);

  // Move to next row
  currRow = (currRow + 1) % NUM_LED_ROWS;

}
#endif






inline void processNewCmd() {
  const byte opCode = i2cCmds[0];
  const byte arg0 = i2cCmds[1];
  const byte arg1 = i2cCmds[2];
  const byte arg2 = i2cCmds[3];
  
  switch(opCode) {
    case OP_CODE_SET_PIXEL:
      setPixel(arg0, arg1, arg2);
      break;
    case OP_CODE_CLEAR_PIXEL:
      clearPixel(arg0, arg1, arg2);
      break;
    case OP_CODE_RESET:
      resetFrameBuffers();
      break;
    case OP_CODE_CLEAR_ALL_FB:
      clearAllFrameBuffers();
      break;
    case OP_CODE_CLEAR_FB:
      clearFrameBuffer(arg0);
      break;
    case OP_CODE_SWITCH_FB:
      switchFrameBufferOnVSync(arg0);
      break;
    case OP_CODE_COPY_FB:
      copyFrameBuffer(arg0, arg1);
      break;
    default:
      break;
  }
}

void i2cRecvEvent(int numBytesRcv) {
  while (0 != Wire.available()) {
    if (currDrawingMode == DRAWING_MODE_BARS) {
      fillBarsFifo(Wire.read());
    }
    else {
      const byte newByte = Wire.read();

      // Check for mode switch
      if (cmdIndex==0 && newByte==START_OF_FRAME) {
        currDrawingMode = DRAWING_MODE_BARS;
        fillBarsFifo(newByte);
      }
      else {
        i2cCmds[cmdIndex++] = newByte;
        if (cmdIndex == I2C_CMD_SIZE_BYTES) {
          processNewCmd();  
          cmdIndex = 0;
        }
      }
    }
  }
}

void loop() {
  loop_sa();
}

