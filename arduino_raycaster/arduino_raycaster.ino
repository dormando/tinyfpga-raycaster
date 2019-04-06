#include <SPI.h>
#include "textures.h"
#include "lookuptables.h"

#define WIDTH 320
#define HEIGHT 240
#define HALF_HEIGHT 120

#define texWidth 32
#define texHeight 32
#define mapWidth 24
#define mapHeight 24

#define ssPin 10
#define busyPin 9
#define fpgaRstPin 8
#define BUTTON_LEFT 2
#define BUTTON_UP 3
#define BUTTON_RIGHT 4
#define BUTTON_DOWN 5

// TODO: This takes up most of the AVR's RAM!
// should put it in PROGMEM as a sequential array, but need to test to see if
// multiplying out the index and fetching from flash would significantly slow
// down the DDA checks.
const int worldMap[mapWidth][mapHeight] =
{
  {8,8,8,8,8,8,8,8,8,8,8,4,4,6,4,4,6,4,6,4,4,4,6,4},
  {8,0,0,0,0,0,0,0,0,0,8,4,0,0,0,0,0,0,0,0,0,0,0,4},
  {8,0,3,3,0,0,0,0,0,8,8,4,0,0,0,0,0,0,0,0,0,0,0,6},
  {8,0,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6},
  {8,0,3,3,0,0,0,0,0,8,8,4,0,0,0,0,0,0,0,0,0,0,0,4},
  {8,0,0,0,0,0,0,0,0,0,8,4,0,0,0,0,0,6,6,6,0,6,4,6},
  {8,8,8,8,0,8,8,8,8,8,8,4,4,4,4,4,4,6,0,0,0,0,0,6},
  {7,7,7,7,0,7,7,7,7,0,8,0,8,0,8,0,8,4,0,4,0,6,0,6},
  {7,7,0,0,0,0,0,0,7,8,0,8,0,8,0,8,8,6,0,0,0,0,0,6},
  {7,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,6,0,0,0,0,0,4},
  {7,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,6,0,6,0,6,0,6},
  {7,7,0,0,0,0,0,0,7,8,0,8,0,8,0,8,8,6,4,6,0,6,6,6},
  {7,7,7,7,0,7,7,7,7,8,8,4,0,6,8,4,8,3,3,3,0,3,3,3},
  {2,2,2,2,0,2,2,2,2,4,6,4,0,0,6,0,6,3,0,0,0,0,0,3},
  {2,2,0,0,0,0,0,2,2,4,0,0,0,0,0,0,4,3,0,0,0,0,0,3},
  {2,0,0,0,0,0,0,0,2,4,0,0,0,0,0,0,4,3,0,0,0,0,0,3},
  {1,0,0,0,0,0,0,0,1,4,4,4,4,4,6,0,6,3,3,0,0,0,3,3},
  {2,0,0,0,0,0,0,0,2,2,2,1,2,2,2,6,6,0,0,5,0,5,0,5},
  {2,2,0,0,0,0,0,2,2,2,0,0,0,2,2,0,5,0,5,0,0,0,5,5},
  {2,0,0,0,0,0,0,0,2,0,0,0,0,0,2,5,0,5,0,5,0,5,0,5},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5},
  {2,0,0,0,0,0,0,0,2,0,0,0,0,0,2,5,0,5,0,5,0,5,0,5},
  {2,2,0,0,0,0,0,2,2,2,0,0,0,2,2,0,5,0,5,0,0,0,5,5},
  {2,2,2,2,1,2,2,2,2,2,2,1,2,2,2,5,5,5,5,5,5,5,5,5}
};

const uint8_t lcd_inits[] PROGMEM = {0xEF, 3, 0x03, 0x80, 0x02,
0xCF, 3, 0x00, 0xC1, 0x30,
0xED, 4, 0x64, 0x03, 0x12, 0x81,
0xE8, 3, 0x85, 0x00, 0x78,
0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
0xF7, 1, 0x20,
0xEA, 2, 0x00, 0x00,
0xC0  , 1, 0x23,             // Power control VRH[5:0] 0x23: 0b100011 4.60V ?
0xC1  , 1, 0x10,             // Power control SAP[2:0];BT[3:0] 0b10000
0xC5  , 2, 0x3e, 0x28,       // VCM control 0b111110 0b101000
0xC7, 1, 0x86,             // VCM control2
0x36  , 1, 0b11001000,             // Memory Access Control (orig: 0x48)
0x37, 1, 0x00,             // Vertical scroll zero
0x3A, 1, 0x55, // pixel format
0xB1, 2, 0x00, 0x18,
0xB6, 3, 0x08, 0x82, 0x27, // Display Function Control
0xF2, 1, 0x00,                         // 3Gamma Function Disable
0x26, 1, 0x01,             // Gamma curve selected
0xE0, 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00, // set gamma
0xE1, 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F, // set gamma
0x11, 0x80,                // Exit Sleep
0x29, 0x80,                // Display on
0x2A, 4, 0x00, 0x00, 0x00, 0xF0, // CA/PA settings
0x2B, 4, 0x00, 0x00, 0x01, 0x40, // CA/PA settings
0xFF, // end
};

float posX, posY, dirX, dirY, planeX, planeY;
int16_t posXF, posYF, dirXF, dirYF, planeXF, planeYF;

// Some handy debugging functions I've decided to leave in.
void printDebugPins(void) {
  Serial.print("byte: ");
  for (int x = 9; x > 1; x--) {
    if (digitalRead(x) == HIGH) {
      Serial.print('1');
    } else {
      Serial.print('0');
    }
  }
  Serial.println();
}

uint8_t getDebugPins(void) {
  int b = 0;
  uint8_t res = 0;
  for (int x = 2; x < 10; x++) {
    if (digitalRead(x) == HIGH) {
      res |= (1<<b);
    }
    b++;
  }
  return res;
}

void setup() {
  uint8_t cmd;
  pinMode(ssPin, OUTPUT); // SS pin.
  pinMode(busyPin, INPUT); // busy indicator from arduino
  pinMode(fpgaRstPin, OUTPUT); // reset hold for FPGA

  digitalWrite(fpgaRstPin, HIGH);
  // this is a little long due to how I test; only has to be a ms or so.
  delay(2000);
  digitalWrite(fpgaRstPin, LOW);
  delay(10);

  // controller pins.
  pinMode(BUTTON_LEFT, INPUT_PULLUP);
  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_RIGHT, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  
  SPI.begin();
  Serial.begin(9600);

  // Starting positions.
  posX = 6.0;
  posY = 7.5;
  dirX = -1.0;
  dirY = 0.0;
  planeX = 0.0;
  planeY = 0.66;
  
  posXF = (int16_t)(posX * 256);
  posYF = (int16_t)(posY * 256);
  dirXF = (int16_t)(dirX * 256);
  dirYF = (int16_t)(dirY * 256);
  planeXF = (int16_t)(planeX * 256);
  planeYF = (int16_t)(planeY * 256);

  // initialize the LCD.
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  // Reset the LCD
  cmd = 0b00000101; // LCD "read" (hack for accessing RST wire)
  BXRegister(cmd, 0, 1);
  delay(250);
  BXRegister(cmd, 0, 0);
  delay(250);
  BXRegister(cmd, 0, 1);
  delay(250);
  Serial.println(F("Sent LCD reset commands"));
  //delay(2000);

  // Run the encoded LCD sequence.
  lcdInit();
  Serial.println(F("Done sending LCD init commands"));
  texInit();
  Serial.println(F("Done initializing textures"));

  SPI.endTransaction();

  // stay in a transaction for the rest of the loop.
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
}

// load from texture_data[] and palette_data[]
void texInit(void) {
  uint16_t toffset = 0;
  uint16_t tidx    = 0;
  uint16_t poffset = 0;
  uint16_t pidx    = 0;
  uint8_t cmd = 0;

  for (int t = 0; t < 8; t++) {
    uint16_t pcount = 0;
    // load one texture
    cmd = 0b11000011;
    BXRegister(cmd, 1023, toffset);

    // pixels are 4bit. each byte stores two.
    for (int x = 0; x < 512; x++) {
      // so wasteful on the SPI wire :)
      uint8_t pair = pgm_read_byte_near(texture_data + tidx++);
      // send front half.
      BXData(pair & 0x0F);
      // ... and back half.
      BXData(pair >> 4);
    }
    
    // load one palette
    cmd = 0b11000001;
    pcount = pgm_read_word_near(palette_data + pidx++);
    BXRegister(cmd, pcount-1, poffset);
    while (pcount--) {
      BXData(pgm_read_word_near(palette_data + pidx++));
    }
    // tick offsets.
    toffset += 1024;
    poffset += 16;
    delay(10);
  }
}

// the mmap protocol is extremely inflated for this LCD routine.
// however, it's not really performance critical.
// though, could "encode" it all in the address and just use reads :p
void lcdInit(void) {
  uint8_t cmd = 0b10000101; // LCD write.
  uint8_t idx = 0;
  for (;;) {
    uint8_t lcmd = pgm_read_byte_near(lcd_inits + idx);
    idx++;
    // end byte.
    if (lcmd == 0xFF) {
      break;
    }

    // ship LCD command byte.
    BXRegister(cmd, 0, 0);
    BXData(lcmd);
    delay(250);
    
    uint8_t cnt = pgm_read_byte_near(lcd_inits + idx);
    idx++;
    // encoded delay in init routine.
    if (cnt == 0x80) {
      delay(250);
    } else {
      
      for (int x = 0; x < cnt; x++) {
        uint16_t dbyte = pgm_read_byte_near(lcd_inits + idx);
        idx++;
        dbyte |= (1<<8); // set DCX bit high for "data".
        BXRegister(cmd, 0, 0);
        BXData(dbyte);
        delay(10);
      }

    }
  }
}

void loop() {
  unsigned long start = millis();
  BXRegister(0b10000111, 0, 0);
  BXData(((int32_t) posXF << 16) | posYF);
  // Kick off an LCD RAMWR
  BXRegister(0b10000101, 0, 0);
  BXData(0x2C);

  castRays();

  // track time spent rendering so we can adjust move/turn rate at a consistent
  // speed.
  // This isn't *quite* right; should account for time since the last loop
  // came through. Haven't fixed because I keep adjusting :)
  unsigned long elapsed = millis() - start;
  movement(elapsed);
  //elapsed = millis() - start;

  //Serial.print("castRays: ");
  //Serial.println(elapsed);
  
  // wait for FPGA's input queue to drain.
  // when queue is clear, pin is high.
  //unsigned long waited = 0;
  if (digitalRead(busyPin) == LOW) {
      //unsigned long start = micros();
      while (digitalRead(busyPin) == LOW) {

      }
      //waited += micros() - start;
  }
  /*if (waited > 0) {
    Serial.print("waited: ");
    Serial.println(waited);
  }*/
  // TODO: don't have another signal for "and the LCD is idle".
  // when that pin goes high, it's still flushing one last line to the LCD.
  delay(1);
}

// In total frame time, this adds up to half a millisecond.
// Could use lookup tables but not worth it (yet).
// Also: keeping these as float's to get some extra initial accuracy, which we
// then reduce into the Q8.8 fixed point representation.
// Since we're using INPUT_PULLUP, LOW == PUSHED.
void movement(unsigned long elapsed) {
  float moveSpeed = elapsed * 0.003; // 3.0 squares/sec
  float rotSpeed = elapsed * 0.002; // 2.0 radians/sec

  bool left = false;
  bool right = false;
  bool up = false;
  bool down = false;
  if (digitalRead(BUTTON_LEFT) == LOW) {
    //Serial.println("BUTTON_LEFT");
    left = true;
  }
  if (digitalRead(BUTTON_RIGHT) == LOW) {
    //Serial.println("BUTTON_RIGHT");
    right = true;
    // Holding both buttons should go nowhere.
    if (left && right) {
      left = false;
      right = false;
    }
  }
  if (digitalRead(BUTTON_UP) == LOW) {
    //Serial.println("BUTTON_UP");
    up = true;
  }
  if (digitalRead(BUTTON_DOWN) == LOW) {
    //Serial.println("BUTTON_DOWN");
    down = true;
    if (up && down) {
      up = false;
      down = false;
    }
  }

  // FIXME: test map for collision.
  if (up) {
    posX += dirX * moveSpeed;
    posY += dirY * moveSpeed;
  } else if (down) {
    posX -= dirX * moveSpeed;
    posY -= dirY * moveSpeed;
  }

  if (left || right) {
    if (right) {
      // invert rotSpeed just for right movement.
      rotSpeed = -rotSpeed;
    }
    float ox = dirX;
    float opx = planeX;
    dirX = dirX * cos(rotSpeed) - dirY * sin(rotSpeed);
    dirY = ox * sin(rotSpeed) + dirY * cos(rotSpeed);
    planeX = planeX * cos(rotSpeed) - planeY * sin(rotSpeed);
    planeY = opx * sin(rotSpeed) + planeY * cos(rotSpeed);
  }

  // update the fixed point Q8.8 representations.
  posXF = (int16_t)(posX * 256);
  posYF = (int16_t)(posY * 256);
  dirXF = (int16_t)(dirX * 256);
  dirYF = (int16_t)(dirY * 256);
  planeXF = (int16_t)(planeX * 256);
  planeYF = (int16_t)(planeY * 256);
}

#define WIDTH_FIXED (((int32_t)320) << 8)
#define HEIGHT_FIXED (((int32_t)240) << 8)

void debugCompare(int16_t a, float b, const char *id) {
  int16_t t = (int16_t)(b * 256);
  if (abs(a - t) > 2) {
    Serial.print(" "); Serial.print(id); Serial.print(": "); Serial.println(a, BIN);
    Serial.print("F"); Serial.print(id); Serial.print(": "); Serial.println(t, BIN);
    Serial.println();
  }
}

void debugPrint(int16_t a, const char *id) {
  Serial.print(" "); Serial.print(id); Serial.print(": "); Serial.println(a, BIN);
}

void castRays(void) {
  unsigned long elapsed = 0;
  unsigned long waited = 0;
  unsigned long DDA = 0;
  unsigned long DDA_start = 0;
  for (int16_t x = 0; x < WIDTH; x++) {
    //int16_t cameraX = fixed_div32(((int32_t)(2 * x)) << 8, WIDTH_FIXED) - 0x0100;
    int16_t cameraX = pgm_read_word_near(camerax_data + x);

    int16_t rayDirX = dirXF + fixed_mul(planeXF, cameraX);
    int16_t rayDirY = dirYF + fixed_mul(planeYF, cameraX);
    // Avoid some divide by zero's later.
    if (rayDirX == 0) {
      rayDirX = 0x0001;
    }
    if (rayDirY == 0) {
      rayDirY = 0x0001;
    }
    
    // current map box.
    int16_t mapX = (posXF & 0xFF00);
    int16_t mapY = (posYF & 0xFF00);

    // length of ray to next x/y side.
    int16_t sideDistX;
    int16_t sideDistY;

    // use the Norm's as inverses of rayDirXY and avoid a division below.
    int16_t deltaDistXNorm = fixed_div(1 << 8, rayDirX);
    int16_t deltaDistX = abs(deltaDistXNorm);
    int16_t deltaDistYNorm = fixed_div(1 << 8, rayDirY);
    int16_t deltaDistY = abs(deltaDistYNorm);

    int16_t perpWallDist;

    // what direction to step x or y
    int16_t stepX;
    int16_t stepY;

    int hit = 0; // wall hit indicator
    int side; // NS or EW wall hit
    int16_t sideStepX = 0;
    int16_t sideStepY = 0;

    // Initial step/sides.
    if (rayDirX < 0) {
      stepX = 0xFF00; // -1
      sideStepX = 0x0100; // (1 - stepX) / 2
      sideDistX = fixed_mul((posXF - mapX), deltaDistX);
    } else {
      stepX = 0x0100; // 1
      sideDistX = fixed_mul((mapX + 0x0100 - posXF), deltaDistX);
    }

    if (rayDirY < 0) {
      stepY = 0xFF00; // -1
      sideStepY = 0x0100; // (1 - stepY) / 2
      sideDistY = fixed_mul((posYF - mapY), deltaDistY);
    } else {
      stepY = 0x0100; // 1
      sideDistY = fixed_mul((mapY + 0x0100 - posYF), deltaDistY);
    }

    // DDA loop.
    //DDA_start = micros();
    while (hit == 0) {
      // jump in X or Y direction.
      if (sideDistX < sideDistY) {
        sideDistX += deltaDistX;
        mapX += stepX;
        side = 0;
      } else {
        sideDistY += deltaDistY;
        mapY += stepY;
        side = 1;
      }
      if (worldMap[(mapX >> 8)][(mapY >> 8)] > 0) hit = 1;
    }
    //DDA += micros() - DDA_start;

    // Calculate distance of perpendicular ray (Euclidean distance will give fisheye effect!)
    //if (side == 0) perpWallDist = fixed_div((mapX - posXF + sideStepX), rayDirX);
    //else           perpWallDist = fixed_div((mapY - posYF + sideStepY), rayDirY);
    if (side == 0) perpWallDist = fixed_mul((mapX - posXF + sideStepX), deltaDistXNorm);
    else           perpWallDist = fixed_mul((mapY - posYF + sideStepY), deltaDistYNorm);

    // set a minimum wall distance, else we overflow the divider.
    if (perpWallDist < 64) {
      perpWallDist = 64;
    }

    // grab a few divisions out of a lookup table.
    // index via 10 bits of perpWallDist.
    uint32_t inverseIdx = ((perpWallDist & 0b0001111111111000) >> 3) * 3;
    int16_t lineHeight = pgm_read_word_near(pdist_to_height_data + inverseIdx++);
    int16_t texScaler  = pgm_read_word_near(pdist_to_height_data + inverseIdx++);
    int16_t invWallDist= pgm_read_word_near(pdist_to_height_data + inverseIdx);
    
    /*int32_t lineHeight = fixed_div32(HEIGHT_FIXED, perpWallDist) >> 8;
    if (lineHeight > 1024) {
      lineHeight = 1024;
    }*/

    // Calculate lowest and highest pixel to fill in current stripe
    int drawStart = ((-lineHeight) >> 1) + HALF_HEIGHT;
    uint16_t texBelow = 0;
    if (drawStart < 0) {
      // need to remember the offset to start the texture crawl with.
      texBelow = 0 - drawStart;
      drawStart = 0;
    }
    int drawEnd = (lineHeight >> 1) + HALF_HEIGHT;
    if (drawEnd >= HEIGHT) drawEnd = HEIGHT - 1;
    // Texturing calculations
    // FIXME: just use hit from the DDA loop?
    int texNum = worldMap[(mapX >> 8)][(mapY >> 8)] - 1; // 1 subtracted from it so that texture 0 can be used!

    // Calculate value of wallX
    int16_t wallX; // Where exactly the wall was hit
    if (side == 0) wallX = posYF + fixed_mul(perpWallDist, rayDirY);
    else           wallX = posXF + fixed_mul(perpWallDist, rayDirX);
    wallX = wallX & 0x00FF;

    // x coordinate on the texture
    int16_t texX = fixed_mul(wallX, texWidth << 8) >> 8;
    if (side == 0 && rayDirX > 0) texX = texWidth - (texX) - 1;
    if (side == 1 && rayDirY < 0) texX = texWidth - (texX) - 1;

    // tell line drawer to darken the texture.
    if (side == 1) texNum |= 0b10000000;

    // find initial offset of the texture and scaler.
    // grabbed from lookup table
    //int16_t texScaler = (int16_t) (texHeight << 8) / (lineHeight);
    // fixed point mul, then cut it back to an integer.
    int16_t texInitial = fixed_mul(texScaler, texBelow) >> 8;

    // floor casting calculations.
    int16_t floorXWall, floorYWall;
    // 4 different wall directions possible
      if(side == 0 && rayDirX > 0)
      {
        floorXWall = mapX;
        floorYWall = mapY + wallX;
      }
      else if(side == 0 && rayDirX < 0)
      {
        floorXWall = mapX + 1.0;
        floorYWall = mapY + wallX;
      }
      else if(side == 1 && rayDirY > 0)
      {
        floorXWall = mapX + wallX;
        floorYWall = mapY;
      }
      else
      {
        floorXWall = mapX + wallX;
        floorYWall = mapY + 1.0;
      }

    // inverse wall distance. allows FPGA to mul intead of div for floor/ceiling casting.
    //int32_t invWallDist = fixed_div(0x0100, perpWallDist);
    // end floor casting

    //unsigned long start = micros();
    BXRegister(0b11000100, 4, 0);
    // TODO: just ship lineHeight
    // do draw start / end calc (with div 2)
    // + internal lookup table for inverse dist 
    BXData(((uint32_t) drawStart << 16) | (uint32_t)drawEnd);
    BXData(((int32_t) texScaler << 16) | (int32_t)texInitial);
    // TODO: texX is 5 bit, texNum is 8 bit. combine into same 16bit half.
    // then combine with invWallDist, cut SPI traffic by 4 bytes.
    BXData(((uint32_t) texX << 16) | (uint32_t)texNum);
    // Q8.8 for floorXYWall
    BXData( ((int32_t)(floorXWall) << 16) | ((int32_t)(floorYWall)));
    BXData(invWallDist);
    //elapsed += micros() - start;
  }
  //Serial.print("casting: ");
  //Serial.println(elapsed);
  //Serial.print("DDA: ");
  //Serial.println(DDA);
}

// need to give some space for the Q8.8 so other operations don't overflow.
int16_t fixed_div(int16_t a, int16_t b) {
  int32_t res = ((int32_t)a << 8) / (int32_t)b;
  if (res < -16384) {
    res = -16384;
  } else if (res > 16384) {
    res = 16384;
  }
  return res;
}

int16_t fixed_mul(int16_t a, int16_t b) {
  return (((int32_t)a * (int32_t)b) >> 8);
}

// I've left these functions here to show how the calculations would be done
// without the lookup tables.

/*int32_t fixed_div32(int32_t a, int32_t b) {
  return (((int32_t)a << 8) / (int32_t)b);
}

int16_t fixed_mul32(int32_t a, int32_t b) {
  return (((int32_t)a * (int32_t)b) >> 8);
}*/

#define BXWrite(val) SPI.transfer(val)

// data packet sent in separate message since there can be multiple.
// We do the port manipulation directly as digitalWrite() was accounting for
// nearly half of the time spent writing to SPI.
void BXRegister(uint8_t cmd, uint16_t count, uint32_t addr) {
  //digitalWrite(ssPin, LOW);
  PORTB &= 0b11111011;
  BXWrite(cmd); // cmd byte.
  // pkt count.
  BXWrite(count & 0xFF);
  BXWrite((count >> 8) & 0xFF); 
  // addr
  BXWrite(addr & 0xFF);
  BXWrite((addr >> 8) & 0xFF);
  BXWrite((addr >> 16) & 0xFF);
  BXWrite((addr >> 24) & 0xFF);
  //digitalWrite(ssPin, HIGH);
  PORTB |= 0b00000100;
}

void BXData(uint32_t data) {
  //digitalWrite(ssPin, LOW);
  PORTB &= 0b11111011;
  BXWrite(data & 0xFF);
  BXWrite((data >> 8) & 0xFF);
  BXWrite((data >> 16) & 0xFF);
  BXWrite((data >> 24) & 0xFF);
  //digitalWrite(ssPin, HIGH);
  PORTB |= 0b00000100;
}
