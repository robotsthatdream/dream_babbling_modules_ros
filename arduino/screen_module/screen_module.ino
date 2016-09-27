#include <base_arduino_module.h>
#include <mkr1000_hx8357d.h>
#include <SD.h>

#define screen_CS 2
#define screen_DC 3
HX8357D screen;

#define SD_CS 4

#define TOUCH_Xp A0
#define TOUCH_Xm 7
#define TOUCH_Yp A1
#define TOUCH_Ym 6

BabblingModule module(BAB_MODULE_TYPE_SCREEN);

uint16_t readX() {
  pinMode(TOUCH_Ym, INPUT);
  pinMode(TOUCH_Yp, INPUT);
  pinMode(TOUCH_Xm, OUTPUT);
  digitalWrite(TOUCH_Xm, LOW);
  pinMode(TOUCH_Xp, OUTPUT);
  digitalWrite(TOUCH_Xp, HIGH);
  return analogRead(TOUCH_Yp);
}

uint16_t readY() {
  pinMode(TOUCH_Xm, INPUT);
  pinMode(TOUCH_Xp, INPUT);
  pinMode(TOUCH_Ym, OUTPUT);
  digitalWrite(TOUCH_Ym, LOW);
  pinMode(TOUCH_Yp, OUTPUT);
  digitalWrite(TOUCH_Yp, HIGH);
  return analogRead(TOUCH_Xp);
}

void bmpDraw(char *filename, uint8_t x, uint16_t y) {

  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();

  if ((x >= screen.width()) || (y >= screen.height())) return;

  Serial.println();
  Serial.print(F("Loading image '"));
  Serial.print(filename);
  Serial.println('\'');

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL) {
    Serial.print(F("File not found"));
    return;
  }

  // Parse BMP header
  if (read16(bmpFile) == 0x4D42) { // BMP signature
    Serial.print(F("File size: ")); Serial.println(read32(bmpFile));
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    Serial.print(F("Image Offset: ")); Serial.println(bmpImageoffset, DEC);
    // Read DIB header
    Serial.print(F("Header size: ")); Serial.println(read32(bmpFile));
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if (read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      Serial.print(F("Bit Depth: ")); Serial.println(bmpDepth);
      if ((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!
        Serial.print(F("Image size: "));
        Serial.print(bmpWidth);
        Serial.print('x');
        Serial.println(bmpHeight);

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if (bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if ((x + w - 1) >= screen.width())  w = screen.width()  - x;
        if ((y + h - 1) >= screen.height()) h = screen.height() - y;

        // Set screen address window to clipped image bounds
        screen.setAddrWindow(x, y, x + w - 1, y + h - 1);
        uint8_t*  sdbuffer = (uint8_t*)malloc(w * 3 * sizeof(uint8_t)); // pixel buffer (R+G+B per pixel)
        uint8_t*  colorbuffer = (uint8_t*)malloc(w * 2 * sizeof(uint8_t)); // color buffer (565 per pixel)

        for (row = 0; row < h; row++) { // For each scanline...

          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if (flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;
          if (bmpFile.position() != pos) { // Need seek?
            bmpFile.seek(pos);
            //buffidx = sizeof(sdbuffer); // Force buffer reload
          }

          if (bmpFile.read(sdbuffer, w * 3) < 0)
            Serial.println("read failed");
          for (int i = 0; i < w; i++) {
            colorbuffer[2 * i] = sdbuffer[3 * i + 2] & 0xf8 | sdbuffer[3 * i + 1] >> 5;
            colorbuffer[2 * i + 1] = (sdbuffer[3 * i + 1] & 0xfc) << 3 | sdbuffer[3 * i] >> 3;
          }
          screen.pushPixelsDMA(colorbuffer, w * 2);
          //screen.pushBuffer(sdbuffer, w * 3);

        } // end scanline
        Serial.print(F("Loaded in "));
        Serial.print(millis() - startTime);
        Serial.println(" ms");
        free(sdbuffer);
        free(colorbuffer);
      } // end goodBmp
    }
  }

  bmpFile.close();
  if (!goodBmp) Serial.println(F("BMP format not recognized."));
}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(File &f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(File &f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}

void setup() {
  Serial.begin(9600);
  //while (!Serial);

  screen.begin(screen_CS, screen_DC);

  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("failed!");
  }
  Serial.println("OK!");

  module.begin();
}

void loop(void) {
  uint8_t in_buf[BAB_MAX_PACKET_SIZE];
  int packet_sz = module.getPacket(in_buf);
  
  if (packet_sz > 0) {
    switch (in_buf[0]) {
      case BAB_CMD_DISPLAY_COLOR:
        screen.fillScreen(HX8357D::rgbTo565(in_buf[1], in_buf[2], in_buf[3]));
        break;
      case BAB_CMD_DISPLAY_FILE:
        bmpDraw((char*)&in_buf[1],0,0);
        break;
      default:
        Serial.println("Unknown command");
        break;
    }
  }  
  
  if (module.running()) {
    uint8_t out_buf[5] = {0};
    out_buf[0] = BAB_CMD_TACTILE_POS;
    uint16_t x = readX();
    uint16_t y = readY();
    memcpy(&out_buf[1],&x,sizeof(uint16_t));
    memcpy(&out_buf[3],&y,sizeof(uint16_t));
    module.sendPacket(out_buf,5);
  }

  delay(50);
}

