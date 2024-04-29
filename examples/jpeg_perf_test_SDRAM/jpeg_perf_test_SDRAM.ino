//
// Perf Test
//
#include <MemoryHexDump.h>
#include <JPEGDEC.h>
#include "../test_images/tulips.h" // 640x480 56k byte test image
JPEGDEC jpeg;
uint16_t *frameBufferSDRAM = (uint16_t *)((((uint32_t)(sdram_malloc(640 * 480 + 32)) + 32) & 0xffffffe0));
//uint8_t* frameBufferSDRAM = (uint8_t*)sdram_malloc(640 * 480 * sizeof(uint8_t));

int JPEGDraw(JPEGDRAW *pDraw)
{
  // do nothing
  return 1; // continue decode
} /* JPEGDraw() */

uint32_t bytesRead = 0;

void setup() {
  Serial.begin(115200);
  delay(2000); // allow time for Serial to start


  bytesRead = sizeof(tulips) / (sizeof *tulips);
  Serial.printf("BYTES READ: %d\n", bytesRead);

  uint32_t words = 0;
  //memcpy(frameBufferSDRAM, tulips, sizeof(tulips));
  for( uint32_t i = 0; i < bytesRead; i+=2) {
    frameBufferSDRAM[words] = ((uint16_t)tulips[i+1] << 8) | tulips[i];
    //Serial.printf("%x, %x, %x\n", tulips[i],tulips[i+1], frameBufferSDRAM[words] );
    words += 1;
  }
  Serial.printf("Words: %d\n", words);
  Serial.printf("First Word in frame: %x\n", frameBufferSDRAM[0]);
  Serial.printf("Last Word in frame: %x\n", frameBufferSDRAM[words-1]);

  // verify that the start of data returned has valid marker... assumes always in first buffer
  uint8_t *pfb = (uint8_t *)frameBufferSDRAM;
  if (1) MemoryHexDump(Serial, frameBufferSDRAM, 128, true, "SOF:\n");
  if ((pfb[0] != 0xff) || (pfb[1] != 0xd8) || (pfb[2] != 0xff)) {
    if (1) Serial.printf("begining of frame not found at position 0\n");
  }

  // Now lets try to verify the returned size for the end marker.
  //if (bytesRead < sizeof(frameBufferSDRAM)) {
  if (1) MemoryHexDump(Serial, pfb + bytesRead - 63, 64, true, "\nEOF:\n");
    if ((pfb[bytesRead - 2] != 0xFF) || (pfb[bytesRead- 1] != 0xd9)) {
      if (1) Serial.printf("Invalid frame ending: %02x %02x\n", pfb[bytesRead - 2], pfb[bytesRead - 1]);
    }
   Serial.printf("Last byte in Framebuffer: %x\n", pfb[bytesRead]);
  //} 
} /* setup() */

void loop() {
long lTime;

  Serial.println("FLash TEST");
  if (jpeg.openFLASH((uint8_t *)tulips, sizeof(tulips), JPEGDraw)) {
    lTime = micros();
    if (jpeg.decode(0,0,0)) { // full sized decode
      lTime = micros() - lTime;
      Serial.printf("full sized decode in %d us\n", (int)lTime);
    }
    jpeg.close();
  }
  if (jpeg.openFLASH((uint8_t *)tulips, sizeof(tulips), JPEGDraw)) {
    lTime = micros();
    if (jpeg.decode(0,0,JPEG_SCALE_HALF)) { // 1/2 sized decode
      lTime = micros() - lTime;
      Serial.printf("half sized decode in %d us\n", (int)lTime);
    }
    jpeg.close();
  }
  if (jpeg.openFLASH((uint8_t *)tulips, sizeof(tulips), JPEGDraw)) {
    lTime = micros();
    if (jpeg.decode(0,0,JPEG_SCALE_QUARTER)) { // 1/4 sized decode
      lTime = micros() - lTime;
      Serial.printf("quarter sized decode in %d us\n", (int)lTime);
    }
    jpeg.close();
  }
  if (jpeg.openFLASH((uint8_t *)tulips, sizeof(tulips), JPEGDraw)) {
    lTime = micros();
    if (jpeg.decode(0,0,JPEG_SCALE_EIGHTH)) { // 1/8 sized decode
      lTime = micros() - lTime;
      Serial.printf("eighth sized decode in %d us\n", (int)lTime);
    }
    jpeg.close();
  }

  Serial.println("SDRAM TEST");

  if (jpeg.openRAM((uint8_t *)frameBufferSDRAM, bytesRead, JPEGDraw)) {
    lTime = micros();
    if (jpeg.decode(0,0,0)) { // full sized decode
      lTime = micros() - lTime;
      Serial.printf("full sized decode in %d us\n", (int)lTime);
    } else {
      Serial.println("Failed to open Framebuffer!!");
    }
    jpeg.close();
  }
  if (jpeg.openRAM((uint8_t *)frameBufferSDRAM, bytesRead, JPEGDraw)) {
    lTime = micros();
    if (jpeg.decode(0,0,JPEG_SCALE_HALF)) { // 1/2 sized decode
      lTime = micros() - lTime;
      Serial.printf("half sized decode in %d us\n", (int)lTime);
    } else {
      Serial.println("Failed to open Framebuffer!!");
    }
    jpeg.close();
  }
  if (jpeg.openRAM((uint8_t *)frameBufferSDRAM, bytesRead, JPEGDraw)) {
    lTime = micros();
    if (jpeg.decode(0,0,JPEG_SCALE_QUARTER)) { // 1/4 sized decode
      lTime = micros() - lTime;
      Serial.printf("quarter sized decode in %d us\n", (int)lTime);
    } else {
      Serial.println("Failed to open Framebuffer!!");
    }
    jpeg.close();
  }
  if (jpeg.openRAM((uint8_t *)frameBufferSDRAM, bytesRead, JPEGDraw)) {
    lTime = micros();
    if (jpeg.decode(0,0,JPEG_SCALE_EIGHTH)) { // 1/8 sized decode
      lTime = micros() - lTime;
      Serial.printf("eighth sized decode in %d us\n", (int)lTime);
    } else {
      Serial.println("Failed to open Framebuffer!!");
    }
    jpeg.close();
  }

  delay(5000);
} /* loop() */
