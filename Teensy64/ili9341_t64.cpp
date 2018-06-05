/*
  Copyright Frank Bösing, 2017

  This file is part of Teensy64.

    Teensy64 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Teensy64 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Teensy64.  If not, see <http://www.gnu.org/licenses/>.

    Diese Datei ist Teil von Teensy64.

    Teensy64 ist Freie Software: Sie können es unter den Bedingungen
    der GNU General Public License, wie von der Free Software Foundation,
    Version 3 der Lizenz oder (nach Ihrer Wahl) jeder späteren
    veröffentlichten Version, weiterverbreiten und/oder modifizieren.

    Teensy64 wird in der Hoffnung, dass es nützlich sein wird, aber
    OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite
    Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
    Siehe die GNU General Public License für weitere Details.

    Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
    Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>.

*/

#include "ili9341_t64.h"
#include "font8x8.h"

#define SPICLOCK 144e6 //Just a number..max speed

// touch
#define SPI_SETTING         SPISettings(2500000, MSBFIRST, SPI_MODE0)
#define XPT2046_CFG_START   _BV(7)
#define XPT2046_CFG_MUX(v)  ((v&0b111) << (4))
#define XPT2046_CFG_8BIT    _BV(3)
#define XPT2046_CFG_12BIT   (0)
#define XPT2046_CFG_SER     _BV(2)
#define XPT2046_CFG_DFR     (0)
#define XPT2046_CFG_PWR(v)  ((v&0b11))
#define XPT2046_MUX_Y       0b101
#define XPT2046_MUX_X       0b001
#define XPT2046_MUX_Z1      0b011
#define XPT2046_MUX_Z2      0b100

//const int dma_linestarts[SCREEN_DMA_NUM_SETTINGS] =

DMAMEM uint16_t screen[ILI9341_TFTHEIGHT][ILI9341_TFTWIDTH];

uint16_t * screen16 = (uint16_t*)&screen[0][0];
uint32_t * screen32 = (uint32_t*)&screen[0][0];
const uint32_t * screen32e = (uint32_t*)&screen[0][0] + sizeof(screen) / 4;

DMAChannel dmatx;
volatile uint8_t rstop = 0;
volatile bool cancelled = false;
volatile uint8_t ntransfer = 0;


static const uint8_t init_commands[] = {
  4, 0xEF, 0x03, 0x80, 0x02,
  4, 0xCF, 0x00, 0XC1, 0X30,
  5, 0xED, 0x64, 0x03, 0X12, 0X81,
  4, 0xE8, 0x85, 0x00, 0x78,
  6, 0xCB, 0x39, 0x2C, 0x00, 0x34, 0x02,
  2, 0xF7, 0x20,
  3, 0xEA, 0x00, 0x00,
  2, ILI9341_PWCTR1, 0x23, // Power control
  2, ILI9341_PWCTR2, 0x10, // Power control
  3, ILI9341_VMCTR1, 0x3e, 0x28, // VCM control
  2, ILI9341_VMCTR2, 0x86, // VCM control2
  2, ILI9341_MADCTL, 0x48, // Memory Access Control
  2, ILI9341_PIXFMT, 0x55,
  3, ILI9341_FRMCTR1, 0x00, 0x18,
  4, ILI9341_DFUNCTR, 0x08, 0x82, 0x27, // Display Function Control
  2, 0xF2, 0x00, // Gamma Function Disable
  2, ILI9341_GAMMASET, 0x01, // Gamma curve selected
  16, ILI9341_GMCTRP1, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08,
  0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00, // Set Gamma
  16, ILI9341_GMCTRN1, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07,
  0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F, // Set Gamma
//  3, 0xb1, 0x00, 0x1f, // FrameRate Control 61Hz
  3, 0xb1, 0x00, 0x10, // FrameRate Control 119Hz
  2, ILI9341_MADCTL, MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR,
  0
};


static void dmaInterrupt() {
  dmatx.clearInterrupt();
  ntransfer++;
  if (ntransfer >= SCREEN_DMA_NUM_SETTINGS) {   
    ntransfer = 0;
    if (cancelled) {
        dmatx.disable();
        rstop = 1;
    }
  }
}


ILI9341_t3DMA::ILI9341_t3DMA(uint8_t cs, uint8_t dc, uint8_t rst, uint8_t mosi, uint8_t sclk, uint8_t miso,  uint8_t touch_cs,  uint8_t touch_irq)
{
  _cs   = cs;
  _dc   = dc;
  _rst  = rst;
  _mosi = mosi;
  _sclk = sclk;
  _miso = miso;
  _touch_irq = touch_irq;
  _touch_cs = touch_cs;
  pinMode(_dc, OUTPUT);
  pinMode(_cs, OUTPUT);
  pinMode(_touch_cs, OUTPUT);
  pinMode(touch_irq, INPUT_PULLUP);  
  digitalWrite(_cs, 1);
  digitalWrite(_dc, 1);
  digitalWrite(_touch_cs, 1);
}


void ILI9341_t3DMA::setArea(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2) {
  SPI.beginTransaction(SPISettings(SPICLOCK, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, 0);

  digitalWrite(_dc, 0);
  SPI.transfer(ILI9341_CASET);
  digitalWrite(_dc, 1);
  SPI.transfer16(x1);
  SPI.transfer16(x2);

  digitalWrite(_dc, 0);
  SPI.transfer(ILI9341_PASET);
  digitalWrite(_dc, 1);
  SPI.transfer16(y1);
  SPI.transfer16(y2);

  digitalWrite(_dc, 0);
  SPI.transfer(ILI9341_RAMWR);
  digitalWrite(_dc, 1);
  
  digitalWrite(_cs, 1);
}


void ILI9341_t3DMA::begin(void) {

  SPI.setMOSI(_mosi);
  SPI.setMISO(_miso);
  SPI.setSCK(_sclk);
  SPI.begin();
      
  // Initialize display
  if (_rst < 255) { // toggle RST low to reset
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, HIGH);
    delay(5);
    digitalWrite(_rst, LOW);
    delay(20);
    digitalWrite(_rst, HIGH);
    delay(120);
  }
  
  SPI.beginTransaction(SPISettings(SPICLOCK, MSBFIRST, SPI_MODE0));
  const uint8_t *addr = init_commands;

  digitalWrite(_cs, 0);

  while (1) {
    uint8_t count = *addr++;
    if (count-- == 0) break;

    digitalWrite(_dc, 0);
    SPI.transfer(*addr++);

    while (count-- > 0) {
      digitalWrite(_dc, 1);
      SPI.transfer(*addr++);
    }
  }
  
  digitalWrite(_dc, 0);
  SPI.transfer(ILI9341_SLPOUT);
  digitalWrite(_dc, 1);
  digitalWrite(_cs, 1);
  SPI.endTransaction();

  digitalWrite(_dc, 1);
  digitalWrite(_cs, 1);
  SPI.endTransaction();

  SPI.beginTransaction(SPISettings(SPICLOCK, MSBFIRST, SPI_MODE0));
  digitalWrite(_dc, 0);
  digitalWrite(_cs, 0);
  SPI.transfer(ILI9341_DISPON);
  digitalWrite(_dc, 1);
  digitalWrite(_cs, 1);
  SPI.endTransaction();

  const uint32_t bytesPerLine = ILI9341_TFTWIDTH * 2;
  const uint32_t maxLines = (SCREEN_DMA_MAX_SIZE / bytesPerLine);
  uint32_t i = 0, sum = 0, lines;
  do {

    //Source:
    lines = min(maxLines, ILI9341_TFTHEIGHT - sum);
    int32_t len = lines * bytesPerLine;
    dmasettings[i].TCD->CSR = 0;
    dmasettings[i].TCD->SADDR = &screen[sum][0];

    dmasettings[i].TCD->SOFF = 2;
    dmasettings[i].TCD->ATTR_SRC = 1;
    dmasettings[i].TCD->NBYTES = 2;
    dmasettings[i].TCD->SLAST = -len;
    dmasettings[i].TCD->BITER = len / 2;
    dmasettings[i].TCD->CITER = len / 2;

    //Destination:
    dmasettings[i].TCD->DADDR = &SPI0_PUSHR;
    dmasettings[i].TCD->DOFF = 0;
    dmasettings[i].TCD->ATTR_DST = 1;
    dmasettings[i].TCD->DLASTSGA = 0;

    dmasettings[i].replaceSettingsOnCompletion(dmasettings[i + 1]);
    dmasettings[i].interruptAtCompletion();
    //dmasettings[i].disableOnCompletion();
    sum += lines;
  } while (++i < SCREEN_DMA_NUM_SETTINGS);

  dmasettings[SCREEN_DMA_NUM_SETTINGS - 1].replaceSettingsOnCompletion(dmasettings[0]);
  dmasettings[SCREEN_DMA_NUM_SETTINGS - 1].interruptAtCompletion(); 
  //dmasettings[SCREEN_DMA_NUM_SETTINGS - 1].disableOnCompletion();

  dmatx.attachInterrupt(dmaInterrupt);
   
  dmatx.begin(false);
  dmatx.triggerAtHardwareEvent(DMAMUX_SOURCE_SPI0_TX );
  dmatx = dmasettings[0];
  cancelled = false; 
};

void ILI9341_t3DMA::flipscreen(bool flip)
{

  SPI.beginTransaction(SPISettings(SPICLOCK, MSBFIRST, SPI_MODE0));
  digitalWrite(_dc, 0);
  digitalWrite(_cs, 0);
  SPI.transfer(ILI9341_MADCTL);
  digitalWrite(_dc, 1);
  if (flip) {
    flipped=true;    
    SPI.transfer(MADCTL_MV | MADCTL_BGR);
  }
  else {
    flipped=false;   
    SPI.transfer(MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
  }
  digitalWrite(_cs, 1);  
  SPI.endTransaction();
}


void ILI9341_t3DMA::start(void) {
  setArea(0, 0, max_screen_width, max_screen_height);
  SPI0_RSER |= SPI_RSER_TFFF_DIRS | SPI_RSER_TFFF_RE;  // Set ILI_DMA Interrupt Request Select and Enable register
  SPI0_MCR &= ~SPI_MCR_HALT;  //Start transfers.
  SPI0_CTAR0 = SPI0_CTAR1;
  (*(volatile uint16_t *)((int)&SPI0_PUSHR + 2)) = (SPI_PUSHR_CTAS(1) | SPI_PUSHR_CONT) >> 16; //Enable 16 Bit Transfers + Continue-Bit
}


void ILI9341_t3DMA::refresh(void) {
  start();
  digitalWrite(_cs, 0);  
  dmasettings[SCREEN_DMA_NUM_SETTINGS - 1].TCD->CSR &= ~DMA_TCD_CSR_DREQ; //disable "disableOnCompletion"
  dmatx.enable();
  ntransfer = 0;  
  dmatx = dmasettings[0];
  rstop = 0;  
}


void ILI9341_t3DMA::stop(void) {
  rstop = 0;
  wait();
  delay(50);
  //dmatx.disable();
  SPI.beginTransaction(SPISettings(SPICLOCK, MSBFIRST, SPI_MODE0));  
  SPI.endTransaction();
  digitalWrite(_cs, 1);
  digitalWrite(_dc, 1);     
}

void ILI9341_t3DMA::wait(void) {
  rstop = 1;
  unsigned long m = millis(); 
  cancelled = true; 
  while (!rstop)  {
    if ((millis() - m) > 100) break;
    delay(10);
    asm volatile("wfi");
  };
  rstop = 0;
}


/***********************************************************************************************
    Touch functions
 ***********************************************************************************************/
/* Code based on ...
 *
 * @file XPT2046.cpp
 * @date 19.02.2016
 * @author Markus Sattler
 *
 * Copyright (c) 2015 Markus Sattler. All rights reserved.
 * This file is part of the XPT2046 driver for Arduino.
 */

#define ADC_MAX                 0x0fff  

void ILI9341_t3DMA::enableTouchIrq() {
  SPI.beginTransaction(SPI_SETTING);
  digitalWrite(_touch_cs, LOW);
  const uint8_t buf[4] = { (XPT2046_CFG_START | XPT2046_CFG_12BIT | XPT2046_CFG_DFR | XPT2046_CFG_MUX(XPT2046_MUX_Y)), 0x00, 0x00, 0x00 };
  SPI.transfer((void*)&buf[0],3);   
  digitalWrite(_touch_cs, HIGH);
  SPI.endTransaction();
}
//Callibration for non flipped
#define TX_MIN 30
#define TY_MIN 20
#define TX_MAX 300
#define TY_MAX 220
//Callibration for flipped
#define TFX_MIN 20
#define TFY_MIN 25
#define TFX_MAX 288
#define TFY_MAX 221

void ILI9341_t3DMA::readRaw(uint16_t * oX, uint16_t * oY, uint16_t * oZ) {
  uint16_t x = 0;
  uint16_t y = 0;
  uint16_t z1 = 0;
  uint16_t z2 = 0;
  uint8_t i = 0;
  int16_t xraw=0, yraw=0;
  SPI.beginTransaction(SPI_SETTING);
  digitalWrite(_touch_cs, LOW);

  for(; i < 15; i++) {
    // SPI requirer 32bit aliment
    uint8_t buf[12] = {
      (XPT2046_CFG_START | XPT2046_CFG_12BIT | XPT2046_CFG_DFR | XPT2046_CFG_MUX(XPT2046_MUX_Y) | XPT2046_CFG_PWR(3)), 0x00, 0x00,
      (XPT2046_CFG_START | XPT2046_CFG_12BIT | XPT2046_CFG_DFR | XPT2046_CFG_MUX(XPT2046_MUX_X) | XPT2046_CFG_PWR(3)), 0x00, 0x00,
      (XPT2046_CFG_START | XPT2046_CFG_12BIT | XPT2046_CFG_DFR | XPT2046_CFG_MUX(XPT2046_MUX_Z1)| XPT2046_CFG_PWR(3)), 0x00, 0x00,
      (XPT2046_CFG_START | XPT2046_CFG_12BIT | XPT2046_CFG_DFR | XPT2046_CFG_MUX(XPT2046_MUX_Z2)| XPT2046_CFG_PWR(3)), 0x00, 0x00
    };
    SPI.transfer(&buf[0], &buf[0], 12);
    y += (buf[1] << 8 | buf[2])>>3;
    x += (buf[4] << 8 | buf[5])>>3;
    z1 += (buf[7] << 8 | buf[8])>>3;
    z2 += (buf[10] << 8 | buf[11])>>3;
  }

  enableTouchIrq();

  if(i == 0) {
      *oX = 0;
      *oY = 0;
      *oZ = 0;
  }
  else {
      x /= i;
      y /= i;
      z1 /= i;
      z2 /= i;
  }

  digitalWrite(_touch_cs, HIGH);
  SPI.endTransaction();
  int z = z1 + ADC_MAX - z2;
  if (flipped) {
    xraw = x;
    yraw = y;
  } else {
    xraw = ADC_MAX - x;
    yraw = ADC_MAX - y;
  }
  // callibrate ...
  xraw=(xraw*320)/(ADC_MAX+1);
  yraw=(yraw*240)/(ADC_MAX+1);
// debug for calibration (top/left and bottom/right) must be adjusted as XMIN/YMIN and XMAX/YMAX
//Serial.print("x:");
//Serial.print(xraw);
//Serial.print(" ,y:");
//Serial.println(yraw);  
  if (flipped) {
    if(xraw >= TFX_MIN) xraw = ((xraw - TFX_MIN)*320)/(TFX_MAX-TFX_MIN);
    if(yraw >= TFY_MIN) yraw = ((yraw - TFY_MIN)*240)/(TFY_MAX-TFY_MIN);
Serial.print("x:");
Serial.print(xraw);
Serial.print(" ,y:");
Serial.println(yraw);  

  }
  else {
    if(xraw >= TX_MIN) xraw = ((xraw - TX_MIN)*320)/(TX_MAX-TX_MIN);
    if(yraw >= TY_MIN) yraw = ((yraw - TY_MIN)*240)/(TY_MAX-TY_MIN);
  }

  *oX = xraw;
  *oY = yraw;
  *oZ = z;
}



/***********************************************************************************************
    no DMA functions
 ***********************************************************************************************/
void ILI9341_t3DMA::fillScreenNoDma(uint16_t color) {
  setArea(0, 0, ILI9341_TFTWIDTH-1, 239);  
  
  SPI.beginTransaction(SPISettings(SPICLOCK, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, 0);
  digitalWrite(_dc, 0);
  SPI.transfer(ILI9341_RAMWR);
  int i,j;
  for (j=0; j<240; j++)
  {
    for (i=0; i<ILI9341_TFTWIDTH; i++) {
      digitalWrite(_dc, 1);
      SPI.transfer16(color);     
    }
  }
  digitalWrite(_dc, 0);
  SPI.transfer(ILI9341_SLPOUT);
  digitalWrite(_dc, 1);
  digitalWrite(_cs, 1);
  SPI.endTransaction();  
  
  setArea(0, 0, max_screen_width, max_screen_height);  
}


void ILI9341_t3DMA::writeScreenNoDma(const uint16_t *pcolors) {
  setArea(0, 0, ILI9341_TFTWIDTH-1, 239);  
  
  SPI.beginTransaction(SPISettings(SPICLOCK, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, 0);
  digitalWrite(_dc, 0);
  SPI.transfer(ILI9341_RAMWR);
  int i,j;
  for (j=0; j<240; j++)
  {
    for (i=0; i<ILI9341_TFTWIDTH; i++) {
      digitalWrite(_dc, 1);
      SPI.transfer16(*pcolors++);     
    }
  }
  digitalWrite(_dc, 0);
  SPI.transfer(ILI9341_SLPOUT);
  digitalWrite(_dc, 1);
  digitalWrite(_cs, 1);
  SPI.endTransaction();  
  
  setArea(0, 0, max_screen_width, max_screen_height);  
}

void ILI9341_t3DMA::drawSpriteNoDma(int16_t x, int16_t y, const uint16_t *bitmap) {
    drawSpriteNoDma(x,y,bitmap, 0,0,0,0);
}

void ILI9341_t3DMA::drawSpriteNoDma(int16_t x, int16_t y, const uint16_t *bitmap, uint16_t arx, uint16_t ary, uint16_t arw, uint16_t arh)
{
  int bmp_offx = 0;
  int bmp_offy = 0;
  uint16_t *bmp_ptr;
    
  int w =*bitmap++;
  int h = *bitmap++;

  if ( (arw == 0) || (arh == 0) ) {
    // no crop window
    arx = x;
    ary = y;
    arw = w;
    arh = h;
  }
  else {
    if ( (x>(arx+arw)) || ((x+w)<arx) || (y>(ary+arh)) || ((y+h)<ary)   ) {
      return;
    }
    
    // crop area
    if ( (x > arx) && (x<(arx+arw)) ) { 
      arw = arw - (x-arx);
      arx = arx + (x-arx);
    } else {
      bmp_offx = arx;
    }
    if ( ((x+w) > arx) && ((x+w)<(arx+arw)) ) {
      arw -= (arx+arw-x-w);
    }  
    if ( (y > ary) && (y<(ary+arh)) ) {
      arh = arh - (y-ary);
      ary = ary + (y-ary);
    } else {
      bmp_offy = ary;
    }
    if ( ((y+h) > ary) && ((y+h)<(ary+arh)) ) {
      arh -= (ary+arh-y-h);
    }     
  }

  setArea(arx, ary, arx+arw-1, ary+arh-1);  
  
  SPI.beginTransaction(SPISettings(SPICLOCK, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, 0);
  digitalWrite(_dc, 0);
  SPI.transfer(ILI9341_RAMWR);      

  bitmap = bitmap + bmp_offy*w + bmp_offx;
  for (int row=0;row<arh; row++)
  {
    bmp_ptr = (uint16_t*)bitmap;
    for (int col=0;col<arw; col++)
    {
        uint16_t color = *bmp_ptr++;
        digitalWrite(_dc, 1);
        SPI.transfer16(color);             
    } 
    bitmap +=  w;
  }
  digitalWrite(_dc, 0);
  SPI.transfer(ILI9341_SLPOUT);
  digitalWrite(_dc, 1);
  digitalWrite(_cs, 1);
  SPI.endTransaction();   
  setArea(0, 0, ILI9341_TFTWIDTH-1, 239);  
}

void ILI9341_t3DMA::drawTextNoDma(int16_t x, int16_t y, const char * text, uint16_t fgcolor, uint16_t bgcolor, bool doublesize) {
  uint16_t c;
  while ((c = *text++)) {
    const unsigned char * charpt=&font8x8[c][0];

    setArea(x,y,x+7,y+(doublesize?15:7));
  
    //SPI.beginTransaction(SPISettings(SPICLOCK, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, 0);
    //digitalWrite(_dc, 0);
    //SPI.transfer(ILI9341_RAMWR);

    digitalWrite(_dc, 1);
    for (int i=0;i<8;i++)
    {
      unsigned char bits;
      if (doublesize) {
        bits = *charpt;     
        digitalWrite(_dc, 1);
        if (bits&0x01) SPI.transfer16(fgcolor);
        else SPI.transfer16(bgcolor);
        bits = bits >> 1;     
        if (bits&0x01) SPI.transfer16(fgcolor);
        else SPI.transfer16(bgcolor);
        bits = bits >> 1;     
        if (bits&0x01) SPI.transfer16(fgcolor);
        else SPI.transfer16(bgcolor);
        bits = bits >> 1;     
        if (bits&0x01) SPI.transfer16(fgcolor);
        else SPI.transfer16(bgcolor);
        bits = bits >> 1;     
        if (bits&0x01) SPI.transfer16(fgcolor);
        else SPI.transfer16(bgcolor);
        bits = bits >> 1;     
        if (bits&0x01) SPI.transfer16(fgcolor);
        else SPI.transfer16(bgcolor);
        bits = bits >> 1;     
        if (bits&0x01) SPI.transfer16(fgcolor);
        else SPI.transfer16(bgcolor);
        bits = bits >> 1;     
        if (bits&0x01) SPI.transfer16(fgcolor);
        else SPI.transfer16(bgcolor);       
      }
      bits = *charpt++;     
      digitalWrite(_dc, 1);
      if (bits&0x01) SPI.transfer16(fgcolor);
      else SPI.transfer16(bgcolor);
      bits = bits >> 1;     
      if (bits&0x01) SPI.transfer16(fgcolor);
      else SPI.transfer16(bgcolor);
      bits = bits >> 1;     
      if (bits&0x01) SPI.transfer16(fgcolor);
      else SPI.transfer16(bgcolor);
      bits = bits >> 1;     
      if (bits&0x01) SPI.transfer16(fgcolor);
      else SPI.transfer16(bgcolor);
      bits = bits >> 1;     
      if (bits&0x01) SPI.transfer16(fgcolor);
      else SPI.transfer16(bgcolor);
      bits = bits >> 1;     
      if (bits&0x01) SPI.transfer16(fgcolor);
      else SPI.transfer16(bgcolor);
      bits = bits >> 1;     
      if (bits&0x01) SPI.transfer16(fgcolor);
      else SPI.transfer16(bgcolor);
      bits = bits >> 1;     
      if (bits&0x01) SPI.transfer16(fgcolor);
      else SPI.transfer16(bgcolor);
    }
    x +=8;
  
    digitalWrite(_dc, 0);
    SPI.transfer(ILI9341_SLPOUT);
    digitalWrite(_dc, 1);
    digitalWrite(_cs, 1);
    SPI.endTransaction();  
  }
  
  setArea(0, 0, max_screen_width, max_screen_height);  
}


void ILI9341_t3DMA::drawRectNoDma(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  setArea(x,y,x+w-1,y+h-1);
  SPI.beginTransaction(SPISettings(SPICLOCK, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, 0);
  digitalWrite(_dc, 0);
  SPI.transfer(ILI9341_RAMWR);
  int i;
  for (i=0; i<(w*h); i++)
  {
    digitalWrite(_dc, 1);
    SPI.transfer16(color);
  }
  digitalWrite(_dc, 0);
  SPI.transfer(ILI9341_SLPOUT);
  digitalWrite(_dc, 1);
  digitalWrite(_cs, 1);
  SPI.endTransaction();  
  
  setArea(0, 0, max_screen_width, max_screen_height);
}



/***********************************************************************************************
    DMA functions
 ***********************************************************************************************/

void ILI9341_t3DMA::fillScreen(uint16_t color) {
  uint32_t col32 = (color << 16) | color;
  uint32_t * p = screen32;
  do {
    *p++ = col32;
  } while (p < screen32e);
}

void ILI9341_t3DMA::writeScreen(const uint16_t *pcolors) {
  memcpy(&screen, pcolors, sizeof(screen));
}

void ILI9341_t3DMA::drawPixel(int16_t x, int16_t y, uint16_t color) {
  screen[y][x] = color;
}

inline uint16_t ILI9341_t3DMA::getPixel(int16_t x, int16_t y) {
  return screen[y][x];
}


void ILI9341_t3DMA::dim()
{
    int p;
	uint8_t r, g, b;

	for (int x = 0; x < ILI9341_TFTWIDTH; x++) {
		for (int y = 0; y < ILI9341_TFTHEIGHT; y++) {
			p = getPixel(x,y);
			color565toRGB(p, r,g,b);			
			drawPixel(x,y, color565(r>>2, g>>2, b>>2));
		}
	}
	
}
/*******************************************************************************************************************/
/*FONTS*************************************************************************************************************/
/*******************************************************************************************************************/

static uint32_t fetchbit(const uint8_t *p, uint32_t index)
{
	if (p[index >> 3] & (1 << (7 - (index & 7)))) return 1;
	return 0;
}

static uint32_t fetchbits_unsigned(const uint8_t *p, uint32_t index, uint32_t required)
{
	uint32_t val = 0;
	do {
		uint8_t b = p[index >> 3];
		uint32_t avail = 8 - (index & 7);
		if (avail <= required) {
			val <<= avail;
			val |= b & ((1 << avail) - 1);
			index += avail;
			required -= avail;
		} else {
			b >>= avail - required;
			val <<= required;
			val |= b & ((1 << required) - 1);
			break;
		}
	} while (required);
	return val;
}

static uint32_t fetchbits_signed(const uint8_t *p, uint32_t index, uint32_t required)
{
	uint32_t val = fetchbits_unsigned(p, index, required);
	if (val & (1 << (required - 1))) {
		return (int32_t)val - (1 << required);
	}
	return (int32_t)val;
}


void ILI9341_t3DMA::drawFontChar(unsigned int c)
{
	uint32_t bitoffset;
	const uint8_t *data;

	//Serial.printf("drawFontChar %d\n", c);

	if (c >= font->index1_first && c <= font->index1_last) {
		bitoffset = c - font->index1_first;
		bitoffset *= font->bits_index;
	} else if (c >= font->index2_first && c <= font->index2_last) {
		bitoffset = c - font->index2_first + font->index1_last - font->index1_first + 1;
		bitoffset *= font->bits_index;
	} else if (font->unicode) {
		return; // TODO: implement sparse unicode
	} else {
		return;
	}
	//Serial.printf("  index =	%d\n", fetchbits_unsigned(font->index, bitoffset, font->bits_index));
	data = font->data + fetchbits_unsigned(font->index, bitoffset, font->bits_index);

	uint32_t encoding = fetchbits_unsigned(data, 0, 3);
	if (encoding != 0) return;
	uint32_t width = fetchbits_unsigned(data, 3, font->bits_width);
	bitoffset = font->bits_width + 3;
	uint32_t height = fetchbits_unsigned(data, bitoffset, font->bits_height);
	bitoffset += font->bits_height;
	//Serial.printf("  size =	%d,%d\n", width, height);

	int32_t xoffset = fetchbits_signed(data, bitoffset, font->bits_xoffset);
	bitoffset += font->bits_xoffset;
	int32_t yoffset = fetchbits_signed(data, bitoffset, font->bits_yoffset);
	bitoffset += font->bits_yoffset;
	//Serial.printf("  offset = %d,%d\n", xoffset, yoffset);

	uint32_t delta = fetchbits_unsigned(data, bitoffset, font->bits_delta);
	bitoffset += font->bits_delta;
	//Serial.printf("  delta =	%d\n", delta);

	//Serial.printf("  cursor = %d,%d\n", cursor_x, cursor_y);

	// horizontally, we draw every pixel, or none at all
	if (cursor_x < 0) cursor_x = 0;
	int32_t origin_x = cursor_x + xoffset;
	if (origin_x < 0) {
		cursor_x -= xoffset;
		origin_x = 0;
	}
	if (origin_x + (int)width > ILI9341_TFTWIDTH) {		
		origin_x = 0;
		if (xoffset >= 0) {
			cursor_x = 0;
		} else {
			cursor_x = -xoffset;
		}
		cursor_y += font->line_space;
	}
	if (cursor_y >= ILI9341_TFTHEIGHT) return;
	cursor_x += delta;

	// vertically, the top and/or bottom can be clipped
	int32_t origin_y = cursor_y + font->cap_height - height - yoffset;
	//Serial.printf("  origin = %d,%d\n", origin_x, origin_y);

	// TODO: compute top skip and number of lines
	int32_t linecount = height;
	//uint32_t loopcount = 0;
	uint32_t y = origin_y;
	while (linecount) {
		//Serial.printf("	 linecount = %d\n", linecount);
		uint32_t b = fetchbit(data, bitoffset++);
		if (b == 0) {
			//Serial.println("	  single line");
			uint32_t x = 0;
			do {
				uint32_t xsize = width - x;
				if (xsize > 32) xsize = 32;
				uint32_t bits = fetchbits_unsigned(data, bitoffset, xsize);
				drawFontBits(bits, xsize, origin_x + x, y, 1);
				bitoffset += xsize;
				x += xsize;
			} while (x < width);
			y++;
			linecount--;
		} else {
			uint32_t n = fetchbits_unsigned(data, bitoffset, 3) + 2;
			bitoffset += 3;
			uint32_t x = 0;
			do {
				uint32_t xsize = width - x;
				if (xsize > 32) xsize = 32;
				//Serial.printf("	 multi line %d\n", n);
				uint32_t bits = fetchbits_unsigned(data, bitoffset, xsize);
				drawFontBits(bits, xsize, origin_x + x, y, n);
				bitoffset += xsize;
				x += xsize;
			} while (x < width);
			y += n;
			linecount -= n;
		}
		//if (++loopcount > 100) {
			//Serial.println("	   abort draw loop");
			//break;
		//}
	}
}

void ILI9341_t3DMA::drawFontBits(uint32_t bits, uint32_t numbits, uint32_t x, uint32_t y, uint32_t repeat)
{
#if 1
	// TODO: replace this *slow* code with something fast...
	//Serial.printf("	   %d bits at %d,%d: %X\n", numbits, x, y, bits);
	if (bits == 0) return;
	do {
		uint32_t x1 = x;
		uint32_t n = numbits;
		do {
			n--;
			if (bits & (1 << n)) {
				drawPixel(x1, y, textcolor);
				//Serial.printf("		 pixel at %d,%d\n", x1, y);
			}
			x1++;
		} while (n > 0);
		y++;
		repeat--;
	} while (repeat);
#endif
#if 0
	if (bits == 0) return;
	int w = 0;
	do {
		uint32_t x1 = x;
		uint32_t n = numbits;
		writecommand_cont(ILI9341_PASET); // Row addr set
		writedata16_cont(y);   // YSTART
		writedata16_cont(y);   // YEND
		do {
			n--;
			if (bits & (1 << n)) {
				w++;
			}
			else if (w > 0) {
				// "drawFastHLine(x1 - w, y, w, textcolor)"
				writecommand_cont(ILI9341_CASET); // Column addr set
				writedata16_cont(x1 - w);	// XSTART
				writedata16_cont(x1);	// XEND
				writecommand_cont(ILI9341_RAMWR);
				while (w-- > 1) { // draw line
					writedata16_cont(textcolor);
				}
				writedata16_last(textcolor);
			}
			x1++;
		} while (n > 0);
		if (w > 0) {
				writecommand_cont(ILI9341_CASET); // Column addr set
				writedata16_cont(x1 - w);	// XSTART
				writedata16_cont(x1);	// XEND
				writecommand_cont(ILI9341_RAMWR);
				while (w-- > 1) { //draw line
					writedata16_cont(textcolor);
				}
				writedata16_last(textcolor);
		}
		y++;
		repeat--;
	} while (repeat);
#endif
}