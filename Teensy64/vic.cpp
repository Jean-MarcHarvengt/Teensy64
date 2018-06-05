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

/*
  TODOs:
  - Fix Bugs..
  - FLD  - (OK 08/17) test this more..
  - Sprite Stretching (requires "MOBcounter")
  - BA Signal -> CPU
  - xFLI
  - ...
  - DMA Delay (?) - needs partial rewrite (idle - > badline in middle of line. Is the 3.6 fast enough??)
  - optimize more
*/

#include "Teensy64.h"
#include "vic.h"


void vic_do(void) {
#if VGA & !VGATFT
  vic_do8();
#elif VGA & VGATFT  
  if (vgaMode) {
    vic_do8();
  }
  else {
    vic_do16();
  }
#else  
  vic_do16();
#endif  
}
 
void vic_displaySimpleModeScreen(void) {
#if VGA & !VGATFT
  vic_displaySimpleModeScreen8();
#elif VGA & VGATFT  
  if (vgaMode) {
    vic_displaySimpleModeScreen8();
  }
  else {
    vic_displaySimpleModeScreen16();
  }
#else  
  vic_displaySimpleModeScreen16();
#endif  
}

void vic_do_simple(void) {
#if VGA & !VGATFT
  vic_do_simple8();
#elif VGA & VGATFT   
  if (vgaMode) {
    vic_do_simple8();
  }
  else {
    vic_do_simple16();
  }
#else
  vic_do_simple16();  
#endif  
}
 
 
/*****************************************************************************************************/
/*****************************************************************************************************/
/*****************************************************************************************************/

void vic_adrchange(void) {
  uint8_t r18 = cpu.vic.R[0x18];
  cpu.vic.videomatrix =  cpu.vic.bank + (unsigned)(r18 & 0xf0) * 64;

  unsigned charsetAddr = r18 & 0x0e;
  if  ((cpu.vic.bank & 0x4000) == 0) {
    if (charsetAddr == 0x04) cpu.vic.charsetPtrBase =  ((uint8_t *)&rom_characters);
    else if (charsetAddr == 0x06) cpu.vic.charsetPtrBase =  ((uint8_t *)&rom_characters) + 0x800;
    else
      cpu.vic.charsetPtrBase = &cpu.RAM[charsetAddr * 0x400 + cpu.vic.bank] ;
  } else
    cpu.vic.charsetPtrBase = &cpu.RAM[charsetAddr * 0x400 + cpu.vic.bank];

  cpu.vic.bitmapPtr = (uint8_t*) &cpu.RAM[cpu.vic.bank | ((r18 & 0x08) * 0x400)];
  if ((cpu.vic.R[0x11] & 0x60) == 0x60)  cpu.vic.bitmapPtr = (uint8_t*)((uintptr_t)cpu.vic.bitmapPtr & 0xf9ff);

}
/*****************************************************************************************************/
void vic_write(uint32_t address, uint8_t value) {

  address &= 0x3F;

  switch (address) {
    case 0x11 :
	  cpu.vic.R[address] = value;
      cpu.vic.intRasterLine = (cpu.vic.intRasterLine & 0xff) | ((((uint16_t) value) << 1) & 0x100);
      if (cpu.vic.rasterLine == 0x30 ) cpu.vic.denLatch |= value & 0x10;

      cpu.vic.badline = (cpu.vic.denLatch && (cpu.vic.rasterLine >= 0x30) && (cpu.vic.rasterLine <= 0xf7) && ( (cpu.vic.rasterLine & 0x07) == (value & 0x07)));

	  if (cpu.vic.badline) {
		cpu.vic.idle = 0;
	  }

	  vic_adrchange();

      break;
    case 0x12 :
      cpu.vic.intRasterLine = (cpu.vic.intRasterLine & 0x100) | value;
      cpu.vic.R[address] = value;
      break;
    case 0x18 :
      cpu.vic.R[address] = value;
      vic_adrchange();
      break;
    case 0x19 : //IRQs
      cpu.vic.R[0x19] &= (~value & 0x0f);
      break;
    case 0x1A : //IRQ Mask
      cpu.vic.R[address] = value & 0x0f;
      break;
    case 0x1e:
    case 0x1f:
      cpu.vic.R[address] = 0;
      break;
    case 0x20 ... 0x2E:
      cpu.vic.R[address] = value & 0x0f;
      cpu.vic.colors[address - 0x20] = cpu.vic.palette[value & 0x0f];
      break;
    case 0x2F ... 0x3F:
      break;
    default :
      cpu.vic.R[address] = value;
      break;
  }

  //#if DEBUGVIC
#if 0
  Serial.print("VIC ");
  Serial.print(address, HEX);
  Serial.print("=");
  Serial.println(value, HEX);
  //logAddr(address, value, 1);
#endif
}

/*****************************************************************************************************/
/*****************************************************************************************************/
/*****************************************************************************************************/

uint8_t vic_read(uint32_t address) {
  uint8_t ret;

  address &= 0x3F;
  switch (address) {

    case 0x11:
      ret = (cpu.vic.R[address] & 0x7F) | ((cpu.vic.rasterLine & 0x100) >> 1);
      break;
    case 0x12:
      ret = cpu.vic.rasterLine;
      break;
    case 0x16:
      ret = cpu.vic.R[address] | 0xC0;
      break;
    case 0x18:
      ret = cpu.vic.R[address] | 0x01;
      break;
    case 0x19:
      ret = cpu.vic.R[address] | 0x70;
      break;
    case 0x1a:
      ret = cpu.vic.R[address] | 0xF0;
      break;
    case 0x1e:
    case 0x1f:
      ret = cpu.vic.R[address];
      cpu.vic.R[address] = 0;
      break;
    case 0x20 ... 0x2E:
      ret = cpu.vic.R[address] | 0xF0;
      break;
    case 0x2F ... 0x3F:
      ret = 0xFF;
      break;
    default:
      ret = cpu.vic.R[address];
      break;
  }

#if DEBUGVIC
  Serial.print("VIC ");
  logAddr(address, ret, 0);
#endif
  return ret;
}

/*****************************************************************************************************/
/*****************************************************************************************************/
/*****************************************************************************************************/

void resetVic(void) {
  enableCycleCounter();

  cpu.vic.intRasterLine = 0;
  cpu.vic.rasterLine = 0;
  cpu.vic.lineHasSprites = 0;
  memset(&cpu.RAM[0x400], 0, 1000);
  memset(&cpu.vic, 0, sizeof(cpu.vic));
  

#if VGA & !VGATFT
  installPalette8();
#elif VGA & VGATFT  
  if (vgaMode) {
    installPalette8();
  }
  else {
    installPalette16();
  }
#else
  installPalette16();  
#endif

  //http://dustlayer.com/vic-ii/2013/4/22/when-visibility-matters
  cpu.vic.R[0x11] = 0x9B;
  cpu.vic.R[0x16] = 0x08;
  cpu.vic.R[0x18] = 0x14;
  cpu.vic.R[0x19] = 0x0f;

  for (unsigned i = 0; i < sizeof(cpu.vic.COLORRAM); i++)
    cpu.vic.COLORRAM[i] = (rand() & 0x0F);

  cpu.RAM[0x39FF] = 0x0;
  cpu.RAM[0x3FFF] = 0x0;
  cpu.RAM[0x39FF + 16384] = 0x0;
  cpu.RAM[0x3FFF + 16384] = 0x0;
  cpu.RAM[0x39FF + 32768] = 0x0;
  cpu.RAM[0x3FFF + 32768] = 0x0;
  cpu.RAM[0x39FF + 49152] = 0x0;
  cpu.RAM[0x3FFF + 49152] = 0x0;

  vic_adrchange();
}


/*
  ?PEEK(678) NTSC =0
  ?PEEK(678) PAL = 1
  PRINT TIME$
*/
/*
          Raster-  Takt-   sichtb.  sichtbare
  VIC-II  System  zeilen   zyklen  Zeilen   Pixel/Zeile
  -------------------------------------------------------
  6569    PAL    312     63    284     403
  6567R8  NTSC   263     65    235     418
  6567R56A  NTSC   262   64    234     411
*/
