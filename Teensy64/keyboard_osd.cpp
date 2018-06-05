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
#include <Arduino.h>
#include <core_pins.h>
#include "Teensy64.h"

#if VGA && !VGATFT
#else 

#include "logokbdstandard.h"
#include "logokbdshift.h"
#include "logoscrollbar.h"
#include "SdFat.h"

// files/directory
#define DIRECTORY "/C64/\0"
#define MAX_FILENAME_SIZE   20
#define MAX_DISPLAY_LINES   6
static File file;
static File entry;
static int nbFiles;
static int curFile;


// layout
#define KEYBOARD_X       0
#define KEYBOARD_W       320
#define ROW_KEYBOARD_H   23
#define TEXT_INPUT_Y     116
#define TEXT_INPUT_H     18
#define FILE_INPUT_Y     132
#define FILE_INPUT_H     108
#define FILE_INPUT_W     ((MAX_FILENAME_SIZE+1)*8+FILE_INPUT_SC_W)
#define FILE_INPUT_X     (320-FILE_INPUT_W)/2
#define FILE_INPUT_SC_W  32
#define FILE_INPUT_TXT_H 16

#define KEY_HIT_BG_COLOR RGBVAL16(0xff,0x00,0x00)
#define TEXT_INPUT_BG_COLOR RGBVAL16(0x80,0x80,0xff)
#define TEXT_INPUT_FG_COLOR RGBVAL16(0xff,0xff,0x40)
#define FILE_INPUT_BG_COLOR RGBVAL16(0x40,0x40,0x40)
#define FILE_INPUT_FG_COLOR RGBVAL16(0xff,0xff,0xe0)

// parameters for virtual keyboard
#define custom_key_shift     1
#define custom_key_backspace 2
#define custom_key_reset     3
#define custom_key_dir       4
#define custom_key_exitkbd   5
#define custom_key_restore   6
#define custom_key_swapjoy   7 // not implemented

const uint32_t ascii2scan[] = {
 //0 1 2 3 4 5 6 7 8 9 A B C D E F
   0,0,0,0,0,0,0,0,0,0,0,0,0,0x28,0,0, // return
 //     17:down                                                     29:right
   0x00,0x51,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x4f,0x00,0x00,
   //sp  !       "     #     $      %      &      '     (        )   *    +    ,    -    .    / 
   0x2c,0x201e,0x201f,0x2020,0x2021,0x2022,0x2023,0x2024,0x2025,0x2026,0x55,0x57,0x36,0x56,0x37,0x54,
   //0  1    2    3    4    5    6    7    8    9    :    ;    <      =    >      ?
   0x27,0x1e,0x1f,0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x33,0x34,0x2036,0x32,0x2037,0x0238,
   //@    A    B    C    D    E    F    G    H    I    J    K    L    M    N    O
   47,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,0x10,0x11,0x12,
   //P  Q    R    S    T    U    V    W    X    Y    Z    [      \     ]     ^    _  
   0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1a,0x1b,0x1c,0x1d,0x2026,0x31,0x2027,0x00,0x00,
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, // ' a b c d e f g h i j k l m n o
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x49,0, // p q r s t u v w x y z { | } ~ DEL
 //up left arr      133:f1   f2   f3   f4   f5   f6   f7   f8 
   75,78,0x00,0x00,0x00,0x3a,0x3b,0x3c,0x3d,0x3e,0x3f,0x40,0x41,0x00,0x00,0x00,  // 128-143
 //     145:up                                                      157:left
   0x00,0x2051,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x204f,0x00,0x00   // 144-159
};

const uint8_t keysw[]={254,4,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,4,27,  254,4,27,18,18,18,18,18,18,18,18,18,18,18,18,18,27,4,27, 254,0,18,18,18,18,18,18,18,18,18,18,18,18,18,18,36,4,27, 254,0,18,27,18,18,18,18,18,18,18,18,18,18,27,18,18,4,27, 
254,22,23,162, 255}; 
const char keys[]={0, custom_key_backspace,'1','2','3','4','5','6','7','8','9','0','+','-',/*(char)'£'*/ '~',0,0,0,133,  0,0,'Q','W','E','R','T','Y','U','I','O','P','@','*',128 /*arr up*/,custom_key_restore,0,135,  0,0,0,'A','S','D','F','G','H','J','K','L',':',';','=',13,0,137, 0,0,custom_key_shift,'Z','X','C','V','B','N','M',',','.','/',custom_key_shift,17,29,0,139,    custom_key_reset,custom_key_dir,' '}; 
const char keysshifted[]={0, 0,'!','\"','#','$','/','&','\'','(',')','0','+','-',/*(char)'£'*/ '~',0,0,0,134,  0,0,'Q','W','E','R','T','Y','U','I','O','P','@','*',128 /*arr up*/,custom_key_restore,0,136,  0,0,0,'A','S','D','F','G','H','J','K','L',':',';','=',13,0,138, 0,0,custom_key_shift,'Z','X','C','V','B','N','M','<','>','?',custom_key_shift,145,157,0,140,  custom_key_reset,custom_key_dir,' '}; 
#define KBUF_SIZE  64
#define KTEXTBUF_SIZE  40
static char kBuf[KBUF_SIZE];
static int kBufPt = 0;
static char kTextBuf[KTEXTBUF_SIZE+1];
static int kTextBufPt = 0;
static bool vkbKeepOn = false;
static bool vkbActive = false;
static bool vkeyRefresh=false;
static bool exitVkbd = false;
static bool shift=false;
static uint8_t prev_zt=0; 


static void sendKeyLocal(char c) {
    setKey(ascii2scan[c]);
    /*
    noInterrupts();
    if (cpu.RAM[198] == 0)  {
      cpu.RAM[631] = c;
      cpu.RAM[198] = 1;
      interrupts();
       delay(50);  
    }
    else {
       interrupts();   
    }
    */
}

static void resetBuffers() {
    kBufPt=0;
    kTextBufPt=0;
    //kBuf[kBufPt] = 0; 
    kTextBuf[kTextBufPt] = 0; 
}

static void flush64Keys(void) {     
    int i = 0;
    while (kBufPt) {
        kBufPt--;    
        sendKeyLocal(kBuf[i++]);  
    }
}


static bool pushCharToEntry(char c) {
    bool retval=false; 
    bool status = machineIsRunning();
    if (!status) {
        if (kBufPt < KBUF_SIZE) { 
            kBuf[kBufPt++] = c;
            //kBuf[kBufPt] = 0;
            if (kTextBufPt < KTEXTBUF_SIZE)  {
                kTextBuf[kTextBufPt++] = c;
                kTextBuf[kTextBufPt] = 0; 
                tft.drawTextNoDma( KEYBOARD_X,TEXT_INPUT_Y, kTextBuf, TEXT_INPUT_FG_COLOR, TEXT_INPUT_BG_COLOR, true);
                retval=true;  
            }
        } 
    }
    if (status) {        
        sendKeyLocal(c);       
        retval=true;  
    }
   
    return retval;
}

static void pushStringToTextEntry(char * text) {
    char c;
    while ((c = *text++)) {
        if (!pushCharToEntry(c)) {
            break;
        }
    }
}


static void process64Key(char c) {
    if (c == custom_key_backspace) { // backspace
        if (kBufPt < (KBUF_SIZE-3)) {
            if (kTextBufPt != 0) {
                kBuf[kBufPt++] = 157;
                kBuf[kBufPt++] = ' ';
                kBuf[kBufPt++] = 157; 
                kTextBufPt--;
                kTextBuf[kTextBufPt] = ' '; 
                tft.drawTextNoDma( KEYBOARD_X,TEXT_INPUT_Y, kTextBuf, TEXT_INPUT_FG_COLOR, TEXT_INPUT_BG_COLOR, true);
            } else {
                resetBuffers();
            } 
        } else {
           resetBuffers();
        }
        if (machineIsRunning()) {
            sendKeyLocal(157);
            sendKeyLocal(' ');
            sendKeyLocal(157); 
        } 
        vkeyRefresh = true;         
    } else {
        if (c != custom_key_exitkbd) {
            if (pushCharToEntry(c)) {
                vkeyRefresh = true; 
            }
            else {
                c = custom_key_exitkbd; //exitVkbd = true;
            }
        }
        if ( (c == 13) || (c == 17) || (c == 29) || (c == 145) || (c == 157) || ( (c>=133) && (c <=140)) | (c == custom_key_exitkbd) ) { // return or arrows exits keyboard
            vkeyRefresh = true;
            exitVkbd = true;
        }
    }
}


static int menu_Readir(bool loadfile, int index) {
  int totalFiles = 0;
  int curLine = 0;
  char filename[MAX_FILENAME_SIZE+1];
  if (!loadfile) {
    tft.drawRectNoDma( FILE_INPUT_X,FILE_INPUT_Y,FILE_INPUT_W-FILE_INPUT_SC_W,FILE_INPUT_H,  FILE_INPUT_BG_COLOR) ;
    tft.drawSpriteNoDma(FILE_INPUT_X+FILE_INPUT_W-FILE_INPUT_SC_W,FILE_INPUT_Y,(uint16_t*)logoscrollbar);
  }
  if (SDinitialized) {
    file = SD.open(DIRECTORY);
    while (true) {
        entry = file.openNextFile();
        if (! entry) {
            // no more files
            break;
        }
        if (!entry.isDirectory()) {
            if (loadfile) {
                if (totalFiles==index) {
                    entry.getName(&filename[0], MAX_FILENAME_SIZE);            
                    if (filename[0]) {
                        tft.drawRectNoDma( KEYBOARD_X,FILE_INPUT_Y,KEYBOARD_W,FILE_INPUT_H, RGBVAL16(0x00,0x00,0x00) ) ;   
                        String str(filename);
                        pushStringToTextEntry("LOAD\"");
                        pushStringToTextEntry((char*)(str.toUpperCase()).c_str());
                        pushStringToTextEntry("\"");
                        pushCharToEntry(13);
                        pushStringToTextEntry("RUN");
                        pushCharToEntry(13);
                        entry.close();
                        break;
                    }
                }
            }
            else {
                if ( (curLine < MAX_DISPLAY_LINES) && ( totalFiles >= curFile ) ) {
                    entry.getName(&filename[0], MAX_FILENAME_SIZE);            
                    tft.drawTextNoDma( FILE_INPUT_X+8, FILE_INPUT_Y + curLine*FILE_INPUT_TXT_H, filename, FILE_INPUT_FG_COLOR, FILE_INPUT_BG_COLOR, true);
                    curLine++;
                } 
            }
            totalFiles++;
        }
        entry.close();
    }
    file.close();

  }
  return totalFiles;
}

static char processFileinputZones(uint16_t xt, uint16_t yt) {
    xt -= FILE_INPUT_X;
    yt -= FILE_INPUT_Y;
    if (nbFiles) {
        if (xt > (FILE_INPUT_W-FILE_INPUT_SC_W)) {
            if (yt < FILE_INPUT_H/2) {
                if (curFile != 0) {
                    curFile -= MAX_DISPLAY_LINES;
                    menu_Readir(false, curFile);
                }
            }
            else {
                if ( (curFile+MAX_DISPLAY_LINES) < nbFiles) {
                    curFile += MAX_DISPLAY_LINES;
                    menu_Readir(false, curFile);
                }
            }
        }
        else {
            if ( yt < (FILE_INPUT_TXT_H*MAX_DISPLAY_LINES) ) {
                int index = curFile + yt/FILE_INPUT_TXT_H;
                if ( (index <  nbFiles) )
                {
                    menu_Readir(true, index);
                    return(custom_key_exitkbd);
                }
            }
        }
        return(0);        
    }
    else {
        return(custom_key_exitkbd); // touch area out of any defined zone
    }

}

static char processKeyboardZones(uint16_t xt, uint16_t yt, int *rx, int *ry, int *rw, int * rh) {
    int i=0;
    int k=0;
    int y2=0, y1=0;
    int x2=0, x1=0;
    uint8_t w;
    while ( (w=keysw[i++]) != 255 ) {
        if (w == 254) {
            y1 = y2;                 
            y2 = y1 + ROW_KEYBOARD_H;
            x2 = 0;
        }
        else {
            x1 = x2;
            x2 = x1+w;
            if ( (yt >= y1) && (yt < y2) && (xt >= x1) && (xt < x2)  ) {
                *rx = x1;
                *ry = y1;
                *rw = x2-x1;
                *rh = y2-y1;
                if (shift) {
                    return (keysshifted[k]);
                }
                else {
                    return (keys[k]);
                }
                break;   
            }
            k++;
        } 
    }
    return(custom_key_exitkbd); // touch area out of any defined zone  
}

static char captureVirtualkeyboard(int *rx, int *ry, int *rw, int * rh) {
    uint16_t xt=0;
    uint16_t yt=0;
    uint16_t zt=0;  
  
    if (tft.isTouching())
     {
        if (prev_zt == 0) {
            prev_zt =1;
            tft.readRaw(&xt,&yt,&zt);
            if ( (yt >= FILE_INPUT_Y) && (yt < (FILE_INPUT_Y+FILE_INPUT_H)) && (xt >= FILE_INPUT_X) && (xt < (FILE_INPUT_X+FILE_INPUT_W))  ) {
                return(processFileinputZones(xt,yt)); 
            }
            else {
                return(processKeyboardZones(xt,yt,rx,ry,rw,rh));
            }
        } 
        prev_zt =1; 
    } else {
        prev_zt=0; 
    } 
  
    return 0;   
}    

bool virtualkeyboardIsActive(void) {
    return (vkbActive);
}


void toggleVirtualkeyboard(bool keepOn) {     
    if (keepOn) {      
        tft.drawSpriteNoDma(0,0,(uint16_t*)logokbdstandard);
        tft.drawRectNoDma( KEYBOARD_X,TEXT_INPUT_Y,KEYBOARD_W,TEXT_INPUT_H, TEXT_INPUT_BG_COLOR );
        tft.drawRectNoDma( KEYBOARD_X,FILE_INPUT_Y,KEYBOARD_W,FILE_INPUT_H, RGBVAL16(0x00,0x00,0x00) ) ;   
        nbFiles = 0;
        prev_zt = 0;
        vkbKeepOn = true;
        vkbActive = true;
        exitVkbd = false;
        resetBuffers();    
    }
    else {
        vkbKeepOn = false;
        if ( (vkbActive) /*|| (exitVkbd)*/ ) {
            tft.fillScreenNoDma( RGBVAL16(0x00,0x00,0x00) );
            tft.begin();
#if VGATFT
            tft.flipscreen(true);
#endif 
            tft.refresh();
            resumeMachine();
            delay(100); 
            flush64Keys();
            resetBuffers(); 
            prev_zt = 0; 
            vkbActive = false;
            exitVkbd = false;
        }
        else {
            tft.stop();
            pauseMachine();
            delay(100);
            kBufPt = 0;
            kTextBufPt = 0;
            tft.begin();
#if VGATFT
            tft.flipscreen(true);
#endif             
            tft.start();
            tft.drawSpriteNoDma(0,0,(uint16_t*)logokbdstandard);
            tft.drawRectNoDma( KEYBOARD_X,TEXT_INPUT_Y,KEYBOARD_W,TEXT_INPUT_H, TEXT_INPUT_BG_COLOR );
            tft.drawRectNoDma( KEYBOARD_X,FILE_INPUT_Y,KEYBOARD_W,FILE_INPUT_H, RGBVAL16(0x00,0x00,0x00) ) ;   
            nbFiles = 0;            
            prev_zt = 0;
            vkbActive = true;
            exitVkbd = false;
            resetBuffers(); 
        }
    }   
}



void handleVirtualkeyboard() {
    if ( (!virtualkeyboardIsActive()) && (tft.isTouching()) ) {
        toggleVirtualkeyboard(false);
        return;
    }
    int rx=0,ry=0,rw=0,rh=0;
    if ( (vkbKeepOn) || (virtualkeyboardIsActive())  ) {
        char c = captureVirtualkeyboard(&rx,&ry,&rw,&rh);
        if (c) {
            tft.drawRectNoDma( rx,ry,rw,rh, KEY_HIT_BG_COLOR ) ;
            if (c == custom_key_shift) {
                if (shift) {
                    shift = false;
                    tft.drawSpriteNoDma(0,0,(uint16_t*)logokbdstandard);            
                }
                else {
                    shift = true;
                    tft.drawSpriteNoDma(0,0,(uint16_t*)logokbdshift);            
                }
            }
            else if (c==custom_key_reset) {
                resetMachine();
            }
            else if (c==custom_key_restore) {
                cpu_nmi();
                vkeyRefresh = true;
                exitVkbd = true;
            }
            else if (c==custom_key_dir) {
                curFile = 0;
                nbFiles = menu_Readir(false,curFile);
                vkeyRefresh = true;
            }
            else {
                process64Key(c);
            }
            delay(50); 
        }   
     }    
     
    if (vkeyRefresh) {
        vkeyRefresh = false;
        if (shift == true) {
            tft.drawSpriteNoDma(0,0,(uint16_t*)logokbdstandard);
            shift = false;
        }
        else {
            tft.drawSpriteNoDma(0,0,(uint16_t*)logokbdstandard, rx, ry, rw, rh);         
        }
    }  
         
    if ( (exitVkbd) && (vkbActive) ) {      
        if (!vkbKeepOn) {             
            toggleVirtualkeyboard(false);
        }
        else {         
            toggleVirtualkeyboard(true);           
        } 
    }       
}
#endif
