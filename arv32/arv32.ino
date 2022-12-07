/*
 * Arv32: Linux on mini-rv32ima emulator on Arduino Platform
 * mini-rv32ima is written by Charles Lohr
 * Ported to Arduino UNO by Giang Vinh Loc
 */

// Standard Arduino libraries
#include <SD.h> // SD card library

// Local headers
#include "types.h"

// Variables
int fail_on_all_faults = 0;
File vram; // File descriptor
UInt32 corebase = 0;
int coresize32 = 0;
uint64_t lastTime = 0;

// Functions prototype
static UInt32 HandleException( UInt32 ir, UInt32 retval );
static UInt32 HandleControlStore( UInt32 addy, UInt32 val );
static UInt32 HandleControlLoad( UInt32 addy );
static void HandleOtherCSRWrite( UInt8 * image, UInt16 csrno, UInt32 value );

// Load / store helper
static UInt32 store4(UInt32 ofs, UInt32 val);
static UInt16 store2(UInt32 ofs, UInt16 val);
static UInt8 store1(UInt32 ofs, UInt8 val);

static UInt32 load4(UInt32 ofs);
static UInt16 load2(UInt32 ofs);
static UInt8 load1(UInt32 ofs);

// Config
#define CS_PIN 10                 // SD card chip select pin
#define RAM_FILE "rv32.bin"       // RAM file on SD card
const UInt32 RAM_SIZE = 12582912; // Minimum RAM amount (in bytes), just tested (may reduce further by custom kernel)
#define DTB_SIZE 1536             // DTB size (in bytes), must recount manually each time DTB changes
#define INSTRS_PER_FLIP 1024      // Number of instructions executed before checking status. See loop()
#define TIME_DIVISOR 1

// This is the functionality we want to override in the emulator.
// think of this as the way the emulator's processor is connected to the outside world.
#define MINIRV32WARN( x... ) Serial.print( x );
#define MINIRV32_DECORATE  static
#define MINI_RV32_RAM_SIZE RAM_SIZE
#define MINIRV32_IMPLEMENTATION // Minimum rv32 emulator
#define MINIRV32_POSTEXEC( pc, ir, retval ) { if( retval > 0 ) { if( fail_on_all_faults ) { Serial.println(F("FAULT")); return 3; } else retval = HandleException( ir, retval ); } }
#define MINIRV32_HANDLE_MEM_STORE_CONTROL( addy, val ) if( HandleControlStore( addy, val ) ) return val;
#define MINIRV32_HANDLE_MEM_LOAD_CONTROL( addy, rval ) rval = HandleControlLoad( addy );
#define MINIRV32_OTHERCSR_WRITE( csrno, value ) HandleOtherCSRWrite( image, csrno, value );
#define MINIRV32_CUSTOM_MEMORY_BUS // Custom RAM handler for swapping to SD card

// Macro for accessing RAM
#define MINIRV32_STORE4( ofs, val ) store4(ofs, val)
#define MINIRV32_STORE2( ofs, val ) store2(ofs, val)
#define MINIRV32_STORE1( ofs, val ) store1(ofs, val)
#define MINIRV32_LOAD4( ofs ) load4(ofs)
#define MINIRV32_LOAD2( ofs ) load2(ofs)
#define MINIRV32_LOAD1( ofs ) load1(ofs)

// Internal headers
#include "rv32ima.h" // rv32ima emulator

struct MiniRV32IMAState core;

// Setup function
void setup() {
  // Initialize UART with baudrate 9600
  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Print banner
  Serial.println(F("Arv32: Linux on Arduino Platform"));

  // Init SD card
  Serial.print(F("Initializing SD card... "));

  if (!SD.begin(CS_PIN)) {
    Serial.println(F("initialization failed!"));
    while (1);
  }
  
  Serial.println(F("initialization done."));

  // Open RAM file
  vram = SD.open(RAM_FILE, FILE_READ | FILE_WRITE);

  if (!vram) {
    Serial.print(F(RAM_FILE));
    Serial.println(F(" failed to open! Halting!"));
  }

  // Calculate variables
  corebase = RAM_SIZE - sizeof(struct MiniRV32IMAState); // Base address of core struct
  coresize32 = sizeof(struct MiniRV32IMAState) / 4;      // Number of UInt32 in core struct

  // Clear the struct
  for (int i = 0; i < coresize32; i++) {
    *(UInt32*)((UInt8*)&core + 4*i) = 0;
  }

  // Setup core
  core.pc = MINIRV32_RAM_IMAGE_OFFSET;
  core.regs[10] = 0x00; //hart ID
  core.regs[11] = corebase - DTB_SIZE + MINIRV32_RAM_IMAGE_OFFSET; // dtb_pa (Must be valid pointer) (Should be pointer to dtb)
  core.extraflags |= 3; // Machine-mode.
}

// Loop function
void loop() {
  // Emulator cycle
  uint64_t * this_ccount = ((uint64_t*)&core.cyclel);
  UInt32 elapsedUs = 0;
  elapsedUs = *this_ccount / TIME_DIVISOR - lastTime;
  lastTime += elapsedUs;
  
  int ret = MiniRV32IMAStep( core, NULL, 0, elapsedUs, INSTRS_PER_FLIP ); // Execute upto INSTRS_PER_FLIP cycles before breaking out.
  switch( ret )
  {
    case 0: break;
    case 1: delay(1); *this_ccount += INSTRS_PER_FLIP; break;
    //case 3: instct = 0; break;
    //case 0x7777: goto restart;  //syscon code for restart
    case 0x5555: Serial.print(F("POWEROFF")); while(1); //syscon code for power-off . halt
    default: Serial.println(F("Unknown failure")); break;
  }
}

// Exception handlers

static UInt32 HandleException( UInt32 ir, UInt32 code )
{
  // Weird opcode emitted by duktape on exit.
  if( code == 3 )
  {
    // Could handle other opcodes here.
  }
  return code;
}

static UInt32 HandleControlStore( UInt32 addy, UInt32 val )
{
  if( addy == 0x10000000 ) //UART 8250 / 16550 Data Buffer
  {
    Serial.print((char)val);
  }
  
  return 0;
}


static UInt32 HandleControlLoad( UInt32 addy )
{
  // Emulating a 8250 / 16550 UART
  if( addy == 0x10000005 )
    return 0x60 | (Serial.available() > 0);
  else if( addy == 0x10000000 && (Serial.available() > 0) )
    return Serial.read();
  return 0;
}

static void HandleOtherCSRWrite( UInt8 * image, UInt16 csrno, UInt32 value )
{
  if( csrno == 0x136 )
  {
    Serial.print(value);
  }
  if( csrno == 0x137 )
  {
    Serial.print(value, HEX);
  }
  /*else if( csrno == 0x138 )
  {
    // Print "string"
    UInt32 ptrstart = value - MINIRV32_RAM_IMAGE_OFFSET;
    UInt32 ptrend = ptrstart;
    if( ptrstart >= ram_amt )
      printf( "DEBUG PASSED INVALID PTR (%08x)\n", value );
    while( ptrend < ram_amt )
    {
      if( image[ptrend] == 0 ) break;
      ptrend++;
    }
    if( ptrend != ptrstart )
      fwrite( image + ptrstart, ptrend - ptrstart, 1, stdout );
  }*/
}

// Memory access functions
static UInt32 store4(UInt32 ofs, UInt32 val) {
  vram.seek(ofs);

  vram.write(((UInt8 *)&val)[0]);
  vram.write(((UInt8 *)&val)[1]);
  vram.write(((UInt8 *)&val)[2]);
  vram.write(((UInt8 *)&val)[3]);
  return val;
}

static UInt16 store2(UInt32 ofs, UInt16 val) {
  vram.seek(ofs);

  vram.write(((UInt8 *)&val)[0]);
  vram.write(((UInt8 *)&val)[1]);
  return val;
}

static UInt8 store1(UInt32 ofs, UInt8 val) {
  vram.seek(ofs);

  vram.write(val);
  return val;
}

static UInt32 load4(UInt32 ofs) {
  vram.seek(ofs);

  UInt32 result;
  ((UInt8 *)&result)[0] = vram.read(); // LSB
  ((UInt8 *)&result)[1] = vram.read();
  ((UInt8 *)&result)[2] = vram.read();
  ((UInt8 *)&result)[3] = vram.read(); // MSB
  return result;
}
static UInt16 load2(UInt32 ofs) {
  vram.seek(ofs);
  
  UInt16 result;
  ((UInt8 *)&result)[0] = vram.read(); // LSB
  ((UInt8 *)&result)[1] = vram.read(); // MSB
  return result;
}
static UInt8 load1(UInt32 ofs) {
  vram.seek(ofs);
  return vram.read();
}
