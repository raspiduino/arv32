/*
 * Arv32: Linux on mini-rv32ima emulator on Arduino Platform
 * mini-rv32ima is written by Charles Lohr
 * Ported to Arduino UNO by Giang Vinh Loc
 */

// Standard Arduino libraries
#include <SD.h> // SD card library

// Variables
int fail_on_all_faults = 0;
File vram; // File descriptor
uint32_t corebase = 0;
int coresize32 = 0;
uint64_t lastTime = 0;

// Functions prototype
static uint32_t HandleException( uint32_t ir, uint32_t retval );
static uint32_t HandleControlStore( uint32_t addy, uint32_t val );
static uint32_t HandleControlLoad( uint32_t addy );
static void HandleOtherCSRWrite( uint8_t * image, uint16_t csrno, uint32_t value );

// Load / store helper
static uint32_t store4(uint32_t ofs, uint32_t val);
static uint16_t store2(uint32_t ofs, uint16_t val);
static uint8_t store1(uint32_t ofs, uint8_t val);

static uint32_t load4(uint32_t ofs);
static uint16_t load2(uint32_t ofs);
static uint8_t load1(uint32_t ofs);

// Config macros
#define CS_PIN 4              // SD card chip select pin
#define RAM_SIZE 12*1024*1024 // Minimum RAM amount (in bytes), just tested (may reduce further by custom kernel)
#define RAM_FILE "rv32.bin"   // RAM file on SD card
#define DTB_SIZE 1536         // DTB size (in bytes), must recount manually each time DTB changes
#define INSTRS_PER_FLIP 1024  // Number of instructions executed before checking status. See loop()
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

struct MiniRV32IMAState * core;

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
  coresize32 = sizeof(struct MiniRV32IMAState) / 4;      // Number of uint32_t in core struct

  // Setup core
  core->pc = MINIRV32_RAM_IMAGE_OFFSET;
  core->regs[10] = 0x00; //hart ID
  core->regs[11] = corebase - DTB_SIZE + MINIRV32_RAM_IMAGE_OFFSET; // dtb_pa (Must be valid pointer) (Should be pointer to dtb)
  core->extraflags |= 3; // Machine-mode.
}

// Loop function
void loop() {
  // Sync core struct in Arduino's RAM with VRAM
  for (int i = 0; i < coresize32; i++) {
    store4(corebase + 4*i, *(uint32_t*)(core + 4*i)); 
  }

  // Emulator cycle
  uint64_t * this_ccount = ((uint64_t*)&core->cyclel);
  uint32_t elapsedUs = 0;
  elapsedUs = *this_ccount / TIME_DIVISOR - lastTime;
  lastTime += elapsedUs;
  
  int ret = MiniRV32IMAStep( core, NULL, 0, elapsedUs, INSTRS_PER_FLIP ); // Execute upto INSTRS_PER_FLIP cycles before breaking out.
  switch( ret )
  {
    case 0: break;
    case 1: delay(1); *this_ccount += INSTRS_PER_FLIP; break;
    //case 3: instct = 0; break;
    //case 0x7777: goto restart;  //syscon code for restart
    case 0x5555: Serial.print(F("POWEROFF")); while(1); //syscon code for power-off -> halt
    default: Serial.println(F("Unknown failure")); break;
  }
}

// Exception handlers

static uint32_t HandleException( uint32_t ir, uint32_t code )
{
  // Weird opcode emitted by duktape on exit.
  if( code == 3 )
  {
    // Could handle other opcodes here.
  }
  return code;
}

static uint32_t HandleControlStore( uint32_t addy, uint32_t val )
{
  if( addy == 0x10000000 ) //UART 8250 / 16550 Data Buffer
  {
    Serial.print((char)val);
  }
  
  return 0;
}


static uint32_t HandleControlLoad( uint32_t addy )
{
  // Emulating a 8250 / 16550 UART
  if( addy == 0x10000005 )
    return 0x60 | (Serial.available() > 0);
  else if( addy == 0x10000000 && (Serial.available() > 0) )
    return Serial.read();
  return 0;
}

static void HandleOtherCSRWrite( uint8_t * image, uint16_t csrno, uint32_t value )
{
  if( csrno == 0x136 )
  {
    Serial.print(value);
  }
  if( csrno == 0x137 )
  {
    Serial.print(value); // TODO: Format should be hex
  }
  /*else if( csrno == 0x138 )
  {
    // Print "string"
    uint32_t ptrstart = value - MINIRV32_RAM_IMAGE_OFFSET;
    uint32_t ptrend = ptrstart;
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
static uint32_t store4(uint32_t ofs, uint32_t val) {
  vram.seek(ofs);
  vram.write((val & 0xff000000UL) >> 24);
  vram.write((val & 0x00ff0000UL) >> 16);
  vram.write((val & 0x0000ff00UL) >> 8);
  vram.write(val & 0x000000ffUL);
  return val;
}

static uint16_t store2(uint32_t ofs, uint16_t val) {
  vram.seek(ofs);
  vram.write(val >> 8);
  vram.write(val & 0xff);
  return val;
}

static uint8_t store1(uint32_t ofs, uint8_t val) {
  vram.seek(ofs);
  vram.write(val);
  return val;
}

static uint32_t load4(uint32_t ofs) {
  vram.seek(ofs);

  uint8_t r1 = vram.read();
  uint8_t r2 = vram.read();
  uint8_t r3 = vram.read();
  uint8_t r4 = vram.read();

  return (((uint32_t)r4 << 24) + ((uint32_t)r3 << 16) + ((uint32_t)r2 << 8) + r1);
}
static uint16_t load2(uint32_t ofs) {
  vram.seek(ofs);
  
  uint8_t r1 = vram.read();
  uint8_t r2 = vram.read();

  return (((uint16_t)r2 << 8) + r1);
}
static uint8_t load1(uint32_t ofs) {
  vram.seek(ofs);
  return vram.read();
}
