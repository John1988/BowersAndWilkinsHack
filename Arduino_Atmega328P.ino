/*
  Drop-in hack solution for digital IO control on B&W's Class D Amp. replacement (with TDA7313 volume / EQ control)

 Receives events from iPod TX (when available), or B&W buttons
 in case AUX input available.

 Code detects certain events such as play/ pause/ power for fade / power management events.

Circuit connections
 Pin 7 & 20 VCC = 5V (with 10 uF to GND)
 Pin 8 & 22 GND
 Pin 1 (reset with 10k resistor to VCC)
 Pin 9 & 10, 16Mhz crystal connected to two 22pf capacitors to ground
 Pin 2 = TX iPod (i.e. RX B&W) (configured as software RX)
 Pin 3 = RX iPod (i.e. TX B&W) (configured as software RX)
 Pin 4 = B&W power enable, pwr in, connected to piggy back board pin 3 from bottom row (3.3V, but 5V compatible)
 pin 5 = pwr out (via transistor to relay) -> Reserved/ not implemented, used capacitor/ transistor to pwr in (Pin 4)

Debug header:
 Pin 2 (RX, red)
 Pin 3 (Tx, blue)
 Pin 1 (Reset, yellow)
 Pin 7 (GND, black)

I2C (TDA7313):
 Pin 28 (A5, I2C SCL)
 A4 (pin 27, I2C SDA)

 Created 8-8-2022
 by John Heesterbeek
 Licence: CC BY-NC 4.0

 */
#include <SoftwareSerial.h>
#include <Wire.h>

// Pinouts
const int g_ver = 1;
const int g_baudrate = 19200; // This is the fixed baudrate for iPod
const byte g_rx_pin_ipod = 2;
const byte g_rx_pin_bnw = 3;
const byte g_pwr_pin_bnw = 4; // This pin is the 3th pin from the bottom row on the long header, it is 3.3V when powered
// A5 (pin 28) = SCL (I2C)
// A4 (pin 27) = SDA (I2C)
SoftwareSerial sw_serial_ipod(g_rx_pin_ipod, -1); // RX, TX = unavailable = -1
SoftwareSerial sw_serial_bnw(g_rx_pin_bnw, -1); // RX, TX = unavailable = -1

// Globally keep track of some variables
int g_adxl_address = 0x44; // Device address in which is also included the 8th bit for selecting the mode, read in this case.
byte g_in_bytes[20]; // Global declaration of max amount of inBytes
bool g_valid_vol = false; // Globally keep track of 5-6 first volume related bytes
int g_ipod_avail_debounce = 0; // Globally keep track of debounce counter (up to 10)
int g_pwr_amp_avail_debounce = 0; // Globally keep track of debounce counter (up to 10)
int g_debounce_treshold = 10; // Max error
int g_byte_pos = 0; // Globally keep track of byte position

// Default settings
int g_volume_last_known = 89; // Default volume, 0-255 -> 63-0 (64 settings)
bool g_mute_last_known;
bool g_play = false;
bool g_pwr_amp_sw_input = false;

// Predefined (incredible surround) audio profles
int g_eq_state = 1; // Default EQ (Flat)
const int g_arr_rows = 11;
const int g_arr_colums = 2;
byte g_audio_profile_array[g_arr_rows][g_arr_colums] = {
  //bass, treble, loudness compensation
  {0x67, 0x77}, //Flat (for testing) (bass: +0dB, treble: 0dB)
  {0x68, 0x70}, //More bass (bass: +14dB, treble: -14dB)
  {0x68, 0x77}, //More treble (bass: +14dB, treble: +0dB)
  {0x68, 0x7C}, //More treble (bass: +14dB, treble: +4dB)
  {0x68, 0x7A}, //More treble (bass: +14dB, treble: +10dB)
  {0x68, 0x78}, //Max sound (bass: +14dB, treble: +14dB)
  
  // speaker attenuation LF and RF (for mid/ treble speakers, while bass remains max)
  {0x80, 0xA0}, //-0 db LF / RF
  {0x84, 0xA4}, //-5 db LF / RF
  {0x88, 0xA8}, //-10 db LF / RF
  {0x90, 0xB0}, //-20 db LF / RF
  {0x98, 0xB8}, //-30 db LF / RF
};                                            
                                            
bool g_pwr_amp_avail = false; // Globally keep track of iPod availability
bool g_ipod_avail = false; // Globally keep track of iPod availability
bool g_debug = false; // Globally keep track of debug-mode
int g_input_source = 0; // Range 0-3 (3= internally connected). (Better to be updated by select_source() only!)

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(g_baudrate);
  Serial.print("Bowers & Wilkins amp EC control by John Heesterbeek, version: "); Serial.println(g_ver);
  while (!Serial) {;} // wait for serial port to connect. Needed for Native USB only
  Serial.println("Connecting to i2c TDA7313...");
  Wire.begin(); // Init i2c, if this fails, code will hang here!
  Serial.println("Connection to i2c succeeded");
  pinMode(g_rx_pin_ipod, INPUT); // Used for iPod detection as well as for Sw. serial
  pinMode(g_pwr_pin_bnw, INPUT); // Used for power detection, 3.3V compatible
  Serial.println("OK: iPod serial init");

  // Set the data rate for the SoftwareSerial port
  sw_serial_ipod.begin(g_baudrate);
  sw_serial_bnw.begin(g_baudrate);

  // Set audio defaults
  select_source (g_input_source); // Select source
  set_eq (6); // Set default equaliser, 6 = MAX bass, MAX treble
  set_eq (8); // Set default equaliser, 8 = with -5 db treble speaker attenuation

  // Overrides (normally controlled by regular button-events);
  // mute_speaker(false); // Unmute + 'restore' initial volume to volume_last_known
  // set_volume (volume_last_known); // Overwrite for testing
  // Done, show welcome message in terminal for debugging (also to check if serial/ baudrate OK)
  Serial.println("Setup done! Type 'd' for Debug, 'e' for Equalizer:");
}

void loop()
{
  handle_mode_toggle(); // Toggle debugging on demand
  handle_power_state(); // Handle power state
  handle_serial_data(); // Handle all iPod + BnW serial data
}

void handle_mode_toggle (){
  // Toggle debugging mode, and EC mode on demand
  if (Serial.available() > 0) {
    int buf = Serial.read();
    if (buf == 'd'){
      g_debug = !g_debug;
      Serial.print("OK: Toggle debug mode = "); Serial.println(g_debug ? "enabled" : "disabled");      
    }
    else if (buf == 'e'){
      toggle_eq();
      Serial.print("OK: Toggle EC = "); Serial.println(g_eq_state);
    }
  }
}

void handle_power_state(){
  // handle_power_state(); handles polling of pin 4, to caputure events for energy saving/ and audible 'plop' suppression. 
  // This might actually be a data pin, so we need to ignore at least about 16 counts (or over 1000 counts if no printing delay)
  // Hence, rather ignore the pin-toggle as the last state will be always correct.
  if (digitalRead(g_pwr_pin_bnw) == true and g_pwr_amp_avail == false){
    g_pwr_amp_avail_debounce++;
    if (g_pwr_amp_avail_debounce > g_debounce_treshold){
      g_pwr_amp_avail = true;
      g_pwr_amp_avail_debounce = 0;
      Serial.println("OK: BnW AMP ON");
      if (!g_play){fade_volume (0, g_volume_last_known);} // Aux cannot be muted
    }
  }
  else if (digitalRead(g_pwr_pin_bnw) == false and g_pwr_amp_avail == true){
    g_pwr_amp_avail_debounce++;
    if (g_pwr_amp_avail_debounce > g_debounce_treshold){
      g_pwr_amp_avail = false;
      g_pwr_amp_avail_debounce = 0;
      Serial.println("OK: BnW AMP OFF");
      g_play = false;
      fade_volume (g_volume_last_known, 0); // Go to mute
      mute_speaker(true); // Cancel audio before it pops
    }

    // This little tweak prevents lockup in mute mode once toggling from iPod to AUX input by a long press
    // of the power button. In case of toggling a ±50 ms brownout occurs. This tweak flags that time-frame 
    // to detect and ignore mute requests during the iPod's Pause routine
    delay(10);
    g_pwr_amp_sw_input = false;
    if (g_pwr_amp_avail_debounce > 4 and g_pwr_amp_avail_debounce < 7){g_pwr_amp_sw_input = true;}
    if (g_pwr_amp_sw_input and g_debug){Serial.println("DEBUG: BnW AMP switching input...");}
  }
  else{g_pwr_amp_avail_debounce = 0;} // Reset counter
}

void select_serial_source(){
  // iPod availability detection (important, as we can listen to one port only)
  // The 'debounce'-mechanism prevents toggling status while initialising iPod
  // Do NOT use interupt for this, sequence is too long, and iPod/ BnW serial has priority! 
  if (digitalRead(g_rx_pin_ipod) == true and g_ipod_avail == false){
    g_ipod_avail_debounce++;
    if (g_ipod_avail_debounce > g_debounce_treshold){
      g_ipod_avail = true;
      g_ipod_avail_debounce = 0;
      sw_serial_ipod.listen();
      Serial.println("OK: iPod avail");
      fade_volume (0, g_volume_last_known);
    }
  }
  else if (digitalRead(g_rx_pin_ipod) == false and g_ipod_avail == true){
    g_ipod_avail_debounce++;
    if (g_ipod_avail_debounce > g_debounce_treshold){
      g_ipod_avail = false;
      g_ipod_avail_debounce = 0;
      sw_serial_bnw.listen();
      Serial.println("OK: iPod NOT avail");
      g_play = false;
      fade_volume (0, g_volume_last_known); // Go to volume_last_known (for aux, because aux has no play to activate)
    }
  }
  else{
    if (g_ipod_avail_debounce > 0){g_ipod_avail_debounce = 0;} // Reset counter
  }
}

int handle_serial_data(){
  select_serial_source(); // Select source for serial trigger
  unsigned long serial_buffer;
  int match_pos = 0;
  // volbuttons FF 09 04 00 0B E0 max; FF 55 04 03 00 00 0E EB . FF 55 05 03 09 04 00 6D 7E
  byte vol_bytes_ipod[] = {0xFF, 0x55, 0x05, 0x03, 0x09, 0x04}; // FF 55 05 03 09 04 00 25 C6
  // On RX pin of ipod volume B&W is received, on TX pin of ipod ipod volume is then transmitted
  // Vol B&W from min FF 55 06 03 0E 04 00 00 01 E4 up to vol max FF 55 06 03 0E 04 00 FE 01 E6
  byte vol_bytes_bnw[] =  {0xFF, 0x55, 0x06, 0x03, 0x0E, 0x04}; // FF 55 06 03 0E 04 00 00 01 E4
  
  // If softwareserial available, read byte for byte
  if (sw_serial_ipod.available() or sw_serial_bnw.available()){
    if (g_ipod_avail == true){
      serial_buffer = sw_serial_ipod.read();
    }
    else{
      serial_buffer = sw_serial_bnw.read();
    }
    // Exception for byte_pos 6, because of volume level '0xFF'! :)
    // FF 55 04 03 00 00 0E EB = + / - buttons B&W (These will also apply when ipod connected too!)
    if (serial_buffer == 0xFF && g_byte_pos != 6){ 
      if (g_byte_pos > 1){
        if (g_debug){Serial.print("DEBUG: Discard CMD buffer = "); print_byte(g_in_bytes, g_byte_pos);}
      }
      g_byte_pos = 0;
      g_valid_vol = false;
      if (g_debug) {Serial.println("\nDEBUG: New CMD");}
    }
    else{
      g_byte_pos++;
    }
    g_in_bytes[g_byte_pos] = serial_buffer;

    // Simple error check and prevent buffer overflow
    if (g_byte_pos > 10){ // expected 9 x up to 0xFF
      if (g_debug) {Serial.print("DEBUG: buffer overflow/ data corrupted = "); print_byte(g_in_bytes, g_byte_pos);}
      g_byte_pos = 0;
    }

    // Verify payload content if valid
    // FF 55 05 03 09 04 00 00 xx up to FF 55 05 03 09 04 00 FF xx, where xx = checksum (via iPod)
    // FF 55 06 03 0E 04 00 00 01 E4 up to vol max FF 55 06 03 0E 04 00 FE 01 E6 (via B&W)
    match_pos = 0;
    if (g_byte_pos == 5){
      for (int i=0; i <= g_byte_pos; i++){
        if (g_in_bytes[i] == vol_bytes_ipod[i]){
           if (match_pos == 5){g_valid_vol = true;}
           match_pos++;
        }
        if (g_in_bytes[i] == vol_bytes_bnw[i]){
           if (match_pos == 5){g_valid_vol = true;}
           match_pos++;
        }      
      }
    }
    
    // Catch wrong max volume exception using checksum-byte(?) for volume init;
    // after undock/ dock; FF 55 05 03 09 04 00 FF EC = valid, FF55*****FF55 = invalid
    // Fun-fact; These special cmd's might contain iPod user name in ascii, as seen in terminal;
    if (g_byte_pos == 8){
      if (g_in_bytes[7] == 0xFF and g_in_bytes[8] != 0xEC) {
        if (g_debug){Serial.print("DEBUG: Invalid volume payload discard = "); print_byte(g_in_bytes, g_byte_pos);}
        g_valid_vol = false;
      }
    }
    
    // Play / Pause detection
    // FF 55 04 03 09 03 02 EB = play music, FF 55 04 03 09 03 01 EC = pause music
    // FF 55 04 03 09 03 03 EA = Fast Forward, FF 55 04 03 09 03 04 E9 = Fast Reverse
    if (g_byte_pos == 7 && g_in_bytes[2] == 0x04 && g_in_bytes[5] == 0x03){ 
      if (g_in_bytes[6] == 0x01){
        Serial.println("OK: Mode = iPod Play");
        if (g_mute_last_known){fade_volume (0, g_volume_last_known);}
        mute_speaker(false);
        g_play = true;
      }
      else if (g_in_bytes[6] == 0x02){
        // Little tweak to avoid mute in case of switching inputs
        Serial.println("OK: Mode = iPod Paused");
        //fade_volume (volume_last_known, 0); // Too slow, and no need to.
        if (!g_pwr_amp_sw_input){mute_speaker(true);} // Skip mute when switch to AUX detected
        g_play = false;
      }
      else if (g_in_bytes[6] == 0x03){Serial.println("OK: Mode = Fast Forward");}
      else if (g_in_bytes[6] == 0x04){Serial.println("OK: Mode = Fast Reverse");}
      else{
        Serial.print("NOK: Mode = play/pause status failed or unknown = "); print_byte(g_in_bytes, g_byte_pos);
      } 
      g_byte_pos = 0;
      g_valid_vol = false; // Reset
    }

    // Volume detection; Check if payload not too long, should be max. 9 blocks up to 0xFF
    if (g_valid_vol == true && g_byte_pos > 8){g_valid_vol == false;} 
    if (g_byte_pos == 8){
      if (g_valid_vol == true){
        g_valid_vol = false; // Reset
        if (g_debug){Serial.print("DEBUG: Volume payload iPod / BnW = "); print_byte(g_in_bytes, g_byte_pos);}
        g_byte_pos = 0;
        Serial.print("OK: Volume decimal iPod / BnW = "); Serial.println(g_in_bytes[7], DEC);
        if (!g_mute_last_known){set_volume(g_in_bytes[7]);}
        g_volume_last_known = g_in_bytes[7];

        // Restore audio un-mute on volume change (especially for AUX)
        if (g_mute_last_known){
           Serial.print("OK: Volume force unmute");
          mute_speaker(false);
          fade_volume (0, g_volume_last_known); // Go to unmute
        }
      }
      else{
        if (g_debug) {Serial.println("DEBUG: Volume invalid!!!");}
      }
    }
  }
}

void print_byte (byte buffer[], int size){
  // Print data buffer of incoming/ outgoing byte, i.e. print array of bytes to string
  for (int i=0; i < size; i++){
    if (buffer[i]<=16) Serial.print('0'); // Take care of leading zero
    Serial.print(buffer[i], HEX); Serial.print(" "); // Print hex and add space for readability
  }
  Serial.println(); // Feed new line (\n)
}

void fade_volume (int from, int to){
  // Fade volume from certain setpoint to certain final setpoint (range 0-255)
  set_volume (from);
  mute_speaker(false);
  int volume_step_ms = 1; // Fade speed, 10-20ms was fine.
  int vol_step_size = 5;
  if (from < to){
    for (int i = from; i <= to-vol_step_size; i=i+vol_step_size) {
      set_volume (i);
      delay(volume_step_ms);
    }
  }
  else {
    for (int i = from; i >= to+vol_step_size; i=i-vol_step_size) {
      set_volume (i);
      delay(volume_step_ms);
    }
  }
}

void mute_speaker(bool mute){
  // Mute and unmute all 4 speakers
  Serial.print("OK: mute_speaker mode = "); Serial.println(mute ? "mute" : "un-mute");
  if (mute){set_volume(0);} // Make completely silent.
  byte buffer[4]; // = {0x9F, 0xBF, 0xDF, 0xEF}; // Speaker LF, RF, LR, RR
  for (int i=0; i <= 3; i++){ // i = iteration for every speaker
    buffer[i] = 0x80 ^ (i << 5) + (mute * 0x1F);
  }
  g_mute_last_known = mute;
  wire_write_buffer(buffer, sizeof(buffer));
}

void select_source (byte source){
  // Select an input source, available sources;
  // 0x40 (input 0), 0x41 (input1), 0x42 (input2), 0x43 (internally connected)
  byte buffer[] = {0x40 + source};
  wire_write_buffer(buffer, sizeof(buffer));
  g_input_source = source; // Also make public available (for set_loudness)
}

void set_volume (int Volume){
  // Volume is inverse of actual 0 down to -70 db Attentuation. 
  // Conversion volume range 0x3F (min vol) - 0x00 (max vol)
  // Modify loudness for larger dynamic volume range; FIXME/ TO do: extend 0-63 range and compensate for MSB? 
  // Eventually extend using speaker attenuation and bass / treble levels up to 255 levels 
  // (or 128 levels realistically as BnW at least also skips half of the levels).  
  int inverted_volume = map(Volume, 0, 255, 63, 0); // inverted_volume is inverted and scaled as well
  switch (inverted_volume) { // Loudness acts as additional MSB (0 = disable)
    case 0  ... 52: set_loudness(3); break;
    case 53 ... 56: set_loudness(2); break;
    case 57 ... 60: set_loudness(1); break;
    case 61 ... 63: set_loudness(0); break;
    default:        set_loudness(0); break; 
  }
  Serial.print("OK: inverted_volume decimal TDA7313 = "); Serial.println(inverted_volume);
  byte buffer[] = {inverted_volume};
  wire_write_buffer(buffer, sizeof(buffer)); 
}

void set_loudness (int level){
  // Set loudness, available levels; 0-3
  // Set internal max for current limitation safety reasons
  int max_loud = 1;
  if (level >= max_loud){level = max_loud;}
  // Set loudness (level; 0 up to 3, for 0dB, +3.75dB, +7.5dB and +11.25dB respectively)
  Serial.print("OK: Loudness level decimal TDA7313 = "); Serial.println(level);
  byte buffer[] = {(0x40 + g_input_source) ^ (3 - level << 3)}; // Set loudness compensation
  wire_write_buffer(buffer, sizeof(buffer)); 
}

void set_eq (int eq_state){
  // Set predefined eq_state, eventually use set_loudness to compensate for volume correction
  Serial.print("OK: EQ state = "); Serial.println(eq_state);
  byte buffer [] = {g_audio_profile_array[eq_state][0],   // Set bass + 0x60
                    g_audio_profile_array[eq_state][1]};  // Set treble + 0x70
  wire_write_buffer(buffer, sizeof(buffer));
  g_eq_state = eq_state; // Update global state as well
}

void wire_write_buffer(byte buffer[], int size){
  // Streamlines the begin and end transmission for I2C
  // Usage: byte buffer [] = {0,1}; wire_write_buffer(buffer, sizeof(buffer))
  Wire.beginTransmission(g_adxl_address);
  Wire.write(buffer, size);
  Wire.endTransmission();
  if (g_debug){Serial.print("DEBUG: raw i2c: "); print_byte(buffer, size);}
}

void toggle_eq (){
  // Toggle to next profile in AudioProfileArray (where length defined by ArrRows)
  if (g_eq_state + 1 >= g_arr_rows){g_eq_state = 0;}
  else{g_eq_state++;}
  set_eq (g_eq_state); // Apply
  // Blink (eq_state + 1); // Blink shows profile number
}
