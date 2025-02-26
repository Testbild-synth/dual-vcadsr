// dvcasr software
#include "Mux.h"
#include <Adafruit_MCP4728.h>
#include <Wire.h>

// uses "Analog digital Multiplexers" library https://docs.arduino.cc/libraries/analog-digital-multiplexers/
// https://github.com/stechio/arduino-ad-mux-lib?tab=readme-ov-file
using namespace admux;
Mux mux(Pin(A7, INPUT, PinType::Analog), Pinset(8, 7, 6));
Adafruit_MCP4728 mcp;

// pin definitions
#define TRIG1_PIN 3
#define TRIG2_PIN 2
#define MODE1_PIN A2
#define MODE2_PIN A3
#define CRV1_PIN 10
#define CRV2_PIN 9

// possible states of AR/adsr
enum State {
  ATTACK,   
  DECAY,    
  SUSTAIN,  
  RELEASE,
  OFF, 
};

// operation mode
enum Mode {
  AR,
  ADSR,
  LONG_ADSR,
  LFO,
  RAND
};

Mode current_modes[2] = {AR,AR};
State current_states[2] = {OFF,OFF};

// internal states: duty cycle from 0 to 200. Last cycle start time is kept, as starting value for current cycle
// which is needed to calculate sustain and release steps
float last_cycle_start[2] = {0,0};
uint8_t current_cycle_positions[2] = {0,0};
float cycle_start_values[2] = {0,0};

// current values for output
float current_values[2] = {0,0};

// current trigger state
bool triggers_on[2] = {false, false};

float settings[8] = {512,512,512,512,512,512,512,512};

// constants for duty cycle
const static uint16_t LOG[200] = {
 0,  64, 93, 122,  150,  177,  203,  228,  253,  277,  299,  322,  343,  364,  384,  404,  423,  441,  459,  476,  493,  509,  524,  539,  554,  568,  582,  595,  608,  620,  632,  644,  655,  666,  677,  687,  697,  707,  716,  725,  734,  742,  751,  759,  766,  774,  781,  788,  795,  801,  808,  814,  820,  825,  831,  836,  842,  847,  852,  856,  861,  865,  870,  874,  878,  882,  886,  889,  893,  896,  900,  903,  906,  909,  912,  915,  917,  920,  923,  925,  928,  930,  932,  934,  937,  939,  941,  942,  944,  946,  948,  950,  951,  953,  954,  956,  957,  959,  960,  961,  963,  964,  965,  966,  967,  968,  969,  970,  971,  972,  973,  974,  975,  976,  977,  978,  978,  979,  980,  980,  981,  982,  982,  983,  984,  984,  985,  985,  986,  986,  987,  987,  988,  988,  988,  989,  989,  990,  990,  990,  991,  991,  991,  992,  992,  992,  993,  993,  993,  993,  994,  994,  994,  994,  995,  995,  995,  995,  995,  996,  996,  996,  996,  996,  997,  997,  997,  997,  997,  997,  997,  998,  998,  998,  998,  998,  998,  998,  998,  998,  999,  999,  999,  999,  999,  999,  999,  999,  999,  999,  999,  999,  999,  1000, 1000, 1000, 1000, 1000, 1000, 1000
};
const static uint16_t RAMP[200] = {
 0,  10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100,  105,  110,  115,  120,  125,  130,  135,  140,  145,  150,  155,  160,  165,  170,  175,  180,  185,  190,  195,  200,  205,  210,  215,  220,  225,  230,  235,  240,  245,  250,  255,  260,  265,  270,  275,  280,  285,  290,  295,  300,  305,  310,  315,  320,  325,  330,  335,  340,  345,  350,  355,  360,  365,  370,  375,  380,  385,  390,  395,  400,  405,  410,  415,  420,  425,  430,  435,  440,  445,  450,  455,  460,  465,  470,  475,  480,  485,  490,  495,  500,  505,  510,  515,  520,  525,  530,  535,  540,  545,  550,  555,  560,  565,  570,  575,  580,  585,  590,  595,  600,  605,  610,  615,  620,  625,  630,  635,  640,  645,  650,  655,  660,  665,  670,  675,  680,  685,  690,  695,  700,  705,  710,  715,  720,  725,  730,  735,  740,  745,  750,  755,  760,  765,  770,  775,  780,  785,  790,  795,  800,  805,  810,  815,  820,  825,  830,  835,  840,  845,  850,  855,  860,  865,  870,  875,  880,  885,  890,  895,  900,  905,  910,  915,  920,  925,  930,  935,  940,  945,  950,  955,  960,  965,  970,  975,  980,  985,  990,  995,  1000
};
const static uint16_t EXP[200] = {
 0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,  2,  2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,  5,  5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9,  10, 10, 10, 11, 11, 12, 12, 12, 13, 13, 14, 14, 15, 15, 16, 16, 17, 18, 18, 19, 20, 20, 21, 22, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 39, 40, 41, 43, 44, 46, 47, 49, 50, 52, 54, 56, 58, 59, 61, 63, 66, 68, 70, 72, 75, 77, 80, 83, 85, 88, 91, 94, 97, 100,  104,  107,  111,  114,  118,  122,  126,  130,  135,  139,  144,  148,  153,  158,  164,  169,  175,  180,  186,  192,  199,  205,  212,  219,  226,  234,  241,  249,  258,  266,  275,  284,  293,  303,  313,  323,  334,  345,  356,  368,  380,  392,  405,  418,  432,  446,  461,  476,  491,  507,  524,  541,  559,  577,  596,  616,  636,  657,  678,  701,  723,  747,  772,  797,  823,  850,  878,  907,  936,  967,  1000
};

// constants for quantization
const uint16_t note_values[61] = {0, 68, 136, 205, 273, 341, 410, 478, 546, 614, 682, 751, 819, 887, 956, 1024, 1092, 1160, 1228, 1297, 1365, 1433, 1502, 1570, 1638, 1706, 1774, 1843, 1911, 1979, 2048, 2116, 2184, 2252, 2320, 2389, 2457, 2525, 2594, 2662, 2730, 2798, 2866, 2935, 3003, 3071, 3140, 3208, 3276, 3344, 3412, 3481, 3549, 3617, 3686, 3754, 3822, 3890, 3958, 4027, 4095};
const uint8_t scales[8][12] = {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, //chromatic
  {0, 0, 2, 2, 4, 5, 5, 7, 7, 9, 9 , 11}, //major
  {0, 0, 2, 3, 3, 5, 5, 7, 8, 8, 10, 10}, //minor
  {0, 0, 2, 3, 3, 5, 5, 7, 7, 9, 9, 11}, //melodic minor
  {0, 0, 0, 3, 3, 5, 6, 7, 7, 9, 9, 11}, //blues
  {0, 0, 0, 3, 3, 5, 5, 7, 7, 7, 10, 10}, // pentatonic
  {0, 0, 0, 0, 0, 0, 0, 7, 7, 7, 12, 12}, //oct and quint
  {0, 0, 0, 0, 0, 0, 0, 12, 12, 12, 12, 12} // oct
}; //oct
uint8_t scale_select[2] = {0,0};
uint8_t note_select[2] = {0,0};

void check_trigs(){
  if(digitalRead(TRIG1_PIN)!=triggers_on[0]){
    triggers_on[0] = digitalRead(TRIG1_PIN);
    delay(5); // software debounce before starting new cycle
    check_cycle_trig(0);
  }
  if(digitalRead(TRIG2_PIN)!=triggers_on[1]){
    triggers_on[1] = digitalRead(TRIG2_PIN);
    delay(5); // software debounce before starting new cycle
    check_cycle_trig(1);
  }
}

// start new state in cycle and reset values
void start_cycle_state(int ch, State state){
  last_cycle_start[ch] = millis();
  current_states[ch] = state;
  current_cycle_positions[ch] = 0;
  cycle_start_values[ch] = current_values[ch];
}

void update_random_value(int ch){
  // random update logic: min value is attack pot, range is decay pot, 
  // scale is sustain pot
  scale_select[ch] = round(settings[ch+2] / 128);
  current_values[ch] = random(settings[ch], min(settings[ch]+ settings[ch+1], 4095));
  if (scale_select[ch] != 0) {
    note_select[ch] = constrain(int((current_values[ch] / 67)),0,60);
    note_select[ch] = note_select[ch] - note_select[ch] % 12  + scales[scale_select[ch]][note_select[ch] % 12];
    current_values[ch] = note_values[note_select[ch]];
  }
}

// ceck if we should trigger a new cycle depending on trigger
void check_cycle_trig(int ch){
  if(triggers_on[ch]){
    if(current_modes[ch]==RAND){
      update_random_value(ch);
    }
    else{
      start_cycle_state(ch, ATTACK);
    }
  }
}

// loop through mux channels and read all potentiometers
void read_pot_vals(){
  for(int i=0; i<8; i++){
    settings[i] = mux.read(i);
  }
}

// cycle state switching logic for AR
void switch_cycle_AR(int ch){
  if(current_states[ch] == ATTACK){
      start_cycle_state(ch, RELEASE); 
    }
    else{
      start_cycle_state(ch, OFF);
    }
}

// cycle state switching logic for LFO. 
void switch_cycle_LFO(int ch){
  if(current_states[ch] == OFF ||current_states[ch] == RELEASE){
    start_cycle_state(ch, ATTACK);
  }
  else if(current_states[ch] == ATTACK){
    start_cycle_state(ch, DECAY);
  }
  else if(current_states[ch] == DECAY){
    start_cycle_state(ch, RELEASE);
  }
}

// cycle state switch logic for ADSR envelope
void switch_cycle_ADSR(int ch){
  if(current_states[ch] == ATTACK){
    if(triggers_on[ch]){
      start_cycle_state(ch, DECAY);
    }
    else{
      start_cycle_state(ch, RELEASE);
    }
  }
  else if (current_states[ch] == DECAY){
    if(triggers_on[ch]){
      start_cycle_state(ch, SUSTAIN);
    }
    else{
      start_cycle_state(ch, RELEASE);
    }
  }
  else if(current_states[ch] == SUSTAIN){
    if(!triggers_on[ch]){
      start_cycle_state(ch, RELEASE);
    }
  }
  else{
    start_cycle_state(ch, OFF);
  }
}

// switch cycle states. called at the end of a duty cycle to trigger the next state depending on current mode
void switch_cycle(int ch){
  if(current_modes[ch] == AR){
    switch_cycle_AR(ch);
  }
  else if(current_modes[ch] == ADSR || current_modes[ch] == LONG_ADSR){
    switch_cycle_ADSR(ch);
  }
  else if(current_modes[ch] == LFO){
    switch_cycle_LFO(ch);
  }
}

// udpate modes depending on potentiometer values
void update_modes(){
  Mode new_mode1 = static_cast<Mode>(int(analogRead(MODE1_PIN) / 230));
  Mode new_mode2 = static_cast<Mode>(int(analogRead(MODE2_PIN) / 230));
  if(current_modes[0] != new_mode1){
    current_modes[0] = new_mode1;
    start_cycle_state(0, OFF);
  }
  if(current_modes[1] != new_mode2){
    current_modes[1] = new_mode2;
    start_cycle_state(1, OFF);
  }
}

// select one of the curves for values. log exp or linear
uint16_t* select_curve(){
  if(!digitalRead(CRV2_PIN)){
    return LOG;
  }
  else if(!digitalRead(CRV1_PIN)){
    return EXP;
  }
  else{
    return RAMP;
  }
}

void update_output_value(int ch){
  // random mode: skip all other update just use stored value (updated in trig)
  if(current_modes[ch] == RAND){
    return;
  }
  uint16_t *active_curve = select_curve();
  
  if(current_states[ch] == ATTACK){
    current_values[ch] = 4*active_curve[current_cycle_positions[ch]];
    return;
  }
  else if(current_states[ch] == DECAY){
    // decay: difference between top and sustain level (settings 2 and 6);
    current_values[ch] = 4 * int(settings[4*ch + 2] + (active_curve[199-current_cycle_positions[ch]] * (1024.0-settings[4*ch + 2])/1024.0));
  }
  else if(current_states[ch] == SUSTAIN){
    current_values[ch] = cycle_start_values[ch];
  }
  else if(current_states[ch] == RELEASE){
    current_values[ch] =  int(cycle_start_values[ch] * float((active_curve[199-current_cycle_positions[ch]])/1000.0));
  }
  else{
    current_values[ch] = 0;
  }
}

void update_duty_cycle(int ch){
  if(current_states[ch] == SUSTAIN && !triggers_on[ch]){
    switch_cycle(ch);
  }
  else if(current_states[ch] != OFF || current_modes[ch]==LFO){
    if(current_modes[ch] == LONG_ADSR){
       current_cycle_positions[ch] = int(20*float(millis()-last_cycle_start[ch])/float(settings[ch*4 + current_states[ch]]+20));
    }
    else{
       current_cycle_positions[ch] = int(200*float(millis()-last_cycle_start[ch])/float(settings[ch*4 + current_states[ch]]+20));
    }
    if(current_cycle_positions[ch] > 199){
      switch_cycle(ch);
    }
  }
}

void setup() {
  while (!mcp.begin()) {
    delay(1000);
  }
  pinMode(CRV1_PIN, INPUT_PULLUP);
  pinMode(CRV2_PIN, INPUT_PULLUP);
  digitalWrite(CRV1_PIN, HIGH);
  digitalWrite(CRV2_PIN, HIGH);
  mcp.setChannelValue(MCP4728_CHANNEL_A,  4000);
  mcp.setChannelValue(MCP4728_CHANNEL_B,  4000);
  delay(500);
  mcp.setChannelValue(MCP4728_CHANNEL_A,  0);
  mcp.setChannelValue(MCP4728_CHANNEL_B,  0);
  delay(500);
  mcp.setChannelValue(MCP4728_CHANNEL_A,  4000);
  mcp.setChannelValue(MCP4728_CHANNEL_B,  4000);
  delay(500);
  mcp.setChannelValue(MCP4728_CHANNEL_A,  0);
  mcp.setChannelValue(MCP4728_CHANNEL_B,  0);
  delay(500);
}

void loop() {
  read_pot_vals();
  update_modes();
  check_trigs();
  update_duty_cycle(0);
  update_duty_cycle(1);
  update_output_value(0);
  update_output_value(1);
  mcp.setChannelValue(MCP4728_CHANNEL_A,  current_values[0]);
  mcp.setChannelValue(MCP4728_CHANNEL_B, current_values[1]);
  delayMicroseconds(100);
  
}
