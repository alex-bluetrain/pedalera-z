
#include "HX711.h"
#include "Joystick.h"
#include "stdlib.h"
#include "limits.h"
#include "avr/boot.h"
#include "EEPROM.h"
#include "SevenSegmentTM1637.h"
#include "SevenSegmentFun.h"
#include "Encoder.h"

const char RELEASE_VERSION[] = "Pedalera-Z v2.1.0 by Alex Verstraeten (alex@okular.com.ar)";
const int sn=808;
int esn=1234;

// load cells
#define DT3 A10
#define CLK3 16

#define DT2 8
#define CLK2 9

#define DT1 6
#define CLK1 7

// display
#define DISPLAY_CLK 3
#define DISPLAY_DIO 2

// encoder
#define ENC_PIN_A 0
#define ENC_PIN_B 1
#define ENC_PIN_SW 4

// shifter
#define BTN_1 A3
#define BTN_2 A1
#define BTN_3 A2

#define CELDAS_COUNT 3
HX711 celdas[CELDAS_COUNT] = {
  HX711(),
  HX711(),
  HX711()
};

char pedals[CELDAS_COUNT] = {'x','y','z'};
uint8_t fallas[CELDAS_COUNT] = {0, 0, 0};

// cantidad maxima de fallas consecutivas para considerar un error y apagar la celda.
uint8_t fallas_max = 100;

//                      (hid   type                   btns hat X      Y      Z     RX     RY    RZ   rudder  throtle accel  brake  steer
Joystick_ joy = Joystick_(0x03, JOYSTICK_TYPE_GAMEPAD, 3,  0,  true, true, true, false, false,  false, false, false, false, false, false);
SevenSegmentFun display(DISPLAY_CLK, DISPLAY_DIO);
Encoder knob(ENC_PIN_A, ENC_PIN_B);

int last_displayed_value=-1;

enum DISPLAY_MODE {
  SHOW_DIGITS,
  SHOW_BARS
};

enum CMDS {
  PEDAL_STATE = 'e',
  PEDAL_SCALE = 's',
  PEDAL_FILTER = 'f',
  PEDAL_TARE = 't',
  PEDAL_MIN = '<',
  PEDAL_MAX = '>',
  PEDAL_DZ_TOP = 'u',
  PEDAL_DZ_BOTTOM = 'd',
  PEDAL_CURVE = 'c',
  LOAD = '?',
  SAVE = '!',
  OUTPUT_DATA = 'o',
  VERSION = 'v'
};

struct CalibrationData
{
  // enabled pedals
  bool throttle_enabled;
  bool brake_enabled;
  bool clutch_enabled;

  // loadcell scale (grams / hx711 value)
  float throttle_scale;
  float brake_scale;
  float clutch_scale;

  // signal filter (0-1 range)
  float throttle_filter;
  float brake_filter;
  float clutch_filter;

  // tare offset (hx711 values)
  long throttle_tare;
  long brake_tare;
  long clutch_tare;

  // limits (hx711 grams units)
  float throttle_min;
  float throttle_max;
  float brake_min;
  float brake_max;
  float clutch_min;
  float clutch_max;

  // deadzones (
  float throttle_dz_bottom;
  float throttle_dz_top;
  float brake_dz_bottom; // el freno no tiene deadzone top
  float clutch_dz_bottom;
  float clutch_dz_top;
  
  // curves
  int throttle_curve_0;
  int throttle_curve_20;
  int throttle_curve_40;
  int throttle_curve_60;
  int throttle_curve_80;
  int throttle_curve_100;
  int brake_curve_0;
  int brake_curve_20;
  int brake_curve_40;
  int brake_curve_60;
  int brake_curve_80;
  int brake_curve_100;
  int clutch_curve_0;
  int clutch_curve_20;
  int clutch_curve_40;
  int clutch_curve_60;
  int clutch_curve_80;
  int clutch_curve_100;
};

struct Calibration {
  struct CalibrationData data;
  uint32_t crc;
};

CalibrationData calib = {
  true,     // enable
  true,
  true,
  110.2,    // scale
  43.62,
  110.2,
  0,        // filter
  0,
  0,
  134133,   // tare
  20900,
  158836,
  0, 6000,  // limits
  0, 40000,
  0, 6000,
  0, 0,     // deadzones
  0,
  0, 0,
  0, 20, 40, 60, 80, 100, // curves
  0, 20, 40, 60, 80, 100,
  0, 20, 40, 60, 80, 100
};

unsigned long crc(uint8_t *buffer, uint16_t length) {
  const unsigned long crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };
  unsigned long crc = ~0L;
  for (int index = 0 ; index < length ; ++index) {
    crc = crc_table[(crc ^ buffer[index]) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (buffer[index] >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }
  return crc;
}

unsigned long startT;
unsigned long now;
unsigned int readings = 0;
unsigned int readings_per_second;
unsigned int elapsed;

// filter
// float ff = 0.5; // factor de filtrado (entre 0:pesado 1:liviano)
float new_value;
float values[CELDAS_COUNT] = {0, 0, 0};

uint16_t x;
uint16_t y;
uint16_t z;

int buttons_state[4] = {0, 0, 0, 0};
bool output_enabled = false;

DISPLAY_MODE display_mode = SHOW_DIGITS;

void process_pedal(int pedal) {
  // range               = calibration range
  // min_val to max_val  = is the new range after deadzone formula
  // curves              = remaps the value within the deadzoned range
  float dzmin, dzmax, range;
  
  // handle disabled pedals
  switch(pedal) {
    case 0:
      if (!calib.throttle_enabled) {
        values[pedal] = 0;
        joy.setXAxis(0);
        return;
      };
      break;
    case 1:
      if (!calib.brake_enabled) {
        values[pedal] = 0;
        joy.setYAxis(0);
        return;
      };
      break;
    case 2:
      if (!calib.clutch_enabled) {
        values[pedal] = 0;
        joy.setZAxis(0);
        return;
      };
      break;                  
  }
  
  // handle read errors
  bool failed = false;
  long readout = celdas[pedal].read();
  if (readout == LONG_MIN) {
    failed = true;
    fallas[pedal]++;
    if (fallas[pedal] >= fallas_max) {
      Serial.print("failure detected in pedal ");
      Serial.println(pedals[pedal]);
//      set_pedal_state(pedal, false);
      fallas[pedal] = 0;
    }
  } else {
    fallas[pedal] = 0;
  }

  // if (pedal == 1 && !failed) {
  //   show_readings_per_second();
  // }
  switch (pedal) {
    case 0:
      new_value = failed ? values[pedal] : (readout - calib.throttle_tare) / calib.throttle_scale;
      values[pedal] += (1-calib.throttle_filter) * (new_value - values[pedal]);
      range = calib.throttle_max - calib.throttle_min;
      dzmin = calib.throttle_min + (range / 100 * calib.throttle_dz_bottom);
      dzmax = calib.throttle_max - (range / 100 * calib.throttle_dz_top);
      values[pedal] = constrain(values[pedal], dzmin, dzmax);
      x = mymap(values[pedal], dzmin, dzmax, 0, 65535);
      if      (x <= 13107) x = mymap(x,     0, 13107, round(655.35*calib.throttle_curve_0),  round(655.35*calib.throttle_curve_20));
      else if (x <= 26214) x = mymap(x, 13107, 26214, round(655.35*calib.throttle_curve_20), round(655.35*calib.throttle_curve_40));
      else if (x <= 39321) x = mymap(x, 26214, 39321, round(655.35*calib.throttle_curve_40), round(655.35*calib.throttle_curve_60));
      else if (x <= 52428) x = mymap(x, 39321, 52428, round(655.35*calib.throttle_curve_60), round(655.35*calib.throttle_curve_80));
      else                 x = mymap(x, 52428, 65535, round(655.35*calib.throttle_curve_80), round(655.35*calib.throttle_curve_100));
      joy.setXAxis(x);
      break;
    case 1:
      new_value = failed ? values[pedal] : (readout - calib.brake_tare)/ calib.brake_scale; // convertir a gramos
      values[pedal] += (1-calib.brake_filter) * (new_value - values[pedal]);
      range = calib.brake_max - calib.brake_min;
      dzmin = calib.brake_min + (range / 100 * calib.brake_dz_bottom);
      dzmax = calib.brake_max;
      values[pedal] = constrain(values[pedal], dzmin, dzmax);
      y = mymap(values[pedal], dzmin, dzmax, 0, 65535);
      if      (y <= 13107) y = mymap(y,     0, 13107, round(655.35*calib.brake_curve_0),  round(655.35*calib.brake_curve_20));
      else if (y <= 26214) y = mymap(y, 13107, 26214, round(655.35*calib.brake_curve_20), round(655.35*calib.brake_curve_40));
      else if (y <= 39321) y = mymap(y, 26214, 39321, round(655.35*calib.brake_curve_40), round(655.35*calib.brake_curve_60));
      else if (y <= 52428) y = mymap(y, 39321, 52428, round(655.35*calib.brake_curve_60), round(655.35*calib.brake_curve_80));
      else                 y = mymap(y, 52428, 65535, round(655.35*calib.brake_curve_80), round(655.35*calib.brake_curve_100));
      joy.setYAxis(y);
      break;
    case 2:
      new_value = failed ? values[pedal] : (readout - calib.clutch_tare) / calib.clutch_scale; // convertir a gramos
      values[pedal] += (1-calib.clutch_filter) * (new_value - values[pedal]);
      range = calib.clutch_max - calib.clutch_min;
      dzmin = calib.clutch_min + (range / 100 * calib.clutch_dz_bottom);
      dzmax = calib.clutch_max - (range / 100 * calib.clutch_dz_top);
      values[pedal] = constrain(values[pedal], dzmin, dzmax);
      z = mymap(values[pedal], dzmin, dzmax, 0, 65535);
      if      (z <= 13107) z = mymap(z,     0, 13107, round(655.35*calib.clutch_curve_0),  round(655.35*calib.clutch_curve_20));
      else if (z <= 26214) z = mymap(z, 13107, 26214, round(655.35*calib.clutch_curve_20), round(655.35*calib.clutch_curve_40));
      else if (z <= 39321) z = mymap(z, 26214, 39321, round(655.35*calib.clutch_curve_40), round(655.35*calib.clutch_curve_60));
      else if (z <= 52428) z = mymap(z, 39321, 52428, round(655.35*calib.clutch_curve_60), round(655.35*calib.clutch_curve_80));
      else                 z = mymap(z, 52428, 65535, round(655.35*calib.clutch_curve_80), round(655.35*calib.clutch_curve_100));
      joy.setZAxis(z);
      break;
  }
}

void show_readings_per_second() {
  now = millis();
  if (now - startT >= 1000) {
    elapsed = now - startT;
    startT = now;
    readings_per_second=readings;
    readings=0;
  }
  readings++;
  Serial.print(readings_per_second);
  Serial.print("\n");
  // displayShow(readings_per_second);
}

const byte DATA_MAX_SIZE = 32;
char data[DATA_MAX_SIZE];
bool new_data = false;

//==================================
//    UNIQUE ID
//==================================
void showUniqueId() {
  delay(10000);
  for( int i=0; i<256; i++)
  {
    if( i%16 == 0)
    {
      char buffer[20];
      Serial.println();
      sprintf( buffer, "%5d   ", i);
      Serial.print( buffer);
    }
    else
    {
      Serial.print(" ");
    }

    int x = boot_signature_byte_get( i);
    if( x < 0x10)
      Serial.print( "0");
    Serial.print( x, HEX);
  }
  Serial.println();
}


//=============================================================================================
//                         SETUP
//=============================================================================================
void setup() {
  
  pinMode(ENC_PIN_SW, INPUT_PULLUP);
  pinMode(BTN_1, INPUT_PULLUP);
  pinMode(BTN_2, INPUT_PULLUP);
  pinMode(BTN_3, INPUT_PULLUP);

  display.begin();
  display.setBacklight(30); 
  factory_reset();
  load_calib();
  knob.write(round(calib.brake_max/1000)*4);
  Serial.begin(115200);

  joy.begin();
  joy.setXAxisRange(0, 65535);
  joy.setYAxisRange(0, 65535);
  joy.setZAxisRange(0, 65535);

  celdas[0].begin(DT1, CLK1);
  celdas[1].begin(DT2, CLK2);
  celdas[2].begin(DT3, CLK3);

  celdas[0].set_scale(calib.throttle_scale);
  celdas[1].set_scale(calib.brake_scale);
  celdas[2].set_scale(calib.clutch_scale);

  celdas[0].set_offset(calib.throttle_tare);
  celdas[1].set_offset(calib.brake_tare);
  celdas[2].set_offset(calib.clutch_tare);
  

  startT = millis();
  
//  showUniqueId();
}

// si mantengo el encoder apretado 3 segundos en el booteo, resetea la eeprom al default
void factory_reset() {
  bool should_reset = true;
  for (int i=0; i<30; i++) {
    if (digitalRead(ENC_PIN_SW)) {
      should_reset = false;
      break;
    }
    delay(100);
  }
  if (should_reset) {
    save_calib();
    display.snake(3);
    last_displayed_value=-1;
  }
}

//=============================================================================================
//                         LOOP
//=============================================================================================
void loop() {
  
  // -----sn validation-----------
  if (esn==1234) EEPROM.get(666, esn);
  if (esn != sn) {
    Serial.print("invalid device:");
    Serial.println(esn);
    delay(5000);
    return;
  }
  // ----sn validation end-------
  process_pedal(0);
  process_pedal(1);
  process_pedal(2);
  process_buttons();
  
  joy.sendState();
  receiveCommands();
  output_axis();
  process_knob();
  process_display();
//  show_readings_per_second();
}


void output_axis() {
  if (!output_enabled) return;
  Serial.print("<");
  Serial.print(values[0]);
  Serial.print(" ");
  Serial.print(values[1]);
  Serial.print(" ");
  Serial.print(values[2]);

  // Serial.print(" ");
  // Serial.print( calib.throttle_enabled ? x : '0');
  // Serial.print(" ");
  // Serial.print( calib.brake_enabled ? y : '0');
  // Serial.print(" ");
  // Serial.print( calib.clutch_enabled ? z : '0');
  Serial.print(">\n");
}

//=============================================================================================
//                         SERIAL PROTOCOL
//=============================================================================================
void receiveCommands() {
  if (Serial.available() <= 0) return;

  char b;
  int idx = 0;

  memset(data, 0, sizeof(data));

  while (Serial.available() > 0) {
    b = Serial.read();

    if (b == '\n') {
      data[idx] = '\0';
      new_data = true;
      onSerialData();
      return;
    }

    data[idx] = b;
    idx++;

    if (idx >= DATA_MAX_SIZE) {
      break;
    }
  }
  memset(data, 32, sizeof(data));
}

void onSerialData() {
  char cmd = data[0];
  char pedal = data[1];
  char value = data[2];
  bool has_value;

  if (cmd == '\n' || cmd == '\0') return;

  if ( cmd != PEDAL_STATE && 
       cmd != PEDAL_SCALE && 
       cmd != PEDAL_FILTER && 
       cmd != PEDAL_TARE && 
       cmd != PEDAL_MIN && 
       cmd != PEDAL_MAX && 
       cmd != PEDAL_DZ_TOP && 
       cmd != PEDAL_DZ_BOTTOM && 
       cmd != PEDAL_CURVE &&
       cmd != LOAD &&
       cmd != SAVE &&
       cmd != OUTPUT_DATA && 
       cmd != VERSION) 
  {
    Serial.println("err: invalid cmd");
    return;
  }
  
  if (cmd == OUTPUT_DATA || cmd == VERSION) {
    has_value = data[1] != '\0';
  } else {
    has_value = data[2] != '\0';
  }
  
  if (cmd != LOAD &&
      cmd != SAVE &&
      cmd != OUTPUT_DATA && 
      cmd != VERSION && 
      cmd != PEDAL_TARE && 
      pedal != 'x' && 
      pedal != 'y' && 
      pedal != 'z')
  {
    Serial.println("err: invalid pedal.");
    return;
  }

  if (cmd == PEDAL_TARE && 
      !has_value && 
      pedal != 'x' && 
      pedal != 'y' && 
      pedal != 'z') 
  {
    Serial.println("err: invalid pedal..");
    return;
  }

  if (cmd == PEDAL_TARE && 
      value == 'n' && 
      pedal != 'x' && 
      pedal != 'y' && 
      pedal != 'z' && 
      pedal != 'a') 
  {
    Serial.println("err: invalid pedal...");
    return;
  }

  switch (cmd) {
    case PEDAL_STATE:
      if (has_value) {
        bool state = data[2] != '0';
        set_pedal_state(pedal, state);
      } else {
        get_pedal_state(pedal);
      }
      break;
    case PEDAL_SCALE:
      if (has_value) {
        float scale = atof(&data[2]);
        set_pedal_scale(pedal, scale);
      } else {
        get_pedal_scale(pedal);
      }
      break;
    case PEDAL_FILTER:
      if (has_value) {
        float filter = atof(&data[2]);
        set_pedal_filter(pedal, filter);
      } else {
        get_pedal_filter(pedal);
      }
      break;
    case PEDAL_TARE:
      if (has_value) {
        if (value == 'n') {
          tare_pedal_now(pedal);
        } else {
          long offset = atol(&data[2]);
          set_pedal_tare(pedal, offset);
        }
      } else {
        get_pedal_tare(pedal);
      }
      break;
    case PEDAL_MIN:
      if (has_value) {
        float grams = atof(&data[2]);
        set_pedal_min(pedal, grams);
      } else {
        get_pedal_min(pedal);
      }
      break;
    case PEDAL_MAX:
      if (has_value) {
        float grams = atof(&data[2]);
        set_pedal_max(pedal, grams);
      } else {
        get_pedal_max(pedal);
      }
      break;
    case PEDAL_DZ_TOP:
      if (has_value) {
        float dz = atof(&data[2]);
        set_pedal_dz_top(pedal, dz);
      } else {
        get_pedal_dz_top(pedal);
      }
      break;
    case PEDAL_DZ_BOTTOM:
      if (has_value) {
        float dz = atof(&data[2]);
        set_pedal_dz_bottom(pedal, dz);
      } else {
        get_pedal_dz_bottom(pedal);
      }
      break;
    case PEDAL_CURVE:
      if (has_value) {
        char *token;
        token = strtok(data," "); // skip the cmd and pedal
        if (token == NULL) break;
        
        token = strtok(NULL," "); // read c0 
        if (token == NULL) break;
        int c0 = atoi(token);
        
        token = strtok(NULL," "); // read c20
        if (token == NULL) break;
        int c20 = atoi(token);

        token = strtok(NULL," "); // read c40
        if (token == NULL) break;
        int c40 = atoi(token);

        token = strtok(NULL," "); // read c60
        if (token == NULL) break;
        int c60 = atoi(token);

        token = strtok(NULL," "); // read c80
        if (token == NULL) break;
        int c80 = atoi(token); 

        token = strtok(NULL," "); // read c100
        if (token == NULL) break;
        int c100 = atoi(token);

        set_pedal_curve(pedal, c0, c20, c40, c60, c80, c100);
      } else {
        get_pedal_curve(pedal);
      }
      break;
    case SAVE:
      save_calib();
      break;
    case LOAD:
      load_calib();
      break;
    case OUTPUT_DATA:
      if (has_value) {
        bool state = data[1] != '0';
        set_output_state(state);
      } else {
        get_output_state();
      }
      break;
    case VERSION:
      get_version();
      break;
  }
}

//=============================================================================================
//                         SERIAL PROTOCOL HANDLERS
//=============================================================================================

void get_version() {
  Serial.println(RELEASE_VERSION);
}

void set_output_state(bool state) {
  output_enabled = state;
  get_output_state();
}

void get_output_state() {
  if (output_enabled) {
    Serial.println("output enabled"); 
  } else {
    Serial.println("output disabled");
  }
}

void set_pedal_scale(char pedal, float scale){
  if (pedal == 'x') {
    calib.throttle_scale = scale;
    celdas[0].set_scale(scale);
  }
  else if (pedal == 'y') {
    calib.brake_scale = scale;
    celdas[1].set_scale(scale);
  }
  else if (pedal == 'z') {
    calib.clutch_scale = scale;
    celdas[2].set_scale(scale);    
  }
  get_pedal_scale(pedal);
}

void get_pedal_scale(char pedal) {
  float scale;
  if      (pedal == 'x') scale = calib.throttle_scale;
  else if (pedal == 'y') scale = calib.brake_scale;
  else if (pedal == 'z') scale = calib.clutch_scale;
  Serial.print("scale ");
  Serial.print(pedal);
  Serial.print(" ");
  Serial.println(scale);  
}

void set_pedal_state(char pedal, bool state) {
  if      (pedal == 'x') { calib.throttle_enabled = state; get_pedal_state(pedal); }
  else if (pedal == 'y') { calib.brake_enabled = state; get_pedal_state(pedal); }
  else if (pedal == 'z') { calib.clutch_enabled = state; get_pedal_state(pedal); }
}

void set_pedal_state(int pedal, bool state) {
  if (pedal <0 || pedal >2) return;
  if      (pedal == 0) { calib.throttle_enabled = state; get_pedal_state('x'); }
  else if (pedal == 1) { calib.brake_enabled = state; get_pedal_state('y'); }
  else if (pedal == 2) { calib.clutch_enabled = state; get_pedal_state('z'); }
}

void get_pedal_state(char pedal) {
  if (pedal == 'x') {
    Serial.println( (calib.throttle_enabled) ? "state x enabled" : "state x disabled");
  }
  else if (pedal == 'y') {
    Serial.println( (calib.brake_enabled) ? "state y enabled" : "state y disabled");
  }
  else if (pedal == 'z') {
    Serial.println( (calib.clutch_enabled) ? "state z enabled" : "state z disabled");
  }
  
}

void set_pedal_filter(char pedal, float filter) {
  if      (pedal == 'x') calib.throttle_filter = filter;
  else if (pedal == 'y') calib.brake_filter = filter;
  else if (pedal == 'z') calib.clutch_filter = filter;
  get_pedal_filter(pedal);
}

void get_pedal_filter(char pedal) {
  float filter;
  if      (pedal == 'x') filter = calib.throttle_filter;
  else if (pedal == 'y') filter = calib.brake_filter;
  else if (pedal == 'z') filter = calib.clutch_filter;
  Serial.print("filter ");
  Serial.print(pedal);
  Serial.print(" ");
  Serial.println(filter);
}

void tare_pedal_now(char pedal) {
  long max_value;
  long current_value;
  int samples = 25; // 50 samples = 1000 ms aprox
  
  if (pedal == 'x' || pedal == 'a') {
    max_value = LONG_MIN;
    if (calib.throttle_enabled) {
      for (int i=0; i<samples; i++) {
        max_value = max(max_value, celdas[0].read());
      }
    } else {
      max_value = 0;
    }
    calib.throttle_tare = max_value;
    celdas[0].set_offset(max_value);
    get_pedal_tare('x');
  }
  if (pedal == 'y' || pedal == 'a') {
    max_value = LONG_MIN;
    if (calib.brake_enabled) {
      for (int i=0; i<samples; i++) {
        current_value = celdas[1].read();
        max_value = max(max_value, current_value);
        Serial.println(max_value);
      }
    } else {
      max_value = 0;
    }
    calib.brake_tare = max_value;
    celdas[1].set_offset(max_value);
    get_pedal_tare('y');
  }
  if (pedal == 'z' || pedal == 'a') {
    max_value = LONG_MIN;
    if (calib.clutch_enabled) {
      for (int i=0; i<samples; i++) {
        max_value = max(max_value, celdas[2].read());
      }
    } else {
      max_value = 0;
    }
    calib.clutch_tare = max_value;
    celdas[2].set_offset(max_value);
    get_pedal_tare('z');
  }  
}
void set_pedal_tare(char pedal, long offset) {
  if (pedal == 'x' || pedal == 'a') {
    calib.throttle_tare = offset;
    celdas[0].set_offset(offset);
  }
  if (pedal == 'y' || pedal == 'a') {
    calib.brake_tare = offset;
    celdas[1].set_offset(offset);
  }
  if (pedal == 'z' || pedal == 'a') {
    calib.clutch_tare = offset;
    celdas[2].set_offset(offset);
  }
  if (pedal == 'x') get_pedal_tare('x');
  if (pedal == 'y') get_pedal_tare('y');
  if (pedal == 'z') get_pedal_tare('z');
  if (pedal == 'a') {
    get_pedal_tare('x');
    get_pedal_tare('y');
    get_pedal_tare('z');
  }
}

void get_pedal_tare(char pedal) {
  long offset;
  if      (pedal == 'x') offset = calib.throttle_tare;
  else if (pedal == 'y') offset = calib.brake_tare;
  else if (pedal == 'z') offset = calib.clutch_tare;
  Serial.print("tare ");
  Serial.print(pedal);
  Serial.print(" ");
  Serial.println(offset);
}


// obsoleto, se usa tare y deadzone solamente, forzado a 0
void set_pedal_min(char pedal, float grams) {
  if      (pedal == 'x') calib.throttle_min = 0;
  else if (pedal == 'y') calib.brake_min = 0;
  else if (pedal == 'z') calib.clutch_min = 0;
  get_pedal_min(pedal);
}

void get_pedal_min(char pedal) {
  float grams;
  if      (pedal == 'x') grams = calib.throttle_min;
  else if (pedal == 'y') grams = calib.brake_min;
  else if (pedal == 'z') grams = calib.clutch_min;
  Serial.print("min ");
  Serial.print(pedal);
  Serial.print(" ");
  Serial.println(grams);
}

void set_pedal_max(char pedal, float grams) {
  if      (pedal == 'x') calib.throttle_max = grams;
  else if (pedal == 'y') {calib.brake_max = grams; knob.write(round(calib.brake_max/1000)*4);}
  else if (pedal == 'z') calib.clutch_max = grams;
  get_pedal_max(pedal);
}

void get_pedal_max(char pedal) {
  float grams;
  if      (pedal == 'x') grams = calib.throttle_max;
  else if (pedal == 'y') grams = calib.brake_max;
  else if (pedal == 'z') grams = calib.clutch_max;
  Serial.print("max ");
  Serial.print(pedal);
  Serial.print(" ");
  Serial.println(grams);
}

void get_pedal_dz_top(char pedal) {
  float dz;
  if      (pedal == 'x') dz = calib.throttle_dz_top;
  else if (pedal == 'y') return; // el freno no tiene deadzone top
  else if (pedal == 'z') dz = calib.clutch_dz_top;
  Serial.print("dz-top ");
  Serial.print(pedal);
  Serial.print(" ");
  Serial.println(dz);
}

void get_pedal_dz_bottom(char pedal) {
  float dz;
  if      (pedal == 'x') dz = calib.throttle_dz_bottom;
  else if (pedal == 'y') dz = calib.brake_dz_bottom;
  else if (pedal == 'z') dz = calib.clutch_dz_bottom;
  Serial.print("dz-bottom ");
  Serial.print(pedal);
  Serial.print(" ");
  Serial.println(dz);
}

void set_pedal_dz_top(char pedal, float dz) {
  if      (pedal == 'x') calib.throttle_dz_top = dz;
  else if (pedal == 'y') return; // el freno no tiene deadzone top
  else if (pedal == 'z') calib.clutch_dz_top = dz;
  get_pedal_dz_top(pedal);
}

void set_pedal_dz_bottom(char pedal, float dz) {
  if      (pedal == 'x') calib.throttle_dz_bottom = dz;
  else if (pedal == 'y') calib.brake_dz_bottom = dz;
  else if (pedal == 'z') calib.clutch_dz_bottom = dz;
  get_pedal_dz_bottom(pedal);
}

void set_pedal_curve(char pedal, int c0, int c20, int c40, int c60, int c80, int c100) {
  if (pedal == 'x') {
    calib.throttle_curve_0   = c0;
    calib.throttle_curve_20  = c20;
    calib.throttle_curve_40  = c40;
    calib.throttle_curve_60  = c60;
    calib.throttle_curve_80  = c80;
    calib.throttle_curve_100 = c100;
  } else if (pedal == 'y') {
    calib.brake_curve_0   = c0;
    calib.brake_curve_20  = c20;
    calib.brake_curve_40  = c40;
    calib.brake_curve_60  = c60;
    calib.brake_curve_80  = c80;
    calib.brake_curve_100 = c100;
  } else if (pedal == 'z') {
    calib.clutch_curve_0   = c0;
    calib.clutch_curve_20  = c20;
    calib.clutch_curve_40  = c40;
    calib.clutch_curve_60  = c60;
    calib.clutch_curve_80  = c80;
    calib.clutch_curve_100 = c100;
  }
  // get_pedal_curve(pedal);
}

void get_pedal_curve(char pedal) {
  if (pedal == 'x') {
    Serial.print("curve ");
    Serial.print(pedal);
    Serial.print(" "); Serial.print(calib.throttle_curve_0);
    Serial.print(" "); Serial.print(calib.throttle_curve_20);
    Serial.print(" "); Serial.print(calib.throttle_curve_40);
    Serial.print(" "); Serial.print(calib.throttle_curve_60);
    Serial.print(" "); Serial.print(calib.throttle_curve_80);
    Serial.print(" "); Serial.print(calib.throttle_curve_100);
    Serial.print("\n");
  } else if (pedal == 'y') {
    Serial.print("curve ");
    Serial.print(pedal);
    Serial.print(" "); Serial.print(calib.brake_curve_0);
    Serial.print(" "); Serial.print(calib.brake_curve_20);
    Serial.print(" "); Serial.print(calib.brake_curve_40);
    Serial.print(" "); Serial.print(calib.brake_curve_60);
    Serial.print(" "); Serial.print(calib.brake_curve_80);
    Serial.print(" "); Serial.print(calib.brake_curve_100);
    Serial.print("\n");
  } else if (pedal == 'z') {
    Serial.print("curve ");
    Serial.print(pedal);
    Serial.print(" "); Serial.print(calib.clutch_curve_0);
    Serial.print(" "); Serial.print(calib.clutch_curve_20);
    Serial.print(" "); Serial.print(calib.clutch_curve_40);
    Serial.print(" "); Serial.print(calib.clutch_curve_60);
    Serial.print(" "); Serial.print(calib.clutch_curve_80);
    Serial.print(" "); Serial.print(calib.clutch_curve_100);
    Serial.print("\n");
  }
}

void save_calib() {
  Calibration toEEPROM;
  toEEPROM.data = calib;
  // save all pedal state to enabled 
//  toEEPROM.data.throttle_enabled = true;
//  toEEPROM.data.brake_enabled = true;
//  toEEPROM.data.clutch_enabled = true;

  toEEPROM.crc = crc((uint8_t*)&toEEPROM.data, sizeof(calib));
  EEPROM.put(0, toEEPROM);
  Serial.println("calibration saved");
  display.snake(1);
  last_displayed_value=-1;
}

void load_calib() {
  // reads calib from eeprom 
  Calibration fromEEPROM;
  EEPROM.get(0, fromEEPROM);

  // if crc is valid, use the new calibration
  uint32_t eeprom_crc = crc((uint8_t*)&fromEEPROM.data, sizeof(fromEEPROM.data));
  if (eeprom_crc == fromEEPROM.crc) {
    calib = fromEEPROM.data;
  } else {
    Serial.println("calibration load error, using defaults");
    display.snake(5);
    last_displayed_value=-1;
  }

  // notify the UI (via serial port)
  for (int i=0; i<3; i++) {
    get_pedal_curve(pedals[i]);
    get_pedal_dz_bottom(pedals[i]);
    get_pedal_dz_top(pedals[i]);
    get_pedal_max(pedals[i]);
    get_pedal_min(pedals[i]);
    get_pedal_tare(pedals[i]);
  }
  
  // apply the new values
  celdas[0].set_offset(calib.throttle_tare);
  celdas[1].set_offset(calib.brake_tare);
  celdas[2].set_offset(calib.clutch_tare);
}

void displayShow(int val) {
  char buffer[4]; 
  sprintf(buffer, "%03d", (int)val);
  if (buffer[0]=='0' && buffer[1] == '0')  {
    buffer[0] = ' ';
    buffer[1] = ' ';
  }
  if (buffer[0]=='0') buffer[0] = ' ';

  if (last_displayed_value != val) {
    display.clear();
    display.print(buffer);
    display.setColonOn(0);
    last_displayed_value = val;
  }
}

void displayBars(float val) {
  byte levels[4];
  for (int i=0; i<4; i++) {
    if (val > 0 && val <= 25) {
        levels[0] = map(val, 0, 25, 0, 100);
        levels[1] = 0;
        levels[2] = 0;
        levels[3] = 0;
    }
    if (val >25 && val <= 50) {
        levels[0] = 100;
        levels[1] = map(val, 25, 50, 0, 100);
        levels[2] = 0;
        levels[3] = 0;
    }
    if (val >50 && val <= 75) {
        levels[0] = 100;
        levels[1] = 100;
        levels[2] = map(val, 50, 75, 0, 100);
        levels[3] = 0;
    }
    if (val >75 && val <= 100) {
        levels[0] = 100;
        levels[1] = 100;
        levels[2] = 100;
        levels[3] = map(val, 75, 100, 0, 100);
    }
  }
  display.printLevelHorizontal(levels);
}

void process_knob() {
  // knob turn
  long newMax = knob.read()/4;
  if (newMax < 1) {
    newMax =1;
    knob.write(1*4);
  }
  if (newMax > 100) {
    newMax = 100;
    knob.write(100*4);
  }
  if (newMax != round(calib.brake_max/1000)) {
    display.setBacklight(30);
    calib.brake_max = newMax*1000;
    get_pedal_max('y'); // to update UI
  }
}

void process_buttons() {
  int state_btn_1 = !digitalRead(BTN_1);
  int state_btn_2 = !digitalRead(BTN_2);
  int state_btn_3 = !digitalRead(BTN_3);
  int state_btn_4 = !digitalRead(ENC_PIN_SW);
  
  if (state_btn_1 != buttons_state[0]) {
    joy.setButton(0, state_btn_1);
    buttons_state[0] = state_btn_1;
  }
  if (state_btn_2 != buttons_state[1]) {
    joy.setButton(1, state_btn_2);
    buttons_state[1] = state_btn_2;
  }
  if (state_btn_3 != buttons_state[2]) {
    joy.setButton(2, state_btn_3);
    buttons_state[2] = state_btn_3;
  }
  // special case for knob button:
  if (state_btn_4 != buttons_state[3]) {
    if (state_btn_4) 
      on_knob_push();
    else 
      on_knob_release();
    buttons_state[3] = state_btn_4;
  }
}

void process_display() {
  if (display_mode == SHOW_DIGITS) {
    displayShow(round(calib.brake_max/1000));
  } else {
    float porcentaje = mymap(y, 0, 65535, 0, 100);
    displayBars(porcentaje);
    Serial.print(y);
    Serial.print(" ");
    Serial.println(porcentaje);
  }
}

void on_knob_push() {
  //display_mode = (display_mode == SHOW_DIGITS) ? SHOW_BARS : SHOW_DIGITS;
  display_mode = SHOW_DIGITS;
  display.setBacklight(30);
  save_calib();  
  last_displayed_value=-1;
}

void on_knob_release() {
}
