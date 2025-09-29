#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h>

#define ONEWIRE_PIN    2
#define ONEWIRE_BIT    (1 << ONEWIRE_PIN)
#define ONEWIRE_PORT   PORTD
#define ONEWIRE_DDR    DDRD
#define ONEWIRE_PINREG PIND
#define BUTTON_PIN A1

const uint8_t segmenti[15] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F, // 9
    0x77, // 10: E
    0x7C, // 11: r
    0x54, // 12: n
    0x5E, // 13: d  
    0x40  // 14: -
};

#define DIGIT_ON LOW
#define DIGIT_OFF HIGH
#define SEGMENT_ON HIGH
#define SEGMENT_OFF LOW

#define setOutput(port, bit) ((port) |= (1 << (bit)))
#define setInput(ddr, bit) ((ddr) &= ~(1 << (bit)))
#define enablePullup(port, bit) ((port) |= (1 << (bit)))
#define disablePullup(port, bit) ((port) &= ~(1 << (bit)))
#define readPin(pinReg, bit) ((pinReg & (1 << bit)) != 0)
#define writePin(port, bit, val) ((val) ? (port |= (1 << bit)) : (port &= ~(1 << bit)))
#define releaseLine(ddr, port, bit) do { \
  (ddr) &= ~(1 << (bit));     /* Input */ \
  (port) &= ~(1 << (bit));    /* Pull-up off */ \
} while(0)

// OneWire helpers
inline void ow_drive_low() { ONEWIRE_DDR |= ONEWIRE_BIT; ONEWIRE_PORT &= ~ONEWIRE_BIT; }
inline void ow_release()   { ONEWIRE_DDR &= ~ONEWIRE_BIT; ONEWIRE_PORT |= ONEWIRE_BIT; }
inline bool ow_read_pin()  { return (ONEWIRE_PINREG & ONEWIRE_BIT); }

// Variabili per temperatura
volatile uint8_t ow_state = 0;
volatile uint8_t ow_bitpos = 0;
volatile uint8_t ow_byte = 0;
volatile bool temperature_ready = false;
volatile bool first_conversion = true;
volatile bool lsb_done = false;
volatile uint8_t lsb, msb;
volatile uint8_t wait_for_conversion_counter = 0;
int16_t lastTemp = 0.0;
uint32_t lastCountTemp = 0;
volatile int32_t temp_Media = 0.0;
volatile uint32_t countTemp = 0;

// Variabili per display
volatile uint8_t currentDigit = 0;
volatile uint8_t displayBuffer[4] = {0,0,0,0};

// Gestione errori
volatile bool flagErroreTemp = false;
volatile bool flagMediaNd = false;

// Variabili per pulsante
volatile bool flagMedia = false;
volatile uint16_t debounceCounter = 50;

volatile uint32_t start = 10000;

volatile bool dpBuffer[4] = {false,false,false,false}; // punto decimale per ogni digit

// Forward declarations
void ow_send_byte(uint8_t b);
void ow_read_byte();
void start_conversion();

void start_conversion() {
    temperature_ready = false;
    flagErroreTemp = false;
    ow_state = 0;
    first_conversion = true;
    cli();
    // CTC mode
    TCCR1A = 0;
    TCCR1B = (1 << WGM12);
    // Prescaler / 8, so 1 tick = 0.5 us
    TCCR1B |= (1 << CS11);
    TCNT1 = 0;
    OCR1A = 480*2; // Initial delay for reset pulse (480 us)
    TIMSK1 |= (1 << OCIE1A);
    sei();
}

void setupTimer2() {
    cli();
    TCCR2A = (1 << WGM21); // CTC mode
    OCR2A = 249; // 1kHz with 16 MHz /64 prescaler
    TIMSK2 = (1 << OCIE2A); // Abilita interrupt compare match
    TCCR2B = (1 << CS22); // prescaler 64
    sei();
}


void ow_send_byte(uint8_t b) {
    ow_byte = b;
    ow_bitpos = 0;
    ow_write_Sensor();
}

void ow_read_byte() {
    ow_byte = 0;
    ow_bitpos = 0;
    ow_read_Sensor();
}

void ow_reset_Sensor(){
  Serial.println("basso");
  ow_drive_low();
  OCR1A = 480*2; // 480 us
  Serial.println("Alto");
  ow_release();
  OCR1A = 90*2; // 70 us

  if (!ow_read_pin()) {
    // Presence detected
    Serial.println("Presence detected");
    ow_send_byte(0xCC);
    OCR1A = 410*2 + 5*2; // 410 us + 5us
  } else {
    Serial.println("Errore");
    ow_fail_sensor();
  }

}

void ow_write_Sensor(){
  do{
    ow_drive_low();
    if (ow_byte & (1 << ow_bitpos)) {
      // write '1'
      OCR1A = 6*2; // 6 us
      ow_release();
      OCR1A = 2*64; // 64 us (total 70 us)
    } else {
      // write '0'
      OCR1A = 60*2; // 60 us
      ow_release();
      OCR1A = 10*2; // 10 us (total 70 us)
    }
    ow_bitpos++;
  }while(ow_bitpos < 8);
  if (ow_byte == 0xCC && first_conversion) {
    ow_send_byte(0x44); // Convert T Command
  } else if (ow_byte == 0xCC && !first_conversion) {
    ow_send_byte(0xBE); // READ Scratchpad Command
  } else if (ow_byte == 0x44) {
    // Start conversion and wait ~750 ms 
    wait_for_conversion_counter = 30 - 1;
    ow_waitConversion_Sensor();
  } else if (ow_byte == 0xBE) {
    ow_read_byte(); // Read first byte
    lsb_done = false;
  } else {
    ow_finish_Sensor();
  }

}

void ow_read_Sensor(){
  do{
    ow_drive_low();
    OCR1A = 2*2;
    ow_release();
    OCR1A = 18*2;
    if (ow_read_pin()) {
      ow_byte |= (1 << ow_bitpos);
    }
    OCR1A = 45*2;
    ow_bitpos++;
  }while(ow_bitpos < 8);

  if(!lsb_done) {
    lsb_done = true;
    lsb = ow_byte; // store read LSB
    ow_read_byte();
  } else {
    msb = ow_byte; // store read MSB
    ow_finish_Sensor();
  }

  
}

void ow_fail_sensor(){
  TIMSK1 &= ~(1 << OCIE1A);
  flagErroreTemp = true; // Still set flag to indicate process is done
}

void ow_finish_Sensor(){
  TIMSK1 &= ~(1 << OCIE1A); // stop timer
  temperature_ready = true;
}

void ow_waitConversion_Sensor(){
  if(wait_for_conversion_counter-- > 0) {
    OCR1A = 25000*2; // 25 ms
  } else {
    first_conversion = false;
    OCR1A = 400*2;
    ow_reset_Sensor();
  }
}

ISR(TIMER1_COMPA_vect) {
    switch (ow_state) {
        case 0: // Reset
            ow_reset_Sensor();
            ow_state = 1;
            break;

        case 1: // Scrittura
            ow_write_Sensor();
            ow_state = 2;
            break;

        case 2: // Lettura
            ow_read_Sensor();
            ow_state = 0; // ricomincia da capo oppure puoi mettere ow_finish_Sensor()
            break;

        case 255: // Errore
            ow_fail_sensor();
            ow_state = 0; // dopo errore ricomincia o resta fermo
            break;
    }
}
void showDigit(int valore, int posizioneDigit, bool mostraPuntoDec = false) {

    if (valore < 0) {
        // Digit spento
        writePin(PORTB, PB3, (posizioneDigit == 1) ? DIGIT_OFF : DIGIT_OFF);
        writePin(PORTB, PB4, (posizioneDigit == 2) ? DIGIT_OFF : DIGIT_OFF);
        writePin(PORTB, PB5, (posizioneDigit == 3) ? DIGIT_OFF : DIGIT_OFF);
        writePin(PORTC, PC0, (posizioneDigit == 4) ? DIGIT_OFF : DIGIT_OFF);
        return;
    }
    
    // Accendi digit
    writePin(PORTB, PB3, (posizioneDigit == 1) ? DIGIT_ON : DIGIT_OFF);
    writePin(PORTB, PB4, (posizioneDigit == 2) ? DIGIT_ON : DIGIT_OFF);
    writePin(PORTB, PB5, (posizioneDigit == 3) ? DIGIT_ON : DIGIT_OFF);
    writePin(PORTC, PC0, (posizioneDigit == 4) ? DIGIT_ON : DIGIT_OFF);

    // Segmenti
    uint8_t seg = segmenti[valore];
    writePin(PORTD, PD3, (seg & 0x01) ? SEGMENT_ON : SEGMENT_OFF);
    writePin(PORTD, PD4, (seg & 0x02) ? SEGMENT_ON : SEGMENT_OFF);
    writePin(PORTD, PD5, (seg & 0x04) ? SEGMENT_ON : SEGMENT_OFF);
    writePin(PORTD, PD6, (seg & 0x08) ? SEGMENT_ON : SEGMENT_OFF);
    writePin(PORTD, PD7, (seg & 0x10) ? SEGMENT_ON : SEGMENT_OFF);
    writePin(PORTB, PB0, (seg & 0x20) ? SEGMENT_ON : SEGMENT_OFF);
    writePin(PORTB, PB1, (seg & 0x40) ? SEGMENT_ON : SEGMENT_OFF);

    // Punto decimale
    writePin(PORTB, PB2, mostraPuntoDec ? SEGMENT_ON : SEGMENT_OFF);

    resetDisplay();
}

void calcTempPerDisplay(const int16_t temperatura, const uint32_t counterTemperature) {

  // reset buffer
  for (int i=0; i<4; i++) {
    displayBuffer[i] = 0;
    dpBuffer[i] = false;
  }

  if(flagErroreTemp){
    // Mostra "Err"
    displayBuffer[0] = -1;
    displayBuffer[1] = 10; // E
    displayBuffer[2] = 11; // r
    displayBuffer[3] = 11; // r
    return;
  }

  int16_t tempToShow = temperatura;

  if(flagMedia){
    if(counterTemperature == 0){
      // Mostra "nd"
      displayBuffer[0] = -1;
      displayBuffer[1] = -1;
      displayBuffer[2] = 12; // n
      displayBuffer[3] = 13; // d
      return;
    }

    tempToShow = temp_Media;
  }

  const bool negativo = tempToShow < 0;
  tempToShow = negativo? -tempToShow : tempToShow;

  int temp_decimi = (tempToShow * 5) ;   // da raw(0.5°C) a decimi °C

  int cent  = (temp_decimi / 1000) % 10;
  int dec   = (temp_decimi / 100) % 10;
  int unit  = (temp_decimi / 10) % 10;
  int decim = temp_decimi % 10; 

  if (tempToShow / 2 >= 100) {
    displayBuffer[0] = cent;
    displayBuffer[1] = dec;
    displayBuffer[2] = unit; 
    displayBuffer[3] = decim;
    dpBuffer[2] = true; // punto dopo le unitÃ 
  } 
  else if (tempToShow / 2 >= 10) {
    if (negativo) displayBuffer[0] = 14; // '-'
    else displayBuffer[0] = -1;           
    displayBuffer[1] = dec;
    displayBuffer[2] = unit;
    displayBuffer[3] = decim;
    dpBuffer[2] = true; // punto dopo le unitÃ 
  } 
  else { 
    displayBuffer[0] = -1;
    if (negativo) displayBuffer[1] = 14; // '-'
    else displayBuffer[1] = -1;
    displayBuffer[2] = unit;
    displayBuffer[3] = decim;
    dpBuffer[2] = true; // punto dopo le unitÃ 
  }
}

void resetDisplay() {
   writePin(PORTD, PD3, SEGMENT_OFF);
   writePin(PORTD, PD4, SEGMENT_OFF);
   writePin(PORTD, PD5, SEGMENT_OFF);
   writePin(PORTD, PD6, SEGMENT_OFF);
   writePin(PORTD, PD7, SEGMENT_OFF);
   writePin(PORTB, PB0, SEGMENT_OFF);
   writePin(PORTB, PB1, SEGMENT_OFF);
   writePin(PORTB, PB2, SEGMENT_OFF);

   writePin(PORTB, PB3, DIGIT_OFF);
   writePin(PORTB, PB4, DIGIT_OFF);
   writePin(PORTB, PB5, DIGIT_OFF);
   writePin(PORTC, PC0, DIGIT_OFF);
}
void setup() {
    Serial.begin(9600);
    setOutput(DDRD, PD3);
    setOutput(DDRD, PD4);
    setOutput(DDRD, PD5);
    setOutput(DDRD, PD6);
    setOutput(DDRD, PD7);
    setOutput(DDRB, PB0);
    setOutput(DDRB, PB1);
    setOutput(DDRB, PB2);

    setOutput(DDRB, PB3);
    setOutput(DDRB, PB4);
    setOutput(DDRB, PB5);
    setOutput(DDRC, PC0);
    resetDisplay();
    setInput(DDRC, PC1);
    disablePullup(PORTC, PC1);
    ow_release();
    setupTimer2();

}

void loop() {
}

ISR(TIMER2_COMPA_vect) {
  bool buttonPressed = false;
  bool tempReady = false;

  // Avvio temperatura ogni 10 sec
  if(start-- <= 0){
    start_conversion();
    start = 10000;
  }

  // Debouncer pulsante
  bool pressed = readPin(PINC, PC1);
  if(pressed && debounceCounter == 0) {

  } else {
    if(pressed) {
      debounceCounter--;
    } else {
      debounceCounter = 50;
    }
    if(debounceCounter == 0) {
      flagMedia = !flagMedia;
      buttonPressed = true;
    }
  }

  // Safe zone - copio valori volatile in altre variabili locali per uso esclusivo
  cli();
  if(temperature_ready) {
    int16_t raw = (msb << 8) | lsb;
    countTemp++;
    lastTemp = raw;
    lastCountTemp = countTemp;
    tempReady = true;
    temperature_ready = false;
  }
  sei();
  // Sposto il calcolo fuori dalla safe zone per avere gli interrupt disattivati per meno tempo
  if(tempReady) {
    temp_Media = temp_Media + ((lastTemp - temp_Media) / (lastCountTemp*1.0f));  
  }

  // Aggiorna cifre display solo se bottone pressed o nuova temperatura letta
  if(tempReady || buttonPressed){
    calcTempPerDisplay(lastTemp, lastCountTemp);
  }

  // Aggiorna un digit per volta
  showDigit(displayBuffer[currentDigit], currentDigit+1, dpBuffer[currentDigit]);
  currentDigit = (currentDigit + 1) % 4;
}