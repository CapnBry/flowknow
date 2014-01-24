#include <avr/sleep.h>
#include <util/atomic.h>

#include "LiquidCrystal.h"

//#define CONFIG_TONE_OUTPUT 1
//#define SERIAL_INTERFACE 1

#define T1FREQ        (F_CPU / 256.0f)
// Measured actual VCC
#define VCC           5.062f
#define APIN_BATTERY  0
#define DPIN_TONE     2
#define DPIN_CNTRST   5
#define DPIN_BKLT     6
#define DPIN_FLOW     8

LiquidCrystal lcd(13, A1, A2, A3, A4, A5);

#include "floatprint.h"
static FloatPrint<10> fp;

static char g_SerialBuff[40];
static float g_Liters;
static uint32_t g_RunningTime;
static float g_FixedScale;
static uint16_t g_Battery;

#define LOOP_PERIOD     333
#define LOOPCNT_SENSOR  (1000/LOOP_PERIOD)
#define LOOPCNT_LCD     2
#define LOOP_SMOOTH     2.0f

#define LITERS_PER_GALLON 0.264172f
#define SENSOR_HZ_PER_LPM 7.5f

#define ARRAYSIZE(a) (sizeof(a) / sizeof(*(a)))
#define SCALE_LEVEL_CNT (ARRAYSIZE(SCALE_LEVELS))

static struct tagScaleLevels
{
  uint16_t maxTicks;
  float scale;
} SCALE_LEVELS[] = {
  // Measure 1L of liquid as indicated by the total L count
  // If more than 1L adjust the scale HIGHER
  // If less than 1L adjust the scale LOWER
  { 12500, 1.302f },  // 5Hz     0.667 LPM
  { 8333, 1.201f },   // 7.5Hz   1 LPM
  { 5556, 1.150f },   // 11.25Hz 1.5 LPM
  { 4167, 1.105f },   // 15Hz    2 LPM
  { 2778, 1.093f },   // 22.5Hz  3 LPM
  //{ 2083, 1.093f },   // 30Hz    4 LPM
  //{ 1667, 1.093f },   // 37.5Hz  5 LPM
};

static struct timerInfo
{
  uint16_t totalTime;
  uint8_t cnt;
} g_TimerInfo;

ISR(TIMER1_CAPT_vect)
{
  TCNT1 = 0;
  // If there has been an overflow since the last capture, the period is longer
  // than 1 second so we're not interested in such slowness
  if (bit_is_set(TIFR1, TOV1))
  {
    bitSet(TIFR1, TOV1); // clear overflow
    return;
  }
  g_TimerInfo.totalTime += ICR1;
  ++g_TimerInfo.cnt;
}

#if SERIAL_INTERFACE
static void Serial_nl(void)
{
  Serial.write('\n');
  Serial.flush();
}
#endif /* SERIAL_INTERFACE */

static void calcExpMovingAverage(const float smooth, float *currAverage, float newValue)
{
  if (isnan(*currAverage))
    *currAverage = newValue;
  else
  {
    newValue = newValue - *currAverage;
    *currAverage = *currAverage + (smooth * newValue);
  }
}

static void lcd_printTime(uint32_t t)
{
  // We only print 8 characters so if greater than 99:59:59 display overflow
  if (t >= 360000)
  {
    lcd.print(">100 hrs");
    return;
  }
  uint8_t h = t / 3600UL;
  uint8_t m = (t - (h * 3600UL)) / 60U;
  uint8_t s = t - (h * 3600UL) - (m * 60U);
  /*
  char buf[9];
  sprintf(buf, "%2u:%2.2u:%2.2u", h, m, s);
  lcd.print(buf);
  */

  if (h < 10)
    lcd.print(' ');
  lcd.print(h, DEC);
  lcd.write(':');
  if (m < 10)
    lcd.print('0');
  lcd.print(m, DEC);
  lcd.print(':');
  if (s < 10)
    lcd.print('0');
  lcd.print(s, DEC);
}

static void sleep(uint16_t ms)
{
  //delay(ms); return;
  uint32_t start = millis();

  set_sleep_mode(SLEEP_MODE_IDLE);
  while (millis() - start < ms)
    sleep_mode();
}

static void setTone(uint16_t freq)
{
#if CONFIG_TONE_OUTPUT
  if (freq == 0)
    noTone(DPIN_TONE);
  else
    tone(DPIN_TONE, freq);
  #if SERIAL_INTERFACE
    Serial.print("Tone set: "); Serial.print(freq, DEC); Serial_nl();
  #endif
#endif /* CONFIG_TONE_OUTPUT */
}

static void setHzScaleLevel(float val)
{
  g_FixedScale = val;
}

static void resetCounts(void)
{
  g_Liters = 0.0;
}

#if SERIAL_INTERFACE
void csvParseI(char *vals, void (*c)(unsigned char idx, int val))
{
  unsigned char idx = 0;
  while (*vals)
  {
    if (*vals == ',')
    {
      ++idx;
      ++vals;
    }
    else
    {
      int val = atoi(vals);
      c(idx, val);
      while (*vals && *vals != ',')
        ++vals;
    }
  }
}

static void handleCommandUrl(char *URL)
{
  if (strcmp(URL, "r") == 0)
    resetCounts();
  else if (strncmp(URL, "t=", 2) == 0)
    setTone(atoi(URL+2));
  else if (strncmp(URL, "scale=", 6) == 0)
    setHzScaleLevel(atof(URL+6));
}

static void serial_update(void)
{
  unsigned char len = strlen(g_SerialBuff);
  while (Serial.available())
  {
    char c = Serial.read();
    // support CR, LF, or CRLF line endings
    if (c == '\n' || c == '\r')
    {
      if (len != 0 && g_SerialBuff[0] == '/')
        handleCommandUrl(&g_SerialBuff[1]);
      len = 0;
    }
    else {
      g_SerialBuff[len++] = c;
      // if the buffer fills without getting a newline, just reset
      if (len >= sizeof(g_SerialBuff))
        len = 0;
    }
    g_SerialBuff[len] = '\0';
  }  /* while Serial */
}
#endif /* SERIAL_INTERFACE */

static void lcd_MeasureBattery(void)
{
  g_Battery = analogRead(APIN_BATTERY);

  // Assumes a 1/2 divider on the input
  float v = (2.0f * VCC / 1023.0f) * g_Battery;
  lcd.print("Batt ");
  fp.print(lcd, v, 4, 2);
  lcd.print('V');
#if SERIAL_INTERFACE
  Serial.print("Battery: "); Serial.print(v, 2); Serial_nl();
#endif

  // Disable ADC
  bitClear(ADCSRA, ADEN);
  bitSet(PRR, PRADC);
}

static void lcd_updateAnim(void)
{
  lcd.setCursor(0, 3);
  if (millis() < 5000)
  {
    if (g_Battery == 0) lcd_MeasureBattery();
    return;
  }
  
  uint8_t pos = (uint16_t)(g_Liters * 10) % 10;
  for (uint8_t i=0; i<10; ++i)
  {
    if (i == pos)
      lcd.write(' ');
    else
      lcd.write('\xff');
  }
}

static float lerp(float a, float b, float pct)
{
  return a + ((b - a) * pct);
}

static float getHzScale(uint16_t t1ticks)
{
  if (g_FixedScale != 0.0f)
    return g_FixedScale;

  if (t1ticks >= SCALE_LEVELS[0].maxTicks)
    return SCALE_LEVELS[0].scale;

  for (uint8_t i=1; i<SCALE_LEVEL_CNT; ++i)
  {
    if (t1ticks < SCALE_LEVELS[i].maxTicks)
      continue;
    float pct = (float)(t1ticks - SCALE_LEVELS[i].maxTicks) / 
      (float)(SCALE_LEVELS[i-1].maxTicks - SCALE_LEVELS[i].maxTicks);

    return lerp(SCALE_LEVELS[i].scale, SCALE_LEVELS[i-1].scale, pct);
  }

  return SCALE_LEVELS[SCALE_LEVEL_CNT-1].scale;
}

void setup()
{
  // Disable Analog Comparator
  ACSR = bit(ACD);
  // Disable Digital Input on ADC pins
  DIDR0 = bit(ADC5D) | bit(ADC4D) | bit(ADC3D) | bit(ADC2D) | bit(ADC1D) | bit(ADC0D);
  // PRR unused units
  PRR = bit(PRTWI) | bit(PRTIM2) | bit(PRSPI) | bit(PRUSART0); // | bit(PRADC);
#if CONFIG_TONE_OUTPUT
  bitClear(PRR, PRTIM2);
#endif

#if SERIAL_INTERFACE
  bitClear(PRR, PRUSART0);
  Serial.begin(38400);
  Serial.print("$UCID,flow," __DATE__ " " __TIME__); Serial_nl();
#endif
  lcd.begin(20, 4);
  lcd.print("\xff\xff Flow \xff\xff\xff Total \xff\xff");
  analogWrite(DPIN_BKLT, 40);
  analogWrite(DPIN_CNTRST, 25);

  pinMode(DPIN_FLOW, INPUT_PULLUP);

  // Timer1 Normal operation
  TCCR1A = 0;
  // Input Capture noise canceller | rising edge | CS12=256 prescaler = 62.5k/sec
  TCCR1B = bit(ICNC1) | bit(ICES1) | bit(CS12);
  // input capture interrupt
  TIMSK1 |= bit(ICIE1);

  // Set the ADMUX which will charge the input cap
  ADMUX = (DEFAULT << 6) | APIN_BATTERY;
}

void loop()
{
  static uint8_t loopcntSensor;
  static uint8_t loopcntLcd;
  static uint32_t lastLoopMillis;
  static float hzLast = 0.0f;
  static float hzFastAvg = NAN;

  /* Read from the sensor */
  if (++loopcntSensor >= LOOPCNT_SENSOR)
  {
    loopcntSensor = 0;
    struct timerInfo localTimerInfo;

    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
      localTimerInfo = g_TimerInfo;
      g_TimerInfo.totalTime = 0;
      g_TimerInfo.cnt = 0;
    }

#if SERIAL_INTERFACE
    Serial.print('C'); Serial.print(localTimerInfo.cnt); Serial.print(' ');
#endif
    if (localTimerInfo.cnt != 0)
    {
      uint16_t localT1 = localTimerInfo.totalTime / localTimerInfo.cnt;
      float scale = getHzScale(localT1);
      hzLast = T1FREQ * scale / localT1;
      g_Liters += localTimerInfo.cnt * scale / (SENSOR_HZ_PER_LPM * 60.0f);
      g_RunningTime += millis() - lastLoopMillis;

#if SERIAL_INTERFACE
      Serial.print('S'); Serial.print(scale, 3); Serial.print(' ');
      Serial.print('T'); Serial.print(localT1, DEC); Serial_nl();
#endif
    }
    else
      hzLast = 0.0f;

    lastLoopMillis = millis();
  }

  /* Update the LCD */
  if (++loopcntLcd >= LOOPCNT_LCD)
  {
    loopcntLcd = 0;
    calcExpMovingAverage(2.0f/(1 + LOOP_SMOOTH), &hzFastAvg, hzLast);

    float lpm = hzFastAvg / SENSOR_HZ_PER_LPM;
    lcd.setCursor(0, 1);
    fp.print(lcd, lpm, 5, 2);
    lcd.print(" L/m ");
    fp.print(lcd, g_Liters, 6, 2);
    lcd.print(" L  ");

    float gallons = g_Liters * LITERS_PER_GALLON;
    float gpm = lpm * LITERS_PER_GALLON;
    lcd.setCursor(0, 2);
    fp.print(lcd, gpm, 5, 3);
    lcd.print(" GPM ");
    fp.print(lcd, gallons, 6, 2);
    lcd.print(" gal");
   
    lcd.setCursor(11, 3);
    lcd_printTime(g_RunningTime / 1000UL);

#if SERIAL_INTERFACE
    Serial.print("{hz,T,"); Serial.print(hzFastAvg, 2); Serial.print("}"); Serial_nl();
    Serial.print("{lpm,T,"); Serial.print(lpm, 2); Serial.print("}"); Serial_nl();
#endif
  }

  sleep(LOOP_PERIOD);
  lcd_updateAnim();

#if SERIAL_INTERFACE
  serial_update();
#endif
}
