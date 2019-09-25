#include <M5Stack.h>
#include "Free_Fonts.h"
#include <Wire.h>

#include <Wire.h>

//Click here to get the library: http://librarymanager/All#SparkFun_SCD30
#include "SparkFun_SCD30_Arduino_Library.h"

SCD30 airSensor;


#define TFT_GREY 0x7BEF



#define X_LOCAL 40
#define Y_LOCAL 30

#define X_OFFSET 160
#define Y_OFFSET 23


# define PPM_MIN 300
# define PPM_MAX 2000

#define DATA_LEN 320
int Air_val[DATA_LEN] = {0};
int16_t p_val[16] = {0};
int16_t dcount = 0;

// pointer into circular buffer
int dptr = 0;



//uint16_t CheckSum;
//uint16_t CheckCode;

// Print the header for a display screen
void header(const char *string, uint16_t color)
{
  M5.Lcd.fillScreen(color);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(TFT_MAGENTA, TFT_BLUE);
  M5.Lcd.fillRect(0, 0, 320, 30, TFT_BLUE);
  M5.Lcd.setTextDatum(TC_DATUM);
  M5.Lcd.drawString(string, 160, 3, 4);
}
void setup() {

  M5.begin();



  M5.Lcd.fillScreen(TFT_BLACK);
  header("CO2 ppm", TFT_BLACK);

  Wire.begin();

  Serial.begin(9600);
  Serial.println("SCD30 Example");

  airSensor.begin();

  delay(100);


  // synthetic data to test plot
  /*
    for (int i = 0; i < DATA_LEN; i++) {
    Air_val[i] = (int)(PPM_MAX - PPM_MIN) * (0.5 * sin(((i * 1.13) * 3.14) / 180) + 0.5) + PPM_MIN;
    }
  */
}


#define FRONT 2



void loop() {


  if (airSensor.dataAvailable())
  {
    Serial.print("co2(ppm):");

    Air_val[dcount] = airSensor.getCO2();
    Serial.print(Air_val[dcount]);
    dcount++;
    plot_array();


    Serial.print(" temp(C):");
    Serial.print(airSensor.getTemperature(), 1);

    Serial.print(" humidity(%):");
    Serial.print(airSensor.getHumidity(), 1);

    Serial.println();
  }
  //else
  //  Serial.println("No data");



  /*
    if (Serial2.available()) {
      Air_val[i] = Serial2.read();
      Serial.write( Air_val[i]);
      i++;
    } else {
      i = 0;
    }
  */


  if (dcount == DATA_LEN) {
    //LCD_Display_Val();
    //TempHumRead();
    dcount = 0;
  }
  delay(100);
}



void plot_array() {

  int buf[318];
  int x, x2;
  int y, y2;
  int r;

  // Clear the screen and draw the frame
  M5.Lcd.fillScreen(TFT_BLACK);

  /*
    M5.Lcd.fillRect(0, 0, 319, 14, TFT_RED);

    M5.Lcd.fillRect(0, 226, 319, 14, TFT_GREY);

    M5.Lcd.setTextColor(TFT_BLACK, TFT_RED);
    M5.Lcd.drawCentreString("* TFT_eSPI *", 160, 4, 1);
    M5.Lcd.setTextColor(TFT_YELLOW, TFT_GREY);
    M5.Lcd.drawCentreString("Adapted by Bodmer", 160, 228, 1);

  */

  M5.Lcd.drawRect(0, 14, 319, 211, TFT_BLUE);

  // Draw crosshairs
  M5.Lcd.drawLine(159, 15, 159, 224, TFT_BLUE);
  M5.Lcd.drawLine(1, 119, 318, 119, TFT_BLUE);
  for (int i = 9; i < 310; i += 10)
    M5.Lcd.drawLine(i, 117, i, 121, TFT_BLUE);
  for (int i = 19; i < 220; i += 10)
    M5.Lcd.drawLine(157, i, 161, i, TFT_BLUE);

  // Draw data,
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);

  M5.Lcd.drawString("CO2", 5, 15, 2);

  M5.Lcd.drawString("0 ppm", 300, 225, 2);
  M5.Lcd.drawString("2000", 300, 15, 2);


  int oldx = 0;
  int oldy =  240 - (int) 240.*(Air_val[0]) / (float)(PPM_MAX);

  for (int i = 1; i < 320; i++)
  {
    int y  = 240 - (int) 240.*(Air_val[i]) / (float)(PPM_MAX);
    if (y < 0) {
      y = 0;
    }

    if (i == dcount - 1) {
      M5.Lcd.fillCircle(i, y, 5, TFT_RED);
      oldx = i;
      oldy = y;
    }

    M5.Lcd.drawLine(i, y, oldx, oldy, (i >= dcount) ? (TFT_BLUE) : (TFT_CYAN));
    //M5.Lcd.fillCircle(oldx, oldy, 10, TFT_RED);

    oldx = i;
    oldy = y;
    //M5.Lcd.drawPixel(i, 119 + (sin(((i * 1.13) * 3.14) / 180) * 95), TFT_CYAN);
  }
  //M5.Lcd.fillCircle(oldx, oldy, 5, TFT_RED);
}
