

#include <Wire.h>

#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX

#define N_WIDTH 4 //4 or 8^2
#define TOF_SPEED_FPS 10

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM

#define INT_PIN 4 //Connect VL53L5CX INT pin to pin 4 on your microcontroller
volatile bool dataReady = false; //Goes true when interrupt fires

int imageResolution = 0; //Used to pretty print output
int imageWidth = 0; //Used to pretty print output


void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("SparkFun VL53L5CX Imager Example");

  Serial2.begin(9600);

  Wire.begin(); //This resets I2C bus to 100kHz
  Wire.setClock(1000000); //Sensor has max I2C freq of 1MHz

  Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
  if (myImager.begin() == false)
  {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1) ;
  }

  Serial.print(F("Current sharpener value: "));
  Serial.print(myImager.getSharpenerPercent());
  Serial.println(F("%"));

  myImager.setAddress('0x52');

  bool response = myImager.setSharpenerPercent(50);
  if (response == true)
  {
    Serial.print(F("Current sharpener value: "));
    Serial.print(myImager.getSharpenerPercent());
    Serial.println(F("%"));
  }
  else
  {
    Serial.println(F("Cannot set sharpener value."));
    Serial.println(F("System halted."));
    while (1)
    {
    }
  }

  SF_VL53L5CX_TARGET_ORDER order = myImager.getTargetOrder();
  switch (order)
  {
    case SF_VL53L5CX_TARGET_ORDER::STRONGEST:
      Serial.println(F("Current target order is strongest."));
      break;

    case SF_VL53L5CX_TARGET_ORDER::CLOSEST:
      Serial.println(F("Current target order is closest."));
      break;

    default:
      Serial.println(F("Cannot get target order."));
      break;
  }

  response = myImager.setTargetOrder(SF_VL53L5CX_TARGET_ORDER::CLOSEST);
  if (response == true)
  {
    order = myImager.getTargetOrder();
    switch (order)
    {
      case SF_VL53L5CX_TARGET_ORDER::STRONGEST:
        Serial.println(F("Target order set to strongest."));
        break;

      case SF_VL53L5CX_TARGET_ORDER::CLOSEST:
        Serial.println(F("Target order set to closest."));
        break;

      default:
        break;
    }
  }
  else
  {
    Serial.println(F("Cannot set target order. Freezing..."));
    while (1) ;
  }

  myImager.setResolution(N_WIDTH * N_WIDTH ); //Enable all 64 pads

  imageResolution = myImager.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution); //Calculate printing width

  //Using 4x4, min frequency is 1Hz and max is 60Hz
  //Using 8x8, min frequency is 1Hz and max is 15Hz
  response = myImager.setRangingFrequency(TOF_SPEED_FPS);
  if (response == true)
  {
    int frequency = myImager.getRangingFrequency();
    if (frequency > 0)
    {
      Serial.print("Ranging frequency set to ");
      Serial.print(frequency);
      Serial.println(" Hz.");
    }
    else
      Serial.println(F("Error recovering ranging frequency."));
  }
  else
  {
    Serial.println(F("Cannot set ranging frequency requested. Freezing..."));
    while (1) ;
  }


  attachInterrupt(digitalPinToInterrupt(INT_PIN), interruptRoutine, FALLING);
  Serial.println(F("Interrupt pin configured."));

  myImager.startRanging();
}


#define KNT_FRAMES 20 //20 frame / 10fps = 2 seconds
float sensorMesauements[N_WIDTH][N_WIDTH] = {{}};  //check for change
float noiseSubtraction[N_WIDTH][N_WIDTH][KNT_FRAMES] = {{{}}};  //check for change
int resultGrid[N_WIDTH][N_WIDTH] = {{}};  //check for change
int frame = 0;
int positionToSend=-99;
void loop()
{
  //Poll sensor for new data
  if (dataReady == true) {
    dataReady = false;

    int xScore[N_WIDTH] = {};
    int liveGrid[N_WIDTH][N_WIDTH] = {{}};  //check for change

    if (myImager.getRangingData(&measurementData)) { //Read distance data into array{

      //The ST library returns the data transposed from zone mapping shown in datasheet
      //Pretty-print data with increasing y, decreasing x to reflect reality
      int nMod = 0;
      for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth) {
        for (int x = imageWidth - 1 ; x >= 0 ; x--) {
          int measurement = measurementData.distance_mm[x + y];
          //add frames to buffer in XYOrder + frame
          // noiseSubtraction[nMod % imageWidth][int(floor(nMod / imageWidth))][frame] = measurement;
          sensorMesauements[nMod % imageWidth][int(floor(nMod / imageWidth))] = measurement; 

          nMod ++;
        }
      }
    
    //   for (int y = 0; y < N_WIDTH; y ++) {
    //     for (int x = 0; x < N_WIDTH; x ++) {
    //       for (int k = 0; k < KNT_FRAMES; k ++){
    //       if((k+1)< KNT_FRAMES){
    //         noiseSubtraction[x][y][k+1] = abs(noiseSubtraction[x][y][k+1] - noiseSubtraction[x][y][k]);
    //       }
    //       else{resultGrid[x][y] = sensorMesauements[x][y] + ((noiseSubtraction[x][y][k]));}
    //     }
    //   }
    // }

    frame ++;
    frame %= KNT_FRAMES;
    Serial.println("frame" + String(frame));


      int nLowest = 40000 * N_WIDTH;  //Max distance is 4meters*1000
      // int nLowest = 0;  //Max distance is 4meters*1000
      int nLow = -1;
      int absDifference = -1;
      int absDifferenceDifference = 1;

      //Print Frames
      for (int y = 0; y < N_WIDTH; y ++) {
        for (int x = 0; x < N_WIDTH; x ++) {
          // resultGrid[x][y] /= KNT_FRAMES;
          resultGrid[x][y] = sensorMesauements[x][y];
          Serial.print("\t");
          Serial.print(resultGrid[x][y]);
          xScore[x] += resultGrid[x][y];
        }
        Serial.println();
      }
      Serial.println();

      Serial.print("Score");
      for (int x = 0; x < N_WIDTH; x ++) {
        Serial.print("\t");
        Serial.print(String(xScore[x]));
        if (xScore[x] < nLowest ) {
          nLowest = xScore[x];
          nLow = x;
        }
      }

      
      Serial.println();
      Serial.print("Position= ");
      Serial.print(String(nLow));
      // Serial2.print(String(nLow) + "\r");
      Serial.println();
      Serial.print("LowVal= ");
      Serial.println(String(nLowest));
      Serial.print("AbsDifference= ");
      Serial.println(String(absDifference));
      Serial.println("--------------------------------------------------");
      positionToSend = nLow;

    }
  }
  delay(5); //Small delay between polling

/*testing using cat alone send an ACK everytime it reads, very frustrating.. To remove this use:
echo "1" > /dev/ttyLP0
stty -echo -opost -F /dev/ttyLP0 9200
cat /dev/ttyLP0
*/
  bool incoming = false;
  char incomingBuffer[10] = "";
  while(Serial2.available()>0){
    Serial2.read();
    incoming = true;
  }
  if(incoming){
    Serial2.print(String(positionToSend) + '\r');
    Serial2.flush();
    incoming = false;
  }
}

void interruptRoutine() {
  // Just set the flag that we have updated data and return from the ISR
  dataReady = true;
}
