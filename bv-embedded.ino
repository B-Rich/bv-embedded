/*
    UFPB - Centro de Informática

    BlueVending
*/

#include "Keypad/Keypad.h"
#include "MyLiquidCrystal/MyLiquidCrystal.h"
#include <SoftwareSerial.h>


// Shift register (LCD e keypad)
const byte srclkPin = 3, srdatPin = 2;


// Keypad setup
const byte rows = 4;
const byte cols = 3;
char keys[rows][cols] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
byte rowPins[rows] = {8, 7, 6, 5};
byte colPins[cols] = {0, 0, 0};       // usando shift register para colunas
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, rows, cols );


// LCD setup
MyLiquidCrystal lcd(0, 4, 0, 0, 0, 0);  // RS, EN (usando shift register para todos, exceto EN)


// Bluetooth setup
const byte btPowerPin = A2, btStatePin = A3, btTxPin = A0, btRxPin = A1;
char btPowerState = 0, btPairState = 0;
SoftwareSerial btSerial(btRxPin, btTxPin);


// RGB LED setup
const byte ledRedPin = 9, ledGreenPin = 10, ledBluePin = 11;
unsigned char red = 0, green = 0, blue = 0;


// Sensor de queda do produto
const byte sensorPin = 12;

void setup() {
    Serial.begin(9600);
    Serial.println("Hello");

    pinMode(srclkPin, OUTPUT);
    pinMode(srdatPin, OUTPUT);
    digitalWrite(srclkPin, LOW);

    keypad.setSerial(srdatPin, srclkPin);       // para usar shift register

    lcd.setSerial(srdatPin, srclkPin);          // para usar shift register
    lcd.begin(16, 2);
    lcd.print("Escolha produto");

    pinMode(btPowerPin, OUTPUT);
    pinMode(btStatePin, INPUT);

    btSerial.begin(9600);
    digitalWrite(btPowerPin, HIGH);
}

void loop() {
    char key = keypad.getKey();

    if(key != NO_KEY) {

        switch(key)
        {
            case '1': if(red <= 225) red += 25; break;
            case '4': if(red >= 25) red -= 25; break;

            case '2': if(green <= 225) green += 25; break;
            case '5': if(green >= 25) green -= 25; break;

            case '3': if(blue <= 225) blue += 25; break;
            case '6': if(blue >= 25) blue -= 25; break;

            case '#':
                if(btPowerState)
                {
                    btPowerState = 0;
                    Serial.println("Bt off");
                    digitalWrite(btPowerPin, HIGH);
                }
                else
                {
                    btPowerState = 1;
                    Serial.println("Bt on");
                    digitalWrite(btPowerPin, LOW);
                }

                delay(100);
                
                break;

            case '*':
                Serial.print("Bt ");
                
                if(btPowerState == 1) {
                    Serial.print("on, ");
                    if(btPairState == 1) Serial.print("paired");
                    else Serial.print("unpaired");
                }
                else Serial.print("off");

                Serial.print(" // RGB = (");
                Serial.print(red);
                Serial.print(", ");
                Serial.print(green);
                Serial.print(", ");
                Serial.print(blue);
                Serial.print(") // ");

                Serial.print("Sensor ");
                Serial.println(digitalRead(sensorPin));

                break;

            default: Serial.print(key);
        }

        analogWrite(ledRedPin, red);
        analogWrite(ledGreenPin, green);
        analogWrite(ledBluePin, blue);
    }

    while(btSerial.available()) {
        char c = btSerial.read();
        if(btPowerState == 1) Serial.write(c);
    }

    while(Serial.available())
    {
        char c = Serial.read();
        Serial.write(c);
        if(btPowerState == 1) btSerial.write(c);
    }

    if(btPowerState == 1)
    {
        if(digitalRead(btStatePin) == HIGH && btPairState == 0)
        {
            btPairState = 1;
            Serial.println("Bt paired");
        } else if(digitalRead(btStatePin) == LOW && btPairState == 1)
        {
            btPairState = 0;
            Serial.println("Bt disconnect");
        }
    }
    
    delay(5);
}
