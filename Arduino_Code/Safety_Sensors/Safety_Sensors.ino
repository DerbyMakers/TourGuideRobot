int ledPin = 13; // LED connected to digital pin 13
int safePin = 12; // LED connected to digital pin 13
int inPin_IR1 = 2;   // IR sensor connected to digital pin 2
int inPin_IR2 = 3;   // IR sensor connected to digital pin 3
int val_IR1 = 0;     // variable to store the read value
int val_IR2 = 0;     // variable to store the read value

void setup()
{
  pinMode(ledPin, OUTPUT);      // sets the digital pin 13 as output
  pinMode(inPin_IR1, INPUT);      // sets the digital pin 2 as input
  pinMode(inPin_IR2, INPUT);      // sets the digital pin 3 as input
}

void loop()
{
    val_IR1 = !digitalRead(inPin_IR1);   // read the input pin
    val_IR2 = !digitalRead(inPin_IR2);   // read the input pin

  
  if (
    val_IR1 == 1
    ||
    val_IR2 == 1
    )
    {
    digitalWrite(ledPin, 1);    // sets the LED to the button's value
    digitalWrite(safePin, 1);    // sets the LED to the button's value
    }
    else{
        digitalWrite(ledPin, 0);    // sets the LED to the button's value
        }
}
