
#include <AccelStepper.h> //accelstepper library
AccelStepper stepper(1, 11, 12); // DIR:12, STEP: 11

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

const byte Analog_X_pin = A0;
int Analog_X = 0;
int Analog_X_AVG = 0;
int AVG_Diff = 0;// different between the initialized and current value
//---------------------------------------------------------------------
const byte RotaryCLK = 2;
const byte RotaryDT  = 4;
const byte RotarySW  = 5;

const int LimitSwitch_1 = 10; //Input for the limit switch
const int LimitSwitch_2 = 11; //Input for the limit switch
// statuses of CLK and DT pin on the encoder
int CLKNow;
int CLKPrevious;

int RotaryButtonValue;
volatile int menuCounter = 0;// menu item number
float RotaryButtonTime = 0;

bool stepperHoming_Selected = false;
int homingPosition = 1;
bool joystickMovement_Selected = false;
bool stepperSpeed_Selected = false;
bool stepperDistance_Selected = false;
bool stepperTime_Selected = false;
bool startSlider_Selected = false;

bool valueChanged;
bool menuChanged;
bool updateValueSelection;


volatile float travelDistance = 10;
float travelSteps;
volatile float travelSpeed = 10;
float travelVelocity;
volatile float travelTime = 0.17;
float microStepping = 3200;



void setup() {
  // put your setup code here, to run once:
  lcd.init();
  lcd.backlight();
  //PINS
  pinMode(Analog_X_pin, INPUT); //A0
  pinMode(RotaryCLK, INPUT); //CLK
  pinMode(RotaryDT, INPUT); //DT
  pinMode(RotarySW, INPUT_PULLUP); //SW
  //--------------------------------------------------------------------

  InitialValues(); //averaging the values of the analog pin (value from potmeter)

  CLKPrevious = digitalRead(RotaryCLK);

  //Rotary encoder interrupts
  attachInterrupt(digitalPinToInterrupt(RotaryCLK), RotaryEncoder, CHANGE);
  //Stepper parameters
  stepper.setMaxSpeed(5000);
  stepper.setAcceleration(1000);
  delay(200);

  //Load the menu
  updateMenuPosition();
}

void ReadAnalog()
{
  if (joystickMovement_Selected == true)
  {
    //Reading the X potentiometer in the joystick
    Analog_X = analogRead(Analog_X_pin);
    AVG_Diff = Analog_X - Analog_X_AVG;


    if (abs(AVG_Diff) > 25)
    {
      stepper.setSpeed(3 * (AVG_Diff)); //the x3 multiplier is an arbitrary/empirical value.
      stepper.runSpeed();
    }
    else
    {
      stepper.setSpeed(0);
    }
    //valueChanged = true; //This would show the value of the joystick, but the Nano is too slow for this rapid display update
  }
  else
  {
    //Do nothing
  }
}
void InitialValues()
{
  //Set the values to zero before averaging
  float tempX = 0;
  //----------------------------------------------------------------------------
  //read the analog 50x, then calculate an average.
  for (int i = 0; i < 50; i++)
  {
    tempX += analogRead(Analog_X_pin);
    delay(10); //allowing a little time between two readings
  }
  Analog_X_AVG = tempX / 50;
}
void recalculateSpeed()
{
  // v = s/t
  travelSpeed = (10 * travelDistance) / (60.0 * travelTime); //60x because the time is expressed in minutes
  //(10*[cm]) /(60*[min]) ---> (mm)/[s]
}
void recalculateTime()
{
  // t = s/v
  travelTime = (10 * travelDistance / travelSpeed) / 60.0; //x10 because cm to mm, division by 60 is because time is in minutes
  //(10*[cm])/[mm/s]/60 ---> [mm]/[mm/s]/60 ---> [s]/60 ---> [min]
}
void convertValues()
{
  //distance in cm to steps
  //Pulley's pitch diameter = 12.732 mm, 20 teeth, GT2 pulley.
  travelSteps = (10 * travelDistance) * (microStepping / (12.732 * 3.1415)); //microstepping/(diameter*pi)
  travelVelocity = travelSpeed * (microStepping / (12.732 * 3.1415)); //microstepping/(diameter*pi), because mm/s to steps/s
}
void stepperHoming()
  {
    if (homingPosition == 1)
    {
      //homing part - negative direction
      while (digitalRead(LimitSwitch_1) == 0)
      {
        stepper.setSpeed(-600); //going towards the motor
        stepper.runSpeed();
      }
      //parking part - positive direction
      while (digitalRead(LimitSwitch_1) == 1)
      {
        stepper.setSpeed(200);
        stepper.runSpeed();
      }
      stepper.setCurrentPosition(0);
    }
    else //homingPosition == 2;
    {
      while (digitalRead(LimitSwitch_2) == 0)
      {
        stepper.setSpeed(600);
        stepper.runSpeed();
      }

      //parking part - positive direction
      while (digitalRead(LimitSwitch_2) == 1)
      {
        stepper.setSpeed(-200);
        stepper.runSpeed();
      }
    }
    lcd.setCursor(10, 0);
    lcd.print("1-Homing");
    lcd.setCursor(0, 20);
    lcd.print("Parked!");
  }
void RotaryEncoder()
{
  if (stepperHoming_Selected == true) //homing to which side
  {
    CLKNow = digitalRead(RotaryCLK); //Read the state of the CLK pin
    // If last and current state of CLK are different, then a pulse occurred
    if (CLKNow != CLKPrevious  && CLKNow == 1)
    {
      // If the DT state is different than the CLK state then
      // the encoder is rotating in A direction, so we increase
      if (digitalRead(RotaryDT) != CLKNow)
      {
        if (homingPosition < 2)
        {
          homingPosition++;
        }
        else
        {
          //Do nothing
        }
      }
      else
      {
        if (homingPosition == 1)
        {
          //Do nothing
        }
        else
        {
          homingPosition--;
        }
      }
      valueChanged = true;
    }
    CLKPrevious = CLKNow;  // Store last CLK state
  }
  else if (joystickMovement_Selected == true)
  {
    //do nothing
  }
  else if (stepperDistance_Selected == true) //set distance in millimeters
  {
    CLKNow = digitalRead(RotaryCLK); //Read the state of the CLK pin
    // If last and current state of CLK are different, then a pulse occurred
    if (CLKNow != CLKPrevious  && CLKNow == 1)
    {
      // If the DT state is different than the CLK state then
      // the encoder is rotating in A direction, so we increase
      if (digitalRead(RotaryDT) != CLKNow)
      {
        travelDistance++;
      }
      else
      {
        travelDistance--;
      }
      recalculateTime();
      //when the distance is changed, the speed is kept constant and the time is recalculated
      valueChanged = true;
    }
    CLKPrevious = CLKNow;  // Store last CLK state
  }
  //Set distance
  else if (stepperSpeed_Selected == true)
  {
    CLKNow = digitalRead(RotaryCLK); //Read the state of the CLK pin
    // If last and current state of CLK are different, then a pulse occurred
    if (CLKNow != CLKPrevious  && CLKNow == 1)
    {
      // If the DT state is different than the CLK state then
      // the encoder is rotating in A direction, so we increase
      if (digitalRead(RotaryDT) != CLKNow)
      {
        travelSpeed++;
      }
      else
      {
        travelSpeed--;
      }
      recalculateTime();
      valueChanged = true;
    }
    CLKPrevious = CLKNow;  // Store last CLK state
  }
  //Set time
  else if (stepperTime_Selected == true)
  {
    CLKNow = digitalRead(RotaryCLK); //Read the state of the CLK pin
    // If last and current state of CLK are different, then a pulse occurred
    if (CLKNow != CLKPrevious  && CLKNow == 1)
    {
      // If the DT state is different than the CLK state then
      // the encoder is rotating in A direction, so we increase
      if (digitalRead(RotaryDT) != CLKNow)
      {
        travelTime = travelTime + 0.1;
      }
      else
      {
        travelTime = travelTime - 0.1;
      }
      recalculateSpeed();
      valueChanged = true;
    }
    CLKPrevious = CLKNow;  // Store last CLK state
  }
  else if (startSlider_Selected == true)
  {
    //do nothing
  }
  else //MENU COUNTER----------------------------------------------------------------------------
  {
    CLKNow = digitalRead(RotaryCLK); //Read the state of the CLK pin
    // If last and current state of CLK are different, then a pulse occurred
    if (CLKNow != CLKPrevious  && CLKNow == 1)
    {
      // If the DT state is different than the CLK state then
      // the encoder is rotating CCW so increase
      if (digitalRead(RotaryDT) != CLKNow)
      {
        if (menuCounter < 5) //5 menu items 0-5
        {
          menuCounter++;
        }
        else
        {
          menuCounter = 0; //0 comes after 5, so we move in a "ring"
        }
        menuChanged = true;

      }
      else
      {
        // Encoder is rotating CW so decrease
        if (menuCounter > 0)
        {
          menuCounter--;
        }
        else
        {
          menuCounter = 5; //5 comes after 0 when we decrease the numbers
        }
        menuChanged = true;
      }
    }
    CLKPrevious = CLKNow;  // Store last CLK state
  }
}
void CheckRotaryButton()
{
  RotaryButtonValue = digitalRead(RotarySW);
  if (RotaryButtonValue == 0)
  {
    if (millis() - RotaryButtonTime > 1000)
    {
      switch (menuCounter)
      {
        case 0:
          stepperHoming_Selected = !stepperHoming_Selected;

          if (stepperHoming_Selected == false)
          {
            stepperHoming();
          }
          break;

        case 1:
          joystickMovement_Selected = !joystickMovement_Selected;
          break;
        case 2:
          stepperDistance_Selected = !stepperDistance_Selected;
          break;
        //
        case 3:
          stepperSpeed_Selected = !stepperSpeed_Selected;
          break;
        //
        case 4:
          stepperTime_Selected = !stepperTime_Selected;
          break;

        case 5:
          startSlider_Selected = !startSlider_Selected;
          if (startSlider_Selected == false) //If we exit this part, the motor stops
          {
            if (stepper.distanceToGo() != 0) //If the stepper is currently moving
            {
              stepper.stop(); //"soft" stop - decelerates to 0.
            }
            lcd.setCursor(1, 0);
            lcd.print("stopped!");
          }
          break;
      }
      RotaryButtonTime = millis();
      updateValueSelection = true;
    }
  }
}
void updateMenuPosition()
{
  switch (menuCounter)
  {
    //homing
    case 0:
      lcd.setCursor(1, 0);
      lcd.print("1_Homing");

      //lcd.setCursor(1, 1);
      //lcd.print("1_Homing");
      break;

    case 1:
      lcd.setCursor(1, 0);
      lcd.print("2-Joystick");
      break;
    case 2:
      lcd.setCursor(1, 0);
      lcd.print("3-Distance(cm)");
      break;
    case 3:
      lcd.setCursor(1, 0);
      lcd.print("4-Speed(mm/s)");
      break;
    case 4:
      lcd.setCursor(1, 0);
      lcd.print("5-Time(min)");
      break;
    case 5:
      lcd.setCursor(1, 0);
      lcd.print("6-Start slider");
      break;
  }
  menuChanged = false;
}
void updateSelection()
{
  if (stepperHoming_Selected == true)
  {
    lcd.setCursor(0, 0);
    lcd.print(">");
  }
  else if (joystickMovement_Selected == true)
  {
    lcd.setCursor(0, 0);
    lcd.print(">");
  }
  else if (stepperDistance_Selected == true)
  {
    lcd.setCursor(0, 0);
    lcd.print(">");
  }
  else if (stepperSpeed_Selected == true)
  {
    lcd.setCursor(0, 0);
    lcd.print(">");
  }
  else if (stepperTime_Selected == true)
  {
    lcd.setCursor(0, 0);
    lcd.print(">");
  }
  else if (startSlider_Selected == true)
  {
    lcd.setCursor(0, 0);
    lcd.print(">");
  }
    else
    {
      lcd.setCursor(0, 0);
      lcd.print(" ");
    }
    updateValueSelection = false;
  }
  void updateValue()
  {
    switch (menuCounter)
    {
      case 0:
        lcd.setCursor(3, 1);
        lcd.print(homingPosition);
        break;
      case 1:
        lcd.setCursor(3, 1);
        lcd.print("  ");
        break;
      case 2:
        lcd.setCursor(3, 1);
        lcd.print(travelDistance, 0);
        break;
      case 3:
        lcd.setCursor(3, 1);
        lcd.print(travelSpeed, 1);
        break;
      case 4:
        lcd.setCursor(3, 1);
        lcd.print(abs(travelTime), 2);
        break;
    }
    valueChanged = false;
  }

  
  void loop() {
    // put your main code here, to run repeatedly:
    ReadAnalog();
    CheckRotaryButton();

    if (menuChanged == true)
    {
      updateMenuPosition();
    }

    if (updateValueSelection == true)
    {
      updateSelection();
    }

    if (valueChanged == true )
    {
      updateValue();
    }
  }
