 /*
  * ----------------------------------------------------------------------------------------------------
  * Backyard Brains 2015
  * Muscle SpikerShield Arduino UNO Code for Control of Robotic Gripper
  *
  * Code monitors amplitude of EMG envelope, displays EMG strength on LED bar and controls 
  * robotic gripper by controlling servo motor.
  * 
  * V1.0
  * Written by Marcio Amorim
  * Updated by Stanislav Mircic
  * 
  * V3.0
  * Updated by Aaron Kyle, 3/9/22
  * This version is responsive to an FSR placed on an adaptive gripper arm. It is able to halt motion in 
  * response to an applied force and release when muscle activity ceases.
  *
  * Tested with Muscle SpikerShield V2.31
  * ----------------------------------------------------------------------------------------------------
  */
  
  #include <Servo.h>  
  #define GRIPPER_STATE_BUTTON_PIN 4          //pin for button that switches defult state of the gripper (opened/closed)
  #define SERVO_PIN 2                         //pin for servo motor
  #define SENSITIVITY_BUTTON_PIN 7            //pin for button that selects sesitivity
  #define NUM_LED 6                           //number of LEDs in LED bar
  #define GRIPPER_MINIMUM_STEP 5                 //5 degree dead zone (used to avoid aiming oscilation)
  #define OPEN_MODE 1                         //default gripper state is opened, muscle activation closes
  #define CLOSED_MODE 2                       //default gripper state is closed
  #define MINIMUM_SERVO_UPDATE_TIME 25       //update servo position every 125ms
  
  Servo Gripper;                                 //servo for gripper
  byte ledPins[] = {8, 9, 10, 11, 12, 13};    //pins for LEDs in LED bar
  
  //EMG saturation values (when EMG reaches this value the gripper will be fully opened/closed)
  int sensitivities[] = {200, 350, 520, 680, 840, 1000};
  int lastSensitivitiesIndex = 2;             //set initial sensitivity index
  
  int emgSaturationValue = 0;                 //selected sensitivity/EMG saturation value
  int analogReadings;                         //measured value for EMG
  byte ledbarHeight = 0;                      //temporary variable for led bar height
  
  unsigned long oldTime = 0;                  //timestamp of last servo angle update (ms)
  int oldDegrees = 0;                         //old value of angle for servo
  int newDegree;                              //new value of angle for servo
  
  unsigned long debouncerTimer = 0;           //timer for button debouncer         
  int gripperStateButtonValue = 0;            //temporary variable that stores state of button 
  int userReleasedButton = 1;                 //flag that is used to avoid multiple button events when user holds button
  
  int currentFunctionality = OPEN_MODE;       //current default position of claw

  int force_sensor;                           //measured value for force sensor
  int threshold = 100;                        //threshold voltage for touch detected (modify based on sensor placement)
    


  //-----------------------------------------------------------------------------------
  //   Setup servo, inputs and outputs
  // ----------------------------------------------------------------------------------
  void setup(){
    //init servo
    Gripper.attach(SERVO_PIN); 
    Serial.begin(9600);
    
    //init button pins to input   
    pinMode(GRIPPER_STATE_BUTTON_PIN, INPUT);                             
    pinMode(SENSITIVITY_BUTTON_PIN, INPUT);                            
    
    //initialize all LED pins to output
    for(int i = 0; i < NUM_LED; i++){ 
      pinMode(ledPins[i], OUTPUT);
    }
    
    //get current sensitivity
    emgSaturationValue = sensitivities[lastSensitivitiesIndex];
  }



  //-----------------------------------------------------------------------------------
  //   Main loop
  //
  //   - Checks state of sesitivity button
  //   - Checks state of default-gripper-state button
  //   - Measure EMG
  //   - Shows EMG strength on LED bar
  //   - Sets angle of servo based on EMG strength and current mode (open/closed)
  // ----------------------------------------------------------------------------------
  void loop()
  {
   
        //-----------------------  Switch sensitivity ------------------------------------
    
        //check if button is pressed (HIGH)
        if (digitalRead(SENSITIVITY_BUTTON_PIN))
        { 
            //turn off all the LEDs in LED bar
            for(int j = 0; j < NUM_LED; j++)
            {  
              digitalWrite(ledPins[j], LOW);
            }
          
            //increment sensitivity index
            lastSensitivitiesIndex++;
            if(lastSensitivitiesIndex==NUM_LED)
            {
              lastSensitivitiesIndex = 0;
            }
          
            //get current sensitivity value
            emgSaturationValue = sensitivities[lastSensitivitiesIndex]; 
            
            //light up LED at lastSensitivitiesIndex position for visual feedback
            digitalWrite(ledPins[lastSensitivitiesIndex], HIGH);
           
            //wait user to release button
            while (digitalRead(SENSITIVITY_BUTTON_PIN)) 
            {  
              delay(10);
            }       
            //whait a bit more so that LED light feedback is always visible
            delay(100);        
        }
 
    
        //----------------------------  Switch gripper default position open/close --------------------
     
        //check if enough time has passed for button contact to settle down
        if((millis() - debouncerTimer) > 50)
        {
            gripperStateButtonValue = digitalRead(GRIPPER_STATE_BUTTON_PIN);
            //if button is pressed
            if(gripperStateButtonValue == HIGH)
            {
                //if last time we checked button was not pressed
                if(userReleasedButton)
                {
                    debouncerTimer = millis();
                    //block button events untill user releases it
                    userReleasedButton = 0;
                    
                    //toggle operation mode
                    if(currentFunctionality == OPEN_MODE)
                    {
                      currentFunctionality = CLOSED_MODE;
                    }
                    else
                    {
                      currentFunctionality = OPEN_MODE;
                    }
                }
             }
             else
             {
                userReleasedButton = 1;
             }
        }


        //-----------------------------  Measure EMG -----------------------------------------------
    
        analogReadings = analogRead(A0);    //read EMG value from analog input A0
        force_sensor = analogRead(A1);      //read the force sensor from  

        
        //---------------------- Show EMG strength on LED ------------------------------------------
        
        //turn OFF all LEDs on LED bar
        for(int j = 0; j < NUM_LED; j++)
        {  
          digitalWrite(ledPins[j], LOW);
        }
         
        //calculate what LEDs should be turned ON on the LED bar
        analogReadings= constrain(analogReadings, 30, emgSaturationValue);
        ledbarHeight = map(analogReadings, 30, emgSaturationValue, 0, NUM_LED);
        
        //turn ON LEDs on the LED bar
        for(int k = 0; k < ledbarHeight; k++)
        {
          digitalWrite(ledPins[k], HIGH);
        }
   

        //-------------------- Drive Claw according to EMG strength -----------------------------
        
        //set new angle if enough time passed
        if (millis() - oldTime > MINIMUM_SERVO_UPDATE_TIME)
        {
              //calculate new angle for servo
              if(currentFunctionality == OPEN_MODE)
              {  

                //Gripper is opening
                analogReadings = constrain(analogReadings, 40, emgSaturationValue);     //Gripper is opening when EMG trends towards low value (40)
                newDegree = map(analogReadings, 40 ,emgSaturationValue, 190, 105);      //Servo motor angle decreases, opening the gripper
              }
              else
              {     

                 //Gripper is closing                  
               analogReadings = constrain(analogReadings, 120, emgSaturationValue);   //Gripper closing when EMG exceeds 120
                newDegree = map(analogReadings, 120 ,emgSaturationValue, 105, 190);   //Servo motor increases, closing the gripper  
                
              }
              
          
              //check if we are in servo dead zone
              if(abs(newDegree-oldDegrees) > GRIPPER_MINIMUM_STEP) {
                if (force_sensor < threshold) {                
                  //Angle updates if the pressure sensor is not engaged
                  //set new servo angle
                  Gripper.write(newDegree);
                }

                //If the segment below is not included, then the gripper will latch based on the FSR response
                if (force_sensor > threshold && analogReadings < 120) {
                  Gripper.write(190);           //Open the gripper when muscle contraction ceases, even if the pressure sensor is engaged
                  force_sensor = 0;             //Temporarily Forces the sensor to a sub-threshold level
                }
              
             
              oldTime = millis();
              oldDegrees = newDegree;
        }

//Stream the data from the EMG and force sensor acquisition, added by AMK, 2/8/22

//        Serial.print(oldTime/1000);           //Time stamp of each acquisition (ms)
//        Serial.print(" ");
        Serial.println(force_sensor);  //Voltage from Force sensor
//        Serial.print(" ");
//        Serial.println(analogReadings);    //Voltage from EMG
}
  }
