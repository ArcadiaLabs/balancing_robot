char BluetoothData;   // the Bluetooth data received
float pad_x,pad_y;      //control pad values sent from Android device
//boolean acc_on=false; //Flag to inidicate if to use accelerometer values
//int roll,pitch;       //roll and pitch sent from Android device

//Serial1.print("*LR0G255B0*");   // Virtual LED example

void BT_MsgSend()
{
  Serial1.print("*P"+String(angle_adjusted+90)+"*");
  Serial1.print("*S"+String(estimated_speed_filtered+500)+"*");
  if (acro_on == true)
    Serial1.print("*WR0G255B0*");
  else
    Serial1.print("*WR0G0B0*");
  if (auto_on == true){
    Serial1.print("*AR0G255B0*");                
  }
  else{
    Serial1.print("*AR0G0B0*");   
  }
  if (ir_on == true){
    Serial1.print("*E"+String(ir_distance)+"*"); // Sonar distance (cm)
    if (ir_distance <= OBSTACLE_DISTANCE_MIN){
      Serial1.print("*eR255G0B0*");
    }
    else if (ir_distance <= WALK_DISTANCE_MIN){
      Serial1.print("*eR255G255B0*");
    }
    else{
      Serial1.print("*eR0G255B0*");
    }    
  }
  else{
    Serial1.print("*E0*");            // Sonar send zero
    Serial1.print("*eR0G0B0*");
  }
  if (motors_on == true){
    Serial1.print("*MR0G255B0*");
  }
  else{
    Serial1.print("*MR0G0B0*");
  }
}

void BT_MsgRead()
{
  
  //Check Bluetooth for new Instructions
  if (Serial1.available())
  {
    BluetoothData=Serial1.read(); //Get next character from bluetooth
     
//    //**** Accelerometer  -  sends 'Aroll,pitch*' every 150 ms
//    if(BluetoothData=='A')
//    {
//      roll=Serial1.parseInt(); 
//      while (BluetoothData!='*')
//      {
//        if (Serial1.available())
//        {
//          BluetoothData=Serial1.read(); //Get next character from bluetooth
//          if(BluetoothData==',')
//          {
//            pitch=Serial1.parseInt();
//          }
//        }
//      }
//    }

    if(BluetoothData=='W')
    {
      if (acro_on == false){
        acro_on = true;
        Serial.println("Pro mode ON");
        // Change to PRO settings
        max_throttle = MAX_THROTTLE_PRO;
        max_steering = MAX_STEERING_PRO;
        max_target_angle = MAX_TARGET_ANGLE_PRO;
      }
      else{
        acro_on = false;
        Serial.println("Pro mode  OFF");
        // Change to NORMAL settings
        max_throttle = MAX_THROTTLE;
        max_steering = MAX_STEERING;
        max_target_angle = MAX_TARGET_ANGLE;
      }
      
    }

    if(BluetoothData=='A')
    {
      if (auto_on == false){
        auto_on = true;
        acro_on = false;
        ir_on = true;
        autonomous_mode_status = 0;
        autonomous_mode_counter = 0;
        // Change to NORMAL settings
        max_throttle = MAX_THROTTLE;
        max_steering = MAX_STEERING;
        max_target_angle = MAX_TARGET_ANGLE;
        Serial.println("Autonomous ON");
      }
      else{
        auto_on = false;
        acro_on = false;
        ir_on = false;
        // Change to NORMAL settings
        max_throttle = MAX_THROTTLE;
        max_steering = MAX_STEERING;
        max_target_angle = MAX_TARGET_ANGLE;
        throttle = 0;
        steering = 0;
        Serial.println("Autonomous OFF");
      }
    }

    if(BluetoothData=='E')
    {
      if (ir_on == false){
        ir_on = true;
        Serial.println("SONAR ON");
      }
      else{
        ir_on = false;
        Serial.println("SONAR OFF");
      }
    }

    if(BluetoothData=='S')
    {
      servo_on = true;
      Serial.println("SERVO ON");
    }
//    else 
//    {
//      servo_on = false;
//    }
    if(BluetoothData=='s')
    {
      servo_on = false;
      Serial.println("SERVO OFF");
    }
    
//    if(BluetoothData=='M')
//    {
//      if (motors_on == false){
//        Motors_Enable(true);
//        Serial.println("MOTORS ON");
//      }
//      else{
//        Motors_Enable(false);
//        Serial.println("MOTORS OFF");
//      }
//    }
      
//    //**** LEDs
//    if(BluetoothData=='P') Serial.println(BluetoothData);
//    if(BluetoothData=='p') Serial.println(BluetoothData);
//    if(BluetoothData=='Q') Serial.println(BluetoothData);
//    if(BluetoothData=='q') Serial.println(BluetoothData);
//    if(BluetoothData=='R') Serial.println(BluetoothData);
//    if(BluetoothData=='r') Serial.println(BluetoothData);
//    if(BluetoothData=='S') Serial.println(BluetoothData);
//    if(BluetoothData=='s') Serial.println(BluetoothData);

    //**** Control Pad -  Sends 'X__,Y___*' every 150ms
    if(BluetoothData=='X')
    {
      pad_x=Serial1.parseInt();
      pad_x=pad_x/10;
      steering = pad_x - 0.5;
      if (steering > 0)
      {
        steering = (steering * steering + 0.5 * steering) * max_steering;
      }
      else
      {
        steering = (-steering * steering + 0.5 * steering) * max_steering;
      }
//      Serial.print("Pad X : "); Serial.print(pad_x);
//      Serial.print("\tSteering : "); Serial.println(steering);
      while (BluetoothData!='*')
      {
        if (Serial1.available())
        {
          BluetoothData=Serial1.read(); //Get next character from bluetooth
          if(BluetoothData=='Y')
          {
            pad_y=Serial1.parseInt();
            pad_y=pad_y/10;
//            pad_y=-pad_y+1;
            throttle = (pad_y - 0.5) * max_throttle;
            throttle=-throttle;
//            Serial.print("Pad Y : "); Serial.print(pad_y);
//            Serial.print("\tThrottle : "); Serial.println(throttle);
          }
        }
      }
    }

//    //**** Sliders controls
//    if(BluetoothData=='X') 
//    {
//      pad_x=Serial1.parseInt();
//      pad_x=pad_x/100;
//      steering = pad_x - 0.5;
//      if (steering > 0)
//      {
//        steering = (steering * steering + 0.5 * steering) * max_steering;
//      }
//      else
//      {
//        steering = (-steering * steering + 0.5 * steering) * max_steering;
//      }
//      Serial.print("Pad X : "); Serial.print(pad_x);
//      Serial.print("\tSteering : "); Serial.println(steering);
//    }
//    if(BluetoothData=='Y')
//    {
//      pad_y=Serial1.parseInt();
//      pad_y=pad_y/100;
//      throttle = (pad_y - 0.5) * max_throttle;
//      Serial.print("Pad Y : "); Serial.print(pad_y);
//      Serial.print("\tThrottle : "); Serial.println(throttle);
//    }
    
//    //**** Control Pad on Left
//    if(BluetoothData=='0') Serial.println("Release"); //Release 
//    if(BluetoothData=='1') Serial.println("Up");      //Up
//    if(BluetoothData=='3') Serial.println("Down");    //Down
//    if(BluetoothData=='4') Serial.println("Left");    //Left
//    if(BluetoothData=='2') Serial.println("Right");   //Right   
  }  
}

