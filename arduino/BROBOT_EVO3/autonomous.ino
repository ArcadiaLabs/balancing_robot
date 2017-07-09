void autonomousMode()
{
  if (autonomous_mode_status==0)   // Walking
  { 
    if (throttle < 75) //125
      throttle = throttle + 1;  //8
    if (throttle > 75) //125
      throttle = 75;   //125
    if (ir_distance < OBSTACLE_DISTANCE_MIN)  // obstacle near robot
      {
      autonomous_mode_status=1;
      autonomous_mode_counter=0;
      }
    // If something stop the robot, we start an obstacle avoiding
    if ((throttle == 75)&&(estimated_speed_filtered < 10))
      {
      autonomous_mode_counter++;
      if (autonomous_mode_counter>200)
        {
        autonomous_mode_status=1;
        autonomous_mode_counter=0;
        }
      }
    }
  else if (autonomous_mode_status==1)  // Obstacle: Waiting to stop
    {
    autonomous_mode_counter++;
    throttle = throttle - 2;
    if (throttle < 0)
      throttle = 0;
    if (autonomous_mode_counter > 250)  // Wait 1 second
      {
      autonomous_mode_status=2;
      autonomous_mode_counter=0;
      autonomous_mode_distance = WALK_DISTANCE_MIN;
      throttle = 0;
      if ((millis() % 2)==0)
        steering = 20;
      else
        steering = -20;
      }
    }
  else if (autonomous_mode_status==2) // Obstacle Steering
    {
    autonomous_mode_counter++;
    if (ir_distance > autonomous_mode_distance)
      {
      autonomous_mode_status = 3;  // We find a way...
      autonomous_mode_counter = 0;
      steering = -steering;     // rectification in steering
      }
    else if (autonomous_mode_counter > 600)
      {
      autonomous_mode_distance--;
      autonomous_mode_counter = 500;
      }
    else if (autonomous_mode_counter == 500)  // if we don´t find an exit in 2 seconds we reverse the steering command
      {
      steering = -steering;
      }
    }
  else if (autonomous_mode_status==3) // Small rectification in steering
    {
    autonomous_mode_counter++;
    if (autonomous_mode_counter > 20)  // 0.1 seconds rectification
      {
      autonomous_mode_status = 0;  // We find a way...
      autonomous_mode_counter=0;
      steering = 0;        
      }
    }  
  else
    {
    throttle = 0;
    steering = 0;
    }
}
