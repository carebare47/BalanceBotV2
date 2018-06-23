//void RobotController::regTask()
//{
//  imu.resetFifo();
//
//  pidIMU.reset();
//  pidOut_IMU = 0.0;
//
//  pidV.reset();
//  pidOut_V = 0.0;
//
//  wheelLeft.reset();
//  wheelRight.reset();
//
//  while (1) {
//    dt = sys.getRefTime() - lastTime;
//    lastTime = sys.getRefTime();
//    
//    //get new angle
//    angleNow = imu.getAngle();  //wait for interrupt and a new angle (every 20ms)
//
//    //compute mean speed
//    speedNow = (wheelLeft.getSpeed() + wheelRight.getSpeed()) / 2;
//
//    //PID to control robot speed
//    errSpeed = speedNow - targetSpeed;
//    pidOut_V = pidV.update(errSpeed, dt);
//
//    targetAngle = angle0 - pidOut_V;
//
//    //PID for controlling robot angle
//    errAngle = angleNow - targetAngle;
//    pidOut_IMU = pidIMU.update(errAngle, dt);
//
//    if ((angleNow - angle0) > 30.0 || (angleNow - angle0) < -30.0) {
//      pidIMU.reset();
//      pidOut_IMU = 0.0;
//
//      pidV.reset();
//      pidOut_V = 0.0;
//
//      wheelLeft.reset();
//      wheelRight.reset();
//    } 
//    
//    wheelLeft.setSpeed(pidOut_IMU + turnRight);
//    wheelRight.setSpeed(pidOut_IMU - turnRight);
//
//    wheelLeft.update(dt);
//    wheelRight.update(dt);
//  }
//} 
