#include "WPILib.h"
class RobotDemo : public SimpleRobot
{
 RobotDemo(void)
 {
 // put initialization code here
 }
 void Autonomous(void)
 {
 // put autonomous code here
 }
 void OperatorControl(void)
 {
 // put operator control code here
 }
};
START_ROBOT_CLASS(RobotDemo); 

//RobotDrive drive(1, 2);
//Joystick leftStick(1);
//Joystick rightStick(2);

//RobotDemo(void)
//{
// GetWatchdog().SetEnabled(false);
//} 

//void Autonomous(void)
//{
// for (int i = 0; i < 4; i++)
// {
// drivetrain.Drive(0.5, 0.0); // drive 50% of full forward with 0% turn
// Wait(2.0); // wait 2 seconds
// drivetrain.Drive(0.0, 0.75); // drive 0% forward and 75% turn
// }
// Drivetrain.Drive(0.0, 0.0); // drive 0% forward, 0% turn (stop)
//} 

//void OperatorControl(void)
//{
// while (1) // loop forever
// {
// drivetrain.TankDrive(leftStick, rightStick);// drive with the joysticks
// Wait(0.005);
// }
//} 

//Joystick stick1(1); // this is an instance of a Joystick object stick1
//stick1.GetX(); // instances dereferenced using the dot (.) operator
//bot->ArcadeDrive(stick1); // and can be passed to methods as a reference
//Joystick * stick2; // a pointer to an uncreated Joystick object
//stick2 = new Joystick(1); // creates the instance of the Joystick object
//stick2->GetX(); // pointers are dereferenced with the arrow (->)
//bot->ArcadeDrive(stick2);

//Jaguar(UINT32 channel); // channel with default slot (4)
//Jaguar(UINT32 slot, UINT32 channel); // channel and slot
//Gyro(UINT32 slot, UINT32 channel); // channel with explicit slot
//Gyro(UINT32 channel); // channel with default slot (1) 

//class SimpleRobot: public RobotBase
//{
//public:
// SimpleRobot(void);
// virtual void Autonomous();
// virtual void OperatorControl();
// virtual void RobotMain();
// virtual void StartCompetition();
//private:
// bool m_robotMainOverridden;
//}; 

//void SimpleRobot::StartCompetition(void)
//{
//while (IsDisabled()) Wait(0.01); // wait for match to start
//if (IsAutonomous()) // if starts in autonomous
// {
// Autonomous(); // run user-supplied Autonomous code
// }
//while (IsAutonomous()) Wait(0.01); // wait until end of autonomous period
//while (IsDisabled()) Wait(0.01); // make sure robot is enabled
// OperatorControl(); // start user-supplied OperatorControl
//} 

//void Autonomous(void)
//{
// GetWatchdog().SetEnabled(false); // disable the watchdog timer
// Drivetrain.Drive(0.75, 0.0); // drive straight at 75% power
// Wait(2.0); // wait for 2 seconds
// .
// .
// .
// GetWatchdog().SetEnabled(true); // reenable the watchdog timer
//} 

//class GyroSample : public SimpleRobot
//{
//RobotDrive myRobot; // robot drive system
//Gyro gyro;
// static const float Kp = 0.03;
//public:
// GyroSample(void):
// myRobot(1, 2), // initialize the sensors in initialization list
// gyro(1)
// {
// GetWatchdog().SetExpiration(0.1);
// }
// void Autonomous(void)
// {
// gyro.Reset();
// while (IsAutonomous())
// {
// GetWatchdog().Feed();
// float angle = gyro.GetAngle(); // get heading
// myRobot.Drive(-1.0, -angle * Kp); // turn to correct heading
// Wait(0.004);
// }
// myRobot.Drive(0.0, 0.0); // stop robot
// }
//}; 

// HiTechnicCompass compass(4);
// compVal = compass.GetAngle(); 

//Ultrasonic ultra(ULTRASONIC_PING, ULTRASONIC_ECHO); 

// Ultrasonic ultra(ULTRASONIC_PING, ULTRASONIC_ECHO);
// ultra.SetAutomaticMode(true);
// int range = ultra.GetRangeInches(); 

//Encoder encoder(1, 2, true); 

//void SetAverageBits(UINT32 bits);
// UINT32 GetAverageBits(void);
//void SetOversampleBits(UINT32 bits);
// UINT32 GetOversampleBits(void); 

//if (StartCameraTask() == -1) {
// dprintf( LOG_ERROR,"Failed to spawn camera task; Error code %s",
// GetErrorText(GetLastError()) );
//}

//int frameRate = 15; // valid values 0 - 30
//int compression = 0; // valid values 0 – 100
//ImageSize resolution = k160x120; // k160x120, k320x240, k640480
//ImageRotation rot = ROT_180; // ROT_0, ROT_90, ROT_180, ROT_270
//StartCameraTask(frameRate, compression, resolution, rot);

//double timestamp; // timestamp of image returned
//Image* cameraImage = frcCreateImage(IMAQ_IMAGE_HSL);
//if (!cameraImage) { printf(“error: %s”, GetErrorText(GetLastError()) };
//if ( !GetImage(cameraImage, &timestamp) ) {
// printf(“error: %s”, GetErrorText(GetLastError()) };

//Image* cameraImage = frcCreateImage(IMAQ_IMAGE_HSL);
//double timestamp; //timestamp of image returned
//double lastImageTimestamp; //timestamp of last image to ensure image is new
//int success = GetImageBlocking(cameraImage, &timestamp, lastImageTimestamp);

//int result = GetCameraMetric(CAM_NUM_IMAGE);

//StartCameraTask(); // Initialize the camera
//PCVideoServer pc; // The constructor starts the image server task
//pc.Stop(); // Stop image server task
//pc.Start(); // Restart task and serve images again 

//void SetBounds(INT32 max,
// INT32 deadbandMax,
// INT32 center,
// INT32 deadbandMin,
// INT32 min); 

//Servo servo(3); // create a servo on PWM port 3 on the first module
//float servoRange = servo.GetMaxAngle() - servo.GetMinAngle();
//for (float angle = servo.GetMinAngle(); // step through range of angles
// angle < servo.GetMaxAngle();
// angle += servoRange / 10.0)
// {
// servo.SetAngle(angle); // set servo to angle
// Wait(1.0); // wait 1 second
// } 

// #include “BaeUtilities.h”
// panInit(); // optional parameters can adjust pan speed
// bool targetFound = false;
//while(!targetFound) {
// panForTarget(servo, 0.0); // sinStart from -1 to +1
// code to identify target
// } 

//RobotDrive drive(1, 2); // left, right motors on ports 1,2 

//RobotDrive drive(1, 2, 3, 4); // four motor drive case 

//SetInvertedMotor(kFrontLeftMotor); 

// Compressor *c = new Compressor(4, 2);
// c->Start(); 

//Solenoid *s[8];
//for (int i = 0; i < 8; i++)
// s[i] = new Solenoid(i + 1); // allocate the Solenoid objects
//for (int i = 0; i < 8; i++)
// {
// s[i]->Set(true); // turn them all on
// }
//for (int i = 0; i < 8; i++)
// {
// s[i]->Set(false); // turn them each off in turn
// Wait(1.0);
// }
//for (int i = 0; i < 8; i++)
// {
// s[i]->Set(true); // turn them back on in turn
// Wait(1.0);
// delete s[i]; // delete the objects
// } 

//TrackingThreshold tdata = GetTrackingData(BLUE, FLUORESCENT);
//ParticleAnalysisReport par;
// if (FindColor(IMAQ_HSL, &tdata.hue, &tdata.saturation,
// &tdata.luminance, &par)
// {
// printf(“color found at x = %i, y = %i",
// par.center_mass_x_normalized, par.center_mass_y_normalized);
// printf(“color as percent of image: %d",
// par.particleToImagePercent);
// } 

//Range hue, sat, lum;
// hue.minValue = 140; // Hue
// hue.maxValue = 155;
// sat.minValue = 100; // Saturation
// sat.maxValue = 255;
// lum.minValue = 40; // Luminance
// lum.maxValue = 255;
// FindColor(IMAQ_HSL, &hue, &sat, &lum, &par);

//RobotDrive *myRobot
//Range greenHue, greenSat, greenLum; 

//if (StartCameraTask() == -1) {
// printf( "Failed to spawn camera task; Error code %s",
// GetErrorText(GetLastError()) );
//}
//myRobot = new RobotDrive(1, 2);
// values for tracking a target - may need tweaking in your environment
//greenHue.minValue = 65; greenHue.maxValue = 80;
//greenSat.minValue = 100; greenSat.maxValue = 255;
//greenLum.minValue = 100; greenLum.maxValue = 255; 

//while (IsAutonomous())
//{
//if ( FindColor(IMAQ_HSL, &greenHue, &greenSat, &greenLum, &par)
// && par.particleToImagePercent < MAX_PARTICLE_TO_IMAGE_PERCENT
// && par.particleToImagePercent > MIN_PARTICLE_TO_IMAGE_PERCENT )
// {
// myRobot->Drive(1.0, (float)par.center_mass_x_normalized);
// }
//else myRobot->Drive(0.0, 0.0);
// Wait(0.05);
//}
//myRobot->Drive(0.0, 0.0); 

//{
// Synchronized s(semaphore);
 // access shared code here
// if (condition) return;
 // more code here
//} 

//CRITICAL_REGION(semaphore)
//{
 // access shared code here
// if (condition) return;
 // more code here
//}
//END_REGION; 

//static int interruptCounter = 0;
// The interrupt handler that counts number of square wave cycles
//static void tiHandler(tNIRIO_u32 interruptAssertedMask, void *param)
//{
// interruptCounter++;
//}
//void InterruptTestHandler(void)
//{
 // create the two digital ports (Output and Input)
// DigitalOutput digOut(CROSS_CONNECT_A_PORT1);
// DigitalInput digIn(CROSS_CONNECT_A_PORT2);
 // create the counter that will also count square waves
// Counter counter(&digIn);
 // initialize the digital output to 0
// digOut.Set(0);
 // start the counter counting at 0
// counter.Reset();
// counter.Start();
 // register and enable the interrupt handler
// digIn.RequestInterrupts(tiHandler);
// digIn.EnableInterrupts();
 // count 5 times
// while (counter.Get() < 5)
// {
// Wait(1.0);
// digOut.Set(1);
// Wait(1.0);
// digOut.Set(0);
// }
 // verify correct operation
// if (interruptCounter == 5 && counter.Get() == 5)
// printf(“Test passed!\n”);
 // free resources
// digIn.DisableInterrupts();
// digIn.CancelInterrupts();
//}
//END_TEST(TestInterruptHandler) 
