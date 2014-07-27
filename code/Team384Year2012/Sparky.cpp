/*
 * $Id$
 */

#include "WPILib.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "NiVision.h"
#include "math.h"

static AxisCamera *camera;

// lights
static Relay *g_lights;
static DigitalInput *g_top;
static DigitalInput *g_middle;
static DigitalInput *g_shooter;

// encoder
static SEM_ID armSem;
static int encPos;
static bool armSet;
static double armSpeed;

// trigger release
static SEM_ID releaseSem;
static bool releaseSet;
static bool intakeOff;

// auto aim
static SEM_ID autoAimSem;
static bool g_autoAimSet;
static double g_targetDistance;
typedef enum {TARGET_LEFT, TARGET_RIGHT, TARGET_CENTER, TARGET_NONE} targetAlignment;
static targetAlignment g_targetAlign;
static RobotDrive *g_sparky;
 
/**
 * Sparky class.  Describes the 2012 FRC robot.
 */
class Sparky : public SimpleRobot
{
	RobotDrive sparky;
	Joystick stick1, stick2, stick3;
	Task targeting, blinkyLights, autoAim;
	DigitalInput top, middle, shooter, trigger, bridgeArmUp, bridgeArmDown;
	DriverStation *ds;
	DriverStationLCD *dsLCD;
	Jaguar arm;
	Victor floorPickup, shooterLoader, bridgeArm;
	Relay release, lights;
	Encoder tension;
	
	// constants
	static const double MOTOR_OFF = 0.0;
	static const double TENSION_BRAKE = -0.06;
	static const double ARM_SPEED_COARSE = 0.5;
	static const double ARM_SPEED_COARSE_LOAD = -0.5;
	static const double ARM_SPEED_COARSE_UNLOAD = 0.5;
	static const double ARM_SPEED_FINE_LOAD = -0.3;
	static const double ARM_SPEED_FINE_UNLOAD = 0.2;
	static const double ARM_SPEED_FULL_LOAD = -1.0;
	static const double ARM_SPEED_FULL_UNLOAD = 1.0;
	static const double ARM_ZERO_THRESH = 75;
	static const double INTAKE_LOAD = 1.0;
	static const double INTAKE_UNLOAD = -1.0;
	static const double INTAKE_OFF = 0.0;
	static const double BRIDGE_ARM_DOWN = 0.9;
	static const double BRIDGE_ARM_UP = -0.9;
	static const double BRIDGE_ARM_OFF = 0.0;
	static const double AUTO_AIM_SPEED = 0.2;

public:
	Sparky(void):
		sparky(3, 2),
		stick1(1),
		stick2(2),
		stick3(3),
		targeting("targeting", (FUNCPTR)Targeting, 102),
		blinkyLights("blinkyLights", (FUNCPTR)BlinkyLights, 103),
		autoAim("autoAim", (FUNCPTR)AutoAim),
		top(13),
		middle(14),
		shooter(12),
		trigger(11),
		bridgeArmUp(3),
		bridgeArmDown(4),
		ds(DriverStation::GetInstance()),
		dsLCD(DriverStationLCD::GetInstance()),
		arm(1),
		floorPickup(5),
		shooterLoader(4),
		bridgeArm(7),
		release(6),
		lights(4),
		tension(1,2)  // measures tension-revolutions 
	{
		printf("Sparky: start\n");
		encPos = 0;
		armSet = false;
		g_autoAimSet = false;
		g_targetDistance = 0;
		g_targetAlign = TARGET_NONE;
		g_sparky = &sparky;
		tension.Reset();
		tension.Start();
		sparky.SetExpiration(0.1);
		sparky.SetSafetyEnabled(false);
		sparky.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
		sparky.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		g_lights = &lights;
		g_top = &top;
		g_middle = &middle;
		g_shooter = &shooter;
		Wait(5);
		/*
		camera = &AxisCamera::GetInstance("10.3.84.11");
		camera->WriteResolution(AxisCameraParams::kResolution_640x480);
		*/
		camera = &AxisCamera::GetInstance("10.3.84.12");
		camera->WriteResolution(AxisCameraParams::kResolution_320x240);
		camera->WriteWhiteBalance(AxisCameraParams::kWhiteBalance_Hold);
		camera->WriteExposureControl(AxisCameraParams::kExposure_Hold);
		camera->WriteColorLevel(100);
	    camera->WriteCompression(30);
		camera->WriteBrightness(30);
		camera->WriteMaxFPS(10);
		Wait(5);
		printf("Sparky: done\n");
	}
	
	/**
	 * When disabled, suspend the targeting Task.
	 */
	void Disabled()
	{
		if(targeting.IsReady() && !targeting.IsSuspended())
			targeting.Suspend();
		
		if(blinkyLights.IsReady() && !blinkyLights.IsSuspended())
			blinkyLights.Suspend();
	}
	
	/**
	 * Overridden to avoid runtime message.
	 */
	void RobotInit()
	{	
	}
	
	/**
	 * Score two baskets from the key.
	 */
	void Autonomous(void)
	{
		printf("Autonomous: start\n");
		sparky.SetSafetyEnabled(false);
		/*
		if(targeting.IsSuspended())
			targeting.Resume();
		else
			targeting.Start();
	    */

		if(IsAutonomous() && IsEnabled())
		{
			if(ds->GetDigitalIn(1))
			{
				printf("Waiting 1...");
				Wait(3);
				printf("Waiting done!\n");
			}
			else if(ds->GetDigitalIn(2))
			{
				printf("Waiting 2...");
				Wait(5);
				printf("Waiting done!\n");
			}
			else if(ds->GetDigitalIn(3))
			{
				printf("Waiting 3...");
				Wait(7);
				printf("Waiting done!\n");
			}
			
			int p = 190;
			
			ArmToPosition(p);
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "encoder: %d", tension.Get());
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "s: %d, t: %d, m: %d", shooter.Get(), top.Get(), middle.Get());
			dsLCD->UpdateLCD();
			ReleaseNotifier(this);
			ArmToPositionNoEye(p);
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "encoder: %d", tension.Get());
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "s: %d, t: %d, m: %d", shooter.Get(), top.Get(), middle.Get());
			dsLCD->UpdateLCD();
			ReleaseNotifier(this);
			while(IsAutonomous() && IsEnabled())
			{
				Wait(0.05);
			}
		}
		//targeting.Suspend();
		printf("Autonomous: stop\n");
	}
	
	/**
	 * Tele-op period.
	 */
	void OperatorControl(void)
	{
		printf("OperatorControl: start\n");
		Notifier armToPositionNotifier(ArmToPositionNotifier, this);
		Notifier releaseNotifier(ReleaseNotifier, this);
		Timer armTimer;
		bool armUp = false;
		bool armDown = false;
		int lastPosition = 0;
		sparky.SetSafetyEnabled(false);
		armSet = false;
		releaseSet = false;
		intakeOff = false;
		
		if(targeting.IsSuspended())
			targeting.Resume();
		else
			targeting.Start();
		
		if(blinkyLights.IsSuspended())
			blinkyLights.Resume();
		else
			blinkyLights.Start();
		
		armTimer.Start();

		while (IsOperatorControl() && IsEnabled())
		{
			// drive
			if(!g_autoAimSet)
			{
				if(stick1.GetTrigger() && !stick2.GetTrigger())
				{
					sparky.ArcadeDrive(stick1);
				}
				else if(stick1.GetTrigger() && stick2.GetTrigger())
				{
					sparky.TankDrive(stick2, stick1);
				}
				else if(stick1.GetRawButton(8) && !ds->GetDigitalIn(5))
				{
					g_autoAimSet = true;
					autoAim.Start();
				}
				else
				{
					sparky.TankDrive(MOTOR_OFF, MOTOR_OFF);
				}
			}
			
			// bridge arm
			if(stick1.GetRawButton(6))
			{
				if(!bridgeArmDown.Get())
				{
					armDown = true;
				}
				if(armUp && !bridgeArmUp.Get())
				{
					armUp = false;
				}
				if(!armDown || ds->GetDigitalIn(6))
				{
					bridgeArm.Set(BRIDGE_ARM_UP);
				}
				else
				{
					bridgeArm.Set(BRIDGE_ARM_OFF);
				}
			}
			else if(stick1.GetRawButton(7))
			{
				if(!bridgeArmUp.Get())
				{
					armUp = true;
				}
				if(armDown && !bridgeArmDown.Get())
				{
					armDown = false;
				}
				if(!armUp || ds->GetDigitalIn(6))
				{
					bridgeArm.Set(BRIDGE_ARM_DOWN);
				}
				else
				{
					bridgeArm.Set(BRIDGE_ARM_OFF);
				}
			}
			else
			{
				bridgeArm.Set(BRIDGE_ARM_OFF);
			}
			
			// shooter arm
			if(!armSet)
			{
				// zero encoder
				if(ds->GetDigitalIn(4))
				{
					if(stick3.GetRawButton(8))
					{
						tension.Reset();
					}
				}
				
				// coarse adjustment
				if(stick3.GetRawButton(3))
				{
					if(tension.Get() > 0 || ds->GetDigitalIn(4))
					{
						arm.Set(ARM_SPEED_COARSE_UNLOAD);
					}
				}
				else if(stick3.GetRawButton(2) && shooter.Get())
				{
					arm.Set(ARM_SPEED_COARSE_LOAD);
				}
				// fine adjustment
				else if(stick3.GetRawButton(5) && shooter.Get())
				{
					arm.Set(ARM_SPEED_FINE_LOAD);
				}
				else if(stick3.GetRawButton(4))
				{
					if(tension.Get() > 0 || ds->GetDigitalIn(4))
					{
						arm.Set(ARM_SPEED_FINE_UNLOAD);
					}
				}
				// move to preset
				else if(stick3.GetRawButton(9))
				{
					encPos = 115;
					armSet = true;
					armSpeed = ARM_SPEED_COARSE;
					armToPositionNotifier.StartSingle(0);
				}
				else if(stick3.GetRawButton(8))
				{
					encPos = 0;
					armSet = true;
					armSpeed = ARM_SPEED_FULL_UNLOAD;
					armToPositionNotifier.StartSingle(0);
				}
				else if(stick3.GetRawButton(10))
				{
					encPos = 175;
					armSet = true;
					armSpeed = ARM_SPEED_COARSE;
					armToPositionNotifier.StartSingle(0);
				}
				else if(stick3.GetRawButton(11))
				{
					encPos = lastPosition;
					armSet = true;
					armSpeed = ARM_SPEED_COARSE;
					armToPositionNotifier.StartSingle(0);
				}
				else
				{
					arm.Set(TENSION_BRAKE); // brake spool
				}
			}
			
			// make sure that ball isn't settling in the arm
			if(shooter.Get())
			{
				armTimer.Reset();
			}
			
			// ball loading
			if(!intakeOff)
			{
				if(stick3.GetRawButton(6))
				{
					if(shooter.Get() && top.Get() && middle.Get())
					{
						floorPickup.Set(INTAKE_OFF);
					}
					else if(top.Get() && middle.Get() && tension.Get() > ARM_ZERO_THRESH)
					{
						floorPickup.Set(INTAKE_OFF);
					}
					else
					{
						floorPickup.Set(INTAKE_LOAD);
					}
					if(!shooter.Get() && tension.Get() < ARM_ZERO_THRESH && armTimer.Get() > 1.0)
					{
						shooterLoader.Set(INTAKE_LOAD);
					}
					else if(!top.Get() && shooter.Get())

					{
						shooterLoader.Set(INTAKE_LOAD);
					}
					else if(!top.Get())
					{
						shooterLoader.Set(INTAKE_LOAD);
					}
					else
					{
						shooterLoader.Set(INTAKE_OFF);
					}
				}
				else if(stick3.GetRawButton(7))
				{
					floorPickup.Set(INTAKE_UNLOAD);
					shooterLoader.Set(INTAKE_UNLOAD);
				}
				else
				{
					floorPickup.Set(INTAKE_OFF);
					shooterLoader.Set(INTAKE_OFF);
				}
			}
		
			// release
			if(!releaseSet)
			{
				if(stick3.GetTrigger())
				{
					lastPosition = tension.Get();
					releaseSet = true;
					releaseNotifier.StartSingle(0);
				}
			}
			
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "encoder: %d", tension.Get());
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "shooter: %d", shooter.Get());
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "top: %d", top.Get());
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "middle: %d", middle.Get());
			dsLCD->UpdateLCD();
			
			Wait(0.005); // wait for a motor update time
		}
		autoAim.Stop();
		targeting.Suspend();
		blinkyLights.Suspend();
		armToPositionNotifier.Stop();
		releaseNotifier.Stop();
		printf("OperatorControl: stop\n");
	}
	
	/**
	 * Task to handle targeting with the webcam.  Displays distance and target offset.
	 */
	static int Targeting(void)
	{
		printf("Targeting: start\n");
		vector<Threshold> thresholds;
		thresholds.push_back(Threshold(141, 253, 103, 253, 72, 255)); // LED flashlight
		//thresholds.push_back(Threshold(126, 224, 210, 255, 0, 138));  // field
		//thresholds.push_back(Threshold(0, 177, 165, 255, 0, 141));    // practice field
		//thresholds.push_back(Threshold(0, 158, 123, 255, 0, 160)); // night
		//thresholds.push_back(Threshold(107, 189, 150, 255, 68, 167)); // day
		//thresholds.push_back(Threshold(78, 210, 184, 255, 0, 190)); // day close
		ParticleFilterCriteria2 criteria[] = {
			{IMAQ_MT_BOUNDING_RECT_WIDTH, 10, 400, false, false},
			{IMAQ_MT_BOUNDING_RECT_HEIGHT, 10, 400, false, false}
		};
		double degsVert = 20;
		double pi = 3.141592653589;
		double rads = pi / (double)180;
		double tapeHeight = 1.5;
		ColorImage *image = NULL;
		double fovVert, dv = 0;
		double lastDist = 0;
		double distCount = 0;
		int centerMassX;
		int centerWidth = 320 / 2;
		int centerThresh = 20;
		bool found = false;
		ParticleAnalysisReport *target = NULL;
		BinaryImage *thresholdImage = NULL;
		BinaryImage *convexHullImage = NULL;
		BinaryImage *bigObjectsImage = NULL;
		BinaryImage *filteredImage = NULL;
		vector<ParticleAnalysisReport> *reports = NULL;
		ParticleAnalysisReport *r = NULL;
		bool imageError = false;
		unsigned i, j;
		
		DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "");
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "");
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "");
		dsLCD->UpdateLCD();
		
		DriverStation *ds = DriverStation::GetInstance();

		while(true) {
			if(!camera->IsFreshImage()) 
			{
				printf("Image is not fresh.\n");
				Wait(1.0);
				continue;
			}
			if(ds->GetDigitalIn(5))
			{
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Targeting Disabled");
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "");
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "");
				dsLCD->UpdateLCD();
				Wait(1.0);
				continue;
			}
			
			found = false;
			image = new RGBImage();
			camera->GetImage(image);
			
			if(image->GetWidth() == 0 || image->GetHeight() == 0)
			{
				printf("Image width or height is 0.\n");
				delete image;
				Wait(1.0);
				continue;
			}
						
			// loop through our threshold values
			for(i = 0; i < thresholds.size() && !found; i++)
			{
				thresholdImage = image->ThresholdRGB(thresholds.at(i));
				if(!thresholdImage)
				{
					imageError = true;
				}
				if(!imageError)
				{
					convexHullImage = thresholdImage->ConvexHull(false);  // fill in partial and full rectangles
					if(!convexHullImage)
					{
						imageError = true;
					}
				}
				if(!imageError)
				{
					bigObjectsImage = convexHullImage->ParticleFilter(criteria, 2);  // find the rectangles
					if(!bigObjectsImage)
					{
						imageError = true;
					}
				}
				if(!imageError)
				{
					filteredImage = bigObjectsImage->RemoveSmallObjects(false, 2);  // remove small objects (noise)
					if(!filteredImage)
					{
						imageError = true;
					}
				}
				if(!imageError)
				{
					reports = filteredImage->GetOrderedParticleAnalysisReports();  // get the results
				}
				
				// loop through the reports
				for (j = 0; reports && j < reports->size(); j++)
				{
					r = &(reports->at(j));

					// get the bottom-most basket
					if(!target || target->center_mass_y < r->center_mass_y)
					{
						fovVert = (double)(tapeHeight * (double)r->imageHeight) / (double)r->boundingRect.height;
						dv = (double)(fovVert / (double)2) / tan(degsVert * rads);
						target = r;
						centerMassX = target->center_mass_x;
					}
					found = true;
				}
				
				if(reports && !reports->size())
				{
					printf("No particles found.\n");
				}
				else if(imageError)
				{
					printf("Image processing error.\n");
				}
				else
				{
					printf("Particles found.\n");
				}
				
			    delete filteredImage;
				delete convexHullImage;
				delete bigObjectsImage;
				delete thresholdImage;
				delete reports;
				filteredImage = NULL;
				convexHullImage = NULL;
				bigObjectsImage = NULL;
				thresholdImage = NULL;
				reports = NULL;
				target = NULL;
				imageError = false;
			}
			
			// determine how many times we've seen a reading
			if(!distCount)
			{
				distCount++;
			}
			else
			{	
				if(lastDist == dv ||
				   (lastDist < dv && dv - lastDist < 1) ||
				   (lastDist > dv && lastDist - dv < 1))
				{
					distCount++;
				}
				else
				{
					distCount = 0;
				}	
			}
			lastDist = dv;
			
			// write to the dashboard if we've seen the same value a certain number of times
			if(distCount > 3)
			{
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "target: %f", dv);
				if(centerMassX == centerWidth ||
				   (centerMassX > centerWidth && centerMassX - centerWidth < centerThresh) ||
				   (centerMassX < centerWidth && centerWidth - centerMassX < centerThresh))
				{
					dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "%s (%d px %s)", "CENTER",
							centerMassX > centerWidth ? centerMassX - centerWidth : centerWidth - centerMassX,
						    centerMassX > centerWidth ? "right" : "left");
					g_targetAlign = TARGET_CENTER;
				}
				else if((centerMassX > centerWidth && centerMassX - centerWidth > centerThresh))
				{
					dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "align: %s", "RIGHT");
					g_targetAlign = TARGET_RIGHT;
				}
				else if ((centerMassX < centerWidth && centerWidth - centerMassX > centerThresh))
				{
					dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "align: %s", "LEFT");
					g_targetAlign = TARGET_LEFT;
				}
			}
			else
			{
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "*** NO TARGET ***");
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "");
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "");
				g_targetAlign = TARGET_NONE;
			}
			dsLCD->UpdateLCD();
			
			g_targetDistance = dv;
			dv = 0;
			
			delete image;
			Wait(0.2);
		}
		printf("Targeting: stop\n");
		
		return 0;
	}
	
	void ArmToPosition(int p)
	{
		if(tension.Get() < p && shooter.Get())
		{
			while(tension.Get() < p && this->IsEnabled())
			{
				arm.Set(ARM_SPEED_COARSE_LOAD);
				sparky.TankDrive(MOTOR_OFF, MOTOR_OFF);
			}
		}
		else if(tension.Get() > p)
		{
			while(tension.Get() > p && this->IsEnabled())
			{
				arm.Set(ARM_SPEED_COARSE_UNLOAD);
				sparky.TankDrive(MOTOR_OFF, MOTOR_OFF);
			}
		}
		arm.Set(TENSION_BRAKE);
	}
	
	void ArmToPositionNoEye(int p)
	{
		if(tension.Get() < p)
		{
			while(tension.Get() < p && this->IsEnabled())
			{
				arm.Set(ARM_SPEED_COARSE_LOAD);
				sparky.TankDrive(MOTOR_OFF, MOTOR_OFF);
			}
		}
		else if(tension.Get() > p)
		{
			while(tension.Get() > p && this->IsEnabled())
			{
				arm.Set(ARM_SPEED_COARSE_UNLOAD);
				sparky.TankDrive(MOTOR_OFF, MOTOR_OFF);
			}
		}
		arm.Set(TENSION_BRAKE);
	}
	
	void ArmToPositionFull(int p)
	{
		if(tension.Get() < p && shooter.Get())
		{
			while(tension.Get() < p && this->IsEnabled())
			{
				arm.Set(ARM_SPEED_FULL_LOAD);
				Wait(0.005);
			}
		}
		else if(tension.Get() > p)
		{
			while(tension.Get() > p && this->IsEnabled())
			{
				arm.Set(ARM_SPEED_FULL_UNLOAD);
				Wait(0.005);

			}
		}
		arm.Set(TENSION_BRAKE);
	}
	
	Encoder* GetTension()
	{
		return &tension;
	}
	
	Jaguar* GetArm()
	{
		return &arm;
	}
	
	DigitalInput* GetShooter()
	{
		return &shooter;
	}
	
	DigitalInput* GetTop()
	{
		return &top;
	}
	
	DigitalInput* GetMiddle()
	{
		return &middle;
	}
	
	Victor* GetBridgeArm()
	{
		return &bridgeArm;
	}
	
	Relay* GetRelease()
	{
		return &release;
	}
	
	DigitalInput* GetTrigger()
	{
		return &trigger;
	}
	
	Victor* GetShooterLoader()
	{
		return &shooterLoader;
	}
	
	Relay* GetLights()
	{
		return &lights;
	}
	
	static void ArmToPositionNotifier(void* p)
	{
		Sparky *s = (Sparky *)p;
		Encoder *t = s->GetTension();
		Jaguar *a = s->GetArm();
		DigitalInput *shoot = s->GetShooter();
		{
			Synchronized sync(armSem);
			if(t->Get() < encPos && shoot->Get())
			{
				while(t->Get() < encPos)
				{
					a->Set(-armSpeed);
					Wait(0.005);
				}
			}
			else if(t->Get() > encPos)
			{
				while(t->Get() > encPos)
				{
					a->Set(armSpeed);
					Wait(0.005);
				}
			}
			a->Set(TENSION_BRAKE);
			armSet = false;
		}
	}
	
	static void ReleaseNotifier(void* p)
	{
		printf("ReleaseNotifier: start\n");
		Sparky *s = (Sparky *)p;
		Relay *r = s->GetRelease();
		Victor *sl = s->GetShooterLoader(); 
		DigitalInput *t = s->GetTrigger();
		DigitalInput *top = s->GetTop();
		Encoder *e = s->GetTension();
		{
			Synchronized sync(releaseSem);
			while(t->Get() && s->IsEnabled())
			{
				r->Set(Relay::kReverse);
				Wait(0.005);
			}
			Wait(0.1);
			r->Set(Relay::kOff);
			Wait(0.3);
			releaseSet = false;
			intakeOff = true;
			armSet = true;
			s->ArmToPositionFull(0);
			while(e->Get() > ARM_ZERO_THRESH && s->IsEnabled())
			{
				Wait(0.1);
			}
			while(top->Get() && s->IsEnabled())
			{
				sl->Set(INTAKE_LOAD);
				Wait(0.005);
			}
			Wait(1.0);
			sl->Set(INTAKE_OFF);
			s->ArmToPosition(125);
			intakeOff = false;
			armSet = false;
			printf("ReleaseNotifier: done\n");
		}
	}
	
	static int BlinkyLights(void)
	{
		printf("BlinkyLights: start\n");
		while(true)
		{
			if(g_shooter->Get() && g_top->Get() && g_middle->Get())
			{
				g_lights->Set(Relay::kForward);
			}
			else
			{
				if(g_shooter->Get())
				{
					g_lights->Set(Relay::kForward);
					Wait(0.2);
					g_lights->Set(Relay::kOff);
					Wait(0.1);
				}
				if(g_middle->Get())
				{
					g_lights->Set(Relay::kForward);
					Wait(0.2);
					g_lights->Set(Relay::kOff);
					Wait(0.1);
				}
				if(g_top->Get())
				{
					g_lights->Set(Relay::kForward);
					Wait(0.2);
					g_lights->Set(Relay::kOff);
					Wait(0.1);
				}
			}
			//printf("Blinking!\n");
			Wait(1.0);
		}
		printf("BlinkyLights: done\n");
		return 0;
	}
	
	static int AutoAim(void)
	{
		Synchronized sync(autoAimSem);
		printf("AutoAim: start\n");
		
		targetAlignment ta = g_targetAlign;
		double d = g_targetDistance;
		
		while(ta != TARGET_CENTER)
		{
			if(ta == TARGET_RIGHT)
			{
				g_sparky->TankDrive(AUTO_AIM_SPEED, -AUTO_AIM_SPEED);
			}
			else if(ta == TARGET_LEFT)
			{
				g_sparky->TankDrive(-AUTO_AIM_SPEED, AUTO_AIM_SPEED);
			}
			else if(ta == TARGET_NONE)
			{
				break;
			}
			Wait(0.1);
			ta = g_targetAlign;
			d = g_targetDistance;
		}

		g_sparky->TankDrive(MOTOR_OFF, MOTOR_OFF);
		g_autoAimSet = false;
		printf("AutoAim: done\n");
		return 0;
	}
};

START_ROBOT_CLASS(Sparky);
