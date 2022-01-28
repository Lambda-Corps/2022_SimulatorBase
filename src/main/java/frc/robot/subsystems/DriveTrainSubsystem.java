// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;


import static frc.robot.Constants.*;

public class DriveTrainSubsystem extends SubsystemBase {
  /** Creates a new DriveTrainSubsystem. */
  private final Timer timer1;
  private final WPI_TalonFX m_leftDrive, m_leftFollower, m_rightDrive, m_rightFollower;
  private final DifferentialDrive m_safety_drive;
  private final AHRS m_navx;
  
  // Variables to hold the invert types for the talons
  private final TalonFXInvertType m_left_invert, m_right_invert;

  /** Config Objects for motor controllers */
  private final TalonFXConfiguration _leftConfig, _rightConfig;

  /* Object for simulated inputs into Talon. */
  private final TalonFXSimCollection m_leftDriveSim, m_rightDriveSim;

  /*
   * Creating my odometry object. Here,
   * our starting pose is 5 meters along the long end of the field and in the
   * center of the field along the short end, facing forward.
   */
  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;
  // These classes help us simulate our drivetrain
  public DifferentialDrivetrainSim m_drivetrainSimulator;
  private final Field2d m_fieldSim;


  // =======================================================================

  public DriveTrainSubsystem() {
    m_navx = new AHRS(SPI.Port.kMXP);
    timer1 = new Timer();
    timer1.start();

    m_leftDrive = new WPI_TalonFX(0, "FastFD");
    m_leftFollower = new WPI_TalonFX(LEFT_TALON_LEADER, "FastFD");
    m_rightDrive = new WPI_TalonFX(RIGHT_TALON_LEADER, "FastFD");
    m_rightFollower = new WPI_TalonFX(3, "FastFD");

    // Setup the Simulation input classes
    m_leftDriveSim = m_leftDrive.getSimCollection();
    m_rightDriveSim = m_rightDrive.getSimCollection();
    
    m_safety_drive = new DifferentialDrive(m_leftDrive, m_rightDrive);

    /* Factory Default for Talons */
    m_rightDrive.configFactoryDefault(); // Defaults to reading quad encoder
    m_leftDrive.configFactoryDefault();
    m_rightFollower.configFactoryDefault();
    m_leftFollower.configFactoryDefault();
 
    _leftConfig = new TalonFXConfiguration();
    _rightConfig = new TalonFXConfiguration();

    /* 
		 * Currently, in order to use a product-specific FeedbackDevice in configAll objects,
		 * you have to call toFeedbackType. This is a workaround until a product-specific
		 * FeedbackDevice is implemented for configSensorTerm
		 */
		_leftConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Feedback Source

		/* Configure the Remote (Left) Talon's selected sensor as a remote sensor for the right Talon */
		_rightConfig.remoteFilter1.remoteSensorDeviceID = m_leftDrive.getDeviceID(); //Device ID of Remote Source
		_rightConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; //Remote Source Type

		/* Config the neutral deadband. */
		_leftConfig.neutralDeadband = DriveConstants.kNeutralDeadband;
		_rightConfig.neutralDeadband = DriveConstants.kNeutralDeadband;

		/* max out the peak output (for all modes).  However you can
		 * limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		_leftConfig.peakOutputForward = +1.0;
		_leftConfig.peakOutputReverse = -1.0;
		_rightConfig.peakOutputForward = +1.0;
		_rightConfig.peakOutputReverse = -1.0;

		/* FPID Gains for turn servo */
		/* FPID for Distance */
		_rightConfig.slot1.kF = DriveConstants.kGains_Turning.kF;
		_rightConfig.slot1.kP = DriveConstants.kGains_Turning.kP;
		_rightConfig.slot1.kI = DriveConstants.kGains_Turning.kI;
		_rightConfig.slot1.kD = DriveConstants.kGains_Turning.kD;
		_rightConfig.slot1.integralZone = DriveConstants.kGains_Turning.kIzone;
		_rightConfig.slot1.closedLoopPeakOutput = DriveConstants.kGains_Turning.kPeakOutput;
			
		/* 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		_rightConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
		_rightConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
		_rightConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
    _rightConfig.slot3.closedLoopPeriod = closedLoopTimeMs;
    
    /* APPLY the config settings */
		m_leftDrive.configAllSettings(_leftConfig);
    m_rightDrive.configAllSettings(_rightConfig);
    
    m_rightFollower.follow(m_rightDrive);
    m_rightFollower.setInverted(InvertType.FollowMaster);
    
    m_leftFollower.follow(m_leftDrive);
    m_leftFollower.setInverted(InvertType.FollowMaster);

    m_leftDrive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    m_rightDrive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    /** Invert Directions for Left and Right */
    m_left_invert = TalonFXInvertType.CounterClockwise; //Same as invert = "false"
    m_right_invert = TalonFXInvertType.Clockwise; //Same as invert = "true"

    // On the robot, the left side is positive forward, so don't change it.
    m_leftDrive.setInverted(m_left_invert);

    // On the robot, the right side output needs to be inverted so that positive is forward.
    m_rightDrive.setInverted(m_right_invert);

    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's position/velocity.
     * 
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
     */
    //m_leftDrive.setSensorPhase(false);
    //m_rightDrive.setSensorPhase(false);
    /* Setup difference signal to be used for turn when performing Drive Straight with encoders */
		setRobotTurnConfigs(m_right_invert, _rightConfig);
    // Reset the external encoders
    resetEncoders();

    // Put the 2d Field on smartdashboard to visualize the robot on the field (estimated)
    m_fieldSim = new Field2d();
    SmartDashboard.putData("Field", m_fieldSim);
    // ==================================================================
    // Code for simulation within the DriveTrain Constructor
    if (RobotBase.isSimulation()) { // If our robot is simulated
      // This class simulates our drivetrain's motion around the field.
      /* Simulation model of the drivetrain */
      m_drivetrainSimulator = new DifferentialDrivetrainSim(
        DCMotor.getFalcon500(2), // 2 Falcon 500s on each side of the drivetrain.
        DriveConstants.kGearRatio, // Standard AndyMark Gearing reduction.
        2.1, // MOI of 2.1 kg m^2 (from CAD model).
        26.5, // Mass of the robot is 26.5 kg.
        Units.inchesToMeters(DriveConstants.kWheelRadiusInches), // Robot uses 3" radius (6" diameter) wheels.
        0.546, // Distance between wheels is _ meters.

        /*
        * The standard deviations for measurement noise:
        * x and y: 0.001 m
        * heading: 0.001 rad
        * l and r velocity: 0.1 m/s
        * l and r position: 0.005 m
        */
        null // VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005) //Uncomment this
            // line to add measurement noise.
      );

      // PhysicsSim.getInstance().addTalonSRX(m_rightDrive, 0.75, 4000);
      // PhysicsSim.getInstance().addTalonSRX(m_leftDrive, 0.75, 4000);
    } // end of constructor code for the simulation

    m_odometry = new DifferentialDriveOdometry(m_navx.getRotation2d());
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_navx.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  // Talan Motor Control - Encoders
  public double getLeftEncoderValue() {
    return m_leftDrive.getSelectedSensorPosition();
  }

  public double getRightEncoderValue() {
    return m_rightDrive.getSelectedSensorPosition();
  }

  public void reset_drivetrain_encoders() {
    m_leftDrive.setSelectedSensorPosition(0, 0, 0);
    m_rightDrive.setSelectedSensorPosition(0, 0, 0);
  }

  /** Resets the Talon drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_rightDrive.setSelectedSensorPosition(0);
    m_leftDrive.setSelectedSensorPosition(0);
  }

  public void manualDrive(double move, double turn) {
    m_safety_drive.arcadeDrive(move, turn);
    m_safety_drive.feed();

    // Test the Gyro by displaying the current value on the shuffleboard
    double currentHeading = get_current_heading();
    int currentHeadingInteger = (int) (currentHeading);
    SmartDashboard.putNumber("RobotHeading", currentHeadingInteger);

    // Test the LED Strip
    // m_LEDSubsystem.SetLEDColor(((int) (64 - move * 64)), ((int) (64 + move *
    // 64)), 0); // Red Green Blue

  }

  public void reset_gyro() {
    m_navx.reset();
  }

  public double get_current_heading() {
    return m_navx.getAngle();
  }

  // ==(Added for Simulation)============================================
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  @Override
  public void periodic() {

    // ==(Added for Simulation)============================================
    /*
     * This will get the simulated sensor readings that we set
     * in the previous article while in simulation, but will use
     * real values on the robot itself.
     */
    m_odometry.update(m_navx.getRotation2d(),
                      nativeUnitsToDistanceMeters(m_leftDrive.getSelectedSensorPosition()),
                      nativeUnitsToDistanceMeters(m_rightDrive.getSelectedSensorPosition()));
    m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    /* Pass the robot battery voltage to the simulated Talon FXs */
    m_leftDriveSim.setBusVoltage(RobotController.getBatteryVoltage());
    m_rightDriveSim.setBusVoltage(RobotController.getBatteryVoltage());

    /*
     * CTRE simulation is low-level, so SimCollection inputs
     * and outputs are not affected by SetInverted(). Only
     * the regular user-level API calls are affected.
     *
     * WPILib expects +V to be forward.
     * Positive motor output lead voltage is ccw. We observe
     * on our physical robot that this is reverse for the
     * right motor, so negate it.
     *
     * We are hard-coding the negation of the values instead of
     * using getInverted() so we can catch a possible bug in the
     * robot code where the wrong value is passed to setInverted().
     */
    m_drivetrainSimulator.setInputs(m_leftDriveSim.getMotorOutputLeadVoltage(),
                         -m_rightDriveSim.getMotorOutputLeadVoltage());

    /*
     * Advance the model by 20 ms. Note that if you are running this
     * subsystem in a separate thread or have changed the nominal
     * timestep of TimedRobot, this value needs to match it.
     */
    m_drivetrainSimulator.update(0.02);

    /*
     * Update all of our sensors.
     *
     * Since WPILib's simulation class is assuming +V is forward,
     * but -V is forward for the right motor, we need to negate the
     * position reported by the simulation class. Basically, we
     * negated the input, so we need to negate the output.
     */
    m_leftDriveSim.setIntegratedSensorRawPosition(
                    distanceToNativeUnits(
                        m_drivetrainSimulator.getLeftPositionMeters()
                    ));
    m_leftDriveSim.setIntegratedSensorVelocity(
                    velocityToNativeUnits(
                        m_drivetrainSimulator.getLeftVelocityMetersPerSecond()
                    ));
    m_rightDriveSim.setIntegratedSensorRawPosition(
                    distanceToNativeUnits(
                        -m_drivetrainSimulator.getRightPositionMeters()
                    ));
    m_rightDriveSim.setIntegratedSensorVelocity(
                    velocityToNativeUnits(
                        -m_drivetrainSimulator.getRightVelocityMetersPerSecond()
                    ));

    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(-m_drivetrainSimulator.getHeading().getDegrees());
  }

  public double getTimer1Value() {
    return timer1.get();
  }

  private int distanceToNativeUnits(double positionMeters){
    double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelRadiusInches));
    double motorRotations = wheelRotations * DriveConstants.kSensorGearRatio;
    int sensorCounts = (int)(motorRotations * DriveConstants.kCountsPerRev);
    return sensorCounts;
  }

  private int velocityToNativeUnits(double velocityMetersPerSecond){
    double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelRadiusInches));
    double motorRotationsPerSecond = wheelRotationsPerSecond * DriveConstants.kSensorGearRatio;
    double motorRotationsPer100ms = motorRotationsPerSecond / DriveConstants.k100msPerSecond;
    int sensorCountsPer100ms = (int)(motorRotationsPer100ms * DriveConstants.kCountsPerRev);
    return sensorCountsPer100ms;
  }

  private double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / DriveConstants.kCountsPerRev;
    double wheelRotations = motorRotations / DriveConstants.kSensorGearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelRadiusInches));
    return positionMeters;
  }

  /** 
	 * Determines if SensorSum or SensorDiff should be used 
	 * for combining left/right sensors into Robot Distance.  
	 * 
	 * Assumes Aux Position is set as Remote Sensor 0.  
	 * 
	 * configAllSettings must still be called on the master config
	 * after this function modifies the config values. 
	 * 
	 * @param masterInvertType Invert of the Master Talon
	 * @param masterConfig Configuration object to fill
	 */
	void setRobotTurnConfigs(TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig){
		/**
		 * Determine if we need a Sum or Difference.
		 * 
		 * The auxiliary Talon FX will always be positive
		 * in the forward direction because it's a selected sensor
		 * over the CAN bus.
		 * 
		 * The master's native integrated sensor may not always be positive when forward because
		 * sensor phase is only applied to *Selected Sensors*, not native
		 * sensor sources.  And we need the native to be combined with the 
		 * aux (other side's) distance into a single robot heading.
		 */

		/* THIS FUNCTION should not need to be modified. 
		   This setup will work regardless of whether the master
		   is on the Right or Left side since it only deals with
		   heading magnitude.  */

		/* Check if we're inverted */
		if (masterInvertType == TalonFXInvertType.Clockwise){
			/* 
				If master is inverted, that means the integrated sensor
				will be negative in the forward direction.
				If master is inverted, the final sum/diff result will also be inverted.
				This is how Talon FX corrects the sensor phase when inverting 
				the motor direction.  This inversion applies to the *Selected Sensor*,
				not the native value.
				Will a sensor sum or difference give us a positive heading?
				Remember the Master is one side of your drivetrain distance and 
				Auxiliary is the other side's distance.
					Phase | Term 0   |   Term 1  | Result
				Sum:  -1 *((-)Master + (+)Aux   )| OK - magnitude will cancel each other out
				Diff: -1 *((-)Master - (+)Aux   )| NOT OK - magnitude increases with forward distance.
				Diff: -1 *((+)Aux    - (-)Master)| NOT OK - magnitude decreases with forward distance
			*/

			masterConfig.sum0Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Integrated Sensor
			masterConfig.sum1Term = TalonFXFeedbackDevice.RemoteSensor1.toFeedbackDevice();   //Aux Selected Sensor
			masterConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); //Sum0 + Sum1

			/*
				PID Polarity
				With the sensor phasing taken care of, we now need to determine if the PID polarity is in the correct direction
				This is important because if the PID polarity is incorrect, we will run away while trying to correct
				Will inverting the polarity give us a positive counterclockwise heading?
				If we're moving counterclockwise(+), and the master is on the right side and inverted,
				it will have a negative velocity and the auxiliary will have a negative velocity
				 heading = right + left
				 heading = (-) + (-)
				 heading = (-)
				Let's assume a setpoint of 0 heading.
				This produces a positive error, in order to cancel the error, the right master needs to
				drive backwards. This means the PID polarity needs to be inverted to handle this
				
				Conversely, if we're moving counterclwise(+), and the master is on the left side and inverted,
				it will have a positive velocity and the auxiliary will have a positive velocity.
				 heading = right + left
				 heading = (+) + (+)
				 heading = (+)
				Let's assume a setpoint of 0 heading.
				This produces a negative error, in order to cancel the error, the left master needs to
				drive forwards. This means the PID polarity needs to be inverted to handle this
			*/
			masterConfig.auxPIDPolarity = true;
		} else {
			/* Master is not inverted, both sides are positive so we can diff them. */
			masterConfig.diff0Term = TalonFXFeedbackDevice.RemoteSensor1.toFeedbackDevice();    //Aux Selected Sensor
			masterConfig.diff1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local IntegratedSensor
			masterConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); //Sum0 + Sum1
			/* With current diff terms, a counterclockwise rotation results in negative heading with a right master */
			masterConfig.auxPIDPolarity = true;
		}
		/**
		 * Heading units should be scaled to ~4000 per 360 deg, due to the following limitations...
		 * - Target param for aux PID1 is 18bits with a range of [-131072,+131072] units.
		 * - Target for aux PID1 in motion profile is 14bits with a range of [-8192,+8192] units.
		 *  ... so at 3600 units per 360', that ensures 0.1 degree precision in firmware closed-loop
		 *  and motion profile trajectory points can range +-2 rotations.
		 */
		masterConfig.auxiliaryPID.selectedFeedbackCoefficient = DriveConstants.kTurnTravelUnitsPerRotation / DriveConstants.kEncoderUnitsPerRotation;
	}
}
