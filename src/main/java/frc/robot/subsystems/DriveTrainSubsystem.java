// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.LEFT_TALON_LEADER;
import static frc.robot.Constants.RANGE_FINDER_PORT;
import static frc.robot.Constants.RIGHT_TALON_LEADER;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.VecBuilder;
import frc.robot.Constants.DriveConstants;
import frc.robot.sim.PhysicsSim;

public class DriveTrainSubsystem extends SubsystemBase {
    /** Creates a new DriveTrainSubsystem. */

    private final Timer timer1;
    private final WPI_TalonSRX m_rightLeader; // Declare motor controllers variables
    private final WPI_TalonSRX m_leftLeader;
    private final DifferentialDrive m_safety_drive; // Declare drive train core function

    // The left-side drive encoder (connected to the RoboRIO DIOs)
    private final Encoder m_leftEncoder = new Encoder(DriveConstants.kLeftEncoderPorts[0],
    DriveConstants.kLeftEncoderPorts[1], DriveConstants.kLeftEncoderReversed);
    // The right-side drive encoder (connected to the RoboRIO DIOs)
    private final Encoder m_rightEncoder = new Encoder(DriveConstants.kRightEncoderPorts[0],
    DriveConstants.kRightEncoderPorts[1], DriveConstants.kRightEncoderReversed);

    // Add the stand-alone gyro sensor
    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

    private final AHRS ahrs;

    private int percentComplete = 0;

    private AnalogInput Rangefinder;

    // ==(Added for simulation)===============================================
    private Field2d m_fieldSim;

    // These classes help us simulate our drivetrain
    public DifferentialDrivetrainSim m_drivetrainSimulator;
    private EncoderSim m_leftEncoderSim;
    private EncoderSim m_rightEncoderSim;
    private ADXRS450_GyroSim m_gyroSim;

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
      Rotation2d.fromDegrees(getHeading()));
     // =======================================================================

    public DriveTrainSubsystem() {

      timer1 = new Timer();
      timer1.start();

      m_rightLeader = new WPI_TalonSRX(RIGHT_TALON_LEADER); // Instantiate motor controllers
      m_leftLeader = new WPI_TalonSRX(LEFT_TALON_LEADER);
      m_safety_drive = new DifferentialDrive(m_leftLeader, m_rightLeader);
      /* Factory Default for Talons */
      m_rightLeader.configFactoryDefault(); // Defaults to reading quad encoder
      m_leftLeader.configFactoryDefault();
      m_rightLeader.setSensorPhase(true); // Invert the Right Talon's quad encoder value

      // ==================================================================
      // Configure the external encoders
      // Sets the distance per pulse for the encoders
      m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
      m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

      // Reset the external encoders
      resetEncoders();

      ahrs = new AHRS(SPI.Port.kMXP);
      Rangefinder = new AnalogInput(RANGE_FINDER_PORT);
      // ==================================================================
      // Code for simulation within the DriveTrain Constructor
      if (RobotBase.isSimulation()) { // If our robot is simulated
          // This class simulates our drivetrain's motion around the field.
          m_drivetrainSimulator = new DifferentialDrivetrainSim(DriveConstants.kDrivetrainPlant,
          DriveConstants.kDriveGearbox, DriveConstants.kDriveGearing, DriveConstants.kTrackwidthMeters,
          DriveConstants.kWheelDiameterMeters / 2.0, VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

          // The encoder and gyro angle sims let us set simulated sensor readings
          m_leftEncoderSim = new EncoderSim(m_leftEncoder);
          m_rightEncoderSim = new EncoderSim(m_rightEncoder);
          m_gyroSim = new ADXRS450_GyroSim(m_gyro);

          // the Field2d class lets us visualize our robot in the simulation GUI.
          m_fieldSim = new Field2d();
          SmartDashboard.putData("Field", m_fieldSim);

          PhysicsSim.getInstance().addTalonSRX(m_rightLeader, 0.75, 4000);
          PhysicsSim.getInstance().addTalonSRX(m_leftLeader, 0.75, 4000);
      } // end of constructor code for the simulation
   }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  // Talan Motor Control - Encoders
  public double getLeftEncoderValue() {
    return m_leftLeader.getSelectedSensorPosition();
  }

  public double getRightEncoderValue() {
    return m_rightLeader.getSelectedSensorPosition();
  }

  public void reset_drivetrain_encoders() {
    m_leftLeader.setSelectedSensorPosition(0, 0, 0);
    m_rightLeader.setSelectedSensorPosition(0, 0, 0);
  }

  /** Resets the Talon drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
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
    ahrs.reset();
  }

  public double get_current_heading() {
    return ahrs.getAngle();
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

  // for finding the distance from the range finder
  public double getRangeFinderDistance() {
    double rangefinderVoltage = Rangefinder.getAverageVoltage();
    double distanceInInches = (rangefinderVoltage * 65.4) - 7.2;
    return distanceInInches;
  }

  @Override
  public void periodic() {

    // ==(Added for Simulation)============================================
    // Update the odometry in the periodic block
    // Read the current heading (not sure where from) and encoder values and feed to
    // Odometry model
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

    // Get the pose from the drive train and send it to the simulated field.
    m_fieldSim.setRobotPose(getPose());

    // ==(Added for Testing and Troubleshooting)===================================

    int encoder_output = m_rightEncoder.getRaw();
    SmartDashboard.putNumber("encoder_output:  ", encoder_output);

    double encoder_distance_output = m_rightEncoder.getDistance();
    SmartDashboard.putNumber("encoder_distance_output:  ", encoder_distance_output);

    double left_Talon_encoder_output = getLeftEncoderValue();
    SmartDashboard.putNumber("left_Talon_encoder_output:  ", left_Talon_encoder_output);

    double right_Talon_encoder_output = getRightEncoderValue();
    SmartDashboard.putNumber("right_Talon_encoder_output:  ", right_Talon_encoder_output);

    SmartDashboard.putNumber("NAVX Gyro output:  ", ahrs.getAngle());

  }

  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and
    // gyro.
    // We negate the right side so that positive voltages make the right side
    // move forward.

    PhysicsSim.getInstance().run();

    m_drivetrainSimulator.setInputs(m_leftLeader.get() * RobotController.getBatteryVoltage(),
        -m_rightLeader.get() * RobotController.getBatteryVoltage());
    m_drivetrainSimulator.update(0.020);

    m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());

    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(-m_drivetrainSimulator.getHeading().getDegrees());
  }

  public double getTimer1Value() {
    return timer1.get();
  }

}
