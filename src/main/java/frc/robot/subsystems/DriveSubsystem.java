package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DriveSubsystem implements Subsystem{
    private final WPI_TalonSRX m_leftMotorLeader = new WPI_TalonSRX(10);
    private final WPI_TalonSRX m_leftMotorFollower = new WPI_TalonSRX(11);
    private final WPI_TalonSRX m_rightMotorLeader = new WPI_TalonSRX(12);
    private final WPI_TalonSRX m_rightMotorFollower = new WPI_TalonSRX(13);

    private final PneumaticHub m_pH = new PneumaticHub(1);
    private final DoubleSolenoid m_solnoidLeft = m_pH.makeDoubleSolenoid(0,1);
    private final DoubleSolenoid m_solnoidRight = m_pH.makeDoubleSolenoid(2,3);
    private boolean m_isGearedFast = false;

    private DifferentialDrive m_robotDrive;

    public DriveSubsystem() {
        m_leftMotorFollower.follow(m_leftMotorLeader);
        m_rightMotorFollower.follow(m_rightMotorLeader);

        m_rightMotorLeader.configPeakCurrentLimit(25);
        m_leftMotorLeader.configPeakCurrentLimit(25);
        m_rightMotorFollower.configPeakCurrentLimit(25);
        m_leftMotorFollower.configPeakCurrentLimit(25);

        m_rightMotorLeader.setNeutralMode(NeutralMode.Brake);
        m_leftMotorLeader.setNeutralMode(NeutralMode.Brake);
        m_rightMotorFollower.setNeutralMode(NeutralMode.Brake);
        m_leftMotorFollower.setNeutralMode(NeutralMode.Brake);
        
        m_rightMotorLeader.setInverted(true);
        m_leftMotorFollower.setInverted(InvertType.FollowMaster);
        m_rightMotorFollower.setInverted(InvertType.FollowMaster);

        m_rightMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        m_leftMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);


        m_rightMotorLeader.config_kP(0, 1.0);
        m_rightMotorLeader.config_kI(0, 0.000001);

        m_leftMotorLeader.config_kP(0, 1.0);
        m_leftMotorLeader.config_kI(0, 0.000001);

        shiftGearLowSpeed();

        m_robotDrive = new DifferentialDrive(m_leftMotorLeader, m_rightMotorLeader);
    }   
    public void shiftGearHighSpeed() {
        m_isGearedFast = true;
        m_solnoidLeft.set(DoubleSolenoid.Value.kReverse);
        m_solnoidRight.set(DoubleSolenoid.Value.kReverse);
    }
    public void shiftGearLowSpeed() {
        m_isGearedFast = false;
        m_solnoidLeft.set(DoubleSolenoid.Value.kForward);
        m_solnoidRight.set(DoubleSolenoid.Value.kForward);   
    }

    public void driveArcade(double forward, double turn) {
        m_robotDrive.arcadeDrive(forward, turn);
    }

    public void driveTank(double left, double right) {
        m_robotDrive.tankDrive(left, right);
    }
    
    @Override
    public void periodic() {
    }
}

