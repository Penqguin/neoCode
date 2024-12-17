package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;



// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
// import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.util.ReplanningConfig;
// import com.kauailabs.navx.frc.AHRS;
// import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;

// import com.pathplanner.lib.auto.ReplanningConfig;


public class drivetrain extends SubsystemBase {

    private static drivetrain drivetrain = null; //put this if you want the robot to periodically call this
    // Initializing objects

    //Motors and Controller
    private CANSparkMax leftfront;
    private CANSparkMax leftrear;
    private CANSparkMax rightfront;
    private CANSparkMax rightrear;

    // Speed Selector
    private double[] modes = {0.25, 0.5, 0.75, 1.0};

    // //Gyro stuff
    // private AHRS navx;
    // private Encoder rightEncoder;
    // private Encoder leftEncoder;
    // // private final Rotation2d rotation2d = new Rotation2d();
    // private DifferentialDriveOdometry odometry;
    // public AutoBuilder autoBuilder;
    // private DifferentialDriveKinematics kinematics;
    // private final PIDController leftPID;
    // private final PIDController rightPID;
    // private final SimpleMotorFeedforward feedforward;
    
    private drivetrain(){
        leftfront = new CANSparkMax(1,MotorType.kBrushless);
        leftrear = new CANSparkMax(2,MotorType.kBrushless);
        rightfront = new CANSparkMax(4,MotorType.kBrushless);
        rightrear = new CANSparkMax(3,MotorType.kBrushless);
        // navx = new AHRS(SPI.Port.kMXP);
        // rightEncoder = new Encoder(0,1);
        // leftEncoder = new Encoder(0,2);
        // odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(navx.getYaw()), leftEncoder.getDistance(), rightEncoder.getDistance());
        // autoBuilder = new AutoBuilder();
        // kinematics = new DifferentialDriveKinematics(1.09);
        // leftPID = new PIDController(1, 0, 0);
        // rightPID = new PIDController(1, 0, 0);
        // feedforward = new SimpleMotorFeedforward(1, 3);

        leftfront.setInverted(true);
        leftrear.setInverted(true);
        // leftrear.setInverted(true);

        leftrear.follow(leftfront);
        rightrear.follow(rightfront);
        
        // leftrear.set(ControlMode.Follower, leftfront.getDeviceId());
        // rightrear.set(ControlMode.Follower, rightfront.getDeviceId());   
    }

    //Drive Methods
    private void Stopdrive() {
        leftfront.set(0);
        rightfront.set(0);
    }

    private int index = 0;

    private double SelectSpeed(boolean rb, boolean lb){
       
        //Max gear 4 = 100% min gear 1 25%
        //right bumber is +1 vice versa
        //You have utlize the modes array and it starts at 0

        if (lb && index > 0) {
            index--;
        }
        if (rb && index < modes.length - 1) {
            index++;
        }

        return modes[index];
    }

    public void Drivecode(double Leftjoy, double Rightjoy, boolean rb, boolean lb){ {

        double s = SelectSpeed(rb, lb);

        if (Math.abs(Leftjoy) > 0.1 || Math.abs(Rightjoy) > 0.1) {
            leftfront.set((Leftjoy + Rightjoy) * s);
           rightfront.set((Leftjoy - Rightjoy) * s);

        } else {
            Stopdrive();
        }
    }

     
    }


        //pathplanner stuff *not working yet :__ (
       //edit 10/15/2024 I got it working B) -Shahriar
       //note that the encoder spin rate values will be off because it's not connected to the shaft at the end of the gearbox 
       //so the code might have to account for gear ratio 

    //     autoBuilder = new AutoBuilder();
    //     AutoBuilder.configureRamsete(

            
    //         this::getPose, // Robot pose supplier
    //         this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
    //         this::getCurrentSpeeds, // Current ChassisSpeeds supplier
    //         this::drive, // Method that will drive the robot given ChassisSpeeds
            
    //         new ReplanningConfig(), // Default path replanning config. See the API for the options here
    //         () -> {
    //           // Boolean supplier that controls when the path will be mirrored for the red alliance
    //           // This will flip the path being followed to the red side of the field.
    //           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //           var alliance = DriverStation.getAlliance();
    //           if (alliance.isPresent()) {
    //             return alliance.get() == DriverStation.Alliance.Red;
    //           }
    //           return false;
    //         },
    //         (Subsystem) this // Reference to this subsystem to set requirements
    // );
    // }
    
    // //auto methods:

    // //Gyro Stuff

    // //getPosition (pose)
    // public Pose2d getPose(){
    //     return odometry.getPoseMeters();
    // }
    // //resetPosition (reset pose)
    // public void resetPose(Pose2d pose) {
    //     leftEncoder.reset();
    //     rightEncoder.reset();
    //     navx.reset();
    //     var gyro = Rotation2d.fromDegrees(navx.getYaw());

    //     odometry.resetPosition(gyro,leftEncoder.getDistance(), rightEncoder.getDistance(), pose);
    // }


    // //Kinematics stuff

    // //get Chasis speed (getCurrentSpeeds)
    // public ChassisSpeeds getCurrentSpeeds() {
    //     return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds((leftEncoder.getRate() * 18.85)/60, rightEncoder.getRate()/60));
    // }
    // //drive method
    // public void drive(ChassisSpeeds chassisSpeeds) {
    //     DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    //     setSpeeds(wheelSpeeds);
    // }
    // //set speeds
    // public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    //     final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
    //     final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);
    
    //     final double leftOutput = leftPID.calculate(leftEncoder.getRate(), speeds.leftMetersPerSecond);
    //     final double rightOutput = rightPID.calculate(rightEncoder.getRate(), speeds.rightMetersPerSecond);
    
    //     leftfront.setVoltage(leftOutput + leftFeedforward);
    //     rightfront.setVoltage(rightOutput + rightFeedforward);
    // }

    //sends to operator interface
    public static drivetrain getInstance(){
        if (drivetrain == null){
            drivetrain = new drivetrain();
        }
        return drivetrain;
    }
}