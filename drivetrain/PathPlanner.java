package frc.libzodiac.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.events.PointTowardsZoneTrigger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

public class PathPlanner {
    private static final boolean SWERVE_SETPOINT_GENERATOR_ENABLED = true;
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    private static PathPlanner instance;
    private final BaseDrivetrain drivetrain;
    private final SwerveSetpointGenerator swerveSetpointGenerator;
    private SwerveSetpoint previousSetpoint = null;
    private RobotConfig config;

    private PathPlanner(BaseDrivetrain drivetrain) {
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception ignored) {
        }

        this.drivetrain = drivetrain;

        if (this.drivetrain.isSwerve()) {
            this.swerveSetpointGenerator = new SwerveSetpointGenerator(config, this.drivetrain.getMaxAngularVelocity());
            this.previousSetpoint = new SwerveSetpoint(this.drivetrain.getRobotRelativeSpeeds(),
                    this.drivetrain.getModuleStates(), DriveFeedforwards.zeros(config.numModules));
        } else {
            this.swerveSetpointGenerator = null;
        }

        AutoBuilder.configure(this.drivetrain::getPose, // Robot pose supplier
                this.drivetrain::setPose,
                // Method to reset odometry (will be called if your auto has a starting pose)
                this.drivetrain::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (ChassisSpeeds speeds, DriveFeedforwards feedforwards) -> this.drivetrain.driveRobotRelative(speeds),
                // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also, optionally outputs individual module feedforwards
                this.drivetrain.getPathFollowingController(), // The path following controller
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                }, this.drivetrain.getSubsystemBase() // Reference to this subsystem to set requirements
        );
    }

    public static void initInstance(BaseDrivetrain drivetrain) {
        if (instance == null) {
            instance = new PathPlanner(drivetrain);
        }
        instance.warmup();
    }

    public void warmup() {
        FollowPathCommand.warmupCommand().schedule();
        PathfindingCommand.warmupCommand().schedule();
    }

    public static PathPlanner getInstance() {
        return instance;
    }

    /**
     * This method will take in desired robot-relative chassis speeds,
     * then generate a swerve setpoint.
     *
     * @param speeds The desired robot-relative speeds.
     * @return SwerveModuleState[] The desired swerve module states.
     */
    public static SwerveModuleState[] generateSwerveSetpoint(ChassisSpeeds speeds) {
        if (!SWERVE_SETPOINT_GENERATOR_ENABLED || instance == null || instance.swerveSetpointGenerator == null) {
            return null;
        }

        // Note: it is important to not discretize speeds before or after
        // using the setpoint generator, as it will discretize them for you
        instance.previousSetpoint = instance.swerveSetpointGenerator.generateSetpoint(instance.previousSetpoint,
                // The previous setpoint
                speeds, // The desired target speeds
                0.02 // The loop time of the robot code, in seconds
        );
        return instance.previousSetpoint.moduleStates();
    }

    public void registerCommand(String name, Command command) {
        NamedCommands.registerCommand(name, command);
    }

    public Trigger getEventTrigger(String name) {
        return new EventTrigger("shoot note");
    }

    public Trigger getPointTowardsZoneTrigger(String name) {
        return new PointTowardsZoneTrigger(name);
    }

    public PathPlannerPath createPath(List<Pose2d> poses, GoalEndState goalEndState) {
        return createPath(poses, goalEndState, false);
    }

    public PathPlannerPath createPath(List<Pose2d> poses, GoalEndState goalEndState, boolean noFlip) {
        // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI,
                4 * Math.PI); // The constraints for this path.

        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, goalEndState);

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = noFlip;

        return path;
    }

    public SendableChooser<Command> buildAutoChooser() {
        return AutoBuilder.buildAutoChooser();
    }
}
