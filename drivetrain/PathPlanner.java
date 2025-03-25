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
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.libzodiac.api.Drivetrain;
import frc.libzodiac.util.GameUtil;

import java.util.List;

/**
 * The PathPlanner used to generate paths and commands.
 */
public class PathPlanner {
    /**
     * This boolean determines whether the swerve setpoint generator is enabled. If it is enabled,
     * the setpoint generator will be used to generate swerve module states from robot-relative
     * chassis speeds. If it is disabled, the setpoint generator will not be used.
     */
    private static final boolean SWERVE_SETPOINT_GENERATOR_ENABLED = false;
    /**
     * The instance of the PathPlanner class.
     */
    private static PathPlanner PATHPLANNER;
    /**
     * The drivetrain subsystem.
     */
    private final Drivetrain drivetrain;
    /**
     * The swerve setpoint generator.
     */
    private final SwerveSetpointGenerator swerveSetpointGenerator;
    private final PathConstraints constraints;
    /**
     * The previous swerve setpoint.
     */
    private SwerveSetpoint previousSetpoint = null;
    /**
     * The robot configuration.
     */
    private RobotConfig config;

    /**
     * The constructor for the PathPlanner class.
     *
     * @param drivetrain The drivetrain subsystem.
     */
    private PathPlanner(Drivetrain drivetrain, PathConstraints constraints) {
        /*
         * Attempt to create a robot configuration from GUI settings.
         */
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception ignored) {
        }

        /*
         * Initialize the drivetrain subsystem.
         */
        this.drivetrain = drivetrain;

        this.constraints = constraints;

        /*
         * If the drivetrain is a swerve drivetrain, create a swerve setpoint generator.
         */
        final var state = this.drivetrain.getModuleStates();
        if (state.isPresent()) {
            this.swerveSetpointGenerator = new SwerveSetpointGenerator(config,
                                                                       this.drivetrain.getMaxAngularVelocity());
            this.previousSetpoint = new SwerveSetpoint(this.drivetrain.getRobotCentricSpeeds(),
                                                       state.get(),
                                                       DriveFeedforwards.zeros(config.numModules));
        } else {
            this.swerveSetpointGenerator = null;
        }

        /*
         * Configure the auto builder.
         */
        AutoBuilder.configure(this.drivetrain::getPose, // Robot pose supplier
                              this.drivetrain::setPose,
                              // Method to reset odometry (will be called if your auto has a starting pose)
                              this.drivetrain::getRobotCentricSpeeds,
                              // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                              (ChassisSpeeds speeds, DriveFeedforwards feedforwards) -> this.drivetrain.driveRobotCentric(
                                      speeds),
                              // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also, optionally outputs individual module feedforwards
                              this.drivetrain.getPathFollowingController(),
                              // The path following controller
                              config, // The robot configuration
                              GameUtil::isRedAlliance, this.drivetrain
                              // Reference to this subsystem to set requirements
                             );
    }

    /**
     * Initialize the instance of the PathPlanner class.
     *
     * @param drivetrain The drivetrain subsystem.
     */
    public static void initInstance(Drivetrain drivetrain, PathConstraints constraints) {
        if (PATHPLANNER == null) {
            PATHPLANNER = new PathPlanner(drivetrain, constraints);
            PATHPLANNER.warmup();
        }
    }

    /**
     * Warm up PathPlanner.
     */
    private void warmup() {
        FollowPathCommand.warmupCommand().schedule();
        PathfindingCommand.warmupCommand().schedule();
    }

    /**
     * Get the instance of the PathPlanner class.
     *
     * @return PathPlanner The instance of the PathPlanner class.
     */
    public static PathPlanner getInstance() {
        return PATHPLANNER;
    }

    /**
     * Generate a swerve setpoint with the desired chassis speeds.
     *
     * @param speeds The desired chassis speeds.
     *
     * @return SwerveSetpoint The desired swerve setpoint.
     */
    public static SwerveSetpoint generateSwerveSetpoint(ChassisSpeeds speeds) {
        if (!SWERVE_SETPOINT_GENERATOR_ENABLED || PATHPLANNER == null ||
            PATHPLANNER.swerveSetpointGenerator == null) {
            return null;
        }

        PATHPLANNER.previousSetpoint = PATHPLANNER.swerveSetpointGenerator.generateSetpoint(
                PATHPLANNER.previousSetpoint,
                // The previous setpoint
                speeds, // The desired target speeds
                0.02 // The loop time of the robot code, in seconds
                                                                                           );
        return PATHPLANNER.previousSetpoint;
    }

    /**
     * Register a command with a name.
     *
     * @param name    The name of the command.
     * @param command The command to register.
     */
    public void registerCommand(String name, Command command) {
        NamedCommands.registerCommand(name, command);
    }

    /**
     * Get the event trigger by name.
     *
     * @param name The name of the event trigger.
     *
     * @return The event trigger with the specified name.
     */
    public Trigger getEventTrigger(String name) {
        return new EventTrigger(name);
    }

    /**
     * Get the point towards zone trigger by name.
     *
     * @param name The name of the point towards zone trigger.
     *
     * @return The point towards zone trigger with the specified name.
     */
    public Trigger getPointTowardsZoneTrigger(String name) {
        return new PointTowardsZoneTrigger(name);
    }

    public Command getFollowPathCommand(PathPlannerPath path) {
        return AutoBuilder.followPath(path);
    }

    public Command getFollowPathCommand(String name) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");
            return AutoBuilder.followPath(path);
        } catch (Exception ignored) {
            return Commands.none();
        }
    }

    /**
     * Create a path from a list of poses.
     *
     * @param poses The list of poses.
     *
     * @return The path created from the list of poses.
     */
    public PathPlannerPath createPath(List<Pose2d> poses, GoalEndState goalEndState) {
        return createPath(poses, goalEndState, false);
    }

    /**
     * Create a path from a list of poses.
     *
     * @param poses  The list of poses.
     * @param noFlip Whether to prevent the path from being flipped.
     *
     * @return The path created from the list of poses.
     */
    public PathPlannerPath createPath(List<Pose2d> poses, GoalEndState goalEndState,
                                      boolean noFlip) {
        // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(waypoints, this.constraints, null, goalEndState);

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = noFlip;

        return path;
    }

    public Command getFindPathCommand(Pose2d targetPose) {
        return this.getFindPathCommand(targetPose, false);
    }

    public Command getFindPathCommand(Pose2d targetPose, boolean noFlip) {
        if (noFlip) {
            return AutoBuilder.pathfindToPose(targetPose, this.constraints);
        }
        return AutoBuilder.pathfindToPoseFlipped(targetPose, this.constraints);
    }

    public Command getFindPathCommand(Pose2d targetPose, LinearVelocity goalEndVelocity) {
        return this.getFindPathCommand(targetPose, goalEndVelocity, false);
    }

    public Command getFindPathCommand(Pose2d targetPose, LinearVelocity goalEndVelocity,
                                      boolean noFlip) {
        if (noFlip) {
            return AutoBuilder.pathfindToPose(targetPose, this.constraints, goalEndVelocity);
        }
        return AutoBuilder.pathfindToPoseFlipped(targetPose, this.constraints, goalEndVelocity);
    }

    public Command getFindPathAndFollowCommand(PathPlannerPath path) {
        return AutoBuilder.pathfindThenFollowPath(path, this.constraints);
    }

    /**
     * Build an auto chooser.
     *
     * @return The auto chooser.
     */
    public SendableChooser<Command> buildAutoChooser() {
        return AutoBuilder.buildAutoChooser();
    }
}
