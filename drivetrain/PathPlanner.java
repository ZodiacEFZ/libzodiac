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
import frc.libzodiac.api.Drivetrain;

import java.util.List;

/**
 * The PathPlanner used to generate paths and commands.
 */
public class PathPlanner {
    /**
     * This boolean determines whether the swerve setpoint generator is enabled.
     * If it is enabled, the setpoint generator will be used to generate swerve
     * module states from robot-relative chassis speeds. If it is disabled, the
     * setpoint generator will not be used.
     */
    private static final boolean SWERVE_SETPOINT_GENERATOR_ENABLED = true;
    /**
     * The instance of the PathPlanner class.
     */
    private static PathPlanner instance;
    /**
     * The drivetrain subsystem.
     */
    private final Drivetrain drivetrain;
    /**
     * The swerve setpoint generator.
     */
    private final SwerveSetpointGenerator swerveSetpointGenerator;
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
    private PathPlanner(Drivetrain drivetrain) {
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

        final var state = this.drivetrain.getModuleStates();

        /*
         * If the drivetrain is a swerve drivetrain, create a swerve setpoint generator.
         */
        if (state.isPresent()) {
            this.swerveSetpointGenerator = new SwerveSetpointGenerator(config, this.drivetrain.getMaxAngularVelocity());
            this.previousSetpoint = new SwerveSetpoint(this.drivetrain.getRobotRelativeSpeeds(),
                    state.get(), DriveFeedforwards.zeros(config.numModules));
        } else {
            this.swerveSetpointGenerator = null;
        }

        /*
         * Configure the auto builder.
         */
        AutoBuilder.configure(this.drivetrain::getPose, // Robot pose supplier
                this.drivetrain::setPose,
                // Method to reset odometry (will be called if your auto has a starting pose)
                this.drivetrain::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (ChassisSpeeds speeds, DriveFeedforwards feedforwards) -> this.drivetrain.driveRobotRelative(speeds),
                // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also, optionally outputs individual module feedforwards
                this.drivetrain.getPathFollowingController(), // The path following controller
                config, // The robot configuration
                () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red,
                this.drivetrain // Reference to this subsystem to set requirements
        );
    }

    /**
     * Initialize the instance of the PathPlanner class.
     *
     * @param drivetrain The drivetrain subsystem.
     */
    public static void initInstance(Drivetrain drivetrain) {
        if (instance == null) {
            instance = new PathPlanner(drivetrain);
            instance.warmup();
        }
    }

    /**
     * Get the instance of the PathPlanner class.
     *
     * @return PathPlanner The instance of the PathPlanner class.
     */
    public static PathPlanner getInstance() {
        return instance;
    }

    /**
     * Generate a swerve setpoint with the desired chassis speeds.
     *
     * @param speeds The desired chassis speeds.
     * @return SwerveModuleState[] The desired swerve module states.
     */
    public static SwerveModuleState[] generateSwerveSetpoint(ChassisSpeeds speeds) {
        if (!SWERVE_SETPOINT_GENERATOR_ENABLED || instance == null || instance.swerveSetpointGenerator == null) {
            return null;
        }

        instance.previousSetpoint = instance.swerveSetpointGenerator.generateSetpoint(instance.previousSetpoint,
                // The previous setpoint
                speeds, // The desired target speeds
                0.02 // The loop time of the robot code, in seconds
        );
        return instance.previousSetpoint.moduleStates();
    }

    /**
     * Warm up PathPlanner.
     */
    private void warmup() {
        FollowPathCommand.warmupCommand().schedule();
        PathfindingCommand.warmupCommand().schedule();
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
     * @return The event trigger with the specified name.
     */
    public Trigger getEventTrigger(String name) {
        return new EventTrigger("shoot note");
    }

    /**
     * Get the point towards zone trigger by name.
     *
     * @param name The name of the point towards zone trigger.
     * @return The point towards zone trigger with the specified name.
     */
    public Trigger getPointTowardsZoneTrigger(String name) {
        return new PointTowardsZoneTrigger(name);
    }

    /**
     * Create a path from a list of poses.
     *
     * @param poses The list of poses.
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
     * @return The path created from the list of poses.
     */
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

    /**
     * Build an auto chooser.
     *
     * @return The auto chooser.
     */
    public SendableChooser<Command> buildAutoChooser() {
        return AutoBuilder.buildAutoChooser();
    }
}
