package frc.robot.utilities;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.minolib.RobotConfiguration;
import frc.minolib.localization.MinoRobotOdometry;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;

/**
 * This singleton class models the field as a collection of regions. This class is used to create a
 * path from a starting pose in one region to an ending pose in another region that passes through
 * the transition points defined for those regions.
 *
 * <p>The coordinate system of the field is oriented such that the origin is in the lower left
 * corner when the blue alliance is to the left (i.e., to the blue alliance driver's right).
 */
public class Field2d {
  private static Field2d instance = null;

  private Region2d[] regions;
  private Region2d reefZone;

  private Alliance alliance = DriverStation.Alliance.Blue;

  private Map<Pose2d, Pose2d> leftReefPoses = new HashMap<Pose2d, Pose2d>();
  private Map<Pose2d, Pose2d> rightReefPoses = new HashMap<Pose2d, Pose2d>();
  private Map<Pose2d, Pose2d> removeAlgaePoses = new HashMap<Pose2d, Pose2d>();
  private Pose2d[] allReefCenterFaces = new Pose2d[12];

  private Pose2d[] processors = new Pose2d[2];
  private Pose2d[] coralStations = new Pose2d[4];

  private static final Pose2d CENTER_BARGE_POSE = /* 305 before change */
      new Pose2d(
          new Translation2d(Units.inchesToMeters(301), Units.inchesToMeters(242.855)),
          Rotation2d.fromDegrees(0.0));

  private static final Pose2d RIGHT_BARGE_POSE =
      new Pose2d(
          new Translation2d(Units.inchesToMeters(305), Units.inchesToMeters(206.855)),
          Rotation2d.fromDegrees(0.0));

  private final boolean COMPETITION_FIELD =
      true; // set TRUE if home field calibration or at competition

  private static final double PIPE_FROM_REEF_CENTER_INCHES =
      6.469; // taken from FieldConstants adjustY for reef y offset

  private static final double REMOVE_ALGAE_Y_TRANSFORMATION_INCHES = -5;

  public class AlgaePosition {
    public Pose2d pose;
    public boolean isHigh;

    public AlgaePosition(Pose2d pose, boolean isHigh) {
      this.pose = pose;
      this.isHigh = isHigh;
    }
  }

  /**
   * Get the singleton instance of the Field2d class.
   *
   * @return the singleton instance of the Field2d class
   */
  public static Field2d getInstance() {
    if (instance == null) {
      instance = new Field2d();
    }
    return instance;
  }
  /**
   * Construct a Field2d from an array of regions. These regions should not be overlapping (aside
   * from edges) and any regions with overlapping edges should be neighbors (see
   * Region2d::addNeighbor).
   *
   * @param regions the regions that define the field
   */
  public void setRegions(Region2d[] regions) {
    this.regions = regions;
  }

  public void populateReefZone() {
    // make a region of the reef center faces transformed by 18 inches
    // the reef zone is 14 inches from the reef
    // however, a few inches are added for safety as well as the region measuring to the center of
    // the robot
    Translation2d[] transformedCenterFaces = new Translation2d[6];
    for (int i = 0; i < 6; i++) {
      Pose2d centerFace =
          FieldConstants.Reef.centerFaces[i].transformBy(
              new Transform2d(Units.inchesToMeters(36.0), 0.0, Rotation2d.fromDegrees(0.0)));
      transformedCenterFaces[i] = new Translation2d(centerFace.getX(), centerFace.getY());
    }
    this.reefZone = new Region2d(transformedCenterFaces);
  }

  public void populateStationsAndProcessors() {
    processors[0] = FieldConstants.Processor.centerFace;
    processors[1] = FlippingUtil.flipFieldPose(processors[0]);

    coralStations[0] = FieldConstants.CoralStation.leftCenterFace;
    coralStations[1] = FieldConstants.CoralStation.rightCenterFace;
    coralStations[2] = FlippingUtil.flipFieldPose(coralStations[0]);
    coralStations[3] = FlippingUtil.flipFieldPose(coralStations[1]);
  }

  /**
   * Create a path from a starting pose in one region to an ending pose in another region that
   * passes through the transition points defined for those regions.
   *
   * @param start the starting pose
   * @param end the ending pose
   * @param pathConstants the path constraints (i.e., max velocity, max acceleration)
   * @param subsystem the drivetrain subsystem
   * @return the path from the starting pose to the ending pose; null if no path exists
   */
  public PathPlannerPath makePath(
      Pose2d start, Pose2d end, PathConstraints pathConstants, DrivetrainSubsystem subsystem) {
    Region2d startRegion = null;
    Region2d endRegion = null;

    // find the starting and ending regions
    for (Region2d region : regions) {
      if (region.contains(start)) {
        startRegion = region;
      }
      if (region.contains(end)) {
        endRegion = region;
      }
    }

    // make sure both start and end are on the field
    if (startRegion == null || endRegion == null) return null;

    // BFS to find the shortest path to the end
    List<Region2d> path = breadthFirstSearch(startRegion, endRegion);
    if (path.isEmpty()) return null;

    // create point locations
    ArrayList<Translation2d> pointLocations = new ArrayList<>();

    // add the starting point
    pointLocations.add(start.getTranslation());

    // add all the transition points
    for (int i = 0; i < path.size() - 1; i++) {
      Region2d from = path.get(i);
      Region2d to = path.get(i + 1);
      pointLocations.add(from.getTransitionPoint(to));
    }

    // add a transition point if starting region & ending region same
    if (startRegion == endRegion) {
      pointLocations.add(
          new Translation2d((start.getX() + end.getX()) / 2, (start.getY() + end.getY()) / 2));
    }

    // add the ending point
    pointLocations.add(end.getTranslation());

    List<Pose2d> pathPoses = createPathPoses(pointLocations);
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(pathPoses);
    return new PathPlannerPath(
        waypoints,
        pathConstants,
        null,
        new GoalEndState(
            RobotConfiguration.getInstance().getMoveToPathFinalVelocity(), end.getRotation()));
  }

  /**
   * Create the path points based on the starting and ending poses and the point locations. The path
   * will be created such that the first path point matches the robot's current heading and velocity
   * to ensure a smooth transition to the path. The the starting and ending poses have different
   * rotations, the change in rotation will occur between the first and second points. The final
   * speed of the robot will be as specified by the robot's configuration class'
   * getMoveToPathFinalVelocity method.
   *
   * @param start the starting pose
   * @param end the ending pose
   * @param subsystem the drivetrain subsystem
   * @param pointLocations the locations of the points in the path
   * @return the path points
   */
  private List<Pose2d> createPathPoses(ArrayList<Translation2d> pointLocations) {
    List<Pose2d> pathPoses = new ArrayList<>();
    Rotation2d lastHeading = null;
    for (int i = 0; i < pointLocations.size() - 1; i++) {
      double deltaX = pointLocations.get(i + 1).getX() - pointLocations.get(i).getX();
      double deltaY = pointLocations.get(i + 1).getY() - pointLocations.get(i).getY();
      lastHeading = new Rotation2d(deltaX, deltaY);
      pathPoses.add(
          new Pose2d(pointLocations.get(i).getX(), pointLocations.get(i).getY(), lastHeading));
    }

    // the final path point will match the ending pose's rotation and the velocity as specified by
    // the robot's configuration class' getMoveToPathFinalVelocity method.
    pathPoses.add(
        new Pose2d(
            pointLocations.get(pointLocations.size() - 1).getX(),
            pointLocations.get(pointLocations.size() - 1).getY(),
            lastHeading));

    return pathPoses;
  }

  private List<Region2d> breadthFirstSearch(Region2d start, Region2d end) {
    Queue<ArrayList<Region2d>> todo = new LinkedList<>();
    Set<Region2d> explored = new HashSet<>();

    // add the starting region to the set of explored regions
    explored.add(start);

    // if the path starts and ends in the same region, return that region
    if (start == end) {
      return new ArrayList<>(Arrays.asList(start));
    }

    todo.add(
        new ArrayList<>(Arrays.asList(start))); // add a path starting with startRegion to the list

    while (!todo.isEmpty()) { // while the list isn't empty, keep looking over the list.
      ArrayList<Region2d> path = todo.poll();
      Region2d region = path.get(path.size() - 1); // last region in the path

      for (Region2d other : region.getNeighbors()) {
        if (!explored.contains(other)) {
          ArrayList<Region2d> newPath = new ArrayList<>(path);
          newPath.add(other);

          if (other == end) {
            return newPath;
          }

          explored.add(other);
          todo.add(newPath);
        }
      }
    }
    return new ArrayList<>();
  }

  /**
   * This method should be invoked once the alliance color is known. Refer to the RobotContainer's
   * checkAllianceColor method for best practices on when to check the alliance's color. The
   * alliance color is needed when running auto paths as those paths are always defined for
   * blue-alliance robots and need to be flipped for red-alliance robots.
   *
   * @param newAlliance the new alliance color
   */
  public void updateAlliance(DriverStation.Alliance newAlliance) {
    this.alliance = newAlliance;
  }

  /**
   * Get the alliance color.
   *
   * @return the alliance color
   */
  public Alliance getAlliance() {
    return alliance;
  }

  public void populateReefBranchPoseMaps() {
    if (COMPETITION_FIELD) {
      Pose2d[] blueReefRightBranches = populateBlueReefRightBranches();
      Pose2d[] blueReefLeftBranches = populateBlueReefLeftBranches();
      Pose2d[] redReefRightBranches = populateRedReefRightBranches();
      Pose2d[] redReefLeftBranches = populateRedReefLeftBranches();

      Pose2d[] blueCenterFaces = FieldConstants.Reef.centerFaces;
      for (int i = 0; i < 6; i++) {
        allReefCenterFaces[i] = blueCenterFaces[i];
        allReefCenterFaces[i + 6] = FlippingUtil.flipFieldPose(blueCenterFaces[i]);
      }

      // remove algae poses are hardcoded to slightly off the center
      // blue alliance poses
      for (int i = 0; i < 6; i++) {
        rightReefPoses.put(allReefCenterFaces[i], blueReefRightBranches[i]);
        leftReefPoses.put(allReefCenterFaces[i], blueReefLeftBranches[i]);
        Pose2d removeAlgaePose =
            allReefCenterFaces[i].transformBy(
                new Transform2d(
                    RobotConfiguration.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0,
                    Units.inchesToMeters(REMOVE_ALGAE_Y_TRANSFORMATION_INCHES),
                    Rotation2d.fromDegrees(180)));
        removeAlgaePoses.put(allReefCenterFaces[i], removeAlgaePose);
      }

      // red alliance poses
      for (int i = 0; i < 6; i++) {
        rightReefPoses.put(allReefCenterFaces[i + 6], redReefRightBranches[i]);
        leftReefPoses.put(allReefCenterFaces[i + 6], redReefLeftBranches[i]);
        Pose2d removeAlgaePose =
            allReefCenterFaces[i + 6].transformBy(
                new Transform2d(
                    RobotConfiguration.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0,
                    Units.inchesToMeters(REMOVE_ALGAE_Y_TRANSFORMATION_INCHES),
                    Rotation2d.fromDegrees(180)));
        removeAlgaePoses.put(allReefCenterFaces[i + 6], removeAlgaePose);
      }

    } else {
      // Populate pose maps with default, non-field calibrated values (for if we ever have an
      // inaccurate field and don't have time to calibrate)

      // get each transformed pose on the reef (center of the hexagonal side)
      // add left or right offset (y) as well as bumper offset (x)
      Pose2d[] reefCenterFaces = FieldConstants.Reef.centerFaces;
      for (Pose2d reefCenterFace : reefCenterFaces) {
        Pose2d leftPose =
            reefCenterFace.transformBy(
                new Transform2d(
                    RobotConfiguration.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0,
                    -Units.inchesToMeters(PIPE_FROM_REEF_CENTER_INCHES),
                    Rotation2d.fromDegrees(180)));
        Pose2d rightPose =
            reefCenterFace.transformBy(
                new Transform2d(
                    RobotConfiguration.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0,
                    Units.inchesToMeters(PIPE_FROM_REEF_CENTER_INCHES),
                    Rotation2d.fromDegrees(180)));
        Pose2d removeAlgaePose =
            reefCenterFace.transformBy(
                new Transform2d(
                    RobotConfiguration.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0,
                    -Units.inchesToMeters(REMOVE_ALGAE_Y_TRANSFORMATION_INCHES),
                    Rotation2d.fromDegrees(180)));

        leftReefPoses.put(reefCenterFace, leftPose);
        rightReefPoses.put(reefCenterFace, rightPose);
        removeAlgaePoses.put(reefCenterFace, removeAlgaePose);
      }
    }
  }

  @SuppressWarnings("unused")
  public Pose2d getNearestBranch(Side side) {
    Pose2d pose = MinoRobotOdometry.getInstance().getEstimatedPose();

    Pose2d nearestReefCenterFace;
    if (COMPETITION_FIELD) {
      nearestReefCenterFace = pose.nearest(Arrays.asList(allReefCenterFaces));
    } else {
      // If we are on the red alliance, flip the current pose to the blue alliance to find the
      // nearest reef face. We will then flip back to the red alliance.
      // ONLY IF NOT USING COMPETITION FIELD
      if (getAlliance() == Alliance.Red) {
        pose = FlippingUtil.flipFieldPose(pose);
      }
      nearestReefCenterFace = pose.nearest(Arrays.asList(FieldConstants.Reef.centerFaces));
    }

    Pose2d bumpersOnReefAlignedToBranch;
    if (side == Side.LEFT) {
      bumpersOnReefAlignedToBranch = leftReefPoses.get(nearestReefCenterFace);
    } else if (side == Side.RIGHT) {
      bumpersOnReefAlignedToBranch = rightReefPoses.get(nearestReefCenterFace);
    } else {
      bumpersOnReefAlignedToBranch = removeAlgaePoses.get(nearestReefCenterFace);
    }

    // If we are on the red alliance, we have flipped the current pose to the blue alliance and
    // have found the nearest reef face on the blue alliance side. We now need to flip the pose for
    // that reef face back to the red alliance.

    if (!COMPETITION_FIELD && getAlliance() == Alliance.Red) {
      bumpersOnReefAlignedToBranch = FlippingUtil.flipFieldPose(bumpersOnReefAlignedToBranch);
    }

    return bumpersOnReefAlignedToBranch;
  }

  public Pose2d getSelectedBranch() {
    int offset = 0;
    if (getAlliance() == Alliance.Red) {
      offset = 6;
    }

    /* 
    if (OISelector.getOperatorInterface().getReefBranchATrigger().getAsBoolean()) {
      return leftReefPoses.get(allReefCenterFaces[offset]);
    } else if (OISelector.getOperatorInterface().getReefBranchBTrigger().getAsBoolean()) {
      return rightReefPoses.get(allReefCenterFaces[offset]);
    } else if (OISelector.getOperatorInterface().getReefBranchCTrigger().getAsBoolean()) {
      return leftReefPoses.get(allReefCenterFaces[offset + 5]);
    } else if (OISelector.getOperatorInterface().getReefBranchDTrigger().getAsBoolean()) {
      return rightReefPoses.get(allReefCenterFaces[offset + 5]);
    } else if (OISelector.getOperatorInterface().getReefBranchETrigger().getAsBoolean()) {
      return leftReefPoses.get(allReefCenterFaces[offset + 4]);
    } else if (OISelector.getOperatorInterface().getReefBranchFTrigger().getAsBoolean()) {
      return rightReefPoses.get(allReefCenterFaces[offset + 4]);
    } else if (OISelector.getOperatorInterface().getReefBranchGTrigger().getAsBoolean()) {
      return leftReefPoses.get(allReefCenterFaces[offset + 3]);
    } else if (OISelector.getOperatorInterface().getReefBranchHTrigger().getAsBoolean()) {
      return rightReefPoses.get(allReefCenterFaces[offset + 3]);
    } else if (OISelector.getOperatorInterface().getReefBranchITrigger().getAsBoolean()) {
      return leftReefPoses.get(allReefCenterFaces[offset + 2]);
    } else if (OISelector.getOperatorInterface().getReefBranchJTrigger().getAsBoolean()) {
      return rightReefPoses.get(allReefCenterFaces[offset + 2]);
    } else if (OISelector.getOperatorInterface().getReefBranchKTrigger().getAsBoolean()) {
      return leftReefPoses.get(allReefCenterFaces[offset + 1]);
    } else if (OISelector.getOperatorInterface().getReefBranchLTrigger().getAsBoolean()) {
      return rightReefPoses.get(allReefCenterFaces[offset + 1]);
    }
      */

    // go to the nearest center face if nothing is selected
    Pose2d pose = MinoRobotOdometry.getInstance().getEstimatedPose();
    return pose.nearest(Arrays.asList(FieldConstants.Reef.centerFaces));
  }

  public AlgaePosition getNearestAlgae() {
    Pose2d pose = MinoRobotOdometry.getInstance().getEstimatedPose();
    boolean isHighAlgae = false;

    // high: A/B , E/F, I/J
    // low: C/D, G/H, K/L
    Pose2d nearestCenterFace = pose.nearest(Arrays.asList(allReefCenterFaces));
    for (int i = 0; i < allReefCenterFaces.length; i++) {
      if (nearestCenterFace == allReefCenterFaces[i]) {
        if (i % 2 == 0) {
          isHighAlgae = true;
        } else {
          isHighAlgae = false;
        }
        break;
      }
    }

    return new AlgaePosition(removeAlgaePoses.get(nearestCenterFace), isHighAlgae);
  }

  public Pose2d getCenterBargePose() {
    // x arbitrary from 20 inches x from the middle cage

    if (getAlliance() == Alliance.Red) {
      return FlippingUtil.flipFieldPose(CENTER_BARGE_POSE);
    }
    return CENTER_BARGE_POSE;
  }

  public Pose2d getShortOfBargePose() {
    Pose2d shortOfBargePose =
        CENTER_BARGE_POSE.transformBy(
            new Transform2d(-Units.inchesToMeters(18), 0.0, Rotation2d.fromDegrees(0.0)));

    if (getAlliance() == Alliance.Red) {
      return FlippingUtil.flipFieldPose(shortOfBargePose);
    }
    return shortOfBargePose;
  }

  public Pose2d getRightBargePose() {
    if (getAlliance() == Alliance.Red) {
      return FlippingUtil.flipFieldPose(RIGHT_BARGE_POSE);
    }
    return RIGHT_BARGE_POSE;
  }

  public boolean isShortOfBarge() {
    Pose2d pose = MinoRobotOdometry.getInstance().getEstimatedPose();
    Transform2d robotRelativeDifference = new Transform2d(pose, getCenterBargePose());

    // 18 inch buffer, lower if we need to
    return robotRelativeDifference.getX() > Units.inchesToMeters(18);
  }

  public Pose2d getFourthAutoCoralPose(Side side, boolean closeAuto) {
    int offset = 0;
    if (getAlliance() == Alliance.Red) {
      offset = 6;
    }

    if (side == Side.LEFT) {
      if (closeAuto) {
        return leftReefPoses.get(allReefCenterFaces[offset]);
      }
      return leftReefPoses.get(allReefCenterFaces[offset + 4]);
    } else {
      if (closeAuto) {
        return rightReefPoses.get(allReefCenterFaces[offset]);
      }
      return rightReefPoses.get(allReefCenterFaces[offset + 2]);
    }
  }

  public Pose2d getNearestProcessor() {
    Pose2d pose = MinoRobotOdometry.getInstance().getEstimatedPose();

    Pose2d nearestProcessor = pose.nearest(Arrays.asList(processors));
    nearestProcessor =
        nearestProcessor.transformBy(
            new Transform2d(
                (RobotConfiguration.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0)
                    + Units.inchesToMeters(12.0),
                Units.inchesToMeters(-4.5),
                Rotation2d.fromDegrees(180)));

    return nearestProcessor;
  }

  public Pose2d getNearestCoralStation() {
    Pose2d pose = MinoRobotOdometry.getInstance().getEstimatedPose();

    Pose2d nearestCoralStation = pose.nearest(Arrays.asList(coralStations));
    nearestCoralStation =
        nearestCoralStation.transformBy(
            new Transform2d(
                (RobotConfiguration.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0),
                0,
                Rotation2d.fromDegrees(0)));

    return nearestCoralStation;
  }

  public Pose2d getLeftCoralStation() {
    int offset = getAlliance() == Alliance.Red ? 2 : 0;

    return coralStations[offset].transformBy(
        new Transform2d(
            (RobotConfiguration.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0),
            0,
            Rotation2d.fromDegrees(0)));
  }

  public Pose2d getRightCoralStation() {
    int offset = getAlliance() == Alliance.Red ? 3 : 1;

    return coralStations[offset].transformBy(
        new Transform2d(
            (RobotConfiguration.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0),
            0,
            Rotation2d.fromDegrees(0)));
  }

  private Pose2d[] populateBlueReefRightBranches() {
    Pose2d[] blueReefRightBranches = new Pose2d[6];
    // ORDER (clockwise): B, L, J, H, F, D
    // B
    blueReefRightBranches[0] =
        new Pose2d(
            3.2268113059503327, 3.8710725939005064, Rotation2d.fromDegrees(-0.5647140062538756));

    // L
    blueReefRightBranches[1] =
        new Pose2d(
            3.7250066199792693, 5.036937290009448, Rotation2d.fromDegrees(-59.867890464426786));

    // J
    blueReefRightBranches[2] =
        new Pose2d(
            4.986117936804482, 5.191363292750105, Rotation2d.fromDegrees(-119.47192699245008));

    // H
    blueReefRightBranches[3] =
        new Pose2d(
            5.750604222554525, 4.175476939019482, Rotation2d.fromDegrees(179.51883588381267));

    // F
    blueReefRightBranches[4] =
        new Pose2d(
            5.262941075891613, 3.0196657160072755, Rotation2d.fromDegrees(120.13242909453064));

    // D
    blueReefRightBranches[5] =
        new Pose2d(4.004433644449748, 2.854353534498152, Rotation2d.fromDegrees(60.67294023039258));

    return blueReefRightBranches;
  }

  private Pose2d[] populateBlueReefLeftBranches() {
    Pose2d[] blueReefLeftBranches = new Pose2d[6];
    // ORDER (clockwise): A, K, I, G, E, C
    // A
    blueReefLeftBranches[0] =
        new Pose2d(
            3.225788742587686, 4.20966744071561, Rotation2d.fromDegrees(0.06562032347683353));

    // K
    blueReefLeftBranches[1] =
        new Pose2d(
            4.008399244704131, 5.203384373823917, Rotation2d.fromDegrees(-59.64662097083009));

    // I
    blueReefLeftBranches[2] =
        new Pose2d(
            5.270315547789745, 5.02883588283863, Rotation2d.fromDegrees(-120.26141835740316));

    // G
    blueReefLeftBranches[3] =
        new Pose2d(5.753868030308526, 3.851059316356047, Rotation2d.fromDegrees(179.5379014932943));

    // E
    blueReefLeftBranches[4] =
        new Pose2d(
            4.96530190111186, 2.8455194340152175, Rotation2d.fromDegrees(120.31359073904765));

    // C
    blueReefLeftBranches[5] =
        new Pose2d(
            3.7012917262920566, 3.0284777563392193, Rotation2d.fromDegrees(60.481819496958735));

    return blueReefLeftBranches;
  }

  private Pose2d[] populateRedReefRightBranches() {
    Pose2d[] redReefRightBranches = new Pose2d[6];
    // ORDER (clockwise): B, L, J, H, F, D
    // B
    redReefRightBranches[0] =
        new Pose2d(
            14.317189968112993, 4.1960610943337535, Rotation2d.fromDegrees(-179.8873082201491));

    // L
    redReefRightBranches[1] =
        new Pose2d(
            13.834336045494839, 3.022140826015172, Rotation2d.fromDegrees(120.43301602882423));

    // J
    redReefRightBranches[2] =
        new Pose2d(
            12.57604023143753, 2.8522190791703568, Rotation2d.fromDegrees(59.787217664785835));

    // H
    redReefRightBranches[3] =
        new Pose2d(
            11.800576072872003, 3.857492215657563, Rotation2d.fromDegrees(0.07882516914361322));

    // F
    redReefRightBranches[4] =
        new Pose2d(
            12.293759499227246, 5.03675742757713, Rotation2d.fromDegrees(-60.217381285620206));

    // D
    redReefRightBranches[5] =
        new Pose2d(
            13.536360178294643, 5.202277095044223, Rotation2d.fromDegrees(-120.30443762827741));

    return redReefRightBranches;
  }

  private Pose2d[] populateRedReefLeftBranches() {

    Pose2d[] redReefLeftBranches = new Pose2d[6];
    // ORDER (clockwise): A, K, I, G, E, C
    // A
    redReefLeftBranches[0] =
        new Pose2d(
            14.317969558245363, 3.859702793398233, Rotation2d.fromDegrees(-179.73294016349274));

    // K
    redReefLeftBranches[1] =
        new Pose2d(
            13.540011899310727, 2.8473574691177603, Rotation2d.fromDegrees(120.53291312169985));

    // I
    redReefLeftBranches[2] =
        new Pose2d(
            12.279914615986707, 3.0206488727306815, Rotation2d.fromDegrees(59.681339810333725));

    // G
    redReefLeftBranches[3] =
        new Pose2d(
            11.797173126841262, 4.212233478940297, Rotation2d.fromDegrees(-0.15273925714418848));

    // E
    redReefLeftBranches[4] =
        new Pose2d(
            12.579056341872844, 5.206389901624839, Rotation2d.fromDegrees(-60.01374977353829));

    // C
    redReefLeftBranches[5] =
        new Pose2d(
            13.841014810144816, 5.0286481010468735, Rotation2d.fromDegrees(-119.71798083715267));

    return redReefLeftBranches;
  }

  public boolean isOutsideOfReefZone() {
    Pose2d pose = MinoRobotOdometry.getInstance().getEstimatedPose();
    if (getAlliance() == Alliance.Red) {
      pose = FlippingUtil.flipFieldPose(pose);
    }

    return !reefZone.contains(pose);
  }

  public enum Side {
    LEFT,
    RIGHT,
    REMOVE_ALGAE
  }
}