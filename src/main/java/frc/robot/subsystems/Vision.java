package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Vision extends SubsystemBase {
    private final PhotonCamera shooterCamera = new PhotonCamera("Shooter");
    private final PhotonCamera backLeftCamera = new PhotonCamera("BackLeft");
    private final PhotonCamera frontLeftCamera = new PhotonCamera("FrontLeft");
    private final PhotonCamera backRightCamera = new PhotonCamera("BackRight");
    private final PhotonCamera frontRightCamera = new PhotonCamera("FrontRight");
    private final PhotonCamera frontCamera = new PhotonCamera("Front");
    
    private Field2d Field = new Field2d();

    private SendableChooser<String> CameraSelect = new SendableChooser<>();

    private Pose3d frontPose = new Pose3d();
    private Pose3d frontLeftPose = new Pose3d();
    private Pose3d frontRightPose = new Pose3d();
    private Pose3d backLeftPose = new Pose3d();
    private Pose3d backRightPose = new Pose3d();

    private boolean FrontPoseToggle = true;
    private boolean FrontRightPoseToggle = false;
    private boolean FrontLeftPoseToggle = false;
    private boolean BackRightPoseToggle = false;
    private boolean BackLeftPoseToggle = false;

    private double RobotDistance = 0.0;


    private final AprilTagFieldLayout fieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private final Transform3d backLeftRobotToCamera = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(8.0525),
            Units.inchesToMeters(6.6524),
            Units.inchesToMeters(14.5819)
        ),
        new Rotation3d(
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(-30.0),
            Units.degreesToRadians(155.0)
        ));
    private final Transform3d frontLeftRobotToCamera = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(10.3128),
            Units.inchesToMeters(6.41808),
            Units.inchesToMeters(14.5819)
        ),
        new Rotation3d(
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(-30.0),
            Units.degreesToRadians(75.0)
        ));
    private final Transform3d backRightRobotToCamera = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(8.398),
            Units.inchesToMeters(-12.9322),
            Units.inchesToMeters(14.58198)
        ),
        new Rotation3d(
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(-30.0),
            Units.degreesToRadians(-135.0)
        ));
    private final Transform3d frontRightRobotToCamera = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(11.9471),
            Units.inchesToMeters(-11.4345),
            Units.inchesToMeters(14.5819)
        ),
        new Rotation3d(
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(-30.0),
            Units.degreesToRadians(-60.0)
        ));
    private final Transform3d frontRobotToCamera = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(12.5286),
            Units.inchesToMeters(3.3754),
            Units.inchesToMeters(14.5819)
        ),
        new Rotation3d(
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(-30.0),
            Units.degreesToRadians(-5.0)
        ));

    private final PhotonPoseEstimator backRightPoseEstimator;
    private final PhotonPoseEstimator backLeftPoseEstimator;
    private final PhotonPoseEstimator frontLeftPoseEstimator;
    private final PhotonPoseEstimator frontRightPoseEstimator;
    private final PhotonPoseEstimator frontPoseEstimator;

   

    public Consumer<PoseEstimate> PassedDistance;

    public Vision(Consumer<PoseEstimate> DistanceConsumer) {
        frontLeftPoseEstimator = new PhotonPoseEstimator(fieldLayout,
             frontLeftRobotToCamera);
        frontRightPoseEstimator = new PhotonPoseEstimator(fieldLayout,
             frontRightRobotToCamera);
        backLeftPoseEstimator = new PhotonPoseEstimator(fieldLayout,
             backLeftRobotToCamera);
        backRightPoseEstimator = new PhotonPoseEstimator(fieldLayout,
             backRightRobotToCamera);
        frontPoseEstimator = new PhotonPoseEstimator(fieldLayout,
             frontRobotToCamera);

        SmartDashboard.putBoolean("FrontPoseToggle", FrontPoseToggle);
        SmartDashboard.putBoolean("BackLeftPoseToggle", BackLeftPoseToggle);
        SmartDashboard.putBoolean("BackRightPoseToggle", BackRightPoseToggle);
        SmartDashboard.putBoolean("FrontLeftPoseToggle", FrontLeftPoseToggle);
        SmartDashboard.putBoolean("FrontRightPoseToggle", FrontRightPoseToggle);

        CameraSelect.setDefaultOption("FrontPose", "FrontPose");
        CameraSelect.addOption("FrontRightPose", "FrontRightPose");
        CameraSelect.addOption("FrontLeftPose", "FrontLeftPose");
        CameraSelect.addOption("BackRightPose", "BackRightPose");
        CameraSelect.addOption("BackLeftPose", "BackLeftPose");

        SmartDashboard.putData(CameraSelect);
        SmartDashboard.putData(Field);

        PassedDistance = DistanceConsumer;
    }

    public void periodic(){
        getEstimatedPose();
    }

    public void getEstimatedPose() {
        PoseEstimate frontEstimate = getEstimatedRobotPoseForCamera(frontCamera, frontPoseEstimator);
        PoseEstimate frontRightEstimate = getEstimatedRobotPoseForCamera(frontRightCamera, frontRightPoseEstimator);
        //PoseEstimate frontLeftEstimate = getEstimatedRobotPoseForCamera(frontLeftCamera, frontLeftPoseEstimator);
        //PoseEstimate backLeftEstimate = getEstimatedRobotPoseForCamera(backLeftCamera, backLeftPoseEstimator);
        //PoseEstimate backRightEstimate = getEstimatedRobotPoseForCamera(backRightCamera, backRightPoseEstimator);

        if (frontEstimate !=null) {
            frontPose = frontEstimate.estimatedRobotPose.estimatedPose;
            if (CameraSelect.getSelected().equals("FrontPose")) {
                Field.setRobotPose(frontPose.toPose2d());
            }
            PassedDistance.accept(frontEstimate);
        }
        /*if (frontLeftEstimate !=null) {
            frontLeftPose = frontLeftEstimate.estimatedRobotPose.estimatedPose;
            if (CameraSelect.getSelected().equals("FrontLeftPose")) {
                Field.setRobotPose(frontLeftPose.toPose2d());
                RobotDistance = Units.metersToInches(blueGoal.getDistance(frontLeftPose.getTranslation().toTranslation2d()));
                PassedDistance.accept(RobotDistance);
            }
        }*/
        if (frontRightEstimate !=null) {
            frontRightPose = frontRightEstimate.estimatedRobotPose.estimatedPose;
            if (CameraSelect.getSelected().equals("FrontRightPose")) {
                Field.setRobotPose(frontRightPose.toPose2d());
            }
            PassedDistance.accept(frontRightEstimate);
        }
        /*if (backLeftEstimate !=null) {
            backLeftPose = backLeftEstimate.estimatedRobotPose.estimatedPose;
            if (CameraSelect.getSelected().equals("BackLeftPose")) {
                Field.setRobotPose(backLeftPose.toPose2d());
                RobotDistance = Units.metersToInches(blueGoal.getDistance(backLeftPose.getTranslation().toTranslation2d()));
                PassedDistance.accept(RobotDistance);
            }
        }
        if (backRightEstimate !=null) {
            backRightPose = backRightEstimate.estimatedRobotPose.estimatedPose;
            if (CameraSelect.getSelected().equals("BackRightPose")) {
                Field.setRobotPose(backRightPose.toPose2d());
                RobotDistance = Units.metersToInches(blueGoal.getDistance(backRightPose.getTranslation().toTranslation2d()));
                PassedDistance.accept(RobotDistance);
            }
        }*/

    }

    private PoseEstimate getEstimatedRobotPoseForCamera(PhotonCamera camera, PhotonPoseEstimator poseEstimator) {
        PoseEstimate currentPoseEstimate = null;
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            if (readingIsValid(result.getTargets())) {
                Optional<EstimatedRobotPose> estimatedPose = poseEstimator.estimateCoprocMultiTagPose(result);
                if (estimatedPose.isPresent()) {
                    EstimatedRobotPose possiblePose = estimatedPose.get();
                    if (poseIsValid(possiblePose)) {
                        currentPoseEstimate = new PoseEstimate(possiblePose, calculateStdDev(possiblePose, result.getTargets()));
                    }
                }
            }
        }

        return currentPoseEstimate;
    }

    private boolean readingIsValid(List<PhotonTrackedTarget> targets) {
        if (targets.isEmpty()) {
            return false;
        }

        for (PhotonTrackedTarget target : targets) {
            if (target.getPoseAmbiguity() > 0.2) {
                return false;
            }
        }

        return true;
    }

    private boolean poseIsValid(EstimatedRobotPose pose) {
        return pose.estimatedPose.getZ() < 0.75 &&
            pose.estimatedPose.getX() > 0.0 &&
            pose.estimatedPose.getX() < fieldLayout.getFieldLength() &&
            pose.estimatedPose.getY() > 0.0 &&
            pose.estimatedPose.getY() < fieldLayout.getFieldWidth();
    }

    private Matrix<N3, N1> calculateStdDev(EstimatedRobotPose pose, List<PhotonTrackedTarget> targets) {
        if (targets.size() == 1) {
            Optional<Pose3d> tagPose = fieldLayout.getTagPose(targets.get(0).getFiducialId());
            if (tagPose.isPresent()) {
                double distance = tagPose.get().toPose2d().getTranslation()
                    .getDistance(pose.estimatedPose.toPose2d().getTranslation());
                return VecBuilder.fill(0.5, 0.5, 1.0).times(Math.pow(distance, 2));
            } else {
                return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            }
        } else {
            double averageDistance = 0.0;
            double numTags = 0.0;
            for (PhotonTrackedTarget target : targets) {
                Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
                if (tagPose.isPresent()) {
                    averageDistance += tagPose.get().toPose2d().getTranslation()
                        .getDistance(pose.estimatedPose.toPose2d().getTranslation());
                    numTags++;
                }
            }

            if (numTags != 0) {
                averageDistance /= numTags;
                return VecBuilder.fill(0.1, 0.1, 0.2).times(averageDistance);
            } else {
                return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            }
        }
    }

    public Pose3d getTagPose(int id) {
        return fieldLayout.getTagPose(id).get();
    }

    public class PoseEstimate {
        private EstimatedRobotPose estimatedRobotPose;
        private Matrix<N3, N1> estimatedStdDev;

        public PoseEstimate(EstimatedRobotPose pose, Matrix<N3, N1> stdDev) {
            estimatedRobotPose = pose;
            estimatedStdDev = stdDev;
        }

        public EstimatedRobotPose getEstimatedRobotPose() {
            return estimatedRobotPose;
        }

        public Matrix<N3, N1> getEstimatedStdDev() {
            return estimatedStdDev;
        }


    }
}
