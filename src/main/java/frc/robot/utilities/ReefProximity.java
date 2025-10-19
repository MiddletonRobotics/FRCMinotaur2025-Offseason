package frc.robot.utilities;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * This class is used to determine the closest reef scoring position to the robot's current position.
 */
public class ReefProximity {

    final List<Entry<Integer, Pose2d>> reefList;

    public static final List<Integer> redReefTags = List.of(6,7,8,9,10,11);
    public static final List<Integer> blueReefTags = List.of(17, 18, 19, 20, 21, 22);
    
    public static HashMap<Integer, Pose2d> m_leftReefHashMap;
    public static HashMap<Integer, Pose2d> m_rightReefHashMap;

    public ReefProximity(HashMap<Integer, Pose2d> leftReefHashMap, HashMap<Integer, Pose2d> rightReefHashMap) {
        reefList = new ArrayList<Entry<Integer, Pose2d>>();
        leftReefHashMap.entrySet().forEach(reefList::add);
        rightReefHashMap.entrySet().forEach(reefList::add);

        m_leftReefHashMap = leftReefHashMap;
        m_rightReefHashMap = rightReefHashMap;
    }

    public Entry<Integer, Pose2d> closestReefPose(Pose2d position, boolean blueAlliance) {
        return closestReefPose(position, blueAlliance ? blueReefTags : redReefTags);
    }

    public Entry<Integer, Pose2d> closestReefPose(Pose2d position, List<Integer> tagOptions) {
        double currentDistance = Double.MAX_VALUE;
        Entry<Integer, Pose2d> closest = null;

        for (var entry : reefList) {
            // Ensure the reef tag is in the list of tags we are looking for
            if (!tagOptions.contains(entry.getKey())) {
                continue;
            }

            double distance = position.getTranslation().getDistance(entry.getValue().getTranslation());
            if (distance < currentDistance) {
                currentDistance = distance;
                closest = entry;
            }
        }

        return closest;
    }

    public Pose2d getReefPose(int tagID, boolean left) {
        if (left) {
            return m_leftReefHashMap.get(tagID);
        } else {
            return m_rightReefHashMap.get(tagID);
        }
    }
}