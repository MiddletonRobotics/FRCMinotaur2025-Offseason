package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GlobalConstants;

import org.littletonrobotics.junction.Logger;

public class LedSubsystem extends SubsystemBase {
    private enum AnimationType {
        STROBE,
        FADE,
        FIRE,
        LARSON,
        TWINKLE,
        RAINBOW
    }

    private static final int NUMBER_OF_LEDS = 76;

    public enum WantedState {
        DISPLAY_OFF,
        DISPLAY_ROBOT_ARM_NOT_ZEROED,
        DISPLAY_POSE_RESET,
        DISPLAY_ROBOT_ARM_ZEROED,
        DISPLAY_ROBOT_IS_PHYSICALLY_READY_FOR_MATCH,
        DISPLAY_POSE_BASED,
        DISPLAY_ROTATION_BASED,
        DISPLAY_NOT_AT_KICKSTAND_POSITION,
        DISPLAY_ROBOT_ZERO_ACTION,
        DISPLAY_READY_FOR_MATCH,
        DISPLAY_INTAKING_ALGAE,
        DISPLAY_HOLDING_ALGAE,
        DISPLAY_ALIGN_TO_PROCESSOR,
        DISPLAY_SCORE_PROCESSOR,
        DISPLAY_ALIGN_TO_BARGE,
        DISPLAY_SCORE_BARGE,
        DISPLAY_INTAKING_CORAL,
        DISPLAY_INTAKING_CORAL_HORIZONTAL,
        DISPLAY_HOLDING_CORAL_POSE_BASED,
        DISPLAY_HOLDING_CORAL_ROTATION_BASED,
        TAG_NOT_SEEN,
        DISPLAY_ALIGN_TO_TARGET_L_ONE,
        DISPLAY_ALIGN_TO_TARGET_L_TWO,
        DISPLAY_ALIGN_TO_TARGET_L_THREE,
        DISPLAY_ALIGN_TO_TARGET_L_FOUR,
        DISPLAY_CAN_SCORE_L_ONE,
        DISPLAY_CAN_SCORE_L_TWO,
        DISPLAY_CAN_SCORE_L_THREE,
        DISPLAY_CAN_SCORE_L_FOUR,
        DISPLAY_CLIMB_ALIGN,
        DISPLAY_CLIMB_AUTOMATED,
        DISPLAY_CONTROLLERS_ACTIVE
    }

    private enum SystemState {
        DISPLAYING_OFF,
        DISPLAYING_ROBOT_ARM_NOT_ZEROED,
        DISPLAYING_POSE_RESETTING,
        DISPLAYING_ROBOT_IS_PHYSICALLY_READY_FOR_MATCH,
        DISPLAYING_ROBOT_ARM_ZEROED,
        DISPLAYING_POSE_BASED,
        DISPLAYING_ROTATION_BASED,
        DISPLAYING_NOT_AT_KICKSTAND_POSITION,
        DISPLAYING_ROBOT_ZERO_ACTION,
        DISPLAYING_READY_FOR_MATCH,
        DISPLAYING_INTAKING_ALGAE,
        DISPLAYING_HOLDING_ALGAE,
        DISPLAYING_ALIGN_TO_PROCESSOR,
        DISPLAYING_SCORE_PROCESSOR,
        DISPLAYING_ALIGN_TO_BARGE,
        DISPLAYING_SCORE_BARGE,
        DISPLAYING_INTAKING_CORAL,
        DISPLAYING_INTAKING_CORAL_HORIZONTAL,
        DISPLAYING_HOLDING_CORAL_POSE_BASED,
        DISPLAYING_HOLDING_CORAL_ROTATION_BASED,
        TAG_NOT_SEEN,
        DISPLAYING_ALIGN_TO_TARGET_L_ONE,
        DISPLAYING_ALIGN_TO_TARGET_L_TWO,
        DISPLAYING_ALIGN_TO_TARGET_L_THREE,
        DISPLAYING_ALIGN_TO_TARGET_L_FOUR,
        DISPLAYING_CAN_SCORE_L_ONE,
        DISPLAYING_CAN_SCORE_L_TWO,
        DISPLAYING_CAN_SCORE_L_THREE,
        DISPLAYING_CAN_SCORE_L_FOUR,
        DISPLAYING_CLIMB_ALIGN,
        DISPLAYING_CLIMB_AUTOMATED,
        DISPLAYING_CONTROLLER_ACTIVE
    }

    private WantedState wantedAction = WantedState.DISPLAY_OFF;
    private final LedIO ledIO;

    private SystemState getStateTransition() {
        return switch (wantedAction) {
            case DISPLAY_ROBOT_ARM_ZEROED -> SystemState.DISPLAYING_ROBOT_ARM_ZEROED;
            case DISPLAY_POSE_RESET -> SystemState.DISPLAYING_POSE_RESETTING;
            case DISPLAY_POSE_BASED -> SystemState.DISPLAYING_POSE_BASED;
            case DISPLAY_ROBOT_IS_PHYSICALLY_READY_FOR_MATCH -> SystemState.DISPLAYING_ROBOT_IS_PHYSICALLY_READY_FOR_MATCH;
            case DISPLAY_ROTATION_BASED -> SystemState.DISPLAYING_ROTATION_BASED;
            case DISPLAY_ROBOT_ARM_NOT_ZEROED -> SystemState.DISPLAYING_ROBOT_ARM_NOT_ZEROED;
            case DISPLAY_NOT_AT_KICKSTAND_POSITION -> SystemState.DISPLAYING_NOT_AT_KICKSTAND_POSITION;
            case DISPLAY_READY_FOR_MATCH -> SystemState.DISPLAYING_READY_FOR_MATCH;
            case DISPLAY_ROBOT_ZERO_ACTION -> SystemState.DISPLAYING_ROBOT_ZERO_ACTION;
            case DISPLAY_OFF -> SystemState.DISPLAYING_OFF;
            case DISPLAY_INTAKING_ALGAE -> SystemState.DISPLAYING_INTAKING_ALGAE;
            case DISPLAY_HOLDING_ALGAE -> SystemState.DISPLAYING_HOLDING_ALGAE;
            case DISPLAY_ALIGN_TO_BARGE -> SystemState.DISPLAYING_ALIGN_TO_BARGE;
            case DISPLAY_SCORE_BARGE -> SystemState.DISPLAYING_SCORE_BARGE;
            case DISPLAY_ALIGN_TO_PROCESSOR -> SystemState.DISPLAYING_ALIGN_TO_PROCESSOR;
            case DISPLAY_SCORE_PROCESSOR -> SystemState.DISPLAYING_SCORE_PROCESSOR;
            case DISPLAY_INTAKING_CORAL -> SystemState.DISPLAYING_INTAKING_CORAL;
            case DISPLAY_INTAKING_CORAL_HORIZONTAL -> SystemState.DISPLAYING_INTAKING_CORAL_HORIZONTAL;
            case DISPLAY_HOLDING_CORAL_POSE_BASED -> SystemState.DISPLAYING_HOLDING_CORAL_POSE_BASED;
            case DISPLAY_HOLDING_CORAL_ROTATION_BASED -> SystemState.DISPLAYING_HOLDING_CORAL_ROTATION_BASED;
            case DISPLAY_ALIGN_TO_TARGET_L_ONE -> SystemState.DISPLAYING_ALIGN_TO_TARGET_L_ONE;
            case DISPLAY_ALIGN_TO_TARGET_L_TWO -> SystemState.DISPLAYING_ALIGN_TO_TARGET_L_TWO;
            case DISPLAY_ALIGN_TO_TARGET_L_THREE -> SystemState.DISPLAYING_ALIGN_TO_TARGET_L_THREE;
            case DISPLAY_ALIGN_TO_TARGET_L_FOUR -> SystemState.DISPLAYING_ALIGN_TO_TARGET_L_FOUR;
            case TAG_NOT_SEEN -> SystemState.TAG_NOT_SEEN;
            case DISPLAY_CAN_SCORE_L_ONE -> SystemState.DISPLAYING_CAN_SCORE_L_ONE;
            case DISPLAY_CAN_SCORE_L_TWO -> SystemState.DISPLAYING_CAN_SCORE_L_TWO;
            case DISPLAY_CAN_SCORE_L_THREE -> SystemState.DISPLAYING_CAN_SCORE_L_THREE;
            case DISPLAY_CAN_SCORE_L_FOUR -> SystemState.DISPLAYING_CAN_SCORE_L_FOUR;
            case DISPLAY_CLIMB_ALIGN -> SystemState.DISPLAYING_CLIMB_ALIGN;
            case DISPLAY_CLIMB_AUTOMATED -> SystemState.DISPLAYING_CLIMB_AUTOMATED;
            case DISPLAY_CONTROLLERS_ACTIVE -> SystemState.DISPLAYING_CONTROLLER_ACTIVE;
        };
    }

    public LedSubsystem(LedIO ledIO) {
        this.ledIO = ledIO;
    }

    public void setWantedAction(WantedState wantedAction) {
        this.wantedAction = wantedAction;
    }

    @Override
    public void periodic() {

        Logger.recordOutput("Subsystems/LED/WantedState", wantedAction);

        switch (getStateTransition()) {
            case DISPLAYING_READY_FOR_MATCH:
                ledIO.setAnimation(new LarsonAnimation(0, 255, 0, 0, 0.5, NUMBER_OF_LEDS - 30, LarsonAnimation.BounceMode.Center, 7));
                break;
            case DISPLAYING_ROBOT_IS_PHYSICALLY_READY_FOR_MATCH:
                if (GlobalConstants.isBlueAlliance()) {
                    ledIO.setAnimation(new LarsonAnimation(0, 0, 255, 0, 0.3, NUMBER_OF_LEDS, LarsonAnimation.BounceMode.Back, 7, 8));
                } else {
                    ledIO.setAnimation(new LarsonAnimation(255, 0, 0, 0, 0.3, NUMBER_OF_LEDS, LarsonAnimation.BounceMode.Back, 7, 8));
                }

                break;
            case DISPLAYING_POSE_RESETTING:
                ledIO.setAnimation(getAnimation(AnimationType.FADE, 255, 255, 255, 0.9));
                break;
            case DISPLAYING_ROBOT_ARM_ZEROED:
                ledIO.clearAnimation();
                ledIO.setLEDs(0, 255, 0);
                break;
            case DISPLAYING_POSE_BASED:
                ledIO.clearAnimation();
                ledIO.setLEDs(255, 0, 255);
                break;
            case DISPLAYING_ROTATION_BASED:
                ledIO.clearAnimation();
                ledIO.setLEDs(237, 198, 0);
                break;
            case DISPLAYING_ROBOT_ARM_NOT_ZEROED:
                ledIO.clearAnimation();
                ledIO.setLEDs(255, 30, 0);
                break;
            case DISPLAYING_NOT_AT_KICKSTAND_POSITION:
                ledIO.setLEDs(255, 100, 230);
                break;
            case DISPLAYING_ROBOT_ZERO_ACTION:
                ledIO.setAnimation(getAnimation(AnimationType.LARSON, 0, 100, 200, 0.9));
                break;
            case DISPLAYING_OFF:
                ledIO.clearAnimation();
                ledIO.setLEDs(0, 0, 0);
                break;
            case DISPLAYING_INTAKING_ALGAE:
                ledIO.setLEDs(66, 245, 233);
                break;
            case DISPLAYING_HOLDING_ALGAE:
                ledIO.setAnimation(getAnimation(AnimationType.FADE, 66, 245, 233, 0.9));
                break;
            case DISPLAYING_ALIGN_TO_BARGE:
                ledIO.setAnimation(getAnimation(AnimationType.FADE, 0, 0, 255, 0.9));
                break;
            case DISPLAYING_SCORE_BARGE:
                ledIO.setAnimation(getAnimation(AnimationType.STROBE, 0, 0, 255, 0.8));
                break;
            case DISPLAYING_ALIGN_TO_PROCESSOR:
                ledIO.setAnimation(getAnimation(AnimationType.FADE, 255, 0, 25, 0.9));
                break;
            case DISPLAYING_SCORE_PROCESSOR:
                ledIO.setAnimation(getAnimation(AnimationType.STROBE, 255, 0, 25, 0.8));
                break;
            case DISPLAYING_INTAKING_CORAL:
                ledIO.setLEDs(255, 255, 255);
                break;
            case DISPLAYING_INTAKING_CORAL_HORIZONTAL:
                ledIO.setLEDs(200, 100, 255);
                break;
            case DISPLAYING_HOLDING_CORAL_POSE_BASED:
                ledIO.setAnimation(getAnimation(AnimationType.FADE, 255, 0, 255, 0.95));
                break;
            case DISPLAYING_HOLDING_CORAL_ROTATION_BASED:
                ledIO.setAnimation(getAnimation(AnimationType.FADE, 237, 198, 0, 0.95));
                break;
            case DISPLAYING_ALIGN_TO_TARGET_L_ONE, DISPLAYING_CAN_SCORE_L_ONE:
                ledIO.clearAnimation();
                ledIO.setLEDs(113, 0, 242);
                break;
            case DISPLAYING_ALIGN_TO_TARGET_L_TWO:
                // ledIO.setAnimation(getAnimation(AnimationType.FADE, 255, 194, 151, 0.9));
                ledIO.setLEDs(0, 0, 0);
                break;
            case DISPLAYING_ALIGN_TO_TARGET_L_THREE:
                // ledIO.setAnimation(getAnimation(AnimationType.FADE, 255, 131, 246, 0.9));
                ledIO.setLEDs(0, 0, 0);
                break;
            case DISPLAYING_ALIGN_TO_TARGET_L_FOUR:
                // ledIO.setAnimation(getAnimation(AnimationType.FADE, 255, 0, 237, 0.9));
                ledIO.setLEDs(0, 0, 0);
                break;
            case TAG_NOT_SEEN:
                ledIO.setAnimation(getAnimation(AnimationType.STROBE, 255, 0, 0, 0.75));
                break;
            case DISPLAYING_CAN_SCORE_L_TWO:
                // ledIO.setAnimation(getAnimation(AnimationType.STROBE, 255, 194, 151, 0.75));
                ledIO.setLEDs(0, 0, 0);
                break;
            case DISPLAYING_CAN_SCORE_L_THREE:
                // ledIO.setAnimation(getAnimation(AnimationType.STROBE, 255, 131, 246, 0.75));
                ledIO.setLEDs(0, 0, 0);
                break;
            case DISPLAYING_CAN_SCORE_L_FOUR:
                // ledIO.setAnimation(getAnimation(AnimationType.STROBE, 255, 0, 237, 0.75));
                ledIO.setLEDs(0, 0, 0);
                break;
            case DISPLAYING_CLIMB_ALIGN:
                ledIO.setAnimation(getAnimation(AnimationType.FADE, 40, 255, 0, 0.9));
                break;
            case DISPLAYING_CLIMB_AUTOMATED:
                ledIO.setAnimation(getAnimation(AnimationType.FIRE, 10, 0, 200, 1));
                break;
            case DISPLAYING_CONTROLLER_ACTIVE:
                ledIO.setAnimation(getAnimation(AnimationType.RAINBOW, 2, 9, 10, 0.95));
                break;
            default:
                System.out.println("Fell through on LED commands");
                break;
        }
    }

    private static Animation getAnimation(AnimationType type, int red, int green, int blue, double speed) {
        return switch (type) {
            default -> new StrobeAnimation(red, green, blue, 0, speed, NUMBER_OF_LEDS);
            case FADE -> new SingleFadeAnimation(red, green, blue, 0, speed, NUMBER_OF_LEDS);
            case FIRE -> new FireAnimation(1.0, speed, NUMBER_OF_LEDS, 0.2, 0.2);
            case LARSON -> new LarsonAnimation(
                    red, green, blue, 0, speed, NUMBER_OF_LEDS, LarsonAnimation.BounceMode.Back, 5);
            case TWINKLE -> new TwinkleAnimation(
                    red, green, blue, 0, speed, NUMBER_OF_LEDS, TwinkleAnimation.TwinklePercent.Percent88);
            case RAINBOW -> new RainbowAnimation(1, speed, NUMBER_OF_LEDS, false, 0);
        };
    }
}