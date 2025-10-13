package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Optional;
import java.util.function.Consumer;

public class ModalControls {
    private static Optional<ModalControls> instance = Optional.empty();

    public enum Mode {
        CORAL,
        ALGAE,
        CORAL_MANUAL
    }

    private Mode currentMode = Mode.CORAL;
    private Consumer<Mode> stateChangeConsumer;

    private Trigger scoreTrigger;
    private Trigger intakeTrigger;

    private Trigger setDefaultRobotWideTrigger;
    private Trigger setDefaultRobotTightTrigger;

    public static ModalControls getInstance() {
        if (instance.isEmpty()) {
            instance = Optional.of(new ModalControls());
        }
        return instance.get();
    }

    public Mode getMode() {
        return currentMode;
    }

    public void setMode(Mode mode) {
        this.currentMode = mode;
    }

    private void maybeTriggerStateChangeConsumer(Mode newMode) {
        if (this.currentMode != newMode && this.stateChangeConsumer != null) {
            this.stateChangeConsumer.accept(newMode);
        }
    }

    private Trigger modeSpecific(Trigger trigger, Mode mode) {
        return trigger.and(new Trigger(() -> this.currentMode == mode));
    }

    public void forceSetMode(Mode mode) {
        maybeTriggerStateChangeConsumer(mode);
        setMode(mode);
    }

    public void configureBindings() {
        Controlboard.getInstance().getCoralMode().onTrue(Commands.runOnce(() -> {
            maybeTriggerStateChangeConsumer(Mode.CORAL);
            setMode(Mode.CORAL);
        }));

        Controlboard.getInstance().getAlgaeMode().onTrue(Commands.runOnce(() -> {
            maybeTriggerStateChangeConsumer(Mode.ALGAE);
            setMode(Mode.ALGAE);
        }));

        Controlboard.getInstance().getCoralManualMode().onTrue(Commands.runOnce(() -> {
            maybeTriggerStateChangeConsumer(Mode.CORAL_MANUAL);
            setMode(Mode.CORAL_MANUAL);
        }));

        scoreTrigger = Controlboard.getInstance().score();
        intakeTrigger = Controlboard.getInstance().intakeFunnel();
        setDefaultRobotWideTrigger = Controlboard.getInstance().setDefaultRobotWide();
        setDefaultRobotTightTrigger = Controlboard.getInstance().setDefaultRobotTight();
    }

    private Trigger intake() {
        return intakeTrigger.and(scoreTrigger.negate());
    }

    private Trigger score() {
        return scoreTrigger.and(intakeTrigger.negate());
    }

    public Trigger setDefaultRobotWide() {
        return setDefaultRobotWideTrigger.and(setDefaultRobotTightTrigger.negate());
    }

    public Trigger setDefaultRobotTight() {
        return setDefaultRobotTightTrigger.and(setDefaultRobotWideTrigger.negate());
    }

    public Trigger algaeClimbMode() {
        return new Trigger(() -> this.currentMode == Mode.ALGAE);
    }

    public Trigger coralMode() {
        return new Trigger(() -> this.currentMode == Mode.CORAL);
    }

    public Trigger coralManualMode() {
        return new Trigger(() -> this.currentMode == Mode.CORAL_MANUAL);
    }

    /////////////// ALGAE CLIMB MODE ///////////////

    public Trigger autoAlignReefIntake() {
        return modeSpecific(Controlboard.getInstance().autoAlignCenter(), Mode.ALGAE);
    }

    public Trigger groundIntakeAlgae() {
        return modeSpecific(intake(), Mode.ALGAE);
    }

    public Trigger bargeManualStage() {
        return modeSpecific(Controlboard.getInstance().bargeManualStage(), Mode.ALGAE);
    }

    public Trigger processorManualStage() {
        return modeSpecific(Controlboard.getInstance().processorManualStage(), Mode.ALGAE);
    }

    public Trigger bargeAutoAlignStage() {
        return modeSpecific(Controlboard.getInstance().povUp(), Mode.ALGAE);
    }

    public Trigger processorAutoAlignStage() {
        return modeSpecific(Controlboard.getInstance().povDown(), Mode.ALGAE);
    }

    public Trigger scoreAlgae() {
        return modeSpecific(score(), Mode.ALGAE);
    }

    public Trigger stageAlgaeL2() {
        return modeSpecific(Controlboard.getInstance().reefIntakeAlgae(), Mode.ALGAE);
    }

    public Trigger stageAlgaeL3() {
        return modeSpecific(Controlboard.getInstance().scoreBarge(), Mode.ALGAE);
    }

    public Trigger manualIntakeAlgae() {
        return modeSpecific(Controlboard.getInstance().manualIntakeAlgae(), Mode.ALGAE);
    }

    public Trigger moveToAlgaeLevel() {
        return modeSpecific(Controlboard.getInstance().score(), Mode.ALGAE);
    }

    public Trigger deployClimber() {
        return modeSpecific(Controlboard.getInstance().povLeft(), Mode.ALGAE);
    }

    public Trigger climb() {
        return modeSpecific(Controlboard.getInstance().povRight(), Mode.ALGAE);
    }

    public Trigger coralIntakeToIndexer() {
        return modeSpecific(Controlboard.getInstance().rightStick(), Mode.ALGAE);
    }

    public Trigger stowAlgae() {
        return modeSpecific(Controlboard.getInstance().stow(), Mode.ALGAE);
    }

    /////////////// CORAL MODE ///////////////

    public Trigger leftBranchSelect() {
        return modeSpecific(Controlboard.getInstance().autoAlignLeft(), Mode.CORAL);
    }

    public Trigger rightBranchSelect() {
        return modeSpecific(Controlboard.getInstance().autoAlignRight(), Mode.CORAL);
    }

    public Trigger stageL1() {
        return modeSpecific(Controlboard.getInstance().stageL1(), Mode.CORAL);
    }

    public Trigger stageL2() {
        return modeSpecific(Controlboard.getInstance().stageL2(), Mode.CORAL);
    }

    public Trigger stageL3() {
        return modeSpecific(Controlboard.getInstance().stageL3(), Mode.CORAL);
    }

    public Trigger stageL4() {
        return modeSpecific(Controlboard.getInstance().stageL4(), Mode.CORAL);
    }

    public Trigger groundCoralIntake() {
        return modeSpecific(intake(), Mode.CORAL);
    }

    public Trigger stageOrScoreCoral() {
        return modeSpecific(score(), Mode.CORAL);
    }

    public Trigger autoHP() {
        return modeSpecific(Controlboard.getInstance().autoAlignFeeder(), Mode.CORAL);
    }

    public Trigger povAutoAlignStage() {
        return new Trigger(() -> false);
    }

    public Trigger funnelIntakeCoral() {
        return modeSpecific(Controlboard.getInstance().intakeFunnel(), Mode.CORAL);
    }

    public Trigger coralExhaust() {
        return modeSpecific(Controlboard.getInstance().exhaust(), Mode.CORAL);
    }

    public Trigger stowCoral() {
        return modeSpecific(Controlboard.getInstance().stow(), Mode.CORAL);
    }

    public Trigger closestFaceAlign() {
        return modeSpecific(Controlboard.getInstance().rightStick(), Mode.CORAL);
    }

    /////////////// CORAL MANUAL MODE ///////////////

    public Trigger stowCoralManual() {
        return modeSpecific(Controlboard.getInstance().stow(), Mode.CORAL_MANUAL);
    }

    public Trigger groundCoralIntakeManual() {
        return modeSpecific(intake(), Mode.CORAL_MANUAL);
    }


    public Trigger coralExhaustManual() {
        return modeSpecific(Controlboard.getInstance().exhaust(), Mode.CORAL_MANUAL);
    }

    public Trigger stageL1Manual() {
        return modeSpecific(Controlboard.getInstance().stageL1(), Mode.CORAL_MANUAL);
    }

    public Trigger stageL2Manual() {
        return modeSpecific(Controlboard.getInstance().stageL2(), Mode.CORAL_MANUAL);
    }

    public Trigger stageL3Manual() {
        return modeSpecific(Controlboard.getInstance().stageL3(), Mode.CORAL_MANUAL);
    }

    public Trigger stageL4Manual() {
        return modeSpecific(Controlboard.getInstance().stageL4(), Mode.CORAL_MANUAL);
    }

    public Trigger stageOrScoreCoralManual() {
        return modeSpecific(score(), Mode.CORAL_MANUAL);
    }

    public Trigger funnelIntakeCoralManual() {
        return modeSpecific(Controlboard.getInstance().intakeFunnel(), Mode.CORAL_MANUAL);
    }
}