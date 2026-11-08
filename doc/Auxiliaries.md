  # Auxiliaries

  This is the command layer for non-drive auto behavior: flywheel, gate, intake,
  and flower.

  The pattern is simple:

  - `beforeSeconds` = stop and wait before the action
  - `actionSeconds` = how long the action runs
  - `afterSeconds` = extra wait after it finishes
  - `moveAtSameTime` = run this action alongside the robot movement

  The gate stays closed unless the plan is intentionally shooting.

  ## Command shape

  Most helpers follow this pattern:

  ```java
  Auxiliaries.shoot(0.0, 1.0, 0.1);
  // before, action, after
  ```

  The shorthand versions still work too:

  ```java
  Auxiliaries.shoot(0.0, 1.0);
  Auxiliaries.shooterOn(0.0, 0.5);
  Auxiliaries.intakeOn(0.0, 1.5);
  ```

  If the action should happen while the robot is moving, attach it to the path:

  ```java
  Auxiliaries.shooterOn(0.0, 0.5, 0.0)
          .with(PedroCommands.follow(follower, path1()));
  ```

  ## Safety rules

  - The gate is closed before any feeder or flywheel action starts.
  - Only the shot commands open the gate.
  - Cleanup always runs on interrupt.
  - The intake and flower stop when the command ends.

  ## Tuning values

  These live in the FTC Dashboard config page `AutoAuxiliaries`:

  - `FLYWHEEL_TARGET_TICKS_PER_SECOND`
  - `GATE_SETTLE_SECONDS`
  - `INTAKE_POWER`
  - `FLOWER_POWER`
  - `FLOWER_RAMP_UP_SECONDS`
  - `FLOWER_RAMP_DOWN_SECONDS`
  - `SHOOT_FEED_SECONDS`

  ## Example

  ```java
  return sequential(
          Auxiliaries.shooterOn(0.0, 0.5),
          PedroCommands.follow(follower, path1()),
          Auxiliaries.shoot(0.0, 1.0),
          Auxiliaries.shooterOff(0.0)
  );
  ```

  This is the basic flow: spin up, drive, shoot, then shut down.
