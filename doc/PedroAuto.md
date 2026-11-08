# Pedro Auto — how to build an autonomous

## File map

| File | What it is | You edit it? |
|---|---|---|
| `auto/PedroAuto.java` | The one OpMode on the controller. Alliance (X/B) + side (D-pad up/down) selection, Ivy scheduler loop, pose handoff to TeleOp. | Rarely. |
| `auto/Auxiliaries.java` | All mechanism actions as Ivy commands (`shoot()`, gate, intake...). | **Yes — mechanism code lives here.** |
| `auto/paths/RedUp.java` `RedDown.java` `BlueUp.java` `BlueDown.java` | One auto each. Past Path-Visualizer output between the markers + one `plan()` wiring it up. | **Yes — paste + plan().** |
| `pedro/Constants.java` | Follower config (motors, Pinpoint, Foresight gains). | Only when tuner output changes. |

## The workflow (draw an autonomous in 3 steps)

1. **Draw** the path in Pedro's Path Visualizer. It generates a `Paths.java`
   with `Pose` fields and `public Path pathN()` methods.
2. **Paste** that generated body between the
   `// VISUALIZER OUTPUT ... END VISUALIZER OUTPUT` markers in the right file
   (`RedUp`, `RedDown`, `BlueUp`, or `BlueDown`). Don't hand-fix anything
   inside the markers; if the path changes, re-paste the whole block.
3. **Wire `plan()`** at the bottom of the same file. It returns the entire
   autonomous as a single Ivy command, usually `sequential(...)`:

```java
@Override
public Command plan(Follower follower) {
    return sequential(
            PedroCommands.follow(follower, path1()),   // Pedro drives path1
            Auxiliaries.shoot(),                       // one-shot action (Auxiliaries.java)
            PedroCommands.follow(follower, path2()),
            Commands.waitMs(500),                      // pause
            PedroCommands.follow(follower, path3())
    );
}
```

```java
// Run the intake alongside the path; the intake is force-stopped as soon
// as the path finishes (.until(...) force-finishes a command early):
sequential(
        Auxiliaries.intakeOn(),
        Auxiliaries.intakeOn()                      // stays on until interrupted
                .until(() -> !follower.isBusy())    // path no longer running
                .with(PedroCommands.follow(follower, path2())),
        Auxiliaries.intakeStop()                    // hard-off, just in case
)
```

(When unsure, the simplest correct pattern is the first one: sequential
actions around `follow()` calls. Reach for `with`/`parallel` only when
you need something running *during* a path.)

## Ivy command system (verified API, ivy 1.1.1)

A command has four parts: `start()` (once, when scheduled), `execute()`
(every loop), `done()` (true = finished), `end(endCondition)` (cleanup —
also runs on interrupt). Builders returned by the factory methods below
already implement `Command`, so you can pass them straight into groups and
`Scheduler.schedule`.

From `com.pedropathing.ivy`:

```java
import static com.pedropathing.ivy.Scheduler.schedule;          // run at start()
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.ivy.pedro.PedroCommands;               // the Pedro glue

// --- com.pedropathing.ivy.commands.Commands ---
Commands.instant(Runnable)                          // run once, done immediately
Commands.infinite(Runnable)                         // never done until interrupted
Commands.waitMs(double)                             // timed wait (seconds!)
Commands.waitUntil(BooleanSupplier)                 // wait for a condition
Commands.onInterrupt(Runnable)                      // cleanup-only command
Commands.conditional(BooleanSupplier, ifTrue, ifFalse)

// --- com.pedropathing.ivy.groups.Groups ---
Groups.sequential(Command...)                       // one after another
Groups.parallel(Command...)                         // all at once, done when all done
Groups.race(Command...)                             // done when the FIRST finishes
Groups.deadline(Command d, Command...)              // others run until d finishes
Groups.repeat(Command, int)                         // N times
Groups.loop(Command)                                // forever

// --- com.pedropathing.ivy.pedro.PedroCommands ---   (the only Pedro glue)
PedroCommands.follow(Follower, Path)                // drive one path to completion
PedroCommands.hold(Follower) / hold(Follower, Pose) // hold a point/pose forever

// --- chaining on any command/builder ---
cmd.then(cmd2...)                                   // = sequential
cmd.with(cmd2...)                                   // run alongside, might outlive it by a tick
cmd.raceWith(cmd2...)                               // longest shared time
cmd.until(BooleanSupplier)                          // force-finish early on condition
cmd.unless(BooleanSupplier)
```

`Scheduler` (used by `PedroAuto`, not by the path files):

```java
Scheduler.schedule(plan.plan(follower));   // at start()
Scheduler.execute();                       // every loop
Scheduler.reset();                         // at stop()
Scheduler.isScheduled(cmd) / cancel(cmd)
```

Requirements/priorities (only matter if two commands fight over the same
subsystem): `Command.build().requiring("intake").setPriority(2)` — a
scheduled command that needs a busy requirement is blocked or bumps the
other one depending on behavior config. For autos the sequential style above
typically needs none of it.

## Auxiliaries — mechanism commands

All mechanism commands (`shooterOn`, `shoot`, intake, flower ramps, the gate
rule, and the wait/run durations on every command) are documented in
**[doc/Auxiliaries.md](Auxiliaries.md)** — read that doc before writing a
`plan()`; the short version is: shut the gate first, only `shoot()` opens it,
and every command takes `(waitSeconds, runSeconds)` at the call site.

Typical plan wiring:

```java
return sequential(
        Auxiliaries.shooterOn(),                            // gate shut, spin up
        PedroCommands.follow(follower, path1()),            // drive at speed
        Auxiliaries.shoot(),                                // gate opens only here
        Auxiliaries.shooterOff()                            // gate shut, spin down
);
// Spin up WHILE driving:
Auxiliaries.shooterOn().with(PedroCommands.follow(follower, path1()))
```

Safety nets around commands:

- `Auxiliaries.init(hardwareMap)` — called by `PedroAuto` at init; the commands
  grab their devices by `RobotConstants` names (deliberately NOT a second
  `RobotHardware`, whose constructor would re-configure the drive motors
  Pedro already owns).
- `Auxiliaries.shutdown()` — called by `PedroAuto` at stop: flywheel off,
  feeder off, flower stopped, gate commanded shut.
- All commands are null-device tolerant: any missing mechanism is a no-op, so
  a partially-built robot still runs paths.
- End-of-ramp: `flowerRampUp/Down` land exactly on their target power even if
  interrupted mid-ramp (`setEnd`).

## `PedroAuto` runtime contract

- `follower.update()` + `Scheduler.execute()` run once per loop, in that order.
- At start the pose is stamped from your file's `startPose()` (return the
  visualizer's `start` pose — the robot must be placed there at init).
- On stop: pose is saved into `PoseStorage` (custom `Navigation` classes), so
  `MainTeleOp` picks up where auto left off.
- `PedroCommands.follow()` is the ONLY correct way to hand a path to Pedro
  inside a command. Don't call `follower.follow(path)` inside `instant(...)`.

## Units / frames

- Poses: x/y inches in the Pedro field frame; headings via
  `PoseFactory.degrees().of(x, y, headingDeg)` (stored in radians).
- Pinpoint config comes from `RobotConstants`; Pedro pose coordinates agree
  with the custom `Navigation.Pinpoint` classes (same device, same values).

## Gotchas

- The four files' `plan()` gets the `Follower` as a parameter — pass it into
  `PedroCommands.follow(...)`, don't cache it in a field.
- Everything inside the visualizer markers is generated code: fields and
  `pathN()` methods live there. Generated names may collide with your
  `plan()`; if you add fields, put them below the end marker.
- `Commands.waitMs(1000)` is one second — despite the name it is verified in
  bytecode to take **milliseconds** (done = `currentTimeMillis() - start >= x`).

## Tuning reminder

The follower's gains live in `pedro/Constants.java` (`ForesightConfig`) and
are PLACEHOLDERS today. Run Pedro's tuning procedures (Mecanum → Foresight)
and paste the generated config over the lambda in `Constants.java` before
trusting any path run.
