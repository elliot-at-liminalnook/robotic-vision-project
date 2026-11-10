# How a small robot arm actually works: seven lessons

A self-contained course on the ideas behind a hobby-servo robot arm: its motors, its control, its
geometry, its simulation and its camera. You need school algebra and nothing else. No calculus, no
prior robotics, no access to any particular codebase.

**How to use it.** The lessons build on each other, so read them in order. Each one has a core
idea, a worked example with real numbers, the mistakes people commonly make, and a few questions
with answers. If you can answer the questions without looking, you have the lesson.

| # | Lesson | The question it answers |
|---|---|---|
| 0 | Vocabulary and units | What do the words and numbers mean? |
| 1 | DC motor behaviour | How much can a motor push, and when? |
| 2 | The soft position loop and feed-forward | Why doesn't the arm go where I tell it? |
| 3 | Overload protection and heat | Why did the gripper suddenly let go? |
| 4 | Arm dynamics and the torque governor | How fast can the arm safely move? |
| 5 | Inverse kinematics | Which joint angles put the hand *there*? |
| 6 | Contact and friction simulation | Why do simulated grasps slip, jitter or explode? |
| 7 | The camera firmware pipeline | How does a picture get from a sensor to a PC? |

## The running example

Every number in this course comes from one arm, so the examples add up across lessons.

It is a desktop arm about 30 cm long, built from 3D-printed parts and six identical hobby servos.
Five servos move the arm and the sixth opens and closes a two-finger gripper:

```
                 elbow
                  o───────o wrist flex
                 /         \
                /           o wrist roll (spins the gripper)
     shoulder  o            ╨ gripper (two jaws)
               │
   base pan  ══╧══   (turns the whole arm left/right)
```

Each servo, powered at 12 V:

| Quantity | Value | Meaning |
|---|---|---|
| Stall torque | 3.18 N·m | The hardest it can twist, which happens at zero speed |
| No-load speed | 5.96 rad/s (~340°/s) | The fastest it can spin, which happens with no load |
| Gear ratio | 345 : 1 | The motor inside turns 345 times per turn of the output |
| Error for full torque | 9° | How far off-target it must be before it pushes its hardest |

The first three are datasheet values. The 9° is a typical figure for this class of servo, not a
measurement of this one, so treat every result that depends on it as approximate.

---

## Lesson 0. Vocabulary and units

**Angles in radians.** Engineers measure angles in radians (rad) because it makes one formula
trivial: *distance moved at the end of a lever = lever length × angle in radians*. One radian is
about 57.3°. A full turn is 2π ≈ 6.28 rad. So a 30 cm arm that rotates 0.05 rad (about 3°) moves
its tip 0.3 × 0.05 = 0.015 m = 15 mm.

**Force** is measured in newtons (N). One newton is roughly the weight of a 100 g object (a small
apple). Weight in newtons = mass in kg × 9.81.

**Torque** is twisting effort: force × the distance from the pivot at which it acts, in
newton-metres (N·m). It is the rotational version of force.

> Hold a 100 g object at arm's length, 30 cm from a pivot, with the lever horizontal. The pivot
> must supply about 1 N × 0.3 m = **0.3 N·m**. Move the object twice as far out and the torque
> doubles, even though the object weighs the same.

**Speed and acceleration** for rotation are in rad/s and rad/s². **Inertia** is resistance to
being spun up (the rotational version of mass). A mass on the end of a lever has inertia
*mass × length²*, so distance from the pivot matters twice over.

**Electricity.** Voltage (volts, V) is electrical pressure. Current (amps, A) is the flow it
causes. Resistance (ohms, Ω) is how hard the wire resists: *voltage = current × resistance*.

**Arm words.**
- A **joint** is a hinge. A **link** is the rigid piece between two joints.
- A **degree of freedom** (DOF) is one independent way to move. A hinge has one.
- The **tip** (or end effector) is the working end: here, the point between the gripper's jaws.
- A **pose** is a position plus an orientation. Describing where a rigid object is takes six
  numbers: three for position, three for which way it faces.
- A **servo** is a package: motor, gearbox, angle sensor and a small controller. You send it a
  target angle and it tries to go there.

---

## Lesson 1. DC motor behaviour

### The core idea

**A motor's torque falls as its speed rises, in a straight line.**

```
torque
  ▲
3.18 ●  stall: all the torque, no speed
  │   ╲
  │     ╲
1.59 ─ ─ ─ ●  half speed: half the torque
  │          ╲
  │            ╲
  0 ────────────●──▶ speed
               5.96  no-load: all the speed, no torque
```

```
torque available = stall torque × (1 − speed ÷ no-load speed)
```

### Why

Two facts about any permanent-magnet DC motor produce that line.

1. **Torque is proportional to current.** More amps through the windings, more twist.
2. **A spinning motor is also a generator.** Turning it produces a voltage of its own, called
   **back-EMF**, which is proportional to speed and *opposes* the supply.

Only the voltage left over after back-EMF can push current through the windings:

```
current = (supply voltage − back-EMF) ÷ winding resistance
```

At standstill there is no back-EMF, so current and torque are at their maximum: that's **stall**.
As the motor speeds up, back-EMF eats into the supply, current falls and torque falls with it. At
the **no-load speed**, back-EMF equals the supply, no current flows and there is no torque left.
The motor cannot go faster.

### Worked example

How much torque does our servo have left at 3 rad/s (about half speed)?

```
3.18 × (1 − 3 ÷ 5.96) = 3.18 × 0.497 = 1.58 N·m
```

Half of its strength is gone just because it is moving.

### What follows from the line

- **The motor is its own brake.** Every extra rad/s costs 3.18 ÷ 5.96 = **0.53 N·m**. That is
  exactly how a shock absorber (a damper) behaves: resistance proportional to speed. Simulators
  often model back-EMF as plain damping for this reason.
- **Voltage moves the whole line.** Stall torque and no-load speed are both proportional to supply
  voltage. Running at 12 V instead of 11.1 V gives about 8 % more of each.
- **Current follows torque, not speed.** A motor straining against something it can't move draws
  its maximum current. A motor spinning freely draws almost none.
- **Heat goes with torque squared.** Heat in the windings is *current² × resistance*, and current
  is proportional to torque. Holding 80 % of stall makes (0.8 ÷ 0.33)² ≈ **6 times** the heat of
  holding 33 %. Small cuts in holding torque buy a lot of cooling.
- **A stalled motor does no work and makes the most heat.** Mechanical power is torque × speed;
  at zero speed that is zero, so every watt drawn becomes heat.

### The gearbox

The motor inside a servo is tiny, fast and weak. A gearbox with ratio 345 : 1 trades one for the
other:

| At the output shaft | Effect of a ratio N |
|---|---|
| Torque | × N |
| Speed | ÷ N |
| The motor's own inertia, as felt at the joint | × N² |

The last row surprises people. The motor's spinning rotor weighs a few grams, but to accelerate
the joint the rotor must accelerate 345 times faster, and the torque needed for that is multiplied
by 345 again on the way out. So the rotor feels 345² ≈ 119,000 times heavier at the joint. In a
small geared arm, a good share of the effort of accelerating goes into spinning up the motors
themselves. Simulators call this term **armature** or reflected inertia.

### Common mistakes

- *"It's not moving, so it's not working hard."* The reverse: holding still against a load is
  when current and heat are highest.
- *"The datasheet says 3 N·m and 340°/s, so I can have both."* You can have one or the other, or
  a trade along the line.

### Check yourself

1. How much torque is available at 4 rad/s?
2. You raise the supply from 11.1 V to 12 V. A stalled joint now pushes harder. Does it also run
   hotter?
3. Why is a gripper squeezing an object for a minute harder on a servo than an arm waving around
   for a minute?

*Answers.* (1) 3.18 × (1 − 4 ÷ 5.96) ≈ 1.05 N·m. (2) Yes: more voltage at stall means more
current, and heat goes with current squared. (3) The gripper is stalled at high torque the whole
time, so it draws high current continuously; the waving arm spends most of its time at modest
torque.

---

## Lesson 2. The soft position loop and feed-forward

### The core idea

**A servo only pushes when it is in the wrong place, so under load it is always slightly in the
wrong place.**

### How a servo decides how hard to push

Inside the servo a small controller repeats one loop thousands of times a second: read the angle
sensor, subtract it from the target, and drive the motor in proportion to the difference (the
**error**). This is **proportional feedback control**.

```
motor drive
   ▲
100%│         ┌───────────  saturated: it has no more to give
    │       ╱
    │     ╱
    │   ╱   drive ∝ error
  0 └─╱──────────────────▶ error
    0°        9°
```

Zero error means zero drive. Our servo reaches full drive at 9° of error and can't exceed it
(that's **saturation**).

**So a servo behaves like a spring** connecting where you asked it to be and where it is. Its
**stiffness** is how much torque you get per unit of error:

```
stiffness = 3.18 N·m ÷ 9°  =  3.18 ÷ 0.157 rad  ≈  20 N·m per rad
```

### Two errors that follow

**Droop (standing still).** To hold a load, the spring must stretch. Suppose gravity puts 1 N·m
on the shoulder:

```
error = torque ÷ stiffness = 1 ÷ 20 = 0.05 rad ≈ 3°
```

On a 30 cm arm that is 0.3 × 0.05 = **15 mm** low. The arm settles below every target, and by a
different amount in every pose.

**Lag (while moving).** From lesson 1, moving costs 0.53 N·m per rad/s. To move at 2.75 rad/s the
motor needs 0.53 × 2.75 ≈ 1.5 N·m of drive just to overcome its own back-EMF, and the only way
it gets drive is from error:

```
error = 1.5 ÷ 20 = 0.073 rad ≈ 4°
```

So the arm trails a moving command by about 4° before it is carrying anything.

### Why not just make the servo stiffer?

Because a stiff loop with any delay or gear slack in it overshoots, then over-corrects, and
buzzes. Stiffness is limited by stability, so cheap servos are soft on purpose.

Fuller controllers add two more terms, giving **PID**: P is the spring described above, D
(derivative) adds damping, and I (integral) slowly builds up extra drive for as long as an error
persists, which removes droop over time. Many hobby servos ship with the I term off or weak,
because it causes its own trouble when the motor saturates. Don't count on it.

### Feed-forward

**Ask for more than you want, by exactly the amount you know you'll fall short.**

Feedback corrects errors after they appear. But droop and lag aren't surprises: you can calculate
both in advance. So shift the command before sending it:

```
target to send = angle you want + (gravity torque + 0.53 × speed you want) ÷ stiffness
```

It is the difference between cruise control that reacts once the car has already slowed on a
hill, and a driver who sees the hill coming and presses the pedal early.

**Worked example.** You want the shoulder at 0.50 rad, moving at 1 rad/s, and gravity there is
1 N·m.

```
extra = (1 + 0.53 × 1) ÷ 20 = 0.077 rad
send 0.577 rad; the arm arrives at about 0.50
```

### What to remember about feed-forward

- **It is only as good as your model.** If the real stiffness is 20 % different from what you
  assumed, about a fifth of the error comes back. Measuring the servo matters.
- **It can't fix what it can't predict**: a bump, an object of unknown weight. Feedback handles
  those. The two work together: feed-forward does the bulk, feedback cleans up.
- **It needs a gravity model**, which is lesson 4.

### Common mistakes

- *"The servo is in position mode, so it goes to the position."* It goes to wherever spring force
  balances the load.
- *"The arm sags, so the servo is too weak."* Usually it has torque to spare; it just isn't using
  it because the error is small.

### Check yourself

1. A joint holds 2 N·m. How far does it droop?
2. The same joint is commanded to move *downward*, with gravity helping. Does it lag more or less
   than when moving up?
3. You apply feed-forward and the arm now sits slightly *above* its targets. What is the likely
   cause?

*Answers.* (1) 2 ÷ 20 = 0.1 rad ≈ 5.7°. (2) Less: gravity supplies some of the torque the motion
needs, so less error is required. (3) The real servo is stiffer than you assumed, or the arm is
lighter than your model says, so you are over-compensating.

---

## Lesson 3. Overload protection and heat

### The core idea

**A servo protects itself from burning out, and when it does, it looks exactly like a failure.**

### The rule

Stall current heats the windings quickly, so the servo's controller watches its own output. In
this servo family the rule is: if output stays above about 80 % of the torque limit for 2
seconds, cut output to about 20 % until the load is removed. (Thresholds vary by model and are
often configurable.)

From the outside: the arm suddenly sags, or the gripper drops what it was holding.

### The classic way to trigger it

Tell a gripper to "close fully" on an object.

```
target: jaws closed ────────────────┐
                                    │  large error that can never shrink
actual: jaws stopped by the object ─┘
```

The object stops the jaws well short of the target. The error stays large, so by lesson 2 the
servo drives at its limit. It is stalled, so by lesson 1 it draws maximum current. Two seconds
later the protection trips and the grip lets go.

### The fix: grip by offset

Use the servo's softness as a force setting.

1. Close the jaws while watching the measured angle.
2. **Detect contact**: the command is well past where the jaws are, yet the jaws have stopped
   moving.
3. Note the angle where they stopped, and set the target just a little beyond it.

With a 3° offset:

```
squeeze torque = stiffness × offset = 20 × 0.052 rad ≈ 1.06 N·m  (33 % of stall)
```

That is a firm grip that stays under the threshold and runs much cooler (lesson 1: heat goes with
torque squared). If the jaw tip were 8 cm from the servo's axis, 1.06 N·m would be about 13 N of
squeeze, roughly the weight of a 1.3 kg object.

Because *force = stiffness × offset*, a soft position servo can control force without a force
sensor. Want a gentler grip? Use a smaller offset.

### Two separate budgets

| Budget | Timescale | Limit | Analogy |
|---|---|---|---|
| Overload | Seconds | 80 % for 2 s | A sprint |
| Heat | Minutes | About a third of stall, continuously | A marathon pace |

Because heat goes with torque squared, the right average for the heat budget is **RMS** (root
mean square): square the torques, average them, take the square root.

> A joint spends 10 s at 80 % of stall, then 50 s at 10 %, and repeats.
> Mean of squares = (0.64 × 10 + 0.01 × 50) ÷ 60 = 0.115. Square root = **0.34**.
> That is right at the one-third continuous limit, even though the plain average is only 22 %.
> Short bursts of high torque cost far more than they seem to.

### The arm has the same problem

Holding an object far from the shoulder is a slow stall. A 300 g object at 30 cm needs
0.3 × 9.81 × 0.3 ≈ 0.9 N·m from the shoulder, 28 % of stall, before counting the arm's own weight.
Choosing poses that keep loads close in is not cosmetic: it decides whether the arm can hold a
pose for a minute.

### Common mistakes

- *"More squeeze is a safer grip."* More squeeze is a grip that lets go after two seconds.
- *"It worked in a five-second test."* Heat problems appear after minutes.

### Check yourself

1. What offset gives a squeeze of 20 % of stall?
2. A joint alternates 5 s at 90 % and 5 s at 0 %. Is it within the heat budget?
3. The gripper holds an object fine, but drops it when the arm swings quickly. Suggest two causes
   from lessons 1–3.

*Answers.* (1) 0.2 × 3.18 = 0.64 N·m; 0.64 ÷ 20 = 0.032 rad ≈ 1.8°. (2) No: RMS =
√(0.81 × 5 ÷ 10) ≈ 0.64, about twice the limit (and each 5 s burst also breaks the 2 s overload
rule). (3) The squeeze was sized for the object's weight alone, and swinging adds force; or the
swing pushed an arm joint into overload and it sagged.

---

## Lesson 4. Arm dynamics and the torque governor

### The core idea

**One equation tells you the torque every joint needs for any motion, so you can check a motion
before you command it.**

### Start with one link

A single link with a mass on the end, driven by one motor, needs torque for two things:

```
torque = inertia × acceleration  +  gravity torque
       = (m × L²) × acceleration +  m × 9.81 × L × cos(angle above horizontal)
```

**Worked example.** A 200 g mass on a 25 cm link.

- Holding it horizontal: 0.2 × 9.81 × 0.25 = **0.49 N·m**. Pointing straight up: zero.
- Accelerating at 10 rad/s² (brisk): inertia is 0.2 × 0.25² = 0.0125, so 0.0125 × 10 =
  **0.125 N·m**.

Two lessons already: gravity is the bigger term on a small arm, and it depends on the pose.

### Now a whole arm

With several links the same equation holds, but each term becomes a table:

```
torque = M(q) × acceleration  +  C(q, speed)  +  g(q)  +  friction
         inertia                 swinging         gravity
```

Here `q` means "all the joint angles at once": the arm's pose.

- **`M(q)`, the inertia matrix.** How hard each joint is to accelerate. It changes with pose: a
  stretched-out arm is far harder to swing than a folded one (think of a skater pulling their
  arms in). It is a *matrix* because joints affect each other: accelerate the elbow and the
  shoulder must push back to stay still.
- **`C`, the swinging terms** (centrifugal and Coriolis). Spin the base and the arm is flung
  outward; the joints must resist. These grow with speed squared and only matter when moving
  fast.
- **`g(q)`, gravity.** The torque needed just to hold the pose. Usually the largest term on a
  small geared arm.
- **Friction** in the gearboxes, plus the motor's own back-EMF damping from lesson 1.

**Two directions of the same equation:**

| | Given | Find | Used for |
|---|---|---|---|
| Inverse dynamics | The motion you want | The torques needed | Planning, feed-forward |
| Forward dynamics | The torques applied | The motion that results | Simulation |

Inverse dynamics is cheap: plug in the numbers. That is what makes the rest of this lesson
possible, and it is where the "gravity torque" in lesson 2's feed-forward comes from.

### The problem with fixed speed limits

The simplest motion planner gives each joint a fixed top speed and acceleration. But what the arm
can do depends on its pose and load. A limit safe at full stretch is needlessly slow when folded;
a limit that is quick when folded saturates the shoulder at full stretch, and a saturated servo
just falls behind.

### The torque governor

Every control tick (say 50–100 times a second), before sending the next command:

1. **Predict** the torque each joint needs for the next small step, using the equation above.
2. **Compare** with what the motor can deliver *at its current speed* (lesson 1), keeping some
   margin.
3. **Command the fastest motion that fits.**

**Worked example.** The shoulder is holding 1.2 N·m against gravity while moving upward at
2 rad/s.

```
budget:      90 % of stall                 = 2.86 N·m
gravity:                                    − 1.20
back-EMF:    0.53 × 2                      − 1.07
left over for accelerating                 = 0.59 N·m
```

If the arm's inertia about the shoulder in this pose is 0.03 kg·m² (an illustrative figure), the
most it can accelerate is 0.59 ÷ 0.03 ≈ 20 rad/s². Fold the arm in and both gravity and inertia
drop, so it can accelerate several times harder. The governor finds that automatically.

### Design choices worth understanding

- **Slow all joints by the same factor.** If only the struggling joint were slowed, the others
  would get ahead and the tip would leave its intended path.
- **Never scale back braking.** If braking would exceed the torque budget, brake anyway. A servo
  asked for too much merely saturates for a moment; a command that fails to brake sails past its
  goal.
- **Keep the command smooth.** A soft servo is a spring with a mass on it, and springs ring when
  kicked. Limiting **jerk** (how suddenly acceleration changes) avoids exciting that.
- **Don't trust the model alone.** Masses are estimates and the payload is unknown. So add a
  second guard that needs no model: if the measured arm falls more than a few degrees behind the
  command, slow down.
- **Respect the budgets from lesson 3** as well as the instantaneous limit.

### Common mistakes

- *"The servo's top speed is 340°/s, so plan at 340°/s."* At top speed there is no torque left to
  hold anything, let alone accelerate.
- *"It saturated, so the servo is broken or stuck."* It was asked for a motion outside the line in
  lesson 1.

### Check yourself

1. The 200 g, 25 cm link is 60° above horizontal. What gravity torque does it need? (cos 60° =
   0.5)
2. In the worked example, the shoulder is instead moving *down* at 2 rad/s. Is there more or less
   torque available for accelerating downward?
3. Why does stretching the arm out hurt twice?

*Answers.* (1) 0.49 × 0.5 ≈ 0.25 N·m. (2) Much more: gravity now helps the motion instead of
opposing it. (3) Gravity torque grows with distance, and inertia grows with distance squared, so
there is more to hold and more to accelerate, with the same motor.

---

## Lesson 5. Inverse kinematics

### The core idea

**Going from joint angles to hand position is a formula. Going from hand position to joint angles
is a search, which may have several answers or none.**

Kinematics is the geometry of the arm, ignoring forces.

### Forward kinematics: easy

Take a flat arm with two links of 12 cm, the first at angle `a` above horizontal and the second
bent a further `b`:

```
x = 12 × cos(a) + 12 × cos(a + b)
y = 12 × sin(a) + 12 × sin(a + b)
```

With a = 30° and b = 30°: x = 10.4 + 6.0 = **16.4 cm**, y = 6.0 + 10.4 = **16.4 cm**. One pose in,
one position out, always.

### Inverse kinematics: hard

Now ask the reverse: which `a` and `b` reach (16.4, 16.4)?

```
        ●  target              ●  target
       ╱                        ╲
      o  elbow up                o  elbow down
     ╱                          ╱
    o                          o
```

- **Two answers** here (elbow up or down). Bigger arms have more.
- **No answer** if the target is more than 24 cm away.
- **No tidy formula** for most real arms. So we search.

### The search, step by step

The key tool is the **Jacobian**: a table saying how the tip moves if each joint is nudged a
little. "A small turn of the shoulder moves the tip this far in this direction; a small turn of
the elbow moves it that far in that direction."

1. Compute where the tip is now (forward kinematics) and how far it is from the target.
2. Use the Jacobian, inverted, to find the joint nudges that move the tip toward the target.
3. Take a small step. The table is only accurate nearby, so recompute and repeat.

It is like walking downhill in fog: you can only feel the slope under your feet, so you take a
step, feel again, and repeat.

### Singularities

Stretch the two-link arm out straight. Turning either joint now moves the tip *sideways*; nothing
moves it further *outward*. The table has no entry for that direction, and inverting it asks for
enormous joint motion to achieve a tiny tip motion. Near such poses a naive solver thrashes.

**Damped least squares** is the standard cure. It solves for the step that gets close to the
target *while also keeping joint motion small*, trading a little accuracy near singular poses for
calm behaviour. A single number sets the trade.

### Counting freedoms

A full pose needs six numbers (lesson 0). An arm needs at least six joints to reach an arbitrary
position *and* orientation.

Our arm has five (the gripper doesn't count: opening the jaws doesn't move the hand). So some
poses are simply unreachable. It helps to see the arm as a flat three-joint arm standing on a
turntable:

| Joint(s) | What they control |
|---|---|
| Base pan | Which direction the arm points |
| Shoulder, elbow, wrist flex | Reach, height, and how the fingers are pitched |
| Wrist roll | Spin about the hand's own axis |

It cannot, for example, point the fingers sideways across the direction it is reaching. **With
too few joints you must decide what to give up.** For picking things off a table the sensible
choice is: demand the position, demand that the fingers point down, and let wrist roll set which
way the jaws face.

### Using the slack

If a task needs fewer numbers than the arm has joints, the leftover freedom can do something
useful *without disturbing the main task*. This is called **task priority**: satisfy the important
goal exactly, then spend what's left on a secondary one.

Example: allow the fingers to tilt up to 15° from straight down. That frees one joint's worth of
motion, which can be spent on choosing the pose that needs the least holding torque (lessons 3
and 4).

### Common mistakes

- *"IK found a solution, so the arm can do it."* IK is pure geometry. It knows nothing about
  torque, heat or collisions. The answer may be a pose the servos can't hold.
- *"The solver is buggy near full stretch."* That is a singularity; it is in the arm, not the
  code.
- Forgetting joint limits: the mathematically nearest solution may need an elbow to bend backward.

### Check yourself

1. With a = 0° and b = 90°, where is the tip of the two-link arm?
2. Why does the arm move jerkily when you ask it to reach the very edge of its workspace?
3. A seven-joint arm is asked to hold its hand perfectly still. Can the arm still move?

*Answers.* (1) x = 12 + 0 = 12 cm, y = 0 + 12 = 12 cm. (2) The edge of reach is a singularity;
small tip motions need large joint motions there. (3) Yes: it has one more joint than the six the
hand's pose uses, so the elbow can swing while the hand stays put.

---

## Lesson 6. Contact and friction simulation

### The core idea

**A physics simulator is trustworthy while things move freely and approximate the moment they
touch. Most of its cost and most of its errors are in contact.**

A simulator advances time in small **timesteps** (often 1–5 ms). Each step it does two jobs:
find what is touching, then work out the forces at each touch.

### Job 1: what is touching

**Simulators collide simplified shapes, not the shapes you see.** A robot model carries two sets
of geometry: detailed meshes for drawing, and simple shapes for collision.

The simple shapes are **convex**: a shape with no dents, where a straight line between any two
points inside stays inside. Convex shapes can be tested for overlap very quickly; dented shapes
can't. So a mesh is usually replaced by its **convex hull**, the shape you'd get by
shrink-wrapping it.

```
  real jaw (U-shaped)        its convex hull
     ┌──┐    ┌──┐              ┌──────────┐
     │  │    │  │              │██████████│
     │  └────┘  │              │██████████│
     └──────────┘              └──────────┘
   an object fits            the gap is solid:
   between the fingers       nothing can enter
```

That is the classic gripper bug: the hull fills the gap between the fingers. The cure is **convex
decomposition**: automatically cutting the shape into several convex pieces whose union keeps the
gap.

**Neighbours are ignored by default.** A link and the link it is attached to always overlap at
their shared joint, so simulators skip collisions between directly connected parts. If you bolt on
an accessory (a camera, say) that could hit a neighbouring link, you must list that pair
explicitly or the simulator will let them pass through each other.

### Job 2: the forces

**Friction is a budget set by squeeze.**

```
maximum sideways grip force = μ × squeeze force
```

`μ` (mu) is the friction coefficient: about 0.2–0.4 for smooth plastic, around 1 for rubber. Push
sideways harder than the budget and the surfaces slide.

**Worked example.** A two-finger gripper holds a 100 g object (weight 0.98 N) with μ = 0.5. Each
finger contributes μ × squeeze:

```
2 × 0.5 × squeeze ≥ 0.98   →   squeeze ≥ 0.98 N
```

Swing the arm and the object effectively gets heavier, so real grasps want two to three times
that. Rubber pads (μ ≈ 1) halve the squeeze needed, which is the cheap way to stay under the
overload threshold of lesson 3.

**Twisting needs its own friction.** With thin point-like contacts, an object held by two fingers
can pivot between them as if on a pin. Simulators have a separate torsional friction setting; a
grasp that holds weight but lets the object swing down is the sign that it is off.

**Contacts are slightly soft.** If objects were perfectly rigid, contact forces would be infinite
for an instant and the maths would break. So simulators let objects sink into each other by a
fraction of a millimetre and push back like a stiff spring. The consequences:

- Grasp forces are approximate.
- Stiffer contacts need smaller timesteps. Too large a step and things jitter, sink, or fly apart.
- Changing the timestep can change whether a grasp holds.

### Diagnosing trouble

| Symptom | Likely cause |
|---|---|
| Object can't go between the fingers | Convex hull filled the gap |
| Parts pass through each other | Collision skipped (neighbour rule) or no collision shape |
| Object slides out slowly | Too little friction or squeeze |
| Object pivots in the grasp | No torsional friction |
| Jitter or sudden explosion | Timestep too large for the contact stiffness |

### Why this matters beyond simulation

Real friction, real rubber and real object shapes never match the simulated ones. This is the
largest part of the **sim-to-real gap** for grasping. The usual defence when training in
simulation is to vary friction, mass and shape randomly so the result doesn't depend on any one
value being right.

### Check yourself

1. Is a doughnut convex? A brick? A mug?
2. With rubber pads (μ = 1), what squeeze holds a 300 g object, with a safety factor of 2?
3. A grasp holds at a 2 ms timestep but slips at 5 ms. Did the physics change?

*Answers.* (1) Only the brick. A line across a doughnut's hole or a mug's opening leaves the
material. (2) Weight = 2.94 N; minimum squeeze = 2.94 ÷ (2 × 1) = 1.47 N; with the safety factor,
about 3 N. (3) No, the approximation did. That sensitivity tells you the grasp was marginal.

---

## Lesson 7. The camera firmware pipeline

### The core idea

**A camera is a bucket brigade: each stage hands the frame to the next, each handoff has a
deadline, and one slow or stuck stage stops everything behind it.**

```
sensor ──▶ image processor ──▶ JPEG encoder ──▶ USB ──▶ PC sees a webcam
raw data    colour, exposure     compress        UVC
```

At 30 frames per second, each stage has 33 ms per frame.

### The stages

**Sensor.** A grid of light-sensitive cells, each behind a red, green or blue filter. So the raw
output is a mosaic with *one* colour per pixel, not a picture. It leaves the sensor over a fast
serial link (MIPI CSI is the common standard on small boards).

**Image signal processor (ISP).** Turns the mosaic into a picture: fills in the two missing
colours at each pixel, then adjusts brightness (exposure), colour balance (white balance) and
sharpness.

**Encoder.** Compression is forced by arithmetic:

```
1920 × 1080 pixels × 2 bytes × 30 frames/s ≈ 124 MB/s ≈ 1 Gbit/s
USB 2.0 carries at most 0.48 Gbit/s
```

Uncompressed full-HD video does not fit down a USB 2.0 cable. JPEG shrinks each frame ten to
twenty times. Chips intended for cameras have a hardware block for it, because doing it in
software would be far too slow.

**USB.** The device announces itself using **UVC** (USB Video Class), the standard "I am a webcam"
protocol. That is why it needs no driver: any PC, and any program that can open a webcam, can use
it.

### Two things that matter for robots

- **Auto-anything hurts.** Auto-exposure, auto-white-balance and autofocus change the image for
  reasons unrelated to the scene: the gripper passes in front of the lens and the whole picture
  brightens. Software that learns from images cannot tell that from a real change. Lock them when
  you can.
- **Delay adds up.** Exposure, processing, compression, USB, and decoding on the PC each take
  time. A robot steering by camera is always acting on a picture that is tens of milliseconds
  old.

### Why the firmware looks defensive

A microcontroller has no desktop operating system. It runs a small **RTOS** (real-time operating
system) that shares the processor between several little programs called **tasks**: one talks to
USB, one runs the camera, and so on. Sharing brings a classic bug.

**The race condition.** Two tasks touch the same thing at the same time:

```
USB task:     "PC wants video. Is the camera running? No. Start it."
Camera task:                 "Is the camera running? No. Start it."
                              → started twice, buffers corrupted
```

It happens one time in a thousand, depending on timing, which makes it miserable to find. The
standard defences:

- **One owner per resource.** Only the camera task may start or stop the camera. Other tasks send
  it requests. Two tasks can no longer do it at once, by construction.
- **A mutex** (a lock). A task takes the lock before touching shared data and releases it after;
  anyone else waits. This stops a task reading a frame that another is halfway through writing.
- **A timeout on every wait.** "Wait for the next frame" with no time limit becomes a permanent
  freeze with no error message the day the sensor cable works loose. With a 200 ms limit it
  becomes a reported error the code can act on.
- **Recover, don't just report.** Count consecutive bad frames; after a few dozen, tear the
  pipeline down and restart it.
- **Check for garbage.** A zero-length or failed frame is dropped, not passed on.
- **Mind the stack.** Each task gets a small fixed block of working memory (a few kilobytes).
  Overrun it and you silently corrupt a neighbour's memory.
- **Stop when nobody is watching.** If the PC stops asking for frames, shut the pipeline down to
  save heat and power.

### Common mistakes

- *"It works on my desk."* Race conditions and missing timeouts show up after hours, or after a
  cable is bumped.
- *"Auto-exposure makes the image look better."* Better to a person; less consistent to a
  program.

### Check yourself

1. Roughly how many megabytes per second is 1280 × 720 at 30 frames/s, at 2 bytes per pixel? Does
   it fit in USB 2.0 uncompressed?
2. Why does the single-owner rule prevent the double-start bug without any lock?
3. A camera freezes forever when its cable is wiggled. Which defence is missing?

*Answers.* (1) 1280 × 720 × 2 × 30 ≈ 55 MB/s ≈ 0.44 Gbit/s: just under the theoretical 0.48, but
not in practice once USB overhead is counted. (2) Only one task ever executes the start code, and
a single task does one thing at a time. (3) A timeout on the wait for a frame, plus a restart
when frames stop arriving.

---

## Putting it together: one pick, seven lessons

Follow a single "pick up that block" from start to finish.

1. **The camera** (7) delivers a frame, with exposure locked so the block looks the same every
   time. Software finds the block in the image.
2. **Inverse kinematics** (5) searches for joint angles that put the jaws over the block with the
   fingers pointing down, using any slack to choose a low-torque pose.
3. **The governor** (4) plans the move, predicting each joint's torque and keeping it inside what
   the **motors** (1) can deliver at that speed.
4. **Feed-forward** (2) shifts every command by the predicted droop and lag, so the soft servos
   arrive where intended.
5. The jaws close until they stop, then hold a small **offset** (3): a firm grip the servo can
   keep up indefinitely.
6. Whether the block actually stays in the jaws when lifted comes down to **friction and
   contact** (6), which is also where a simulation of this pick is least trustworthy.

## Glossary

| Term | Meaning |
|---|---|
| Armature (reflected inertia) | The motor rotor's inertia as felt at the joint: multiplied by gear ratio squared |
| Back-EMF | Voltage a spinning motor generates, opposing the supply; grows with speed |
| Convex | A shape with no dents; a line between any two inside points stays inside |
| Convex hull | The shrink-wrap of a shape |
| Damping | Resistance proportional to speed |
| Degree of freedom (DOF) | One independent way to move |
| Droop | Steady position error under a constant load |
| Feedback | Correcting from measured error |
| Feed-forward | Correcting in advance from a prediction |
| Forward / inverse dynamics | Torques → motion / motion → torques |
| Forward / inverse kinematics | Joint angles → tip pose / tip pose → joint angles |
| ISP | Image signal processor: turns raw sensor data into a picture |
| Jacobian | Table of how the tip moves per small change in each joint |
| Jerk | How quickly acceleration changes |
| Mutex | A lock letting one task at a time use shared data |
| No-load speed | A motor's top speed, with nothing attached |
| Pose | Position plus orientation (six numbers) |
| Race condition | A bug that depends on the timing of two tasks |
| RMS | Root mean square: the right average for heating |
| RTOS | Small operating system that shares a microcontroller between tasks |
| Saturation | Output at its maximum; more demand changes nothing |
| Singularity | A pose where the tip can't move in some direction |
| Stall torque | A motor's maximum torque, at zero speed |
| Stiffness | Torque per unit of position error |
| Timestep | The slice of time a simulator advances per step |
| Torque | Twisting effort: force × distance from the pivot |
| UVC | The standard USB webcam protocol |

---

## Appendix: where these appear in the project this was written for

Optional, and only useful if you have the `robotic-vision-project` repository.

| Lesson | Files |
|---|---|
| 1 | `sim/so101sim/servo_model.py`, `sim/config/servos.toml` |
| 2 | `CommandFeedforward` in `sim/so101sim/command.py` |
| 3 | `GripOffset` in `sim/so101sim/command.py`, `sim/docs/SERVO_LOAD_FINDINGS.md` |
| 4 | `sim/so101sim/dynamics.py`, `sim/so101sim/governor.py`, `sim/docs/FAST_TELEOP_FINDINGS.md` |
| 5 | `sim/so101sim/ik.py` |
| 6 | `sim/tools/build_model.py` |
| 7 | `firmware/p4-uvc-camera/` |
