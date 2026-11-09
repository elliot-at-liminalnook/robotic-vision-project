# Policy training: research notes and ideas (2026-11-08)

How the papers we read (and the prior work they led to) apply to training policies for **our** arm:
- a 5-DOF SO-101 derivative with a parallel gripper;
- HX-30HM position servos at 12 V (fixed-gain PD loop in firmware, overload protection);
- a side-mounted wrist camera (Pi CM3 Wide over ESP32-P4 UVC) plus two fixed scene cameras;
- Newton 1.7 (SolverMuJoCo / MuJoCo Warp 3.15) and plain MuJoCo, one consumer CUDA GPU;
- the existing classical stack: IK (strict and load-aware), torque governor, straight-line moves, realistic
  servo model, keyboard/mouse teleop.

Numbers are as the sources state them. Where a source didn't state something, I say so. "Ours/we" = this repo.
Sources are at the end.

---

## TL;DR: the recipe I'd build

1. **Policy outputs task-space commands; the classical layer executes them.**
   - The action is a small tip-target delta, a roll delta and a gripper command, at 10–15 Hz.
   - IK, the governor and the safety limits turn it into servo targets, in sim and on the real arm alike.
   - This mirrors HANDOFF (policy behind a small task-space command interface), HIL-SERL and Astra (end-effector
     targets plus IK with clipping). It keeps everything we already validated in the loop.
2. **State-based teacher first.** PPO in a massively parallel MuJoCo Warp env:
   - **Actuator:** our realistic servo model as the joint actuator, ideally as MuJoCo's native `<dcmotor>`.
     It runs on MuJoCo Warp 3.15 and mjlab, though Newton's MJCF import drops it (B1b).
   - **Resets:** OmniReset-style reset families: reach, near-object, grasped, near-goal.
   - **Start states:** DemoStart-style starts seeded from about 10 teleop demos.
   - **Reward:** sparse success plus small regularizers.
   - **Critic:** privileged.
   - **Sampling:** SGS success-guided reset sampling once there are enough reset configurations to need it.
3. **Two vision routes, run in parallel.**
   - **(a) Squint-style direct visual SAC** on a 16×16 wrist image: 91% zero-shot on a real SO-101 in prior
     art, about 15 min of training.
   - **(b) DAgger distillation** of the teacher into a wrist + scene-camera RGB student.
   - **Visual tricks for both:** color jitter, camera-pose noise, a segmentation overlay. Optionally, a
     Gaussian-splat scan of the real bench rendered by Newton's own camera (supported in our installed Newton
     1.7).
4. **Add gravity feed-forward to every position command now, and grip by offset.** ✅ *Implemented the same
   day:* realistic actuators in the model, `so101sim/command.py` used by `teleop.py` and `pick_demo.py`
   (DECISIONS P1–P4).
   - The upstream position actuators we inherited (kp 998) are about 75× stiffer than an identified STS-family
     servo, and our own realistic model's 2° default is likely too stiff as well (B0, B2).
   - At realistic softness the scripted pick fails. With the feed-forward from `dynamics.py`
     (q + g(q)/kp) it succeeds again, robust to ±20% mass error (B7).
   - Adding back-EMF velocity feed-forward also cuts fast-teleop lag from 30 to 10 mm and time from 8.6 to
     7.3 s at realistic softness (B7).
   - Commanding the gripper 3–6° past contact instead of "fully closed" removes the overload trips (B4).
5. **Start slow, then go fast.**
   - The two real SO-100/101 transfers we found both used a near-ideal simulated actuator and moved at
     ≤1 rad/s (0.45–0.75 rad/s on the real arm) (B0).
   - Get a slow policy working first; it barely depends on actuator fidelity.
   - Then buy speed with the steps below.
6. **Close the actuator gap on a bench.** First verify which register map the HX servos actually use: the
   vendor's "STS-compatible" claim and the HX register table disagree, and stock LeRobot's connect-time writes
   could land on the wrong registers (B9). Then:
   - BAM-style pendulum identification of the HX-30HM (friction model, effective gain, firmware rate limit,
     delay);
   - PACE-style chirp fit of the assembled arm.
   - Randomize around the identified values.
7. **Evaluate like SO-101 Bench.** Matched sim/real layouts, identical weights, and rank metrics, because sim
   overestimates success by 9–31 points even when it ranks policies correctly. Then do last-mile real-world RL
   if needed: **E2HiL**, a HIL-SERL successor evaluated on the SO-101 itself (F4).

Skip for now:
- differentiable-simulator gradients (MuJoCo Warp has none, and grasp contact is the worst case anyway);
- 1M-env scaling;
- ComFree-Sim;
- running MPC inside the policy;
- LLM-as-controller.

---

## A. Problem framing and action interface (decide first)

**A1. Command-space actions on top of our classical layer** *(HANDOFF, HIL-SERL, Astra, nvidia/so101-reach)*
- **HANDOFF.** One student policy sits behind a 10-D task-space command:
  - base velocity, root height, and wrist positions in the pelvis frame;
  - wrist orientation from a deterministic correction;
  - anything that emits the command can drive it, whether teleop, an agentic planner, or a VLM.
- **HIL-SERL on the SO-101.** It learns end-effector x/y/z deltas (0.02 per step) through IK. The guide says
  some tasks are "nearly impossible" in joint space.
- **Astra's agentic interface.** One absolute end-effector target, held 1–5 steps. The host runs damped
  least-squares IK with per-step clipping of 0.02 m, 0.1 rad and 0.05 rad per joint.
- **For us**, a = (Δx, Δy, Δz, Δroll, grip) at 10–15 Hz:
  - **Execution path:** `ArmIK` → `TorqueGovernor` (or `ProfileLimiter`) → servo targets.
  - **Limits enforced for free:** the camera-safe roll range (−2.74…0.80 rad), MIN_TIP_Z, joint limits and
    straight-line pacing.
  - **Same interface for teleop and policy:** keyboard demos are recorded in exactly the policy's action space.
  - **Cost:** IK per env per step on GPU. The strict solver is iterative, so a batched Warp/torch DLS version of
    `ArmIK` is needed.
    - mjlab ships a differential-IK action term (damped least squares, optional null-space posture and soft
      joint-limit avoidance) that could host our fingers-down secondary task.
    - Newton 1.7 ships `newton.ik` (installed package): `IKSolver` with position, rotation and joint-limit
      objectives, Levenberg-Marquardt / L-BFGS optimizers and samplers. "Fingers down within a cone" maps to a
      weighted rotation objective.
    - Squint instead used joint-target deltas (`pd_joint_target_delta_pos`); keep that as the fallback and
      ablation.
- **Contract file.** Write a `policy_contract.json` per trained policy (nvidia/so101-reach does): action
  semantics, scale, observation layout, rate. Refuse to load a mismatched runtime.

**A2. Control rate 10–15 Hz** *(Squint, Tune to Learn, lerobot-sim2real, OmniReset)*
- **Squint:** trains at 10 Hz for wall-clock speed, then deploys at 30 Hz with actions scaled by 0.15. That's a
  sim/real mismatch they accepted for safety. We can avoid it, because our governor shapes motion identically
  in sim and real.
- **Tune to Learn:** high-frequency jitter falls from 21.8% at 100 Hz to 5.0% at 10 Hz.
- **lerobot-sim2real** deploys at 15 Hz; **OmniReset** runs the policy at 10 Hz over a 500 Hz inner loop.
- **For us:** policy at 10–15 Hz, governor and servo sim at 500 Hz. This matches the fast-teleop finding that
  stepped goals and stop-and-go ring the servo loop.

**A3. Gains are an inductive bias** *(Tune to Learn)*
- **The paper's findings** (Franka, PD sweep over more than 2 orders of magnitude):
  - BC succeeds more with compliant, overdamped gains: block stacking 85% vs 39%. Lower imitation loss did not
    mean better rollouts.
  - RL works in every regime once its hyperparameters are tuned.
  - Sim-to-real fails most with stiff, overdamped gains, through high-frequency oscillation.
- **For us:**
  - The HX register table lists the position-loop P (0x16), D (0x17) and I (0x18) gains as read/write,
    0–32767. There is also a "minimum startup force" (0x1A), a deadband-like knob. On the bench, measure the
    closed-loop stiffness and damping each setting gives.
  - **Community evidence:** LeRobot's SO-100/101 follower driver writes **P = 16, I = 0, D = 32** to every
    servo at connect. That's half the default P. The code gives no reason; ManiSkill's SO-100 config caps
    deltas because fast moves "can cause the robot to shake."
    - For the gripper it also writes Max_Torque_Limit 500 ("50% … to avoid burnout"), Protection_Current 250
      and Overload_Torque 25.
    - Our gripper choice (L4, 50%) matches.
    - Start the HX at P = 16 / D = 32 as well (after verifying which register map it uses, B9), and
      identify it at that setting.
  - Our governor's `bandwidth_rad_s` is effectively the compliance knob in command space.
  - For BC data, prefer a softer, well-damped command profile over a "tight" one.

---

## B. Simulator and actuator fidelity

**B0. What actually transferred on SO-100/101 arms: slow and simple** *(read from their code)*

| | Squint (SO-101) | lerobot-sim2real (SO-100) |
|---|---|---|
| Arm drive in sim | stiffness 1e3, damping 1e2, force limit 100 (near-ideal tracking) | same |
| Armature, joint friction | none modeled | none modeled |
| Action | joint *target* deltas, ±0.1 rad/step (gripper ±0.2) | ±0.05 rad/step ("moving too fast can cause the robot to shake") |
| Train rate, speed | 10 Hz, ≈1.0 rad/s | 20 Hz, ≈1.0 rad/s |
| Deploy rate, speed | 30 Hz with actions ×0.15, ≈0.45 rad/s | 15 Hz, ≈0.75 rad/s |
| γ | 0.9 | 0.9 |
| Episode | 50 steps | 64 steps |
| Observation noise | 5° Gaussian on joint positions | none |
| Gripper | pad friction 2.0; drive stiffness/damping randomized | pad friction 2.0 |
| Visual randomization | colour jitter (0.3/0.3/0.3, hue 0.05); wrist camera ±2 mm/±1°/±1° FOV per step | camera ±2.5 cm, FOV ±2° per step; random cube colour |
| Reward | staged tanh reaching + grasp + return-to-rest | same; success = lifted + grasped + near rest pose |

**The dynamic counterexample: Open Duck Mini** (RL walking on STS3215 servos, MuJoCo Playground).
- **Fast, dynamic motion needed an identified servo.** Its MJCF uses BAM-exported values:
  - position kp **13.37** N·m/rad, kv 0, force range ±3.23 N·m: full torque only at ≈0.24 rad (14°) of error;
  - joint damping 0.56, frictionloss 0.068, armature 0.027;
  - target rate-limited to 5.24 rad/s (the 3400 steps/s firmware cap);
  - action delay 0–2 steps at 50 Hz, ±0.5° backlash joints, joint-position observation noise 0.03–0.08 rad.
- **On the real robot,** P register 32 and D = 0.

**Two implications for our model:**
1. **The 0.56 damping is the motor's back-EMF, kt²/R.** With BAM's STS3215 M1 fit, 1.18²/2.48 ≈ 0.56.
   - Upstream SO-101's joint damping (0.60, with friction 0.052 and armature 0.028) is the same identified
     servo.
   - This confirms the L7 correction: that damping must be zeroed when our servo model adds back-EMF itself.
2. **Upstream's position kp of 998 is about 75× stiffer than the identified 13.37.**
   - So our teleop and pick sims (ideal position actuators) track far more tightly than an STS-family servo.
   - **Where 998 comes from:** MuJoCo Menagerie's `robotstudio_so101` has the identical actuator class (kp
     998.22, kv 2.731, ±2.94 N·m; joint damping 0.60 / friction 0.052 / armature 0.028). Its XML comment
     says the gains "are not a 1-to-1 mapping of the servo gains used in Lerobot". They were calculated from a
     write-up (Gregory119/RBE501-RL-arm-project) "assuming that the servo proportional gain is set to 16". No
     system identification is cited.
   - So the community reference model doesn't fix this; it's where the stiffness comes from.
   - **Root cause.** That write-up (we have a copy from earlier in the project) models the servo's drive
     voltage as V = G_p·(180/π)·(θ_d − θ), i.e. it treats the P register (16) as **volts per degree** of
     error.
     - The 7.4 V supply would then saturate at ≈0.46° of error, giving kp ≈ 998.
     - BAM's oscilloscope-measured firmware gain says saturation happens only at ≈9–10°. That's about a 20×
       overestimate.
   - **Back-EMF is also counted twice upstream.** Its kv = K_m·K_b/R = 2.731 already *is* the motor's back-EMF.
     Upstream (and Menagerie) also put joint damping 0.60, which matches BAM's back-EMF term again (B0
     point 1).
   - **The script behind it** (`calc_actuator_model.py` in that project; we have a copy) also *assumes* the
     winding resistance: `R = 1 # Ohm`. With K_m = 2.94 N·m / 2.7 A ≈ 1.09:
     - kp = K_m·G_p·(180/π)/R = 1.09·16·57.3/1 = **998.2**, reproducing the model exactly with G_p = 16
       (the file's own default is 12);
     - kv = K_m·K_b/R ≈ 1.09·2.51/1 = **2.73**.
     - Both scale as 1/R. With a realistic R ≈ 7.6 Ω (our dcmotor check, B1b) and the measured firmware gain,
       both come out an order of magnitude smaller.
   - Upstream's actuator is therefore much stiffer *and* more damped than the real servo.
   - Grasp margins measured there (the 4 mm clearance) and droop are optimistic.
   - Until the HX bench data exists, the realistic prior is a servo that reaches full torque only at
     ≈10–14° of error. I reran the pick demo that way today: it fails without droop feed-forward and works
     with it (B7).

**Lesson:** the slow projects avoided the actuator gap by staying quasi-static (≤1 rad/s, against a 5+ rad/s
servo) and moving even slower on the real arm. The dynamic one had to identify the servo.
- **For us:** a first sim-to-real policy should do the same. Small delta commands; the governor in GENTLE or
  slower; droop feed-forward (B7).
- **Servo fidelity (B1–B3) is what we pay for speed.** Our fast-teleop work showed that's exactly where the
  servo loop rings and lags.

**B1. Train against the realistic servo model, not ideal position actuators**
- **Why:** the teleop sim uses upstream position actuators (kp 998, kv 2.7). The real HX is a saturating
  P(D) loop with back-EMF, overload protection and (probably) a rate-limited internal target.
- **What:** put `so101sim/servo_model.py` into the vectorized RL env as the joint actuator, ported to
  torch/Warp, with zero engine joint damping (the L7 correction).
- **Randomization, priors from the papers:**

| Parameter | Range | Source |
|---|---|---|
| Action delay | 1–4 ticks (20–80 ms) | nvidia/so101-reach ("a servo bus will not reliably apply a command on the tick it was issued") |
| Actuator gain scale | ×[0.7, 1.3] | nvidia/so101-reach |
| Joint friction, armature, damping | ×U(0.8, 1.2) | OmniReset |
| Motor delay | U{0, 1, 2} steps | OmniReset |
| Controller gains | ×U(0.8, 1.2) | OmniReset |
| Gripper stiffness, damping | ×logU(0.5, 2) | OmniReset |
| Gripper stiffness, damping | U(500, 2000), U(50, 200) | Squint |

- Also sweep our unknowns: `sat_error_deg` and `d_time_s`.

**B1b. Use MuJoCo's native `<dcmotor>` actuator: it runs on raw MuJoCo Warp and mjlab, not via Newton** *(tested today)*
- MuJoCo ≥ 3.7 has a `<dcmotor>` actuator. From `mjs_setToDCMotor` in the installed headers:
  - `motorconst` (K), `resistance` (R), so back-EMF via the bias, τ = K(V − Kω)/R;
  - `nominal`, `saturation`, `inductance`, `cogging`;
  - `controller` (6 values: an internal PID for position or velocity input, with anti-windup and a slew limit,
    per mjlab's docs);
  - `thermal` (I²R heating), `lugre` (LuGre friction), and an input mode.
- **Smoke test on a one-joint pendulum** (K = 1.18, R = 2.48 Ω from BAM's STS3215 fit;
  `controller="30 0 0.5 0 0 0"`; target 0.5 rad):
  - MuJoCo 3.10 (Mac) with `input="position"`: 0.518 rad after 1 s.
  - MuJoCo 3.15 (Newton's pin) with `input="pos"`: 0.509 rad. 3.15 rejects `"position"` with
    "invalid keyword: '(null)'", so the keyword differs across versions.
  - **MuJoCo Warp 3.15 steps it too:** 0.509 rad, identical to C MuJoCo. Tested on Warp's CPU device in the
    VM (no GPU there); these are the same kernels the GPU runs.
  - **But Newton 1.7's MJCF importer drops it.** `ModelBuilder.add_mjcf` then `SolverMuJoCo` gives an
    `mj_model` with `nu = 0`, because Newton rebuilds actuators from its own joint drives. Through Newton we'd
    keep a custom actuator. Newton has an experimental GPU actuator library for this (`newton.actuators`,
    attached with `ModelBuilder.add_actuator`):
    - drives: `DrivePD`, `DrivePID`, and **learned** `DriveNeuralMLP` / `DriveNeuralLSTM` / `DriveNeuralGRU`;
    - clamps: `ClampingDCMotor` (torque–speed envelope), `ClampingMaxEffort`, `ClampingPositionBased`;
    - `Delay`.
    - PID + DC-motor clamp + delay approximates the HX servo, and the neural drives can take an actuator
      network fit to bench logs (B2). The API is marked experimental.
    - Signatures read from the installed package. The composition below is a sketch; my smoke test of it
      failed (next bullet):
      ```python
      builder.add_actuator(newton.actuators.DrivePID, index=dof,       # per joint DOF
          kp=tau_s / sat_rad, ki=0.0, kd=kd_fit, integral_max=0.0,      # HX loop (fit kd on the bench)
          clamping=[(newton.actuators.ClampingDCMotor,
                     dict(saturation_effort=3.18, velocity_limit=5.96, max_motor_effort=3.18))],
          delay_steps=2)                                                # bus latency, randomize 0–4
      ```
      Missing pieces: the firmware's target slew limit (3400 steps/s) and the overload timer. Add both in our
      command layer.
    - **Smoke test failed:** single pendulum, `SolverMuJoCo` on CPU, stepping `actuator.step(state, control,
      cur, next, dt)` before each solver step.
      - `add_actuator` accepted the composition, but the joint didn't hold its 0.5 rad target. It spun to
        6.5 rad (2° loop) or −3.1 rad (9° loop).
      - Either my wiring of this experimental API is wrong (e.g. the output isn't reaching the solver, or a sign
        convention), or it isn't ready.
      - **Treat the Newton actuator path as unverified.** The native `<dcmotor>` on raw MuJoCo Warp / mjlab
        (B1b) did work.
- **So the servo model can be native, vectorized and implicit-stable,** with no Python actuator loop, if
  training runs on raw MuJoCo Warp or mjlab (`BuiltinDcMotorActuatorCfg`). This is another reason to train in
  mjlab (B5).
  - Map our `ServoSpec` onto it: K and R from stall torque and current at 12 V, the P/D controller from the
    identified loop, the slew limit = 3400 steps/s.
  - Keep only the overload timer and heat budget as custom terms.
  - Validate against `servo_model.py` on the fast-teleop click task before switching.
  - I haven't checked how its controller gains map to the HX register units; that's what the bench fit (B2)
    is for.
- **`<dcmotor>` attributes** (MuJoCo docs, read from a summary; verify against the reference):
  - `nominal="V τ_stall ω_noload"`;
  - `saturation="τ I dI/dt"`;
  - `controller="kp ki kd slewmax Imax Vmax"`, with τ = kp·e + kd·ė + ki·∫e + ff, an integrator clamp
    (Imax) and a drive-voltage clamp (Vmax);
  - `input` is any subset of `pos vel ff voltage`. The old keywords `position` / `velocity` were renamed in
    3.12, which explains our 3.10 vs 3.15 difference.
  - MuJoCo stable is now 4.0.0 (6 Nov 2026), with dcmotor parameter-handling fixes.
- **The HX datasheet is inconsistent as an ideal DC motor.**
  - 11.1 V, 2.94 N·m stall and 5.51 rad/s no-load give K = V/ω₀ ≈ 2.01 and R = K·V/τ ≈ 7.6 Ω, so a stall
    current of ≈1.5 A.
  - The datasheet says 3 A. Reconcile with a gearbox efficiency of ≈2.94/(2.01·3) ≈ **0.49** (1:345 spur
    train): output torque ≈ 0.49·K·I.
  - Our `servo_model.py` already uses the 3 A figure for current. Measure stall current and torque on the
    bench, and model the gear loss explicitly (`saturation`, frictionloss, or BAM's load-dependent friction)
    rather than folding it into K.
  - **Confirmed by compiling** `<dcmotor nominal="12 3.18 5.96"/>` in MuJoCo 3.15: it derives R = 7.598 Ω and
    K = 2.0134 (stall 3.18 N·m at 12 V, implied stall current 1.58 A).
    - Its back-EMF damping K²/R = 0.5336 is exactly our `servo_model` constant τ_s/ω₀.
    - So the native motor reproduces our torque–speed line. Only the current/gear-loss model and the P/D loop
      need adding.

**B2. Bench-identify the HX-30HM with BAM's method** *(BAM, "Extended Friction Models", Rhoban)*
- **Bench:** a pendulum on the servo horn; masses 0.5–1.5 kg at 0.10–0.20 m (0.5 kg at 0.15 m ≈ 25% of HX
  stall).
- **Trajectories (6 s each):** `sin(t²)`, lift-and-drop (torque off), up-and-down with a plateau, and
  `sin_sin`. Sweep the P register over kp/6…kp. About 100 logs per actuator.
- **Fit:** CMA-ES (Optuna) on position MAE, 75/25 split. Under 5 min and about 4000 iterations per model.
- **Friction models** M1 (Coulomb-viscous, what MuJoCo implements) through M6 (Stribeck, load-dependent,
  directional, quadratic). The best model beats Coulomb-viscous by 1.5–2.9× MAE on the actuators in the paper.
- **The STS3215 parameters BAM ships are our best prior** (the SO-101's stock servo, same Feetech protocol):
  - 7.4 V; firmware `kp = 32`, `error_gain = 0.166` (oscilloscope-measured);
  - fitted `kt ≈ 1.18–1.28`, `R ≈ 2.5–2.75 Ω`, armature ≈ 0.022–0.026 kg·m², command delay 1–5 ms;
  - an internal target rate-limited to about 5.2 rad/s.
- **Derived from those numbers (worth acting on):** PWM saturates only at about 9–10° of position error.
  - Our servo model assumes full voltage at **2°**. If the HX behaves like the STS3215, the real arm is about
    4–5× softer than our default: more droop and lag, and a lower usable governor bandwidth.
  - Until we measure, treat `sat_error_deg ≈ 9–14` as the plausible prior. The fast-teleop click task was rerun
    at 9° today (below); `servo_load_study.py` still needs the same rerun.
- **Checked today (`fast_teleop_study` click task, softer servo):**

| Servo stiffness | Gentle profile | Governor |
|---|---|---|
| Full voltage at 2° | 8.50 s, 7 mm lag, 4.2 mm droop | 6.97 s |
| Full voltage at 9° | 9.49 s, 28 mm lag, 16.6 mm droop | 8.60 s, 30 mm lag, 15.9 mm droop |

  - No trips in any run, and the governor stays faster.
  - **But 16 mm of droop is 4× our 4 mm grasp clearance**, so droop compensation (B7) becomes essential if the
    servo is that soft.
- **Evidence the HX shares the STS3215's firmware lineage:**
  - The HX's run-speed register (0x2E) and Hiwonder's tool cap speed at **3400 steps/s**. That's exactly the
    STS3215 internal-target rate limit BAM identified (3400·2π/4096 ≈ 5.2 rad/s).
  - BAM's STS3215 model *structure* (P-limited duty, rate-limited target, DC motor) is therefore a strong
    prior. Its *values* still need our own bench.
- **Integration:** BAM ships a vectorized mjlab actuator (`BamActuatorCfg`, mjlab 1.3), with delay and voltage
  ranges for domain randomization.
- **Caveats:**
  - BAM's Feetech controller is P-only, but the HX has D = 32, so add a D term.
  - Its bundled STS3215 parameters are at 7.4 V; ours run at 12 V.
  - Temperature isn't modeled.

**B3. Whole-arm system identification with PACE / OmniReset's procedure**
- **Fixed base,** joint-position chirps tracked by the servos, replayed identically in sim.
  - OmniReset: 8 s chirp at 0.1–3.0 Hz logged at 500 Hz; CMA-ES with 512 envs × 200 iterations; target under 2°
    RMSE per joint (from about 7° with defaults).
- **PACE's parameter set** per joint is armature, viscous damping, Coulomb friction and a position bias, plus a
  global delay: 4n + 1 = 21 parameters for our 5 joints. Gains are assumed known.
- **Don't co-fit gains and armature/damping:** PACE Eq. 7 shows they are not uniquely identifiable together.
- **Validate the closed loop, not just the fit.** Tune to Learn: the gains that minimize model error can
  still transfer worse.
- **Tooling we already have:** MuJoCo ships a system-identification toolbox, `mujoco.sysid`. It's present in
  our Mac install (3.10); the import only needs `pip install colorama`.
  - `Parameter` / `ParameterDict`;
  - model modifiers including `apply_pgain`, `apply_dgain`, `apply_pdgain` and a physically consistent
    `apply_body_inertia`;
  - residuals built from rollouts of logged trajectories (`build_residual_fn`, `model_residual`, custom
    rollout hooks);
  - `optimize`, plus `calculate_intervals` for confidence intervals on the fitted parameters;
  - signal modifiers, `save_results` and an HTML-style `report`.
  - With `mujoco.rollout` (fast batched rollouts) and `mujoco.minimize` (least squares) alongside, this covers
    B2/B3 without porting BAM or PACE. Use BAM's friction-model *forms* and PACE's parameter *set* as the
    model; use `mujoco.sysid` to fit them. (Found today by inspecting the package; I haven't run it.)
  - **Version caveat:** in the VM's MuJoCo 3.15 (Newton's pin), `import mujoco.sysid` succeeds but the
    namespace is empty, so the API moved or was split out after 3.10. Use the Mac's 3.10 for identification
    (it's offline work anyway), or check the current MuJoCo docs for where it lives now.

**B4. Gripper and contact settings**
- **Pad friction:**
  - SGS: OmniReset's Robotiq pad friction of μ = 100 was "a significant source of sim-to-real gap" (implausible
    grasps succeed). μ = 2 was the lowest that trained stably.
  - **Ours is μ = 1.0 against objects at 0.4 (max-combine),** realistic for TPU pads. Keep it, and randomize
    around it rather than inflating it.
- **Ranges in the literature disagree a lot.** Squint randomizes object friction U(0.1, 0.5); OmniReset uses
  static U(1.0, 2.0) for the held object. So measure ours (a tilt test of PLA on TPU) and randomize ±50%.
- **Already learned here:**
  - a 4 mm grasp clearance is needed;
  - the gripper torque limit is 50%;
  - a sustained squeeze trips overload protection, but the grasp still holds.
- **Model the HX gripper's squeeze** with the same servo model (torque limit, overload).
- **Grip by position offset, not "fully closed"** *(tested today)*.
  - Realistic servo model, 12 V, loop at 9°, arm feed-forward on. The jaw contacts the 30 mm cube at +0.198
    rad.

| Gripper command | Pick | Gripper peak (of rated stall) | Overload trips |
|---|---|---|---|
| fully closed (−0.17 rad, current behaviour) | ✅ | 108% | 1 |
| contact − 3° | ✅ | 18% | 0 |
| contact − 6° | ✅ | 40% | 0 |
| contact − 10° | ✅ | 71% | 0 |

  - Squeeze force = loop stiffness × offset, so it's bounded by design. Thermal load and the 80%-for-2-s trip
    disappear.
  - Also checked at the other softness priors with a 4° offset: 2° → peak 48%, 14° → 19%. Both picked, no
    trips (fully closed: 108% and a trip each).
  - Because squeeze scales with stiffness, **set the offset from the identified stiffness** (B2), aiming for
    ≈30–50% of stall. Randomize the stiffness in training so a learned gripper policy isn't tuned to one
    value.
  - **On the real arm, no object model is needed:** close until the gripper's present speed (register 0x3A)
    drops or its load (0x3C) rises, read its present position (0x38), then set the target 3–6° past it.
  - **For learned policies,** make the gripper action a continuous target (or "close with squeeze level k"),
    not a binary open/close, so the policy can learn this too.
- **Squint's real failures:** its worst sim-to-real gap was can-stacking, from "tipping and insufficient
  gripper friction." Keep cylinder tasks in the test set.

**B5. Simulator choice: mjlab for RL, Newton for cameras and splats (decide)**
- **mjlab** (Berkeley; Apache-2.0; "Isaac Lab API, powered by MuJoCo-Warp"):
  - manager-based environments; domain randomization events on bodies, joints, geoms, cameras and lights;
  - physically consistent inertia randomization;
  - actuator tiers including a DC motor, a learned MLP actuator and an actuation-delay wrapper;
  - a GPU camera sensor, CUDA-graph stepping and RSL-RL.
  - Its own manipulation example is only YAM cube lifting.
- **Its ecosystem is where the AMBER papers live:** MPC-RL's code, PAC-MAN and HANDOFF were all trained in
  mjlab. MPC-RL trained 15k iterations in under 6 h on one RTX 5090. BAM integrates with it.
- **Our MJCF ports as an mjlab Entity** (read from the repo; mjlab 1.6):
  - `EntityCfg(spec_fn=lambda: mujoco.MjSpec.from_file(...))`, with an `articulation` actuator config, and
    `cameras` / `CameraSensorCfg` for the wrist and scene cameras (`parent_body=` for body-mounted ones).
  - **Gotcha:** collisions follow a `CollisionCfg` policy, and geoms its name patterns don't match get
    collision **disabled silently**. Our CoACD hulls and pad boxes must match.
  - `geom_size` randomization works only on primitives, not meshes.
- **The actuator we need is nearly built in:** `BuiltinDcMotorActuatorCfg`, MuJoCo's native `<dcmotor>`.
  - τ = K(V − Kω)/R, i.e. back-EMF;
  - a POSITION input mode with an internal PID, anti-windup and slew limit (= the firmware rate limit);
  - optional I²R thermal model, cogging and LuGre friction.
  - The overload timer isn't built in. Write it as a reward/termination term, or subclass the explicit
    `IdealPdActuator` (its `compute(cmd)` hook gets per-env state).
  - Randomization for the dc-motor parameters needs a custom event term: mjlab's `dr.pd_gains` and
    `dr.effort_limits` cover only position, XML and ideal-PD actuators.
  - A per-actuator delay is built in (`delay_min_lag` / `delay_max_lag`, in physics steps).
- **Its YAM arm lift example is a useful template:**
  - 200 Hz physics, 50 Hz policy (decimation 4), 20 s episodes;
  - `nconmax=55, njmax=600`, elliptic cones, `impratio=10` (our settings too);
  - staged reach/lift reward plus action-rate, joint-limit and joint-velocity penalties;
  - a ground-contact termination; fingertip friction randomized U(0.3, 1.5);
  - PPO (512, 256, 128), 5000 iterations.
  - Its vision variant uses a **32×32** camera with a spatial-softmax CNN, 3000 iterations.
- **No official mjlab SO-101 task** turned up (search, 2026-11-08).
  - A community census says an "so-frame" project registered an SO-101 pick-place task (unverified, maybe not
    on mjlab).
  - MuJoCo Menagerie now carries an SO-101 model, `robotstudio_so101`. Compare its actuator kp, forcerange,
    damping and armature with our upstream-derived model (B0: upstream kp 998 vs identified ≈13) before
    porting.
- **Versions:** mjlab 1.6 pins `mujoco` / `mujoco-warp ~= 3.16`; Newton 1.7 pins 3.15. Use separate virtual
  environments, and keep the MJCF compatible with both. Training needs Linux + NVIDIA (macOS is
  evaluation-only).
- **Newton keeps** the ray-traced `SensorCamera` (OpenCV lens rays, segmentation output and Gaussian splats; see
  D4) and the teleop/eval backends we've validated.
- **My recommendation:**
  - Prototype the state-based teacher in mjlab: fastest path, batteries included. Today's tests strengthen
    this:
    - the native `<dcmotor>` servo works on MuJoCo Warp, but Newton's MJCF importer drops it (B1b);
    - my smoke test of Newton's experimental actuator library failed (B1b).
    - mjlab exposes `<dcmotor>` directly (`BuiltinDcMotorActuatorCfg`) plus a delay wrapper and a
      differential-IK action term.
  - Keep one MJCF as the source of truth for both.
  - Pick the vision-training renderer after a throughput benchmark (D1).

**B6. Things not to spend on (yet)**
- **Differentiable simulation:**
  - The MuJoCo Warp README says differentiability via Warp "is not yet available."
  - Suh & Tedrake (ICML 2022): first-order gradients are biased near discontinuities (stick/slip, make/break
    contact) and high-variance under stiff contact. A grasp is both.
  - Follow-ups (SHAC, AHAC) show gains only on locomotion.
- **ComFree-Sim:**
  - Analytic contact; 2–3× faster than MuJoCo Warp; +27 points mean success on LEAP-hand in-hand MPC.
  - But no parallel-gripper validation, a different engine, and no stated code release. Watch it.
- **Risk-aware domain randomization** (Esteban et al.) applies to sampling-based MPC, not RL training.
  - Its finding is still worth knowing: optimistic (min-cost) aggregation over randomized models *widens*
    basins of attraction in contact-rich tasks. Pessimistic aggregation stalls.
  - Relevant only if we add a sampling planner, e.g. for pushing objects into graspable poses.

**B7. Droop compensation: three options, cheapest first**

**Tested today** (scratch scripts, not committed). The scripted `pick_demo` on the MuJoCo backend, with the arm's
position kp lowered from upstream's 998 toward the identified-servo value:

| Arm position kp [N·m/rad] | Plain | + gravity feed-forward |
|---|---|---|
| 998 (upstream) | ✅ lifted 0.104 m | — |
| 40 | ✅ 0.103 m | — |
| 16 (≈ Open Duck's 13.4, scaled to our 12 V torque) | ❌ cube never lifted, tip 0.106 m away | ✅ 0.103 m |
| 10 | — | ✅ 0.100 m |
| 16, true link masses ×1.2 (model not told) | — | ✅ 0.103 m |
| 16, true link masses ×0.8 | — | ✅ 0.103 m |

**Same test on our realistic servo model** (`servo_load_study.run_task("pick")`, 12 V, gentle profile, wrist
camera mounted; feed-forward Δq = τ_g(q)/τ_s · sat_err):

| Servo loop: full voltage at | Plain | + gravity feed-forward |
|---|---|---|
| 2° (our current default) | ✅ lifted 0.103 m, droop 2.7 mm | — |
| 9° (STS3215-like, BAM) | ❌ not lifted, droop 9.1 mm | ✅ 0.103 m, droop 1.5 mm |
| 14° (Open Duck's identified value) | ❌ not lifted, droop 13.7 mm | ✅ 0.103 m, droop 1.7 mm |

The gripper's sustained-squeeze overload trip appears in the passing runs, as in the original load study, and
the grasp holds.

**Both fixes together** (gravity + back-EMF feed-forward on the arm, gripper 4° past contact; same realistic
model):

| Servo loop at | Pick | Droop | Max tip error during the pick | Gripper peak | Trips |
|---|---|---|---|---|---|
| 9° | ✅ 0.102 m | 1.5 mm | 9.7 mm (gravity-only FF: 21.8) | 26% | none |
| 14° | ✅ 0.102 m | 2.0 mm | 14.0 mm (gravity-only FF: 31.8) | 19% | none |

✅ **Implemented:** `so101sim/command.py` (`CommandFeedforward`, `GripOffset`), used by `teleop.py` and
`pick_demo.py`. The generated model now uses the realistic loop (kp 20.2, kv 0, back-EMF damping). With the
fixes, `pick_demo` succeeds on MuJoCo and Newton; with `--no-feedforward` it fails on both.

So our pick only worked before because our assumed servo was unrealistically stiff, both upstream's ideal kp
and our 2° default. One line of feed-forward from `dynamics.py` fixes it, robust to ±20% mass error. **Put it into
`teleop.py`, `pick_demo.py` and the RL action pipeline before anything else.**

**And on the fast-teleop click task** (`fast_teleop_study`, governor, 12 V, servo loop at 9°; 9 moves):

| Feed-forward added to the servo target | Time | Droop at targets | Max tip lag | Trips / table contact |
|---|---|---|---|---|
| none | 8.60 s | 15.9 mm | 30.2 mm | 0 / 0 |
| gravity: τ_g(q)·sat/τ_s | 8.19 s | 1.4 mm | 21.5 mm | 0 / 0 |
| full inverse dynamics (governor's τ_pred, assumes 50 g payload) | 8.26 s | 3.5 mm | 20.3 mm | 0 / 0 |
| **gravity + back-EMF: (τ_g(q) + k·q̇_cmd)·sat/τ_s, k = τ_s/ω₀** | **7.31 s** | **1.2 mm** | **10.3 mm** | 0 / 0 |
| (reference: stiff 2° servo, no feed-forward) | 6.97 s | 4.2 mm | 7.0 mm | 0 / 0 |

- **Why back-EMF matters more than inertia:** a P-only servo needs position error just to supply the voltage
  that sustains a speed (error ≈ k·v/kp). Feeding that forward removes most of the speed-dependent lag.
- With it, a servo 4.5× softer performs close to the stiff one.
- Both terms need only the identified stiffness and the motor's torque–speed constants. That's another reason
  the bench identification (B2) pays off.
- **Sensitivity to getting the stiffness wrong** (feed-forward assumes 9°):
  - true 14°: 9.3 mm droop, 20 mm lag;
  - true 5°: over-compensated, 6.6 mm offset, 11.6 mm lag, shoulder peak 73%;
  - no trips or table contact in either.
  - A ±50% stiffness error leaves 7–9 mm, about 2× our 4 mm grasp clearance. So identify the stiffness to
    roughly ±20%, randomize it in training, and close the last millimetres with the servo's I gain or vision
    (options 2 and 3 below).

1. **Feed-forward position offset (recommended form: gravity + back-EMF).**
   - Command q + Δq with Δq = (τ_g(q) + k·q̇_cmd) · sat_err/τ_s. τ_g(q) is the gravity + payload torque
     `so101sim/dynamics.py` already predicts; q̇_cmd is the governor's commanded velocity.
   - Same idea as Tune to Learn's torque-to-position retargeting, q_des = q + K_p⁻¹(τ + K_d q̇).
   - I found no SO-100/101 project doing follower feed-forward. LeRobot's "gravity compensation" is a low-torque
     mode for moving the *leader* arm by hand, and calibration only stores homing offsets. Every SO-101 policy
     we read is trained and deployed with the droop uncorrected.
   - Needs only the identified stiffness. It also makes demonstrations and policies gain-agnostic.
2. **The servo's own integral term.**
   - The HX register table has a writable position-loop I gain (0x18; default 0, as the load study assumed).
   - Removes steady-state droop on the servo itself. Test for oscillation and windup against the overload timer.
   - **Simulated today** (click task, governor, servo at 9°; integral clamped at ±5° of equivalent error):

| Fix | Time | Droop | Max lag | Trips / table |
|---|---|---|---|---|
| integral only, Ti = 0.5 s | 10.23 s | 5.1 mm | 21.6 mm | 0 / 0 |
| integral only, Ti = 0.15 s | 9.01 s | 2.2 mm | 20.9 mm | 0 / 0 |
| feed-forward (assumes 9°), true 14°, + integral Ti = 0.3 s | 8.64 s | 3.2 mm | 21.5 mm | 0 / 0 |
| feed-forward (assumes 9°), true 5°, + integral Ti = 0.3 s | 9.08 s | 2.4 mm | 7.3 mm | 0 / 0 |

   - An integral alone fixes droop but settles slower than feed-forward.
   - **Feed-forward + a small integral keeps the offset within the 4 mm grasp clearance** even with the
     stiffness ±50% wrong, at some cost in speed.
   - **Recommended:** feed-forward always. Add the integral (or enable it only in the final approach / grasp
     phase) once the real servo's stability with I > 0 is checked on the bench.
3. **Close the loop through vision.** The wrist camera sees jaws and object, which is what the learned
   policies do anyway.

**B8. Optional: run the position loop on the host**
- The HX has an open-loop PWM mode (control-mode register 0x21 = 2).
- A host controller could then send duty directly, running PD plus gravity feed-forward plus the governor's
  torque limits at bus rate.
- That's exactly the DC-motor model BAM simulates. It would give the policy a much better-known actuator.
- **Costs:**
  - bus bandwidth: 6 servos at a few hundred Hz is untested on our ESP32/USB path;
  - losing the firmware's position safety. Check which protections (overload, over-current 0x1F/0x26,
    temperature) still apply in PWM mode.
- Defer until B2 shows the stock loop is a problem.

**B9. ⚠ Verify the HX register map before writing any gain or limit register**

Two of our sources disagree.
- **`third_party/hiwonder-so-arm101/README.md`:** the HX is "protocol-compatible with Feetech STS". Hiwonder's
  LeRobot fork only changes sync read/write retries, so stock LeRobot drives it as `sts3215` and writes the
  **STS3215 control table**:
  - Max_Torque_Limit 16, P 21, D 22, I 23;
  - Protection_Current 28, Overload_Torque 36;
  - Torque_Enable 40, Goal_Position 42, Present_Position 56.
- **The HX-30HM register table I cited in this doc** (firmware 2.43, table version 3.7):
  - 16 (0x10) = under-voltage threshold; 19 (0x13) = max torque limit;
  - 21 (0x15) = LED alarm conditions; 22/23/24 (0x16–0x18) = P/D/I;
  - 40 (0x28) = target position (simple write);
  - only Present_Position at 56 (0x38) agrees.

If the second map is what our servos run, **stock LeRobot's connect-time writes would be wrong**:
- "P = 16" would land in the LED-alarm register;
- "D = 32" in the P gain;
- the gripper's "Max_Torque_Limit = 500" in the under-voltage threshold (50 V, which would trip
  under-voltage protection);
- "Torque_Enable = 1" in a goal position.

Possibly the two documents describe different firmware or protocol modes.
- The vendor's `HX-30HM_Communication_Protocol_v1.0.pdf` (text extracted today) uses Feetech's packet framing
  and instruction set: 0xFF 0xFF header, PING/READ/WRITE/REG WRITE/ACTION/RESET, SYNC READ 0x82 and
  SYNC WRITE 0x83.
- It agrees with both maps on ID at address 5 and present position at 0x38.
- But it defers the control table to an "HX-35HM memory table" that isn't included, so it can't settle this.

**Before writing anything:**
1. Read the model number (addresses 3–4) and firmware version (0–1).
2. Dump a range of registers and compare against both maps.
3. Change a harmless register first.

Every register citation in this doc (B2, B7, B8, A3) assumes the HX table; treat them as unverified until
then.

---

## C. Exploration, curriculum, rewards

**C1. OmniReset-style reset families**
- **The four families, adapted to our arm:**
  1. **Reaching:** object anywhere, tip anywhere reachable. Validate each sample with IK: 5 DOF can't reach
     arbitrary 6-D end-effector poses, and the camera-safe roll range applies.
  2. **Near-object:** tip at a precomputed grasp with a small offset, gripper randomly open or closed.
  3. **Grasped:** object in the air, already held.
  4. **Near-goal:** object at or near its placed pose, gripper in contact.
- **Grasp sampler:** we already have the pieces (`pick_demo.solve_grasp`, jaw-land geometry, 4 mm clearance).
  Sample antipodal grasps per object over yaw, keeping the cube's 4-fold symmetry. Reject by collision check plus
  a few physics settle steps.
- **What OmniReset reports:**
  - "Emergent curriculum": PPO solves near-goal states first, then the whole workspace. No hand-made
    curriculum needed.
  - SGS's ablation: without the near-goal family, rod-in-hole fails outright (0%). With all families,
    96–100%.
- **Use symmetry-aware resets and success checks** (any equivalent orientation counts). SGS's Table 7 (peg
  insertion, OmniReset setup) compares a 55% pose-based policy with a 98% symmetric point-cloud *teacher*. That
  bundles symmetry handling with an observation change, so it isn't a pure symmetry ablation.

**C2. DemoStart from our teleop demos (cheapest high-value curriculum)**
- **Store full simulator state at every timestep of a few teleop demos.** Our teleop already runs the sim,
  and states save and restore.
- **Per actor:**
  - sample one target-distribution start ψ₀ plus K = 8 demo states, one from each eighth of a demo, ordered
    hard to easy;
  - run T = 4 probe episodes per start;
  - walk the sequence: if every probe fails, move to the next (easier) start; if every probe succeeds, or the
    sequence runs out, resample a new sequence; keep the first start whose outcomes are mixed (zero-variance
    filtering);
  - run M = 50 episodes from it into the replay buffer.
- **Demo actions are never trained on,** so keyboard demos in a different action space are fine.
- **Results:**
  - Plug insertion: vanilla RL and demo-starts-alone 0% → with zero-variance filtering 97.2% → full method
    99.6%.
  - Demo counts: 2 (cube reorientation) to 60 (nut-and-bolt). 20 sim demos ≈ 30 min of teleop.

**C3. SGS: success-guided reset sampling, once reset sets get large**
- **Mechanics:** per-configuration ring buffer of the last H = 100 outcomes, p_i = mean.
  - Kernel: w_i = (p_i+ε)^{κt}·(1−p_i+ε)^{κ(1−t)}.
  - Softmax over log w with temperature T = 2.
  - Manipulation settings: κ = 1, t = 0.5, ε = 1e-4. κ = 0 is uniform sampling, a free ablation.
  - About 50 lines on top of PPO.
- **Honest scale caveat:**
  - SGS's manipulation wins are at 64K–1M envs. Franka nut-and-bolt was 0% for *every* sampler at 4K; rod-in-hole
    needs 32K.
  - On easier insertion tasks at 64K, uniform sampling was already about 95%.
  - At our 4–16K envs and pick-place difficulty, uniform sampling over good reset families may be enough.
- **If used, keep the configuration count small** (2–8K) so buffers fill, and pick a prior for empty buffers
  (the paper doesn't say). It combines naturally with C2 (same "train where success is mixed" principle).

**C4. Rewards: sparse success plus regularizers, with our safety terms**
- **SGS's shared manipulation reward:**
  - λ_succ = 100·1[success], λ_fail = 1;
  - action 1e-5, action-rate 1e-4, joint velocity 1e-4, each clipped by C(x) = min(x, 1e4);
  - no per-task shaping beyond a success predicate.
- **OmniReset:** success 1.0; distance 0.1; reach 0.1 (tanh); smoothness 1e-4/1e-3/1e-2; abnormal-state
  penalty 100 with termination.
- **Squint's staged dense reward** (reach 2(1−tanh 5d); grasp +3; place; release +7; success +9; table touch
  −6) trains in minutes at 1024 envs. Use staged dense rewards when not using rich resets.
- **Our additions:**
  - penalty or termination on table contact and on wrist-camera-assembly contact (it has collision geometry);
  - a penalty on actual servo torque above the overload threshold (74% of stall at 12 V), plus the governor's
    heat estimate;
  - for a "held object" success check, require a 5 s hold (SO-101 Bench's rule).
- **Success definitions should accept symmetric poses.**
  - SGS: nut-and-bolt initially failed on the real arm by repeatedly dropping and flipping the nut. They traced
    it to the reward counting only one upright nut orientation as success.

**C5. Model-guided rewards from our own planner** *(PLANC, CLF-RL, MPC-RL)*
- **We have a reference generator already:** straight-line tip moves plus IK plus the scripted pick phases in
  `pick_demo`.
- **Three ways to use it at training time only:**
  1. **CLF tracking reward (CLF-RL):**
     - r_v = 10·exp(−V/σ), V = ηᵀPη with η = [y − y_d; ẏ − ẏ_d];
     - P from a continuous-time algebraic Riccati equation of the double integrator;
     - plus a decay penalty −2·clip((V̇ + λV)/σ̇, 0, 1), with V̇ by finite differences.
     - On the G1: 5–24% lower tracking error than tracking-only.
  2. **MPC-RL "landmark" reward:** reward the realized tip position for agreeing with what earlier plans
     predicted for this time, weights tapered {0.5, 0.25, 0.15, 0.07, 0.03}.
  3. **Planner reference to the critic only** (privileged), the actor never.
- **Why it helps:** PLANC's end-to-end baseline got 0% on the hardest terrain against 97% with the reference,
  and fell into "standing still" local optima without the distillation prior.
- **Our tasks are easier than stepping stones.** Expect the main win in smoothness, speed and robustness, not
  feasibility.
- **Use time-normalized phases** (PLANC's s = t/T with closed-form per-segment durations) for
  approach / descend / close / lift / place.

**C6. Teacher → student, the way that worked for them**
- **Privileged critic everywhere** (HANDOFF, MPC-RL, OmniReset, SGS). For us it sees:
  - object pose and velocity;
  - domain-randomization parameters;
  - planner references;
  - servo internal state (overload timer, true torque, position-loop error).
- **Distillation:**
  - SGS: *online* DAgger, one step across all envs per update, data discarded, 6.67% of envs held out (held-out
    success lags early).
  - PLANC: three stages: teacher → DAgger warm start → noisy PPO fine-tune. A plain asymmetric actor-critic
    "failed to converge" on their task.
- **If we have two teachers** (e.g., a classical free-space mover and an RL grasp policy), HANDOFF's split-KL
  distillation weights each teacher only on the action slice and context it's good at.

---

## D. Vision

**D1. Squint recipe: direct visual RL in minutes (strongest prior art for this arm)**
- **Result:** 8 SO-101 tasks in ManiSkill3; 96.1% sim and **73/80 = 91.3% zero-shot real** with a *single
  wrist camera*; 15 min on one RTX 3090.
- **Algorithm:**
  - SAC with a C51 distributional critic and the average of two critics (not clipped double-Q);
  - LayerNorm everywhere; a 2-layer CNN updated only by the critic;
  - 1024 envs, 256 updates per iteration (update-to-data ratio 0.25), batch 512;
  - 1M FIFO replay on the GPU;
  - `torch.compile`, CUDA graphs and bf16 (more than 5× faster).
- **"Squint":** render 128×128, area-downsample to **16×16**.
  - 32×32 transferred equally well (73/80).
  - Below 16×16 overfits and doesn't transfer.
  - uint8 16×16×3 is 768 B per frame, so 1M frames ≈ 768 MB.
- **Domain randomization:**
  - wrist camera pose noise ±2 mm and ±1° *every step*; FOV ±1° (my two readings of their code disagree on
    whether FOV is resampled per step or once per sensor);
  - ambient light U(0.2, 0.5);
  - **color jitter** (73 → 58/80 without it);
  - joint observation noise σ = 5°; object friction U(0.1, 0.5);
  - the background replaced by a fixed overlay image via segmentation.
- **Their stated weak spots:** brittle to lighting changes, and can stacking.
- **For us:** our wrist camera is side-mounted with a different FOV. Copy the method, not their camera numbers.
- **First, benchmark our renderer.** Our installed MuJoCo Warp 3.15 (Newton 1.7's pin) already has the batch
  renderer (`create_render_context` / `render` / `get_rgb`) and the `mjwarp-testspeed` tool. The tool is GPU-only
  ("testspeed available for gpu only" in the CPU VM), so run this on the Linux PC:
  - add `resolution="16 16" output="rgb"` to the wrist `<camera>` in a copy of `sim/model/so101_follower.xml`;
  - run `mjwarp-testspeed <copy>.xml --function=render --nworld=1024`, and again with `--function=step`;
  - the difference is the render cost.
  - Time Newton `SensorCamera` (or MuJoCo Warp's renderer) at 1024 worlds × 16×16 (and 128×128 → pool), color
    only.
  - The MuJoCo Warp docs say render cost scales with pixel count, camera count and mesh complexity. Put
    primitives or convex hulls in the rendered geom groups, not visual meshes.
  - No published throughput exists for either at this size. Playground's Madrona path got about 37K env steps/s
    with 64×64 pixels on a 4090 for Panda pick-cube; there the policy update, not rendering, dominated (57%).

**D2. Distilled RGB student (route b)**
- **Architecture:** ResNet-18 per camera (OmniReset and SGS), ImageNet-pretrained, trained end-to-end (frozen
  encoders degraded performance). Plus joint positions, tip pose and previous action; 5-frame history.
- **Distillation:**
  - SGS uses online DAgger.
  - OmniReset used BC on 80K teacher rollouts. Sim success saturated by 10K, but *real* transfer kept
    improving with more data.
- **Real results to calibrate expectations:**

| Source | Task | Real success |
|---|---|---|
| OmniReset | peg | 85% |
| OmniReset | furniture leg | 56% |
| OmniReset | drawer | 15% |
| SGS (UR5e) | gear | 94% |
| SGS (UR5e) | rod | 61% |
| SGS (UR5e) | nut | 37.5% |

  - Policies that "grasp cleanly and insert directly" transferred. Contact-exploiting tricks didn't.
- **Keep the wrist camera.** DemoStart's ablation (real plug insertion; read from a summary of the paper's
  Table IV, not verbatim): removing the wrist camera dropped real success from 64% to 17%. Also:
  - 3 cameras: 51%; 2 cameras: 42%;
  - no photoreal training data: 29%.

**D3. Background and appearance matching for the fixed cameras** *(lerobot-sim2real)*
- **The procedure:**
  - hand-align each scene camera to its sim twin with a live overlay (arrow keys adjust FOV);
  - capture the real background with the robot removed, crop it to sim resolution;
  - composite: robot and objects from sim, the rest from the photo;
  - randomize robot color.
- **We get the mask for free:** Newton's `SensorCamera` outputs a per-pixel `shape_index` image.
- **Our cameras are calibrated OpenCV models** (`cameras.toml`), and the Newton backend already traces
  distorted rays. After a checkerboard calibration, randomize k1/k2 slightly.
- **ESP32-P4 pipeline augmentations:**
  - JPEG compression artifacts: the cameras stream MJPEG;
  - auto-exposure brightness jitter;
  - frame drops or latency, matching the measured USB/UVC path.

**D4. Gaussian-splat the real bench into Newton** *(SplatSim, GS-Playground, Newton)*
- **Already in our installed Newton 1.7:**
  - `Gaussian.create_from_ply()` loads a splat;
  - `ModelBuilder.add_shape_gaussian(body=…, gaussian=…, collision_proxy=…)` attaches it to the world
    (`body=-1`) *or to any link*;
  - `SensorCamera` renders splats with meshes through the same ray tracer (`GaussianRenderMode.FAST` or
    `QUALITY`);
  - Newton's own camera example mounts a robot camera on a link.
- **So the moving wrist camera can see a photoreal static bench.** Green-screen compositing can't do that
  for a moving camera.
- **Capture (SplatSim):**
  - a 1–2 min phone video, diffuse lighting, posters on blank walls;
  - COLMAP, then 3DGS for 30k iterations;
  - keep the robot out of the scene, or segment it per link with ICP to the URDF;
  - SplatSim's robot renders: PSNR 22.6 dB, SSIM 0.78.
- **Evidence:**
  - SplatSim: BC trained only on splat renders got 86% real success (97.5% when trained on real data). Without
    image augmentation it was 21%.
  - GS-Playground (a different engine): the AIRBOT arm's PickCube went **18/20 real zero-shot**, vs 0/20 for
    MuJoCo Playground, ManiSkill3 and Isaac Lab renders of the same task.
- **Caveats:**
  - Splats bake in the lighting, so rely on color jitter instead.
  - Pin `open3d==0.19.0` (a Newton 1.7 known issue).
  - Render speed for splats in `SensorCamera` is unmeasured.

**D5. Visual randomization checklist** *(OmniReset Table 2, condensed)*
- **Camera pose:** third-person ±5 cm / ±2°, focal ±2; wrist ±1 cm / ±1°, focal ±1.
- **Textures and lights:** texture vs flat color 50/50; roughness, metallic and specular U(0,1); environment
  lighting (920 HDRIs) at random intensity.
- **Image augmentations:**
  - color jitter: brightness, contrast, saturation 0.1; hue 0.05;
  - Gaussian blur, kernel 5, σ U(0.1, 2);
  - grayscale with p = 0.05; pixel noise std 0.01.
- **Not covered by any source:** a side-mounted wide wrist camera that sees its own jaws. Our camera mount
  study already ensures the jaws stay in view, which is useful for active vision.

**D6. Route (c): estimate object poses, then run the state policy (cheapest real transfer for known objects)**
- **Skip pixels-to-actions.** Run the *state-based* teacher on hardware, fed by a perception module:
  - start with colour segmentation plus our calibrated fixed cameras; `vision_check.py` already projects
    and finds the red cube within about 1 px;
  - move to fiducials, or a learned 6-DoF pose estimator, for harder objects.
- **Precedents:**
  - MuJoCo Playground's LEAP hand ran a state policy on a vision pose estimator;
  - HANDOFF's VLM grounding reached 1.2–1.6 cm;
  - PAC-MAN models perception explicitly in sim: masked depth, dropout, jitter, edge flicker.
- **To make it robust,** train the teacher with the perception module's error model:
  - noise, dropout, latency (OmniReset: 40 ms end-to-end was fine, 60 ms caused jitter);
  - occlusion by the gripper.
  - Here PAC-MAN's lesson (E2) applies directly.
- **Measure our camera chain's latency first** (IMX708 → ESP32-P4 JPEG → UVC → host decode), then randomize
  observation delay around it.
- **Off-the-shelf pose estimators fit our RGB-only cameras poorly:**
  - FoundationPose needs RGB-D (≈32 Hz tracking, ≈1.3 s initial estimate on an RTX 3090).
  - MegaPose has an RGB-only refiner (≈50 ms per iteration, so ≈20 Hz tracking) but needs a CAD model plus a
    2-D detection, and wasn't evaluated on 2–3 cm objects.
  - For our small known objects, start with colour segmentation or fiducials from the calibrated fixed cameras,
    plus a mask tracker. Try MegaPose only for objects that need full 6-DoF pose.

---

## E. Safety in training and deployment

**E1. CBF-RL: safety filter during training, plus our runtime layer as the deploy-time filter**
- **During training,** project the proposed command onto a half-space with a closed-form expression, no QP
  solver needed:
  - v_safe = v + max(0, b − a·v)·a/‖a‖², with a = ∇h and b = −α·h;
  - add r_cbf = 100·(min(a·v − b, 0) + exp(−‖v − v_safe‖²/0.25) − 1).
- **Barriers for us:**
  - tip and gripper-link height above the table;
  - wrist-camera-assembly clearance to the table, objects and shoulder;
  - joint limits;
  - the camera-safe roll range.
- **Results, navigation success with dynamics randomization:**
  - training filter + reward, no runtime filter: 91.7%;
  - with the runtime filter: 99.0%;
  - reward only: 87.6%;
  - filter only, without the runtime filter: 36.8%.
- **Train with both, deploy with both.** Our IK limits, MIN_TIP_Z and straight-line pacing are already that
  runtime filter.
- **Apply the barrier to the command, not the measured state.**
  - CBF-RL assumes the commanded velocity is realized; our servos lag.
  - Define h on the commanded target, as PAC-MAN does: v_des = (q_des − q_t)/Δt.

**E2. Match barrier strength to what the robot can perceive** *(PAC-MAN)*
- **PAC-MAN's finding:** a strong whole-body barrier used only as a reward *hurt* when the policy couldn't see
  the threat (fixed camera: 86% → 76%). It helped when perception was good (gimbal or oracle) or when kept as a
  runtime filter (90%).
- **For us:**
  - **Strong barriers** for things known from encoders and geometry: table, self-collision, joint limits.
  - **Soft per-link penalties** for things seen through noisy cameras: objects, people.

**E3. Servo protection is a time-integrated constraint**
- **Neither CBF paper handles this.** Overload trips at 80% for 2 s; heat is RMS over minutes.
- **Our governor already tracks both.** Add the overload timer and heat estimate to the privileged critic
  state, penalize above threshold, and keep the governor as the runtime enforcement.

---

## F. Data, imitation, VLAs, real-world RL

**F1. Teleop is our data engine**
- **Add a recorder to `teleop.py`:**
  - full sim state per step, for DemoStart resets;
  - command-space actions and camera frames, for BC;
  - LeRobotDataset-compatible output for SmolVLA, ACT and HIL-SERL.
- **Collect at a softer command profile for BC** (A3).

**F1b. Sim-and-real co-training for BC/VLA** *(Maddukuri et al., 2025)*
- **Recipe:** loss = α·L_sim + (1−α)·L_real, with α the per-batch probability of drawing sim data.
  - α of 0.9–0.99 sim worked best; 1:1 was suboptimal.
  - On one task 99% was best, and 99.5–99.9% dropped success from 95% to 60%. Don't starve the real data.
- **Sim data are "digital cousins," not a twin:** same robot, action space, task goal and object/fixture
  *categories*, generated in bulk with MimicGen from about 10–100 source demos.
- **Results** (summary-level; per-task values unverified):
  - real-only 45% average → +sim 81–83%;
  - help continued up to 400 real demos.
- **What mattered most was camera pose:** an unaligned sim camera cost 11–25 points; physics tuning made no
  difference on the task tested.
- **For us:** generate thousands of sim demos from our scripted planner and teleop (MimicGen-style object-frame
  retargeting is easy with our IK). Match the wrist-camera mount and the scene-camera extrinsics to the real
  ones (already in `cameras.toml` / `wrist_mount_pose.json`). Then co-train with 20–50 real demos.

**F2. Residual RL on a base policy** *(ResiP, Policy Decorator)*
- **ResiP:**
  - frozen diffusion-policy base trained on 50 demos, plus a PPO residual (scale 0.1) on [state, base action];
  - one_leg 54% → 98%, round_table 12% → 94%;
  - plain BC plateaus around 56% even with 10K demos.
- **Policy Decorator:**
  - SAC residual on the observation only, bounded α·tanh, base frozen;
  - a progressive ε schedule from base-only to base + residual over 30K–8M steps;
  - "near-perfect" in simulation, no real-world experiments.
- **Our natural base is the classical scripted pick** (`pick_demo` planner through IK and the governor). A
  small Policy-Decorator residual would learn to correct droop, lag and pose noise. Cheap, interpretable and
  safe.

**F3. VLA for language-conditioned tasks later** *(SmolVLA, SO-101 Bench)*
- **SmolVLA (450M; fits one GPU; 30 fps, 50-step action chunks):**
  - SO-100: 78% average across 3 tasks with 50 demos per task (π0 3.5B: 62%; ACT: 48%);
  - SO-101 pick-place: 90% in distribution;
  - without its community pretraining: 52%;
  - async inference completes 30% faster.
- **SO-101 Bench:**
  - policies trained only on simulated demos kept their **exact ranking** on the real arm (Spearman 1.00,
    45 real trials each);
  - but sim overstated success by 9–31 points.
  - Its best system was an LLM agent driving a learned skill policy: 64% real vs 53% for the skill policy alone.
- **RL on VLAs** (SimpleVLA-RL, RL4VLA) gives big sim gains but needs 7B models and A100s; no low-cost arm
  evidence. Not for us yet.

**F4. HIL-SERL for last-mile real-world learning**
- **SO-101 hackathon:**
  - from scratch on the real arm, a working pick policy in about 40 min;
  - the key was tight end-effector workspace bounds;
  - sim pretrain plus 4 h of real fine-tuning also worked; direct zero-shot failed (shadows, camera mount).
- **Setup:** SAC with prior data; ResNet-10 success classifier at 128×128; a few demos; human takeover by
  keyboard or gamepad. Our teleop keyboard maps directly.
- **Use after sim-to-real,** to fix the residual gap.
- **E2HiL** (arXiv 2601.19969, Jan 2026) improves on HIL-SERL *on the SO-101 itself*. It changes RLPD's
  objective to prune "shortcut" and noisy samples by their estimated influence on policy entropy.
  - Four real tasks: touch cube, pick cube, pick & place, stack blocks.
  - Project page: +42.1% average success vs HIL-SERL with 10.1% fewer human interventions on the SO-101 (the
    paper text says a 45% relative gain). Per-task SO-101 numbers are only in a figure I couldn't read.
  - On their other arm (A1_X, 6 tasks; table read): HIL-SERL 66.7% → 84.2% average, intervention rate
    59.4% → 50.0%.
  - A community leaderboard lists real SO-101 pick-cube at 93.2% for E2HiL vs 40% for HIL-SERL (unverified).
  - **The method:** mask actor-loss samples outside the 5th–90th percentile band of |influence on entropy|
    (influence ≈ −η·Cov(log π, π·A_soft), with K = 8 sampled actions). The critic stays unmasked; the base is
    RLPD (half demo, half replay per batch).
  - The project page says SO-101 tasks trained in "about one hour" on average.
  - Code: a LeRobot/SO-101 repo is linked from e2hil.github.io (`E2HiL/E2HiL-project-lerobo`). Published in RA-L
    2026. If we do real-world RL, start from it rather than vanilla HIL-SERL.

---

## G. LLM layer: orchestration, not control

- **Astra (GPT-6 Astra on simulated DROID):**
  - 74.8% on RoboLab-120 with the agentic interface, ahead of every robot foundation model listed;
  - but 13–18 s per call (the paper's full range is 11.5–21.7 s), with inference 95–98% of episode time, and
    about $36 per episode;
  - adding gripper and camera geometry to the prompt took the direct mode from 19% to 73%.
- **Project Fetch (Opus 4.7):** at least 10× faster than the human teams on robodog integration, but couldn't
  close the fast visual loop (nudging a ball).
- **HANDOFF and SO-101 Bench** both put the language/agent layer at 0.001–0.1 Hz over a fast learned skill
  policy.
- **For us,** an LLM agent should:
  - issue straight-line move / grasp / place commands through the teleop API at about 0.1 Hz;
  - call learned skills for the closed loop;
  - get the camera intrinsics/extrinsics and gripper geometry in its prompt (Astra's biggest single gain).
- **Also useful offline:** writing reward and success predicates, reset generators and domain-randomization
  configs.
  - There is published precedent: DrEureka has an LLM write a safety-regularized reward (Eureka). It then
    probes the trained policy across physics settings and has the LLM set randomization ranges from that
    "reward-aware physics prior."
  - It beat manually designed rewards and randomization on real quadruped and cube-rotation tasks.
  - Its ablations: the physics prior was essential; random sampling within the same ranges was worse.
  - I only saw the project page and secondary summaries, not the paper's numbers.

---

## H. Evaluation protocol

- **Sim vs real** (SO-101 Bench):
  - reproduce each sim layout on the real table by projecting object footprints into the overhead camera;
  - identical weights in both domains; alternate policies in blocks;
  - check camera drift against a reference frame before each episode (≤ 2.6 px, ≤ 7% brightness);
  - success must hold 5 s; "touch" = any link over 0.05 N;
  - report Spearman/MMRV *and* absolute success.
- **Real continuous evaluation** (OmniReset, SGS):
  - log first-try success and throughput (successes per minute);
  - stuck detection: joints move under 0.002 rad for 2 s → open the gripper for 1 s.
- **Checkpoint choice:**
  - pick by sim success under action noise (OmniReset's robustness evaluation);
  - beware overtraining (lerobot-sim2real);
  - deploy the best seed, as Squint did.
- **Wall-time realism:** SO-101 Bench's agent paused the sim clock while thinking. Its real median rollout was
  197 s vs 38 s for the learned policy.

---

## Suggested order of work

| Phase | Work | Needs hardware? |
|---|---|---|
| 0 | Benchmark camera rendering at 1024 worlds (D1). Pick mjlab vs Newton for RL (B5). Batched GPU IK + governor + `<dcmotor>` servo as the env's action pipeline (A1, B1, B1b). | No |
| 1 | Add gravity + back-EMF feed-forward and grip-by-offset to `teleop.py` / `pick_demo.py` / the action pipeline (B7, B4). State-based pick-place teacher: reset families + DemoStart from about 10 teleop demos + sparse reward + privileged critic + CBF-in-training (C1, C2, C4, E1). | No |
| 2 | Vision: Squint-style 16×16 wrist SAC (D1) and a DAgger-distilled wrist + scene student (D2) with the D5 randomization; splat or overlay background (D3, D4). | No |
| 3 | Servo bench: BAM-style identification of one HX-30HM, then PACE-style chirps on the arm (B2, B3); re-tune randomization around the fit; rerun all studies. | Yes |
| 4 | Real evaluation with the SO-101 Bench protocol (H); HIL-SERL last mile (F4); residual on the scripted pick (F2). | Yes |
| 5 | Language: SmolVLA on sim + real demos (F3); LLM orchestration over skills (G). | Yes |

---

## Sources

Core papers:
- [Walk the PLANC](https://arxiv.org/abs/2601.06286) · [code](https://github.com/Zolkin1/robot_rl/tree/mlip)
- [SGS: A Balanced Data Diet](https://arxiv.org/abs/2610.12465) · [project](https://sgs-rl.github.io/)
- [MPC-Guided RL](https://arxiv.org/abs/2606.05687) · [code](https://github.com/junhengl/mpc-rl)
- [HANDOFF](https://arxiv.org/abs/2606.06493)
- [CBF-RL](https://arxiv.org/abs/2510.14959)
- [PAC-MAN](https://arxiv.org/abs/2607.28623)
- [Risk-aware DR for SPC](https://arxiv.org/abs/2605.03290)
- [Astranomy](https://research.nvidia.com/labs/srl/projects/astranomy/)
- [Project Fetch: Phase Two](https://www.anthropic.com/news/project-fetch-phase-two)
- [mjlab](https://arxiv.org/abs/2601.22074) · [repo](https://github.com/mujocolab/mjlab) ·
  [SO-101 sim landscape notes](https://mcobzarenco-fontaine-blog.static.hf.space/papers/so101-sim-landscape.html)
- [ComFree-Sim](https://arxiv.org/abs/2603.12185)
- [Suh et al., Do Differentiable Simulators Give Better Policy Gradients?](https://arxiv.org/abs/2202.00817)
- [SHAC](https://arxiv.org/abs/2204.07137), [AHAC](https://arxiv.org/abs/2405.17784)

Adjacent work found while researching:
- [Squint](https://arxiv.org/abs/2602.21203) · [code](https://github.com/aalmuzairee/squint)
- [lerobot-sim2real](https://github.com/StoneT2000/lerobot-sim2real) ·
  [ManiSkill sim2real docs](https://maniskill.readthedocs.io/en/latest/user_guide/tutorials/sim2real/index.html)
- [nvidia/so101-reach](https://huggingface.co/nvidia/so101-reach)
- [SO-101 sim ecosystem notes](https://mcobzarenco-fontaine-blog.static.hf.space/papers/so101-sim-ecosystem.html)
- [OmniReset](https://arxiv.org/abs/2603.15789) · [project](https://weirdlabuw.github.io/omnireset) ·
  [UWLab](https://github.com/UW-Lab/UWLab)
- [BAM: Extended Friction Models for Servo Actuators](https://arxiv.org/abs/2410.08650) ·
  [docs](https://bam.readthedocs.io/)
- [PACE](https://arxiv.org/abs/2509.06342) · [repo](https://github.com/leggedrobotics/pace-sim2real)
- [CLF-RL](https://arxiv.org/abs/2508.09354)
- [DemoStart](https://arxiv.org/abs/2409.06613)
- [E2HiL](https://arxiv.org/abs/2601.19969)
- [HIL-SERL on SO-101](https://trunc8.github.io/projects/hil-serl-lerobot) ·
  [LeRobot HIL-SERL guide](https://huggingface.co/docs/lerobot/hilserl)
- [SO-101 Bench](https://arxiv.org/abs/2610.36420)
- [Tune to Learn](https://arxiv.org/abs/2604.02523)
- [MuJoCo Playground](https://arxiv.org/abs/2502.08844)
- [MuJoCo Warp](https://github.com/google-deepmind/mujoco_warp) ·
  [renderer discussion #1337](https://github.com/google-deepmind/mujoco_warp/discussions/1337)
- [Newton SensorCamera](https://newton-physics.github.io/newton/stable/api/_generated/newton.sensors.SensorCamera.html)
- [ManiSkill3](https://arxiv.org/abs/2410.00425)
- [SplatSim](https://arxiv.org/abs/2409.10161)
- [GS-Playground](https://arxiv.org/abs/2604.25459)
- [SmolVLA](https://arxiv.org/abs/2506.01844)
- [Sim-and-Real Co-Training](https://arxiv.org/abs/2503.24361)
- [FoundationPose](https://arxiv.org/abs/2312.08344), [MegaPose](https://arxiv.org/abs/2212.06870)
- [ACT](https://arxiv.org/abs/2304.13705)
- [ResiP](https://arxiv.org/abs/2407.16677)
- [Policy Decorator](https://arxiv.org/abs/2412.13630)
- [SimpleVLA-RL](https://arxiv.org/abs/2509.09674)
- [RL4VLA](https://arxiv.org/abs/2505.19789)
- [DrEureka](https://eureka-research.github.io/dr-eureka)
- [LeRobot SO follower driver](https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/so_follower/so_follower.py) ·
  [config](https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/so_follower/config_so_follower.py)
- [Open Duck Mini sim2real notes](https://github.com/apirrone/Open_Duck_Mini/blob/v2/docs/sim2real.md) ·
  [Open Duck Playground](https://github.com/apirrone/Open_Duck_Playground) ·
  [runtime](https://github.com/apirrone/Open_Duck_Mini_Runtime)
- HX-30HM register table and manual: `docs/datasheets/hiwonder-hx-servos/` (registers 0x16–0x18 P/I gains,
  0x21 control mode, 0x2E run speed, 0x3C load, 0x45 current)
