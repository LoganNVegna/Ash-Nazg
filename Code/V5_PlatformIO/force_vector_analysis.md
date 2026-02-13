# Force Vector Sum Over One Rotation (Move Toward 0°)

## Model
- **World frame:** 0° = forward (direction we want to go). Angles in degrees; unit vector at angle θ is (cos θ°, sin θ°).
- **Force ∝ throttle:** Assume net thrust magnitude ∝ (max(L,R) − min(L,R)) during the pulse (differential), and **thrust direction = bot forward** (standard diff drive: both wheels positive ⇒ forward).
- **Pulse 1:** angle 0°–54°. Motor: L ramps S→0, R ramps S→1000 ⇒ end state (0, 1000). **Bot forward = world θ.** Thrust = forward ⇒ **world direction = θ.**
- **Pulse 2:** angle 180°–234°. Motor: R ramps S→0, L ramps S→1000 ⇒ end state (1000, 0). **Bot forward = world θ (180°–234°).** Thrust = forward ⇒ **world direction = θ.**

So in both pulses we thrust in **bot forward**, i.e. **world θ**. So:
- Pulse 1: thrust directions **0°–54°**
- Pulse 2: thrust directions **180°–234°**

Those are **opposite** in world frame ⇒ the two pulses point **opposite** ways.

---

## Discrete sum (every 15° through each pulse)

Use magnitude = 1000 × (position through pulse), 0 at pulse start and 1000 at pulse end.  
Unit vector at θ: **(cos θ°, sin θ°)** with θ in radians for cos/sin.

### Pulse 1 (θ = 0, 15, 30, 45, 54°); direction = θ

| θ° | p=θ/54 | mag | cos θ | sin θ | F_x | F_y |
|----|--------|-----|-------|-------|-----|-----|
| 0  | 0.00   | 0   | 1.000 | 0.000 | 0   | 0   |
| 15 | 0.28   | 278 | 0.966 | 0.259 | 268 | 72  |
| 30 | 0.56   | 556 | 0.866 | 0.500 | 481 | 278 |
| 45 | 0.83   | 833 | 0.707 | 0.707 | 589 | 589 |
| 54 | 1.00   | 1000| 0.588 | 0.809 | 588 | 809 |

**Sum pulse 1:** F1 = (1926, 1748) → direction ≈ **42°**, magnitude ≈ 2600.

---

### Pulse 2 (θ = 180, 195, 210, 225, 234°); direction = θ (same convention: thrust = bot forward)

| θ°  | p=(θ-180)/54 | mag | cos θ  | sin θ  | F_x   | F_y   |
|-----|--------------|-----|--------|--------|-------|-------|
| 180 | 0.00         | 0   | -1.000 |  0.000 | 0     | 0     |
| 195 | 0.28         | 278 | -0.966 | -0.259 | -268  | -72   |
| 210 | 0.56         | 556 | -0.866 | -0.500 | -481  | -278  |
| 225 | 0.83         | 833 | -0.707 | -0.707 | -589  | -589  |
| 234 | 1.00         | 1000| -0.588 | -0.809 | -588  | -809  |

**Sum pulse 2:** F2 = (-1926, -1748) → direction ≈ **222°** ( = 42° + 180° ).

---

## Net force over one rotation

**F_net = F1 + F2 = (1926 − 1926, 1748 − 1748) = (0, 0).**

So **yes: if thrust is 1:1 with throttle and thrust direction is “bot forward” for both pulses, the theoretical force vectors add up to zero net force.**

---

## Why that happens

- Pulse 1: we thrust when the bot points **0°–54°** ⇒ thrust vectors in the **0°–54°** direction.
- Pulse 2: we thrust when the bot points **180°–234°** and we still thrust in **bot forward** ⇒ thrust vectors in the **180°–234°** direction.
- 180°–234° is the **opposite** of 0°–54°, so the two pulse contributions are equal in magnitude and opposite in direction ⇒ they cancel.

So with the current logic (both pulses = “forward” in bot frame), there is **no net translation** in this model. To get net motion toward 0°, one of the two pulses would need to produce thrust in the **bot backward** direction (so that when the bot is at 180°, we thrust at **0°** in world, not 180°).
