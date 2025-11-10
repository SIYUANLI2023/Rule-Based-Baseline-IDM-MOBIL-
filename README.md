# Rule-Based-Baseline-IDM-MOBIL

# A Reproducible Rule-Based Baseline for Multi-Lane Highway Interaction: Integrating IDM, MOBIL, and Hysteretic Following

## Abstract
This paper presents a reproducible and interpretable rule-based baseline for highway driving, designed to benchmark prediction-aware decision-making frameworks. The baseline combines the Intelligent Driver Model (IDM) for longitudinal control, the MOBIL criterion for lane-change decisions, and a quintic time-scaling profile for lateral motion generation. To eliminate oscillatory braking often caused by Time-to-Collision (TTC) triggers, a hysteretic car-following mechanism is introduced for the primary interacting vehicle (SV1). Specifically, SV1 performs an event-triggered acceleration when the ego vehicle (EV) initiates a lane change, and subsequently switches to a latching proportional–derivative (PD) following law that regulates spacing and relative velocity. The framework attributes vehicles undergoing lane changes to their target lanes for consistent safety evaluation and employs asynchronous longitudinal and lateral update cycles to mimic realistic driver behavior. The baseline provides a transparent reference for assessing the benefits of prediction-aware or optimization-based planning methods.

---

## 1. Introduction
Highway decision-making requires balancing safety, efficiency, and comfort under interactive uncertainty. Prediction-aware frameworks (e.g., MPC, HMDPs) promise advantages, but these gains must be judged against transparent, reproducible reactive baselines. We integrate IDM (longitudinal), MOBIL (lane-change), and a quintic lateral trajectory, and add a hysteretic following mechanism to emulate human-like responses to cut-ins. The baseline is deterministic and interpretable.

---

## 2. Problem Formulation
We study a straight three-lane highway with lane centers \(y=\{4,0,-4\}\,\mathrm{m}\). Agents: EV (middle lane, allowed to change lanes), SV1 (left-rear, interactive), SV2 (front leader on EV lane), and two trucks on the right lane (long vehicles discouraging merges). EV is discouraged/restricted from the truck lane. Fixed initial conditions and update intervals; no randomness for replicability.

---

## 3. Methodology

### 3.1 Longitudinal Control via IDM
Block equations (GitHub-friendly):

$$
a = a_{\max}\!\left[\,1-\left(\frac{v}{v_0}\right)^{\delta}-\left(\frac{s^*}{s}\right)^2\right],
\qquad
s^* = s_0 + vT + \frac{v\,(v-v_\ell)}{2\sqrt{a_{\max} b_{\mathrm{comf}}}+\varepsilon}.
$$

Here \(v\) is speed, \(s\) net headway, \(v_\ell\) leader speed. Parameters \(s_0,T,a_{\max},b_{\mathrm{comf}},v_0\) denote minimum spacing, desired headway, maximum acceleration, comfortable deceleration, desired speed. We use forward-Euler integration with acceleration saturation.

### 3.2 Lane-Change Decision via MOBIL
EV evaluates adjacent lanes every decision tick:

$$
G=(a^{L'}_{\mathrm{ego}}-a^{L}_{\mathrm{ego}})
+p\big(\Delta a_{\mathrm{f,new}}+\Delta a_{\mathrm{f,old}}\big)
-b_{\mathrm{keep}}.
$$

A lane change is admissible if \(G>a_{\mathrm{thr}}\) and both followers satisfy

$$
a^{\mathrm{after}}_{\mathrm{f,new}}\ge -b_{\mathrm{safe}},\qquad
a^{\mathrm{after}}_{\mathrm{f,old}}\ge -b_{\mathrm{safe}}.
$$

A soft penalty discourages entering the truck lane.

### 3.3 Lateral Motion via Quintic Trajectory
After approval, lateral motion uses a quintic time-scaling:

$$
s(\tau)=10\tau^3-15\tau^4+6\tau^5,\qquad
\tau=\frac{t-t_0}{T_{\mathrm{lc}}}\in[0,1],
$$

$$
y(t)=(1-s)\,y_0+s\,y_1.
$$

This guarantees continuous position/velocity/acceleration.

### 3.4 Hysteretic Following Policy for SV1
**Event-triggered acceleration.** When EV (or optionally SV2) starts a lane change, SV1 applies a temporary positive acceleration to avoid being boxed in.

**Latching PD following.** If spacing is below \(s_{\mathrm{des}}=s_0+T_f v\), SV1 enters a latched PD controller:

$$
a=K_p\,(s-s_{\mathrm{des}})+K_d\,(v_\ell-v),\qquad a\in[a_{\min},a_{\max}].
$$

Exit occurs only when the spacing error exceeds a positive threshold and the approach speed is non-closing (\(v\le v_\ell\)). Hysteresis suppresses TTC-induced on–off oscillations.

**TTC backstop (optional).** A conservative emergency brake triggers if

$$
\mathrm{TTC}=\frac{s}{\max(v-v_\ell,\varepsilon)}
$$

falls below critical bounds.

### 3.5 Leader–Follower Attribution
During lane changes, vehicles are attributed to their **target lane** for IDM/MOBIL leader–follower selection, ensuring consistent safety evaluation across the merge window.

### 3.6 Asynchronous Update Scheme
Longitudinal updates run at a fine step (e.g., \(0.15\,\mathrm{s}\)); lane-change decisions are evaluated on a slower clock (e.g., \(0.6\,\mathrm{s}\)). This mirrors faster car-following vs. slower maneuver selection.

---

## 4. Experimental Setup
Initialization: EV at \(x=35\,\mathrm{m}, v=30\,\mathrm{m/s}\); SV1 at \(x=0\,\mathrm{m}, v=25\,\mathrm{m/s}\) (left lane); SV2 at \(x=150\,\mathrm{m}\) on EV lane; trucks at \(x=120,160\,\mathrm{m}\) on the right lane at \(25\,\mathrm{m/s}\). Only EV changes lanes; SV1 uses the hysteretic policy; others cruise.

**Metrics.**  
(1) Speed/acceleration profiles (comfort).  
(2) Headway convergence to \(s_{\mathrm{des}}\) (stability).  
(3) TTC backstop activations (safety).  
(4) MOBIL gain and follower safety margins (decision quality).

Plots: speed–time with markers, space–time lane diagram, \(s\) vs \(s_{\mathrm{des}}\) with latched segments, TTC time series.

---

## 5. Results and Discussion
When EV initiates a lane change, SV1 briefly accelerates then switches to PD following if spacing is tight. Headway smoothly converges to \(s_{\mathrm{des}}\) with bounded accelerations; TTC backstop rarely triggers. Target-lane attribution removes safety discontinuities at merge boundaries. Asynchronous clocks avoid simultaneous decision conflicts and stabilize trajectories. The baseline yields human-like cooperative reactions and forms a deterministic **reactive lower bound** for prediction-aware planners.

---

## 6. Limitations
Vehicles are point-mass; only EV performs active lane changes; perception/actuation delays are omitted. These simplifications preserve interpretability and repeatability while capturing key interaction patterns.

---

## 7. Conclusion
We consolidate a transparent, deterministic rule-based baseline for multi-lane highway interaction by integrating IDM, MOBIL, a quintic lateral trajectory, and a hysteretic PD following mechanism. It provides a strong, reproducible comparator for prediction-aware decision-making frameworks and can be extended to multi-actor merges, stochastic traffic, and uncertainty-aware evaluations.

