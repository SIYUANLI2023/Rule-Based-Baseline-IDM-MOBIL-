# A Reproducible Rule-Based Baseline for Multi-Lane Highway Interaction

This repository provides a reproducible and interpretable rule-based baseline for highway driving, designed to compare the decision-making philosophy of conventional rule-based methods against the proposed prediction-aware framework (Simulation Case 2). It integrates the Intelligent Driver Model (IDM) for longitudinal control, the MOBIL criterion for lane-change decisions, and a quintic time-scaling profile for lateral motion generation. To mitigate oscillatory braking behavior often induced by Time-to-Collision (TTC) triggers, a hysteretic car-following mechanism is incorporated for the primary interacting vehicle (SV1). Specifically, SV1 performs an event-triggered acceleration when the ego vehicle (EV) initiates a lane change, and subsequently switches to a latching proportional–derivative (PD) following law that regulates inter-vehicle spacing and relative velocity. The framework attributes vehicles undergoing lane changes to their target lanes for consistent safety evaluation and employs asynchronous longitudinal and lateral update cycles to emulate realistic driver behavior. Overall, this baseline serves as a transparent benchmark for assessing the differences between conventional rule-based approaches and the proposed HMDP-MPC decision-making framework.

Technical details are in **IDM-MOBIL.pdf**. Run `main.m` to reproduce all figures and results.

---

## 🧩 File Structure
```
📂 results/                              → Output figures and .mat data (auto-generated)
├── README.md                            → This document
├── main.m                               → Main entry script (run this)
├── run_case2_baseline_IDM_MOBIL.m       → Core baseline simulation (IDM + MOBIL + hysteretic follow)
├── playback_lanechange_from_baseline.m  → Export playback GIF
├── plot_velocity_from_baseline.m        → Plot speed profiles
├── plot_traj_compressed_from_baseline.m → Plot compressed x–y trajectory
└── IDM-MOBIL.pdf                        → Technical details and model description
```

---

## 🚀 Usage
1. Open MATLAB and navigate to this folder.  
2. Run:
   ```matlab
   main
   ```
3. The following outputs will be saved automatically under `./results/`:
   - `baseline_case2.mat` → simulation data  
   - `velocity_case2.png` → EV and SV speed profiles  
   - `trajectory_case2.png` → compressed x–y trajectory  
   - `playback_case2.gif` → optional playback animation (enable in `main.m`)

---

## 🧠 Scenario Summary
- **Ego Vehicle (EV):** IDM longitudinal + MOBIL lane-change  
- **SV1:** event-triggered acceleration + hysteretic PD following  
- **Other vehicles:** constant-speed cruising  
- **Asynchronous updates:** separate longitudinal and lateral cycles

---

## 📚 Citation


---

## 👤 Author
**Siyuan Li**  
Loughborough University, UK  
