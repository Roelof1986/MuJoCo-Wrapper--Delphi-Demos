# MuJoCo-Wrapper Example Projects (Delphi)

Example projects demonstrating how to use the [MuJoCo-Wrapper (Delphi)](https://github.com/Roelof1986/MuJoCo-Wrapper--Delphi-)  
for real-time physics simulation, humanoid control, and neural integration.

<p align="center">
  <img src="docs/demo_humanoid.png" width="600" alt="Humanoid demo running in Delphi">
</p>

---

## ✨ Overview

This repository contains several **Delphi / Object Pascal** examples showcasing real-time integration with the **MuJoCo physics engine** through the custom **C++ DLL wrapper** (`mujoco_delphi_wrapper.dll`).

Each example demonstrates a different aspect of the wrapper API — from basic physics stepping and rendering, to multi-threaded humanoid control and live torque application.

---

## 🧩 Included examples

| Example | Description |
|----------|-------------|
| **Minimal Torso (Featherstone)** | Simple body with one hinge joint and base motor control. |
| **Two-Legs Creature** | Bipod test model with alternating joint torques and balance simulation. |
| **Humanoid Control** | Full humanoid model with random torque input on hips, knees, and ankles. |
| **Console Joint Logger** | Non-visual example showing how to read `qpos` and `qvel` values in real time. |

All examples use:
- **FMX-based rendering** (OpenGL context)
- **Native mouse camera** (orbit / pan / zoom)
- The complete `MJW_*` API from the MuJoCo-Wrapper (Delphi) DLL

---

## 🧠 Requirements

- [MuJoCo-Wrapper (Delphi)](https://github.com/Roelof1986/MuJoCo-Wrapper--Delphi-) — build this DLL first  
- [MuJoCo SDK](https://github.com/google-deepmind/mujoco) installed or extracted  
- **Delphi 10.x or newer**  
- **Windows 64-bit**  
- `mujoco_delphi_wrapper.dll` in the same folder as the EXE  

Optional (recommended):
- A GPU with OpenGL 3.3 or newer  
- FMX multi-threading enabled for smoother rendering  

---

## 🛠️ How to run the demos

1. **Build the wrapper**  
   Follow the build guide from the [main wrapper repository](https://github.com/Roelof1986/MuJoCo-Wrapper--Delphi-)  
   to generate `mujoco_delphi_wrapper.dll`.

2. **Open a demo project**  
   In Delphi, open one of the example `.dpr` or `.lpi` files (e.g. `MuJoCo_Test_Physics_V1.dpr`).

3. **Run**  
   Make sure the DLL and the MuJoCo `models\humanoid.xml` file are in the working directory.

4. **Interact**  
   - **Left mouse** → Orbit camera  
   - **Right mouse** → Pan  
   - **Scroll wheel** → Zoom  
   - (Optional) Use `MJW_SetMouseCamParams` for sensitivity tuning

---

## 📂 Repository structure

MuJoCo-Wrapper--Example-Code/ 
├─ examples/ │   
├─ MuJoCo_Test_Physics_V1/ │   
├─ Minimal_Torso/ │   
├─ Two_Legs_Creature/ │   
├─ Humanoid_Control/ 
│   └─ Console_JointLogger/ │ 
├─ examples_common/ │   
├─ MujocoWrapper.pas │   
├─ PhysicsThreadTemplate.pas 
│   └─ FMXHelpers.pas │ 
├─ models/ │   └─ humanoid.xml │ 
├─ docs/ │   └─ demo_humanoid.png │ 
├─ README.md 
└─ LICENSE.txt

---

## 🧭 Controls (FMX viewer)

| Action | Input |
|--------|--------|
| Rotate camera | Left mouse drag |
| Pan camera | Right mouse drag |
| Zoom | Mouse wheel |
| Pause / resume | F5 |
| Quit | Esc |

---

## 🧱 Building from source

You don’t need to rebuild the C++ DLL for these demos.  
Just copy the latest `mujoco_delphi_wrapper.dll` next to your Delphi project output folder.

To build the DLL from scratch, refer to the [MuJoCo-Wrapper (Delphi)](https://github.com/Roelof1986/MuJoCo-Wrapper--Delphi-) instructions (CMake + Visual Studio).

---

## 📘 Example code snippet

```pascal
procedure TForm1.OnTick(Sender: TObject);
begin
  MJW_ClearApplied(H);

  // Read current joint angle and velocity
  var angle := MJW_Joint1D_Angle(H, 'knee_left');
  var speed := MJW_Joint1D_Vel(H, 'knee_left');

  // Simple PD torque control
  var target := 0.3;
  var torque := (target - angle) * 400 - speed * 10;
  MJW_ApplyDof(H, KneeL_Dv, torque);

  MJW_Step(H);
  MJW_Render(H);
  MJW_PollEvents(H);
end;


---

🧾 License

All Delphi example code in this repository is released under the MIT License © 2025 Roelof Emmerink.
See LICENSE.txt for details.

This repository depends on the MuJoCo-Wrapper (Delphi) project and the MuJoCo SDK, both licensed separately.


---

🙌 Acknowledgements

MuJoCo — Multi-Joint dynamics with Contact

GLFW — OpenGL window and input handling

Google DeepMind for open-sourcing MuJoCo

Roelof Emmerink — author of the MuJoCo-Wrapper (Delphi)
