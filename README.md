# RoboCup SML 2026 — Technical Documentation

Central technical documentation for Team RoboManipal's autonomous mobile manipulation platform (**KARMA**), developed for the **RoboCup Smart Manufacturing League (SML) 2026**.

The portal covers system-level mechanical engineering, embedded electronics, perception pipelines, and autonomous decision-making stacks.

---

## System Overview

The documentation is organized by functional system under [`source/`](source/):

- **[Mobile Base](source/mobile_base/)**: 4-wheel independent Mecanum chassis, spring-loaded dead-wheel odometry pods, power distribution PCB, FreeRTOS motor controller, and Nav2/SLAM navigation stack.
- **[Manipulator](source/manipulator/)**: 5-DOF articulated arm, 20:1 cycloidal gearboxes, carbon fiber composite links, joint braking mechanisms, distributed controller PCBs, and MoveIt 2 motion planning.
- **[Perception](source/perception/)**: Real-time RGB-D vision, YOLO oriented bounding box tracking, PCL point cloud filtering, and 6-DOF grasp/place pose estimation.
- **[Autonomy](source/autonomy/)**: Referee box communication, automated task planning (Unified Planning / PDDL), and mission execution.

---

## Building Locally

### Prerequisites

- Python 3.10+
- `pip`

### 1. Install Dependencies

```bash
pip install -r source/requirements.txt
```

### 2. Build HTML

On Linux / macOS:
```bash
make html
```

On Windows (PowerShell / CMD):
```powershell
.\make.bat html
```

The compiled static site is output to `build/html/`.

### 3. Preview

Start a local server to view the documentation:

```bash
python -m http.server 8000 --directory build/html
```

Then open `http://localhost:8000` in your browser.

---

## Tech Stack

- **Documentation Engine**: [Sphinx](https://www.sphinx-doc.org/)
- **Markup**: [MyST-Parser](https://myst-parser.readthedocs.io/) (Markdown + MathJax)
- **Theme**: [Read the Docs Theme](https://sphinx-rtd-theme.readthedocs.io/) (`sphinx_rtd_theme`)
- **UI Components**: [sphinx-design](https://sphinx-design.readthedocs.io/)
- **Diagrams**: [sphinxcontrib-mermaid](https://github.com/mgaitan/sphinxcontrib-mermaid)
