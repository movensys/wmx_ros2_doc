# WMX ROS2 Documentation

Technical documentation for the **WMX ROS2** robotic manipulator control system, built on the [Movensys](https://www.movensys.com/) WMX3 motion control platform.

**Documentation Site:** [https://rkdcodms.github.io/wmx_ros2_doc/](https://rkdcodms.github.io/wmx_ros2_doc/)

## Overview

WMX ROS2 bridges the Movensys WMX3 EtherCAT-based motion controller with ROS2, enabling control of 6-DOF robotic manipulators (e.g., Dobot CR3A) with motion planning through MoveIt2 and NVIDIA Isaac cuMotion.

## Documentation Contents

| Section | Description |
|---------|-------------|
| **Getting Started** | System requirements, ROS2 installation, workspace setup, mock & physical hardware |
| **Packages** | `wmx_ros2_message` and `wmx_ros2_package` node documentation |
| **Integration** | MoveIt2, NVIDIA Isaac cuMotion, custom planner & application guides |
| **Architecture** | System design, communication flow, and sequence diagrams |
| **Performance** | Non-functional requirements and benchmarks |
| **API Reference** | ROS2 services, topics, and actions |
| **Examples** | NVIDIA Jetson Orin and Intel x86_64 setups |
| **Deployment** | Source build, Debian package, and Docker options |
| **Troubleshooting** | Common issues and solutions |

## Building Locally

### Prerequisites

- Python 3.10+

### Setup

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r doc/requirements.txt
```

### Build

```bash
cd doc
make html
```

Open `doc/_build/html/index.html` in your browser to view the documentation.

## Deployment

Documentation is automatically built and deployed to GitHub Pages via GitHub Actions on every push to `main`.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines on editing documentation, adding new pages, RST/Markdown syntax, and the Git workflow.

## License

Copyright 2026 Movensys. All rights reserved.
