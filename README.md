# harmony
HARMONY provides production-ready ROS 2 modules for human-robot collaboration in packaging operations. Features adaptive pick-and-place for variable SKUs, real-time quality inspection, IoT monitoring, and FIWARE integration. Developed under EU Horizon ARISE project.

├── README.md                    # Main project documentation
├── LICENSE                      # Apache 2.0 for entire project
├── .github/                     # CI/CD for all modules
│   └── workflows/
├── ai_packaging/                # Module 1
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── README.md               # Module-specific docs
│   └── ...
├── monitoring_analytics/        # Module 2
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── README.md
│   └── ...
├── harmony_common/              # Shared components
├── harmony_msgs/                # Shared messages
└── docs/                        # Project-wide documentation
