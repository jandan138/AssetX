# AssetX Documentation

Welcome to AssetX - the multi-format robot simulation asset bridge tool!

## 🚀 Getting Started

- [Project Overview](overview.md) - Learn what AssetX is and why you need it
- [Installation Guide](installation.md) - Get AssetX running on your system  
- [Quick Start Tutorial](quickstart.md) - Your first 15 minutes with AssetX

## 📚 Documentation

### User Guides
- [API Reference](api/README.md) - Complete Python API documentation
- [CLI Reference](cli-reference.md) - Command-line tool documentation
- [Best Practices](best-practices.md) - Professional workflows and tips
- [Troubleshooting](troubleshooting.md) - Common issues and solutions

### Core Architecture
- [USD Architecture Guide](usd_architecture_guide.md) - Deep dive into USD concepts and design patterns
- [AssetX-USD Mapping](assetx_usd_mapping.md) - How AssetX corresponds to USD architecture
- [Asset API Documentation](api/asset.md) - Updated modular Asset API reference

### Advanced Topics
- [Format Support](formats/README.md) - Detailed format specifications
- [Extension Development](development.md) - Contributing and extending AssetX
- [Case Studies](case-studies/README.md) - Real-world usage examples

## 🔗 Quick Links

| Resource | Description |
|----------|-------------|
| [GitHub Repository](https://github.com/jandan138/AssetX) | Source code and issues |
| [PyPI Package](https://pypi.org/project/assetx/) | Install with pip |
| [Examples](examples/README.md) | Code examples and tutorials |
| [FAQ](faq.md) | Frequently asked questions |

## 📋 Table of Contents

```
docs/
├── overview.md           # Project introduction
├── installation.md       # Setup instructions  
├── quickstart.md         # 15-minute tutorial
├── api/                  # API documentation
│   ├── README.md
│   ├── asset.md
│   ├── converter.md
│   └── ...
├── cli-reference.md      # CLI documentation
├── formats/              # Format specifications
├── case-studies/         # Usage examples
├── best-practices.md     # Professional tips
├── troubleshooting.md    # Problem solving
├── faq.md               # Common questions
└── development.md        # Contributing guide
```

## 🎯 What Can AssetX Do?

AssetX is designed to solve the format fragmentation problem in robotics simulation:

### Format Conversion
Convert between major simulation formats:
- **URDF** ↔ **MJCF** (ROS ↔ MuJoCo)
- **USD** → **Genesis** (Isaac Sim → Genesis)
- **URDF** → **USD** (ROS → Isaac Sim)

### Physics Validation
Ensure physical consistency across formats:
- Mass and inertia validation
- Geometry consistency checks
- Joint limit verification
- Cross-format comparison

### Asset Management
Organize and track your simulation assets:
- Metadata management with YAML schema
- Conversion history tracking
- Asset categorization and tagging
- Quality validation records

### Mesh Processing
Optimize 3D meshes for simulation:
- Mesh simplification for performance
- Collision geometry generation
- Unit normalization
- Geometric transformations

## 🏗️ Architecture

```
AssetX Core Architecture

┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   URDF Format   │    │   MJCF Format   │    │   USD Format    │
│   (ROS/Gazebo)  │    │    (MuJoCo)     │    │  (Isaac Sim)    │
└─────────┬───────┘    └─────────┬───────┘    └─────────┬───────┘
          │                      │                      │
          └──────────────┬───────────────┬──────────────┘
                         │               │
                    ┌────▼─────────────▼────┐
                    │                       │
                    │   AssetX Core Engine  │
                    │                       │
                    │  • Asset Parser       │
                    │  • Format Converter   │
                    │  • Physics Validator  │
                    │  • Meta Manager       │
                    │                       │
                    └────┬─────────────┬────┘
                         │             │
          ┌──────────────▼─┐       ┌───▼──────────────┐
          │  Mesh Processor │       │  Visualization   │
          │                │       │                  │
          │ • Simplification│       │ • 3D Preview     │
          │ • Collision Gen │       │ • Quality Check  │
          │ • Optimization  │       │ • Screenshots    │
          └────────────────┘       └──────────────────┘
```

## 🌟 Key Features

### ✅ Multi-Format Support
- URDF (Unified Robot Description Format)
- MJCF (MuJoCo XML Format)  
- USD (Universal Scene Description)
- Genesis JSON (Genesis Simulator)

### ✅ Physics Validation
- Automatic parameter consistency checking
- Cross-format validation
- Inertia matrix verification
- Mass property validation

### ✅ Professional Workflows
- Command-line tools for automation
- Python API for custom workflows
- Metadata tracking and history
- Batch processing capabilities

### ✅ Extensible Design
- Plugin architecture for new formats
- Custom validation rules
- Modular dependency system
- Open source development

## 📊 Supported Platforms

| Platform | Status | Notes |
|----------|--------|-------|
| Windows 10+ | ✅ Fully Supported | Primary development platform |
| Ubuntu 18.04+ | ✅ Fully Supported | CI tested |
| macOS 10.15+ | ✅ Fully Supported | CI tested |
| Python 3.8+ | ✅ Required | Type hints and modern features |

## 🤝 Community

AssetX is an open-source project. We welcome contributions!

- **Report Issues**: [GitHub Issues](https://github.com/jandan138/AssetX/issues)
- **Feature Requests**: [GitHub Discussions](https://github.com/jandan138/AssetX/discussions)
- **Contribute Code**: See [Contributing Guide](../CONTRIBUTING.md)
- **Documentation**: Help improve these docs!

## 📈 Project Status

**Current Version**: 0.1.0 (Initial Release)

**Roadmap**:
- ✅ v0.1.0: Core architecture and URDF↔MJCF conversion
- 🔄 v0.2.0: Complete USD support and Web GUI
- 📋 v1.0.0: Full format coverage and cloud integration

## 🎓 Learning Path

### Beginner (15 minutes)
1. [Installation](installation.md)
2. [Quick Start](quickstart.md)
3. Basic format conversion

### Intermediate (1 hour)
1. [API Reference](api/README.md)
2. [Physics Validation](api/validator.md)
3. [Mesh Processing](api/mesh-processor.md)

### Advanced (2+ hours)
1. [Custom Workflows](advanced/workflows.md)
2. [Extension Development](development.md)
3. [Case Studies](case-studies/README.md)

---

**Ready to get started?** Begin with the [Installation Guide](installation.md)!
