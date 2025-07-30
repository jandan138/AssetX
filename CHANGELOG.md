# Changelog

All notable changes to AssetX will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Initial project structure and core modules
- Multi-format asset support (URDF, MJCF, USD, Genesis JSON)
- Format conversion capabilities
- Physics parameter validation
- Mesh processing utilities
- Metadata management system
- Asset preview and visualization
- Command-line interface
- Comprehensive test suite
- Documentation and examples

### Features
- **FormatConverter**: URDF ↔ MJCF conversion support
- **PhysicsValidator**: Mass, inertia, and geometry validation
- **MeshProcessor**: Simplification, centering, scaling, collision generation
- **MetaManager**: Asset metadata tracking and history
- **Previewer**: Trimesh and Open3D based visualization
- **CLI**: Complete command-line interface for all operations

## [0.1.0] - 2025-01-01

### Added
- Initial release
- Basic project structure
- Core asset representation
- Format detection and validation
- Metadata schema definition
- Example files and documentation

### Technical Details
- Python 3.8+ support
- Modular architecture design
- Type hints throughout
- Comprehensive error handling
- Logging integration
- Test coverage framework

### Dependencies
- click: CLI framework
- pyyaml: YAML processing
- numpy: Numerical operations
- trimesh: 3D mesh processing
- open3d: 3D visualization
- urdfpy: URDF file support (optional)
- mujoco: MuJoCo integration (optional)
- usd-core: USD format support (optional)

### Documentation
- README with feature overview
- Meta schema specification
- Development guide
- API documentation
- Usage examples
