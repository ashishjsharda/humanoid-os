# Contributing to HumanoidOS

Thank you for your interest in contributing to HumanoidOS! This document provides guidelines and instructions for contributing.

## Code of Conduct

We are committed to providing a welcoming and inclusive environment. Please be respectful and professional in all interactions.

## How Can I Contribute?

### Reporting Bugs

Before submitting a bug report:
1. Check if the issue has already been reported
2. Verify the bug exists in the latest version
3. Collect relevant information (OS, Python version, error logs)

**Good bug reports include:**
- Clear, descriptive title
- Steps to reproduce
- Expected vs. actual behavior
- Screenshots/videos if applicable
- System information

### Suggesting Features

We welcome feature suggestions! Please:
1. Check if the feature has been suggested before
2. Explain the use case and motivation
3. Provide examples if possible
4. Consider implementation complexity

### Code Contributions

#### Development Setup

```bash
# Clone the repository
git clone https://github.com/ashishsharda/humanoid-os.git
cd humanoid-os

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Install development dependencies
pip install -e ".[dev]"
```

#### Code Style

We follow PEP 8 with some modifications:
- Line length: 100 characters (not 79)
- Use type hints for function signatures
- Use dataclasses for data structures
- Use descriptive variable names

**Formatting:**
```bash
# Format code
black humanoid-os/

# Check types
mypy humanoid-os/

# Run linter
pylint humanoid-os/
```

#### Testing

All code contributions should include tests:

```bash
# Run all tests
pytest tests/

# Run with coverage
pytest --cov=humanoid-os tests/

# Run specific test
pytest tests/test_balance.py
```

**Test guidelines:**
- Unit tests for individual functions
- Integration tests for subsystems
- Simulation tests for full scenarios
- Aim for >80% code coverage

#### Documentation

- Add docstrings to all public functions/classes
- Update relevant docs in `docs/`
- Include examples for new features
- Update README if needed

**Docstring format:**
```python
def my_function(param1: float, param2: str) -> bool:
    """
    Brief description of function.
    
    Longer description with details about behavior,
    edge cases, and usage examples.
    
    Args:
        param1: Description of param1
        param2: Description of param2
        
    Returns:
        Description of return value
        
    Raises:
        ValueError: When and why this is raised
        
    Example:
        >>> my_function(1.0, "test")
        True
    """
```

#### Pull Request Process

1. **Fork and Branch**
   ```bash
   git checkout -b feature/my-new-feature
   # or
   git checkout -b bugfix/fix-issue-123
   ```

2. **Make Changes**
   - Write clean, documented code
   - Add/update tests
   - Ensure all tests pass
   - Follow code style guidelines

3. **Commit**
   ```bash
   git add .
   git commit -m "feat: add new balance controller feature"
   ```
   
   **Commit message format:**
   - `feat:` New feature
   - `fix:` Bug fix
   - `docs:` Documentation changes
   - `test:` Test additions/changes
   - `refactor:` Code refactoring
   - `perf:` Performance improvements
   - `chore:` Maintenance tasks

4. **Push and Create PR**
   ```bash
   git push origin feature/my-new-feature
   ```
   
   Then create a pull request on GitHub.

5. **PR Description Should Include:**
   - What changes were made and why
   - Related issues (if any)
   - Testing performed
   - Screenshots/videos (if UI changes)
   - Breaking changes (if any)

6. **Review Process**
   - Maintainers will review your PR
   - Address any requested changes
   - Once approved, your PR will be merged

## Development Guidelines

### Architecture Principles

1. **Modularity:** Components should be loosely coupled
2. **Real-time:** Critical paths must be deterministic
3. **Safety:** Always fail safely
4. **Extensibility:** Easy to add new features
5. **Testability:** All code should be testable

### Performance Considerations

- Avoid allocations in control loop
- Use NumPy for vectorized operations
- Profile before optimizing
- Document performance requirements
- Add benchmarks for critical code

### Safety Requirements

All control code must:
- Check for NaN/inf values
- Validate input ranges
- Handle exceptions gracefully
- Implement timeouts
- Have emergency stop capability

### Adding New Subsystems

1. Create module in appropriate directory
2. Implement required interface
3. Add unit tests
4. Add integration example
5. Document in ARCHITECTURE.md
6. Update README

Example:
```python
# locomotion/my_controller.py
from dataclasses import dataclass
import numpy as np

@dataclass
class MyControllerConfig:
    param1: float = 1.0

class MyController:
    def __init__(self, config: MyControllerConfig):
        self.config = config
    
    def compute_control(self, state, dt: float):
        # Your control logic
        return control_output

# tests/test_my_controller.py
def test_my_controller():
    controller = MyController(MyControllerConfig())
    result = controller.compute_control(test_state, 0.001)
    assert result is not None
```

## Areas for Contribution

### High Priority
- [ ] Inverse kinematics solvers
- [ ] Improved gait stability
- [ ] Perception system
- [ ] Hardware drivers
- [ ] Documentation improvements

### Medium Priority
- [ ] Additional locomotion modes (stairs, slopes)
- [ ] Manipulation planning
- [ ] Visualization tools
- [ ] Performance optimizations
- [ ] More examples

### Good First Issues
- [ ] Add more unit tests
- [ ] Improve error messages
- [ ] Fix documentation typos
- [ ] Add code comments
- [ ] Create tutorials

## Communication

- **GitHub Issues:** Bug reports, feature requests
- **GitHub Discussions:** Questions, ideas, general discussion
- **Pull Requests:** Code contributions

## Recognition

Contributors will be recognized in:
- CONTRIBUTORS.md file
- Release notes
- Project documentation

Significant contributors may be invited to join the core team.

## License

By contributing, you agree that your contributions will be licensed under the MIT License.

## Questions?

If you have questions about contributing, please:
1. Check existing documentation
2. Search closed issues
3. Ask in GitHub Discussions
4. Open a new issue

## Thank You!

Your contributions help make HumanoidOS better for everyone. We appreciate your time and effort! 🤖✨

---

*Last updated: November 2024*
