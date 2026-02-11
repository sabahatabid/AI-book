# Contributing to Physical AI & Humanoid Robotics Book

Thank you for your interest in contributing! This document provides guidelines for contributing to the project.

## How to Contribute

### Reporting Issues

- Use GitHub Issues to report bugs
- Include detailed steps to reproduce
- Provide system information (OS, Node version, Python version)
- Include error messages and logs

### Suggesting Enhancements

- Open an issue with the "enhancement" label
- Describe the feature and its benefits
- Provide examples or mockups if possible

### Code Contributions

1. **Fork the Repository**
   ```bash
   git clone https://github.com/yourusername/physical-ai-book.git
   cd physical-ai-book
   ```

2. **Create a Branch**
   ```bash
   git checkout -b feature/your-feature-name
   ```

3. **Make Changes**
   - Follow the existing code style
   - Add comments for complex logic
   - Update documentation as needed

4. **Test Your Changes**
   ```bash
   # Frontend
   npm start
   
   # Backend
   cd backend
   python -m pytest
   ```

5. **Commit Your Changes**
   ```bash
   git add .
   git commit -m "Add: Brief description of changes"
   ```

6. **Push and Create Pull Request**
   ```bash
   git push origin feature/your-feature-name
   ```

## Code Style Guidelines

### Python (Backend)

- Follow PEP 8
- Use type hints
- Write docstrings for functions
- Keep functions focused and small

```python
def process_data(input: str) -> dict:
    """
    Process input data and return result.
    
    Args:
        input: The input string to process
        
    Returns:
        Dictionary containing processed data
    """
    # Implementation
    pass
```

### JavaScript/React (Frontend)

- Use functional components
- Follow React best practices
- Use meaningful variable names
- Add PropTypes or TypeScript types

```javascript
import React from 'react';

export default function MyComponent({ prop1, prop2 }) {
  // Implementation
  return <div>...</div>;
}
```

### Markdown (Documentation)

- Use clear headings
- Include code examples
- Add images where helpful
- Keep paragraphs concise

## Content Contributions

### Adding Course Content

1. Create markdown files in appropriate `docs/` subdirectory
2. Follow existing structure and formatting
3. Include code examples
4. Add to `sidebars.js` if creating new sections

### Improving Existing Content

- Fix typos and grammar
- Clarify confusing sections
- Add examples
- Update outdated information

## Testing

### Frontend Tests

```bash
npm test
```

### Backend Tests

```bash
cd backend
pytest
```

### Manual Testing

- Test all features after changes
- Check responsive design
- Verify API endpoints
- Test authentication flow

## Documentation

- Update README.md for major changes
- Add inline code comments
- Update API documentation
- Include examples in docstrings

## Review Process

1. All contributions require review
2. Address reviewer feedback
3. Ensure CI/CD passes
4. Maintain code quality standards

## Community Guidelines

- Be respectful and inclusive
- Help others learn
- Share knowledge
- Give constructive feedback

## Questions?

- Open a GitHub Discussion
- Check existing issues
- Read the documentation
- Contact maintainers

Thank you for contributing! 🚀
