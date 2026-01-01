# Code Style

- Python formatting: **Black**
- Import ordering: **isort**
- Linting: **flake8**
- Testing: **pytest**

## Guidelines
- Use typing hints for public functions/classes.
- Keep nodes thin; place logic in plain Python modules.
- Avoid global state where possible.
- Fail fast on parameter errors with explicit messages.
