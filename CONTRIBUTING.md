# Contributing

Thanks for contributing to the Axon ROS 2 workspace.

## Development Workflow
1. Create a feature branch.
2. Keep changes scoped and documented.
3. Run formatting, linting, and tests before opening a PR:
   ```bash
   ./scripts/format.sh
   ./scripts/lint.sh
   ./scripts/test.sh
   ```

## ROS 2 Conventions
- Keep nodes thin and move logic into reusable Python modules.
- Use namespaces consistently in launch files.
- Validate parameters via dataclasses with clear errors.

## Pull Requests
- Summarize changes and list tests run.
- Include relevant configuration updates in the PR description.
