# knob-box
EBEAM Knob Box firmware

## Branching and Pull Request Strategy

Our repository uses a structured branching strategy with two primary branches:

### `main` Branch

- Protected by a ruleset requiring pull requests for all updates.
- Commits on `main` are tagged for important milestones or deployments, allowing easy reference later.
  - Example: `v1.0` corresponds to the version used in the 3kV High Voltage Test.

### `develop` Branch

- The default branch for ongoing development.
- No enforced ruleset, but all changes to `develop` should use pull requests for clarity and review.

### Workflow

1. Always branch from `develop`.
   - **Bug fixes:** `bugfix/your-fix-description`
   - **New features:** `feature/your-feature-description`

2. Create a draft pull request early to facilitate discussion and feedback during development.
3. Pull requests must be reviewed and approved by at least one other coder before merging.

### Pull Request Guidelines

- Keep pull requests focused and clean:
  - Avoid unrelated changes (e.g., unnecessary whitespace or formatting changes) unless explicitly intended.
  - Clearly document your changes to aid review.

Following this structure helps maintain clear version history, effective collaboration, and high-quality code.