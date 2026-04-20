# Contributing

## Scope

This repository is currently a research and prototyping codebase for marine route planning. Contributions that improve reproducibility, build reliability, algorithm clarity, and dataset handling are the most useful.

## Development Setup

On Ubuntu:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake qtbase5-dev libqt5svg5-dev
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
```

Optional data preparation:

```bash
./scripts/download_coastline_data.sh
```

## Contribution Guidelines

- Keep changes focused and easy to review.
- Prefer small pull requests over large mixed changes.
- Do not commit generated caches, local virtual environments, or large temporary datasets.
- Update `README.md` when user-facing commands, data expectations, or outputs change.
- Add or update validation steps when changing planner behavior.

## Good Contribution Areas

- CI and cross-platform build improvements
- automated smoke tests
- dataset packaging and size reduction
- benchmark scenarios and reproducibility
- route quality validation and regression checks
- documentation, examples, and demo assets

## Pull Requests

When opening a pull request, include:

- what changed,
- why it changed,
- how you validated it,
- any dataset or environment assumptions.

If your change affects outputs, include at least one screenshot, SVG, or command example.

## Issues

Please use issues for:

- build failures,
- incorrect routing behavior,
- dataset import problems,
- reproducibility gaps,
- documentation problems.

Clear reproduction steps are more useful than long descriptions.
