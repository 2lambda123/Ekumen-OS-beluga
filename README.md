# Beluga

[![CI pipeline](https://github.com/Ekumen-OS/beluga/actions/workflows/ci_pipeline.yml/badge.svg?branch=main)](https://github.com/Ekumen-OS/beluga/actions/workflows/ci_pipeline.yml?query=branch:main)
[![codecov](https://codecov.io/gh/Ekumen-OS/beluga/branch/main/graph/badge.svg?token=rK7BNC5giK)](https://codecov.io/gh/Ekumen-OS/beluga)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)
[![License Apache-2.0](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)

## Overview

Beluga is an extensible C++17 library with a ground-up implementation of the Monte Carlo Localization (MCL) family of estimation algorithms featuring:

- A modular design based on orthogonal components that can be merged together using the mixin pattern.
- Emphasis on the prevention of regressions and facilitation of code improvements through test coverage.
- Semi-automated benchmarks that can be used to validate different configurations.

https://github.com/Ekumen-OS/beluga/assets/33042669/98bda0ee-a633-4e35-8743-72a9ab30b494

<p align="center"><i><b>Beluga AMCL</b> running on an <b>Andino</b> robot (Raspberry Pi 4B), go to <a href="https://github.com/Ekumen-OS/andino">Ekumen-OS/andino</a> for more details!</i></p>

## Packages

This repository contains the following packages:

| Package                                      | Description                                                                                                             |
|----------------------------------------------| ------------------------------------------------------------------------------------------------------------------------|
| [`beluga`](beluga)                           | A ROS-agnostic extensible library to implement algorithms based on particle filters.                                    |
| [`beluga_amcl`](beluga_amcl)                 | A ROS wrapper, providing an executable node and component (or nodelet).<br> It provides interface parity with `nav2_amcl` (and `amcl`). |
| [`beluga_example`](beluga_example)           | Example launch files, showing how to run Beluga-based nodes.                                                            |
| [`beluga_benchmark`](beluga_benchmark)       | Scripts to benchmark, profile and also compare Beluga with other MCL implementations.                                   |
| [`beluga_system_tests`](beluga_system_tests) | System integration tests for Beluga.                                                                                    |

## First Steps

- Get hands-on experience with the [getting started](GETTING_STARTED.md) tutorial.
- Check the [installation guide](INSTALLING.md) to test this on a robot.
- Read the [contributing guidelines](CONTRIBUTING.md).
