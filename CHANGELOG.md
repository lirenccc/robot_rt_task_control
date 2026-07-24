# Changelog

All notable changes to this workspace are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project uses [Semantic Versioning](https://semver.org/).

## [0.3.0] - 2026-07-22

### Added

- `ProviderRegistry` factories for navigation, manipulation, planner, and safety
- `ITaskPlanner` with `SimpleTaskPlanner` and `YamlGraphPlanner` (`graph:` DSL)
- `ReferenceSimHardwareBus` (sequence / watchdog / fault inject)
- Safety policies via `pluginlib` (`robot_safety/VelocityLimitPolicy`)
- Soft RT baseline test and docs
- Launch integration test for in-process mock task stack
- English docs mirrors; `RELEASING.md`
- Optional `robot_ethercat_adapters` (`EcMasterHardwareBus` / `IghHardwareBus`) via QUIET master underlays
- Example runtimes `runtime_ethercat_ecmaster.yaml` / `runtime_ethercat_igh.yaml`

### Changed

- `TaskOrchestrator` takes `shared_ptr<ITaskPlanner>`
- `RuntimeBuilder` routes exclusively through `ProviderRegistry`
- Package versions aligned to 0.3.0

## [0.2.0] - 2026-07-22

### Added

- Extension templates: Nav2 and FollowJointTrajectory adapters, Health/Fault surfaces, Skeleton bus
- Environment setup scripts and apt/pip requirement lists
- EtherCAT external integration contract

## [0.1.0] - 2026-07-21

### Added

- Initial `robot_*` generic framework skeleton
