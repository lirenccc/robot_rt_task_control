# Releasing

Guidelines for cutting a public release of this workspace.

## Versioning

- Use [Semantic Versioning](https://semver.org/) for the workspace meta-version (currently **0.3.0**)
- Keep every `src/*/package.xml` `<version>` tag in sync on each release
- Do **not** treat 0.x as a production ABI freeze

## Checklist

1. CI green on the default branch (`colcon build` + `robot_testkit`, including launch and soft-RT tests)
2. Update [CHANGELOG.md](../CHANGELOG.md) following [Keep a Changelog](https://keepachangelog.com/)
3. Bump all `package.xml` versions together
4. Tag `vX.Y.Z` and push the tag
5. Call out breaking changes under `### Changed` / `### Removed`

## What “generic framework” means in this release

- Pluggable factories + `ITaskPlanner` + safety via `pluginlib`
- Reference sim bus + YAML task graphs
- Soft RT baseline documented (not hard-RT certification)
- EtherCAT masters remain **external** modules — see [ETHERCAT_INTEGRATION.md](ETHERCAT_INTEGRATION.md)

## Documentation language

- `README.md` / `README.zh-CN.md` are the project entry points
- Prefer stable product wording in public docs; avoid internal roadmap labels in user-facing pages
