The files in the directory have not been used in the project yet and need to be checked for retention because some of
them have the same functionality as the ones that WPILIB provides.

- `api` is not used in the project
- `hardware.Pigeon` can be replaced by `Pigeon2`
- `ui.Xbox` can be replaced by `CommandXboxController`
- `util.Axis` and `util.Button` are only used in `ui.Xbox` and `hardware.Pigeon` so they can be removed
- `util.Lazy` can be deleted because lazy initialization does not provide any benefit
- `util.Util` have no use cases in the project
- `util.Vec2` can be replaced by `Translation2d`
- `util.Vec3` can be replaced by `Translation3d`
- `Zambda` is not used in the project
- `ZNav` and `ZPath` can be replaced by `Pathplanner`

- `util.Result` provides a way to handle exceptions in a more functional way. It may be used in the future but not now.