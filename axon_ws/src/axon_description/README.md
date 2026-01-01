# axon_description

Robot description package containing the URDF/xacro model and frame naming conventions.

## Files
- `urdf/axon.urdf.xacro`: base links and placeholder sensor frames.
- `config/frames.yaml`: standard frame names used across the stack.

## Extending
Add new sensors or links by editing `urdf/axon.urdf.xacro` and updating
`config/frames.yaml` to keep frame names consistent across drivers and localization.
