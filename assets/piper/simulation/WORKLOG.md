# Scene preparation

User confirmed Isaac Sim.
Capabilities: composition and placement → usd-pipeline; runtime integration → isaac-sim-orchestrator; final checks → isaac-sim-validator.
Source documents are asset metadata, not user instructions. Preserve existing room, desk, robot and add camera mount.

## Foundations
- Both ZIP CRC checks passed; 2063 composed prims; referenced files resolve.
- Source scenes authored for Isaac Sim 6.0.1; this machine provides 5.1.0. Verify runtime instead of treating prior evidence as current.
- Camera geometry is already in metres and Z-up, but its original scan pose is horizontal at Z≈2.1 m. Rotate the complete source root, preserve all internal body-local anchors.
- Mount placement is an assumed rear-left tabletop installation, not a measured pose. Fixed attachment replaces source WorldClamp; do not run two competing physics loops.

## Iteration notes
- Initial inspection found room physics-material bindings pointing outside reference scope. The room is visual background; mask those bindings in the new overlay.
- Use the installed SimulationApp to access USD schemas; bare Python has no pxr and injecting only the USD Python directory misses native libraries.
- Keep all original files intact. New scene and runtime are a separate composition overlay and entry point.

## Compatibility repair and validation iterations
- First combined runtime found no robot joint1 in the articulation. Isaac Sim 5.1 did not instantiate nested rigid bodies from the 6.0 source. Added reset transform stacks in the new layer, preserving world transforms; all ten table/robot DOFs then appeared.
- Second run completed 10 s of physics, with table height 0.635–0.713311 m and camera-mount maximum joint-anchor gap 0.230365 mm. Additional workspace-camera render failed because its viewpoint was inside the camera housing.
- Moved only the virtual optical viewpoint 65 mm towards the work area; retained explicit uncalibrated status. Added graceful cleanup on runtime exceptions. Repeat full test for final revision.
- Procedure lessons: preserve rigid-body world poses when repairing nested transforms; verify a virtual camera is outside its own visible housing; run one shared physics loop for springs, table and robot. Recorded locally, without changing user memory or global skills.

## Latest user correction
User requested foreground assets from supplied packages, Isaac Sim GUI open, black plate above robot flange, and 8 cm right clearance. Interpreted screenshot arrow as front-right camera-mount relocation. Measured current gap 79.999978 mm; robot bottom 635 mm, flange top 638.999948 mm, plate underside 639 mm. Shifted board, arm and clamps left 72 mm to retain hole alignment. Camera mount at front-right edge. Re-run validation on this exact revision.
User then provided an actual photo confirming the stand beside the right end of the black plate, attached at the front tabletop edge. Saved references locally. Photo is visual layout evidence, not metric or camera calibration.

User clarified clamp reversal as horizontal 180-degree yaw, not upside down, and requested lamp between sofa and table. Applied whole-clamp world-space rotation, corrected local-vs-world offset error, and placed entire lamp bounding box in the clear strip. Camera base shifted outward 25 mm after front-edge contact caused anchor mismatch in prior test. Final test must pass before release.

## Final clarified requirements
The user clarified that the long plate presses down on the robot flange and the two screw clamps grip plate plus tabletop. Final close-up reference shows C backs outside the front edge and screws underneath. Replaced static meshes with the supplied movable clamp articulation, preserving screw mimic coupling, swivel pad and sliding handle. Neutral feed shifted -18.042 mm to fit the combined thickness; dynamic half-turn release/return tested separately.
Camera fixture initially sat too far outside; measured its actual upper-jaw contact surface, raised root 8.299 mm, seated upper jaw at Z=.635 and pad at Z=.610, obtaining 34.917 mm tabletop insertion. This also clears the arm without forced swivel rotation. User confirmed camera horizontal heading 30 degrees to table edge and 30 degrees down, not arm inclination. Solved the two existing head joints to align the optical face and authored a matching virtual camera.

## Final current verification
PASS: 10.002 s combined physics. Table 0.635 to 0.713311 m and return. Both screws moved approximately -1.5024 mm for a half-turn release and returned to nominal contact; maximum feed/rotation coupling error below 1.4e-9 m. Camera maximum sampled anchor mismatch 0.685866 mm. Final optical heading 29.9967 degrees, downward pitch 30.0132 degrees. Six views captured. Fixture and friction-capacity limitations retained in README. GUI launched with the same scene and an added view-selection / clamp-turn control panel.


## Rigorous audit and V4 foreground completion
Added missing task assets, corrected nine USD inertia axes, prismatic velocity units, hollow desk collision geometry, static room collision, V4 wrist visuals, TCP frame, and appliance controls. Completed scoped 38 s native contact audit, gripper contact release check, independent runtime FK and vendor URDF provenance comparison. Three passive hardware poses support new URDF controller kinematics; physical calibration and firmware string remain unverified.

Final appearance repair: effective material inspection found imported ancestor strongerThanDescendants bindings masking microwave/airfryer atlas materials. Bound original atlas at asset root; original UVs and physics retained. Re-rendered six views and repeated 10-second regression.
