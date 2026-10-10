# Generated UR5e, articulated 2F-85 and wrist camera

Regenerate inside the optional ROS image:

```bash
docker compose --profile ros2-observability run --rm ros2-observability \
  python scripts/export-tool-description.py --output configs/models/ur5e_2f85_d405
```

The articulated URDF is for visualization; publish measured passive linkage
joints. Closed constraints are enforced by the matching MuJoCo model, not URDF.
The rigid URDF freezes the nominal 50 mm aperture for six-interface CRISP. It
approximates changing finger dynamics. See ADR 019 and `docs/contracts/tooling.md`.

Source identities and generated hashes are in `provenance.json`. Official UR
source description is BSD-3-Clause, retained in UR-LICENSE. Its UR5e meshes are
referenced from the installed pinned `ur_description` package, not duplicated.
Menagerie tool assets retain BSD-2-Clause in ROBOTIQ-LICENSE. Upstream XML source
notices are retained by the generator. Camera and bracket geometry are original
primitives. No physical mounting or optical accuracy is claimed.
