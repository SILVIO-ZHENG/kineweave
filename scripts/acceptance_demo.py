"""Generate inspectable local acceptance data without opening hardware or cameras.
All camera pixels in this script are explicitly synthetic diagnostic patterns.
"""

from pathlib import Path
import json
import time

import numpy as np

from teleop_v2.cli import run_demo
from teleop_v2.drivers import create_driver
from teleop_v2.kinematics import matrix_pose
from teleop_v2.profiles import list_profiles, load_profile
from teleop_v2.recording import export_csv, inspect_episode
from teleop_v2.runtime import TeleopRuntime
from teleop_v2.types import InputFrame


def main():
    """Exercise five dimensions, Cartesian input, encoded camera frames and exports.
    """
    root = Path(__file__).resolve().parents[1]
    output = root / "outputs" / "validation"
    output.mkdir(parents=True, exist_ok=True)
    results = []
    for profile in list_profiles():
        result = run_demo(profile["id"], .3, str(root / "recordings"))
        results.append({"robot": result["robot"], "dimension": result["dimension"],
                        "episode": result["episode"], "complete": result["inspection"]["complete"],
                        "verified": result["inspection"]["verified"],
                        "samples": result["inspection"]["counts"]["samples"]})

    profile = load_profile("g1_43")
    runtime = TeleopRuntime(profile, create_driver("sim", profile), recording_root=root / "recordings")
    runtime.start(background=False)
    try:
        runtime.action("record_start", task="SYNTHETIC: Cartesian input and diagnostic camera pattern")
        home = runtime.model.forward(profile.initial, "right")
        goal_q = profile.initial.copy()
        joint = profile.chains["right"]["segments"][0]["joint"]
        goal_q[profile.index(joint)] += .12
        goal = runtime.model.forward(goal_q, "right")
        runtime.submit(InputFrame("synthetic-xr", 0, enabled=True, poses={"right": matrix_pose(home)}))
        runtime.action("arm", source_id="synthetic-xr")
        runtime.tick()
        import cv2
        for index in range(1, 31):
            time.sleep(.02)
            runtime.submit(InputFrame("synthetic-xr", index, enabled=True,
                                      poses={"right": matrix_pose(goal)}, raw={"synthetic": True}))
            runtime.tick(.02)
            if runtime.fault:
                raise RuntimeError(runtime.fault)
            if index % 3 == 0:
                image = np.zeros((96, 160, 3), dtype=np.uint8)
                image[:, :, 0] = np.arange(160, dtype=np.uint8)
                image[:, :, 1] = index*7
                image[:, :, 2] = 80
                cv2.putText(image, "SYNTHETIC", (12, 53), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
                ok, jpeg = cv2.imencode(".jpg", image)
                if not ok:
                    raise RuntimeError("Synthetic camera encoding failed")
                runtime.camera("synthetic_preview", jpeg.tobytes(), time.time_ns())
        runtime.action("disarm")
        runtime.action("record_stop")
        inspection = inspect_episode(runtime.last_episode)
        export_csv(runtime.last_episode, output / "cartesian_samples.csv")
        from teleop_v2.cameras import export_video
        video = export_video(runtime.last_episode, "synthetic_preview", output / "synthetic_preview.mp4")
        results.append({"robot": profile.id, "dimension": profile.dimension,
                        "kind": "synthetic Cartesian + camera", "episode": runtime.last_episode,
                        "complete": inspection["complete"], "verified": inspection["verified"],
                        "counts": inspection["counts"], "video": str(video)})
    finally:
        runtime.close()
    (output / "demo_results.json").write_text(json.dumps(results, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(results, indent=2, ensure_ascii=True))


if __name__ == "__main__":
    main()
