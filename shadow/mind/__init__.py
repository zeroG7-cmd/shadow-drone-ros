"""
Mind layer - this package itself is still deliberately empty.

First real implementation lives at shadow_ws/scripts/run_mind.py (Oct
17 2026), not inside this package - it's a plain script, not an rclpy
node, same "don't make something talk to ROS2 that doesn't have to"
reasoning nodes/telemetry_publisher.py's docstring already gives. It
shells out to nodes/run_skill.py, the same proven path a human typing
the command would use, rather than importing anything from `shadow`
directly.

Important: per SKILL_TREE.md, the Mind was meant to unlock once Tier 1
AND Tier 2 have real passing streaks. As of Oct 17, hover (Tier 1) is
still unproven (progress.json: streak 0, unlocked:false) - run_mind.py
hard-excludes hover from its own choices for this reason rather than
waiting for this package to formally gate it. This is a real but
early/bounded first version (hard step cap, stops on first failure,
only the genuinely-unlocked non-hover skills), not the full Tier-3
planner described below once this project gets there.
"""
