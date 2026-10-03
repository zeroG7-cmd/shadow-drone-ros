"""
Shadow's RPG-layered package: body, senses, skills, mind.

Each layer only talks to the one directly below it (per the roadmap's
architecture table). Nothing above shadow/body/ should ever import
mavros_msgs or mention a flight mode string directly - that's what
lets the sim-to-Pi swap, or a future autopilot swap, stay a Body-only
change.
"""
