#!/usr/bin/env python3
"""Generate worlds/airground_arena.wbt.

The world is generated rather than hand-edited because its ground is a 51x51
elevation grid, which is not something to maintain by hand, and because the
crater placement has to stay in step with the sector geometry in
nodes/airground_fsm.py.

    python3 worlds/gen_world.py

Everything else about the arena is inherited verbatim from simulation 2's
`pioneer_arena.wbt`: the same trees, rocks and deer, the same 50 x 50 m
footprint. Two things are deliberately different.

**The ground is ours.** Simulation 2 uses `RectangleArena` (a flat floor) plus
sixteen `Pit` nodes. Those `Pit`s are not holes. The proto's height is
`(1 - g) * g * size.z` with `g` a gaussian peaking at the centre, so it is
zero at the centre, zero at the rim, and about 1.2 m in between: a low BERM.
Measured in Webots, ground height across that arena ran +0.40 to +1.14 m and
never once went negative. A horizontal lidar sees a berm perfectly well, so
the whole premise of this simulation, a hazard the ground tier cannot see and
the air tier can, had no physical basis. Making the `Pit`s negative would not
have helped either, because `RectangleArena`'s solid floor sits on top of them
at z = 0.

So the floor and the sixteen pits are replaced by one elevation grid with real
depressions. It is also far cheaper: ~5k collision triangles against the
sixteen 100x100 pit meshes' ~320k, which is what made a fine physics timestep
unaffordable (0.014x real time at an 8 ms step, against 0.90x at 32 ms).

**The fleet is two Pioneers and two Mavics**, not five Pioneers.
"""

import math
import os
import pathlib
import re

HERE = pathlib.Path(__file__).resolve().parent
ARENA = HERE / ".." / ".." / "simulation_arena" / "worlds" / "pioneer_arena.wbt"
OUT = HERE / "airground_arena.wbt"

SPAN, STEP = 50.0, 1.0                 # arena is 50 x 50 m; 1 m grid
N = int(SPAN / STEP) + 1

# Craters, as (x, y, radius, depth). Both sit on a SECTOR CENTRE, because the
# drones survey centre to centre and would otherwise fly straight past them.
# S05 is the one the launch_test also uses (against a mock crater).
#   3 m deep over a 4.5 m radius: 46 degree walls, and 1.76 m below grade at
#   the coordinator's 2 m `cover_radius`.
#
#   Both numbers had to be tuned against `cover_radius`, and the second one is
#   the subtle part. A bot credits a sector once it is within `cover_radius` of
#   the centre, so if the bowl is narrow enough that its rim lies outside that
#   ring, the bot marks the sector explored from flat ground and never meets
#   the hazard at all. That happened: at radius 3 the ground is only 0.17 m
#   down at 2 m out, and S05 was cheerfully reported explored. The crater has
#   to be WIDER than the credit radius, not just deeper.
#
#   Depth and slope matter for the other half: at 30 degrees a Pioneer simply
#   drives through. Past 45 it cannot, so it either stalls on the wall or ends
#   up in the bowl, and either way it reports what it found.
# ONE crater, not two. A crater steep enough to be worth warning about is
# steep enough to swallow a Pioneer, and it does: the bot that confirms S05
# ends up in it and stops claiming. With two craters both bots can be lost and
# the sweep never finishes. One crater loses one bot, the other finishes the
# remaining eleven sectors, and the run still tells the whole story.
CRATERS = [(-5.0, 0.0, 4.5, 3.0)]      # S05


def height(x, y):
    """Cosine bowl: flat lip, steepest mid-wall, rounded floor."""
    h = 0.0
    for cx, cy, rad, depth in CRATERS:
        d = math.hypot(x - cx, y - cy)
        if d < rad:
            h -= depth * 0.5 * (1.0 + math.cos(math.pi * d / rad))
    return h


def elevation_rows():
    """Heights in Webots' order: X VARIES FASTEST within a row of constant y.

    Getting this backwards transposes the terrain silently. It cost a
    measurement to find: a crater placed at (-5, 0) turned up at (0, -5), read
    off the drone's own ranger.
    """
    rows = []
    for j in range(N):
        y = -SPAN / 2 + j * STEP
        rows.append(" ".join(f"{height(-SPAN / 2 + i * STEP, y):.3f}"
                             for i in range(N)))
    return "\n        ".join(rows)


def carry_over(src, kinds):
    """Lift whole node blocks out of the arena world, verbatim."""
    out = []
    for kind in kinds:
        for m in re.finditer(rf'^{re.escape(kind)} \{{', src, re.M):
            i = m.start()
            out.append(src[i:src.index("\n}\n", i) + 3])
    return out


def main():
    src = ARENA.read_text()
    decor = carry_over(src, ("BigSassafras", "Sassafras", "Pine", "Rock", "Deer"))

    robots = f'''DEF BOT_0 Pioneer3at {{
  translation -23 -12 0.11
  rotation 0 0 1 0
  name "bot_0"
  controller "pioneer_sweeper"
  extensionSlot [
    GPS {{
    }}
    InertialUnit {{
    }}
    Robotino3Webcam {{
      translation 0.170682 0.0048234 0.21
      rotation 0.15222928589302986 0.15222928589302986 -0.9765513243209476 -1.5708053071795867
    }}
    SickLms291 {{
      translation 0.17 0 0.32
    }}
  ]
}}
DEF BOT_1 Pioneer3at {{
  translation -23 -6 0.11
  rotation 0 0 1 0
  name "bot_1"
  controller "pioneer_sweeper"
  extensionSlot [
    GPS {{
    }}
    InertialUnit {{
    }}
    Robotino3Webcam {{
      translation 0.170682 0.0048234 0.21
      rotation 0.15222928589302986 0.15222928589302986 -0.9765513243209476 -1.5708053071795867
    }}
    SickLms291 {{
      translation 0.17 0 0.32
    }}
  ]
}}
'''
    for name, x, y in (("DRONE_0", -24, -24), ("DRONE_1", 24, 24)):
        robots += f'''DEF {name} Mavic2Pro {{
  translation {x} {y} 0.3
  rotation 0 0 1 0
  name "{name.lower()}"
  controller "mavic_surveyor"
  bodySlot [
    # The reason the air tier exists. A Pioneer's Sick LMS 291 is horizontal
    # and cannot see a hole; from above, a crater reads as ground further away
    # than the drone's own altitude. Mounted in bodySlot, an MFNode that sits
    # directly in the proto's Robot.children, so no derived proto is needed.
    # A "laser" DistanceSensor must have exactly one ray.
    DistanceSensor {{
      name "ground_ranger"
      translation 0 0 -0.05
      rotation 0 1 0 1.5708          # point straight down (-Z)
      type "laser"
      numberOfRays 1
      lookupTable [
        0    0    0
        40   40   0.002
      ]
    }}
  ]
}}
'''

    walls = ""
    for i, (tx, ty, sx, sy) in enumerate([
            (0, SPAN / 2, SPAN, 0.2), (0, -SPAN / 2, SPAN, 0.2),
            (SPAN / 2, 0, 0.2, SPAN), (-SPAN / 2, 0, 0.2, SPAN)]):
        walls += f'''Solid {{
  translation {tx} {ty} 0.5
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.55 0.5 0.45
        roughness 0.9
        metalness 0
      }}
      geometry DEF WALL_{i} Box {{
        size {sx} {sy} 1
      }}
    }}
  ]
  name "wall_{i}"
  boundingObject USE WALL_{i}
}}
'''

    world = f'''#VRML_SIM R2025a utf8
# GENERATED by worlds/gen_world.py — edit that, not this.

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/adept/pioneer3/protos/Pioneer3at.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/dji/mavic/protos/Mavic2Pro.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/devices/sick/protos/SickLms291.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/festo/robotino3/protos/Robotino3Webcam.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/appearances/protos/DryMud.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/trees/protos/BigSassafras.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/trees/protos/Sassafras.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/trees/protos/Pine.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/rocks/protos/Rock.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/animals/protos/Deer.proto"

WorldInfo {{
  # 8 ms is what the Mavic sample asks for, and with this ground it is free.
  # The old world could not afford it: its sixteen 100x100 Pit meshes put
  # physics at ~595 ms/step and real time at 0.014x, so it ran at 32. Measured
  # again on this ground, 32 gives 43x real time and 8 gives 19x, so the finer
  # step costs nothing that matters and buys solver convergence. That matters
  # here because a Pioneer sitting in the crater's steep walls at 32 ms made
  # ODE give up: "the current physics step could not be computed correctly".
  # Damping matters for the drones too; the control law is tuned with it.
  basicTimeStep 8
  defaultDamping Damping {{
    linear 0.5
    angular 0.5
  }}
}}
Viewpoint {{
  orientation 1 0 0 0.72
  position 0 -42 48
  follow ""
}}
TexturedBackground {{
}}
TexturedBackgroundLight {{
}}

# The ground: one elevation grid with REAL craters, replacing simulation 2's
# flat RectangleArena floor and its sixteen berm-shaped "Pit" nodes. See the
# module docstring of gen_world.py for why.
DEF GROUND Solid {{
  translation {-SPAN / 2:.1f} {-SPAN / 2:.1f} 0
  children [
    Shape {{
      appearance DryMud {{
        IBLStrength 0
      }}
      geometry DEF GROUND_GRID ElevationGrid {{
        xDimension {N}
        xSpacing {STEP:.1f}
        yDimension {N}
        ySpacing {STEP:.1f}
        height [
        {elevation_rows()}
        ]
      }}
    }}
  ]
  name "ground"
  boundingObject USE GROUND_GRID
}}

{walls}
{"".join(decor)}
{robots}'''

    OUT.write_text(world)
    print(f"wrote {OUT.name}: {N}x{N} ground grid, {len(CRATERS)} craters "
          f"(deepest {min(-d for _, _, _, d in CRATERS):.1f} m), "
          f"{len(decor)} decorative nodes, 2 Pioneers, 2 Mavics")


if __name__ == "__main__":
    main()
