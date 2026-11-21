"""Base stations: reaching a board that is deeper than the arm's reach.

The myCobot 280 reaches only 7..27 cm from its base with the tool pointing
down.  If the board is deeper, the LIMO drives straight forward/backward
(under the overhanging table) between a few "stations" - offsets along its
own x axis from the home position.  The camera always reads the board from
station 0 (home), where the observe pose was set up.
"""


def reachable_from(geometry, kinematics, label, offset):
    saved = geometry.base_offset
    geometry.base_offset = offset
    try:
        return (kinematics.ik_down(geometry.grasp_point(label)) is not None and
                kinematics.ik_down(geometry.travel_point(label)) is not None)
    finally:
        geometry.base_offset = saved


def choose_station(geometry, kinematics, label, stations, current=0.0):
    """Offset to use for `label`: the current one if it works, otherwise the
    nearest station from which the label is reachable; None if none is."""
    if reachable_from(geometry, kinematics, label, current):
        return current
    for offset in sorted(stations, key=lambda o: abs(o - current)):
        if reachable_from(geometry, kinematics, label, offset):
            return offset
    return None


def coverage(geometry, kinematics, labels, stations):
    """{label: [offsets it is reachable from]}"""
    return {lab: [o for o in stations if reachable_from(geometry, kinematics, lab, o)]
            for lab in labels}
