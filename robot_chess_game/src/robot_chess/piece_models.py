"""SDF of the chess pieces spawned in Gazebo.

A piece is a static model (it only moves when an arm carries it): a base
disc and a body whose height depends on the piece type.  The model origin is
at the bottom centre.
"""
from robot_chess.pieces import BODY_RGB, PIECE_HEIGHT, PIECE_RADIUS


def _material(rgb):
    r, g, b = rgb
    return ("<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>"
            "<specular>0.1 0.1 0.1 1</specular></material>").format(r=r, g=g, b=b)


def piece_sdf(model_name, color, piece_type):
    h = PIECE_HEIGHT[piece_type]
    r = PIECE_RADIUS
    m = _material(BODY_RGB[color])
    visuals = (
        "<visual name='base'><pose>0 0 {z1} 0 0 0</pose><geometry><cylinder>"
        "<radius>{r}</radius><length>{l1}</length></cylinder></geometry>{m}</visual>"
        "<visual name='body'><pose>0 0 {z2} 0 0 0</pose><geometry><cylinder>"
        "<radius>{r2}</radius><length>{l2}</length></cylinder></geometry>{m}</visual>"
        "<visual name='top'><pose>0 0 {z3} 0 0 0</pose><geometry><cylinder>"
        "<radius>{r}</radius><length>0.003</length></cylinder></geometry>{m}</visual>"
    ).format(z1=0.002, r=r, l1=0.004, z2=h / 2.0, r2=r * 0.7, l2=h, z3=h - 0.0015, m=m)
    collision = ("<collision name='collision'><pose>0 0 {z} 0 0 0</pose><geometry>"
                 "<cylinder><radius>{r}</radius><length>{h}</length></cylinder>"
                 "</geometry></collision>").format(z=h / 2.0, r=r, h=h)
    return ("<?xml version='1.0'?><sdf version='1.6'><model name='{name}'>"
            "<static>true</static><link name='link'>{c}{v}</link></model></sdf>").format(
                name=model_name, c=collision, v=visuals)
