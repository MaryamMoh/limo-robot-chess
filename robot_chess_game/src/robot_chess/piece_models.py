"""SDF generation for the chess pieces spawned in Gazebo.

A piece is a static model (it only moves when a robot carries it):
    base disc -> body cylinder with a coloured type band -> head sphere.
The model origin is at the bottom centre of the piece.
"""
from robot_chess.pieces import (BAND_BOTTOM, BAND_RGB, BAND_TOP, BASE_HEIGHT,
                                BODY_RGB, PIECE_HEIGHT, PIECE_RADIUS)


def _material(rgb, emissive=0.0):
    r, g, b = rgb
    return ("<material><ambient>{r} {g} {b} 1</ambient>"
            "<diffuse>{r} {g} {b} 1</diffuse>"
            "<specular>0.1 0.1 0.1 1</specular>"
            "<emissive>{er} {eg} {eb} 1</emissive></material>").format(
                r=r, g=g, b=b, er=r * emissive, eg=g * emissive, eb=b * emissive)


def _cylinder_visual(name, radius, z0, z1, material):
    return ("<visual name='{n}'><pose>0 0 {z} 0 0 0</pose><geometry><cylinder>"
            "<radius>{r}</radius><length>{l}</length></cylinder></geometry>{m}"
            "</visual>").format(n=name, z=(z0 + z1) / 2.0, r=radius,
                                l=z1 - z0, m=material)


def piece_sdf(model_name, color, piece_type):
    height = PIECE_HEIGHT[piece_type]
    body = _material(BODY_RGB[color])
    band = _material(BAND_RGB[piece_type], emissive=0.3)
    head_r = 0.055
    neck_top = height - head_r
    visuals = [
        _cylinder_visual("base", PIECE_RADIUS, 0.0, BASE_HEIGHT, body),
        _cylinder_visual("body_low", 0.07, BASE_HEIGHT, BAND_BOTTOM, body),
        _cylinder_visual("band", 0.075, BAND_BOTTOM, BAND_TOP, band),
        _cylinder_visual("body_high", 0.06, BAND_TOP, neck_top, body),
        ("<visual name='head'><pose>0 0 {z} 0 0 0</pose><geometry><sphere>"
         "<radius>{r}</radius></sphere></geometry>{m}</visual>").format(
             z=neck_top, r=head_r, m=body),
    ]
    collision = ("<collision name='collision'><pose>0 0 {z} 0 0 0</pose>"
                 "<geometry><cylinder><radius>{r}</radius><length>{l}</length>"
                 "</cylinder></geometry></collision>").format(
                     z=height / 2.0, r=PIECE_RADIUS, l=height)
    return ("<?xml version='1.0'?><sdf version='1.6'><model name='{name}'>"
            "<static>true</static><link name='link'>{collision}{visuals}"
            "</link></model></sdf>").format(name=model_name, collision=collision,
                                            visuals="".join(visuals))
