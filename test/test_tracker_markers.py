"""RViz marker colours: a buoy shows its own colour; other labels keep a stable one."""
from visualization_msgs.msg import Marker

from mhseals_nav.tracker_markers import PALETTE, colour, rgb_of


def test_a_buoy_takes_the_colour_in_its_class_name():
    for label in ('red_pole_buoy', 'red_buoy', 'red_light_buoy'):
        assert rgb_of(label) == PALETTE['red']
    assert rgb_of('black_cross') == PALETTE['black'] and rgb_of('green_buoy') == PALETTE['green']


def test_other_labels_keep_one_stable_colour():
    assert rgb_of('unknown') == rgb_of('unknown') and rgb_of('unknown') not in PALETTE.values()
    marker = Marker()
    colour('blue_buoy', marker, 0.5)
    assert (marker.color.r, marker.color.b, marker.color.a) == (PALETTE['blue'][0], PALETTE['blue'][2], 0.5)
