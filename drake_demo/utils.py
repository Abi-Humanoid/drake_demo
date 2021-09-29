""" Utilities for Furuta pendulum control
Written by: J. X. J. Bannwarth for the Melbourne Space Program.
"""
import pydrake
import pydot


def print_system(system, output_name=None):
    """
    Print Drake system to svg file

    Parameters
    ----------
    system
        Drake system to print
    output_name : str, optional
        Name of the output svg file, by default the name of the Drake system
    """
    svg_diagram = pydot.graph_from_dot_data(system.GetGraphvizString())[0].create_svg()

    if output_name is None:
        output_name = system.get_name() + ".svg"

    with open(output_name, "w") as file:
        file.write(svg_diagram.decode("utf8"))
        file.close()
