""" Demonstration of Furuta pendulum control
Written by: J. X. J. Bannwarth for the Melbourne Space Program.
"""
import numpy as np
import matplotlib.pyplot as plt
import pydrake
import pydrake.systems.analysis

from drake_demo.furuta_pendulum import FurutaPendulum
from drake_demo.controllers import SwingUpAndCatchController
from drake_demo.utils import print_system


def run_simulation():
    """
    Create closed-loop system and run simulation

    Notes
    -----
    A Drake diagram is a combination of Drake systems connected to one
    another.

    A closed-loop system is a system composed of a plant and a feedback
    controller.

    Using the `set_name()` method of every system is highly recommended
    to make the resulting diagram easier to follow.

    Returns
    -------
    Union[ FurutaPendulum, pydrake.systems.framework.Diagram ]
        Drake system of the Furuta pendulum and Drake diagram of the closed-loop system
    """
    # Create the required Drake systems
    furuta_system = FurutaPendulum()
    furuta_system.set_name("furuta_pendulum")
    controller_system = SwingUpAndCatchController(furuta_system)
    controller_system.set_name("controller")

    # Create the diagram, add the plant and controller, and link everything together
    builder = pydrake.systems.framework.DiagramBuilder()
    furuta = builder.AddSystem(furuta_system)
    controller = builder.AddSystem(controller_system)
    builder.Connect(furuta.get_output_port(0), controller.get_input_port(0))
    builder.Connect(controller.get_output_port(0), furuta.get_input_port(0))

    # Add state loggers to enable visualisation of the data after simulation
    logger_furuta_state = pydrake.systems.primitives.LogVectorOutput(
        furuta.get_output_port(0), builder
    )
    logger_furuta_state.set_name("furuta_state_logger")
    logger_input = pydrake.systems.primitives.LogVectorOutput(
        controller.get_output_port(0), builder
    )
    logger_input.set_name("input_logger")

    # Define the ouput of the combined system (not necessary here, but useful if
    # you want to use the diagram as a building block to an even larger model)
    builder.ExportOutput(furuta.get_output_port(0))

    # Build the diagram into the closed-loop system
    cl_system = builder.Build()
    cl_system.set_name("cl_system")

    # Export the diagram to svg to visualise it
    print_system(cl_system)

    # Create the simulation 'context'
    # In Drake, the 'context' of a simulation contains the values of anthing
    # that can change during the simulation, including system states (e.g. joint
    # angles) and system parameters (link mass or length) that are declared as
    # such
    # You *need* to create a context before you are able to run a simulation
    cl_context = cl_system.CreateDefaultContext()

    # Get the portion of the context corresponding to the Furuta pendulum dynamics
    # and assign the initial state
    # Note that furuta_context is linked back to cl_context - any change you make
    # to it will be visible by cl_context
    furuta_context = furuta_system.GetMyMutableContextFromRoot(cl_context)

    # Set the initial pendulum angle to 180 degrees, which corresponds to the
    # pendulum hanging down at rest
    furuta_context.SetContinuousState([180.0 * np.pi / 180.0, 0.0, 0.0, 0.0])

    # Create simulator
    print("Start simulation")
    simulator = pydrake.systems.analysis.Simulator(cl_system, cl_context)
    simulator.AdvanceTo(15)
    print("End simulation")

    # Retrieve logged data
    state_log = logger_furuta_state.FindLog(cl_context)
    input_log = logger_input.FindLog(cl_context)

    # Plot the results
    # Note that while matplotlib is practical for simple plots like this, it
    # quickly becomes really slow, and packages like plotly are much better
    # alternatives
    fig, axs = plt.subplots(3, sharex="all")

    # Extract and convert to degrees
    theta = (state_log.data()[0].transpose() + np.pi) % (2 * np.pi) - np.pi
    phi = (state_log.data()[1].transpose() + np.pi) % (2 * np.pi) - np.pi

    # Plot the data
    # You can see that the arm starts accelerating and decelerating to excite
    # the pendulum, eventually bringing it up to the upright position
    axs[0].plot(state_log.sample_times(), theta * 180.0 / np.pi, ".")
    axs[1].plot(state_log.sample_times(), phi * 180.0 / np.pi, ".")
    axs[2].plot(input_log.sample_times(), input_log.data()[0].transpose())

    # Set axis labels
    axs[0].set(ylabel=r"$\theta$ ($^\circ$)")
    axs[1].set(ylabel=r"$\phi$ ($^\circ$)")
    axs[2].set(ylabel="$V$ (V)", xlabel="Time (s)")
    plt.show()


if __name__ == "__main__":
    run_simulation()
