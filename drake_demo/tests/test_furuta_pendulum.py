"""Tests for furuta pendulum model
Written by: J. X. J. Bannwarth for the Melbourne Space Program.
"""
from typing import Union
import numpy as np
import pydrake
import pytest

from drake_demo.furuta_pendulum import FurutaPendulum
from drake_demo.controllers import stabilising_controller


def analytical_linearization(
    alpha: float, beta: float, gamma: float, epsilon: float, phidot0: float
) -> Union[np.ndarray, np.ndarray]:
    """
    Calculate analytical linearisation of the Furuta pendulum dynamics

    Notes
    -----
    Linearising a system refers to taking a nonlinear system in the form
    .. math::\dot{\mathbf{x}} = \mathbf{f}(\mathbf{x}, \mathbf{u}),
    where :math:`\mathbf{x}` and :math:`\mathbf{u}` are the state and input,
    respectively, and approximating it into the form
    .. math::\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u},
    where :math:`\mathbf{A}` and :math:`\mathbf{B}` are matrices of constants.

    The analytical linearisation of the Furuta pendulum around the upright
    equilibrium is given in [1]_.

    Comparing the analytical linearisation with that obtained using Drake (or
    another tool such as MATLAB) is a great way to check if you implemented
    the nonlinear equations properly. Ideally, you would compare the
    linearisation at a variety of points.

    .. [1] Åkesson, J 1999, Safe Reference Following on the Inverted Pendulum.
       Technical Reports TFRT-7587, Department of Automatic Control,
       Lund Institute of Technology (LTH).
       https://portal.research.lu.se/portal/files/11127725/7587.pdf

    Parameters
    ----------
    alpha : float
        Lumped parameter `alpha`
    beta : float
        Lumped parameter `beta`
    gamma : float
        Lumped parameter `gamma`
    epsilon : float
        Lumped parameter `epsilon`
    phidot0 : float
        Angular velocity of the arm at which to linearise the system

    Returns
    -------
    Union[np.ndarray, np.ndarray]
        State matrix `A` and input matrix `B`
    """
    A02 = (alpha * beta * phidot0 ** 2 + beta * epsilon) / (alpha * beta - gamma ** 2)
    A03 = (-alpha * beta * phidot0 ** 2 - gamma * epsilon) / (alpha * beta - gamma ** 2)
    A = np.array([[0, 0, 1, 0], [0, 0, 0, 1], [A02, 0, 0, 0], [A03, 0, 0, 0]])
    B = np.array(
        [
            [0],
            [0],
            [-gamma / (alpha * beta - gamma ** 2)],
            [alpha / (alpha * beta - gamma ** 2)],
        ]
    )
    return (A, B)


def drake_linearization(
    system, state0: list, input0: list
) -> pydrake.systems.primitives.LinearSystem:
    """
    Use Drake to linearise a system

    Parameters
    ----------
    system
        Drake system to linearise
    state0 : list
        Linearisation point (state)
    input0 : list
        Linearisation point (input)

    Returns
    -------
    linearised_system : pydrake.systems.primitives.LinearSystem
        Linearised system
    """
    # Create system
    context = system.CreateDefaultContext()
    system.get_input_port(0).FixValue(context, input0)
    context.SetContinuousState(state0)

    # Linearise the system
    linearised_system = pydrake.systems.primitives.Linearize(system, context)
    return linearised_system


def test_FurutaPendulum_linearization() -> None:
    """
    Test the implementation of the `FurutaPendulum` dynamics by comparing linearisations

    Notes
    -----
    Writing tests such as this one is highly recommended to check that your functions and
    classes are implemented properly.
    Simply call the function you want to test and compare its output with an output you
    calculated by hand or using another implementation that you know is correct using
    assertions, such as `assert` or `np.testing.assert_almost_equal`. If your implementation
    is correct, running `pytest` should return no warnings. If not, you will know where to
    look for debugging.
    """
    # Create pendulum system
    furuta_system = FurutaPendulum()
    furuta_system.set_name("furuta_pendulum")

    # Get the two linearisations
    phidot0 = 0
    linearised_system = drake_linearization(
        furuta_system, [0.0, 0.0, 0.0, phidot0], [0]
    )
    A_expected, B_expected = analytical_linearization(
        furuta_system.alpha,
        furuta_system.beta,
        furuta_system.gamma,
        furuta_system.epsilon,
        phidot0,
    )

    # Check that the A and B matrices are the same for both linearisation
    # methods
    # Note that we use `assert_almost_equal()` because numerical rounding
    # might make `assert X == Y` to fail
    np.testing.assert_almost_equal(linearised_system.B(), B_expected)
    np.testing.assert_almost_equal(linearised_system.A(), A_expected)


def test_stabilising_controller() -> None:
    """
    Test `stabilising_controller()` by checking it stabilises the closed-loop linearised system

    Notes
    -----
    For control system applications, you can write tests such as this one to automatically check
    if your controller meets your control requirements. Stabilising the system is a basic and
    essential requirement, but you could write more specific tests for settling time, RMS error,
    disturbance rejection requirements, among others.

    If you apply a full-state feedback control law
    .. math::\mathbf{u} = - \mathbf{K} \mathbf{x}
    to a linear system
    .. math::\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u},
    the poles of the resulting closed-loop system are the eigenvalues of
    .. math::(\mathbf{A} - \mathbf{B}\mathbf{K}).

    If all the poles of the system are in the left hand plane, the (linearised)
    system is stable.
    """
    # Create linearised system
    furuta_system = FurutaPendulum()
    furuta_system.set_name("furuta_pendulum")
    phidot0 = 0
    linearised_system = drake_linearization(
        furuta_system, [0.0, 0.0, 0.0, phidot0], [0]
    )
    controller_system = stabilising_controller(furuta_system)

    # Note: in numpy, B.dot(D), where B and D are matrices, represents matrix multiplication
    poles, _ = np.linalg.eig(
        linearised_system.A() - linearised_system.B().dot(controller_system.D())
    )

    # If the controller stabilises the linearized model, all closed-loop poles should be in the
    # left-half plane
    assert all(poles < 0)
