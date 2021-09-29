"""Controllers for Furuta pendulum
Written by: J. X. J. Bannwarth for the Melbourne Space Program.
"""
from copy import copy
from typing import Union
import numpy as np
import pydrake
import pydrake.systems.primitives
import pydrake.systems.controllers

from drake_demo.furuta_pendulum import FurutaPendulum


def stabilising_controller(
    furuta: FurutaPendulum,
) -> pydrake.systems.primitives.MatrixGain:
    """
    Analytically derived full-state feedback controller to stabilise the upright equilibrium

    Notes
    -----
    The controller, derived by Åkesson [1]_, takes the form of a full-state feedback (FSFB) law:
    .. math::u = - \mathbf{K} \mathbf{x},
    where :math:`\mathbf{K}` is the FSFB gain matrix and
    .. math::\mathbf{x} = \begin{bmatrix}\mathbf{q}&\dot{\mathbf{q}}\end{matrix}
    is the system state, which consists of the two angular positions and velocities.

    The elements of the gain matrix were analytically derived to allow for pole placement
    of the system linearised around a desired arm angular speed :math:`\dot{\phi}_0`.
    This placement is achieved by determining the natural frequency and damping ratio of
    each link.

    Note that the sign used in the control law in the original paper is incorrect. The
    equations presented here assume negative feedback.

    This is a linear controller and will only stabilise the closed-loop system within the
    region of attraction of its upright equilibrium point.

    .. [1]_ Åkesson, J 1999, Safe Reference Following on the Inverted Pendulum.
       Technical Reports TFRT-7587, Department of Automatic Control,
       Lund Institute of Technology (LTH).
       https://portal.research.lu.se/portal/files/11127725/7587.pdf

    Parameters
    ----------
    furuta : FurutaPendulum
        Drake system of the furuta pendulum

    Returns
    -------
    pydrake.systems.primitives.MatrixGain
        The feedback gain matrix `K`
    """
    # Pendulum parameters
    alpha = furuta.alpha
    beta = furuta.beta
    gamma = furuta.gamma
    epsilon = furuta.epsilon

    # Tuning variables - values obtained from Åkesson
    phidot0 = 0.0
    omega0 = 8.5
    omega1 = 8.5
    damping0 = 0.9
    damping1 = 0.9

    # Elements of the gain matrix retrieved form the paper
    K0 = (
        -(alpha * beta * phidot0 ** 2 + beta * epsilon) / gamma
        - (alpha * beta - gamma ** 2)
        * (
            alpha * (omega0 ** 2) * (omega1 ** 2) / (alpha * phidot0 ** 2 + epsilon)
            + omega0 ** 2
            + 4.0 * damping0 * damping1 * omega0 * omega1
            + omega1 ** 2
        )
        / gamma
    )
    K1 = (
        -(alpha * beta - gamma ** 2)
        * omega0 ** 2
        * omega1 ** 2
        / (alpha * phidot0 ** 2 + epsilon)
    )
    K2 = (
        -(alpha * beta - gamma ** 2)
        / gamma
        * (
            2.0
            * alpha
            * (damping1 * omega0 ** 2 * omega1 + damping0 * omega1 ** 2 * omega0)
            / (alpha * phidot0 ** 2 + epsilon)
            + 2.0 * damping0 * omega0
            + 2.0 * damping1 * omega1
        )
    )
    K3 = (
        -(alpha * beta - gamma ** 2)
        * (
            2.0 * damping1 * omega0 ** 2 * omega1
            + 2.0 * damping0 * omega1 ** 2 * omega0
        )
        / (alpha * phidot0 ** 2 + epsilon)
    )
    return pydrake.systems.primitives.MatrixGain(D=np.array([[K0, K1, K2, K3]]))


def lqr_controller(furuta: FurutaPendulum) -> Union[np.ndarray, np.ndarray]:
    """
    Linear quadratic regulator for catching the pendulum at the upright position

    Notes
    -----
    For an introduction to linear quadratic control, see [1]_. The control law
    used is linear and is the same as that used by the analytical FSFB controller:
    .. math::u = - \mathbf{K} \mathbf{x}.
    However, the way the `\mathbf{K}` matrix is obtained is different. Here, it
    minimises a cost function in an optimal way.

    The :math:`\mathbf{Q}` and :math:`R` weights are chosen arbitrarily. For
    real applications, some tuning will be required.

    .. [1] Russ Tedrake. Linear Quadratic Regulators. In: Underactuated Robotics:
       Algorithms for Walking, Running, Swimming, Flying, and Manipulation
       (Course Notes for MIT 6.832).
       http://underactuated.mit.edu/lqr.html

    Parameters
    ----------
    furuta : FurutaPendulum
        Drake system of the Furuta pendulum

    Returns
    -------
    Union[np.ndarray, np.ndarray]
        The gain matrix `K` and the solution of the algebraic Riccati equation
        used to obtain it, `S`
    """
    context = furuta.CreateDefaultContext()

    furuta.get_input_port(0).FixValue(context, [0])
    context.SetContinuousState([0.0, 0.0, 0.0, 0.0])

    # Arbitrarily chosen values
    Q = np.diag((10.0, 0.1, 0.1, 0.1))
    R = [1]

    # Get a linearised model of the plant
    linearised_furuta = pydrake.systems.primitives.Linearize(furuta, context)
    (K, S) = pydrake.systems.controllers.LinearQuadraticRegulator(
        linearised_furuta.A(), linearised_furuta.B(), Q, R
    )
    return (K, S)


class SwingUpController(pydrake.systems.framework.VectorSystem):
    """Swing-up controller to bring the pendulum to the upright position

    Notes
    -----
    Use the energy controller presented in [1]_:
    .. math::u = \mathrm{sat}\left(k(E_n - E_0) \mathrm{sign}(\dot{\theta}\cos\theta)\right),
    where the energy is defined as
    .. math:: E_n = \frac{\dot{\theta}^2}{2\omega_0^2} + \cos\theta - 1.

    Due to actuation limits, if the pendulum starts at the bottom equilibrium point, it will not
    be able to go to the top equilibrium point in a straight path. Instead, it will need to rock
    back and forth and build up its kinetic energy until it is sufficient to reach the upright
    position.

    The energy controller is an elegant way to 'pump' kinetic energy into the system. This is an
    example of nonlinear control.

    .. [1]_ Åkesson, J 1999, Safe Reference Following on the Inverted Pendulum.
       Technical Reports TFRT-7587, Department of Automatic Control,
       Lund Institute of Technology (LTH).
       https://portal.research.lu.se/portal/files/11127725/7587.pdf

    Parameters
    ----------
    omega_0 : float
        Desired natural frequency
    k : float
        Proportional gain applied on the energy error
    u_min : float
        Lower saturation limit of the actuator
    u_max : float
        Upper saturation limit of the actuator
    """

    def __init__(self, omega_0, k=100.0, u_min=-1.0, u_max=1.0) -> None:
        """Constructor method"""
        # Define input/output sizes: input is 4 elements (2 angles + 2 angular
        # velocities), output is 1 element (torque)
        pydrake.systems.framework.VectorSystem.__init__(self, 4, 1)
        self.omega_0 = omega_0
        self.k = k
        self.u_min = u_min
        self.u_max = u_max

    def DoCalcVectorOutput(
        self, context: pydrake.systems.framework.Context, pendulum_state, _, output
    ) -> None:
        """
        Calculate the output of the controller given the state of the pendulum

        Notes
        -----
        The inputs to the function are standard and any `pydrake.systems.framework.VectorSystem`
        should have them.

        Parameters
        ----------
        context : pydrake.systems.framework.Context
            Drake context.
        pendulum_state
            State of the furuta pendulum
        _
            Placeholder for compatibility
        output
            Output of the controller to populate
        """
        # Retrieve states
        theta = pendulum_state[0]
        thetadot = pendulum_state[2]

        # Calculate the energy of the pendulum using linear cart-pole model approximation
        energy = thetadot ** 2 / (2.0 * self.omega_0 ** 2) + np.cos(theta) - 1.0

        # Write to the output vector
        output[:] = np.clip(
            self.k * energy * np.sign(thetadot * np.cos(theta)), self.u_min, self.u_max
        )


class SwingUpAndCatchController(pydrake.systems.framework.VectorSystem):
    """
    Combined controller to swing the pendulum up and catch it at the top

    Notes
    -----
    This controller combines both the swing-up controller and the LQR
    controller. Switching between the two is performed using the threshold
    highlighted in [1]_, but it could also be done based on a rule of thumb
    such as 'if the angle from the top is less than X degrees, switch to the
    LQR controller'.

    .. [1] Russ Tedrake. The Simple Pendulum. In: Underactuated Robotics:
       Algorithms for Walking, Running, Swimming, Flying, and Manipulation
       (Course Notes for MIT 6.832).
       http://underactuated.mit.edu/pend.html

    Parameters
    ----------
    pendulum : FurutaPendulum
        Drake system of Furuta pendulum
    """

    def __init__(self, furuta: FurutaPendulum) -> None:
        """Constructor method"""
        # Define input/output sizes: input is 4 elements (2 angles + 2 angular
        # velocities), output is 1 element (torque)
        pydrake.systems.framework.VectorSystem.__init__(self, 4, 1)

        # Create the two controllers
        (self.K, self.S) = lqr_controller(furuta)
        self.swing_up = SwingUpController(omega_0=8.5)
        self.swing_up_context = self.swing_up.CreateDefaultContext()

    def DoCalcVectorOutput(
        self, context: pydrake.systems.framework.Context, pendulum_state, _, output
    ) -> None:
        """
        Calculate the controller output given the state of the pendulum

        Notes
        -----
        The inputs to the function are standard and any `pydrake.systems.framework.VectorSystem`
        should have them.

        Parameters
        ----------
        context : pydrake.systems.framework.Context
            Drake context.
        pendulum_state
            State of the furuta pendulum
        _
            Placeholder for compatibility
        output
            Output of the controller to populate
        """
        # Retrieve state and wrap angles from -pi to pi
        # This wrapping is necessary, because even though pi = 3pi = 5pi from
        # an orientation point of view, if you apply the same linear gain to 1pi
        # or 5pi you will get very different torques
        xbar = copy(pendulum_state)
        xbar[0] = pydrake.math.wrap_to(xbar[0], -np.pi, np.pi)
        xbar[1] = pydrake.math.wrap_to(xbar[1], -np.pi, np.pi)

        # If x'Sx <= 2, then use the LQR controller
        if xbar.dot(self.S.dot(xbar)) < 2.0:
            output[:] = -self.K.dot(xbar)
        else:
            # While the combined controller is connected to the rest of the Drake
            # diagram, the swing-up controller alone is not; it is essentially
            # floating. We therefore need to manually give it inputs and evaluate
            # its outputs
            self.swing_up.get_input_port(0).FixValue(
                self.swing_up_context, pendulum_state
            )
            output[:] = self.swing_up.get_output_port(0).Eval(self.swing_up_context)
