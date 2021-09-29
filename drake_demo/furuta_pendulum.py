""" Model of a Furuta pendulum
Written by: J. X. J. Bannwarth for the Melbourne Space Program.
"""
import numpy as np
import pydrake
from pydrake.systems.scalar_conversion import TemplateSystem


@TemplateSystem.define("FurutaPendulum_")
def FurutaPendulum_(T):
    """Furuta pendulum system for Drake

    Notes
    -----
    The Furuta pendulum is composed of two links: an arm rotating about a vertical
    axis and a pendulum rotating about the end of that arm. It is a highly nonlinear
    system that is well suited to testing out nonlinear control algorithms. The goal
    is typically to bring the pendulum to the upright (unstable) equilibrium point of
    the system and keep it there.

    The notation and equations used here are based on that presented in [1]_. The
    angle of the arm is defined as :math:`\phi`, and the angle of the pendulum
    is defined as :math:`\theta`. The angle of the pendulum is measured from the
    upright. I.e., when :math:`\theta=0`, the pendulum is at the upright unstable
    equilibrium point.

    The pendulum model is implemented using a Drake LeafSystem. LeafSystem objects are
    very powerful and essentially give you full control over the form of your simulation
    model. You can define the number and size of the inputs, outputs, states, and
    parameters (which is not done in this example). Moreover, you define the 'dynamics'
    of your system, which refer to the derivatives of the state variables.

    For full compatibility with Drake, LeafSystem objects have to be implemented in the
    form used here rather than as a standalone class like you would expect. This odd
    implementation is because LeafSystem objects need to support templated inputs, which
    allows for Drake to not only pass floating point numbers as input to the dynamics, but
    also symbolic variables and other objects. This capability allows Drake to get insight
    into the governing equations of your system, which in turn allows it to create
    analytical linearisations, dynamic-inversion controllers, among others.

    .. [1] Åkesson, J 1999, Safe Reference Following on the Inverted Pendulum.
       Technical Reports TFRT-7587, Department of Automatic Control,
       Lund Institute of Technology (LTH).
       https://portal.research.lu.se/portal/files/11127725/7587.pdf

    """

    class Impl(pydrake.systems.framework.LeafSystem_[T]):
        def _construct(self, converter=None) -> None:
            """Constructor method"""
            pydrake.systems.framework.LeafSystem_[T].__init__(self, converter)

            # 1 input: base torque
            self.DeclareVectorInputPort(
                "u", pydrake.systems.framework.BasicVector_[T](1)
            )

            # 4 states: [theta, phi, thetadot, phidot]
            state_index = self.DeclareContinuousState(2, 2, 0)

            # 4 outputs: all state variables
            # You need to pass a function to DeclareVectorOutputPort() that copies the
            # states
            self.DeclareVectorOutputPort(
                "state", pydrake.systems.framework.BasicVector_[T](4), self.CopyStateOut
            )

            # Raw parameters from Åkesson
            self.l = 0.413  # m
            self.r = 0.235  # m
            self.M = 0.01  # kg
            self.J = 0.05  # kgm^2
            self.Jp = 0.0009  # kgm^2
            self.m = 0.02  # kg
            self.g = 9.81  # m/s^2

            # 'Lumped' parameters
            self.alpha = self.Jp + self.M * self.l ** 2
            self.beta = self.J + self.M * self.r ** 2 + self.m * self.r ** 2
            self.gamma = self.M * self.r * self.l
            self.epsilon = self.l * self.g * (self.M + self.m / 2.0)

        def _construct_copy(self, other, converter=None) -> None:
            """Copy of constructor method (necessary for LeafSystem)"""
            Impl._construct(self, converter=converter)

        def CopyStateOut(
            self,
            context: pydrake.systems.framework.Context,
            output: pydrake.systems.framework.BasicVector,
        ) -> None:
            """
            Copy the system state

            Parameters
            ----------
            context : pydrake.systems.framework.Context
                Drake context
            output : pydrake.systems.framework.BasicVector
                Output vector to write the state to
            """
            x = context.get_continuous_state_vector().CopyToVector()
            y = output.SetFromVector(x)

        def DoCalcTimeDerivatives(
            self,
            context: pydrake.systems.framework.Context,
            derivatives: pydrake.systems.framework.ContinuousState,
        ) -> None:
            """Calculate time derivatives of the Furuta pendulum

            Notes
            -----
            The furuta pendulum equations of motion can be rewritten in the standard
            'manipulator equation' form:
            .. math::
                \mathbf{M}(\mathbf{q}) \ddot{\mathbf{q}}
                + \mathbf{C}(\mathbf{q},\dot{\mathbf{q}}) \dot{\mathbf{q}}
                + \mathbf{g}(\mathbf{q}) = \tau

            When implementing equations manually it is really helpful to create test
            functions to make sure they are correct. For example, you can check the
            linearisation of the system obtained using Drake to that derived
            analytically.

            Parameters
            ----------
            context : pydrake.systems.framework.Context
                Drake context within which to evaluate the derivatives
            derivatives : pydrake.systems.framework.ContinuousState
                ContinuousState object in which to write the derivatives
            """
            # Retrieve from context
            state = context.get_continuous_state_vector().CopyToVector()

            # CopyToVector() returns a numpy array even if it has a length of 1
            # If you want to use the value as a scalar, make sure to extract it
            # from the array first or you will run into issues
            u = self.EvalVectorInput(context, 0).CopyToVector()[0]

            # Extract parameters and states
            theta = state[0]
            phi = state[1]
            thetadot = state[2]
            phidot = state[3]

            # Group states
            qdot = np.array([thetadot, phidot])
            tau = np.array([0, u])

            # Calculate matrices
            M = np.array(
                [
                    [self.alpha, self.gamma * np.cos(theta)],
                    [
                        self.gamma * np.cos(theta),
                        self.beta + self.alpha * np.sin(theta) ** 2,
                    ],
                ]
            )
            C = np.array(
                [
                    [0, -self.alpha * phidot * np.sin(theta) * np.cos(theta)],
                    [
                        -self.gamma * thetadot * np.sin(theta),
                        2 * self.alpha * thetadot * np.sin(theta) * np.cos(theta),
                    ],
                ]
            )
            g = np.array([-self.epsilon * np.sin(theta), 0])

            # The determinant of M never goes to zero, so taking the inverse is
            # safe - but cannot use np.linalg.inv due to the way drake works
            det = (
                self.alpha * self.beta
                + self.alpha ** 2 * np.sin(theta) ** 2
                - self.gamma ** 2 * np.cos(theta) ** 2
            )
            M_inv = np.array([[M[1, 1], -M[0, 1]], [-M[1, 0], M[0, 0]]]) / det

            # Compute derivatives
            qddot = np.inner(M_inv, -np.inner(C, qdot) - g + tau)
            derivative_vector = derivatives.get_mutable_vector()
            derivative_vector.SetFromVector(np.concatenate((qdot, qddot)))

    return Impl


FurutaPendulum = FurutaPendulum_[None]  # Default instantiation
