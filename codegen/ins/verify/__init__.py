"""Independent checks of the emitted INS covariance kernel.

Keep reference equations and source validation independent of the generator.
"""


def require(CONDITION, MESSAGE):
    """Reject invalid evidence even when Python assertions are disabled."""
    if not CONDITION:
        raise ValueError(MESSAGE)
