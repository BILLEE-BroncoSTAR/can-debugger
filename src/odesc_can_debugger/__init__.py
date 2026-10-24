"""ODrive CANSimple candump decoder and desktop viewer."""


def main() -> None:
    """Launch the Tk desktop app without importing Tk for decoder-only use."""
    from .app import main as run_app

    run_app()


__all__ = ["main"]
