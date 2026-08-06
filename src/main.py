"""Module entrypoint for calibrated-wheeled-odometry."""

import asyncio

from viam.module.module import Module

# Importing the model registers it via EasyResource.__init_subclass__.
from models.wheeled import CalibratedWheeledOdometry  # noqa: F401


if __name__ == "__main__":
    asyncio.run(Module.run_from_registry())
