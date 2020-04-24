# Copyright (c) 2020, CNRS
# Authors: Guilhem Saurel <guilhem.saurel@laas.fr>

import subprocess
import time


class ServerManager:
    """A context to ensure a server is running."""
    def __init__(self, server):
        self.server = server
        subprocess.run(['/usr/bin/killall', self.server])

    def __enter__(self):
        self.process = subprocess.Popen(self.server)
        # give it some time to start
        time.sleep(3)

    def __exit__(self, exc_type, exc_value, exc_traceback):
        self.process.kill()
