# setup script for py2exe to create the miniterm.exe
# $Id: setup-rfc2217_server-py2exe.py 320 2009-08-07 18:22:49Z cliechti $

from distutils.core import setup
import glob, sys, py2exe, os

sys.path.append('..')

sys.argv.extend("py2exe --bundle 1".split())

setup(
    name='rfc2217_server',
    zipfile=None,
    options = {"py2exe":
        {
            'dist_dir': 'bin',
            'excludes': ['javax.comm'],
            'compressed': 1,
        }
    },
    console = [
        "rfc2217_server.py",
    ],
)
