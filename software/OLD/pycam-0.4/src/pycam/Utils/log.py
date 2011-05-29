# -*- coding: utf-8 -*-
"""
$Id: log.py 676 2010-09-16 15:54:49Z sumpfralle $

Copyright 2010 Lars Kruse <devel@sumpfralle.de>

This file is part of PyCAM.

PyCAM is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

PyCAM is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with PyCAM.  If not, see <http://www.gnu.org/licenses/>.
"""

import logging

def get_logger(suffix=None):
    name = "PyCAM"
    if suffix:
        name += ".%s" % str(suffix)
    logger = logging.getLogger(name)
    if len(logger.handlers) == 0:
        init_logger(logger)
    return logger

def init_logger(log, logfilename=None):
    if logfilename:
        datetime_format = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
        logfile_handler = logging.FileHandler(logfilename)
        logfile_handler.setFormatter(datetime_format)
        log.addHandler(logfile_handler)
    console_output = logging.StreamHandler()
    log.addHandler(console_output)
    log.setLevel(logging.INFO)

def add_stream(stream, level=None):
    log = get_logger()
    logstream = logging.StreamHandler(stream)
    if not level is None:
        logstream.setLevel(level)
    log.addHandler(logstream)

def add_hook(callback, level=None):
    log = get_logger()
    loghook = HookHandler(callback)
    if not level is None:
        loghook.setLevel(level)
    log.addHandler(loghook)

def add_gtk_gui(parent_window, level=None):
    log = get_logger()
    loggui = GTKHandler(parent_window)
    if not level is None:
        loggui.setLevel(level)
    log.addHandler(loggui)


class GTKHandler(logging.Handler):

    def __init__(self, parent_window=None, **kwargs):
        logging.Handler.__init__(self, **kwargs)
        self.parent_window = parent_window

    def emit(self, record):
        message = self.format(record).encode("utf-8")
        import gtk
        if record.levelno <= 20:
            message_type = gtk.MESSAGE_INFO
            message_title = "Information"
        elif record.levelno <= 30:
            message_type = gtk.MESSAGE_WARNING
            message_title = "Warning"
        else:
            message_type = gtk.MESSAGE_ERROR
            message_title = "Error"
        window = gtk.MessageDialog(self.parent_window, type=message_type,
                buttons=gtk.BUTTONS_OK, message_format=str(message))
        try:
            message_title = message_title.encode("utf-8")
        except UnicodeDecodeError:
            # remove all non-ascii characters
            message_title = "".join([char for char in message_title
                    if ord(char) < 128])
        window.set_title(message_title)
        window.run()
        window.destroy()

class HookHandler(logging.Handler):

    def __init__(self, callback, **kwargs):
        logging.Handler.__init__(self, **kwargs)
        self.callback = callback

    def emit(self, record):
        message = self.format(record)
        message_type = record.levelname
        self.callback(message_type, message, record=record)

