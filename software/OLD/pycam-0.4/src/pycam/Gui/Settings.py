# -*- coding: utf-8 -*-
"""
$Id: Settings.py 784 2010-10-16 10:08:35Z sumpfralle $

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

from pycam.Toolpath import Bounds
import pycam.Cutters
import pycam.Utils.log
import pycam.Toolpath
import ConfigParser
import StringIO
import os

CONFIG_DIR = "pycam"

log = pycam.Utils.log.get_logger()

def get_config_dirname():
    try:
        from win32com.shell import shellcon, shell            
        homedir = shell.SHGetFolderPath(0, shellcon.CSIDL_APPDATA, 0, 0)
        config_dir = os.path.join(homedir, CONFIG_DIR)
    except ImportError:
        # quick semi-nasty fallback for non-windows/win32com case
        homedir = os.path.expanduser("~")
        # hide the config directory for unixes
        config_dir = os.path.join(homedir, "." + CONFIG_DIR)
    if not os.path.isdir(config_dir):
        try:
            os.makedirs(config_dir)
        except OSError:
            config_dir = None
    return config_dir

def get_config_filename(filename=None):
    if filename is None:
        filename = "preferences.conf"
    config_dir = get_config_dirname()
    if config_dir is None:
        return None
    else:
        return os.path.join(config_dir, filename)


class Settings:

    GET_INDEX = 0
    SET_INDEX = 1
    VALUE_INDEX = 2
    
    def __init__(self):
        self.items = {}
        self.values = {}

    def add_item(self, key, get_func=None, set_func=None):
        self.items[key] = [None, None, None]
        self.define_get_func(key, get_func)
        self.define_set_func(key, set_func)
        self.items[key][self.VALUE_INDEX] = None

    def define_get_func(self, key, get_func=None):
        if not self.items.has_key(key):
            return
        if get_func is None:
            get_func = lambda: self.items[key][self.VALUE_INDEX]
        self.items[key][self.GET_INDEX] = get_func

    def define_set_func(self, key, set_func=None):
        if not self.items.has_key(key):
            return
        def default_set_func(value):
            self.items[key][self.VALUE_INDEX] = value
        if set_func is None:
            set_func = default_set_func
        self.items[key][self.SET_INDEX] = set_func

    def get(self, key, default=None):
        if self.items.has_key(key):
            return self.items[key][self.GET_INDEX]()
        else:
            return default

    def set(self, key, value):
        if not self.items.has_key(key):
            self.add_item(key)
        self.items[key][self.SET_INDEX](value)
        self.items[key][self.VALUE_INDEX] = value

    def has_key(self, key):
        """ expose the "has_key" function of the items list """
        return self.items.has_key(key)

    def __str__(self):
        result = {}
        for key in self.items.keys():
            result[key] = self.get(key)
        return str(result)

    def get_keys(self):
        return self.items.keys()


class ProcessSettings:

    BASIC_DEFAULT_CONFIG = """
[ToolDefault]
shape: CylindricalCutter
name: Cylindrical (d=3)
tool_radius: 1.5
torus_radius: 0.25
feedrate: 200
speed: 1000

[ProcessDefault]
name: Remove material
engrave_offset: 0.0
path_strategy: PushCutter
path_direction: x
milling_style: ignore
material_allowance: 0.0
step_down: 3.0
overlap_percent: 0

[BoundsDefault]
name: No Margin
type: relative_margin
x_low: 0.0
x_high: 0.0
y_low: 0.0
y_high: 0.0
z_low: 0.0
z_high: 0.0

[TaskDefault]
name: Default
enabled: yes
tool: 0
process: 0
bounds: 0
"""

    DEFAULT_CONFIG = """
[ToolDefault]
torus_radius: 0.25
feedrate: 200
speed: 1000

[Tool0]
name: Cylindrical (d=3)
shape: CylindricalCutter
tool_radius: 1.5

[Tool1]
name: Toroidal (d=2)
shape: ToroidalCutter
tool_radius: 1
torus_radius: 0.2

[Tool2]
name: Spherical (d=1)
shape: SphericalCutter
tool_radius: 0.5

[ProcessDefault]
path_direction: x
path_strategy: SurfaceStrategy
milling_style: ignore
engrave_offset: 0.0
step_down: 3.0
material_allowance: 0.0

[Process0]
name: Remove material
path_strategy: PushRemoveStrategy
material_allowance: 0.5
step_down: 3.0
overlap_percent: 0

[Process1]
name: Carve contour
path_strategy: ContourFollowStrategy
material_allowance: 0.2
step_down: 1.5
overlap_percent: 40

[Process2]
name: Cleanup
path_strategy: SurfaceStrategy
material_allowance: 0.0
overlap_percent: 60

[Process3]
name: Gravure
path_strategy: EngraveStrategy
step_down: 1.0
overlap_percent: 50

[BoundsDefault]
type: relative_margin
x_low: 0.0
x_high: 0.0
y_low: 0.0
y_high: 0.0
z_low: 0.0
z_high: 0.0

[Bounds0]
name: Minimum

[Bounds1]
name: 10% margin
x_low: 0.10
x_high: 0.10
y_low: 0.10
y_high: 0.10

[TaskDefault]
enabled: yes
bounds: 1

[Task0]
name: Rough
tool: 0
process: 0

[Task1]
name: Semi-finish
tool: 1
process: 1

[Task2]
name: Finish
tool: 2
process: 2

[Task3]
name: Gravure
enabled: no
tool: 2
process: 3

"""

    SETTING_TYPES = {
            "name": str,
            "shape": str,
            "tool_radius": float,
            "torus_radius": float,
            "speed": float,
            "feedrate": float,
            "path_strategy": str,
            "path_direction": str,
            "milling_style": str,
            "material_allowance": float,
            "overlap_percent": int,
            "step_down": float,
            "engrave_offset": float,
            "tool": object,
            "process": object,
            "bounds": object,
            "enabled": bool,
            "type": str,
            "x_low": float,
            "x_high": float,
            "y_low": float,
            "y_high": float,
            "z_low": float,
            "z_high": float,
    }

    CATEGORY_KEYS = {
            "tool": ("name", "shape", "tool_radius", "torus_radius", "feedrate",
                    "speed"),
            "process": ("name", "path_strategy", "path_direction",
                    "milling_style", "material_allowance",
                    "overlap_percent", "step_down", "engrave_offset"),
            "bounds": ("name", "type", "x_low", "x_high", "y_low",
                    "y_high", "z_low", "z_high"),
            "task": ("name", "tool", "process", "bounds", "enabled"),
    }

    SECTION_PREFIXES = {
        "tool": "Tool",
        "process": "Process",
        "task": "Task",
        "bounds": "Bounds",
    }

    DEFAULT_SUFFIX = "Default"
    REFERENCE_TAG = "_reference_"

    def __init__(self):
        self.config = None
        self._cache = {}
        self.reset()

    def reset(self, config_text=None):
        self._cache = {}
        self.config = ConfigParser.SafeConfigParser()
        if config_text is None:
            config_text = StringIO.StringIO(self.DEFAULT_CONFIG)
        else:
            # Read the basic default config first - in case some new options
            # are missing in an older config file.
            basic_default_config = StringIO.StringIO(self.BASIC_DEFAULT_CONFIG)
            self.config.readfp(basic_default_config)
            # Read the real config afterwards.
            config_text = StringIO.StringIO(config_text)
        self.config.readfp(config_text)

    def load_file(self, filename):
        try:
            content = file(filename).read()
        except IOError, err_msg:
            log.error("Settings: Failed to read config file '%s': %s" \
                    % (filename, err_msg))
            return False
        try:
            self.reset(content)
        except ConfigParser.ParsingError, err_msg:
            log.error("Settings: Failed to parse config file '%s': %s" \
                    % (filename, err_msg))
            return False
        return True

    def load_from_string(self, config_text):
        input_text = StringIO.StringIO(config_text)
        try:
            self.reset(input_text)
        except ConfigParser.ParsingError, err_msg:
            log.error("Settings: Failed to parse config data: %s" % \
                    str(err_msg))
            return False
        return True

    def write_to_file(self, filename, tools=None, processes=None, bounds=None,
            tasks=None):
        text = self.get_config_text(tools, processes, bounds, tasks)
        try:
            fi = open(filename, "w")
            fi.write(text)
            fi.close()
        except IOError, err_msg:
            log.error("Settings: Failed to write configuration to file " \
                    + "(%s): %s" % (filename, err_msg))
            return False
        return True

    def get_tools(self):
        return self._get_category_items("tool")

    def get_processes(self):
        return self._get_category_items("process")

    def _get_bounds_instance_from_dict(self, indict):
        """ get Bounds instances for each bounds definition
        @value model: the model that should be used for relative margins
        @type model: pycam.Geometry.Model.Model or callable
        @returns: list of Bounds instances
        @rtype: list(Bounds)
        """
        low_bounds = (indict["x_low"], indict["y_low"], indict["z_low"])
        high_bounds = (indict["x_high"], indict["y_high"], indict["z_high"])
        if indict["type"] == "relative_margin":
            bounds_type = Bounds.TYPE_RELATIVE_MARGIN
        elif indict["type"] == "fixed_margin":
            bounds_type = Bounds.TYPE_FIXED_MARGIN
        else:
            bounds_type = Bounds.TYPE_CUSTOM
        new_bound = Bounds(bounds_type, low_bounds, high_bounds)
        new_bound.set_name(indict["name"])
        return new_bound

    def get_bounds(self):
        return self._get_category_items("bounds")

    def get_tasks(self):
        return self._get_category_items("task")

    def _get_category_items(self, type_name):
        if not self._cache.has_key(type_name):
            item_list = []
            index = 0
            prefix = self.SECTION_PREFIXES[type_name]
            current_section_name = "%s%d" % (prefix, index)
            while current_section_name in self.config.sections():
                item = {}
                for key in self.CATEGORY_KEYS[type_name]:
                    value_type = self.SETTING_TYPES[key]
                    raw = value_type == str
                    try:
                        value_raw = self.config.get(current_section_name, key,
                                raw=raw)
                    except ConfigParser.NoOptionError:
                        try:
                            try:
                                value_raw = self.config.get(
                                        prefix + self.DEFAULT_SUFFIX, key, raw=raw)
                            except (ConfigParser.NoSectionError,
                                    ConfigParser.NoOptionError):
                                value_raw = None
                        except ConfigParser.NoOptionError:
                            value_raw = None
                    if not value_raw is None:
                        try:
                            if value_type == object:
                                # try to get the referenced object
                                value = self._get_category_items(key)[
                                        int(value_raw)]
                            elif value_type == bool:
                                if value_raw.lower() in (
                                        "1", "true", "yes", "on"):
                                    value = True
                                else:
                                    value = False
                            else:
                                # just do a simple type cast
                                value = value_type(value_raw)
                        except (ValueError, IndexError):
                            value = None
                        if not value is None:
                            item[key] = value
                if type_name == "bounds":
                    # don't add the pure dictionary, but the "bounds" instance
                    item_list.append(self._get_bounds_instance_from_dict(item))
                else:
                    item_list.append(item)
                index += 1
                current_section_name = "%s%d" % (prefix, index)
            self._cache[type_name] = item_list
        return self._cache[type_name][:]

    def _value_to_string(self, lists, key, value):
        value_type = self.SETTING_TYPES[key]
        if value_type == bool:
            if value:
                return "1"
            else:
                return "0"
        elif value_type == object:
            try:
                return lists[key].index(value)
            except ValueError:
                # special handling for non-direct object references ("bounds")
                for index, item in enumerate(lists[key]):
                    if (self.REFERENCE_TAG in item) \
                            and (value is item[self.REFERENCE_TAG]):
                        return index
                return None
        else:
            return str(value_type(value))

    def get_config_text(self, tools=None, processes=None, bounds=None,
            tasks=None):
        def get_dictionary_of_bounds(b):
            """ this function should be the inverse operation of 
            '_get_bounds_instance_from_dict'
            """
            result = {}
            result["name"] = b.get_name()
            bounds_type_num = b.get_type()
            if bounds_type_num == Bounds.TYPE_RELATIVE_MARGIN:
                bounds_type_name = "relative_margin"
            elif bounds_type_num == Bounds.TYPE_FIXED_MARGIN:
                bounds_type_name = "fixed_margin"
            else:
                bounds_type_name = "custom"
            result["type"] = bounds_type_name
            low, high = b.get_bounds()
            for index, axis in enumerate("xyz"):
                result["%s_low" % axis] = low[index]
                result["%s_high" % axis] = high[index]
            # special handler to allow tasks to track this new object
            result[self.REFERENCE_TAG] = b
            return result
        result = []
        if tools is None:
            tools = []
        if processes is None:
            processes = []
        if bounds is None:
            bounds = []
        if tasks is None:
            tasks = []
        lists = {}
        lists["tool"] = tools
        lists["process"] = processes
        lists["bounds"] = [get_dictionary_of_bounds(b) for b in bounds]
        lists["task"] = tasks
        for type_name in lists.keys():
            type_list = lists[type_name]
            # generate "Default" section
            common_keys = []
            for key in self.CATEGORY_KEYS[type_name]:
                try:
                    values = [item[key] for item in type_list]
                except KeyError:
                    values = None
                # check if there are values and if they all have the same value
                if values and (values.count(values[0]) == len(values)):
                    common_keys.append(key)
            if common_keys:
                section = "[%s%s]" % (self.SECTION_PREFIXES[type_name],
                        self.DEFAULT_SUFFIX)
                result.append(section)
                for key in common_keys:
                    value = type_list[0][key]
                    value_string = self._value_to_string(lists, key, value)
                    if not value_string is None:
                        result.append("%s: %s" % (key, value_string))
                # add an empty line to separate sections
                result.append("")
            # generate individual sections
            for index in range(len(type_list)):
                section = "[%s%d]" % (self.SECTION_PREFIXES[type_name], index)
                result.append(section)
                item = type_list[index]
                for key in self.CATEGORY_KEYS[type_name]:
                    if key in common_keys:
                        # skip keys, that are listed in the "Default" section
                        continue
                    if item.has_key(key):
                        value = item[key]
                        value_string = self._value_to_string(lists, key, value)
                        if not value_string is None:
                            result.append("%s: %s" % (key, value_string))
                # add an empty line to separate sections
                result.append("")
        return os.linesep.join(result)


class ToolpathSettings:

    SECTIONS = {
        "Bounds": {
            "minx": float,
            "maxx": float,
            "miny": float,
            "maxy": float,
            "minz": float,
            "maxz": float,
        },
        "Tool": {
            "shape": str,
            "tool_radius": float,
            "torus_radius": float,
            "speed": float,
            "feedrate": float,
        },
        "SupportGrid": {
            "type": str,
            "distance_x": float,
            "distance_y": float,
            "thickness": float,
            "height": float,
            "offset_x": float,
            "offset_y": float,
            "adjustments_x": "list_of_float",
            "adjustments_y": "list_of_float",
            "average_distance": float,
            "minimum_bridges": int,
            "length": float,
        },
        "Program": {
            "unit": str,
            "enable_ode": bool,
        },
        "Process": {
            "generator": str,
            "postprocessor": str,
            "path_direction": str,
            "material_allowance": float,
            "overlap": float,
            "step_down": float,
            "engrave_offset": float,
            "milling_style": str,
        },
    }

    META_MARKER_START = "PYCAM_TOOLPATH_SETTINGS: START"
    META_MARKER_END = "PYCAM_TOOLPATH_SETTINGS: END"

    def __init__(self):
        self.program = {}
        self.bounds = {}
        self.tool_settings = {}
        self.support_grid = {}
        self.process_settings = {}

    def set_bounds(self, bounds):
        low, high = bounds.get_absolute_limits()
        self.bounds = {
                "minx": low[0],
                "maxx": high[0],
                "miny": low[1],
                "maxy": high[1],
                "minz": low[2],
                "maxz": high[2],
        }

    def get_bounds(self):
        low = (self.bounds["minx"], self.bounds["miny"], self.bounds["minz"])
        high = (self.bounds["maxx"], self.bounds["maxy"], self.bounds["maxz"])
        return Bounds(Bounds.TYPE_CUSTOM, low, high)

    def set_tool(self, index, shape, tool_radius, torus_radius=None, speed=0.0,
            feedrate=0.0):
        self.tool_settings = {"id": index,
                "shape": shape,
                "tool_radius": tool_radius,
                "torus_radius": torus_radius,
                "speed": speed,
                "feedrate": feedrate,
        }

    def get_tool(self):
        return pycam.Cutters.get_tool_from_settings(self.tool_settings)

    def get_tool_settings(self):
        return self.tool_settings

    def set_support_grid(self, distance_x, distance_y, thickness, height,
            offset_x=0.0, offset_y=0.0, adjustments_x=None,
            adjustments_y=None):
        if adjustments_x is None:
            adjustments_x = []
        if adjustments_y is None:
            adjustments_y = []
        self.support_grid["type"] = "grid"
        self.support_grid["distance_x"] = distance_x
        self.support_grid["distance_y"] = distance_y
        self.support_grid["offset_x"] = offset_x
        self.support_grid["offset_y"] = offset_y
        self.support_grid["thickness"] = thickness
        self.support_grid["height"] = height
        self.support_grid["adjustments_x"] = adjustments_x
        self.support_grid["adjustments_y"] = adjustments_y

    def set_support_distributed(self, average_distance, minimum_bridges,
            thickness, height, length):
        self.support_grid["type"] = "distributed"
        self.support_grid["average_distance"] = average_distance
        self.support_grid["minimum_bridges"] = minimum_bridges
        self.support_grid["thickness"] = thickness
        self.support_grid["height"] = height
        self.support_grid["length"] = length

    def get_support_grid(self):
        result = {}
        if self.support_grid:
            options = self.support_grid
        else:
            options = {}
        # add all keys from the default list
        for key in self.SECTIONS["SupportGrid"].keys():
            if options.has_key(key):
                result[key] = options[key]
            else:
                result[key] = None
        return result

    def set_calculation_backend(self, backend=None):
        self.program["enable_ode"] = (backend.upper() == "ODE")

    def get_calculation_backend(self):
        if self.program.has_key("enable_ode"):
            if self.program["enable_ode"]:
                return "ODE"
            else:
                return None
        else:
            return None

    def set_unit_size(self, unit_size):
        self.program["unit"] = unit_size

    def get_unit_size(self):
        if self.program.has_key("unit"):
            return self.program["unit"]
        else:
            return "mm"

    def set_process_settings(self, generator, postprocessor, path_direction,
            material_allowance=0.0, overlap=0.0, step_down=1.0,
            engrave_offset=0.0, milling_style="ignore"):
        # TODO: this hack should be somewhere else, I guess
        if generator in ("ContourFollow", "EngraveCutter"):
            material_allowance = 0.0
            milling_style = "ignore"
        self.process_settings = {
                "generator": generator,
                "postprocessor": postprocessor,
                "path_direction": path_direction,
                "material_allowance": material_allowance,
                "overlap": overlap,
                "step_down": step_down,
                "engrave_offset": engrave_offset,
                "milling_style": milling_style,
        }

    def get_process_settings(self):
        return self.process_settings

    def parse(self, text):
        text_stream = StringIO.StringIO(text)
        config = ConfigParser.SafeConfigParser()
        config.readfp(text_stream)
        for config_dict, section in ((self.bounds, "Bounds"),
                (self.tool_settings, "Tool"),
                (self.support_grid, "SupportGrid"),
                (self.process_settings, "Process")):
            for key, value_type in self.SECTIONS[section].items():
                value_raw = config.get(section, key, None)
                if value_raw is None:
                    continue
                elif value_type == bool:
                    value = value_raw.lower() in ("1", "true", "yes", "on")
                elif isinstance(value_type, basestring) \
                        and (value_type.startswith("list_of_")):
                    item_type = value_type[len("list_of_"):]
                    if item_type == "float":
                        item_type = float
                    else:
                        continue
                    try:
                        value = [item_type(one_val)
                                for one_val in value_raw.split(",")]
                    except ValueError:
                        log.warn("Settings: Ignored invalid setting due to " \
                                + "a failed list type parsing: " \
                                + "(%s -> %s): %s" % (section, key, value_raw))
                else:
                    try:
                        value = value_type(value_raw)
                    except ValueError:
                        log.warn("Settings: Ignored invalid setting " \
                                + "(%s -> %s): %s" % (section, key, value_raw))
                config_dict[key] = value

    def __str__(self):
        return self.get_string()

    def get_string(self):
        result = []
        for config_dict, section in ((self.bounds, "Bounds"),
                (self.tool_settings, "Tool"),
                (self.support_grid, "SupportGrid"),
                (self.process_settings, "Process")):
            # skip empty sections
            if not config_dict:
                continue
            result.append("[%s]" % section)
            for key, value_type in self.SECTIONS[section].items():
                if config_dict.has_key(key):
                    value = config_dict[key]
                    if isinstance(value_type, basestring) \
                            and (value_type.startswith("list_of_")):
                        result.append("%s = %s" % (key,
                                ",".join([str(val) for val in value])))
                    elif type(value) == value_type:
                        result.append("%s = %s" % (key, value))
                # add one empty line after each section
            result.append("")
        return os.linesep.join(result)

