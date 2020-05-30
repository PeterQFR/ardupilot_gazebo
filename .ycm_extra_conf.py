# -*- coding: utf-8 -*-
import logging
import os
import subprocess
import yaml

# ROS Kinetic compatability (doesn't have python3 modules)
try:
    import rospkg
except ImportError:
    import shlex
    import xml.etree.ElementTree as et

    class _RosPkgResourceNotFound(BaseException):
        pass

    class RosPkg(object):

        def __init__(self):
            def _get_package_name(filename):
                """ Return name of package which contains filename """
                filename, unused = os.path.split(filename)
                while filename != os.path.sep:
                    package_xml = os.path.join(filename, 'package.xml')
                    if os.path.exists(package_xml):
                        tree = et.parse(package_xml)
                        package = tree.find('name').text
                        return package
                    filename, unused = os.path.split(filename)
                return ''

            self.get_package_name = _get_package_name
            self.ResourceNotFound = _RosPkgResourceNotFound

        class RosPack(object):

            def list(self):
                """ Return list of packages """
                packages = subprocess.check_output(shlex.split('rospack list-names')).split()
                return [package.decode('utf-8') for package in packages]

            def get_path(self, package):
                """ Return path to package. """
                return subprocess.check_output(shlex.split('rospack find %s' % package)).decode('utf-8')

    rospkg = RosPkg()

import ycm_core

LOGGER = logging.getLogger('vim-ros-ycm')

SOURCE_EXTENSIONS = ['.cpp', '.cxx', '.cc', '.c', '.m', '.mm']
LAST_CWD = None
ROS_WORKSPACE = None
ROS_WORKSPACE_FLAGS = None

# These are the compilation flags that will be used in case there's no
# compilation database set (by default, one is not set).
# You can get CMake to generate the compilation_commands.json file for you by
# adding:
#   set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
# to your CMakeLists.txt file or by once entering
#   catkin config --cmake-args '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON'
# in your shell.
DEFAULT_FLAGS = [
    '-Wall',
    '-Wextra',
    '-Werror',
    '-Wc++98-compat',
    '-Wno-long-long',
    '-Wno-variadic-macros',
    '-fexceptions',
    '-DNDEBUG',
    # THIS IS IMPORTANT! Without a "-std=<something>" flag, clang won't know
    # which language to use when compiling headers. So it will guess. Badly. So
    # C++ headers will be compiled as C headers. You don't want that so ALWAYS
    # specify a "-std=<something>".
    # For a C project, you would set this to something like 'c99' instead.
    '-std=c++14',
    # ...and the same thing goes for the magic -x option which specifies the
    # language that the files to be compiled are written in. This is mostly
    # relevant for c++ headers.
    # For a C project, you would set this to 'c' instead of 'c++'.
    '-x',
    'c++',
    '-I',
    '.',
    '-isystem/usr/include/ignition/gazebo3',
    '-isystem/usr/include/sdformat-9.2'
]

def DefaultFlags():
    global DEFAULT_FLAGS
    global ROS_WORKSPACE_FLAGS
    if ROS_WORKSPACE_FLAGS:
        return ROS_WORKSPACE_FLAGS + DEFAULT_FLAGS
    return DEFAULT_FLAGS


def GetDevelSpace(ros_workspace):
    """Return path to the devel space, relative to ros_workspace.
    """

    with open(os.path.join(ros_workspace, '.catkin_tools/profiles/profiles.yaml')) as f:
        profiles_yaml = yaml.load(f)
    active_profile_name = profiles_yaml['active']

    with open(os.path.join(ros_workspace, '.catkin_tools/profiles', active_profile_name, 'config.yaml')) as f:
        profile_yaml = yaml.load(f)

    devel_space = profile_yaml['devel_space']

    return devel_space

def GetProfile(ros_workspace):

    with open(os.path.join(ros_workspace, '.catkin_tools/profiles/profiles.yaml')) as f:
        profiles_yaml = yaml.load(f)
    active_profile_name = profiles_yaml['active']
    return active_profile_name

def GetRosIncludePaths(ros_workspace):
    """Return a list of potential include directories

    The directories are looked for in $ROS_WORKSPACE.
    """
    rospack = rospkg.RosPack()
    includes = []
    includes.append(os.path.join(ros_workspace, GetDevelSpace(ros_workspace)))
    for p in rospack.list():
        rospack_path = os.path.join(rospack.get_path(p), 'include')
        if os.path.exists(rospack_path):
            includes.append(rospack_path)
    for distribution in os.listdir('/opt/ros'):
        includes.append('/opt/ros/' + distribution + '/include')
    return includes


def GetRosIncludeFlags(ros_workspace):
    includes = GetRosIncludePaths(ros_workspace)
    flags = []
    for include in includes:
        flags.append('-isystem')
        flags.append(include)
    return flags


def DirectoryOfThisScript():
    return os.path.dirname(os.path.abspath(__file__))


def UpdateRosWorkspace():
    global DEFAULT_FLAGS
    global FLAGS
    global LAST_CWD
    global ROS_WORKSPACE
    cwd = os.getcwd()
    if not LAST_CWD or cwd != LAST_CWD:
        LAST_CWD = cwd
        ROS_WORKSPACE = None
        ROS_WORKSPACE_FLAGS = None
        try:
            ROS_WORKSPACE = subprocess.check_output(['catkin', 'locate'],
                                                    stderr=subprocess.STDOUT).strip().decode('utf-8')
        except subprocess.CalledProcessError as e:
            if e.returncode == 1:
                LOGGER.info('catkin locate error: %s', result.output)
            else:
                LOGGER.error(
                    'Unknown error. Is catkin-tools installed? '
                    '(sudo apt install python-catkin-tools): %s',
                    result.output)
        if ROS_WORKSPACE is not None:
            ROS_WORKSPACE_FLAGS = MakeRelativePathsInFlagsAbsolute(
                DEFAULT_FLAGS + GetRosIncludeFlags(ROS_WORKSPACE),
                DirectoryOfThisScript())
        LOGGER.info('ROS workspace updated: %s', ROS_WORKSPACE)


def MakeRelativePathsInFlagsAbsolute(flags, working_directory):
    if not working_directory:
        return list(flags)
    new_flags = []
    make_next_absolute = False
    path_flags = ['-isystem', '-I', '-iquote', '--sysroot=']
    for flag in flags:
        new_flag = flag
        if make_next_absolute:
            make_next_absolute = False
            if not flag.startswith('/'):
                new_flag = os.path.join(working_directory, flag)
        for path_flag in path_flags:
            if flag == path_flag:
                make_next_absolute = True
                break
            if flag.startswith(path_flag):
                path = flag[len(path_flag):]
                new_flag = path_flag + os.path.join(working_directory, path)
                break
        new_flags.append(new_flag)
    return new_flags


def GetCompilationDatabaseFolder(ros_workspace, filename, profile):
    """Return the directory potentially containing compilation_commands.json

    Return the absolute path to the folder (NOT the file!) containing the
    compile_commands.json file to use that instead of 'flags'. See here for
    more details: http://clang.llvm.org/docs/JSONCompilationDatabase.html.
    The compilation_commands.json for the given file is returned by getting
    the package the file belongs to.
    """
    if profile not in ['default']:
        pkg_name = rospkg.get_package_name(filename)
        if not pkg_name:
            return ''
        return os.path.join(ros_workspace, 'build_{}'.format(profile), pkg_name)
    else:
        pkg_name = rospkg.get_package_name(filename)
        if not pkg_name:
            return ''
        return os.path.join(ros_workspace, 'build', pkg_name)


def GetDatabase(compilation_database_folder):
    if os.path.exists(compilation_database_folder):
        if not hasattr(ycm_core, 'CompilationDatabase'):
            raise RuntimeError(
                'YouCompleteMe must be compiled with the --clang-completer flag'
            )
        return ycm_core.CompilationDatabase(compilation_database_folder)
    return None


def IsHeaderFile(filename):
    extension = os.path.splitext(filename)[1]
    return extension in ['.h', '.hxx', '.hpp', '.hh']


def GetCompilationInfoForHeaderSameDir(headerfile, database):
    """Return compile flags for src file with same base in the same directory
    """
    filename_no_ext = os.path.splitext(headerfile)[0]
    for extension in SOURCE_EXTENSIONS:
        replacement_file = filename_no_ext + extension
        if os.path.exists(replacement_file):
            compilation_info = database.GetCompilationInfoForFile(
                replacement_file)
            if compilation_info.compiler_flags_:
                return compilation_info
    return None


def GetCompilationInfoForHeaderRos(headerfile, database):
    """Return the compile flags for the corresponding src file in ROS

    Return the compile flags for the source file corresponding to the header
    file in the ROS where the header file is.
    """
    pkg_name = rospkg.get_package_name(headerfile)
    if not pkg_name:
        return None
    try:
        pkg_path = rospkg.RosPack().get_path(pkg_name)
    except rospkg.ResourceNotFound:
        return None
    filename_no_ext = os.path.splitext(headerfile)[0]
    hdr_basename_no_ext = os.path.basename(filename_no_ext)
    for path, dirs, files in os.walk(pkg_path):
        for src_filename in files:
            src_basename_no_ext = os.path.splitext(src_filename)[0]
            if hdr_basename_no_ext != src_basename_no_ext:
                continue
            for extension in SOURCE_EXTENSIONS:
                if src_filename.endswith(extension):
                    compilation_info = database.GetCompilationInfoForFile(
                        os.path.join(path, src_filename))
                    if compilation_info.compiler_flags_:
                        return compilation_info
    return None


def GetCompilationInfoForFile(filename, database):
    # The compilation_commands.json file generated by CMake does not have
    # entries for header files. So we do our best by asking the db for flags
    # for a corresponding source file, if any. If one exists, the flags for
    # that file should be good enough.
    # Corresponding source file are looked for in the same package.
    if IsHeaderFile(filename):
        # Look in the same directory.
        compilation_info = GetCompilationInfoForHeaderSameDir(
            filename, database)
        if compilation_info:
            return compilation_info
        # Look in the package.
        compilation_info = GetCompilationInfoForHeaderRos(filename, database)
        if compilation_info:
            return compilation_info
    return database.GetCompilationInfoForFile(filename)


def Settings(**kwargs):
    filename = kwargs['filename']
    language = kwargs['language']
    if language != 'cfamily':
        return {}
    try:
        flags = DefaultFlags()
        return {'flags': flags, 'do_cache': True}
    except Exception as e:
        LOGGER.error('Unhandled exception', exc_info=True)
        raise
