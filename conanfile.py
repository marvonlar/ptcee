# Copyright (c) 2022 Norwegian Defence Research Establishment (FFI)

from conans import CMake, ConanFile, tools
from conans.tools import load, SystemPackageTool
import re

def get_version():
    try:
        return re.search(r"project\(\S+ VERSION (\d+\.\d+(\.\d+)?).*\)", load("CMakeLists.txt")).group(1).strip()
    except Exception:
        return None

class PTCeeConan(ConanFile):
    name = "ptcee"
    version = get_version()
    exports = "README.md"
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False],
    }
    default_options = {
        "shared":   False,
        "gtsam:rot3_expmap": True,
        "gtsam:use_quaternions": True,
        "gtsam:shared": False,
        "gtsam:install_cppunitlite": False,
    }
    generators = "cmake_paths", "cmake_find_package"
    exports_sources = "*", "!.gitlab-ci.yml"

    def requirements(self):
        self.requires("eigen/3.4.0")
        self.requires("gtsam/4.1.1")
        self.requires("sophus/22.04.1")

        self.requires("boost/1.79.0", override=True)

    def configure_cmake(self):
        cmake = CMake(self)
        cmake.definitions["BUILD_SHARED_LIBS"] = self.options.shared
        cmake.configure()
        return cmake

    def imports(self):
        self.copy("*.so*", src="@libdirs")

    def build(self):
        cmake = self.configure_cmake()
        cmake.build()

    def package(self):
        cmake = self.configure_cmake()
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = tools.collect_libs(self)
