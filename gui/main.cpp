// gui/main.cpp — application entry point.

#include <QApplication>
#include <QFont>
#include <QSurfaceFormat>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

#include "main_window.h"

namespace {

// ':'-joined list of the paths that actually exist. On Windows the natural
// list separator for these env vars is ';' (OpenEB splits on the platform
// separator), so pick it per-platform.
std::string path_list(const std::vector<std::filesystem::path> &paths) {
#ifdef _WIN32
    const char sep = ';';
#else
    const char sep = ':';
#endif
    std::string value;
    for (const auto &path : paths) {
        if (path.empty() || !std::filesystem::exists(path)) {
            continue;
        }
        if (!value.empty()) {
            value += sep;
        }
        value += path.string();
    }
    return value;
}

void set_env_if_unset(const char *name, const std::string &value) {
    if (!std::getenv(name) && !value.empty()) {
#ifdef _WIN32
        ::_putenv_s(name, value.c_str());
#else
        ::setenv(name, value.c_str(), 0);
#endif
    }
}

#ifdef __APPLE__
std::filesystem::path executable_path(const char *argv0) {
    uint32_t size = 0;
    _NSGetExecutablePath(nullptr, &size);
    std::string buffer(size, '\0');
    if (_NSGetExecutablePath(buffer.data(), &size) == 0) {
        buffer.resize(std::strlen(buffer.c_str()));
        return std::filesystem::weakly_canonical(buffer);
    }
    return std::filesystem::weakly_canonical(argv0);
}

std::filesystem::path bundle_contents_dir(const char *argv0) {
    const auto exe_dir = executable_path(argv0).parent_path();
    if (exe_dir.filename() == "MacOS" && exe_dir.parent_path().filename() == "Contents") {
        return exe_dir.parent_path();
    }
    return {};
}
#endif

#ifdef _WIN32
std::filesystem::path executable_dir(const char *argv0) {
    char buffer[MAX_PATH];
    const DWORD len = ::GetModuleFileNameA(nullptr, buffer, MAX_PATH);
    if (len > 0 && len < MAX_PATH) {
        return std::filesystem::weakly_canonical(buffer).parent_path();
    }
    return std::filesystem::weakly_canonical(argv0).parent_path();
}
#endif

} // namespace

// Sets default OpenEB runtime env vars only if the user has not already
// configured them.  This allows users with non-standard HAL plugin locations
// (e.g. CenturyArks cameras) to simply export MV_HAL_PLUGIN_PATH in their
// shell and have the GUI respect it.
static void ensure_openeb_env_defaults(const char *argv0) {
#ifdef __APPLE__
    const auto contents_dir = bundle_contents_dir(argv0);
    // The app bundle is self-contained: plugins live in Contents/PlugIns.
    // Avoid OpenEB loading this directory a second time through its
    // installation-path fallback.
    set_env_if_unset("MV_HAL_PLUGIN_PATH",
                     path_list({contents_dir / "PlugIns" / "metavision" / "hal" / "plugins"}));
    set_env_if_unset("MV_HAL_PLUGIN_SEARCH_MODE", "PLUGIN_PATH_ONLY");
    set_env_if_unset("MV_HAL_INSTALL_PATH",
                     path_list({contents_dir / "Resources" / "metavision" / "hal"}));
    set_env_if_unset("HDF5_PLUGIN_PATH",
                     path_list({
                         contents_dir / "PlugIns" / "hdf5",
                         "/opt/homebrew/lib/hdf5/plugin",
                         "/usr/local/lib/hdf5/plugin",
                     }));
#elif defined(_WIN32)
    // Windows install layout: the installer (or a zip deployment) puts the
    // HAL plugins next to the exe under metavision\hal\plugins. Homebrew-style
    // absolute defaults make no sense here, so fall back to exe-relative and
    // then to the standard Prophesee install location.
    const auto exe_dir = executable_dir(argv0);
    set_env_if_unset("MV_HAL_PLUGIN_PATH",
                     path_list({
                         exe_dir / "metavision" / "hal" / "plugins",
                         "C:/Program Files/Prophesee/lib/metavision/hal/plugins",
                     }));
    set_env_if_unset("HDF5_PLUGIN_PATH",
                     path_list({
                         exe_dir / "hdf5" / "plugin",
                         "C:/Program Files/Prophesee/share/hdf5/plugin",
                     }));
    // Windows defaults to the direct3d RHI backend; force OpenGL like on the
    // other platforms (same black-viewport workaround, see run.bat).
    set_env_if_unset("QSG_RHI_BACKEND", "opengl");
#else
    // HAL plugin path — where the SDK looks for camera driver .so files.
    // Default: Prophesee openEB install location. Override by exporting
    // MV_HAL_PLUGIN_PATH before launching the app.
    set_env_if_unset("MV_HAL_PLUGIN_PATH", "/usr/local/lib/metavision/hal/plugins");
    // HDF5 plugin path — needed for reading HDF5 event files.
    set_env_if_unset("HDF5_PLUGIN_PATH", "/usr/local/lib/hdf5/plugin");
#endif
    // Platform plugin on Wayland sessions — a deliberate, documented
    // trade-off (both sides are field-observed on this host):
    //  - PRO xcb: older Qt/driver stacks rendered a black QOpenGLWidget
    //    viewport under the native Wayland plugin; xcb (via XWayland) was
    //    the reliable path.
    //  - CON xcb: while ANOTHER window is fullscreen, XWayland presentation
    //    is throttled and every animated view stutters until that window
    //    closes; native Wayland does not have this problem (verified).
    // Conservative default: keep xcb (a black viewport is a fatal failure,
    // the stutter is not), and tell the user about the wayland escape hatch
    // on the terminal. Any user override is respected (no print then).
#ifndef _WIN32
    if (!std::getenv("QT_QPA_PLATFORM") && std::getenv("WAYLAND_DISPLAY")) {
        ::setenv("QT_QPA_PLATFORM", "xcb", 0);
        std::fprintf(stderr,
                     "[EBplus] Wayland session: defaulting QT_QPA_PLATFORM=xcb (XWayland).\n"
                     "[EBplus] If animated views stutter while another window is fullscreen,\n"
                     "[EBplus] relaunch with QT_QPA_PLATFORM=wayland.\n");
    }
    // Qt 6 may default to the Vulkan RHI backend and produce a black viewport
    // on this GPU; force OpenGL unless the user has set it.
    if (!std::getenv("QSG_RHI_BACKEND")) {
        ::setenv("QSG_RHI_BACKEND", "opengl", 0);
    }
#endif
    // Note: LD_LIBRARY_PATH cannot be set here (the dynamic linker has
    // already resolved shared libraries at process start).  If the SDK
    // libraries are in a non-standard path, set it in the shell or use
    // the run.sh launcher.
}

int main(int argc, char* argv[]) {
    ensure_openeb_env_defaults(argc > 0 ? argv[0] : "");

    // Request a core-profile OpenGL 3.3 context for the display widget.
    QSurfaceFormat fmt;
    fmt.setVersion(3, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    fmt.setSwapInterval(1); // vsync on
    QSurfaceFormat::setDefaultFormat(fmt);

    QApplication app(argc, argv);
    QApplication::setApplicationName("GUI for openEB");
    QApplication::setOrganizationName("GUI-for-openEB");
    QApplication::setApplicationVersion("3.0.1");

    // Global UI font — Inter (design §3.9.1) with platform fallbacks so the
    // typeface stays consistent on systems without Inter installed.
    QFont font(QStringLiteral("Inter"), 10);
    font.setFamilies({QStringLiteral("Inter"),
                      QStringLiteral("Segoe UI"),
                      QStringLiteral("Ubuntu"),
                      QStringLiteral("Noto Sans"),
                      QStringLiteral("Sans Serif")});
    font.setStyleStrategy(QFont::PreferAntialias);
    app.setFont(font);

    gui::MainWindow window;
    window.show();

    return app.exec();
}
