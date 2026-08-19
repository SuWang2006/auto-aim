set_project("auto-aim")
set_version("0.1.0")
set_languages("cxx23")

add_rules("mode.debug", "mode.release", "mode.asan")
add_requires("opencv")

target("autoaim")
    set_kind("binary")
    add_files("src/**.cpp")
    add_includedirs("include")
    add_packages("opencv")
    add_packages("ffmpeg")
